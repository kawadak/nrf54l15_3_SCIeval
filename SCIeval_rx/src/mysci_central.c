#include "mysci_central.h"
#include "myble_central.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

LOG_MODULE_REGISTER(mysci_central, LOG_LEVEL_INF);

/* セマフォ（PHY/IFS の更新完了待ち） */
static K_SEM_DEFINE(phy_updated_sem, 0, 1);
static K_SEM_DEFINE(frame_space_updated_sem, 0, 1);

/* ローカルでサポートされる最小接続間隔(µs)を保持 */
static uint16_t local_min_interval_us = 1000;

/* 現在/希望モード */
static mysci_mode_t desired_mode = MYSCI_MODE_LOW_LATENCY;

/* 接続後にSCIシーケンスを実施するワーク */
static struct k_work_delayable sci_start_work;

/* 内部ヘルパ */
static inline uint16_t us_to_125us(uint32_t us) { return (uint16_t)(us / 125); }

/* IFS更新: 2M PHY, ACL IFS を最小化 */
static int sci_select_lowest_frame_space(struct bt_conn *conn)
{
    const struct bt_conn_le_frame_space_update_param params = {
        .phys = BT_HCI_LE_FRAME_SPACE_UPDATE_PHY_2M_MASK,
        .spacing_types = BT_CONN_LE_FRAME_SPACE_TYPES_MASK_ACL_IFS,
        .frame_space_min = 0,
        .frame_space_max = 150, /* us */
    };

    int err = bt_conn_le_frame_space_update(conn, &params);
    if (err) {
        LOG_WRN("frame_space_update failed (err %d)", err);
        return err;
    }

    if (k_sem_take(&frame_space_updated_sem, K_MSEC(1500)) != 0) {
        LOG_WRN("frame_space_update timeout");
        return -ETIMEDOUT;
    }
    return 0;
}

/* 2M PHYへ更新 */
static int sci_update_to_2m_phy(struct bt_conn *conn)
{
    const struct bt_conn_le_phy_param phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    int err = bt_conn_le_phy_update(conn, &phy);
    if (err) {
        LOG_WRN("phy_update failed (err %d)", err);
        return err;
    }
    if (k_sem_take(&phy_updated_sem, K_MSEC(1500)) != 0) {
        LOG_WRN("phy_update timeout");
        return -ETIMEDOUT;
    }
    return 0;
}

/* SCIリクエストを投げる（モード別の min/max を組み立て） */
static int sci_request_mode(struct bt_conn *conn, mysci_mode_t mode)
{
    uint32_t min_us;
    uint32_t max_us;

    switch (mode) {
    case MYSCI_MODE_LOW_LATENCY:
        /* 最短はローカル最小値と 1ms の大きい方、上限は 4ms */
        min_us = MAX((uint32_t)local_min_interval_us, (uint32_t)1000);
        max_us = 4000;
        break;
    case MYSCI_MODE_BALANCED:
    default:
        /* 下限は 4ms 以上（ローカル制約を満たす）、上限は 10ms */
        min_us = MAX((uint32_t)local_min_interval_us, (uint32_t)4000);
        max_us = 10000;
        break;
    }

    if (min_us > max_us) {
        /* 端末制約が厳しすぎる場合のフォールバック */
        min_us = max_us;
    }

    const struct bt_conn_le_conn_rate_param params = {
        .interval_min_125us = us_to_125us(min_us),
        .interval_max_125us = us_to_125us(max_us),
        .subrate_min = 1,
        .subrate_max = 1,
        .max_latency = 0, /* peripheral latency = 0 (スキップなし) */
        .continuation_number = 0,
        .supervision_timeout_10ms = 400, /* 4s */
        .min_ce_len_125us = BT_HCI_LE_SCI_CE_LEN_MIN_125US,
        .max_ce_len_125us = BT_HCI_LE_SCI_CE_LEN_MAX_125US,
    };

    int err = bt_conn_le_conn_rate_request(conn, &params);
    if (err) {
        LOG_WRN("conn_rate_request failed (err %d), range=%u..%u us",
                err, (unsigned)min_us, (unsigned)max_us);
        return err;
    }
    LOG_INF("SCI requested: %u..%u us (mode=%d)", (unsigned)min_us, (unsigned)max_us, mode);
    return 0;
}

/* 接続後の一連のSCI手順を実行（ワークハンドラ） */
static void sci_start_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    if (my_conn == NULL) {
        return;
    }

    /* ローカル最小接続間隔（µs）を取得 */
    int err = bt_conn_le_read_min_conn_interval(&local_min_interval_us);
    if (err) {
        LOG_WRN("read_min_conn_interval failed (err %d). Fallback to 1000us", err);
        local_min_interval_us = 1000;
    }
    LOG_INF("Local min conn interval: %u us", local_min_interval_us);

    /* 2M PHY へ */
    err = sci_update_to_2m_phy(my_conn);
    if (err) {
        LOG_WRN("2M PHY setup failed (err %d)", err);
        /* 失敗しても続行（環境によっては1Mのまま） */
    }

    /* ACL IFS を可能な限り縮小 */
    err = sci_select_lowest_frame_space(my_conn);
    if (err) {
        LOG_WRN("Frame space update failed (err %d)", err);
    }

    /* 直近の希望モードで SCI を要求 */
    sci_request_mode(my_conn, desired_mode);
}

/* ===== コールバック ===== */
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(param);
    /* 旧来のパラメータ更新（Peer起点）は不採用にしてSCI更新で統一 */
    return false;
}

static void le_phy_updated_cb(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    ARG_UNUSED(conn);
    LOG_INF("PHY updated: TX %u, RX %u", param->tx_phy, param->rx_phy);
    k_sem_give(&phy_updated_sem);
}

static void frame_space_updated_cb(struct bt_conn *conn,
                                   const struct bt_conn_le_frame_space_updated *params)
{
    if (params->status == BT_HCI_ERR_SUCCESS) {
        LOG_INF("Frame space updated: %u us, phys=0x%02x, types=0x%04x",
                params->frame_space, params->phys, params->spacing_types);
    } else {
        LOG_WRN("Frame space update failed: 0x%02x", params->status);
    }
    k_sem_give(&frame_space_updated_sem);
}

static void conn_rate_changed_cb(struct bt_conn *conn, uint8_t status,
                                 const struct bt_conn_le_conn_rate_changed *params)
{
    if (status == BT_HCI_ERR_SUCCESS) {
        LOG_INF("SCI applied: interval %u us, subrate %d, periph_latency %d, cont %d, sup_to %d ms",
                params->interval_us, params->subrate_factor, params->peripheral_latency,
                params->continuation_number, params->supervision_timeout_10ms * 10);
    } else {
        LOG_WRN("SCI change failed: 0x%02x", status);
    }
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        return;
    }
    /* 接続直後のSCIシーケンスを少し遅延して開始（他セットアップ後に上書きする） */
    k_work_schedule(&sci_start_work, K_MSEC(200));
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(reason);
    /* 次回接続に備えてワークをキャンセル */
    k_work_cancel_delayable(&sci_start_work);
}

/* このモジュールのコールバック（myble_central のものと併用登録OK） */
static struct bt_conn_cb sci_conn_cbs = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
    .le_param_req = le_param_req,
    .le_phy_updated = le_phy_updated_cb,
    .frame_space_updated = frame_space_updated_cb,
    .conn_rate_changed = conn_rate_changed_cb,
};

/* ===== 公開関数 ===== */
void mysci_central_init(void)
{
    /* デフォルト許容レンジ（Central側の受入範囲）を広めに設定 */
    uint16_t local_min_us = 1000;
    if (bt_conn_le_read_min_conn_interval(&local_min_us) != 0) {
        local_min_us = 1000;
    }
    const struct bt_conn_le_conn_rate_param defaults = {
        .interval_min_125us = us_to_125us(local_min_us),
        .interval_max_125us = us_to_125us(10000), /* 10ms */
        .subrate_min = 1,
        .subrate_max = 1,
        .max_latency = 5, /* デフォルトは多少許容 */
        .continuation_number = 0,
        .supervision_timeout_10ms = 400,
        .min_ce_len_125us = BT_HCI_LE_SCI_CE_LEN_MIN_125US,
        .max_ce_len_125us = BT_HCI_LE_SCI_CE_LEN_MAX_125US,
    };
    int err = bt_conn_le_conn_rate_set_defaults(&defaults);
    if (err) {
        LOG_WRN("SCI defaults set failed (err %d)", err);
    } else {
        LOG_INF("SCI defaults set: %u..%u us", local_min_us, 10000);
    }

    /* ローカル最小値を保持 */
    local_min_interval_us = local_min_us;

    /* コールバック登録、ワーク初期化 */
    bt_conn_cb_register(&sci_conn_cbs);
    k_work_init_delayable(&sci_start_work, sci_start_work_handler);
}

void mysci_central_set_mode(mysci_mode_t mode)
{
    desired_mode = mode;

    /* 接続済みなら即時SCIを要求（PHY/IFS前提が未完了でも試みる） */
    if (my_conn) {
        (void)sci_request_mode(my_conn, desired_mode);
    }
}
