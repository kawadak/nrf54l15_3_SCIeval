#include "myble.h"
#include "mygpio.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/lbs.h>

LOG_MODULE_REGISTER(myble, LOG_LEVEL_INF);

/* ------------------------- BLE/Advertising ------------------------- */
struct bt_conn *my_conn = NULL;
struct k_work adv_work;
bool ble_ready = false;
bool lbs_ready = false;

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_FAST_INT_MIN_1,
    BT_GAP_ADV_FAST_INT_MAX_1,
    NULL);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, (sizeof(CONFIG_BT_DEVICE_NAME) - 1)),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)),
};

void adv_work_handler(struct k_work *work){
    ARG_UNUSED(work);
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising successfully started");
}

void advertising_start(void){
    k_work_submit(&adv_work);
}

/* ------------------------- BLE ユーティリティ ------------------------- */
static struct bt_gatt_exchange_params exchange_params;

static void update_phy(struct bt_conn *conn){
    int err;
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err) {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

static void update_data_length(struct bt_conn *conn){
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params){
    ARG_UNUSED(params);
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   /* 3 bytes for ATT header */
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}

static void update_mtu(struct bt_conn *conn){
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

static void update_conn_params(struct bt_conn *conn){
    struct bt_le_conn_param param = {
        .interval_min = 6,   /* 6 * 1.25ms = 7.5ms */
        .interval_max = 6,   /* 固定要求 */
        .latency      = 0,
        .timeout      = 400, /* 4s */
    };
    int err = bt_conn_le_param_update(conn, &param);
    if (err) {
        LOG_ERR("bt_conn_le_param_update failed (err %d)", err);
    } else {
        LOG_INF("Requested connection params: 7.5ms, latency 0, timeout 4s");
    }
}

/* ------------------------- LBS 初期化（main から渡されるコールバック） ------------------------- */
void lbs_led_cb(bool led_state){
    /* 必要に応じて LED 連動 */
    ARG_UNUSED(led_state);
}

/* ------------------------- BLE コールバック ------------------------- */
static void on_connected(struct bt_conn *conn, uint8_t err){
    if (err) {
        LOG_ERR("Connection error %d", err);
        return;
    }
    LOG_INF("Connected");
    my_conn = bt_conn_ref(conn);

    /* 接続 LED ON */
    pin_set_logic(LED2, true);
    k_sleep(K_MSEC(100));

    struct bt_conn_info info;
    err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("bt_conn_get_info() returned %d", err);
        return;
    }

    double connection_interval = BT_GAP_US_TO_CONN_INTERVAL(info.le.interval_us) * 1.25; /* ms */
    uint16_t supervision_timeout = info.le.timeout * 10; /* ms */
    LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms",
            connection_interval, info.le.latency, supervision_timeout);

    /* 低遅延化リクエスト */
    update_conn_params(conn);   /* 7.5ms を要求 */
    update_phy(conn);           /* 2M PHY を要求 */
    update_data_length(conn);   /* Data Length Extension */
    update_mtu(conn);           /* ATT MTU 交換 */
}

static void on_disconnected(struct bt_conn *conn, uint8_t reason){
    ARG_UNUSED(conn);
    LOG_INF("Disconnected. Reason %d", reason);
    /* 接続 LED OFF */
    pin_set_logic(LED2, false);
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }
}

static void on_recycled(void){
    advertising_start();
}

static void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout){
    ARG_UNUSED(conn);
    double connection_interval = interval * 1.25;         /* ms */
    uint16_t supervision_timeout = timeout * 10;          /* ms */
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms",
            connection_interval, latency, supervision_timeout);
}

static void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param){
    ARG_UNUSED(conn);
    switch (param->tx_phy) {
    case BT_GAP_LE_PHY_1M:
        LOG_INF("PHY updated. New PHY: 1M");
        break;
    case BT_GAP_LE_PHY_2M:
        LOG_INF("PHY updated. New PHY: 2M");
        break;
    case BT_GAP_LE_PHY_CODED:
        LOG_INF("PHY updated. New PHY: Coded");
        break;
    default:
        LOG_INF("PHY updated. New PHY: 0x%02x", param->tx_phy);
        break;
    }
}

static void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info){
    ARG_UNUSED(conn);
    uint16_t tx_len  = info->tx_max_len;
    uint16_t tx_time = info->tx_max_time;
    uint16_t rx_len  = info->rx_max_len;
    uint16_t rx_time = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

struct bt_conn_cb connection_callbacks = {
    .connected            = on_connected,
    .disconnected         = on_disconnected,
    .recycled             = on_recycled,
    .le_param_updated     = on_le_param_updated,
    .le_phy_updated       = on_le_phy_updated,
    .le_data_len_updated  = on_le_data_len_updated,
};
