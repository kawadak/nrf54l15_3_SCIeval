#include "myble_central.h"
#include "mygpio.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>

LOG_MODULE_REGISTER(myble_central, LOG_LEVEL_INF);

/* BLEグローバル */
struct bt_conn *my_conn = NULL;
bool ble_ready = false;

/* 送信側デバイス名（Peripheralのprj.confまたはCONFIG_BT_DEVICE_NAMEと一致させる） */
static const char target_name[] = "0514_Test_peripheral";

/* スキャンパラメータ（Active scan） */
static const struct bt_le_scan_param scan_param = {
    .type     = BT_LE_SCAN_TYPE_ACTIVE,
    .options  = BT_LE_SCAN_OPT_NONE,
    .interval = BT_GAP_SCAN_FAST_INTERVAL,
    .window   = BT_GAP_SCAN_FAST_WINDOW,
};

/* 直近に見つけた対象候補 */
static bool have_candidate = false;
static bt_addr_le_t candidate_addr;
static int8_t candidate_rssi = 0;

/* CASIOカスタムUUID（送信側と一致） */
static struct bt_uuid_128 casio_svc_uuid = BT_UUID_INIT_128(
    0xF0,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12);
static struct bt_uuid_128 casio_chr_uuid = BT_UUID_INIT_128(
    0xF1,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12);

/* ディスカバリ／購読 */
static struct bt_gatt_discover_params disc_params;
static struct bt_gatt_subscribe_params sub_params;

static uint16_t svc_start = 0;
static uint16_t svc_end   = 0;
static uint16_t chr_val_handle = 0;
static uint16_t ccc_handle     = 0;

/* 最適化（MTU/DLE/PHY）。SCIは mysci_central が担当 */
static void update_phy(struct bt_conn *conn){
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    int err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err) {
        LOG_WRN("bt_conn_le_phy_update err=%d", err);
    }
}

static struct bt_gatt_exchange_params exchange_params;
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params){
    ARG_UNUSED(conn); ARG_UNUSED(params);
    LOG_INF("MTU exchange %s", att_err == 0 ? "OK" : "FAIL");
}
static void update_mtu(struct bt_conn *conn){
    exchange_params.func = exchange_func;
    int err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_WRN("bt_gatt_exchange_mtu err=%d", err);
    }
}
static void update_data_len(struct bt_conn *conn){
    struct bt_conn_le_data_len_param p = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    int err = bt_conn_le_data_len_update(conn, &p);
    if (err) {
        LOG_WRN("data_len_update err=%d", err);
    }
}

/* Notify受信 */
static uint8_t casio_notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
                               const void *data, uint16_t length){
    ARG_UNUSED(conn); ARG_UNUSED(params);
    if (!data) {
        LOG_INF("CASIO notify disabled by peer");
        return BT_GATT_ITER_CONTINUE;
    }
    if (length >= 1) {
        uint8_t v = ((const uint8_t*)data)[0];
        on_casio_value(v);
    }
    return BT_GATT_ITER_CONTINUE;
}

/* ディスカバリ進行 */
static int discover_next_characteristic(struct bt_conn *conn);
static int discover_next_ccc(struct bt_conn *conn);

static uint8_t discover_func(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params){
    if (!attr) {
        LOG_WRN("Discovery complete but target not found (type=%u)", params->type);
        memset(params, 0, sizeof(*params));
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_PRIMARY) {
        const struct bt_gatt_service_val *svc = attr->user_data;
        svc_start = attr->handle + 1;
        svc_end   = svc->end_handle;
        LOG_INF("Service found: start=0x%04x end=0x%04x", svc_start, svc_end);
        /* 次はキャラクタリスティック探索 */
        discover_next_characteristic(conn);
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
        const struct bt_gatt_chrc *chrc = attr->user_data;
        chr_val_handle = chrc->value_handle;
        LOG_INF("Characteristic found: value_handle=0x%04x", chr_val_handle);
        /* 次はCCC探索 */
        discover_next_ccc(conn);
        return BT_GATT_ITER_STOP;
    }

    if (params->type == BT_GATT_DISCOVER_DESCRIPTOR) {
        ccc_handle = attr->handle;
        LOG_INF("CCC found: handle=0x%04x", ccc_handle);
        memset(params, 0, sizeof(*params));

        /* 購読設定 */
        memset(&sub_params, 0, sizeof(sub_params));
        sub_params.ccc_handle   = ccc_handle;
        sub_params.value        = BT_GATT_CCC_NOTIFY;
        sub_params.value_handle = chr_val_handle;
        sub_params.notify       = casio_notify_cb;

        int err = bt_gatt_subscribe(conn, &sub_params);
        if (err && err != -EALREADY) {
            LOG_ERR("bt_gatt_subscribe err=%d", err);
        } else {
            LOG_INF("CASIO characteristic subscribed");
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_STOP;
}

static int discover_service(struct bt_conn *conn){
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.uuid         = &casio_svc_uuid.uuid;
    disc_params.func         = discover_func;
    disc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    disc_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    disc_params.type         = BT_GATT_DISCOVER_PRIMARY;
    return bt_gatt_discover(conn, &disc_params);
}

static int discover_next_characteristic(struct bt_conn *conn){
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.uuid         = &casio_chr_uuid.uuid;
    disc_params.func         = discover_func;
    disc_params.start_handle = svc_start;
    disc_params.end_handle   = svc_end;
    disc_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;
    return bt_gatt_discover(conn, &disc_params);
}

static int discover_next_ccc(struct bt_conn *conn){
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.uuid         = BT_UUID_GATT_CCC;
    disc_params.func         = discover_func;
    disc_params.start_handle = chr_val_handle + 1;
    disc_params.end_handle   = svc_end;
    disc_params.type         = BT_GATT_DISCOVER_DESCRIPTOR;
    return bt_gatt_discover(conn, &disc_params);
}

/* 接続コールバック */
static void connected_cb(struct bt_conn *conn, uint8_t err){
    if (err) {
        LOG_WRN("Connection failed (err=%u)", err);
        on_ble_connect_failed(err);
        return;
    }
    my_conn = bt_conn_ref(conn);
    LOG_INF("Connected");

    on_ble_connected();

    /* 最適化（SCIは mysci_central が別途実施） */
    update_phy(conn);
    update_data_len(conn);
    update_mtu(conn);

    /* カスタムサービス探索開始 */
    int derr = discover_service(conn);
    if (derr) {
        LOG_WRN("Discovery start err=%d", derr);
    }
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason){
    ARG_UNUSED(conn);
    LOG_INF("Disconnected (reason=%u)", reason);
    if (my_conn) {
        bt_conn_unref(my_conn);
        my_conn = NULL;
    }
    on_ble_disconnected(reason);
}

static void le_phy_updated_cb(struct bt_conn *conn, struct bt_conn_le_phy_info *param){
    ARG_UNUSED(conn);
    LOG_INF("PHY updated: tx=%u rx=%u", param->tx_phy, param->rx_phy);
}

static void le_data_len_updated_cb(struct bt_conn *conn, struct bt_conn_le_data_len_info *info){
    ARG_UNUSED(conn);
    LOG_INF("Data len updated: tx_len=%u rx_len=%u", info->tx_max_len, info->rx_max_len);
}

static struct bt_conn_cb conn_cbs = {
    .connected           = connected_cb,
    .disconnected        = disconnected_cb,
    .le_phy_updated      = le_phy_updated_cb,
    .le_data_len_updated = le_data_len_updated_cb,
};

/* 広告データからデバイス名を抽出して一致判定 */
static bool parse_name_cb(struct bt_data *data, void *user_data){
    bool *match = (bool *)user_data;
    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
        char name[32];
        size_t len = MIN((size_t)data->data_len, sizeof(name) - 1);
        memcpy(name, data->data, len);
        name[len] = '\0';
        if (strcmp(name, target_name) == 0) {
            *match = true;
            return false; /* 打ち切り */
        }
    }
    return true; /* 継続 */
}

/* スキャンコールバック（legacy形式） */
static void device_found_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                            struct net_buf_simple *ad){
    bool name_match = false;
    bt_data_parse(ad, parse_name_cb, &name_match);
    if (!name_match) {
        return;
    }

    have_candidate = true;
    candidate_addr = *addr;
    candidate_rssi = rssi;

    on_scan_target_found(addr, rssi);
}

/* 公開関数群 */
void ble_setup(void){
    bt_conn_cb_register(&conn_cbs);
}

int start_scan(void){
    have_candidate = false;
    int err = bt_le_scan_start(&scan_param, device_found_cb);
    if (err && err != -EALREADY) {
        LOG_ERR("bt_le_scan_start err=%d", err);
        return err;
    }
    LOG_INF("Scanning started");
    return 0;
}

int stop_scan(void){
    int err = bt_le_scan_stop();
    if (err && err != -EALREADY) {
        LOG_WRN("bt_le_scan_stop err=%d", err);
        return err;
    }
    LOG_INF("Scanning stopped");
    return 0;
}

bool target_found(void){
    return have_candidate;
}

int connect_to_current_target(void){
    if (!have_candidate) {
        return -ENODEV;
    }

    int err = stop_scan();
    if (err && err != -EALREADY) {
        LOG_WRN("stop_scan err=%d (continue)", err);
    }

    struct bt_conn_le_create_param create_param = {
        .options  = BT_CONN_LE_CREATE_CONN,
        .interval = BT_GAP_SCAN_FAST_INTERVAL,
        .window   = BT_GAP_SCAN_FAST_WINDOW,
    };

    err = bt_conn_le_create(&candidate_addr, &create_param,
                            BT_LE_CONN_PARAM_DEFAULT, &my_conn);
    if (err) {
        LOG_ERR("bt_conn_le_create err=%d", err);
        have_candidate = false;
        return err;
    }

    have_candidate = false;
    LOG_INF("Connecting...");
    return 0;
}
