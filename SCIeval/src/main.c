#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/lbs.h>
#include <stdio.h>
#include <string.h>

#include "mygpio.h"
#include "myble.h"
#include "mysci.h"

#define DK //DK or XIAO

LOG_MODULE_REGISTER(blinky, LOG_LEVEL_INF);

#define RUN_LED_BLINK_INTERVAL 1000

/* ======== BTN2押下状態（mygpio.c から更新される） ======== */
atomic_t btn2_pressed_atomic = ATOMIC_INIT(0);

/* ======== カスタムGATT（Notifyで1バイト値を送る） ======== */
/* 任意の128-bit Vendor UUID（アプリ側と合わせればOK） */
static struct bt_uuid_128 casio_svc_uuid = BT_UUID_INIT_128(
    0xF0,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12);
static struct bt_uuid_128 casio_chr_uuid = BT_UUID_INIT_128(
    0xF1,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x34,0x12,0x78,0x56,0x34,0x12,0x56,0x12);

static uint8_t casio_last_value = 0;
static uint8_t casio_notify_enabled = 0;

/* 送信完了のバックプレッシャ制御（1本インフライト） */
static K_SEM_DEFINE(notify_sem, 1, 1);
static struct bt_gatt_notify_params casio_notify_params;

static void casio_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    casio_notify_enabled = (value == BT_GATT_CCC_NOTIFY) ? 1 : 0;
}

static ssize_t casio_read(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset) {
    const uint8_t *val = &casio_last_value;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, val, sizeof(*val));
}

/* 静的サービス定義: [0]=Primary, [1]=Char Decl, [2]=Char Value, [3]=CCC */
BT_GATT_SERVICE_DEFINE(casio_svc,
    BT_GATT_PRIMARY_SERVICE(&casio_svc_uuid),
    BT_GATT_CHARACTERISTIC(&casio_chr_uuid.uuid,
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ, casio_read, NULL, &casio_last_value),
    BT_GATT_CCC(casio_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* ======== 周期送信ワーク（BTN2押下中のみ送信） ======== */
#define BTN2_TX_INTERVAL_MS 1
static struct k_work_delayable btn2_stream_work;

/* notify 完了コールバック：セマフォを返し、押下中は即次弾を起動 */
static void notify_complete_cb(struct bt_conn *conn, void *user_data)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(user_data);
    k_sem_give(&notify_sem);

    if (atomic_get(&btn2_pressed_atomic) && ble_ready && (my_conn != NULL) && casio_notify_enabled) {
        /* 可能であれば次の送信を即実行（遅延を最小化） */
        k_work_reschedule(&btn2_stream_work, K_NO_WAIT);
    }
}

/* 値を通知する（1本だけインフライト許可） */
static int casio_notify_u8(uint8_t v) {
    if (!ble_ready || my_conn == NULL || !casio_notify_enabled) {
        return -EAGAIN;
    }

    /* インフライトがある場合はスキップ（次のタイミングへ） */
    if (k_sem_take(&notify_sem, K_NO_WAIT) != 0) {
        return -EAGAIN;
    }

    casio_last_value = v;

    memset(&casio_notify_params, 0, sizeof(casio_notify_params));
    casio_notify_params.attr = &casio_svc.attrs[2]; /* Characteristic Value */
    casio_notify_params.data = &casio_last_value;
    casio_notify_params.len  = sizeof(casio_last_value);
    casio_notify_params.func = notify_complete_cb;

    int err = bt_gatt_notify_cb(my_conn, &casio_notify_params);
    if (err) {
        /* 送信失敗時はセマフォを返却しておく */
        k_sem_give(&notify_sem);
    }
    return err;
}

static void btn2_stream_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    static uint8_t counter = 0;
    static bool prev_pressed = false;

    bool pressed = (atomic_get(&btn2_pressed_atomic) != 0);

    /* 押下/解放のエッジで SCI のモードを切り替える */
    if (pressed && !prev_pressed) {
        /* 低遅延モード（1～4ms を許容範囲として要求） */
        mysci_set_mode(MYSCI_MODE_LOW_LATENCY);
    } else if (!pressed && prev_pressed) {
        /* バランスモード（4～10ms を許容範囲として要求） */
        mysci_set_mode(MYSCI_MODE_BALANCED);
    }
    prev_pressed = pressed;

    /* 押下中かつ接続＆Notify有効なら送信 */
    if (pressed && ble_ready && (my_conn != NULL) && casio_notify_enabled) {
        int err = casio_notify_u8(counter);
        if (err && err != -EAGAIN) {
            LOG_DBG("notify err=%d", err);
        }
        counter++; /* 自動で 0→255→0 と回る */
    }

    /* 保険として 1ms 後に再チェック（送信完了時は即時再スケジュールが入る） */
    k_work_schedule(&btn2_stream_work, K_MSEC(BTN2_TX_INTERVAL_MS));
}

/* ------------------------- main ------------------------- */
int main(void) {
    int err;
    LOG_INF("Main loop\n");

    /* gpio initialize */
    err = gpio_init_all();
    if (err) {
        LOG_ERR("GPIO init failed (err %d)", err);
        return -1;
    }

    pin_set_logic(LED0, true);

    /* BLE スタック起動 */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return -1;
    }
    ble_ready = true;
    LOG_INF("Bluetooth initialized");

    /* SCI 初期化（デフォルト許容レンジの設定、コールバック登録など） */
    mysci_init();

    /* LBS サービス初期化 */
    const struct bt_lbs_cb lbs_cb = {
        .led_cb = lbs_led_cb,
    };
    err = bt_lbs_init(&lbs_cb);
    if (err) {
        LOG_ERR("bt_lbs_init failed (err %d)", err);
        return -1;
    }
    lbs_ready = true;

    pin_set_logic(LED0, false);

    /* コネクションコールバック登録とアドバタイズ開始 */
    err = bt_conn_cb_register(&connection_callbacks);
    if (err) {
        LOG_ERR("Connection callback register failed (err %d)", err);
    }

    k_work_init(&adv_work, adv_work_handler);
    advertising_start();

    #ifdef DK
    /* 周期送信ワーク開始（1msごと、押下中のみ送信＋SCI制御） */
    k_work_init_delayable(&btn2_stream_work, btn2_stream_work_handler);
    k_work_schedule(&btn2_stream_work, K_MSEC(BTN2_TX_INTERVAL_MS));
    #endif

    /* BLE/LBS 準備完了後に、ボタン割り込みを有効化 */
    err = gpio_enable_button_irq();
    if (err) {
        LOG_ERR("Enable button IRQ failed (err %d)", err);
        return -1;
    }

    pin_set_logic(LED0, true);

    /* main loop */
    for (;;) {
        k_sleep(K_SECONDS(1));
    }
}
