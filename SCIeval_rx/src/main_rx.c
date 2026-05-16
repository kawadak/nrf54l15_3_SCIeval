#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/sys/atomic.h>
#include <string.h>
#include <stdio.h>

#include "mygpio.h"
#include "myble_central.h"
#include "mysci_central.h"

LOG_MODULE_REGISTER(main_rx, LOG_LEVEL_INF);

/* LED0 点滅管理 */
static struct k_work_delayable led0_blink_work;
static uint32_t led0_blink_interval_ms = 1000; /* 初期1秒 */
static bool led0_state = false;

/* プロトタイプ（mygpio_rx / myble_central から呼ばれる） */
void on_btn0_event(bool pressed);
void on_scan_target_found(const bt_addr_le_t *addr, int8_t rssi);
void on_ble_connected(void);
void on_ble_connect_failed(uint8_t err);
void on_ble_disconnected(uint8_t reason);
void on_casio_value(uint8_t v);

/* LED0 点滅ワーク */
static void led0_blink_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    /* 接続中はLED0消灯＆点滅停止 */
    if (my_conn != NULL) {
        pin_set_logic(LED0, false);
        led0_state = false;
        return;
    }

    /* トグル */
    led0_state = !led0_state;
    pin_set_logic(LED0, led0_state);

    /* 次回 */
    k_work_schedule(&led0_blink_work, K_MSEC(led0_blink_interval_ms));
}

/* スキャン中に対象が見つかった */
void on_scan_target_found(const bt_addr_le_t *addr, int8_t rssi) {
    ARG_UNUSED(addr);
    LOG_INF("Target peripheral found (RSSI=%d). LED0 blink -> 500ms", rssi);
    led0_blink_interval_ms = 500;
}

/* 接続成功 */
void on_ble_connected(void) {
    LOG_INF("Connected: LED0 OFF, LED1 ON");
    pin_set_logic(LED0, false);
    pin_set_logic(LED1, true);

    /* LED0点滅一旦停止（接続中は点滅不要） */
    k_work_cancel_delayable(&led0_blink_work);

    /* 接続直後は低遅延優先に設定（SCI） */
    mysci_central_set_mode(MYSCI_MODE_LOW_LATENCY);
}

/* 接続失敗（タイムアウト/拒否など） */
void on_ble_connect_failed(uint8_t err) {
    LOG_WRN("Connect failed (err=%u): LED2 5s then resume scanning", err);
    pin_set_logic(LED2, true);
    k_sleep(K_SECONDS(5));
    pin_set_logic(LED2, false);

    /* 再スキャン＆LED0点滅1秒へ戻す */
    start_scan();
    led0_blink_interval_ms = 1000;
    k_work_schedule(&led0_blink_work, K_MSEC(led0_blink_interval_ms));
}

/* 切断 */
void on_ble_disconnected(uint8_t reason) {
    LOG_WRN("Disconnected (reason=%u). Resume scanning", reason);

    pin_set_logic(LED1, false);

    start_scan();
    led0_blink_interval_ms = 1000;
    k_work_schedule(&led0_blink_work, K_MSEC(led0_blink_interval_ms));
}

/* CASIO Notify受信（1バイト） */
void on_casio_value(uint8_t v) {
    LOG_INF("CASIO notify value: %u", v);
    /* 例: 瞬間的にLED3を点灯 */
    pin_set_logic(LED3, true);
    k_sleep(K_MSEC(1));
    pin_set_logic(LED3, false);
}

/* BTN0押下で接続試行 */
void on_btn0_event(bool pressed) {
    if (!pressed) {
        return;
    }
    if (!ble_ready) {
        LOG_WRN("BLE not ready");
        return;
    }
    if (my_conn != NULL) {
        LOG_INF("Already connected");
        return;
    }
    if (!target_found()) {
        LOG_INF("No target found yet (still scanning)");
        return;
    }
    LOG_INF("BTN0 pressed: try to connect to target");
    int err = connect_to_current_target();
    if (err) {
        LOG_ERR("connect_to_current_target failed (err %d)", err);
        on_ble_connect_failed((uint8_t)(-err));
    }
}

int main(void) {
    LOG_INF("Central (Receiver) start");

    /* GPIO初期化 */
    int err = gpio_init_all();
    if (err) {
        LOG_ERR("GPIO init failed (%d)", err);
        return -1;
    }

    /* 初期LED状態 */
    pin_set_logic(LED0, false);
    pin_set_logic(LED1, false);
    pin_set_logic(LED2, false);
    pin_set_logic(LED3, false);

    /* BLE開始 */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (%d)", err);
        return -1;
    }
    ble_ready = true;
    LOG_INF("Bluetooth initialized (Central)");

    /* SCI（Central側）初期化：デフォルト設定とコールバック登録 */
    mysci_central_init();

    /* BLE中央ロジック初期化（スキャン・接続・ディスカバリ・購読） */
    ble_setup();

    /* スキャン開始 */
    err = start_scan();
    if (err) {
        LOG_ERR("Scan start failed (%d)", err);
        return -1;
    }

    /* BTN0 IRQを有効化（mygpio_rx.cがon_btn0_eventを呼び出す） */
    err = gpio_enable_button_irq();
    if (err) {
        LOG_ERR("Enable button IRQ failed (%d)", err);
        return -1;
    }

    /* LED0点滅開始（初期1秒） */
    k_work_init_delayable(&led0_blink_work, led0_blink_work_handler);
    k_work_schedule(&led0_blink_work, K_MSEC(led0_blink_interval_ms));

    for (;;) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}

/*
送信側と接続する仕様は、
・SCAN中対象となる送信機が見つからない間はLED0を1秒周期で点滅させる
・対象が見つかったらLED0を0.5秒周期の点滅に早める
・BTN0をおしたら接続を試行、成功したらLED0を消灯しLED1を点灯する。
　接続が失敗した場合はLED2を5秒点灯してからSCANに戻る
*/
