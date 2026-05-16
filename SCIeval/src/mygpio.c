#include "mygpio.h"
#include "myble.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <bluetooth/services/lbs.h>

/* このモジュールのログ名 */
LOG_MODULE_REGISTER(mygpio, LOG_LEVEL_INF);

/* 他ファイル（main.c）で定義しているBTN2押下フラグ */
extern atomic_t btn2_pressed_atomic;

/* ------------------------- GPIO 設定 ------------------------- */
#define DEV_GPIO0 DEVICE_DT_GET(DT_NODELABEL(gpio0)) /* P0.x */
#define DEV_GPIO1 DEVICE_DT_GET(DT_NODELABEL(gpio1)) /* P1.x */
#define DEV_GPIO2 DEVICE_DT_GET(DT_NODELABEL(gpio2)) /* P2.x */

#define DK //DK or XIAO

typedef struct {
    const struct device *port; /* 例: DEV_GPIO1/DEV_GPIO2 */
    uint8_t pin;               /* Px.y の y 部分 */
    bool is_output;            /* 出力なら true, 入力なら false */
    bool active_low;           /* 論理 ON を物理 Low で表すなら true */
    bool pull_up;              /* 入力時: プルアップ使用なら true */
    bool use_irq;              /* 入力時: 割り込み使用なら true */
} pin_cfg_t;

// しばらくは dk で開発
static const pin_cfg_t pins[PIN_COUNT] = {
    // for dk
    #ifdef DK
    [LED0] = { .port = DEV_GPIO2, .pin = 9,  .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false }, /* P2.09 */
    [LED1] = { .port = DEV_GPIO1, .pin = 10, .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false }, /* P1.10 */
    [LED2] = { .port = DEV_GPIO2, .pin = 7,  .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false }, /* P2.07 */
    [LED3] = { .port = DEV_GPIO1, .pin = 14, .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false }, /* P1.14 */
    [BTN0] = { .port = DEV_GPIO1, .pin = 13, .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  }, /* P1.13 (押下=Low) */
    [BTN1] = { .port = DEV_GPIO1, .pin = 9,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  }, /* P1.09 (押下=Low) */
    [BTN2] = { .port = DEV_GPIO1, .pin = 8,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  }, /* P1.08 (押下=Low) */
    [BTN3] = { .port = DEV_GPIO0, .pin = 4,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  }, /* P0.04 (押下=Low) */
    #endif
    //for xiao
    #ifdef XIAO
    [BTN0] = { .port = DEV_GPIO0, .pin = 0, .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  }, /* P0.00 (押下=Low) */
    [LED0] = { .port = DEV_GPIO2, .pin = 0,  .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false }, /* P2.00 */
    #endif
};

/* 論理 ON/OFF をアクティブ極性に従って出力するヘルパ */
int pin_set_logic(pin_id_t id, bool on){
    const pin_cfg_t *c = &pins[id];
    if (!c->is_output) {
        return -EINVAL;
    }
    int level = c->active_low ? !on : on;
    return gpio_pin_set(c->port, c->pin, level);
}

/* 入力/出力の論理値を取得（active_low を吸収） */
static bool pin_get_logic(pin_id_t id){
    const pin_cfg_t *c = &pins[id];
    int v = gpio_pin_get(c->port, c->pin);
    if (v < 0) {
        return false; /* 読み取り失敗時は false 扱い */
    }
    return c->active_low ? (v == 0) : (v == 1);
}

/* ------------------------- ボタンイベント（ワークキュー処理） ------------------------- */
static struct gpio_callback btn_cb;
static struct k_work button_work;   /* BTN0 -> LBS通知 */
static struct k_work btn1_work;     /* BTN1 -> LED3トグル */
static struct k_work btn2_work;     /* BTN2 -> LED1点灯/消灯 + 送信制御 */
static atomic_t btn_state_atomic;

static void button_work_handler(struct k_work *work){
    ARG_UNUSED(work);
    /* BLE/LBS 準備完了かつ接続済みのときだけ通知を送る */
    if (!ble_ready || !lbs_ready || (my_conn == NULL)) {
        return;
    }
    bool pressed = (atomic_get(&btn_state_atomic) != 0);
    int err = bt_lbs_send_button_state(pressed);
    if (err) {
        LOG_ERR("Couldn't send notification. (err: %d)", err);
    }
    LOG_INF("BTN0 %s", pressed ? "pressed" : "released");
}
#ifdef DK
static void btn1_work_handler(struct k_work *work){
    ARG_UNUSED(work);
    /* 押下時のみトグル（active_lowなので論理 ON = 押下） */
    bool pressed = pin_get_logic(BTN1);
    if (!pressed) {
        return;
    }
    bool current = pin_get_logic(LED3);
    pin_set_logic(LED3, !current);
    LOG_INF("BTN1 pressed: LED3 -> %s", (!current) ? "ON" : "OFF");
}


static void btn2_work_handler(struct k_work *work){
    ARG_UNUSED(work);
    /* 押下（active_lowなので論理 ON）でLED1点灯・送信開始、解放で消灯・送信停止 */
    bool pressed = pin_get_logic(BTN2);
    if (!pressed) {
        pin_set_logic(LED1, false);
        atomic_set(&btn2_pressed_atomic, 0);
        LOG_INF("BTN2 released: LED1 -> OFF");
    } else {
        pin_set_logic(LED1, true);
        atomic_set(&btn2_pressed_atomic, 1);
        LOG_INF("BTN2 pressed: LED1 -> ON");
    }
}
#endif

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins_mask){
    ARG_UNUSED(cb);

    /* BTN0イベント: 押下/解放でLBS通知 */
    if ((port == pins[BTN0].port) && (pins_mask & BIT(pins[BTN0].pin))) {
        bool pressed0 = pin_get_logic(BTN0); /* 論理 ON = 押下 */
        atomic_set(&btn_state_atomic, pressed0 ? 1 : 0);
        k_work_submit(&button_work);
    }

    #ifdef DK
    /* BTN1イベント: 押下でLED3をトグル */
    if ((port == pins[BTN1].port) && (pins_mask & BIT(pins[BTN1].pin))) {
        /* トグルはワークキューで実施 */
        k_work_submit(&btn1_work);
    }

    /* BTN2イベント: 押下/解放でLED1制御＋送信制御 */
    if ((port == pins[BTN2].port) && (pins_mask & BIT(pins[BTN2].pin))) {
        k_work_submit(&btn2_work);
    }
    #endif
}

/* 起動時は割り込みを有効化しない。BLE/LBS 初期化後に有効化する */
int gpio_enable_button_irq(void){
    int err;

    /* BTN0: 押下/解放の両エッジで通知 */
    err = gpio_pin_interrupt_configure(pins[BTN0].port, pins[BTN0].pin, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Interrupt config for BTN0 failed (%d)", err);
        return err;
    }

    #ifdef DK
    /* BTN1: 押下（立下り）でLEDトグル */
    err = gpio_pin_interrupt_configure(pins[BTN1].port, pins[BTN1].pin, GPIO_INT_EDGE_FALLING);
    if (err) {
        LOG_ERR("Interrupt config for BTN1 failed (%d)", err);
        return err;
    }

    /* BTN2: 押下/解放の両エッジでLED制御＋送信制御 */
    err = gpio_pin_interrupt_configure(pins[BTN2].port, pins[BTN2].pin, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Interrupt config for BTN2 failed (%d)", err);
        return err;
    }

    /* 同一ポート(P1)の BTN0/BTN1/BTN2 をひとつのコールバックで受ける */
    gpio_init_callback(&btn_cb, button_isr, BIT(pins[BTN0].pin) | BIT(pins[BTN1].pin) | BIT(pins[BTN2].pin));
    gpio_add_callback(pins[BTN0].port, &btn_cb);
    #endif

    #ifdef XIAO
    /* 同一ポート(P1)の BTN0 をひとつのコールバックで受ける */
    gpio_init_callback(&btn_cb, button_isr, BIT(pins[BTN0].pin));
    gpio_add_callback(pins[BTN0].port, &btn_cb);
    #endif

    return 0;
}

int gpio_init_all(void){
    /* すべての使用ポートの ready チェック */
    for (int i = 0; i < PIN_COUNT; i++) {
        if (!device_is_ready(pins[i].port)) {
            LOG_ERR("GPIO port not ready for pin index %d", i);
            return -ENODEV;
        }
    }

    /* ピンを一括設定 */
    for (int i = 0; i < PIN_COUNT; i++) {
        const pin_cfg_t *c = &pins[i];
        int err;
        if (c->is_output) {
            err = gpio_pin_configure(c->port, c->pin, GPIO_OUTPUT_INACTIVE);
            if (err) {
                LOG_ERR("Configure output pin %d failed (%d)", i, err);
                return err;
            }
        } else {
            gpio_flags_t flags = GPIO_INPUT | (c->pull_up ? GPIO_PULL_UP : 0);
            err = gpio_pin_configure(c->port, c->pin, flags);
            if (err) {
                LOG_ERR("Configure input pin %d failed (%d)", i, err);
                return err;
            }
            /* 割り込みはここでは有効化しない（BLE/LBS 初期化後に有効化） */
        }
    }

    /* ボタンワーク初期化 */
    k_work_init(&button_work, button_work_handler);
    #ifdef DK
    k_work_init(&btn1_work, btn1_work_handler);
    k_work_init(&btn2_work, btn2_work_handler);
    #endif

    return 0;
}
