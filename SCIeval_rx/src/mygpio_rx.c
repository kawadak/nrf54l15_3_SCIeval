#include "mygpio.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(mygpio_rx, LOG_LEVEL_INF);

/* アプリ側コールバック（main_rx.cで定義） */
extern void on_btn0_event(bool pressed);

/* ------------------------- GPIO 設定 ------------------------- */
#define DEV_GPIO0 DEVICE_DT_GET(DT_NODELABEL(gpio0)) /* P0.x */
#define DEV_GPIO1 DEVICE_DT_GET(DT_NODELABEL(gpio1)) /* P1.x */
#define DEV_GPIO2 DEVICE_DT_GET(DT_NODELABEL(gpio2)) /* P2.x */

typedef struct {
    const struct device *port;
    uint8_t pin;
    bool is_output;
    bool active_low;
    bool pull_up;
    bool use_irq;
} pin_cfg_t;

/* nrf54l15-dk ピン割り当て（送信側と同一） */
static const pin_cfg_t pins[PIN_COUNT] = {
    [LED0] = { .port = DEV_GPIO2, .pin = 9,  .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false },
    [LED1] = { .port = DEV_GPIO1, .pin = 10, .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false },
    [LED2] = { .port = DEV_GPIO2, .pin = 7,  .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false },
    [LED3] = { .port = DEV_GPIO1, .pin = 14, .is_output = true,  .active_low = false, .pull_up = false, .use_irq = false },
    [BTN0] = { .port = DEV_GPIO1, .pin = 13, .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = true  },
    [BTN1] = { .port = DEV_GPIO1, .pin = 9,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = false },
    [BTN2] = { .port = DEV_GPIO1, .pin = 8,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = false },
    [BTN3] = { .port = DEV_GPIO0, .pin = 4,  .is_output = false, .active_low = true,  .pull_up = true,  .use_irq = false },
};

/* 出力ヘルパ */
int pin_set_logic(pin_id_t id, bool on){
    const pin_cfg_t *c = &pins[id];
    if (!c->is_output) {
        return -EINVAL;
    }
    int level = c->active_low ? !on : on;
    return gpio_pin_set(c->port, c->pin, level);
}

/* 内部: 入力の論理値（active_low吸収） */
static bool pin_get_logic(pin_id_t id){
    const pin_cfg_t *c = &pins[id];
    int v = gpio_pin_get(c->port, c->pin);
    if (v < 0) { return false; }
    return c->active_low ? (v == 0) : (v == 1);
}

/* BTN0用コールバック */
static struct gpio_callback btn_cb;

static void button_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins_mask){
    ARG_UNUSED(cb);
    if ((port == pins[BTN0].port) && (pins_mask & BIT(pins[BTN0].pin))) {
        bool pressed = pin_get_logic(BTN0); /* active_lowのため、論理ON=押下 */
        on_btn0_event(pressed);
    }
}

/* IRQ有効化（BTN0のみ） */
int gpio_enable_button_irq(void){
    int err;
    err = gpio_pin_interrupt_configure(pins[BTN0].port, pins[BTN0].pin, GPIO_INT_EDGE_BOTH);
    if (err) {
        LOG_ERR("Interrupt config for BTN0 failed (%d)", err);
        return err;
    }
    gpio_init_callback(&btn_cb, button_isr, BIT(pins[BTN0].pin));
    gpio_add_callback(pins[BTN0].port, &btn_cb);
    return 0;
}

/* 初期化 */
int gpio_init_all(void){
    for (int i = 0; i < PIN_COUNT; i++) {
        if (!device_is_ready(pins[i].port)) {
            LOG_ERR("GPIO port not ready for pin %d", i);
            return -ENODEV;
        }
    }
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
            /* IRQはここでは有効化しない */
        }
    }
    return 0;
}
