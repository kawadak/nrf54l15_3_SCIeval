#ifndef MYGPIO_H_
#define MYGPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* main と myble から参照する GPIO のピンID */
typedef enum {
    LED0 = 0,
    LED1,
    LED2,
    LED3,
    BTN0,
    BTN1,
    BTN2,
    BTN3,
    PIN_COUNT
} pin_id_t;

/* main から呼ぶ初期化系 */
int gpio_init_all(void);
int gpio_enable_button_irq(void);

/* main と myble から使う LED 出力ヘルパ */
int pin_set_logic(pin_id_t id, bool on);


#ifdef __cplusplus
}
#endif

#endif /* MYGPIO_H */