#ifndef MYBLE_H_
#define MYBLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* 前方宣言（ヘッダで重い Zephyr ヘッダを引き込まないため） */
struct k_work;
struct bt_conn_cb;
struct bt_conn;

/* main / mygpio から参照する BLE 状態 */
extern struct bt_conn *my_conn;
extern bool ble_ready;
extern bool lbs_ready;

/* main から直接使うアドバタイズ関連オブジェクト */
extern struct k_work adv_work;
void adv_work_handler(struct k_work *work);
void advertising_start(void);

/* main から登録するコネクションコールバック */
extern struct bt_conn_cb connection_callbacks;

/* LBS の LED 書き込みコールバック（main から渡す） */
void lbs_led_cb(bool led_state);


#ifdef __cplusplus
}
#endif

#endif /* MYBLE_H */