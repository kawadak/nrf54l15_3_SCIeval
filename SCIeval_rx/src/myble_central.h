#ifndef MYBLE_CENTRAL_H_
#define MYBLE_CENTRAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/addr.h>

/* 外部参照可能なBLE状態 */
extern struct bt_conn *my_conn;
extern bool ble_ready;

/* 初期化 */
void ble_setup(void);

/* スキャン操作 */
int start_scan(void);
int stop_scan(void);

/* 対象候補の状態取得 */
bool target_found(void);

/* 現在の対象へ接続 */
int connect_to_current_target(void);

/* アプリ側（main）に通知するコールバック */
void on_scan_target_found(const bt_addr_le_t *addr, int8_t rssi);
void on_ble_connected(void);
void on_ble_connect_failed(uint8_t err);
void on_ble_disconnected(uint8_t reason);

/* CASIO Notify受信（mainで定義） */
void on_casio_value(uint8_t v);

#ifdef __cplusplus
}
#endif

#endif /* MYBLE_CENTRAL_H_ */
