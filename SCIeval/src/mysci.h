#ifndef MYSCI_H_
#define MYSCI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* SCI動作モード */
typedef enum {
    MYSCI_MODE_BALANCED = 0,   /* 4～10ms程度（省電力寄り） */
    MYSCI_MODE_LOW_LATENCY,    /* 1～4ms程度（低遅延） */
} mysci_mode_t;

/* 初期化：SCIデフォルト設定、コールバック登録 */
void mysci_init(void);

/* 動作モード切替（接続中なら即時要求、未接続なら次回接続後に適用） */
void mysci_set_mode(mysci_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* MYSCI_H_ */
