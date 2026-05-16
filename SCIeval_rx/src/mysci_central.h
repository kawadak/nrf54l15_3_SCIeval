#ifndef MYSCI_CENTRAL_H_
#define MYSCI_CENTRAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* SCI動作モード（TX側と同一の定義） */
typedef enum {
    MYSCI_MODE_BALANCED = 0,   /* 4～10ms程度（省電力寄り） */
    MYSCI_MODE_LOW_LATENCY,    /* 1～4ms程度（低遅延） */
} mysci_mode_t;

/* 初期化：SCIデフォルト設定、コールバック登録（Central側） */
void mysci_central_init(void);

/* 動作モード切替（接続中なら即時要求、未接続なら次回接続後に適用） */
void mysci_central_set_mode(mysci_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* MYSCI_CENTRAL_H_ */
