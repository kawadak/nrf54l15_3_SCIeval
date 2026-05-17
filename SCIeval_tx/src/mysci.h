#ifndef MYSCI_H
#define MYSCI_H

#ifdef __cplusplus
extern "C" {
#endif


int mysci_init(void);
void mysci_select_mode(void);
void mysci_start(void);
void mysci_run(void);


#ifdef __cplusplus
}
#endif

#endif /* MYSCI_H */