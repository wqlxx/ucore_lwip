#ifndef UCORE_LWIP_SYS_ARCH_H_
#define UCORE_LWIP_SYS_ARCH_H_

#define SYS_ARCH_DECL_PROTECT(lev)
#define SYS_ARCH_PROTECT(lev)
#define SYS_ARCH_UNPROTECT(lev)

#include "cc.h"
#include <types.h>
#include <assert.h>
#include <stdio.h>
#include <lwip/err.h>
#include <sem.h>

#define SYS_MBOX_NULL 0
#define SYS_SEM_NULL 0

// typedef sem_t sys_sem_t;
typedef semaphore_t * sys_sem_t;

struct mbox;
typedef struct mbox * sys_mbox_t;

typedef int sys_thread_t;

sys_sem_t sys_sem_new(u8_t count);
void sys_sem_free(sys_sem_t sem);
void sys_sem_signal(sys_sem_t sem);
u32_t sys_arch_sem_wait(sys_sem_t sem, u32_t timeout);

sys_mbox_t sys_mbox_new(int size);
void sys_mbox_free(sys_mbox_t mbox);
void sys_mbox_post(sys_mbox_t mbox, void *msg);
err_t sys_mbox_trypost(sys_mbox_t mbox, void *msg);
u32_t sys_arch_mbox_fetch(sys_mbox_t mbox, void **msg, u32_t timeout);
u32_t sys_arch_mbox_tryfetch(sys_mbox_t mbox, void **msg);

struct sys_timeouts *sys_arch_timeouts(void);

sys_thread_t sys_thread_new(char *name, void (* thread)(void *arg), void *arg, int stacksize, int prio);

void sys_init(void);


#endif // UCORE_LWIP_SYS_ARCH_H_
