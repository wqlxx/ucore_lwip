#include <arch/sys_arch.h>
#include <sem.h>
#include <mbox.h>
#include <lwip/sys.h>
#include <proc.h>
#include <slab.h>
#include <clock.h>
#include <sched.h>

struct mbox {
	sys_sem_t free, used, lock;
	int head, next;
	void *slots[_MBOX_SIZE];
};

//#define LWIP_ARCH_DEBUG

#ifdef USE_IPC_SEM
sys_sem_t sys_sem_new(u8_t count)
{
	int sem = ipc_sem_init(count);
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem_new count %d return: %d\n", current->pid, count, sem);
#endif
	return sem;
}

void sys_sem_free(sys_sem_t sem)
{
	ipc_sem_free(sem);
}

void sys_sem_signal(sys_sem_t sem)
{
	ipc_sem_post(sem);
}

u32_t sys_arch_sem_wait(sys_sem_t sem, u32_t timeout)
{
	int ret = ipc_sem_wait(sem, timeout);
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem: %d ret: %d timeout: %d\n", current->pid, sem, ret, timeout);
#endif
	if (ret == 0)
		return 0;
	else
		return SYS_ARCH_TIMEOUT;
}
#else

sys_sem_t sys_sem_new(u8_t count)
{
	semaphore_t *sem = (semaphore_t *)kmalloc(sizeof(semaphore_t));
	sem_init(sem, count);
    //cprintf("%08lx file: %s, line: %d \n", sem, __FILE__, __LINE__);
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem_new count %d return %08x\n", current->pid, count, sem);
#endif
	return sem;
}

void sys_sem_free(sys_sem_t sem)
{
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem_free %08x\n", current->pid, sem);
#endif
	kfree(sem);
}

void sys_sem_signal(sys_sem_t sem)
{
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem_up %08x\n", current->pid, sem);
#endif
	up(sem);
}

u32_t sys_arch_sem_wait(sys_sem_t sem, u32_t timeout)
{
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] sem_down %08x timeout %d\n", current->pid, sem, timeout);
#endif
	if (timeout == 0)
	{
		down(sem);
		return 0;
	} else {
		timeout = timeout / 10;
		int tick_start = ticks;
		while (1) {
			bool ret = try_down(sem);
			if (ret == 1)
			{
				return (ticks - tick_start) * 10;
			} else {
				if (ticks - tick_start > timeout)
				{
					return SYS_ARCH_TIMEOUT;
				}
			}
			schedule();
		}
	}
}

#endif

sys_mbox_t sys_mbox_new(int size)
{
	if (size > _MBOX_SIZE)
		return NULL;
#ifdef LWIP_ARCH_DEBUG
	cprintf("[%d] mbox_new size: %d\n", current->pid, size);
#endif
	struct mbox *m = kmalloc(sizeof(struct mbox));
	if (size == 0) size = _MBOX_SIZE;
	m->free = sys_sem_new(size);
	m->used = sys_sem_new(0);
	m->lock = sys_sem_new(1); // since we do not have spinlock, ...
	m->head = 0;
	m->next = 0;
	return m;
}

void sys_mbox_free(sys_mbox_t mbox)
{
	sys_sem_free(mbox->free);
	sys_sem_free(mbox->used);
	sys_sem_free(mbox->lock);
	kfree(mbox);
}

static void sys_mbox_dopost(sys_mbox_t mbox, void *msg)
{
	sys_arch_sem_wait(mbox->lock, 0);
	{
		int slot = mbox->next;
		mbox->next = (slot + 1) % _MBOX_SIZE;
		mbox->slots[slot] = msg;
	}
	sys_sem_signal(mbox->lock);
	sys_sem_signal(mbox->used);
}

void sys_mbox_post(sys_mbox_t mbox, void *msg)
{
	int ret = sys_arch_sem_wait(mbox->free, 0);
	assert(ret != SYS_ARCH_TIMEOUT);
	sys_mbox_dopost(mbox, msg);
}

err_t sys_mbox_trypost(sys_mbox_t mbox, void *msg)
{
	int ret = sys_arch_sem_wait(mbox->free, 1);
	if (ret == SYS_ARCH_TIMEOUT)
		return ERR_MEM;
	sys_mbox_dopost(mbox, msg);
	return ERR_OK;
}

static void sys_mbox_dofetch(sys_mbox_t mbox, void **msg)
{
	sys_arch_sem_wait(mbox->lock, 0);
	{
		int slot = mbox->head;
		mbox->head = (slot + 1) % _MBOX_SIZE;
		if (msg != NULL)
			*msg = mbox->slots[slot];
	}
	sys_sem_signal(mbox->lock);
	sys_sem_signal(mbox->free);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t mbox, void **msg, u32_t timeout)
{
	int ret = sys_arch_sem_wait(mbox->used, timeout);
	if (ret == SYS_ARCH_TIMEOUT)
		return ret;
	sys_mbox_dofetch(mbox, msg);
	return ret;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t mbox, void **msg)
{
	int ret = sys_arch_sem_wait(mbox->used, 1);
	if (ret == SYS_ARCH_TIMEOUT)
		return SYS_MBOX_EMPTY;
	sys_mbox_dofetch(mbox, msg);
	return 0;
}

struct sys_timeouts *sys_arch_timeouts(void)
{
	assert(current);
	if (current->lwip_timeouts == NULL) {
		current->lwip_timeouts = (struct sys_timeouts *)kmalloc(sizeof(struct sys_timeouts));
        current->lwip_timeouts->next = NULL;
    }
	return current->lwip_timeouts;
}

typedef int (*kthrfunc)(void*);

sys_thread_t sys_thread_new(char *name, void (* thread)(void *arg), void *arg, int stacksize, int prio)
{
    int pid = kernel_thread((kthrfunc)thread, arg, 0);
    if (pid <= 0) {
        panic("create tcp_ip thread failed");
    }
    set_proc_name(find_proc(pid), name);
	return pid;
}

void sys_init(void)
{
}


