#ifndef __KERN_DRIVER_PICIRQ_H__
#define __KERN_DRIVER_PICIRQ_H__
#include <trap.h>

typedef void (*irq_handler_t)(struct trapframe *);
extern irq_handler_t irq_handler[];
void reg_irq_handler(int irq_num, irq_handler_t handler);

void pic_init(void);
void pic_enable(unsigned int irq);

#define IRQ_OFFSET		32
#define IRQ_MAX			256

#endif /* !__KERN_DRIVER_PICIRQ_H__ */

