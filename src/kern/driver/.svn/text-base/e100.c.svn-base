#include <types.h>
#include <x86.h>
#include <pci.h>
#include <pcireg.h>
#include <e100.h>
#include <proc.h>
#include <picirq.h>
#include <lwip/ip_addr.h>
#include <lwip/ip.h>
#include <lwip/netif.h>
#include <netif/e100if.h>
#include <slab.h>
#include <string.h>
#include <sched.h>
#include <pmm.h>
#include <netif/etharp.h>

//#define E100_DEBUG    1

static int dev_count = 0;

typedef struct e100_devinfo {
    int irq;
    uint32_t regbase[6], regsize[6];
    uint32_t iobase;
    struct netif netif;
    semaphore_t culock;
    uint32_t cubase;
    uint32_t cusize;
//    uint32_t cuhead;
//    uint32_t cuend;
    uint32_t cucount;
    int cufirst;
    int cuidle;
    command_block *cu_last_pkt;
    command_block *cu_next_pkt;
    semaphore_t rulock;
    uint32_t rubase;
    uint32_t rusize;
    uint32_t rucount;
    rf_desc *ru_first;
    rf_desc *ru_last;
    rf_desc *ru_prev;
    int ru_full;
    semaphore_t rxlock;
    int eeprom_size;
    char macaddr[6];
} e100_dev;

static e100_dev e100_devs[E100_MAX_DEVS];
int e100_reset(e100_dev *dev);
scb_status_word e100_print_state(e100_dev *dev);
void e100_put_state(e100_dev *dev, scb_status_word stat);
uint8_t e100_read_scb_command(e100_dev *dev);
void e100_cu_command(e100_dev *dev, int command, void *cmd_addr);
void e100_ru_command(e100_dev *dev, int command, void *cmd_addr);
void e100_intr(struct trapframe *tf);
void e100_ru_start(e100_dev *dev);
void ring_printinfo(e100_dev *dev);
void ring_init(e100_dev *dev);
int e100_receive_dev(e100_dev *dev, void *buffer, int len);
int e100_rx_thread(void *arg);
void e100_read_eeprom(e100_dev *dev, uint16_t *data, int offset, int len);

char *cu_state_name[];
char *ru_state_name[];

#define PAGE 4096
int ether_e100_attach(struct pci_func *pcif) {
    cprintf("PCI: %02x:%02x.%d: "
            "Intel 82559ER Fast Ethernet PCI Controller %04x:%04x\n",
        pcif->bus->busno, pcif->dev, pcif->func,
        PCI_VENDOR(pcif->dev_id), PCI_PRODUCT(pcif->dev_id));
    dev_count++;
    int index = dev_count-1;
    e100_dev *dev = &e100_devs[index];

    dev->cubase = (uint32_t)(kmalloc(E100_CU_RING_SIZE * PAGE)); // 128K
    if (!dev->cubase)
    {
        cprintf("   Failed to allocate CU ring\n");
        dev_count--;
        return -1;
    }
#ifdef E100_DEBUG
    cprintf("   CU ring %08x\n", PADDR(dev->cubase));
#endif
    dev->rubase = (uint32_t)(kmalloc(E100_RU_RING_SIZE * PAGE)); // 128K
    if (!dev->rubase)
    {
        cprintf("   Failed to allocate RU ring\n");
        dev_count--;
        return -1;
    }
#ifdef E100_DEBUG
    cprintf("   RU ring %08x\n", PADDR(dev->rubase));
#endif
    dev->cusize = E100_CU_RING_SIZE * PAGE;
    dev->rusize = E100_RU_RING_SIZE * PAGE;
    dev->eeprom_size = 6;
    ring_init(dev);
//    dev->cuhead = 0;
//    dev->cuend = 0;

    pci_func_enable(pcif);
    sem_init(&dev->culock, 1);
    cprintf("%08lx file: %s, line: %d \n", &dev->culock, __FILE__, __LINE__);
    sem_init(&dev->rulock, 1);
    cprintf("%08lx file: %s, line: %d \n", &dev->rulock, __FILE__, __LINE__);
    sem_init(&dev->rxlock, 0);
    cprintf("%08lx file: %s, line: %d \n", &dev->rxlock, __FILE__, __LINE__);
/*    initlock(&dev->culock, "e100 CU lock");
    initlock(&dev->rulock, "e100 RU lock");
    initlock(&dev->rxlock, "e100 RX lock");*/
    dev->irq = pcif->irq_line;
    int i;
    for (i=0; i<6; i++)
    {
        dev->regbase[i] = pcif->reg_base[i];
        dev->regsize[i] = pcif->reg_size[i];
        if (dev->regsize[i] == E100_IOPORT_SIZE)
            dev->iobase = dev->regbase[i];
    }
    e100_reset(dev);
    dev->cufirst = 1;
    dev->cuidle = 1;
    e100_print_state(dev);

    uint16_t ma[3];
    e100_read_eeprom(dev, ma, 0, 3);
    dev->macaddr[0] = ma[0] & 0xff;
    dev->macaddr[1] = (ma[0] >> 8) & 0xff;
    dev->macaddr[2] = ma[1] & 0xff;
    dev->macaddr[3] = (ma[1] >> 8) & 0xff;
    dev->macaddr[4] = ma[2] & 0xff;
    dev->macaddr[5] = (ma[2] >> 8) & 0xff;
    cprintf("   MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
            dev->macaddr[0],
            dev->macaddr[1],
            dev->macaddr[2],
            dev->macaddr[3],
            dev->macaddr[4],
            dev->macaddr[5]);

    struct ip_addr ipaddr;
    IP4_ADDR(&ipaddr, 192, 168, 1, 13);
    struct ip_addr netmask;
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    struct ip_addr gw;
    IP4_ADDR(&gw, 192, 168, 1, 1);

    netif_add(&dev->netif, &ipaddr, &netmask, &gw, 0, ethernetif_init, ethernet_input);
    netif_set_up(&dev->netif);
    struct ethernetif *eif = dev->netif.state;
    eif->receive = e100_receive;
    eif->send = e100_send;
    eif->ethaddr = (struct eth_addr *)dev->macaddr;
    dev->netif.hwaddr_len = 6;
    memcpy(dev->netif.hwaddr, dev->macaddr, 6);
    reg_irq_handler(dev->irq, e100_intr);
    pic_enable(dev->irq);
//    ioapic_enable(dev->irq, ncpu-1);
    kernel_thread(e100_rx_thread, dev, 0); //, 0, "[e100 rx thread]");
    e100_ru_start(dev);
#ifdef E100_DEBUG
    cprintf("e100 attach finished.\n");
#endif
    return 1;
}

static void delay(int n) {
    volatile int i;
    int j;
    for (j=0; j<n; j++)
    {
    for (i=0; i<1000; i++)
        ;
    }
}

int e100_reset(e100_dev *dev) {
    outl(dev->iobase + SCB_PORT, PORT_SOFT_RESET);
    delay(10);
    scb_command_word cmd;
    cmd.word = 0;
    cmd.cmd.cu_command = CUC_DUMP_RESET;
    outw(dev->iobase + SCB_COMMAND, cmd.word);
    return 0;
}

scb_status_word e100_get_state(e100_dev *dev) {
    scb_status_word st;
    st.word = inw(dev->iobase + SCB_STATUS);
    return st;
}

void e100_put_state(e100_dev *dev, scb_status_word stat) {
    outw(dev->iobase + SCB_STATUS, stat.word);
}

scb_status_word e100_print_state(e100_dev *dev) {
    scb_status_word state = e100_get_state(dev);
    cprintf("   RU Status: %s\n", ru_state_name[state.status.rus]);
    cprintf("   CU Status: %s\n", cu_state_name[state.status.cus]);

    struct scb_status_word stat = state.status;
    if (stat.cx) cprintf("   CU executed\n");
    if (stat.fr)
        cprintf("   RU received\n");
    if (stat.cna)
        cprintf("   CU state change\n");
    if (stat.rnr)
        cprintf("   RU not ready\n");
    if (stat.mdi)
        cprintf("   MDI operation completed\n");
    if (stat.swi)
        cprintf("   Software Interrupt\n");
    if (stat.fcp)
        cprintf("   Flow Control Pause\n");

    return state;
}

char *cu_state_name[4] = {
    "Idle",
    "Suspended",
    "LPQ Active",
    "HQP Active",
};

char *ru_state_name[16] = {
    "Idle",
    "Suspended",
    "No resources",
    "Reserved",
    "Ready",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
};

uint8_t e100_read_scb_command(e100_dev *dev) {
    return inb(dev->iobase + SCB_COMMAND + 1);
}

void e100_set_gp(e100_dev *dev, void *addr) {
    outl(dev->iobase + SCB_GENPTR, (uint32_t)addr);
}

void e100_cu_command(e100_dev *dev, int command, void *cmd_addr) {
    scb_command_word cmd;
    cmd.word = 0;
    cmd.cmd.cu_command = command;
    e100_set_gp(dev, cmd_addr);
    outw(dev->iobase + SCB_COMMAND, cmd.word);
    while (e100_read_scb_command(dev)) ;
}

void e100_ru_command(e100_dev *dev, int command, void *cmd_addr) {
    scb_command_word cmd;
    cmd.word = 0;
    cmd.cmd.ru_command = command;
    e100_set_gp(dev, cmd_addr);
    outw(dev->iobase + SCB_COMMAND, cmd.word);
    while (e100_read_scb_command(dev)) ;
}

// Interrupt
void e100_intr(struct trapframe *tf) {
#ifdef E100_DEBUG_INTR
    cprintf("e100_intr\n");
#endif
    int i;
    int index = -1;
    for (i=0; i< dev_count; i++)
    {
        if (tf->tf_trapno == IRQ_OFFSET + e100_devs[i].irq)
        {
            index = i;
            break;
        }
    }
    if (index == -1)
    {
        cprintf("e100_intr: can't find corresponding device\n");
        return;
    }
    e100_dev *dev = &e100_devs[index];
#ifdef E100_DEBUG_INTR
    e100_print_state(dev);
#endif
    scb_status_word state = e100_get_state(dev);
    struct scb_status_word stat = state.status;
    state.word |= 0xff00;
    e100_put_state(dev, state);

    if (stat.cx)
    {
#ifdef E100_DEBUG
        cprintf("   CU executed\n");
#endif
    }
    if (stat.fr)
    {
        if (!dev->ru_first)
        {
            dev->ru_first = dev->ru_last;
//            wakeup(&dev->ru_first);
        }
        up(&dev->rxlock);
/*        while (dev->ru_last->eof)
        {
            ethernetif_input(&dev->netif);
            dev->ru_last = (void*)dev->ru_last->head.link;
        }*/
    }
    if (stat.cna)
    {
/*        if (dev->cu_next_pkt != 0)
        {
            cprintf("   finished 0x%08x\n", dev->cu_next_pkt);
            if (dev->cu_next_pkt->link != 0)
            {
                cprintf("   starting next packet: "
                        "0x%08x\n", dev->cu_next_pkt->link);
                e100_cu_command(dev, CUC_START, 
                        (void*)dev->cu_next_pkt->link);
                dev->cuhead = dev->cu_next_pkt->link - dev->cubase;
                dev->cu_next_pkt = (command_block *)dev->cu_next_pkt->link;
            }
            else
            {
                dev->cuhead = dev->cuend;
                dev->cu_last_pkt = 0;
                dev->cu_next_pkt = 0;
            }
            ring_printinfo(dev);
        }*/
        if (dev->cu_last_pkt)
        {
            if (dev->cu_last_pkt->c)
            {
                dev->cuidle = 1;
//                dev->cuhead = dev->cuend;
                dev->cu_next_pkt = 0;
//                dev->cu_last_pkt = 0;
            } else {
                dev->cuidle = 0;
                while (dev->cu_next_pkt->c)
                {
                    if (!dev->cu_next_pkt->ok)
                        cprintf("send error 0x%08x\n", dev->cu_next_pkt);
                    dev->cu_next_pkt = (void*)dev->cu_next_pkt->link;
                }
                e100_cu_command(dev, CUC_RESUME, 0);
//                dev->cuhead = (uint32_t)dev->cu_next_pkt - dev->cubase;
            }
        }
//        ring_printinfo(dev);
    }
    if (stat.rnr)
    {
        cprintf("e100: RU overrun!\n");
        dev->ru_full = 1;
    }
    if (stat.mdi)
        cprintf("   MDI operation completed\n");
    if (stat.swi)
        cprintf("   Software Interrupt\n");
    if (stat.fcp)
        cprintf("   Flow Control Pause\n");

}

// Ring management
void ring_printinfo(e100_dev *dev) {
    cprintf("Ring Info:   ");
    cprintf("head: 0x%08x  ", dev->cu_next_pkt);
    cprintf("end:  0x%08x\n", dev->cu_last_pkt);
}

void ring_init(e100_dev *dev) {
    int i;
    dev->cucount = dev->cusize / E100_CB_SIZE;
    dev->cu_next_pkt = 0;
    for (i=0; i<dev->cucount ; i++)
    {
        command_block *cmd = (void*)((dev->cubase) + i * E100_CB_SIZE);
        cmd->cmd_word = 0;
        cmd->status_word = 0;
        cmd->c = 1;
        cmd->el = 1;
        if (i == dev->cucount - 1)
        {
            cmd->link = (void*)PADDR(dev->cubase);
            dev->cu_last_pkt = cmd;
        }
        else
        {
            cmd->link = (void*)PADDR((uint32_t)cmd + E100_CB_SIZE);
        }
#ifdef E100_DEBUG_RING
        cprintf("CB %d: start 0x%08x next 0x%08x\n", i, cmd, cmd->link);
#endif
    }
    dev->rucount = dev->rusize / E100_RFD_SIZE;
    for (i=0; i<dev->rucount; i++)
    {
        rf_desc *rfd = (void*)((dev->rubase) + i * E100_RFD_SIZE);
        rfd->head.cmd_word = 0;
        rfd->head.status_word = 0;
        rfd->size = E100_RFD_SIZE - sizeof(rf_desc);
        if (rfd->size & 1)
            rfd->size--;
        rfd->eof = 0;
        rfd->f = 0;
        if (i == dev->rucount - 1)
        {
            rfd->head.link = (void*)PADDR(dev->rubase);
            rfd->head.el = 1;
            dev->ru_prev = rfd;
        }
        else
        {
            rfd->head.link = (void*)PADDR((uint32_t)rfd + E100_RFD_SIZE);
        }
#ifdef E100_DEBUG_RING
        cprintf("RFD %d: start 0x%08x next 0x%08x\n", i, rfd, rfd->head.link);
#endif
    }
    dev->ru_first = 0;
    dev->ru_last = (void*)(dev->rubase);
    
}

void * ring_alloc(e100_dev *dev, uint32_t len) {
    if (len > E100_CB_SIZE)
        return (void*) 0;
    if (dev->cu_next_pkt)
    {
        if ((void*)dev->cu_last_pkt->link == dev->cu_next_pkt)
            return (void*) 0;
        return (void*)KADDR((uintptr_t)dev->cu_last_pkt->link);
    } else {
        return (void*)KADDR((uintptr_t)dev->cu_last_pkt->link);
    }
/*    
    len = E100_CB_SIZE;
    if (dev->cuhead <= dev->cuend)
    {
        uint32_t newend = dev->cuend + len;
        uint32_t oldend = dev->cuend;
        if (newend >= dev->cusize)
        {
            newend = len; // Start from base
            if (newend < dev->cuhead)
            {
                dev->cuend = newend;
                return (void*)dev->cubase;
            }
            else
                return (void*)0;
        }
        else
        {
            dev->cuend = newend;
            return (void*)(oldend + dev->cubase);
        }
    }
    else
    {
        uint32_t newend = dev->cuend + len;
        uint32_t oldend = dev->cuend;
        if (newend < dev->cuhead)
        {
            dev->cuend = newend;
            return (void*)(oldend + dev->cubase);
        }
        else
            return (void*)0;
    }*/
}

// Send
int e100_send_dev(e100_dev *dev, void *buffer, uint32_t len) {
#ifdef E100_DEBUG
    cprintf("e100_send_dev\n");
#endif
    if (len > TBD_DATA_LIMIT)
        return -1; // ETOOBIG
    down(&dev->culock);
    op_transmit_cmd *start = 
        ring_alloc(dev, sizeof(op_transmit_cmd) + len);
#ifdef E100_DEBUG
    cprintf("   start: 0x%08x\n", start);
#endif
    if (!start)
    {
        up(&dev->culock);
        return -1; // No space in buffer
    }
    op_transmit_cmd_cmd cmd_word;
    cmd_word.word = 0;
    cmd_word.cmd = OP_TRANSMIT;
//    cmd_word.el = 0;
    cmd_word.cid = 0xe;
    cmd_word.i = 1;
    cmd_word.s = 1;
    if (!dev->cuidle)
        dev->cu_last_pkt->s = 0;
    start->base_cmd.cmd_word = cmd_word.word;
    start->base_cmd.status_word = 0;
//    start->base_cmd.link = 0;
    start->tbd_addr = 0xffffffff;
    start->byte_count = len;
    start->eof = 1;
    start->trans_thres = 0xE0;
    start->tbd_num = 0;
    memcpy((void*)(start+1), buffer, len);
/*  int i;
    for (i=0; i<len; i++)
        cprintf("%02x ", ((unsigned char*)buffer)[i]);
    cprintf("\n");*/
    
//    e100_print_state(dev);
//    if (e100_get_state(dev).status.cus == CUS_IDLE)
//    if (dev->cu_next_pkt == 0)
//    This test would fail in real i82559er
//    But it passed in qemu
/*   if ((!dev->cufirst) && (!dev->cuidle))
        if (start->base_cmd.link == dev->cu_next_pkt)
            e100_cu_command(dev, CUC_RESUME, start);*/
    if (dev->cufirst)
    {
        dev->cufirst = 0;
        dev->cu_next_pkt = &start->base_cmd;
#ifdef E100_DEBUG
        cprintf("CUC_START\n");
#endif
        e100_cu_command(dev, CUC_START, (void *)PADDR(start));
    }
    else if (dev->cuidle)
    {
        dev->cuidle = 0;
        dev->cu_next_pkt = &start->base_cmd;
#ifdef E100_DEBUG
        cprintf("CUC_RESUME\n");
#endif
        e100_cu_command(dev, CUC_RESUME, (void *)PADDR(start)); // the GENPTR is nonsense
    }
    dev->cu_last_pkt = &start->base_cmd;
    up(&dev->culock);
    return len;
}

int e100_send(void *buffer, uint32_t len) {
    if (dev_count == 0)
        return -2; // Device not found
    return e100_send_dev(&e100_devs[0], buffer,len);
}

// Receive
void e100_ru_start(e100_dev *dev) {
#ifdef E100_DEBUG
    cprintf("e100_ru_start addr: %08x\n", PADDR(dev->rubase));
#endif
    dev->ru_full = 0;
    e100_ru_command(dev, RUC_START, (void*)PADDR(dev->rubase));
}
 
int e100_receive(void *buffer, uint32_t len) {
    if (dev_count == 0)
        return -2; // Device not found
    return e100_receive_dev(&e100_devs[0], buffer, len);
}

int e100_receive_dev(e100_dev *dev, void *buffer, int len) {
    int count;
#ifdef E100_DEBUG
    cprintf("e100_receive_dev\n");
#endif
    down(&dev->rulock);
    while (dev->ru_first == 0)
        schedule();
#ifdef E100_DEBUG
    cprintf("ru_first: %08x\n", dev->ru_first);
#endif
//        sleep(&dev->rulock, &dev->rulock);
    if (len < dev->ru_first->count)
    {
        up(&dev->rulock);
        return -1; // ETOOBIG
    }
    memcpy(buffer, dev->ru_first + 1, dev->ru_first->count);
#ifdef E100_DEBUG
    cprintf("received %d bytes into 0x%08x\n", dev->ru_first->count, buffer);
#endif
    count = dev->ru_first->count;
    dev->ru_first->eof = 0;
    dev->ru_first->f = 0;
    dev->ru_prev->head.el = 0;
    dev->ru_first->head.el = 1;
    dev->ru_prev = dev->ru_first;
    dev->ru_first = (struct rf_desc *)KADDR((uintptr_t)dev->ru_first->head.link);
    if (dev->ru_full)
    {
        // RU full, in No Resource state
        dev->ru_full = 0;
        e100_ru_command(dev, RUC_START, (void *)PADDR(dev->ru_prev));
    }
    if (dev->ru_first == dev->ru_last)
        dev->ru_first = 0;
    up(&dev->rulock);
    return count;
}

int e100_rx_thread(void *arg) {
#ifdef E100_DEBUG
    cprintf("e100_rx_thread start\n");
#endif
    e100_dev *dev = arg;
    while(1)
    {
        down(&dev->rxlock);
#ifdef E100_DEBUG
        cprintf("rx_thread wakeup! %d\n", dev->ru_last->eof);
#endif
//        sleep(&dev->rxlock, &dev->rxlock);
//      schedule();
        while (dev->ru_last->eof)
        {
#ifdef E100_DEBUG
            cprintf("Read! %d\n", dev->ru_last->eof);
#endif
            ethernetif_input(&dev->netif);
#ifdef E100_DEBUG
            cprintf("advance ru_last: from %08x to %08x\n", dev->ru_last, KADDR(dev->ru_last->head.link));
#endif
            dev->ru_last = (void*)KADDR((uintptr_t)dev->ru_last->head.link);
        }
    }
//    up(&dev->rxlock);
}

// EEPROM
void e100_eeprom_out(e100_dev *dev, uint16_t val) {
    outw(dev->iobase + SCB_EEPROM_CTL, val);
}

uint16_t e100_eeprom_in(e100_dev *dev) {
    return inw(dev->iobase + SCB_EEPROM_CTL);
}

void e100_eeprom_shiftin(e100_dev *dev, int data, int len) {
    int val;
    int x;
    for (x = 1 << (len - 1); x > 0; x >>= 1) {
        if (data & x)
            val = EEPROM_EEDI | EEPROM_EECS;
        else
            val = EEPROM_EECS;
        e100_eeprom_out(dev, val);
        delay(1);
        e100_eeprom_out(dev, val | EEPROM_EESK);
        delay(1);
        e100_eeprom_out(dev, val);
        delay(1);
    }
}

uint16_t e100_eeprom_shiftout(e100_dev *dev) {
    int val;
    int data = 0;
    val = EEPROM_EECS;
    int x;
    for (x = 1 << 15; x > 0; x >>= 1)
    {
        e100_eeprom_out(dev, val | EEPROM_EESK);
        delay(1);
        if (e100_eeprom_in(dev) & EEPROM_EEDO)
            data |= x;
        e100_eeprom_out(dev, val);
        delay(1);
    }
    return data;
}

uint16_t e100_eeprom_getword(e100_dev *dev, int offset) {
    uint16_t data;
    e100_eeprom_out(dev, EEPROM_EECS);
    e100_eeprom_shiftin(dev, EEPROM_OP_READ, 3);
    e100_eeprom_shiftin(dev, offset, dev->eeprom_size);
    data = e100_eeprom_shiftout(dev);
    e100_eeprom_out(dev, 0);
    return data;
}

void
e100_read_eeprom(e100_dev *dev, uint16_t *data, int offset, int len) {
    int i;
    for (i=0; i<len; i++)
        data[i] = e100_eeprom_getword(dev, offset+i);
}
