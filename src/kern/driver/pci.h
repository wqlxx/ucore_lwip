#ifndef XV6_PCI_H_
#define XV6_PCI_H_

#include <types.h>

typedef int pci_dev_t;
// PCI subsystem interface
enum {
    pci_res_bus, 
    pci_res_mem, 
    pci_res_io, 
    pci_res_max };

struct pci_bus;

// bus：一个指向pci_bus结构的指针，表示桥接器的主总线。
// dev：一个无符号整数，表示设备的PCI地址中的设备号。
// func：一个无符号整数，表示设备的PCI地址中的功能号。
// dev_id：一个无符号整数，表示设备的ID。
// dev_class：一个无符号整数，表示设备的类别。
// reg_base：一个无符号整数数组，表示设备的6个寄存器的基地址。
// reg_size：一个无符号整数数组，表示设备的6个寄存器的大小。
// reg_type：一个无符号整数数组，表示设备的6个寄存器的类型。
// irq_line：一个无符号字节，表示设备的中断线号。
struct pci_func {
    struct pci_bus *bus;	// Primary bus for bridges

    uint32_t dev;  // 设备id
    uint32_t func;  // 

    uint32_t dev_id;
    uint32_t dev_class;

    uint32_t reg_base[6];
    uint32_t reg_size[6];
    uint32_t reg_type[6];
    uint8_t irq_line;
};

struct pci_bus {
    struct pci_func *parent_bridge;
    uint32_t busno;
};

int pci_init(void);
void pci_func_enable(struct pci_func *f);

int pci_write_config_byte(int bus, int dev, int fn, int reg, uint8_t val);
int pci_read_config_byte(int bus, int dev, int fn, int reg, uint8_t *val);
int pci_write_config_word(int bus, int dev, int fn, int reg, uint16_t val);
int pci_read_config_word(int bus, int dev, int fn, int reg, uint16_t *val);
int pci_write_config_dword(int bus, int dev, int fn, int reg, uint32_t val);
int pci_read_config_dword(int bus, int dev, int fn, int reg, uint32_t *val);

#endif 
