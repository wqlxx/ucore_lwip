#ifndef XV6_E100_H_
#define XV6_E100_H_
#include "pci.h"

int ether_e100_attach(struct pci_func *pcif);
int e100_send(void *buffer, uint32_t len);
int e100_receive(void *buffer, uint32_t len);

#define PCI_VENDOR_INTEL    0x8086
#define PCI_PRODUCT_E100    0x1209

#define E100_MAX_DEVS       10

#define E100_IOPORT_SIZE    64
#define E100_CU_RING_SIZE   32
#define E100_RU_RING_SIZE   32
#define E100_CB_SIZE       2048
#define E100_RFD_SIZE      2048

// Control / Status Register
#define SCB_STATUS          0x0 // Status
#define SCB_COMMAND         0x2 // Command
#define SCB_GENPTR          0x4 // General Pointer
#define SCB_PORT            0x8 // PORT
#define SCB_EEPROM_CTL      0xE // EEPROM Control
#define SCB_MDI_CTL         0x10    // MDI Control
#define SCB_RX_COUNT        0x14    // RX DMA Byte Count
#define SCB_FLOW_CTL        0x19    // Flow Control
#define SCB_PMDR            0x21    // PMDR
#define SCB_GEN_CTL         0x1C    // General Control
#define SCB_GEN_STATUS      0x1D    // General Status
#define SCB_FUNC_EVT        0x30    // Function Event
#define SCB_FUN_MASK        0x34    // Function Event Mask
#define SCB_FUNC_STATE      0x38    // Function Present State
#define SCB_FORCE_EVT       0x3C    // Force Event

// CU Command
#define CUC_NOP             0x0
#define CUC_START           0x1
#define CUC_RESUME          0x2
#define CUC_LOAD_DC_ADDR    0x4
#define CUC_DUMP            0x5
#define CUC_LOAD_BASE       0x6
#define CUC_DUMP_RESET      0x7
#define CUC_STAT_RESUME     0xA

// RU Command
#define RUC_NOP             0x0
#define RUC_START           0x1
#define RUC_RESUME          0x2
#define RUC_RCV_DMA         0x3
#define RUC_ABORT           0x4
#define RUC_LOAD_HDS        0x5
#define RUC_LOAD_BASE       0x6

// PORT Function
#define PORT_SOFT_RESET     0
#define PORT_SELF_TEST      1
#define PORT_SElECT_TEST    2
#define PORT_DUMP           3
#define PORT_DUMP_WAKE      7

// EEPROM OP
#define EEPROM_OP_READ      6
#define EEPROM_OP_WRITE     5
#define EEPROM_OP_ERASE     7
#define EEPROM_OP_OTHER     4

// EEPROM Control
#define EEPROM_EEDO         1 << 3
#define EEPROM_EEDI         1 << 2
#define EEPROM_EECS         1 << 1
#define EEPROM_EESK         1 << 0

// Operation Codes
#define OP_NOP              0
#define OP_ADDR_SETUP       1
#define OP_CONFIG           2
#define OP_MUL_ADDR_SETUP   3
#define OP_TRANSMIT         4
#define OP_LOAD_MC          5
#define OP_DUMP             6
#define OP_DIAG             7

// CU Status
#define CUS_IDLE            0
#define CUS_SUSPENDED       1
#define CUS_LPQ             2
#define CUS_HQP             3

struct scb_status_word {
    char zero : 2;
    char rus  : 4;
    char cus  : 2;
    // STAT / ACK
    char fcp  : 1;
    char rsv  : 1;
    char swi  : 1;
    char mdi  : 1;
    char rnr  : 1;
    char cna  : 1;
    char fr   : 1;
    char cx   : 1;
};

typedef union scb_status_word_tag {
    uint16_t word;
    struct scb_status_word status;
} scb_status_word;

struct scb_command_word {
    char ru_command : 3;
    char res : 1;
    char cu_command : 4;
    // Interrupt Masks
    char m : 1;
    char si : 1;
    char fcp_mask : 1;
    char er_mask : 1;
    char rnr_mask : 1;
    char cna_mask : 1;
    char fr_mask : 1;
    char cx_mask : 1;
};

typedef union scb_command_word_tag {
    uint16_t word;
    struct scb_command_word cmd;
} scb_command_word;

typedef struct command_block_tag {
    volatile union {
        uint16_t status_word;
        struct {
            short stat :    13;
            char ok :       1;    // No Error
            char x :        1;
            char c :        1;     // Completed
        };
    };
    union {
        uint16_t cmd_word;
        struct {
            char cmd :  3;
            short res : 10;
            char i :    1;     // Interrupt after finish
            char s :    1;     // Suspend after complete
            char el :   1;    // Last one
        };
    };
    struct command_block_tag * link;
} command_block;

typedef struct op_transmit_cmd_cmd_tag {
    union {
        uint16_t word;
        struct {
            char cmd    : 3;
            char sf     : 1;
            char nc     : 1;
            char res    : 3;
            char cid    : 5;
            char i      : 1;
            char s      : 1;
            char el     : 1;
        };
    };
} op_transmit_cmd_cmd;

typedef struct op_transmit_cmd {
    command_block base_cmd;
    uint32_t tbd_addr;
    struct {
        int byte_count :14;
        char res :      1;
        char eof :      1;
        uint8_t trans_thres;
        uint8_t tbd_num;
    };
} op_transmit_cmd;

typedef struct rf_desc {
    volatile command_block head;
    uint32_t res;
    volatile struct {
        uint16_t count :    14;
        char f :            1;
        char eof :          1;
        uint16_t size :     14;
        char res2 :         2;
    };
} rf_desc;

typedef struct rf_status {
    char tco :      1;
    char ia :       1;
    char nomatch :  1;
    char res :      1;
    char rcv_err :  1;
    char type :     1;
    char res2 :     1;
    char tooshort : 1;
    char dma_err :  1;
    char no_buf :   1;
    char align_err :1;
    char crc_err :  1;
    char res3 :     1;
} rf_status;

#define TBD_DATA_LIMIT  1600


#endif	// XV6_E100_H_
