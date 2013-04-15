#include <stdio.h>
#include <e1000.h>
#include <pcireg.h>
#include <x86.h>
#include <string.h>
#include <assert.h>
#include <lwip/ip_addr.h>
#include <lwip/ip.h>
#include <lwip/netif.h>
#include <netif/e100if.h>
#include <netif/etharp.h>

static int dev_count = 0;

struct e1000_hw devs[10];

static char tx_pool[128 + 16];
static char rx_pool[128 + 16];
static char packet[2096];

static struct e1000_tx_desc *tx_base;
static struct e1000_rx_desc *rx_base;

static int tx_tail;
static int rx_tail, rx_last;

static void udelay(int n) {
    volatile int i;
    int j;
    for (j=0; j < n; j++)
        for (i = 0; i < 1000; i++);
}

static void mdelay(n) {
    int i;
    for (i = 0; i < n; i++)
        udelay(1000);
}

static int e1000_read_eeprom(struct e1000_hw *hw, uint16_t offset, uint16_t * data);
static int e1000_setup_copper_link(struct e1000_hw*);
static int e1000_setup_fiber_link(struct e1000_hw *hw);

// doc p217
static inline int e1000_read_reg(struct e1000_hw *dev, uint32_t offset) {
    // set address
    outl(dev->iobase, offset);
    return inl(dev->iobase + 4);
}

static inline void e1000_write_reg(struct e1000_hw *dev, uint32_t offset, uint32_t val) {
    outl(dev->iobase, offset);
    outl(dev->iobase + 4, val);
}

#define E1000_READ_REG(dev,reg) (e1000_read_reg(dev, E1000_##reg))
#define E1000_WRITE_REG(dev,reg,val) (e1000_write_reg(dev, E1000_##reg, val))
#define E1000_WRITE_FLUSH(a) {uint32_t x; x = E1000_READ_REG(a, STATUS);}
#define E1000_WRITE_REG_ARRAY(a, reg, offset, value) (\
            e1000_write_reg(a, E1000_##reg + ((offset) << 2), value))
#define E1000_READ_REG_ARRAY(a, reg, offset) ( \
    e1000_read_reg(a, E1000_##reg + ((offset) << 2)))


/******************************************************************************
 * Raises the EEPROM's clock input.
 *
 * hw - Struct containing variables accessed by shared code
 * eecd - EECD's current value
 *****************************************************************************/
static void
e1000_raise_ee_clk(struct e1000_hw *hw, uint32_t * eecd)
{
    /* Raise the clock input to the EEPROM (by setting the SK bit), and then
     * wait 50 microseconds.
     */
    *eecd = *eecd | E1000_EECD_SK;
    E1000_WRITE_REG(hw, EECD, *eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);
}

/******************************************************************************
 * Lowers the EEPROM's clock input.
 *
 * hw - Struct containing variables accessed by shared code
 * eecd - EECD's current value
 *****************************************************************************/
static void
e1000_lower_ee_clk(struct e1000_hw *hw, uint32_t * eecd)
{
    /* Lower the clock input to the EEPROM (by clearing the SK bit), and then
     * wait 50 microseconds.
     */
    *eecd = *eecd & ~E1000_EECD_SK;
    E1000_WRITE_REG(hw, EECD, *eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);
}

/******************************************************************************
 * Verifies that the EEPROM has a valid checksum
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Reads the first 64 16 bit words of the EEPROM and sums the values read.
 * If the the sum of the 64 16 bit words is 0xBABA, the EEPROM's checksum is
 * valid.
 *****************************************************************************/
static int
e1000_validate_eeprom_checksum(struct e1000_hw *hw)
{
    uint16_t checksum = 0;
    uint16_t i, eeprom_data;

    DEBUGFUNC();

    for (i = 0; i < (EEPROM_CHECKSUM_REG + 1); i++) {
        if (e1000_read_eeprom(hw, i, &eeprom_data) < 0) {
            DEBUGOUT("EEPROM Read Error\n");
            return -E1000_ERR_EEPROM;
        }
        checksum += eeprom_data;
    }

    if (checksum == (uint16_t) EEPROM_SUM) {
        DEBUGOUT("[ II ] EEPROM Checksum correct\n");
        return 0;
    } else {
        DEBUGOUT("[ EE ] EEPROM Checksum Invalid\n");
        return -E1000_ERR_EEPROM;
    }
}

/******************************************************************************
 * Detects the current speed and duplex settings of the hardware.
 *
 * hw - Struct containing variables accessed by shared code
 * speed - Speed of the connection
 * duplex - Duplex setting of the connection
 *****************************************************************************/
static void
e1000_get_speed_and_duplex(struct e1000_hw *hw, uint16_t * speed, uint16_t * duplex) {
    uint32_t status;

    DEBUGFUNC();

    status = E1000_READ_REG(hw, STATUS);
    if (status & E1000_STATUS_SPEED_1000) {
        *speed = SPEED_1000;
        DEBUGOUT("1000 Mbs, ");
    } else if (status & E1000_STATUS_SPEED_100) {
        *speed = SPEED_100;
        DEBUGOUT("100 Mbs, ");
    } else {
        *speed = SPEED_10;
        DEBUGOUT("10 Mbs, ");
    }

    if (status & E1000_STATUS_FD) {
        *duplex = FULL_DUPLEX;
        DEBUGOUT("Full Duplex\r\n");
    } else {
        *duplex = HALF_DUPLEX;
        DEBUGOUT(" Half Duplex\r\n");
    }
}

/******************************************************************************
 * Forces the MAC's flow control settings.
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Sets the TFCE and RFCE bits in the device control register to reflect
 * the adapter settings. TFCE and RFCE need to be explicitly set by
 * software when a Copper PHY is used because autonegotiation is managed
 * by the PHY rather than the MAC. Software must also configure these
 * bits when link is forced on a fiber connection.
 *****************************************************************************/
static int
e1000_force_mac_fc(struct e1000_hw *hw) {
    uint32_t ctrl;

    DEBUGFUNC();

    /* Get the current configuration of the Device Control Register */
    ctrl = E1000_READ_REG(hw, CTRL);

    /* Because we didn't get link via the internal auto-negotiation
     * mechanism (we either forced link or we got link via PHY
     * auto-neg), we have to manually enable/disable transmit an
     * receive flow control.
     *
     * The "Case" statement below enables/disable flow control
     * according to the "hw->fc" parameter.
     *
     * The possible values of the "fc" parameter are:
     *      0:  Flow control is completely disabled
     *      1:  Rx flow control is enabled (we can receive pause
     *          frames but not send pause frames).
     *      2:  Tx flow control is enabled (we can send pause frames
     *          frames but we do not receive pause frames).
     *      3:  Both Rx and TX flow control (symmetric) is enabled.
     *  other:  No other values should be possible at this point.
     */

    switch (hw->fc) {
    case e1000_fc_none:
        ctrl &= (~(E1000_CTRL_TFCE | E1000_CTRL_RFCE));
        break;
    case e1000_fc_rx_pause:
        ctrl &= (~E1000_CTRL_TFCE);
        ctrl |= E1000_CTRL_RFCE;
        break;
    case e1000_fc_tx_pause:
        ctrl &= (~E1000_CTRL_RFCE);
        ctrl |= E1000_CTRL_TFCE;
        break;
    case e1000_fc_full:
        ctrl |= (E1000_CTRL_TFCE | E1000_CTRL_RFCE);
        break;
    default:
        DEBUGOUT("Flow control param set incorrectly\n");
        return -E1000_ERR_CONFIG;
    }

    E1000_WRITE_REG(hw, CTRL, ctrl);
    return 0;
}

/******************************************************************************
 * Reset the transmit and receive units; mask and clear all interrupts.
 *
 * hw - Struct containing variables accessed by shared code
 *****************************************************************************/
void e1000_reset_hw(struct e1000_hw *hw) {
    uint32_t ctrl;
    uint32_t ctrl_ext;
    uint32_t icr;
    uint32_t manc;

    DEBUGFUNC();

    /* Clear interrupt mask to stop board from generating interrupts */
    DEBUGOUT("Masking off all interrupts\n");
    E1000_WRITE_REG(hw, IMC, 0xffffffff);

    /* Disable the Transmit and Receive units.  Then delay to allow
     * any pending transactions to complete before we hit the MAC with
     * the global reset.
     */
    E1000_WRITE_REG(hw, RCTL, 0);
    E1000_WRITE_REG(hw, TCTL, E1000_TCTL_PSP);
    E1000_WRITE_FLUSH(hw);

    /* The tbi_compatibility_on Flag must be cleared when Rctl is cleared. */
    hw->tbi_compatibility_on = FALSE;

    /* Delay to allow any outstanding PCI transactions to complete before
     * resetting the device
     */
    mdelay(10);

    /* Issue a global reset to the MAC.  This will reset the chip's
     * transmit, receive, DMA, and link units.  It will not effect
     * the current PCI configuration.  The global reset bit is self-
     * clearing, and should clear within a microsecond.
     */
    DEBUGOUT("Issuing a global reset to MAC\n");
    ctrl = E1000_READ_REG(hw, CTRL);

    E1000_WRITE_REG(hw, CTRL, (ctrl | E1000_CTRL_RST));

    /* Force a reload from the EEPROM if necessary */
    if (hw->mac_type < e1000_82540) {
        /* Wait for reset to complete */
        udelay(10);
        ctrl_ext = E1000_READ_REG(hw, CTRL_EXT);
        ctrl_ext |= E1000_CTRL_EXT_EE_RST;
        E1000_WRITE_REG(hw, CTRL_EXT, ctrl_ext);
        E1000_WRITE_FLUSH(hw);
        /* Wait for EEPROM reload */
        mdelay(2);
    } else {
        /* Wait for EEPROM reload (it happens automatically) */
        mdelay(4);
        /* Dissable HW ARPs on ASF enabled adapters */
        manc = E1000_READ_REG(hw, MANC);
        manc &= ~(E1000_MANC_ARP_EN);
        E1000_WRITE_REG(hw, MANC, manc);
    }

    /* Clear interrupt mask to stop board from generating interrupts */
    DEBUGOUT("Masking off all interrupts\n");
    E1000_WRITE_REG(hw, IMC, 0xffffffff);

    /* Clear any pending interrupt events. */
    icr = E1000_READ_REG(hw, ICR);
}


/******************************************************************************
 * Prepares EEPROM for access
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Lowers EEPROM clock. Clears input pin. Sets the chip select pin. This
 * function should be called before issuing a command to the EEPROM.
 *****************************************************************************/
static void
e1000_setup_eeprom(struct e1000_hw *hw) {
    uint16_t eecd;

    eecd = E1000_READ_REG(hw, EECD);

    /* Clear SK and DI */
    eecd &= ~(E1000_EECD_SK | E1000_EECD_DI);
    E1000_WRITE_REG(hw, EECD, eecd);

    /* Set CS */
    eecd |= E1000_EECD_CS;
    E1000_WRITE_REG(hw, EECD, eecd);
}

/******************************************************************************
 * Shift data bits out to the EEPROM.
 *
 * hw - Struct containing variables accessed by shared code
 * data - data to send to the EEPROM
 * count - number of bits to shift out
 *****************************************************************************/
static void
e1000_shift_out_ee_bits(struct e1000_hw *hw, uint16_t data, uint16_t count) {
    uint32_t eecd;
    uint32_t mask;

    /* We need to shift "count" bits out to the EEPROM. So, value in the
     * "data" parameter will be shifted out to the EEPROM one bit at a time.
     * In order to do this, "data" must be broken down into bits.
     */
    mask = 0x01 << (count - 1);
    eecd = E1000_READ_REG(hw, EECD);
    eecd &= ~(E1000_EECD_DO | E1000_EECD_DI);
    do {
        /* A "1" is shifted out to the EEPROM by setting bit "DI" to a "1",
         * and then raising and then lowering the clock (the SK bit controls
         * the clock input to the EEPROM).  A "0" is shifted out to the EEPROM
         * by setting "DI" to "0" and then raising and then lowering the clock.
         */
        eecd &= ~E1000_EECD_DI;

        if (data & mask)
            eecd |= E1000_EECD_DI;

        E1000_WRITE_REG(hw, EECD, eecd);
        E1000_WRITE_FLUSH(hw);

        udelay(50);

        e1000_raise_ee_clk(hw, &eecd);
        e1000_lower_ee_clk(hw, &eecd);

        mask = mask >> 1;

    } while (mask);

    /* We leave the "DI" bit set to "0" when we leave this routine. */
    eecd &= ~E1000_EECD_DI;
    E1000_WRITE_REG(hw, EECD, eecd);
}

/******************************************************************************
 * Returns EEPROM to a "standby" state
 *
 * hw - Struct containing variables accessed by shared code
 *****************************************************************************/
static void
e1000_standby_eeprom(struct e1000_hw *hw)
{
    uint32_t eecd;

    eecd = E1000_READ_REG(hw, EECD);

    /* Deselct EEPROM */
    eecd &= ~(E1000_EECD_CS | E1000_EECD_SK);
    E1000_WRITE_REG(hw, EECD, eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);

    /* Clock high */
    eecd |= E1000_EECD_SK;
    E1000_WRITE_REG(hw, EECD, eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);

    /* Select EEPROM */
    eecd |= E1000_EECD_CS;
    E1000_WRITE_REG(hw, EECD, eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);

    /* Clock low */
    eecd &= ~E1000_EECD_SK;
    E1000_WRITE_REG(hw, EECD, eecd);
    E1000_WRITE_FLUSH(hw);
    udelay(50);
}

/******************************************************************************
 * Shift data bits in from the EEPROM
 *
 * hw - Struct containing variables accessed by shared code
 *****************************************************************************/
static uint16_t
e1000_shift_in_ee_bits(struct e1000_hw *hw)
{
    uint32_t eecd;
    uint32_t i;
    uint16_t data;

    /* In order to read a register from the EEPROM, we need to shift 16 bits
     * in from the EEPROM. Bits are "shifted in" by raising the clock input to
     * the EEPROM (setting the SK bit), and then reading the value of the "DO"
     * bit.  During this "shifting in" process the "DI" bit should always be
     * clear..
     */

    eecd = E1000_READ_REG(hw, EECD);

    eecd &= ~(E1000_EECD_DO | E1000_EECD_DI);
    data = 0;

    for (i = 0; i < 16; i++) {
        data = data << 1;
        e1000_raise_ee_clk(hw, &eecd);

        eecd = E1000_READ_REG(hw, EECD);

        eecd &= ~(E1000_EECD_DI);
        if (eecd & E1000_EECD_DO)
            data |= 1;

        e1000_lower_ee_clk(hw, &eecd);
    }

    return data;
}

/******************************************************************************
* Sets the collision distance in the Transmit Control register
*
* hw - Struct containing variables accessed by shared code
*
* Link should have been established previously. Reads the speed and duplex
* information from the Device Status register.
******************************************************************************/
static void
e1000_config_collision_dist(struct e1000_hw *hw) {
    uint32_t tctl;

    tctl = E1000_READ_REG(hw, TCTL);

    tctl &= ~E1000_TCTL_COLD;
    tctl |= E1000_COLLISION_DISTANCE << E1000_COLD_SHIFT;

    E1000_WRITE_REG(hw, TCTL, tctl);
    E1000_WRITE_FLUSH(hw);
}


/******************************************************************************
 * Reads a 16 bit word from the EEPROM.
 *
 * hw - Struct containing variables accessed by shared code
 * offset - offset of  word in the EEPROM to read
 * data - word read from the EEPROM
 *****************************************************************************/
static int e1000_read_eeprom(struct e1000_hw *hw, uint16_t offset, uint16_t * data) {
    uint16_t eecd;
    uint32_t i = 0;
    int large_eeprom = FALSE;

    /* Request EEPROM Access */
    eecd = E1000_READ_REG(hw, EECD);
//    cprintf("eecd = %p\n", eecd);
    if (eecd & E1000_EECD_SIZE)
        large_eeprom = TRUE;
    eecd |= E1000_EECD_REQ;
    E1000_WRITE_REG(hw, EECD, eecd);
    eecd = E1000_READ_REG(hw, EECD);
    while ((!(eecd & E1000_EECD_GNT)) && (i < 100)) {
        i++;
        udelay(10);
        eecd = E1000_READ_REG(hw, EECD);
    }
    if (!(eecd & E1000_EECD_GNT)) {
        eecd &= ~E1000_EECD_REQ;
        E1000_WRITE_REG(hw, EECD, eecd);
        DEBUGOUT("Could not acquire EEPROM grant\n");
        return -E1000_ERR_EEPROM;
    }

    /*  Prepare the EEPROM for reading  */
    e1000_setup_eeprom(hw);

    /*  Send the READ command (opcode + addr)  */
    e1000_shift_out_ee_bits(hw, EEPROM_READ_OPCODE, 3);
    e1000_shift_out_ee_bits(hw, offset, (large_eeprom) ? 8 : 6);

    /* Read the data */
    *data = e1000_shift_in_ee_bits(hw);

    /* End this read operation */
    e1000_standby_eeprom(hw);

    /* Stop requesting EEPROM access */
    eecd = E1000_READ_REG(hw, EECD);
    eecd &= ~E1000_EECD_REQ;
    E1000_WRITE_REG(hw, EECD, eecd);

    return 0;
}

/**
 * e1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/

static void
e1000_configure_tx(struct e1000_hw *hw) {
    unsigned long ptr;
    unsigned long tctl;
    unsigned long tipg;

    ptr = (uint32_t) tx_pool;
    if (ptr & 0xf)
        ptr = (ptr + 0x10) & (~0xf);

    tx_base = (typeof(tx_base)) ptr;

    E1000_WRITE_REG(hw, TDBAL, (uint32_t) tx_base);
    E1000_WRITE_REG(hw, TDBAH, 0);

    E1000_WRITE_REG(hw, TDLEN, 128);

    /* Setup the HW Tx Head and Tail descriptor pointers */
    E1000_WRITE_REG(hw, TDH, 0);
    E1000_WRITE_REG(hw, TDT, 0);
    tx_tail = 0;

    /* Set the default values for the Tx Inter Packet Gap timer */
    tipg = DEFAULT_82543_TIPG_IPGT_COPPER;
    tipg |= DEFAULT_82543_TIPG_IPGR1 << E1000_TIPG_IPGR1_SHIFT;
    tipg |= DEFAULT_82543_TIPG_IPGR2 << E1000_TIPG_IPGR2_SHIFT;
    E1000_WRITE_REG(hw, TIPG, tipg);
    /* Program the Transmit Control Register */
    tctl = E1000_READ_REG(hw, TCTL);
    tctl &= ~E1000_TCTL_CT;
    tctl |= E1000_TCTL_EN | E1000_TCTL_PSP |
        (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT);
    E1000_WRITE_REG(hw, TCTL, tctl);

    e1000_config_collision_dist(hw);
}

void fill_rx(struct e1000_hw *hw) {
    struct e1000_rx_desc *rd;

    rx_last = rx_tail;
    rd = rx_base + rx_tail;
    rx_tail = (rx_tail + 1) % 8;
    memset(rd, 0, 16);
    rd->buffer_addr = ((uint32_t) & packet);    // TODO: cpu_to_64 before 
    E1000_WRITE_REG(hw, RDT, rx_tail);
}

/**
 * e1000_setup_rctl - configure the receive control register
 * @adapter: Board private structure
 **/
static void
e1000_setup_rctl(struct e1000_hw *hw)
{
    uint32_t rctl;

    rctl = E1000_READ_REG(hw, RCTL);

    rctl &= ~(3 << E1000_RCTL_MO_SHIFT);

    rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF; /* |
                                                   (hw.mc_filter_type << E1000_RCTL_MO_SHIFT); */

    if (hw->tbi_compatibility_on == 1)
        rctl |= E1000_RCTL_SBP;
    else
        rctl &= ~E1000_RCTL_SBP;

    rctl &= ~(E1000_RCTL_SZ_4096);
#if 0
    switch (adapter->rx_buffer_len) {
    case E1000_RXBUFFER_2048:
    default:
#endif
        rctl |= E1000_RCTL_SZ_2048;
        rctl &= ~(E1000_RCTL_BSEX | E1000_RCTL_LPE);
#if 0
        break;
    case E1000_RXBUFFER_4096:
        rctl |= E1000_RCTL_SZ_4096 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
        break;
    case E1000_RXBUFFER_8192:
        rctl |= E1000_RCTL_SZ_8192 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
        break;
    case E1000_RXBUFFER_16384:
        rctl |= E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX | E1000_RCTL_LPE;
        break;
    }
#endif
    E1000_WRITE_REG(hw, RCTL, rctl);
}

/**
 * e1000_configure_rx - Configure 8254x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/
static void
e1000_configure_rx(struct e1000_hw *hw)
{
    unsigned long ptr;
    unsigned long rctl;
#if 0
    unsigned long rxcsum;
#endif
    rx_tail = 0;
    /* make sure receives are disabled while setting up the descriptors */
    rctl = E1000_READ_REG(hw, RCTL);
    E1000_WRITE_REG(hw, RCTL, rctl & ~E1000_RCTL_EN);
#if 0
    /* set the Receive Delay Timer Register */

    E1000_WRITE_REG(hw, RDTR, adapter->rx_int_delay);
#endif
    if (hw->mac_type >= e1000_82540) {
#if 0
        E1000_WRITE_REG(hw, RADV, adapter->rx_abs_int_delay);
#endif
        /* Set the interrupt throttling rate.  Value is calculated
         * as DEFAULT_ITR = 1/(MAX_INTS_PER_SEC * 256ns) */
#define MAX_INTS_PER_SEC        8000
#define DEFAULT_ITR             1000000000/(MAX_INTS_PER_SEC * 256)
        E1000_WRITE_REG(hw, ITR, DEFAULT_ITR);
    }

    /* Setup the Base and Length of the Rx Descriptor Ring */
    ptr = (uint32_t) rx_pool;
    if (ptr & 0xf)
        ptr = (ptr + 0x10) & (~0xf);
    rx_base = (typeof(rx_base)) ptr;
    E1000_WRITE_REG(hw, RDBAL, (uint32_t) rx_base);
    E1000_WRITE_REG(hw, RDBAH, 0);

    E1000_WRITE_REG(hw, RDLEN, 128);

    /* Setup the HW Rx Head and Tail Descriptor Pointers */
    E1000_WRITE_REG(hw, RDH, 0);
    E1000_WRITE_REG(hw, RDT, 0);

    /* Enable Receives */
    E1000_WRITE_REG(hw, RCTL, rctl);
    fill_rx(hw);
}

/******************************************************************************
 * Initializes receive address filters.
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Places the MAC address in receive address register 0 and clears the rest
 * of the receive addresss registers. Clears the multicast table. Assumes
 * the receiver is in reset when the routine is called.
 *****************************************************************************/
static void
e1000_init_rx_addrs(struct e1000_hw *hw) {
    uint32_t i;
    uint32_t addr_low;
    uint32_t addr_high;

    DEBUGFUNC();

    /* Setup the receive address. */
    DEBUGOUT("Programming MAC Address into RAR[0]\n");
    addr_low = (hw->mac[0] |
            (hw->mac[1] << 8) |
            (hw->mac[2] << 16) | (hw->mac[3] << 24));

    addr_high = (hw->mac[4] | (hw->mac[5] << 8) | E1000_RAH_AV);

    E1000_WRITE_REG_ARRAY(hw, RA, 0, addr_low);
    E1000_WRITE_REG_ARRAY(hw, RA, 1, addr_high);

    /* Zero out the other 15 receive addresses. */
    DEBUGOUT("Clearing RAR[1-15]\n");
    for (i = 1; i < E1000_RAR_ENTRIES; i++) {
        E1000_WRITE_REG_ARRAY(hw, RA, (i << 1), 0);
        E1000_WRITE_REG_ARRAY(hw, RA, ((i << 1) + 1), 0);
    }
}

/******************************************************************************
 * Clears the VLAN filer table
 *
 * hw - Struct containing variables accessed by shared code
 *****************************************************************************/
static void e1000_clear_vfta(struct e1000_hw *hw) {
    uint32_t offset;

    for (offset = 0; offset < E1000_VLAN_FILTER_TBL_SIZE; offset++)
        E1000_WRITE_REG_ARRAY(hw, VFTA, offset, 0);
}

/******************************************************************************
 * Configures flow control and link settings.
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Determines which flow control settings to use. Calls the apropriate media-
 * specific link configuration function. Configures the flow control settings.
 * Assuming the adapter has a valid link partner, a valid link should be
 * established. Assumes the hardware has previously been reset and the
 * transmitter and receiver are not enabled.
 *****************************************************************************/
static int
e1000_setup_link(struct e1000_hw *hw) {
    int32_t ret_val;
    uint16_t eeprom_data;

    DEBUGFUNC();

    /* Read and store word 0x0F of the EEPROM. This word contains bits
     * that determine the hardware's default PAUSE (flow control) mode,
     * a bit that determines whether the HW defaults to enabling or
     * disabling auto-negotiation, and the direction of the
     * SW defined pins. If there is no SW over-ride of the flow
     * control setting, then the variable hw->fc will
     * be initialized based on a value in the EEPROM.
     */
    if (e1000_read_eeprom(hw, EEPROM_INIT_CONTROL2_REG, &eeprom_data) < 0) {
        DEBUGOUT("EEPROM Read Error\n");
        return -E1000_ERR_EEPROM;
    }

    if (hw->fc == e1000_fc_default) {
        if ((eeprom_data & EEPROM_WORD0F_PAUSE_MASK) == 0)
            hw->fc = e1000_fc_none;
        else if ((eeprom_data & EEPROM_WORD0F_PAUSE_MASK) ==
             EEPROM_WORD0F_ASM_DIR)
            hw->fc = e1000_fc_tx_pause;
        else
            hw->fc = e1000_fc_full;
    }

    /* We want to save off the original Flow Control configuration just
     * in case we get disconnected and then reconnected into a different
     * hub or switch with different Flow Control capabilities.
     */

    if ((hw->mac_type < e1000_82543) && (hw->report_tx_early == 1))
        hw->fc &= (~e1000_fc_rx_pause);

    hw->original_fc = hw->fc;

    DEBUGOUT("After fix-ups FlowControl is now = %x\n", hw->fc);

    /* Call the necessary subroutine to configure the link. */
    ret_val = (hw->media_type == e1000_media_type_fiber) ?
        e1000_setup_fiber_link(hw) : e1000_setup_copper_link(hw);
    if (ret_val < 0) {
        return ret_val;
    }

    /* Initialize the flow control address, type, and PAUSE timer
     * registers to their default values.  This is done even if flow
     * control is disabled, because it does not hurt anything to
     * initialize these registers.
     */
    DEBUGOUT
        ("Initializing the Flow Control address, type and timer regs\n");

    E1000_WRITE_REG(hw, FCAL, FLOW_CONTROL_ADDRESS_LOW);
    E1000_WRITE_REG(hw, FCAH, FLOW_CONTROL_ADDRESS_HIGH);
    E1000_WRITE_REG(hw, FCT, FLOW_CONTROL_TYPE);
    E1000_WRITE_REG(hw, FCTTV, hw->fc_pause_time);

    /* Set the flow control receive threshold registers.  Normally,
     * these registers will be set to a default threshold that may be
     * adjusted later by the driver's runtime code.  However, if the
     * ability to transmit pause frames in not enabled, then these
     * registers will be set to 0.
     */
    if (!(hw->fc & e1000_fc_tx_pause)) {
        E1000_WRITE_REG(hw, FCRTL, 0);
        E1000_WRITE_REG(hw, FCRTH, 0);
    } else {
        /* We need to set up the Receive Threshold high and low water marks
         * as well as (optionally) enabling the transmission of XON frames.
         */
        if (hw->fc_send_xon) {
            E1000_WRITE_REG(hw, FCRTL,
                    (hw->fc_low_water | E1000_FCRTL_XONE));
            E1000_WRITE_REG(hw, FCRTH, hw->fc_high_water);
        } else {
            E1000_WRITE_REG(hw, FCRTL, hw->fc_low_water);
            E1000_WRITE_REG(hw, FCRTH, hw->fc_high_water);
        }
    }
    return ret_val;
}

/******************************************************************************
 * Performs basic configuration of the adapter.
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Assumes that the controller has previously been reset and is in a
 * post-reset uninitialized state. Initializes the receive address registers,
 * multicast table, and VLAN filter table. Calls routines to setup link
 * configuration and flow control settings. Clears all on-chip counters. Leaves
 * the transmit and receive units disabled and uninitialized.
 *****************************************************************************/
static int
e1000_init_hw(struct e1000_hw * hw) {
    uint32_t ctrl, status;
    uint32_t i;
    int32_t ret_val;
    e1000_bus_type bus_type = e1000_bus_type_unknown;

    DEBUGFUNC();

    /* Set the Media Type and exit with error if it is not valid. */
    hw->tbi_compatibility_en = FALSE;

    status = E1000_READ_REG(hw, STATUS);
    if (status & E1000_STATUS_TBIMODE) {
        hw->media_type = e1000_media_type_fiber;
        /* tbi_compatibility not valid on fiber */
        hw->tbi_compatibility_en = FALSE;
    } else {
        hw->media_type = e1000_media_type_copper;
    }

    /* Disabling VLAN filtering. */
    DEBUGOUT("Initializing the IEEE VLAN\n");
    E1000_WRITE_REG(hw, VET, 0);

    e1000_clear_vfta(hw);

    /* Setup the receive address. This involves initializing all of the Receive
     * Address Registers (RARs 0 - 15).
     */
    e1000_init_rx_addrs(hw);

    /* Zero out the Multicast HASH table */
    DEBUGOUT("Zeroing the MTA\n");
    for (i = 0; i < E1000_MC_TBL_SIZE; i++)
        E1000_WRITE_REG_ARRAY(hw, MTA, i, 0);

    status = E1000_READ_REG(hw, STATUS);
    bus_type = (status & E1000_STATUS_PCIX_MODE) ?
        e1000_bus_type_pcix : e1000_bus_type_pci;
    /* Workaround for PCI-X problem when BIOS sets MMRBC incorrectly. */
    if (bus_type == e1000_bus_type_pcix) {
        panic("PCIX in "__FILE__);
    }

    /* Call a subroutine to configure the link and setup flow control. */
    ret_val = e1000_setup_link(hw);

    /* Set the transmit descriptor write-back policy */
    ctrl = E1000_READ_REG(hw, TXDCTL);
    ctrl =
        (ctrl & ~E1000_TXDCTL_WTHRESH) |
        E1000_TXDCTL_FULL_TX_DESC_WB;
    E1000_WRITE_REG(hw, TXDCTL, ctrl);

    return ret_val;
}

/******************************************************************************
 * Sets up link for a fiber based adapter
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Manipulates Physical Coding Sublayer functions in order to configure
 * link. Assumes the hardware has been previously reset and the transmitter
 * and receiver are not enabled.
 *****************************************************************************/
static int
e1000_setup_fiber_link(struct e1000_hw *hw) {
    uint32_t ctrl;
    uint32_t status;
    uint32_t txcw = 0;
    uint32_t i;
    uint32_t signal;

    DEBUGFUNC();
    /* On adapters with a MAC newer that 82544, SW Defineable pin 1 will be
     * set when the optics detect a signal. On older adapters, it will be
     * cleared when there is a signal
     */
    ctrl = E1000_READ_REG(hw, CTRL);
    if ((hw->mac_type > e1000_82544) && !(ctrl & E1000_CTRL_ILOS))
        signal = E1000_CTRL_SWDPIN1;
    else
        signal = 0;

    /* Take the link out of reset */
    ctrl &= ~(E1000_CTRL_LRST);

    e1000_config_collision_dist(hw);

    /* Check for a software override of the flow control settings, and setup
     * the device accordingly.  If auto-negotiation is enabled, then software
     * will have to set the "PAUSE" bits to the correct value in the Tranmsit
     * Config Word Register (TXCW) and re-start auto-negotiation.  However, if
     * auto-negotiation is disabled, then software will have to manually
     * configure the two flow control enable bits in the CTRL register.
     *
     * The possible values of the "fc" parameter are:
     *      0:  Flow control is completely disabled
     *      1:  Rx flow control is enabled (we can receive pause frames, but
     *          not send pause frames).
     *      2:  Tx flow control is enabled (we can send pause frames but we do
     *          not support receiving pause frames).
     *      3:  Both Rx and TX flow control (symmetric) are enabled.
     */
    switch (hw->fc) {
    case e1000_fc_none:
        /* Flow control is completely disabled by a software over-ride. */
        txcw = (E1000_TXCW_ANE | E1000_TXCW_FD);
        break;
    case e1000_fc_rx_pause:
        /* RX Flow control is enabled and TX Flow control is disabled by a
         * software over-ride. Since there really isn't a way to advertise
         * that we are capable of RX Pause ONLY, we will advertise that we
         * support both symmetric and asymmetric RX PAUSE. Later, we will
         *  disable the adapter's ability to send PAUSE frames.
         */
        txcw = (E1000_TXCW_ANE | E1000_TXCW_FD | E1000_TXCW_PAUSE_MASK);
        break;
    case e1000_fc_tx_pause:
        /* TX Flow control is enabled, and RX Flow control is disabled, by a
         * software over-ride.
         */
        txcw = (E1000_TXCW_ANE | E1000_TXCW_FD | E1000_TXCW_ASM_DIR);
        break;
    case e1000_fc_full:
        /* Flow control (both RX and TX) is enabled by a software over-ride. */
        txcw = (E1000_TXCW_ANE | E1000_TXCW_FD | E1000_TXCW_PAUSE_MASK);
        break;
    default:
        DEBUGOUT("Flow control param set incorrectly\n");
        return -E1000_ERR_CONFIG;
        break;
    }

    /* Since auto-negotiation is enabled, take the link out of reset (the link
     * will be in reset, because we previously reset the chip). This will
     * restart auto-negotiation.  If auto-neogtiation is successful then the
     * link-up status bit will be set and the flow control enable bits (RFCE
     * and TFCE) will be set according to their negotiated value.
     */
    DEBUGOUT("Auto-negotiation enabled (%#x)\n", txcw);

    E1000_WRITE_REG(hw, TXCW, txcw);
    E1000_WRITE_REG(hw, CTRL, ctrl);
    E1000_WRITE_FLUSH(hw);

    hw->txcw = txcw;
    mdelay(1);

    /* If we have a signal (the cable is plugged in) then poll for a "Link-Up"
     * indication in the Device Status Register.  Time-out if a link isn't
     * seen in 500 milliseconds seconds (Auto-negotiation should complete in
     * less than 500 milliseconds even if the other end is doing it in SW).
     */
    if ((E1000_READ_REG(hw, CTRL) & E1000_CTRL_SWDPIN1) == signal) {
        DEBUGOUT("Looking for Link\n");
        for (i = 0; i < (LINK_UP_TIMEOUT / 10); i++) {
            mdelay(10);
            status = E1000_READ_REG(hw, STATUS);
            if (status & E1000_STATUS_LU)
                break;
        }
        if (i == (LINK_UP_TIMEOUT / 10)) {
            DEBUGOUT("Never got a valid link from auto-neg!!!\n");
        } else {
            hw->autoneg_failed = 0;
            DEBUGOUT("Valid Link Found\n");
        }
    } else {
        DEBUGOUT("No Signal Detected\n");
        return -E1000_ERR_NOLINK;
    }
    return 0;
}

/*****************************************************************************
* Reads the value from a PHY register
*
* hw - Struct containing variables accessed by shared code
* reg_addr - address of the PHY register to read
******************************************************************************/
static int
e1000_read_phy_reg(struct e1000_hw *hw, uint32_t reg_addr, uint16_t * phy_data) {
    uint32_t i;
    uint32_t mdic = 0;
    const uint32_t phy_addr = 1;

    if (reg_addr > MAX_PHY_REG_ADDRESS) {
        DEBUGOUT("PHY Address %d is out of range\n", reg_addr);
        return -E1000_ERR_PARAM;
    }

    /* Set up Op-code, Phy Address, and register address in the MDI
     * Control register.  The MAC will take care of interfacing with the
     * PHY to retrieve the desired data.
     */
    mdic = ((reg_addr << E1000_MDIC_REG_SHIFT) |
        (phy_addr << E1000_MDIC_PHY_SHIFT) |
        (E1000_MDIC_OP_READ));

    E1000_WRITE_REG(hw, MDIC, mdic);

    /* Poll the ready bit to see if the MDI read completed */
    for (i = 0; i < 64; i++) {
        udelay(10);
        mdic = E1000_READ_REG(hw, MDIC);
        if (mdic & E1000_MDIC_READY)
            break;
    }
    if (!(mdic & E1000_MDIC_READY)) {
        DEBUGOUT("MDI Read did not complete\n");
        return -E1000_ERR_PHY;
    }
    if (mdic & E1000_MDIC_ERROR) {
        DEBUGOUT("MDI Error\n");
        return -E1000_ERR_PHY;
    }
    *phy_data = (uint16_t) mdic;
    return 0;
}

/******************************************************************************
* Writes a value to a PHY register
*
* hw - Struct containing variables accessed by shared code
* reg_addr - address of the PHY register to write
* data - data to write to the PHY
******************************************************************************/
static int
e1000_write_phy_reg(struct e1000_hw *hw, uint32_t reg_addr, uint16_t phy_data) {
    uint32_t i;
    uint32_t mdic = 0;
    const uint32_t phy_addr = 1;

    if (reg_addr > MAX_PHY_REG_ADDRESS) {
        DEBUGOUT("PHY Address %d is out of range\n", reg_addr);
        return -E1000_ERR_PARAM;
    }

    /* Set up Op-code, Phy Address, register address, and data intended
     * for the PHY register in the MDI Control register.  The MAC will take
     * care of interfacing with the PHY to send the desired data.
     */
    mdic = (((uint32_t) phy_data) |
        (reg_addr << E1000_MDIC_REG_SHIFT) |
        (phy_addr << E1000_MDIC_PHY_SHIFT) |
        (E1000_MDIC_OP_WRITE));

    E1000_WRITE_REG(hw, MDIC, mdic);

    /* Poll the ready bit to see if the MDI read completed */
    for (i = 0; i < 64; i++) {
        udelay(10);
        mdic = E1000_READ_REG(hw, MDIC);
        if (mdic & E1000_MDIC_READY)
            break;
    }
    if (!(mdic & E1000_MDIC_READY)) {
        DEBUGOUT("MDI Write did not complete\n");
        return -E1000_ERR_PHY;
    }
    return 0;
}

/******************************************************************************
* Resets the PHY
*
* hw - Struct containing variables accessed by shared code
*
* Sets bit 15 of the MII Control regiser
******************************************************************************/
static int e1000_phy_reset(struct e1000_hw *hw) {
    uint16_t phy_data;

    DEBUGFUNC();

    if (e1000_read_phy_reg(hw, PHY_CTRL, &phy_data) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    phy_data |= MII_CR_RESET;
    if (e1000_write_phy_reg(hw, PHY_CTRL, phy_data) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }
    udelay(1);
    return 0;
}

/******************************************************************************
* Probes the expected PHY address for known PHY IDs
*
* hw - Struct containing variables accessed by shared code
******************************************************************************/
static int
e1000_detect_gig_phy(struct e1000_hw *hw) {
    uint16_t phy_id_high, phy_id_low;
    int match = FALSE;

    DEBUGFUNC();

    /* Read the PHY ID Registers to identify which PHY is onboard. */
    if (e1000_read_phy_reg(hw, PHY_ID1, &phy_id_high) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    hw->phy_id = (uint32_t) (phy_id_high << 16);
    udelay(2);
    if (e1000_read_phy_reg(hw, PHY_ID2, &phy_id_low) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    hw->phy_id |= (uint32_t) (phy_id_low & PHY_REVISION_MASK);

    if (hw->phy_id == M88E1011_I_PHY_ID)
        match = TRUE;

    if (match) {
        DEBUGOUT("PHY ID 0x%X detected\n", hw->phy_id);
        return 0;
    }
    DEBUGOUT("Invalid PHY ID 0x%X\n", hw->phy_id);
    return -E1000_ERR_PHY;
}

/******************************************************************************
* Configures PHY autoneg and flow control advertisement settings
*
* hw - Struct containing variables accessed by shared code
******************************************************************************/
static int
e1000_phy_setup_autoneg(struct e1000_hw *hw) {
    uint16_t mii_autoneg_adv_reg;
    uint16_t mii_1000t_ctrl_reg;

    DEBUGFUNC();

    /* Read the MII Auto-Neg Advertisement Register (Address 4). */
    if (e1000_read_phy_reg(hw, PHY_AUTONEG_ADV, &mii_autoneg_adv_reg) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }

    /* Read the MII 1000Base-T Control Register (Address 9). */
    if (e1000_read_phy_reg(hw, PHY_1000T_CTRL, &mii_1000t_ctrl_reg) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }

    /* Need to parse both autoneg_advertised and fc and set up
     * the appropriate PHY registers.  First we will parse for
     * autoneg_advertised software override.  Since we can advertise
     * a plethora of combinations, we need to check each bit
     * individually.
     */

    /* First we clear all the 10/100 mb speed bits in the Auto-Neg
     * Advertisement Register (Address 4) and the 1000 mb speed bits in
     * the  1000Base-T Control Register (Address 9).
     */
    mii_autoneg_adv_reg &= ~REG4_SPEED_MASK;
    mii_1000t_ctrl_reg &= ~REG9_SPEED_MASK;

    DEBUGOUT("autoneg_advertised %x\n", hw->autoneg_advertised);

    /* Do we want to advertise 10 Mb Half Duplex? */
    if (hw->autoneg_advertised & ADVERTISE_10_HALF) {
        DEBUGOUT("Advertise 10mb Half duplex\n");
        mii_autoneg_adv_reg |= NWAY_AR_10T_HD_CAPS;
    }

    /* Do we want to advertise 10 Mb Full Duplex? */
    if (hw->autoneg_advertised & ADVERTISE_10_FULL) {
        DEBUGOUT("Advertise 10mb Full duplex\n");
        mii_autoneg_adv_reg |= NWAY_AR_10T_FD_CAPS;
    }

    /* Do we want to advertise 100 Mb Half Duplex? */
    if (hw->autoneg_advertised & ADVERTISE_100_HALF) {
        DEBUGOUT("Advertise 100mb Half duplex\n");
        mii_autoneg_adv_reg |= NWAY_AR_100TX_HD_CAPS;
    }

    /* Do we want to advertise 100 Mb Full Duplex? */
    if (hw->autoneg_advertised & ADVERTISE_100_FULL) {
        DEBUGOUT("Advertise 100mb Full duplex\n");
        mii_autoneg_adv_reg |= NWAY_AR_100TX_FD_CAPS;
    }

    /* We do not allow the Phy to advertise 1000 Mb Half Duplex */
    if (hw->autoneg_advertised & ADVERTISE_1000_HALF) {
        DEBUGOUT
            ("Advertise 1000mb Half duplex requested, request denied!\n");
    }

    /* Do we want to advertise 1000 Mb Full Duplex? */
    if (hw->autoneg_advertised & ADVERTISE_1000_FULL) {
        DEBUGOUT("Advertise 1000mb Full duplex\n");
        mii_1000t_ctrl_reg |= CR_1000T_FD_CAPS;
    }

    /* Check for a software override of the flow control settings, and
     * setup the PHY advertisement registers accordingly.  If
     * auto-negotiation is enabled, then software will have to set the
     * "PAUSE" bits to the correct value in the Auto-Negotiation
     * Advertisement Register (PHY_AUTONEG_ADV) and re-start auto-negotiation.
     *
     * The possible values of the "fc" parameter are:
     *      0:  Flow control is completely disabled
     *      1:  Rx flow control is enabled (we can receive pause frames
     *          but not send pause frames).
     *      2:  Tx flow control is enabled (we can send pause frames
     *          but we do not support receiving pause frames).
     *      3:  Both Rx and TX flow control (symmetric) are enabled.
     *  other:  No software override.  The flow control configuration
     *          in the EEPROM is used.
     */
    switch (hw->fc) {
    case e1000_fc_none: /* 0 */
        /* Flow control (RX & TX) is completely disabled by a
         * software over-ride.
         */
        mii_autoneg_adv_reg &= ~(NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
        break;
    case e1000_fc_rx_pause: /* 1 */
        /* RX Flow control is enabled, and TX Flow control is
         * disabled, by a software over-ride.
         */
        /* Since there really isn't a way to advertise that we are
         * capable of RX Pause ONLY, we will advertise that we
         * support both symmetric and asymmetric RX PAUSE.  Later
         * (in e1000_config_fc_after_link_up) we will disable the
         *hw's ability to send PAUSE frames.
         */
        mii_autoneg_adv_reg |= (NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
        break;
    case e1000_fc_tx_pause: /* 2 */
        /* TX Flow control is enabled, and RX Flow control is
         * disabled, by a software over-ride.
         */
        mii_autoneg_adv_reg |= NWAY_AR_ASM_DIR;
        mii_autoneg_adv_reg &= ~NWAY_AR_PAUSE;
        break;
    case e1000_fc_full: /* 3 */
        /* Flow control (both RX and TX) is enabled by a software
         * over-ride.
         */
        mii_autoneg_adv_reg |= (NWAY_AR_ASM_DIR | NWAY_AR_PAUSE);
        break;
    default:
        DEBUGOUT("Flow control param set incorrectly\n");
        return -E1000_ERR_CONFIG;
    }

    if (e1000_write_phy_reg(hw, PHY_AUTONEG_ADV, mii_autoneg_adv_reg) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }

    DEBUGOUT("Auto-Neg Advertising %x\n", mii_autoneg_adv_reg);

    if (e1000_write_phy_reg(hw, PHY_1000T_CTRL, mii_1000t_ctrl_reg) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }
    return 0;
}

/******************************************************************************
 * Configures flow control settings after link is established
 *
 * hw - Struct containing variables accessed by shared code
 *
 * Should be called immediately after a valid link has been established.
 * Forces MAC flow control settings if link was forced. When in MII/GMII mode
 * and autonegotiation is enabled, the MAC flow control settings will be set
 * based on the flow control negotiated by the PHY. In TBI mode, the TFCE
 * and RFCE bits will be automaticaly set to the negotiated flow control mode.
 *****************************************************************************/
static int
e1000_config_fc_after_link_up(struct e1000_hw *hw) {
    int32_t ret_val;
    uint16_t mii_status_reg;
    uint16_t mii_nway_adv_reg;
    uint16_t mii_nway_lp_ability_reg;
    uint16_t speed;
    uint16_t duplex;

    DEBUGFUNC();

    /* Check for the case where we have fiber media and auto-neg failed
     * so we had to force link.  In this case, we need to force the
     * configuration of the MAC to match the "fc" parameter.
     */
    if ((hw->media_type == e1000_media_type_fiber) && (hw->autoneg_failed)) {
        ret_val = e1000_force_mac_fc(hw);
        if (ret_val < 0) {
            DEBUGOUT("Error forcing flow control settings\n");
            return ret_val;
        }
    }

    /* Check for the case where we have copper media and auto-neg is
     * enabled.  In this case, we need to check and see if Auto-Neg
     * has completed, and if so, how the PHY and link partner has
     * flow control configured.
     */
    if (hw->media_type == e1000_media_type_copper) {
        /* Read the MII Status Register and check to see if AutoNeg
         * has completed.  We read this twice because this reg has
         * some "sticky" (latched) bits.
         */
        if (e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg) < 0) {
            DEBUGOUT("PHY Read Error \n");
            return -E1000_ERR_PHY;
        }
        if (e1000_read_phy_reg(hw, PHY_STATUS, &mii_status_reg) < 0) {
            DEBUGOUT("PHY Read Error \n");
            return -E1000_ERR_PHY;
        }

        if (mii_status_reg & MII_SR_AUTONEG_COMPLETE) {
            /* The AutoNeg process has completed, so we now need to
             * read both the Auto Negotiation Advertisement Register
             * (Address 4) and the Auto_Negotiation Base Page Ability
             * Register (Address 5) to determine how flow control was
             * negotiated.
             */
            if (e1000_read_phy_reg
                (hw, PHY_AUTONEG_ADV, &mii_nway_adv_reg) < 0) {
                DEBUGOUT("PHY Read Error\n");
                return -E1000_ERR_PHY;
            }
            if (e1000_read_phy_reg
                (hw, PHY_LP_ABILITY,
                 &mii_nway_lp_ability_reg) < 0) {
                DEBUGOUT("PHY Read Error\n");
                return -E1000_ERR_PHY;
            }

            /* Two bits in the Auto Negotiation Advertisement Register
             * (Address 4) and two bits in the Auto Negotiation Base
             * Page Ability Register (Address 5) determine flow control
             * for both the PHY and the link partner.  The following
             * table, taken out of the IEEE 802.3ab/D6.0 dated March 25,
             * 1999, describes these PAUSE resolution bits and how flow
             * control is determined based upon these settings.
             * NOTE:  DC = Don't Care
             *
             *   LOCAL DEVICE  |   LINK PARTNER
             * PAUSE | ASM_DIR | PAUSE | ASM_DIR | NIC Resolution
             *-------|---------|-------|---------|--------------------
             *   0   |    0    |  DC   |   DC    | e1000_fc_none
             *   0   |    1    |   0   |   DC    | e1000_fc_none
             *   0   |    1    |   1   |    0    | e1000_fc_none
             *   0   |    1    |   1   |    1    | e1000_fc_tx_pause
             *   1   |    0    |   0   |   DC    | e1000_fc_none
             *   1   |   DC    |   1   |   DC    | e1000_fc_full
             *   1   |    1    |   0   |    0    | e1000_fc_none
             *   1   |    1    |   0   |    1    | e1000_fc_rx_pause
             *
             */
            /* Are both PAUSE bits set to 1?  If so, this implies
             * Symmetric Flow Control is enabled at both ends.  The
             * ASM_DIR bits are irrelevant per the spec.
             *
             * For Symmetric Flow Control:
             *
             *   LOCAL DEVICE  |   LINK PARTNER
             * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
             *-------|---------|-------|---------|--------------------
             *   1   |   DC    |   1   |   DC    | e1000_fc_full
             *
             */
            if ((mii_nway_adv_reg & NWAY_AR_PAUSE) &&
                (mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE)) {
                /* Now we need to check if the user selected RX ONLY
                 * of pause frames.  In this case, we had to advertise
                 * FULL flow control because we could not advertise RX
                 * ONLY. Hence, we must now check to see if we need to
                 * turn OFF  the TRANSMISSION of PAUSE frames.
                 */
                if (hw->original_fc == e1000_fc_full) {
                    hw->fc = e1000_fc_full;
                    DEBUGOUT("Flow Control = FULL.\r\n");
                } else {
                    hw->fc = e1000_fc_rx_pause;
                    DEBUGOUT
                        ("Flow Control = RX PAUSE frames only.\r\n");
                }
            }
            /* For receiving PAUSE frames ONLY.
             *
             *   LOCAL DEVICE  |   LINK PARTNER
             * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
             *-------|---------|-------|---------|--------------------
             *   0   |    1    |   1   |    1    | e1000_fc_tx_pause
             *
             */
            else if (!(mii_nway_adv_reg & NWAY_AR_PAUSE) &&
                 (mii_nway_adv_reg & NWAY_AR_ASM_DIR) &&
                 (mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE) &&
                 (mii_nway_lp_ability_reg & NWAY_LPAR_ASM_DIR))
            {
                hw->fc = e1000_fc_tx_pause;
                DEBUGOUT
                    ("Flow Control = TX PAUSE frames only.\r\n");
            }
            /* For transmitting PAUSE frames ONLY.
             *
             *   LOCAL DEVICE  |   LINK PARTNER
             * PAUSE | ASM_DIR | PAUSE | ASM_DIR | Result
             *-------|---------|-------|---------|--------------------
             *   1   |    1    |   0   |    1    | e1000_fc_rx_pause
             *
             */
            else if ((mii_nway_adv_reg & NWAY_AR_PAUSE) &&
                 (mii_nway_adv_reg & NWAY_AR_ASM_DIR) &&
                 !(mii_nway_lp_ability_reg & NWAY_LPAR_PAUSE) &&
                 (mii_nway_lp_ability_reg & NWAY_LPAR_ASM_DIR))
            {
                hw->fc = e1000_fc_rx_pause;
                DEBUGOUT
                    ("Flow Control = RX PAUSE frames only.\r\n");
            }
            /* Per the IEEE spec, at this point flow control should be
             * disabled.  However, we want to consider that we could
             * be connected to a legacy switch that doesn't advertise
             * desired flow control, but can be forced on the link
             * partner.  So if we advertised no flow control, that is
             * what we will resolve to.  If we advertised some kind of
             * receive capability (Rx Pause Only or Full Flow Control)
             * and the link partner advertised none, we will configure
             * ourselves to enable Rx Flow Control only.  We can do
             * this safely for two reasons:  If the link partner really
             * didn't want flow control enabled, and we enable Rx, no
             * harm done since we won't be receiving any PAUSE frames
             * anyway.  If the intent on the link partner was to have
             * flow control enabled, then by us enabling RX only, we
             * can at least receive pause frames and process them.
             * This is a good idea because in most cases, since we are
             * predominantly a server NIC, more times than not we will
             * be asked to delay transmission of packets than asking
             * our link partner to pause transmission of frames.
             */
            else if (hw->original_fc == e1000_fc_none ||
                 hw->original_fc == e1000_fc_tx_pause) {
                hw->fc = e1000_fc_none;
                DEBUGOUT("Flow Control = NONE.\r\n");
            } else {
                hw->fc = e1000_fc_rx_pause;
                DEBUGOUT
                    ("Flow Control = RX PAUSE frames only.\r\n");
            }

            /* Now we need to do one last check...  If we auto-
             * negotiated to HALF DUPLEX, flow control should not be
             * enabled per IEEE 802.3 spec.
             */
            e1000_get_speed_and_duplex(hw, &speed, &duplex);

            if (duplex == HALF_DUPLEX)
                hw->fc = e1000_fc_none;

            /* Now we call a subroutine to actually force the MAC
             * controller to use the correct flow control settings.
             */
            ret_val = e1000_force_mac_fc(hw);
            if (ret_val < 0) {
                DEBUGOUT
                    ("Error forcing flow control settings\n");
                return ret_val;
            }
        } else {
            DEBUGOUT
                ("Copper PHY and Auto Neg has not completed.\r\n");
        }
    }
    return 0;
}

/******************************************************************************
* Sets MAC speed and duplex settings to reflect the those in the PHY
*
* hw - Struct containing variables accessed by shared code
* mii_reg - data to write to the MII control register
*
* The contents of the PHY register containing the needed information need to
* be passed in.
******************************************************************************/
static int
e1000_config_mac_to_phy(struct e1000_hw *hw) {
    uint32_t ctrl;
    uint16_t phy_data;

    DEBUGFUNC();

    /* Read the Device Control Register and set the bits to Force Speed
     * and Duplex.
     */
    ctrl = E1000_READ_REG(hw, CTRL);
    ctrl |= (E1000_CTRL_FRCSPD | E1000_CTRL_FRCDPX);
    ctrl &= ~(E1000_CTRL_SPD_SEL | E1000_CTRL_ILOS);

    /* Set up duplex in the Device Control and Transmit Control
     * registers depending on negotiated values.
     */
    if (e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_STATUS, &phy_data) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    if (phy_data & M88E1000_PSSR_DPLX)
        ctrl |= E1000_CTRL_FD;
    else
        ctrl &= ~E1000_CTRL_FD;

    e1000_config_collision_dist(hw);

    /* Set up speed in the Device Control register depending on
     * negotiated values.
     */
    if ((phy_data & M88E1000_PSSR_SPEED) == M88E1000_PSSR_1000MBS)
        ctrl |= E1000_CTRL_SPD_1000;
    else if ((phy_data & M88E1000_PSSR_SPEED) == M88E1000_PSSR_100MBS)
        ctrl |= E1000_CTRL_SPD_100;
    /* Write the configured values back to the Device Control Reg. */
    E1000_WRITE_REG(hw, CTRL, ctrl);
    return 0;
}

/******************************************************************************
* Blocks until autoneg completes or times out (~4.5 seconds)
*
* hw - Struct containing variables accessed by shared code
******************************************************************************/
static int
e1000_wait_autoneg(struct e1000_hw *hw) {
    uint16_t i;
    uint16_t phy_data;

    DEBUGFUNC();
    DEBUGOUT("Waiting for Auto-Neg to complete.\n");

    /* We will wait for autoneg to complete or 4.5 seconds to expire. */
    for (i = PHY_AUTO_NEG_TIME; i > 0; i--) {
        /* Read the MII Status Register and wait for Auto-Neg
         * Complete bit to be set.
         */
        if (e1000_read_phy_reg(hw, PHY_STATUS, &phy_data) < 0) {
            DEBUGOUT("PHY Read Error\n");
            return -E1000_ERR_PHY;
        }
        if (e1000_read_phy_reg(hw, PHY_STATUS, &phy_data) < 0) {
            DEBUGOUT("PHY Read Error\n");
            return -E1000_ERR_PHY;
        }
        if (phy_data & MII_SR_AUTONEG_COMPLETE) {
            DEBUGOUT("Auto-Neg complete.\n");
            return 0;
        }
        mdelay(100);
    }
    DEBUGOUT("Auto-Neg timedout.\n");
    return -E1000_ERR_TIMEOUT;
}

/******************************************************************************
* Detects which PHY is present and the speed and duplex
*
* hw - Struct containing variables accessed by shared code
******************************************************************************/
static int
e1000_setup_copper_link(struct e1000_hw *hw) {
    uint32_t ctrl;
    int32_t ret_val;
    uint16_t i;
    uint16_t phy_data;

    DEBUGFUNC();

    ctrl = E1000_READ_REG(hw, CTRL);

    /* we need to force speed and duplex on the MAC equal to what
     * the PHY speed and duplex configuration is. In addition, we need to
     * perform a hardware reset on the PHY to take it out of reset.
     */
    ctrl |= E1000_CTRL_SLU;
    ctrl &= ~(E1000_CTRL_FRCSPD | E1000_CTRL_FRCDPX);
    E1000_WRITE_REG(hw, CTRL, ctrl);

    /* Make sure we have a valid PHY */
    ret_val = e1000_detect_gig_phy(hw);
    if (ret_val < 0) {
        DEBUGOUT("Error, did not detect valid phy.\n");
        return ret_val;
    }
    DEBUGOUT("Phy ID = %x \n", hw->phy_id);

    /* Enable CRS on TX. This must be set for half-duplex operation. */
    if (e1000_read_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, &phy_data) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    phy_data |= M88E1000_PSCR_ASSERT_CRS_ON_TX;

    phy_data |= M88E1000_PSCR_AUTO_X_MODE;

    phy_data &= ~M88E1000_PSCR_POLARITY_REVERSAL;
    if (e1000_write_phy_reg(hw, M88E1000_PHY_SPEC_CTRL, phy_data) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }

    /* Force TX_CLK in the Extended PHY Specific Control Register
     * to 25MHz clock.
     */
    if (e1000_read_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, &phy_data) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    phy_data |= M88E1000_EPSCR_TX_CLK_25;
    /* Configure Master and Slave downshift values */
    phy_data &= ~(M88E1000_EPSCR_MASTER_DOWNSHIFT_MASK |
              M88E1000_EPSCR_SLAVE_DOWNSHIFT_MASK);
    phy_data |= (M88E1000_EPSCR_MASTER_DOWNSHIFT_1X |
             M88E1000_EPSCR_SLAVE_DOWNSHIFT_1X);
    if (e1000_write_phy_reg(hw, M88E1000_EXT_PHY_SPEC_CTRL, phy_data) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }

    /* SW Reset the PHY so all changes take effect */
    ret_val = e1000_phy_reset(hw);
    if (ret_val < 0) {
        DEBUGOUT("Error Resetting the PHY\n");
        return ret_val;
    }

    /* Options:
     *   autoneg = 1 (default)
     *      PHY will advertise value(s) parsed from
     *      autoneg_advertised and fc
     *   autoneg = 0
     *      PHY will be set to 10H, 10F, 100H, or 100F
     *      depending on value parsed from forced_speed_duplex.
     */

    /* Is autoneg enabled?  This is enabled by default or by software override.
     * If so, call e1000_phy_setup_autoneg routine to parse the
     * autoneg_advertised and fc options. If autoneg is NOT enabled, then the
     * user should have provided a speed/duplex override.  If so, then call
     * e1000_phy_force_speed_duplex to parse and set this up.
     */
    /* Perform some bounds checking on the hw->autoneg_advertised
     * parameter.  If this variable is zero, then set it to the default.
     */
    hw->autoneg_advertised &= AUTONEG_ADVERTISE_SPEED_DEFAULT;

    /* If autoneg_advertised is zero, we assume it was not defaulted
     * by the calling code so we set to advertise full capability.
     */
    if (hw->autoneg_advertised == 0)
        hw->autoneg_advertised = AUTONEG_ADVERTISE_SPEED_DEFAULT;

    DEBUGOUT("Reconfiguring auto-neg advertisement params\n");
    ret_val = e1000_phy_setup_autoneg(hw);
    if (ret_val < 0) {
        DEBUGOUT("Error Setting up Auto-Negotiation\n");
        return ret_val;
    }
    DEBUGOUT("Restarting Auto-Neg\n");

    /* Restart auto-negotiation by setting the Auto Neg Enable bit and
     * the Auto Neg Restart bit in the PHY control register.
     */
    if (e1000_read_phy_reg(hw, PHY_CTRL, &phy_data) < 0) {
        DEBUGOUT("PHY Read Error\n");
        return -E1000_ERR_PHY;
    }
    phy_data |= (MII_CR_AUTO_NEG_EN | MII_CR_RESTART_AUTO_NEG);
    if (e1000_write_phy_reg(hw, PHY_CTRL, phy_data) < 0) {
        DEBUGOUT("PHY Write Error\n");
        return -E1000_ERR_PHY;
    }
#if 0
    /* Does the user want to wait for Auto-Neg to complete here, or
     * check at a later time (for example, callback routine).
     */
    if (hw->wait_autoneg_complete) {
        ret_val = e1000_wait_autoneg(hw);
        if (ret_val < 0) {
            DEBUGOUT
                ("Error while waiting for autoneg to complete\n");
            return ret_val;
        }
    }
#else
    /* If we do not wait for autonegtation to complete I
     * do not see a valid link status.
     */
    ret_val = e1000_wait_autoneg(hw);
    if (ret_val < 0) {
        DEBUGOUT("Error while waiting for autoneg to complete\n");
        return ret_val;
    }
#endif

    /* Check link status. Wait up to 100 microseconds for link to become
     * valid.
     */
    for (i = 0; i < 10; i++) {
        if (e1000_read_phy_reg(hw, PHY_STATUS, &phy_data) < 0) {
            DEBUGOUT("PHY Read Error\n");
            return -E1000_ERR_PHY;
        }
        if (e1000_read_phy_reg(hw, PHY_STATUS, &phy_data) < 0) {
            DEBUGOUT("PHY Read Error\n");
            return -E1000_ERR_PHY;
        }
        if (phy_data & MII_SR_LINK_STATUS) {
            /* We have link, so we need to finish the config process:
             *   1) Set up the MAC to the current PHY speed/duplex
             *      if we are on 82543.  If we
             *      are on newer silicon, we only need to configure
             *      collision distance in the Transmit Control Register.
             *   2) Set up flow control on the MAC to that established with
             *      the link partner.
             */
            if (hw->mac_type >= e1000_82544) {
                e1000_config_collision_dist(hw);
            } else {
                ret_val = e1000_config_mac_to_phy(hw);
                if (ret_val < 0) {
                    DEBUGOUT
                        ("Error configuring MAC to PHY settings\n");
                    return ret_val;
                }
            }
            ret_val = e1000_config_fc_after_link_up(hw);
            if (ret_val < 0) {
                DEBUGOUT("Error Configuring Flow Control\n");
                return ret_val;
            }
            DEBUGOUT("Valid link established!!!\n");
            return 0;
        }
        udelay(10);
    }

    DEBUGOUT("Unable to establish link!!!\n");
    return -E1000_ERR_NOLINK;
}

/*reset function*/
static inline int
e1000_reset(struct e1000_hw *hw) {

    e1000_reset_hw(hw);
    E1000_WRITE_REG(hw, WUC, 0);
    return e1000_init_hw(hw);
}

/**************************************************************************
INIT - set up ethernet interface(s)
***************************************************************************/
static int
e1000_init(struct e1000_hw * hw)
{
    int ret_val = 0;

    ret_val = e1000_reset(hw);
    if (ret_val < 0) {
        if ((ret_val == -E1000_ERR_NOLINK) ||
            (ret_val == -E1000_ERR_TIMEOUT)) {
            E1000_ERR("Valid Link not detected, ret is %d\n", ret_val);
        } else {
            E1000_ERR("Hardware Initialization Failed\n");
        }
        return 0;
    }
    e1000_configure_tx(hw);
    e1000_setup_rctl(hw);
    e1000_configure_rx(hw);
    return 1;
}

int e1000_receive(void *buffer, uint32_t len) {
    if (!dev_count) {
        return -2;  // device not found
    }
    return 0;
}

int ether_e1000_attach(struct pci_func *pcif) {
    cprintf("PCI: %02x:%02x.%d: "
            "Intel 82540EM e1000 Ethernet PCI Controller %04x:%04x\n",
            pcif->bus->busno, pcif->dev, pcif->func,
            PCI_VENDOR(pcif->dev_id), PCI_PRODUCT(pcif->dev_id));
    struct e1000_hw * dev = &devs[dev_count];

    // init to 0 for later check
    dev->iobase = 0;
    
    // enable this device and read BARS into this pcif struct
    pci_func_enable(pcif);

    // interrupt line
    dev->irq = pcif->irq_line;
    dev->mac_type = e1000_82540;
    dev->lan_loc = e1000_lan_a;
    dev->media_type = e1000_media_type_copper;
    dev->fc_high_water = E1000_FC_HIGH_THRESH;
    dev->fc_low_water = E1000_FC_LOW_THRESH;
    dev->fc_pause_time = E1000_FC_PAUSE_TIME;
    dev->fc_send_xon = 1;
    
    int i;
    for (i=0; i<6; i++) {
//      dev->regbase[i] = pcif->reg_base[i];
//      dev->regsize[i] = pcif->reg_size[i];
        if (pcif->reg_type[i] == PCI_MAPREG_TYPE_IO) {
            dev->iobase = pcif->reg_base[i];
            break;
        }
    }

    if (!dev->iobase) {
        panic("no iobase assigned");       
    } else {
        cprintf("[ II ] iobase = %p\n", dev->iobase);
    }
    e1000_validate_eeprom_checksum(dev);
    
    // get MAC address
    uint16_t eeprom_data;
    for (i = 0; i < NODE_ADDRESS_SIZE; i += 2) {
        if (e1000_read_eeprom(dev, i >> 1, &eeprom_data) < 0) {
            DEBUGOUT("EEPROM Read Error\n");
            return -E1000_ERR_EEPROM;
        }
        dev->mac[i] = eeprom_data & 0xff;
        dev->mac[i + 1] = (eeprom_data >> 8) & 0xff;
    }
    cprintf("   MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",
            dev->mac[0],
            dev->mac[1],
            dev->mac[2],
            dev->mac[3],
            dev->mac[4],
            dev->mac[5]);

    // init hardware
    e1000_init(dev);

    struct ip_addr ipaddr;
    IP4_ADDR(&ipaddr, 192, 168, 1, 13);
    struct ip_addr netmask;
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    struct ip_addr gw;
    IP4_ADDR(&gw, 192, 168, 1, 1);

    netif_add(&dev->netif, &ipaddr, &netmask, &gw, 0, ethernetif_init, ethernet_input);
    netif_set_up(&dev->netif);

    struct ethernetif *eif = dev->netif.state;
//    eif->receive = e1000_poll;  // e1000_poll
//    eif->send = e1000_transmit;     // e1000_transmit

//    eif->receive = e1000_receive;
//    eif->send = e1000_send;
    eif->ethaddr = (struct eth_addr *)dev->mac;
    dev->netif.hwaddr_len = 6;
    memcpy(dev->netif.hwaddr, dev->mac, 6);
//    reg_irq_handler(dev->irq, e1000_intr);
//    pic_enable(dev->irq);
//    ioapic_enable(dev->irq, ncpu-1);
//    kernel_thread(e100_rx_thread, dev, 0); //, 0, "[e100 rx thread]");
//    e100_ru_start(dev);

    dev_count++;
    return 1;
}

