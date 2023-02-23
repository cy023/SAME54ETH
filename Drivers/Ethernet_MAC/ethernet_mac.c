/**
 * @file mac.c
 * @author cy023
 * @date 2022.08.20
 * @brief 
 * 
 */

#include <string.h>
#include <stdio.h>
#include "assert.h"
#include "config_mac.h"
#include "ethernet_mac.h"

#define GMAC_CRITICAL_SECTION_ENTER()
#define GMAC_CRITICAL_SECTION_LEAVE()

typedef struct _mac_txbuf_desc {
    uint32_t address;
    union gmac_tx_status {
        uint32_t val;
        struct _gmac_tx_status_bm {
            uint32_t len : 14,                   /* Length of buffer */
                reserved : 1, last_buf : 1,      /* Last buffer (in the current frame) */
                no_crc : 1,                      /* No CRC */
                reserved1 : 3, checksum_err : 3, /* Transmit checksum generation offload error */
                reserved2 : 3, lco : 1,          /* Late collision, transmit error detected */
                exhausted : 1,                   /* Buffer exhausted in mid frame */
                reserved3 : 1, error : 1,        /* Retry limit exceeded, error detected */
                wrap : 1,                        /* Marks last descriptor in TD list */
                used : 1;                        /* User clear, GMAC sets this to 1 once a frame
                                                    has been successfully transmitted */
        } bm;
    } status;
} mac_txbuf_desc_t;

typedef struct _mac_rxbuf_desc {
    union _gmac_rx_addr {
        uint32_t val;
        struct rx_addr_bm {
            uint32_t ownership : 1, /* clear before buffer can be used again */
                wrap : 1,           /* Marks last entry in array */
                addr : 30;          /* Address in number of DW */
        } bm;
    } address;
    union gmac_rx_status {
        uint32_t val;
        struct _gmac_rx_status_bm {
            uint32_t len : 13,                    /* Length of frame including FCS */
                fcs : 1,                          /* Frame has bad FCS */
                sof : 1,                          /* Start of frame */
                eof : 1,                          /* End of frame */
                cfi : 1,                          /* Concatenation Format Indicator */
                vlan_priority : 3,                /* VLAN priority (if VLAN detected) */
                priority_detected : 1,            /* Priority tag detected */
                vlan_detected : 1,                /* VLAN tag detected */
                type_id_match : 2,                /* Type ID match */
                checksumoffload : 1,              /* Checksum offload specific function */
                addrmatch : 2,                    /* Address register match */
                ext_addr_match : 1,               /* External address match found */
                reserved : 1, uni_hash_match : 1, /* Unicast hash match */
                multi_hash_match : 1,             /* Multicast hash match */
                boardcast_detect : 1;             /* Global broadcast address detected */
        } bm;
    } status;
} mac_rxbuf_desc_t;

/* Transmit and Receive buffer descriptor array */
COMPILER_ALIGNED(8) static mac_txbuf_desc_t mac_txbuffer_desc[CONF_GMAC_TX_DESC_NUM];
COMPILER_ALIGNED(8) static mac_rxbuf_desc_t mac_rxbuffer_desc[CONF_GMAC_RX_DESC_NUM];

/* Transmit buffer data array */
COMPILER_ALIGNED(32)
static uint8_t _txbuf[CONF_GMAC_TX_DESC_NUM][CONF_GMAC_TXBUF_SIZE];
COMPILER_ALIGNED(32)
static uint8_t _rxbuf[CONF_GMAC_RX_DESC_NUM][CONF_GMAC_RXBUF_SIZE];
COMPILER_PACK_RESET()

/* Transmit and receive Buffer index */
static volatile uint32_t _txbuf_index;
static volatile uint32_t _last_txbuf_index;
static volatile uint32_t _rxbuf_index;

/* GMAC TX/RX callback function */
mac_callbacks_t cb;

/* Forward Declaration */
static void mac_read_cb(void);
static void mac_write_cb(void);

/*******************************************************************************
 * static function
 ******************************************************************************/
static void mac_buf_init(void)
{
    uint32_t i;

    // TX buffer descriptor
    for (i = 0; i < CONF_GMAC_TX_DESC_NUM; i++) {
        mac_txbuffer_desc[i].address        = (uint32_t)_txbuf[i];
        mac_txbuffer_desc[i].status.val     = 0;
        mac_txbuffer_desc[i].status.bm.used = 1;
    }

    mac_txbuffer_desc[CONF_GMAC_TX_DESC_NUM - 1].status.bm.wrap = 1;
    _txbuf_index                                                = 0;
    _last_txbuf_index                                           = 0;

    // RX buffer descriptor
    for (i = 0; i < CONF_GMAC_RX_DESC_NUM; i++) {
        mac_rxbuffer_desc[i].address.val = (uint32_t)_rxbuf[i];
        mac_rxbuffer_desc[i].status.val  = 0;
    }

    mac_rxbuffer_desc[CONF_GMAC_RX_DESC_NUM - 1].address.bm.wrap = 1;
    _rxbuf_index                                                 = 0;

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_TBQB = (uint32_t)mac_txbuffer_desc;
    GMAC_CRITICAL_SECTION_LEAVE();

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_RBQB = (uint32_t)mac_rxbuffer_desc;
    GMAC_CRITICAL_SECTION_LEAVE();
}

/*******************************************************************************
 * GMAC interrupt handler
 ******************************************************************************/
void GMAC_Handler(void)
{
    volatile uint32_t tsr;
    volatile uint32_t rsr;

    tsr = GMAC_REGS->GMAC_TSR;
    rsr = GMAC_REGS->GMAC_RSR;

    // Must be Clear ISR (Clear on read)
    __attribute__((unused)) uint32_t cor = GMAC_REGS->GMAC_ISR;

    // Frame transmited
    if (tsr & GMAC_TSR_TXCOMP_Msk) {
        GMAC_CRITICAL_SECTION_ENTER();
        GMAC_REGS->GMAC_TSR = tsr;
        GMAC_CRITICAL_SECTION_LEAVE();

        if ((mac_txbuffer_desc[_txbuf_index].status.bm.used) && (cb.transmit != NULL)) {
            cb.transmit(NULL);
        }
    }

    // Frame received
    if (rsr & GMAC_RSR_REC_Msk) {
        if (cb.receive != NULL) {
            cb.receive(NULL);
        }
    }
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_RSR = rsr;
    GMAC_CRITICAL_SECTION_LEAVE();
}

/*******************************************************************************
 * GMAC HAL
 ******************************************************************************/
void mac_init(void)
{
    // GMAC Network Control Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR =  ((CONF_GMAC_NCR_LBL    ? GMAC_NCR_LBL_Msk    : 0) | \
                            (CONF_GMAC_NCR_MPE    ? GMAC_NCR_MPE_Msk    : 0) | \
                            (CONF_GMAC_NCR_WESTAT ? GMAC_NCR_WESTAT_Msk : 0) | \
                            (CONF_GMAC_NCR_BP     ? GMAC_NCR_BP_Msk     : 0) | \
                            (CONF_GMAC_NCR_ENPBPR ? GMAC_NCR_ENPBPR_Msk : 0) | \
                            (CONF_GMAC_NCR_TXPBPF ? GMAC_NCR_TXPBPF_Msk : 0));
    GMAC_CRITICAL_SECTION_LEAVE();

    // GMAC Network Configuration Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCFGR = ((CONF_GMAC_NCFGR_SPD   ? GMAC_NCFGR_SPD_Msk    : 0) | \
                            (CONF_GMAC_NCFGR_FD     ? GMAC_NCFGR_FD_Msk     : 0) | \
                            (CONF_GMAC_NCFGR_DNVLAN ? GMAC_NCFGR_DNVLAN_Msk : 0) | \
                            (CONF_GMAC_NCFGR_JFRAME ? GMAC_NCFGR_JFRAME_Msk : 0) | \
                            (CONF_GMAC_NCFGR_CAF    ? GMAC_NCFGR_CAF_Msk    : 0) | \
                            (CONF_GMAC_NCFGR_NBC    ? GMAC_NCFGR_NBC_Msk    : 0) | \
                            (CONF_GMAC_NCFGR_MTIHEN ? GMAC_NCFGR_MTIHEN_Msk : 0) | \
                            (CONF_GMAC_NCFGR_UNIHEN ? GMAC_NCFGR_UNIHEN_Msk : 0) | \
                            (CONF_GMAC_NCFGR_MAXFS  ? GMAC_NCFGR_MAXFS_Msk  : 0) | \
                            (CONF_GMAC_NCFGR_RTY    ? GMAC_NCFGR_RTY_Msk    : 0) | \
                            (CONF_GMAC_NCFGR_PEN    ? GMAC_NCFGR_PEN_Msk    : 0) | \
                            GMAC_NCFGR_RXBUFO(CONF_GMAC_NCFGR_RXBUFO)            | \
                            (CONF_GMAC_NCFGR_LFERD  ? GMAC_NCFGR_LFERD_Msk  : 0) | \
                            (CONF_GMAC_NCFGR_RFCS   ? GMAC_NCFGR_RFCS_Msk   : 0) | \
                            GMAC_NCFGR_CLK(CONF_GMAC_NCFGR_CLK)                  | \
                            (CONF_GMAC_NCFGR_DCPF   ? GMAC_NCFGR_DCPF_Msk   : 0) | \
                            (CONF_GMAC_NCFGR_RXCOEN ? GMAC_NCFGR_RXCOEN_Msk : 0) | \
                            (CONF_GMAC_NCFGR_EFRHD  ? GMAC_NCFGR_EFRHD_Msk  : 0) | \
                            (CONF_GMAC_NCFGR_IRXFCS ? GMAC_NCFGR_IRXFCS_Msk : 0) | \
                            (CONF_GMAC_NCFGR_IPGSEN ? GMAC_NCFGR_IPGSEN_Msk : 0) | \
                            (CONF_GMAC_NCFGR_RXBP   ? GMAC_NCFGR_RXBP_Msk   : 0) | \
                            (CONF_GMAC_NCFGR_IRXER  ? GMAC_NCFGR_IRXER_Msk  : 0));
    GMAC_CRITICAL_SECTION_LEAVE();

    // GMAC User Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_UR = (CONF_GMAC_UR_MII ? GMAC_UR_MII_Msk : 0);
    GMAC_CRITICAL_SECTION_LEAVE();

    // GMAC DMA Configuration Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_DCFGR = (GMAC_DCFGR_FBLDO(CONF_GMAC_DCFGR_FBLDO)             | \
                            (CONF_GMAC_DCFGR_ESMA   ? GMAC_DCFGR_ESMA_Msk   : 0) | \
                            (CONF_GMAC_DCFGR_ESPA   ? GMAC_DCFGR_ESPA_Msk   : 0) | \
                            GMAC_DCFGR_RXBMS(CONF_GMAC_DCFGR_RXBMS)              | \
                            (CONF_GMAC_DCFGR_TXPBMS ? GMAC_DCFGR_TXPBMS_Msk : 0) | \
                            (CONF_GMAC_DCFGR_TXCOEN ? GMAC_DCFGR_TXCOEN_Msk : 0) | \
                            GMAC_DCFGR_DRBS(CONF_GMAC_DCFGR_DRBS)                | \
                            (CONF_GMAC_DCFGR_DDRP   ? GMAC_DCFGR_DDRP_Msk   : 0));
    GMAC_CRITICAL_SECTION_LEAVE();

    // GMAC Wake on LAN Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_WOL = 0;
    GMAC_CRITICAL_SECTION_LEAVE();

    // IPG Stretch Register
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_IPGS = (GMAC_IPGS_FL((CONF_GMAC_IPGS_FL_MUL << 8) | CONF_GMAC_IPGS_FL_DIV));
    GMAC_CRITICAL_SECTION_LEAVE();

    mac_buf_init();

    NVIC_DisableIRQ(GMAC_IRQn);
    NVIC_ClearPendingIRQ(GMAC_IRQn);
    NVIC_EnableIRQ(GMAC_IRQn);
}

void mac_deinit(void)
{
    // Disable all GMAC Interrupt
    GMAC_REGS->GMAC_IDR = 0xFFFFFFFF;

    // Disable transmit/receive
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR = 0x0;
    GMAC_CRITICAL_SECTION_LEAVE();

    // Disable Interrupt
    NVIC_DisableIRQ(GMAC_IRQn);
}

void mac_enable(void)
{
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR |= (GMAC_NCR_RXEN_Msk | GMAC_NCR_TXEN_Msk);
    GMAC_CRITICAL_SECTION_LEAVE();
}

void mac_disable(void)
{
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR &= ~(GMAC_NCR_RXEN_Msk | GMAC_NCR_TXEN_Msk);
    GMAC_CRITICAL_SECTION_LEAVE();
}

int32_t mac_write(uint8_t *buf, uint32_t len)
{
    uint32_t pos;
    uint32_t blen;
    uint32_t i;

    ASSERT(buf && len);

    if (mac_txbuffer_desc[_last_txbuf_index].status.bm.used && !mac_txbuffer_desc[_last_txbuf_index].status.bm.last_buf) {
        // Set used flag from first descriptor to last descriptor, as DMA olny set the first used flag
        for (i = 1; i < CONF_GMAC_TX_DESC_NUM; i++) {
            pos = _last_txbuf_index + i;
            if (pos >= CONF_GMAC_TX_DESC_NUM) {
                pos -= CONF_GMAC_TX_DESC_NUM;
            }
            mac_txbuffer_desc[pos].status.bm.used = 1;

            if (mac_txbuffer_desc[pos].status.bm.last_buf) {
                break;
            }
        }
    }

    if (!mac_txbuffer_desc[_txbuf_index].status.bm.used) {
        return ERR_NO_RESOURCE;
    }

    // Check if have enough buffers, the first buffer already checked
    if (len > CONF_GMAC_TXBUF_SIZE) {
        for (i = 1; i < CONF_GMAC_TX_DESC_NUM; i++) {
            pos = _txbuf_index + i;
            if (pos >= CONF_GMAC_TX_DESC_NUM) {
                pos -= CONF_GMAC_TX_DESC_NUM;
            }

            if (!mac_txbuffer_desc[pos].status.bm.used) {
                return ERR_NO_RESOURCE;
            }

            if ((len - (CONF_GMAC_TXBUF_SIZE * i)) < CONF_GMAC_TXBUF_SIZE) {
                break;
            }
        }
    }
    _last_txbuf_index = _txbuf_index;

    // Write data to transmit buffer
    for (i = 0; i < CONF_GMAC_TX_DESC_NUM; i++) {
        blen = min(len, CONF_GMAC_TXBUF_SIZE);
        memcpy(_txbuf[_txbuf_index], buf + (i * CONF_GMAC_TXBUF_SIZE), blen);
        len -= blen;

        if (len > 0) {
            // Here the Used flag be set to zero
            mac_txbuffer_desc[_txbuf_index].status.val = blen;
        } else {
            mac_txbuffer_desc[_txbuf_index].status.val         = blen;
            mac_txbuffer_desc[_txbuf_index].status.bm.last_buf = 1;
        }
        _txbuf_index++;
        if (_txbuf_index == CONF_GMAC_TX_DESC_NUM) {
            _txbuf_index                                            = 0;
            mac_txbuffer_desc[CONF_GMAC_TX_DESC_NUM - 1].status.bm.wrap = 1;
        }
        if (len == 0) {
            break;
        }
    }

    // Data synchronization barrier
    __DSB();

    // Active Transmit
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR |= GMAC_NCR_TSTART_Msk;
    GMAC_CRITICAL_SECTION_LEAVE();

    return ERR_NONE;
}

uint32_t mac_read(uint8_t *buf, uint32_t len)
{
    uint32_t i;
    uint32_t j;
    uint32_t pos;
    uint32_t n;
    uint32_t sof       = 0xFFFFFFFF; // Start of Frame index
    uint32_t eof       = 0xFFFFFFFF; // End of Frame index
    uint32_t total_len = 0;          // Total length of received package

    for (i = 0; i < CONF_GMAC_RX_DESC_NUM; i++) {
        pos = _rxbuf_index + i;

        if (pos >= CONF_GMAC_RX_DESC_NUM) {
            pos -= CONF_GMAC_RX_DESC_NUM;
        }

        // No more data for Ethernet package
        if (!mac_rxbuffer_desc[pos].address.bm.ownership) {
            break;
        }

        if (mac_rxbuffer_desc[pos].status.bm.sof) {
            sof = i;
        }

        if ((mac_rxbuffer_desc[pos].status.bm.eof) && (sof != 0xFFFFFFFF)) {
            // eof now indicate the number of bufs the frame used
            eof = i;
            n   = mac_rxbuffer_desc[pos].status.bm.len;
            len = min(n, len);
            // Break process since the last data has been found
            break;
        }
    }

    if (eof != 0xFFFFFFFF) {
        j = eof + 1;
    } else if (sof != 0xFFFFFFFF) {
        j = sof;
    } else {
        j = i;
    }

    // Copy data to user buffer
    for (i = 0; i < j; i++) {
        if (eof != 0xFFFFFFFF && i >= sof && i <= eof && len > 0) {
            n = min(len, CONF_GMAC_RXBUF_SIZE);
            memcpy(buf, _rxbuf[_rxbuf_index], n);
            buf += n;
            total_len += n;
            len -= n;
        }

        mac_rxbuffer_desc[_rxbuf_index].address.bm.ownership = 0;
        _rxbuf_index++;

        if (_rxbuf_index == CONF_GMAC_RX_DESC_NUM) {
            _rxbuf_index = 0;
        }
    }

    return total_len;
}

uint32_t mac_read_len(void)
{
    uint32_t i;
    uint32_t pos;
    bool     sof       = false; // Start of Frame
    uint32_t total_len = 0;     // Total length of received package

    for (i = 0; i < CONF_GMAC_RX_DESC_NUM; i++) {
        pos = _rxbuf_index + i;

        if (pos >= CONF_GMAC_RX_DESC_NUM) {
            pos -= CONF_GMAC_RX_DESC_NUM;
        }

        // No more data for Ethernet package
        if (!(mac_rxbuffer_desc[pos].address.bm.ownership)) {
            break;
        }

        if (mac_rxbuffer_desc[pos].status.bm.sof) {
            sof = true;
        }
        if (sof == true) {
            total_len += mac_rxbuffer_desc[pos].status.bm.len;
        }

        if (mac_rxbuffer_desc[pos].status.bm.eof) {
            // Break process since the last data has been found
            break;
        }
    }

    return total_len;
}

void mac_enable_irq(void)
{
    NVIC_EnableIRQ(GMAC_IRQn);
}

void mac_disable_irq(void)
{
    NVIC_DisableIRQ(GMAC_IRQn);
}

int32_t mac_register_callback(const enum mac_cb_type type, const mac_cb_t func)
{
    switch (type) {
    case MAC_RECEIVE_CB:
        cb.receive = (mac_cb_t)func;
        if (func) {
            GMAC_REGS->GMAC_IER = GMAC_IMR_RCOMP_Msk;
        } else {
            GMAC_REGS->GMAC_IDR = GMAC_IMR_RCOMP_Msk;
        }
        return ERR_NONE;
    case MAC_TRANSMIT_CB:
        cb.transmit = (mac_cb_t)func;
        if (func) {
            GMAC_REGS->GMAC_IER = GMAC_IMR_TCOMP_Msk;
        } else {
            GMAC_REGS->GMAC_IDR = GMAC_IMR_TCOMP_Msk;
        }
        return ERR_NONE;
    default:
        return ERR_INVALID_ARG;
    }
    return ERR_INVALID_ARG;
}

void mac_set_filter(uint8_t index, struct mac_filter *filter)
{
    ASSERT(filter);
    ASSERT(index < 4);

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->SA[index].GMAC_SAB = *((uint32_t *)(filter->mac));
    GMAC_CRITICAL_SECTION_LEAVE();

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->SA[index].GMAC_SAT = *((uint16_t *)(filter->mac + 4));
    GMAC_CRITICAL_SECTION_LEAVE();

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_TIDM[index] = GMAC_TIDM_TID(*((uint16_t *)(filter->tid)) | filter->tid_enable << 31);
    GMAC_CRITICAL_SECTION_LEAVE();
}

void mac_set_filter_ex(uint8_t mac[6])
{
    ASSERT(mac);

    uint8_t j;
    uint8_t m;
    uint8_t n;
    uint8_t k = 0;

    // Apply the hash function
    for (j = 0; j < 48; j += 6) {
        // Calculate the shift count
        n = j / 8;
        m = j % 8;

        // Update hash value
        if (!m) {
            k ^= mac[n];
        } else {
            k ^= (mac[n] >> m) | (mac[n + 1] << (8 - m));
        }
    }

    // The hash value is reduced to a 6-bit index
    k &= 0x3F;

    if (k < 32) {
        GMAC_CRITICAL_SECTION_ENTER();
        GMAC_REGS->GMAC_HRB |= (1 << k);
        GMAC_CRITICAL_SECTION_LEAVE();
    } else {
        GMAC_CRITICAL_SECTION_ENTER();
        GMAC_REGS->GMAC_HRT |= (1 << (k % 32));
        GMAC_CRITICAL_SECTION_LEAVE();
    }
}

void mac_write_phy_reg(uint16_t addr, uint16_t reg, uint16_t val)
{
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR |= GMAC_NCR_MPE_Msk;
    GMAC_CRITICAL_SECTION_LEAVE();

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_MAN = (GMAC_MAN_OP(1) | CONF_GMAC_CLTTO << 30 | GMAC_MAN_WTN(2) | \
                            GMAC_MAN_PHYA(addr) | GMAC_MAN_REGA(reg) | GMAC_MAN_DATA(val));
    GMAC_CRITICAL_SECTION_LEAVE();

    // Wait for the write operation complete
    while (!((GMAC_REGS->GMAC_NSR & GMAC_NSR_IDLE_Msk) >> GMAC_NSR_IDLE_Pos)) {}

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR &= ~GMAC_NCR_MPE_Msk;
    GMAC_CRITICAL_SECTION_LEAVE();
}

void mac_read_phy_reg(uint16_t addr, uint16_t reg, uint16_t *val)
{
    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR |= GMAC_NCR_MPE_Msk;
    GMAC_CRITICAL_SECTION_LEAVE();

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_MAN = ( GMAC_MAN_OP(2)          | \
                            CONF_GMAC_CLTTO << 30   | \
                            GMAC_MAN_WTN(0x2)       | \
                            GMAC_MAN_PHYA(addr)     | \
                            GMAC_MAN_REGA(reg));
    GMAC_CRITICAL_SECTION_LEAVE();


    // Wait for the read operation complete
    while (!((GMAC_REGS->GMAC_NSR & GMAC_NSR_IDLE_Msk) >> GMAC_NSR_IDLE_Pos)) {;}

    *val = GMAC_MAN_DATA(GMAC_REGS->GMAC_MAN);

    GMAC_CRITICAL_SECTION_ENTER();
    GMAC_REGS->GMAC_NCR &= ~GMAC_NCR_MPE_Msk;
    GMAC_CRITICAL_SECTION_LEAVE();
}

static void mac_read_cb(void)
{
    if (cb.receive) {
        cb.receive(NULL);
    }
}

static void mac_write_cb(void)
{
    if (cb.transmit) {
        cb.transmit(NULL);
    }
}
