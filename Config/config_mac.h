/**
 * @file config_mac.c
 * @author cy023
 * @date 2022.08.23
 * @brief MAC Layer configuration
 * 
 */

#ifndef CONFIG_MAC_H
#define CONFIG_MAC_H

/*******************************************************************************
 * Network Control Register
 ******************************************************************************/
// LoopBack Local
//
// Writing '1' to this bit, connects GTX to GRX, GTXEN to GRXDV 
// and forces full duplex mode.
//
// 0: Loop back local is disabled
// 1: Loop back local is enabled
#ifndef CONF_GMAC_NCR_LBL
#define CONF_GMAC_NCR_LBL 0
#endif

// Management Port Enable
//
// 0: Management Port (MDIO, MDC) is disabled
// 1: Management Port (MDIO, MDC) is enabled
#ifndef CONF_GMAC_NCR_MPE
#define CONF_GMAC_NCR_MPE 1
#endif

// Write Enable for Statistics Registers
//
// 0: Statistics Registers are write-protected
// 1: Statistics Registers are write-enabled
#ifndef CONF_GMAC_NCR_WESTAT
#define CONF_GMAC_NCR_WESTAT 0
#endif

// Enable Back pressure
//
// In 10M or 100M half duplex mode, writing a '1' to this bit forces collisions on all received frames.
// Ignored in gigabit half duplex mode.
//
// 0: Frame collisions are not forced
// 1: Frame collisions are forced in 10M and 100M half duplex mode
#ifndef CONF_GMAC_NCR_BP
#define CONF_GMAC_NCR_BP 0
#endif

// Enable PFC Priority-based Pause Reception
//
// Writing a '1' to this bit enables PFC Priority Based Pause Reception capabilities,
// enabling PFC negotiation and recognition of priority-based pause frames.
//
// 0: Normal operation
// 1: PFC Priority-based Pause frames are recognized
#ifndef CONF_GMAC_NCR_ENPBPR
#define CONF_GMAC_NCR_ENPBPR 0
#endif

// Transmit PFC Priority-based Pause Frame
//
// Takes the values stored in the Transmit PFC Pause Register
#ifndef CONF_GMAC_NCR_TXPBPF
#define CONF_GMAC_NCR_TXPBPF 0
#endif

/*******************************************************************************
 * GMAC Network Configuration Register
 ******************************************************************************/
// Speed
//
// 0: 10Mbps
// 1: 100Mbps
#ifndef CONF_GMAC_NCFGR_SPD
#define CONF_GMAC_NCFGR_SPD 1
#endif

// Full Duplex
//
// 0: Disable Full Duplex.
// 1: Enable Full Duplex. transmit block ignores the state of collision and
//      carrier sense and allows receive while transmitting.
#ifndef CONF_GMAC_NCFGR_FD
#define CONF_GMAC_NCFGR_FD 1
#endif

// Discard Non-VLAN Frames
//
// 0: Allows both VLAN_tagged and untagged frames to pass to the
//      address matching logic.
// 1: Allows only VLAN-tagged frames to pass to the address matching logic.
#ifndef CONF_GMAC_NCFGR_DNVLAN
#define CONF_GMAC_NCFGR_DNVLAN 0
#endif

// Jumbo Frame Size
//
// 1: enables jumbo frames of up to 10240 bytes to be accepted.
// The default length is 10240 bytes.
#ifndef CONF_GMAC_NCFGR_JFRAME
#define CONF_GMAC_NCFGR_JFRAME 0
#endif

// Copy All Frames
//
// All valid frames will be accepted.
#ifndef CONF_GMAC_NCFGR_CAF
#define CONF_GMAC_NCFGR_CAF 1
#endif

// No broadcast
//
// 0: Allow broadcast address 0xFFFFFFFFFFFF
// 1: Reject broadcast address 0xFFFFFFFFFFFF
#ifndef CONF_GMAC_NCFGR_NBC
#define CONF_GMAC_NCFGR_NBC 0
#endif

// Multicast Hash Enable
//
// 0: disables multicast hashing.
// 1: Multicast frames will be accepted when the 6-bit hash function of the
//      destination address points to a bit that is set in the Hash Register.
#ifndef CONF_GMAC_NCFGR_MTIHEN
#define CONF_GMAC_NCFGR_MTIHEN 0
#endif

// Unicast Hash Enable
//
// 0: disables Unicast hashing.
// 1: Unicast frames will be accepted when the 6-bit hash function of the
//      destination address points to a bit that is set in the Hash Register.
#ifndef CONF_GMAC_NCFGR_UNIHEN
#define CONF_GMAC_NCFGR_UNIHEN 0
#endif

// 1536 Maximum Frame Size
//
// 0: any frame above 1518 bytes in length is rejected.
// 1: increases the maximum accepted frame size to 1536 bytes in length
#ifndef CONF_GMAC_NCFGR_MAXFS
#define CONF_GMAC_NCFGR_MAXFS 1
#endif

// Retry Test
//
// Must be set to zero for normal operation.
// If set to '1' the backoff between collisions will always be one slot time.
// Setting this bit to '1' helps test the too many retries condition.
// Also used in the pause frame tests to reduce the pause counter's decrement
// time from 512 bit times, to every GRXCK cycle.
#ifndef CONF_GMAC_NCFGR_RTY
#define CONF_GMAC_NCFGR_RTY 0
#endif

// Pause Enable
//
// When set, transmission will pause if a non-zero 802.3 classic pause frame
// is received and PFC has not been negotiated
#ifndef CONF_GMAC_NCFGR_PEN
#define CONF_GMAC_NCFGR_PEN 0
#endif

// Receive Buffer Offset <0-3>
//
// Indicates the number of bytes by which the received data is offset from
// the start of the receive buffer.
#ifndef CONF_GMAC_NCFGR_RXBUFO
#define CONF_GMAC_NCFGR_RXBUFO 0
#endif

// Length Field Error Frame Discard
//
// 1: discards frames with a measured length shorter than the extracted length
//      field (as indicated by bytes 13 and 14 in a non-VLAN tagged frame).
//      This only applies to frames with a length field less than 0x0600.
#ifndef CONF_GMAC_NCFGR_LFERD
#define CONF_GMAC_NCFGR_LFERD 0
#endif

// Remove FCS
//
// 1: cause received frames to be written to memory without their frame check 
//      sequence (last 4 bytes). The indicated frame length will be reduced 
//      by four bytes in this mode.
#ifndef CONF_GMAC_NCFGR_RFCS
#define CONF_GMAC_NCFGR_RFCS 0
#endif

// MDC Clock Division
//
// Set according to MCK speed. These three bits determine the number MCK
// will be divided by to generate Management Data Clock (MDC). For
// conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
// Note: (MDC is only active during MDIO read and write operations).
//
// 0: MCK_8 MCK divided by 8 (MCK up to 20MHz)
// 1: MCK_16 MCK divided by 16 (MCK up to 40MHz)
// 2: MCK_32 MCK divided by 32 (MCK up to 80MHz)
// 3: MCK_48 MCK divided by 48 (MCK up to 120MHz)
// 4: MCK_64 MCK divided by 64 (MCK up to 160MHz)
// 5: MCK_96 MCK divided by 96 (MCK up to 240MHz)
#ifndef CONF_GMAC_NCFGR_CLK
#define CONF_GMAC_NCFGR_CLK 4
#endif


// For conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
#ifndef CONF_GMAC_MCK_FREQUENCY
#if CONF_GMAC_NCFGR_CLK == 0
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 8)
#elif CONF_GMAC_NCFGR_CLK == 1
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 16)
#elif CONF_GMAC_NCFGR_CLK == 2
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 32)
#elif CONF_GMAC_NCFGR_CLK == 3
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 48)
#elif CONF_GMAC_NCFGR_CLK == 4
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 64)
#elif CONF_GMAC_NCFGR_CLK == 5
#define CONF_GMAC_MCK_FREQUENCY (CONF_GMAC_FREQUENCY / 96)
#endif
#endif

#if CONF_GMAC_MCK_FREQUENCY > 2500000
#warning For conformance with the 802.3 specification, MDC must not exceed 2.5 MHz
#endif

// Disable Copy of Pause Frames
//
// Set to one to prevent valid pause frames being copied to memory. When
// set, pause frames are not copied to memory regardless of the state of
// the Copy All Frames bit, whether a hash match is found or whether a
// type ID match is identified. If a destination address match is found,
// the pause frame will be copied to memory. Note that valid pause frames
// received will still increment pause statistics and pause the
// transmission of frames as required.
#ifndef CONF_GMAC_NCFGR_DCPF
#define CONF_GMAC_NCFGR_DCPF 0
#endif

// Receive Checksum Offload Enable
//
// When set, the receive checksum engine is enabled. Frames with bad IP,
// TCP or UDP checksums are discarded.
#ifndef CONF_GMAC_NCFGR_RXCOEN
#define CONF_GMAC_NCFGR_RXCOEN 0
#endif

// Enable Frames Received in Half Duplex
//
// Enable frames to be received in half-duplex mode while transmitting.
#ifndef CONF_GMAC_NCFGR_EFRHD
#define CONF_GMAC_NCFGR_EFRHD 1
#endif

// Ignore RX FCS
//
// For normal operation this bit must be written to zero.
// When set, frames with FCS/CRC errors will not be rejected. FCS error
// statistics will still be collected for frames with bad FCS and FCS
// status will be recorded in the DMA descriptor of the frame.
#ifndef CONF_GMAC_NCFGR_IRXFCS
#define CONF_GMAC_NCFGR_IRXFCS 0
#endif

// IP Stretch Enable
//
// When set, the transmit IPG can be increased above 96 bit times depending
// on the previous frame length using the IPG Stretch Register.
#ifndef CONF_GMAC_NCFGR_IPGSEN
#define CONF_GMAC_NCFGR_IPGSEN 0
#endif

// Receive Bad Preamble
//
// When set, frames with non-standard preamble are not rejected.
#ifndef CONF_GMAC_NCFGR_RXBP
#define CONF_GMAC_NCFGR_RXBP 0
#endif

// Ignore IPG GRXER
//
// When set, GRXER has no effect on the GMAC's operation when GRXDV is low.
#ifndef CONF_GMAC_NCFGR_IRXER
#define CONF_GMAC_NCFGR_IRXER 1
#endif

/*******************************************************************************
 * GMAC User Register
 ******************************************************************************/
// Reduced MII Mode
//
// 0: RMII mode is selected
// 1: MII mode is selected
#ifndef CONF_GMAC_UR_MII
#define CONF_GMAC_UR_MII 0
#endif

/*******************************************************************************
 * GMAC PHY Maintenance Register
 ******************************************************************************/
// Clause 22 Operation
//
// 0: Clause 45 Operation
// 1: Clause 22 Operation
#ifndef CONF_GMAC_CLTTO
#define CONF_GMAC_CLTTO 1
#endif

/*******************************************************************************
 * GMAC PHY Maintenance Register
 ******************************************************************************/
// Enable Stacked VLAN Processing Mode
//
// When enabled, the first VLAN tag in a received frame will only be
// accepted if the VLAN type field is equal to the User defined VLAN Type,
// OR equal to the standard VLAN type (0x8100). Note that the second VLAN
// tag of a Stacked VLAN packet will only be matched correctly if its
// VLAN_TYPE field equals 0x8100.
//
// 0: Disable the stacked VLAN processing mode
// 1: Enable the stacked VLAN processing mode
#ifndef CONF_GMAC_SVLAN_ENABLE
#define CONF_GMAC_SVLAN_ENABLE 0
#endif

// User Defined VLAN_TYPE Field <0x0-0xFFFF>
//
// When Stacked VLAN is enabled (ESVLAN=1), the first VLAN tag in a received
// frame will only be accepted if the VLAN type field is equal to this user
// defined VLAN_TYPE, OR equal to the standard VLAN type (0x8100).
#ifndef CONF_GMAC_SVLAN_TYPE
#define CONF_GMAC_SVLAN_TYPE 0x8100
#endif

/*******************************************************************************
 * GMAC DMA Configuration Register
 ******************************************************************************/
// DMA Configuration
//
// The GMAC DMA controller is connected to the MAC FIFO interface and
// provides a scatter-gather type capability for packet data storage.
// The DMA implements packet buffering where dual-port memories are used
// to buffer multiple frames.
// <id> gmac_arch_dma_cfg
// FIXME:
// #ifndef CONF_GMAC_DMA_CFG
// #define CONF_GMAC_DMACFG 1
// #endif

// Fixed Burst Length for DMA Data Operations
// 
// Selects the burst length to attempt to use on the AHB when transferring frame
// data. Not used for DMA management operations and only used where space and
// data size allow. Otherwise SINGLE type AHB transfers are used.
// One-hot priority encoding enforced automatically on register writes as
// follows. ‘x’ represents don’t care.
//
//  0:     -      Reserved
//  1:   SINGLE   00001: Always use SINGLE AHB bursts
//  2:     -      Reserved
//  4:   INCR4    001xx: Attempt to use INCR4 AHB bursts (Default)
//  8:   INCR8    01xxx: Attempt to use INCR8 AHB bursts
// 16:   INCR16   1xxxx: Attempt to use INCR16 AHB bursts
#ifndef CONF_GMAC_DCFGR_FBLDO
#define CONF_GMAC_DCFGR_FBLDO 4
#endif

//  Endian Swap Mode Enable for Management Descriptor Accesses
//
// 0: Little endian mode for AHB transfers selected.
// 1: Big endian mode for AHB transfers selected.
#ifndef CONF_GMAC_DCFGR_ESMA
#define CONF_GMAC_DCFGR_ESMA 0
#endif

//  Endian Swap Mode Enable for Packet Data Accesses
//
// 0: Little endian mode for AHB transfers selected.
// 1: Big endian mode for AHB transfers selected.
#ifndef CONF_GMAC_DCFGR_ESPA
#define CONF_GMAC_DCFGR_ESPA 0
#endif

// Receiver Packet Buffer Memory Size Select
//
// The default receive packet buffer size is FULL=RECEIVE_BUFFER_SIZE= 4Kbytes.
// The table below shows how to configure this memory to FULL, HALF, QUARTER
// or EIGHTH of the default size.
//
// 0: EIGHTH RECEIVE_BUFFER_SIZE = 4/8 Kbyte Memory Size
// 1: QUARTER RECEIVE_BUFFER_SIZE = 4/4 Kbytes Memory Size
// 2: HALF RECEIVE_BUFFER_SIZE = 4/2 Kbytes Memory Size
// 3: FULL RECEIVE_BUFFER_SIZE = 4 Kbytes Memory Size
#ifndef CONF_GMAC_DCFGR_RXBMS
#define CONF_GMAC_DCFGR_RXBMS 3
#endif

// Transmitter Packet Buffer Memory Size Select
//
// When written to zero, the amount of memory used for the transmit packet
// buffer is reduced by 50%. This reduces the amount of memory used by the GMAC.
// It is important to write this bit to '1' if the full configured physical
// memory is available. The value in parentheses represents the size that would
// result for the default maximum configured memory size of 4KBytes.
//
// 0: Top address bits not used. (2KByte used.)
// 1: Full configured addressable space (4KBytes) used.
#ifndef CONF_GMAC_DCFGR_TXPBMS
#define CONF_GMAC_DCFGR_TXPBMS 1
#endif

// Transmitter Checksum Generation Offload Enable
//
// Transmitter IP, TCP and UDP checksum generation offload enable.
//
// 0: Frame data is unaffected.
// 1: The transmitter checksum generation engine calculates and substitutes
//      checksums for transmit frames.
#ifndef CONF_GMAC_DCFGR_TXCOEN
#define CONF_GMAC_DCFGR_TXCOEN 0
#endif

// DMA Receive Buffer Size <1-255>
//
// These bits defined by these bits determines the size of buffer to use in main
// AHB system memory when writing received data.
//
// The value is defined in multiples of 64 bytes. For example:
//      0x02: 128 bytes
//      0x18: 1536 bytes (1 × max length frame/buffer)
//      0xA0: 10240 bytes (1 × 10K jumbo frame/buffer)
#ifndef CONF_GMAC_DCFGR_DRBS
#define CONF_GMAC_DCFGR_DRBS 2
#endif

// DMA Discard Receive Packets
//
// A write to this bit is ignored if the DMA is not configured in the packet
// buffer full store and forward mode.
//
// 0: Received packets are stored in the SRAM based packet buffer until next 
//      AHB buffer resource becomes available.
// 1: Receive packets from the receiver packet buffer memory are automatically
//      discarded when no AHB resource is available.
#ifndef CONF_GMAC_DCFGR_DDRP
#define CONF_GMAC_DCFGR_DDRP 0
#endif

/*******************************************************************************
 * GMAC TX Partial Store and Forward Register
 ******************************************************************************/
// Enable TX Partial Store and Forward Operation
//
// This allows for a reduced latency but there are performance implications.
// #ifndef CONF_GMAC_TPSF_EN
// #define CONF_GMAC_TPSF_EN 0
// #endif

// Watermark <20-4095>
//
// Transmit Partial Store and Forward Address Watermark value.
// <i> Byte size of buffer for each transmit buffer descriptor.
// #ifndef CONF_GMAC_TPSF_WM
// #define CONF_GMAC_TPSF_WM 100
// #endif

/*******************************************************************************
 * GMAC RX Partial Store and Forward Register
 ******************************************************************************/
// Enable RX Partial Store and Forward Operation
// This allows for a reduced latency but there are performance implications.
// #ifndef CONF_GMAC_RPSF_EN
// #define CONF_GMAC_RPSF_EN 0
// #endif

// Watermark <20-4095>
// Receive Partial Store and Forward Address Watermark value. Reset = 1.
// Byte size of buffer for each transmite buffer descriptor.
// #ifndef CONF_GMAC_RPSF_WM
// #define CONF_GMAC_RPSF_WM 100
// #endif

/*******************************************************************************
 * GMAC IPG Stretch Register
 ******************************************************************************/
// Frame Length
// Bits FL[7:0] are multiplied with the previously transmitted frame length
// (including preamble), and divided by FL[15:8]+1 (adding 1 to prevent division
// by zero).
//
// RESULT = FL[7:0] / (FL[15:8] + 1)
//
// If RESULT > 96 and the IP Stretch Enable bit in the Network Configuration
// Register (NCFGR.IPGSEN) is written to '1', RESULT is used for the transmit
// inter-packet-gap.

// IPG Stretch Multiple <0-15>
// This value will multiplied with the previously transmitted frame length
// (including preamble) FIXME:
#ifndef CONF_GMAC_IPGS_FL_MUL
#define CONF_GMAC_IPGS_FL_MUL 1
#endif

// IPG Stretch Divide <1-16>
// Divide the frame length. If the resulting number is greater than 96 and
// IP Stretch Enabled then the resulting number is used for the transmit
// inter-packet-gap FIXME:
#ifndef CONF_GMAC_IPGS_FL_DIV
#define CONF_GMAC_IPGS_FL_DIV 1
#endif

/*******************************************************************************
 * Advanced configuration
 ******************************************************************************/

// Number of Transmit Buffer Descriptor <1-255>
#ifndef CONF_GMAC_TX_DESC_NUM
#define CONF_GMAC_TX_DESC_NUM 2
#endif

// Number of Receive Buffer Descriptor <1-255>
#ifndef CONF_GMAC_RX_DESC_NUM
#define CONF_GMAC_RX_DESC_NUM 16
#endif

// Byte size of Transmit Buffer <64-10240>
#ifndef CONF_GMAC_TXBUF_SIZE
#define CONF_GMAC_TXBUF_SIZE 1500
#endif

#ifndef CONF_GMAC_RXBUF_SIZE
#define CONF_GMAC_RXBUF_SIZE (CONF_GMAC_DCFGR_DRBS * 64)
#endif

#endif /* CONFIG_MAC_H */
