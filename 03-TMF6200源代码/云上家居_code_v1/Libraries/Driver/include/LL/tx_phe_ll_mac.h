/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_mac.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    08-10-2018
  * @brief   This file contains all the MAC LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  *
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TX_PHE_LL_MAC_H
#define __TX_PHE_LL_MAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "typedef.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup mac_interface_gr MAC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup MAC_LL_Driver MAC LL Driver
  * @ingroup  mac_interface_gr
  * @brief Mainly the driver part of the MAC module, which includes \b MAC \b Register 
  * \b Constants, \b MAC \b Exported \b Constants, \b MAC \b Exported \b Struct, \b MAC
  * \b Data \b transfers \b functions, \b MAC \b Initialization \b and \b MAC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup MAC_LL_Register_Constants MAC LL Register Constants
  * @ingroup  MAC_LL_Driver
  * @brief    MAC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the MAC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the MAC register, mainly for convenience. Understand the 
    configuration of the MAC.
    
@endverbatim
  *
  * @{
  */

/***** CSR0(The bus mode register) *****/
/*! Transmit automatic polling interval setting.
 */
#define LL_MAC_CSR0_TAP_SET(n)                      (((n)&0x07) << 17)
#define LL_MAC_CSR0_TAP_MASK                        (0x07UL << 17)
/*! Specifies the maximum number of words that can be transferred within one
 *  DMA transaction.
 *  @note PBL cannot be 0, the recommended setting is 16
 */
#define LL_MAC_CSR0_PBL_SET(n)                      (((n)&0x3F) << 8)
/*! Specifies the number of 128-bit words between two consecutive unchained
 *  descriptors.
 */
#define LL_MAC_CSR0_DSL_SET(n)                      (((n)&0x1F) << 2)
/*! Software reset.
 */
#define LL_MAC_CSR0_SWR_EN                          (1UL << 0)


/***** CSR5(The status register) *****/
/*! Transmit process state.
 */
#define LL_MAC_CSR5_TS_GET(n)                       (((n) >> 20) & 0x07)
/*! Receive process state .
 */
#define LL_MAC_CSR5_RS_GET(n)                       (((n) >> 17) & 0x07)
/*! Normal interrupt summary.
 */
#define LL_MAC_CSR5_NIS_PENDING                     (1UL << 16)
/*! Abnormal interrupt summary.
 */
#define LL_MAC_CSR5_AIS_PENDING                     (1UL << 15)
/*! Early receive interrupt.
 */
#define LL_MAC_CSR5_ERI_PENDING                     (1UL << 14)
/*! General-purpose timer expiration.
 */
#define LL_MAC_CSR5_GTI_PENDING                     (1UL << 11)
/*! Early transmit interrupt.
 */
#define LL_MAC_CSR5_ETI_PENDING                     (1UL << 10)
/*! Receive process stopped.
 */
#define LL_MAC_CSR5_RPS_PENDING                     (1UL << 8)
/*! Receive buffer unavailable.
 */
#define LL_MAC_CSR5_RU_PENDING                      (1UL << 7)
/*! Receive interrupt.
 */
#define LL_MAC_CSR5_RI_PENDING                      (1UL << 6)
/*! Transmit underflow.
 */
#define LL_MAC_CSR5_UNF_PENDING                     (1UL << 5)
/*! Transmit buffer unavailable.
 */
#define LL_MAC_CSR5_TU_PENDING                      (1UL << 2)
/*! Transmit process stopped.
 */
#define LL_MAC_CSR5_TPS_PENDING                     (1UL << 1)
/*! Transmit interrupt.
 */
#define LL_MAC_CSR5_TI_PENDING                      (1UL << 0)


/***** CSR6(The operation mode register) *****/
/*! Receive all.
 */
#define LL_MAC_CSR6_RA_EN                           (1UL << 30)
/*! mask output of mac module.
 */
#define LL_MAC_CSR6_MASK_EN                         (1UL << 29)
/*! Write 1 to reset txfifo & tx buffer addr, auto clear.
 */
#define LL_MAC_CSR6_CPU_RST_TX                      (1UL << 28)
/*! Write 1 to reset rxfifo & rx buffer addr, auto clear.
 */
#define LL_MAC_CSR6_CPU_RST_RX                      (1UL << 27)
/*! auto reset txfifo & tx buffer addr, if chained frame finished and the
 *  next frame is not chained.
 */
#define LL_MAC_CSR6_TCH_RST_EN                      (1UL << 26)
/*! auto reset rxfifo & rx buffer addr, if chained frame finished and the
 *  next frame is not chained.
 */
#define LL_MAC_CSR6_RCH_RST_EN                      (1UL << 25)
/*! auto reset rxfifo & rx buffer addr, if receive buffer unavailable.
 */
#define LL_MAC_CSR6_RU_RST_EN                       (1UL << 24)
/*! auto disable RX if receive buffer unavailable.
 */
#define LL_MAC_CSR6_RU_DIS_EN                       (1UL << 23)
/*! Transmit threshold mode.
 */
#define LL_MAC_CSR6_TTM_EN                          (1UL << 22)
/*! When set, the transmission starts after a full packet is written into
 *  the transmit FIFO, regardless of the current FIFO threshold level.
 */
#define LL_MAC_CSR6_SF_EN                           (1UL << 21)
/*! rx address scatter mode(use RXDES2).
 */
#define LL_MAC_CSR6_RXADR_UP_EN                     (1UL << 19)
/*! tx address scatter mode(use TXDES2).
 */
#define LL_MAC_CSR6_TXADR_UP_EN                     (1UL << 18)
/*! Ethernet Speed Selection.
 */
#define LL_MAC_CSR6_SPEED_SET(n)                    (((n)&0x03) << 16)
#define LL_MAC_CSR6_SPEED_MASK                      (0x03UL << 16)
/*! Threshold control bits.
 */
#define LL_MAC_CSR6_TR_SET(n)                       (((n)&0x03) << 14)
/*! Start/stop transmit command.
 */
#define LL_MAC_CSR6_ST_EN                           (1UL << 13)
/*! Full duplex mode.
 */
#define LL_MAC_CSR6_FD                              (1UL << 9)
/*! Pass all multicast.
 */
#define LL_MAC_CSR6_PM_EN                           (1UL << 7)
/*! Promiscuous mode.
 */
#define LL_MAC_CSR6_PR_EN                           (1UL << 6)
/*! Inverse filtering.
 */
#define LL_MAC_CSR6_IF                              (1UL << 4)
/*! Pass bad frames.
 */
#define LL_MAC_CSR6_PB_EN                           (1UL << 3)
/*! Hash only filtering mode.
 */
#define LL_MAC_CSR6_HO                              (1UL << 2)
/*! Start/stop receive command.
 */
#define LL_MAC_CSR6_SR_EN                           (1UL << 1)
/*! Hash/perfect receive filtering mode.
 */
#define LL_MAC_CSR6_HP                              (1UL << 0)


/***** CSR7(The interrupt enable register) *****/
/*! Normal interrupt summary enable.
 */
#define LL_MAC_CSR7_NIE_EN                          (1UL << 16)
/*! Abnormal interrupt summary enable.
 */
#define LL_MAC_CSR7_AIE_EN                          (1UL << 15)
/*! Early receive interrupt enable.
 */
#define LL_MAC_CSR7_ERE_EN                          (1UL << 14)
/*! General-purpose timer overflow enable.
 */
#define LL_MAC_CSR7_GTE_EN                          (1UL << 11)
/*! Early transmit interrupt enable.
 */
#define LL_MAC_CSR7_ETE_EN                          (1UL << 10)
/*! Receive stopped enable.
 */
#define LL_MAC_CSR7_RSE_EN                          (1UL << 8)
/*! Receive buffer unavailable enable.
 */
#define LL_MAC_CSR7_RUE_EN                          (1UL << 7)
/*! Receive interrupt enable.
 */
#define LL_MAC_CSR7_RIE_EN                          (1UL << 6)
/*! Underflow interrupt enable.
 */
#define LL_MAC_CSR7_UNE_EN                          (1UL << 5)
/*! Transmit buffer unavailable enable.
 */
#define LL_MAC_CSR7_TUE_EN                          (1UL << 2)
/*! Transmit stopped enable.
 */
#define LL_MAC_CSR7_TSE_EN                          (1UL << 1)
/*! Transmit interrupt enable.
 */
#define LL_MAC_CSR7_TIE_EN                          (1UL << 0)


/***** CSR8(The missed frames counter register) *****/
/*! Missed frame overflow.
 */
#define LL_MAC_CSR8_MFO_PENDING                     (1UL << 16)
/*! Missed frame counter.
 */
#define LL_MAC_CSR8_MFC(n)                          (((n)&0xFFFF) >> 0)


/***** CSR9(The serial ROM/Software MII Serial Management register) *****/
/*! MII management data in signal.
 */
#define LL_MAC_CSR9_MDI_GET(n)                      (((n) >> 19) & 0x01)
/*! MII management interface mode.
 */
#define LL_MAC_CSR9_MII_SET(n)                      (((n)&0x01) << 18)
/*! MII management write data.
 */
#define LL_MAC_CSR9_MDO_SET(n)                      (((n)&0x01) << 17)
/*! MII management clock.
 */
#define LL_MAC_CSR9_MDC_SET(n)                      (((n)&0x01) << 16)
/*! Serial ROM data output.
 */
#define LL_MAC_CSR9_SDO_SET(n)                      (((n)&0x01) << 3)
/*! Serial ROM data input.
 */
#define LL_MAC_CSR9_SDI_GET(n)                      (((n) >> 2) & 0x01)
/*! Serial ROM clock.
 */
#define LL_MAC_CSR9_SCLK_SET(n)                     (((n)&0x01) << 1)
/*! Serial ROM chip select.
 */
#define LL_MAC_CSR9_SCS_SET(n)                      (((n)&0x01) << 0)


/***** CSR10(The MII serial management register) *****/
/*! START/BUSY.
 */
#define LL_MAC_CSR10_SB                             (1UL << 31)
/*! clock divider settings.
 */
#define LL_MAC_CSR10_CLKDIV_SET(n)                  (((n)&0x07) << 28)
/*! MIISM operation code.
 */
#define LL_MAC_CSR10_OPCODE_SET(n)                  (((n)&0x03) << 26)
/*! Physical layer address for current transfer.
 */
#define LL_MAC_CSR10_PHYADD_SET(n)                  (((n)&0x1F) << 21)
/*! Register address for the current transfer.
 */
#define LL_MAC_CSR10_REGADD_SET(n)                  (((n)&0x1F) << 16)
/*! Register data.
 */
#define LL_MAC_CSR10_DATA(n)                        (((n)&0xFFFF) << 0)


/***** CSR11(Timer and interrupt mitigation control) *****/
/*! Controls the time units for the transmit and receive timers.
 */
#define LL_MAC_CSR11_CS_SET(n)                      (((n)&0x01) << 31)
/*! Controls the time that must elapse between the end of a transmit
 *  operation and issuing the transmit interrupt.
 */
#define LL_MAC_CSR11_TT_SET(n)                      (((n)&0x0F) << 27)
/*! Controls the number of the frames transmitted before issuing the
 *  transmit interrupt.
 */
#define LL_MAC_CSR11_NTP_SET(n)                     (((n)&0x07) << 24)
/*! Controls the time that must elapse between the end of a receive
 *  operation and issuing the receive interrupt.
 */
#define LL_MAC_CSR11_RT_SET(n)                      (((n)&0x0F) << 20)
/*! Controls the number of the received frames before issuing the receive
 *  interrupt.
 */
#define LL_MAC_CSR11_NRP_SET(n)                     (((n)&0x07) << 17)
/*! Continuous mode.
 */
#define LL_MAC_CSR11_CON_EN                         (1UL << 16)
/*! Contains the number of iterations of the general-purpose timer.
 */
#define LL_MAC_CSR11_TIM_SET(n)                     (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup MAC_LL_Exported_Constants MAC LL Exported Constants
  * @ingroup  MAC_LL_Driver
  * @brief    MAC LL external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the abstraction 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
  
/***** DRIVER API *****/



/***** LL API *****/


  
/***** LL API AND DRIVER API *****/
/**
  * @brief Transmit automatic polling intervals
  */
typedef enum {
/*  Time interval                           10Mbps          100Mbps             */
    LL_MAC_TAP_DISABLE          = 0,    /*    -                -                */
    LL_MAC_TAP_128_PERIOD       = 4,    /*  51.2us          5.12us              */
    LL_MAC_TAP_256_PERIOD       = 5,    /*  102.4us         10.24us             */
    LL_MAC_TAP_512_PERIOD       = 6,    /*  204.8us         20.48us             */
    LL_MAC_TAP_1024_PERIOD      = 7,    /*  409.6us         40.96us             */
    LL_MAC_TAP_2048_PERIOD      = 1,    /*  819.2us         81.92us             */
    LL_MAC_TAP_4096_PERIOD      = 2,    /*  1.638ms         163.8us             */
    LL_MAC_TAP_8192_PERIOD      = 3,    /*  3.276ms         327.6us             */
} TYPE_ENUM_LL_MAC_TAP_SEL;

/**
  * @brief Programmable burst length for data buffer access
  * @note  PBL cannot be 0, GMAC_PBL_16 is recommended.
  */
typedef enum {
    LL_MAC_PBL_0                = 0,
    LL_MAC_PBL_1                = 1,
    LL_MAC_PBL_2                = 2,
    LL_MAC_PBL_4                = 4,
    LL_MAC_PBL_8                = 8,
    LL_MAC_PBL_16               = 16,
    LL_MAC_PBL_32               = 32,
} TYPE_ENUM_LL_MAC_PBL_SEL;

/**
  * @brief MAC Transmit process state
  */
typedef enum {
    /*! stop
    */
    LL_MAC_TX_STOP              = 0,
    /*! fetching tx descriptor
    */
    LL_MAC_TX_FETCHING          = 1,
    /*! waiting for end of transmission
    */
    LL_MAC_TX_WAITING_END       = 2,
    /*! data buf from host to FIFO
    */
    LL_MAC_TX_TO_FIFO           = 3,
    /*! setup packet processing
    */
    LL_MAC_TX_SETUP_PROCESSING  = 5,
    /*! FIFO underflow/unavailable descriptor
    */
    LL_MAC_TX_SUSPENDED         = 6,
    /*! closing transmit descriptor
    */
    LL_MAC_TX_CLOSING           = 7,
} TYPE_ENUM_LL_MAC_TX_STATE;

/**
  * @brief MAC Receive process state
  */
typedef enum {
    /*! stop
    */
    LL_MAC_RX_STOP              = 0, 
    /*! fetching rx descriptor
    */
    LL_MAC_RX_FETCHING          = 1,
    /*! waiting for the end of receive packet
    */
    LL_MAC_RX_WAITING_END       = 2,
    /*! waiting for receive packet
    */
    LL_MAC_RX_WAITING_RECEIVE   = 3,
    /*! unavailable receive buffer
    */
    LL_MAC_RX_SUSPENDED         = 4,
    /*! closing receive descriptor
    */
    LL_MAC_RX_CLOSING           = 5,
    /*! transferring data from FIFO to host memory
    */
    LL_MAC_RX_TO_MEMORY         = 7,
} TYPE_ENUM_LL_MAC_RX_STATE;

/**
  * @brief MAC Ethernet speed selection
  */
typedef enum {
    LL_MAC_ETHERNET_100M        = 0,
    LL_MAC_ETHERNET_1G          = 1,
    LL_MAC_ETHERNET_10M         = 2,
} TYPE_ENUM_LL_MAC_SPEED_SEL;

/**
  * @brief MAC Software MII management interface mode
  */
typedef enum {
    LL_MAC_MII_WRITE            = 0,
    LL_MAC_MII_READ             = 1,
} TYPE_ENUM_LL_MAC_MII_SEL;

/**
  * @brief MAC MII serial management clock divider setting
  */
typedef enum {
    LL_MAC_CLK_DIV_8            = 0,
    LL_MAC_CLK_DIV_16           = 1,
    LL_MAC_CLK_DIV_32           = 2,
    LL_MAC_CLK_DIV_64           = 3,
    LL_MAC_CLK_DIV_128          = 4,
    LL_MAC_CLK_DIV_256          = 5,
    LL_MAC_CLK_DIV_512          = 6,
    LL_MAC_CLK_DIV_1024         = 7,
} TYPE_ENUM_LL_MAC_CLKDIV_SEL;

/**
  * @brief MAC MIISM operation code
  */
typedef enum {
    LL_MAC_DISABLE_CLK          = 0,
    LL_MAC_REG_WRITE            = 1,
    LL_MAC_REG_READ             = 2,
    LL_MAC_CLK_DIVIDER_SET      = 3,
} TYPE_ENUM_LL_MAC_OPCODE;

/**
  * @brief the time units for the transmit and receive timers
  */
typedef enum {
/*  time unit                               10Mbps      100Mbps                 */
    LL_MAC_128_MII_PERIOD       = 1,    /*  51.2us      5.12us                  */
    LL_MAC_2048_MII_PERIOD      = 0,    /*  819.2us     81.92us                 */
} TYPE_ENUM_LL_MAC_CS_SEL;

/**
  * @brief MAC filter type selection
  */
typedef enum {
    /*! Setup frame buffer is interpreted as a set of 16 48-bit physical
     *  addresses.
     * @note Whitelist mode
     */
    LL_MAC_PERFECT_FILTER = 0,
    /*! Setup frame buffer contains 512-bit hash table plus a single 48-bit
     *  physical address.
     * @note Whitelist mode. The hash table is only valid for multicast
     *       addresses.
     */
    LL_MAC_HASH_FILTER,
    /*! Setup frame buffer is interpreted as a set of 16 48-bit physical
     *  addresses.
     * @note Blacklist mode
     */
    LL_MAC_INVERSE_FILTER,
    /*! Setup frame buffer is interpreted as a 512-bit hash table.
     * @note Whitelist mode. The hash table is only valid for multicast
     *       addresses.
     */
    LL_MAC_HASH_ONLY_FILTER,
} TYPE_ENUM_LL_MAC_FILTER;

/**
  * @brief frame tx/rx status
  */
typedef enum {
    /*! frame transfer completed
     */
    LL_MAC_FRAME_COMPLETE = 0,
    /*! frame transmission error
     */
    LL_MAC_FRAME_ERR,
    /*! frame waiting for transmission completed
     */
    LL_MAC_FRAME_WAITTING_COMPLETE,
} TYPE_ENUM_LL_MAC_FRAME_STATUS;

/**
  * @}
  */

/** @defgroup MAC_LL_Exported_Struct MAC LL Exported Struct
  * @ingroup  MAC_LL_Driver
  * @brief    MAC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the MAC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_mac_init. Function, you can configure the MAC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief MAC configuration structure
  */
typedef struct __ll_mac_init {
    /*! RX descriptor start address
     * @note rx_descriptor_start_addr are aligned in 8 bytes and can only be
     *       located in SRAM0/SRAM10/SRAM1.  
     *       Statement example:  
     *        static u8 buf[1024] __attribute__((aligned(8),section("SRAM0")));
     */
    u32  rx_descriptor_start_addr;
    /*! TX descriptor start address
     * @note tx_descriptor_start_addr are aligned in 8 bytes and can only be
     *       located in SRAM0/SRAM10/SRAM1.  
     *       Statement example:  
     *        static u8 buf[1024] __attribute__((aligned(8),section("SRAM0")));
     */
    u32  tx_descriptor_start_addr;
    /* GMAC rx receive mode */
    /*! receive all frame
     */
    bool promiscuous_en;
    /*! receive all multicast frame
     */
    bool pass_all_multicast_en;
    /*! When set, the gmac transfers all frames into the data buffers,
     *  regardless of the receive errors
     */
    bool pass_bad_frame_en;
} TYPE_LL_MAC_INIT;

/**
  * @brief MAC interrupt configuration structure
  */
typedef struct __ll_mac_irq_cfg {
    /*! General-purpose timer overflow enable ?
     */
    bool timer_overflow_intr_en;
    
    /* rx */
    /*! Early receive interrupt enable ?
     */      
    bool rx_early_intr_en;
    /*! Receive buffer unavailable enable ?
     */
    bool rx_buf_unavailable_intr_en;
    /*! Receive interrupt enable ?
     */
    bool rx_intr_en;
    /*! Receive stopped enable ?
     */
    bool rx_stop_intr_en;
    
    /* tx */
    /*! Early transmit interrupt enable ?
     */
    bool tx_early_intr_en;
    /*! Transmit buffer unavailable enable ?
    */
    bool tx_buf_unavilable_intr_en;
    /*! Underflow interrupt enable ?
     */
    bool tx_underflow_intr_en;
    /*! Transmit interrupt enable ?
     */
    bool tx_intr_en;
    /*! Transmit stopped enable ?
     */
    bool tx_stop_intr_en;
} TYPE_LL_MAC_IRQ_CFG;

/**
  * @brief GMAC descriptor generic structure
  */
typedef struct __ll_mac_descriptor {
    u32 reg0;
    u32 reg1;
    u32 reg2;
    u32 reg3;
} TYPE_LL_MAC_DESCRIPTOR;

/**
  * @brief GMAC RX descriptor structure
  */
typedef struct __ll_mac_rx_descriptor {
    /*! This bit is reset for the frames with the legal length. When set,
     *  indicates that the frame was truncated due to the receive FIFO
     *  overflow.
     */
    u32 zero                : 1,
    /*! This bit is valid only when the last_descriptor is set.
     *  Additionally, the CE is not valid when the received frame is a
     *  runt frame.
     */
        crc_err             : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        dribbling_bit       : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        mii_err             : 1,
        reserved0           : 1,
    /*! When set, indicates that the frame has the length field greater
     *  then 1500 (Ethernet type frame). When cleared, indicates the
     *  802.3 type frame.  
     *  This bit is valid only when the last_descriptor is set.
     *  Additionally, the FT is invalid for the runt frames of a length
     *  shorter then 14 bytes.
     */
        frame_type          : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        collision_seen      : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        frame_too_long      : 1,
        last_descriptor     : 1,
        first_descriptor    : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        multicast_frame     : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        runt_frame          : 1,
        reserved1           : 2,
    /*! This bit is valid only when the last_descriptor is set.
     */
        descriptor_err      : 1,
    /*! This bit is valid only when the last_descriptor is set.
     */
        err_summary         : 1,
    /*! This bit is valid only when the last_descriptor is set and
     *  descriptor_err is cleared.
     */
        frame_len           : 14,
    /*! This bit is valid only for the last descriptor of the frame,
     *  when the CSR6.30 (receive all) bit is set, and the frame is at
     *  least 64 bytes long.
     */
        filtering_fail      : 1,
        own                 : 1;
    /*! Indicates the size, in bytes, of memory space used by the first
     *  data buffer. This number must be a multiple of 4.
     */
    u32 buf_1_size          : 11,
    /*! Useless, please set to 0.
     */
        buf_2_size          : 11,
        reserved2           : 2,
    /*! When set, indicates that the second buffer's address points to
     *  the next descriptor and not to the data buffer.
     * @note Note that the end_of_ring takes precedence over the
     *       second_addr_chained.
     */
        second_addr_chained : 1,
    /*! When set, indicates that this is the last descriptor in the
     *  receive descriptor ring.
     */
        end_of_ring         : 2;
    /*! Indicates the length, in bytes, of memory allocated for the
     *  first receive buffer. This number must be 64 byte aligned.
     *  @note rx_buf_addr_1 are aligned in 4 bytes and can only be located
     *        in SRAM3.  
     *        Statement example:  
     *         static u8 buf[1024] __attribute__((aligned(4),section("SRAM3")));
     */
    u32 rx_buf_addr_1;
    union {
        /*£¡no use
         */
        u32 rx_buf_addr_2;
        /*! A pointer to the next rx descriptor.
         */
        struct __ll_mac_rx_descriptor *p_next;
    };
} TYPE_LL_MAC_RX_DESCRIPTOR;

/**
  * @brief GMAC TX descriptor structure type
  */
typedef struct __ll_mac_tx_descriptor {
    /*! When set, indicates that the frame was deferred befor
     *  transmission. Deferring occurs if the carrier is detected when
     *  the transmission is ready to start.  
     *  This bit is valid only when last_descriptor is set.
     */
    u32 deferred            : 1,
    /*! This bit is valid only when last_descriptor is set.
     */
        underflow_err       : 1,
        reserved0           : 1,
    /*! This value is not valid when excessive_collisions is set. This
     *  bit is valid only when last_descriptor is set.
     */
        collision_cnt       : 4,
        reserved1           : 1,
    /*! This bit is valid only when last_descriptor is set.
     */
        excessive_collision : 1,
    /*! This bit is not valid when underflow_err is set. This bit is
     *  valid only when last_descriptor is set.
     */
        late_collision      : 1,
    /*! This bit is valid only when last_descriptor is set.
     */
        no_carrier          : 1,
    /*! This bit is valid only when last_descriptor is set.
     */
        loss_of_carrier     : 1,
        reserved2           : 3,
    /*! This bit is valid only when last_descriptor is set.
     */
        err_summary         : 1,
        reserved3           : 15,
        own                 : 1;
    /*! Indicates the size, in bytes, of memory space used by the first
     *  data buffer. 
     */
    u32 buf_1_size          : 11,
    /*! Useless, please set to 0.
     */
        buf_2_size          : 11,
    /*! This bit is valid only for the setup frames.
     */
        filtering_type_0    : 1,
    /*! When set, automatic byte padding is disabled.
     */
        disable_padding     : 1,
    /*! When set, indicates that the second descriptor's address points
     *  to the next descriptor and not to the data buffer.  
     *  This bit is valid only when end_of_ring is 0.
     */
        second_addr_chained : 1,
    /*! When set, indicates the last descriptor in the descriptors ring.
     */
        end_of_ring         : 1,
        crc_disable         : 1,
    /*! When set, indicates that this is a setup frame descriptor.
     */
        setup_packet        : 1,
    /*! This bit is valid only for the setup frames.
     */
        filtering_type_1    : 1,
        first_descriptor    : 1,
        last_descriptor     : 1,
    /*! Interrupt on completion.  
     *  Setting this flag instructs the GMAC to set CSR5.0 (transmit
     *  interrupt) immediately after processing a current frame.  
     *  This bit is valid when last_descriptor is set, or for a setup
     *  packet.
     */
        int_on_completion   : 1;
    /*! Contains the address of the first data buffer. For the setup
     *  frame this address must be 64 byte aligned. In all other cases
     *  there are no restrictions on buffer alignment.
     *  @note tx_buf_addr_1 are aligned in 4 bytes and can only be located
     *        in SRAM2.  
     *        Statement example:  
     *         static u8 buf[1024] __attribute__((aligned(4),section("SRAM2")));
     */
    u32 tx_buf_addr_1;
    union {
        /*£¡no use
         */
        u32 tx_buf_addr_2;
        /*! A pointer to the next tx descriptor.
         */
        struct __ll_mac_tx_descriptor *p_next;
    };
} TYPE_LL_MAC_TX_DESCRIPTOR;

/**
  * @brief GMAC Hash table setup frame buffer format
  */
typedef struct __ll_mac_setup_hash_table {
    /*! Multicast address hash table
     */
    u32 hash_filter[32];
    u32 reserved[7];
    /* 1 mac address */
    
    /*! MAC address 0bit~15bit
     */
    u32 addr_15_00;
    /*! MAC address 16bit~31bit
     */
    u32 addr_31_16;
    /*! MAC address 32bit~47bit
     */
    u32 addr_47_32;
} TYPE_LL_MAC_SETUP_HASH_TABLE;

/**
  * @brief GMAC Perfect filtering setup frame buffer format
  */
typedef struct __ll_mac_setup_perfect_filter {
    /*! 16 MAC address
     */
    struct {
        /*! MAC address 0bit~15bit
         */
        u32 addr_15_00;
        /*! MAC address 16bit~31bit
         */
        u32 addr_31_16;
        /*! MAC address 32bit~47bit
         */
        u32 addr_47_32;
    } mac_addr[16];
} TYPE_LL_MAC_SETUP_PERFECT_FILTER;

/**
  * @brief GMAC setup frame union type
  */
typedef union __ll_mac_setup_frame {
    /*! perfect filter
     */
    TYPE_LL_MAC_SETUP_PERFECT_FILTER perfect_filter;
    /*! hash table filter
     */
    TYPE_LL_MAC_SETUP_HASH_TABLE     hash_table_filter;
} TYPE_LL_MAC_SETUP_FRAME;

/**
  * @brief GMAC setup frame control structure
  */
typedef struct __ll_mac_setup_frame_ctl {
    /*! GMAC setup frame pointer
     *  @note The address pointed to by the p_setup_frame are aligned in 8
     *        bytes and can only be located in SRAM0/SRAM10/SRAM1.  
     *        Statement example:  
     *         static u8 buf[1024] __attribute__((aligned(8),section("SRAM0")));
     
     */
    TYPE_LL_MAC_SETUP_FRAME   *p_setup_frame;
    /*! rx address filtering mode
     */
    TYPE_ENUM_LL_MAC_FILTER   filter_mode;
    /*! The descriptor used for sending, the user does not need to
     *  configure this.
     */
    TYPE_LL_MAC_TX_DESCRIPTOR *p_tx_descriptor;
} TYPE_LL_MAC_SETUP_FRAME_CTL;

/**
  * @brief GMAC tx control structure type
  */
typedef struct __ll_mac_tx_frame_ctl {
    /*! The frame buf(excluding crc) pointer to be sent
     */
    void                      *frame_buf;
    /*! Frame length to send (excluding crc)
     */
    u32                       frame_len;
    /*! The descriptor used for sending, the user does not need to
     *  configure this.
     */
    TYPE_LL_MAC_TX_DESCRIPTOR *p_tx_descriptor;
} TYPE_LL_MAC_TX_FRAME_CTL;

/**
  * @brief GMAC rx control structure type
  */
typedef struct __ll_mac_rx_frame_ctl {
    /*! Stores the buf pointer of the received frame (including crc).
     */
    void *frame_buf;
    /*! Received frame (including crc) length
     */
    u32  frame_len;
    /*! The status of the received frame is used for query usage, and the
     *  user does not need to pay attention to it.
     */
    u32  status;
} TYPE_LL_MAC_RX_FRAME_CTL;

/**
  * @}
  */

/** @defgroup MAC_LL_Interrupt MAC LL Interrupt Handle function
  * @brief   MAC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MAC  
    Interrupt Handle function.

    how to use?

    The MAC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the MAC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup MAC_LL_Inti_Cfg MAC LL Initialization And Configuration
  * @brief    MAC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MAC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief GMAC register operation
  * @param expression: expression.
  */
#define LL_MAC_REG_OPT(expression) \
do {                               \
    expression;                    \
    __ASM volatile ("nop");        \
} while(0)

/**
  * @brief  GMAC module initialization
  * @param  p_gmac: GMAC module pointer
  * @param  p_init: GMAC initialization structure pointer
  * @retval None
  */
void ll_mac_init(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_INIT *p_init);

/**
  * @brief  GMAC module deinitialization
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_deinit(GMAC_TypeDef *p_gmac);

/**
  * @brief  GMAC interrupt enable setting
  * @param  p_gmac: GMAC module pointer
  * @param  p_cfg : GMAC interrupt enable control structure pointer
  * @retval None
  */
void ll_mac_irq_config(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_IRQ_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup MAC_LL_Data_Transfers MAC LL Data transfers functions
  * @brief    MAC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MAC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Enable GMAC receiving function
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_start(GMAC_TypeDef *p_gmac);

/**
  * @brief  Stop the GMAC module
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_stop(GMAC_TypeDef *p_gmac);

/**
  * @brief  The MII management interface sends data
  * @param  p_gmac  : GMAC module pointer
  * @param  phy_addr: PHY address
  * @param  reg_addr: register address
  * @param  data    : Data to send
  * @retval None
  * @note   Data format symbol IEEE802.3 clause 22
  */
void ll_mac_mdio_write(GMAC_TypeDef *p_gmac, u8 phy_addr, u8 reg_addr, u16 data);

/**
  * @brief  The MII management interface receives data
  * @param  p_gmac  : GMAC module pointer
  * @param  phy_addr: PHY address
  * @param  reg_addr: register address
  * @retval Return the read data
  * @note   Data format symbol IEEE802.3 clause 22
  */
u16 ll_mac_mdio_read(GMAC_TypeDef *p_gmac, u8 phy_addr, u8 reg_addr);

/**
  * @brief  GMAC has received the frame?
  * @param  p_gmac: GMAC module pointer
  * @retval Returns true if received, false if none.
  */
bool ll_mac_has_received_frame(GMAC_TypeDef *p_gmac);

/**
  * @brief  GMAC allows to send a frame?
  * @param  p_gmac: GMAC module pointer
  * @retval Returning true means permission, returning false means that the
  *         TX descriptor has run out and cannot be sent.
  */
bool ll_mac_allow_send_frame(GMAC_TypeDef *p_gmac);

/**
  * @brief  GMAC sends a frame of data
  * @param  p_gmac        : GMAC module pointer
  * @param  p_tx_frame_ctl: tx frame structure pointer
  * @retval None
  */
void ll_mac_send_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_TX_FRAME_CTL *p_tx_frame_ctl);

/**
  * @brief  GMAC setup frame transmission
  * @param  p_gmac           : GMAC module pointer
  * @param  p_setup_frame_ctl: setup frame structure pointer
  * @retval Returning true means that the setup frame is placed in the send
  *         queue. Returning false means that the previous setup frame was
  *         not sent.
  * @note   Wait for a setup frame to be sent before you can continue to
  *         send a setup frame.
  */
bool ll_mac_setup_send_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_SETUP_FRAME_CTL *p_setup_frame_ctl);

/**
  * @brief  GMAC receives a frame of data
  * @param  p_gmac        : GMAC module pointer
  * @param  p_rx_frame_ctl: rx frame structure pointer
  * @retval None
  */
void ll_mac_receive_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_RX_FRAME_CTL *p_rx_frame_ctl);

/**
  * @brief  Get the status of the setup frame
  * @param  p_setup_frame_ctl: setup frame structure pointer
  * @retval tx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_setup_frame_status(TYPE_LL_MAC_SETUP_FRAME_CTL *p_setup_frame_ctl);

/**
  * @brief  Get the status of the send frame
  * @param  p_gmac        : GMAC module pointer
  * @param  p_tx_frame_ctl: tx frame structure pointer
  * @retval tx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_tx_frame_status(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_TX_FRAME_CTL *p_tx_frame_ctl);

/**
  * @brief  Get the status of the received frame
  * @param  p_gmac        : GMAC module pointer
  * @param  p_rx_frame_ctl: rx frame structure pointer
  * @retval rx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_rx_frame_status(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_RX_FRAME_CTL *p_rx_frame_ctl);

/**
  * @brief  Check GMAC normal interrupt summary enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_NIE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_NIE_EN)

/**
  * @brief  Check GMAC abnormal interrupt summary enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_AIE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_AIE_EN)

/**
  * @brief  Check GMAC early receive interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_ERE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_ERE_EN)

/**
  * @brief  Check GMAC general-purpose timer overflow enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_GTE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_GTE_EN)

/**
  * @brief  Check GMAC early transmit interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_ETE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_ETE_EN)

/**
  * @brief  Check GMAC receive stopped enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_RSE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_RSE_EN)

/**
  * @brief  Check GMAC receive buffer unavailable enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_RUE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_RUE_EN)

/**
  * @brief  Check GMAC receive interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_RIE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_RIE_EN)

/**
  * @brief  Check GMAC underflow interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_UNE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_UNE_EN)

/**
  * @brief  Check GMAC transmit buffer unavailable enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_TUE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_TUE_EN)

/**
  * @brief  Check GMAC transmit stopped enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_TSE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_TSE_EN)

/**
  * @brief  Check GMAC transmit interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_MAC_CHECK_TIE_INTERRUPT_ENABLE(p_gmac)   ((p_gmac)->CSR7 & LL_MAC_CSR7_TIE_EN)

/**
  * @brief  Normal interrupt summary enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_nie_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_NIE_EN;
}

/**
  * @brief  Abnormal interrupt summary enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_aie_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_AIE_EN;
}

/**
  * @brief  Early receive interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_ere_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_ERE_EN;
}

/**
  * @brief  General-purpose timer overflow enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_gte_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_GTE_EN;
}

/**
  * @brief  Early transmit interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_ete_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_ETE_EN;
}

/**
  * @brief  Receive stopped enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rse_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_RSE_EN;
}

/**
  * @brief  Receive buffer unavailable enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rue_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_RUE_EN;
}

/**
  * @brief  Receive interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rie_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_RIE_EN;
}

/**
  * @brief  Underflow interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_une_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_UNE_EN;
}

/**
  * @brief  Transmit buffer unavailable enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tue_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_TUE_EN;
}

/**
  * @brief  Transmit stopped enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tse_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_TSE_EN;
}

/**
  * @brief  Transmit interrupt enable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tie_interrupt_enable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 |= LL_MAC_CSR7_TIE_EN;
}

/**
  * @brief  Normal interrupt summary disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_nie_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_NIE_EN;
}

/**
  * @brief  Abnormal interrupt summary disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_aie_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_AIE_EN;
}

/**
  * @brief  Early receive interrupt disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_ere_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_ERE_EN;
}

/**
  * @brief  General-purpose timer overflow disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_gte_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_GTE_EN;
}

/**
  * @brief  Early transmit interrupt disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_ete_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_ETE_EN;
}

/**
  * @brief  Receive stopped disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rse_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_RSE_EN;
}

/**
  * @brief  Receive buffer unavailable disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rue_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_RUE_EN;
}

/**
  * @brief  Receive interrupt disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_rie_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_RIE_EN;
}

/**
  * @brief  Underflow interrupt disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_une_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_UNE_EN;
}

/**
  * @brief  Transmit buffer unavailable disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tue_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_TUE_EN;
}

/**
  * @brief  Transmit stopped disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tse_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_TSE_EN;
}

/**
  * @brief  Transmit interrupt disable
  * @param  p_gmac: GMAC module pointer
  * @retval none
  */
__STATIC_INLINE void ll_mac_tie_interrupt_disable(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR7 &= ~LL_MAC_CSR7_TIE_EN;
}

/**
  * @brief  Check GMAC normal interrupt summary pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC normal interrupt summary pending
  */
#define LL_MAC_GET_NIS_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_NIS_PENDING)

/**
  * @brief  Check GMAC abnormal interrupt summary pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC abnormal interrupt summary pending
  */
#define LL_MAC_GET_AIS_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_AIS_PENDING)

/**
  * @brief  Check GMAC early receive interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC early receive interrupt pending
  */
#define LL_MAC_GET_ERI_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_ERI_PENDING)

/**
  * @brief  Check GMAC general-purpose timer expiration pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC general-purpose timer expiration pending
  */
#define LL_MAC_GET_GTI_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_GTI_PENDING)

/**
  * @brief  Check GMAC early transmit interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC early transmit interrupt pending
  */
#define LL_MAC_GET_ETI_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_ETI_PENDING)

/**
  * @brief  Check GMAC receive process stopped pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC receive process stopped pending
  */
#define LL_MAC_GET_RPS_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_RPS_PENDING)

/**
  * @brief  Check GMAC receive buffer unavailable pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC receive buffer unavailable pending
  */
#define LL_MAC_GET_RU_PENDING(p_gmac)               ((p_gmac)->CSR5 & LL_MAC_CSR5_RU_PENDING)

/**
  * @brief  Check GMAC receive interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC receive interrupt pending
  */
#define LL_MAC_GET_RI_PENDING(p_gmac)               ((p_gmac)->CSR5 & LL_MAC_CSR5_RI_PENDING)

/**
  * @brief  Check GMAC transmit underflow pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC transmit underflow pending
  */
#define LL_MAC_GET_UNF_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_UNF_PENDING)

/**
  * @brief  Check GMAC transmit buffer unavailable pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC transmit buffer unavailable pending
  */
#define LL_MAC_GET_TU_PENDING(p_gmac)               ((p_gmac)->CSR5 & LL_MAC_CSR5_TU_PENDING)

/**
  * @brief  Check GMAC transmit process stopped pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC transmit process stopped pending
  */
#define LL_MAC_GET_TPS_PENDING(p_gmac)              ((p_gmac)->CSR5 & LL_MAC_CSR5_TPS_PENDING)

/**
  * @brief  Check GMAC transmit interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval Return to GMAC transmit interrupt pending
  */
#define LL_MAC_GET_TI_PENDING(p_gmac)               ((p_gmac)->CSR5 & LL_MAC_CSR5_TI_PENDING)

/**
  * @brief  clear GMAC normal interrupt summary pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_nis_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_NIS_PENDING;
}

/**
  * @brief  clear GMAC abnormal interrupt summary pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_ais_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_AIS_PENDING;
}

/**
  * @brief  clear GMAC early receive interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_eri_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_ERI_PENDING;
}

/**
  * @brief  clear GMAC general-purpose timer expiration pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_gti_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_GTI_PENDING;
}

/**
  * @brief  clear GMAC early transmit interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_eti_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_ETI_PENDING;
}

/**
  * @brief  clear GMAC receive process stopped pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_rps_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_RPS_PENDING;
}

/**
  * @brief  clear GMAC receive buffer unavailable pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_ru_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_RU_PENDING;
}

/**
  * @brief  clear GMAC receive interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_ri_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_RI_PENDING;
}

/**
  * @brief  clear GMAC transmit underflow pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_unf_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_UNF_PENDING;
}

/**
  * @brief  clear GMAC transmit buffer unavailable pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_tu_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_TU_PENDING;
}

/**
  * @brief  clear GMAC transmit process stopped pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_tps_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_TPS_PENDING;
}

/**
  * @brief  clear GMAC transmit interrupt pending
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
__STATIC_INLINE void ll_mac_clear_ti_pending(GMAC_TypeDef *p_gmac) {
    p_gmac->CSR5 = LL_MAC_CSR5_TI_PENDING;
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif //__TX_PHE_LL_MAC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
