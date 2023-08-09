
/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_dmac.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DMAC LL firmware functions.
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
#ifndef __TX_PHE_LL_DMAC_H
#define __TX_PHE_LL_DMAC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup dmac_interface_gr DMAC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup DMAC_LL_Driver DMAC LL Driver
  * @ingroup  dmac_interface_gr
  * @brief Mainly the driver part of the DMAC module, which includes \b DMAC \b Register 
  * \b Constants, \b DMAC \b Exported \b Constants, \b DMAC \b Exported \b Struct, \b DMAC
  * \b Data \b transfers \b functions, \b DMAC \b Initialization \b and \b DMAC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup DMAC_LL_Register_Constants DMAC LL Register Constants
  * @ingroup  DMAC_LL_Driver
  * @brief    DMAC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the DMAC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the DMAC register, mainly for convenience. Understand the 
    configuration of the DMAC.
    
@endverbatim
  *
  * @{
  */

/***** DW_PARAMS Register *****/
/*! number of channels
 */
#define LL_DW_PARAMS_NR_CHAN               4
/*! number of AHB masters
 */
#define LL_DW_PARAMS_NR_MASTER             2


/***** DWC_PARAMS Register *****/
/*! multi block transfer
 */
#define LL_DWC_PARAMS_MBLK_EN              11


/***** CTLx Register *****/
/*! irqs enabled
 */
#define LL_DWC_CTLL_INT_EN                 (1UL << 0)
/*! The size of each element of destination transmission.
 */
#define LL_DWC_CTLL_DST_WIDTH(n)           (((n)&0x7) << 1)
/*! The size of each element of source transmission.
 */
#define LL_DWC_CTLL_SRC_WIDTH(n)           (((n)&0x7) << 4)
/*! Indicates destination address how to change on every source transfer.
 */
#define LL_DWC_CTLL_DST_DIR(n)             (((n)&0x3) << 7)
/*! Indicates source address how to change on every source transfer.
 */
#define LL_DWC_CTLL_SRC_DIR(n)             (((n)&0x3) << 9)
/*! Number of data items to be written to de destination every time. 
 */
#define LL_DWC_CTLL_DST_MSIZE(n)           (((n)&0x7) << 11)
/*! Number of data items to be read from the source every time. 
 */
#define LL_DWC_CTLL_SRC_MSIZE(n)           (((n)&0x7) << 14)
/*! src gather
 */
#define LL_DWC_CTLL_S_GATH_EN              (1UL << 17)
/* dst scatter 
 */
//#define LL_DWC_CTLL_D_SCAT_EN            (1UL << 18)
/*! Transfer type and flow control.
 */
#define LL_DWC_CTLL_FC(n)                  (((n)&0x7) << 20)
/*! dst master select : 1 for memory, 0 for peripherals
 */
#define LL_DWC_CTLL_DMS(n)                 (((n)&0x3) << 23)
/*! src master select : 1 for memory, 0 for peripherals
 */
#define LL_DWC_CTLL_SMS(n)                 (((n)&0x3) << 25)
/* dest block chain 
 */
//#define LL_DWC_CTLL_LLP_D_EN             (1UL << 27)
/* src block chain 
 */
//#define LL_DWC_CTLL_LLP_S_EN             (1UL << 28)


/***** CTL_HI Register *****/
/*! Block transfer done
 */
#define LL_DWC_CTLH_DONE                   (0x00001000)
/*! Block transfer size. The max block size is 4095.
 */
#define LL_DWC_CTLH_BLOCK_TS_MASK          (0x00000FFF)


/***** CFG_LO Register *****/
/*! Channel priority mask
 */
#define LL_DWC_CFGL_CH_PRIOR_MASK          (0x7UL << 5)
/*! Channel priority
 */
#define LL_DWC_CFGL_CH_PRIOR(n)            (((n)&0x7) << 5)
/*! Channle Suspend. Suspends all DMA data transfers from the source until this
 *  bit is cleared.
 */
#define LL_DWC_CFGL_CH_SUSP                (1UL << 8)
/*! Indicates if there is data left in the channel FIFO.
 */
#define LL_DWC_CFGL_FIFO_EMPTY             (1UL << 9)
/*! Destination software or hardware handshaking select.
 */
#define LL_DWC_CFGL_HS_DST                 (1UL << 10)
/*! Source software or hardware handshaking select.
 */
#define LL_DWC_CFGL_HS_SRC                 (1UL << 11)
//#define LL_DWC_CFGL_LOCK_CH_LEVEL(n)     (((n)&0x3) << 12)
//#define LL_DWC_CFGL_LOCK_BUS_LEVEL(n)    (((n)&0x3) << 14)
//#define LL_DWC_CFGL_LOCK_CH              (1UL << 16)
//#define LL_DWC_CFGL_LOCK_BUS             (1UL << 17)
/*! Destination handshaking interface polarity.
 */
#define LL_DWC_CFGL_HS_DST_POL             (1UL << 18)
/*! Source handshaking interface polarity.
 */
#define LL_DWC_CFGL_HS_SRC_POL             (1UL << 19)
/*! Maximum AMBA burst length that is used for DMA transfers on this channel.
 */
#define LL_DWC_CFGL_MAX_BURST(n)           (((n)&0x2FF) << 20)
/*! Automatic source reload.
 */
#define LL_DWC_CFGL_RELOAD_SAR             (1UL << 30)
/*! Automatic desination reload.
 */
#define LL_DWC_CFGL_RELOAD_DAR             (1UL << 31)


/***** CFG_HI Register *****/
/*! Flow control mode.
 */
#define LL_DWC_CFGH_FCMODE                 (1UL << 0)
/*! FIFO mode select.
 */
#define LL_DWC_CFGH_FIFO_MODE              (1UL << 1)
/*! Protection control.
 */
#define LL_DWC_CFGH_PROTCTL(n)             (((n)&0x7) << 2)
//#define LL_DWC_CFGH_DS_UPD_EN            (1UL << 5)
//#define LL_DWC_CFGH_SS_UPD_EN            (1UL << 6)
/*! Assigns a hardware handshaking interface to the source of channel.
 */
#define LL_DWC_CFGH_SRC_PER(n)             (((n)&0xF) << 7)
/*! Assigns a hardware handshaking interface to the destination of channel.
 */
#define LL_DWC_CFGH_DST_PER(n)             (((n)&0xF) << 11)


/***** SGR Register *****/
/*! Source gather interval.
 */
#define LL_DWC_SGR_SGI(n)                  (((n)&0xFFFFF) << 0)
/*! Source gather count.
 */
#define LL_DWC_SGR_SGC(n)                  (((n)&0xF) << 20)


/***** DSR Register *****/
//#define LL_DWC_DSR_DSI(x)                (((n)&0xFFFFF) << 0)
//#define LL_DWC_DSR_DSC(x)                (((n)&0xF) << 20)


/***** DmaCfgReg Register *****/
/*! dmac enable.
 */
#define LL_DW_CFG_DMA_EN                   (1UL << 0)

/***** ChEnRegL/MaskTfrL/MaskErrL/MaskDstTranL/MaskSrcTranL/MaskBlockL/ Register *****/
/*! Source gather count.
 */
#define LL_DMAC_WRITE_EN(n)                (BIT(n+8) | BIT(n))
/*! Source gather count.
 */
#define LL_DMAC_WRITE_DIS(n)               (BIT(n+8))

/**
  * @}
  */

/** @defgroup DMAC_LL_Exported_Constants DMAC LL Exported Constants
  * @ingroup  DMAC_LL_Driver
  * @brief    DMAC LL external constant definition
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

/*! max dmac hardware channel
 */
#define LL_MAX_DMAC_CHN                 4
/*! max dmac hardware channel mask
 */
#define LL_MAX_DMAC_CHN_MASK            0xF


/***** LL API *****/


/***** LL API AND DRIVER API *****/

/**
  * @brief Number of data items to be transferred(of width TR_WIDTH).
  */
typedef enum {
    /*! damc data width : 1byte
     */
    LL_DW_DMAC_MSIZE_1,
    /*! damc data width : 4byte
     */

    LL_DW_DMAC_MSIZE_4,
    /*! damc data width : 8byte
     */
    LL_DW_DMAC_MSIZE_8,
    // The following MSIZE is not supported.
//    DW_DMAC_MSIZE_16,
//    DW_DMAC_MSIZE_32,
//    DW_DMAC_MSIZE_64,
//    DW_DMAC_MSIZE_128,
//    DW_DMAC_MSIZE_256,
} TYPE_ENUM_LL_DMAC_MSIZE;

/**
  * @brief The size of each element of destination transmission. The unit is bit.
  */
typedef enum {
    /*! damc xfer width : 8 bit
     */
    LL_DW_DMAC_TR_8,
    /*! damc xfer width : 16bit
     */
    LL_DW_DMAC_TR_16,
    /*! damc xfer width : 32bit
     */
    LL_DW_DMAC_TR_32,
    // The following TR_WIDTH is not supported.
//    DW_DMAC_TR_64,
//    DW_DMAC_TR_128,
//    DW_DMAC_TR_256,
} TYPE_ENUM_LL_DMAC_TR_WIDTH;

/**
  * @brief Indicates destination address how to change on every source transfer.
  */
typedef enum {
    /*! Increment
     */
    LL_DW_DMAC_ADDR_INC,
    /*! Decrement
     */
    LL_DW_DMAC_ADDR_DEC,
    /*! No change
     */
    LL_DW_DMAC_ADDR_NO_CHANGE,
} TYPE_ENUM_LL_DMAC_ADDR_DIR;

/**
  * @brief flow controller
@verbatim   
-------------------------------------------------------------------------------
CTLL.TT_FC Field        Transfer Type                   Flow Controller    
-------------------------------------------------------------------------------
      000               Memory to Memory                DW_anb_dmac
      001               Memory to Peripheral            DW_anb_dmac
      010               Peripheral to Memory            DW_anb_dmac
      011               Peripheral to Peripheral        DW_anb_dmac
      100               Peripheral to Memory            Peripheral
      101               Peripheral to Peripheral        Source Peripheral
      110               Memory to Peripheral            Peripheral
      111               Peripheral to Peripheral        Destination Peripheral
@endverbatim
  */
typedef enum {
    /*! Memory to Memory
     */
    LL_DW_DMA_FC_D_M2M,
    /*! Memory to Peripheral
     */
    LL_DW_DMA_FC_D_M2P,
    /*! Peripheral to Memory
     */
    LL_DW_DMA_FC_D_P2M,
    /*! Peripheral to Peripheral
     */
    LL_DW_DMA_FC_D_P2P,
    /*! EFLASH : Memory to Memory
     */
    LL_DW_DMA_FC_D_M2M_EFLASH,
    // The following flow control is not supported.
//  DW_DMA_FC_P_P2M,
//  DW_DMA_FC_SP_P2P,
//  DW_DMA_FC_P_M2P,
//  DW_DMA_FC_DP_P2P,
} TYPE_ENUM_LL_DMAC_FC;

/**
  * @brief hardware handshaking interface(DMAC req channel)
  */
typedef enum {
    /*! Peripheral channel : MEMORY
     */
    LL_DMA_CH_MEMORY    = 0,
    /*! Peripheral channel : I2C0_TX
     */
    LL_DMA_CH_I2C0_TX   = 0,
    /*! Peripheral channel : I2C0_RX 
     */
    LL_DMA_CH_I2C0_RX   = 1,
    /*! Peripheral channel : I2C1_TX 
     */
    LL_DMA_CH_I2C1_TX   = 2,
    /*! Peripheral channel : I2C1_RX 
     */
    LL_DMA_CH_I2C1_RX   = 3,
    /*! Peripheral channel : SPI0_TX
     */
    LL_DMA_CH_SPI0_TX   = 4,
    /*! Peripheral channel : SPI0_RX 
     */
    LL_DMA_CH_SPI0_RX   = 5,
    /*! Peripheral channel : SPI1_TX 
     */
    LL_DMA_CH_SPI1_TX   = 6,
    /*! Peripheral channel : SPI1_RX 
     */
    LL_DMA_CH_SPI1_RX   = 7,
    /*! Peripheral channel : USART0_TX
     */
    LL_DMA_CH_USART0_TX = 8,
    /*! Peripheral channel : USART0_RX
     */
    LL_DMA_CH_USART0_RX = 9,
    /*! Peripheral channel : USART1_TX 
     */
    LL_DMA_CH_USART1_TX = 10,
    /*! Peripheral channel : USART1_RX 
     */
    LL_DMA_CH_USART1_RX = 11,
    /*! Peripheral channel : USART2_TX 
     */
    LL_DMA_CH_USART2_TX = 12,
    /*! Peripheral channel : USART2_RX
     */
    LL_DMA_CH_USART2_RX = 13,
} TYPE_ENUM_LL_DMAC_PER_INTERFACE;


/**
  * @}
  */

/** @defgroup DMAC_LL_Exported_Struct DMAC LL Exported Struct
  * @ingroup  DMAC_LL_Driver
  * @brief    DMAC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the DMAC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_dmac_init. Function, you can configure the DMAC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief Peripheral attributes
  */
typedef struct __ll_dmac_flow_attr {
    /*! DMAC Peripheral address
     *  @note The addr is aligned in 4bytes and can only be located in
     *        EFLASH/SRAM0/SRAM10/SRAM1/SRAM2/SRAM3.  
     *        Statement example:  
     *          static u8 buf[1024] __attribute__(section("SRAM0"));
     */
    u32                             addr;
    /*! DMAC Peripheral address direction : TYPE_ENUM_LL_DMAC_ADDR_DIR
     */
    TYPE_ENUM_LL_DMAC_ADDR_DIR      addr_dir;
    /*! DMAC Peripheral TYPE_ENUM_LL_DMAC_PER_INTERFACE
     */
    TYPE_ENUM_LL_DMAC_PER_INTERFACE interface_chn;
} TYPE_LL_DMAC_FLOW_ATTR;

/**
  * @brief DMAC interrupt configuration type.
  */
typedef struct __ll_dmac_irq_cfg {
    /*! DMAC channel number 
     */
    u32  chn;
    /*! DMAC interrupt enable
     */
    bool intr_en;
    /*! true means mask, false means unmask.
     */
    bool xfer_cplt_intr_dis;
    /*! true means mask, false means unmask.
     */
    bool block_intr_dis;
    /*! true means mask, false means unmask. 
     */
    bool src_xfer_cplt_intr_dis;
    /*! true means mask, false means unmask.
     */
    bool dst_xfer_cplt_intr_dis;
    /*! true means mask, false means unmask. 
     */
    bool xfer_err_intr_dis;
} TYPE_LL_DMAC_IRQ_CFG;

/**
  * @brief DMAC configure the structure type.
  */
typedef struct __ll_dmac_cfg {
    /*! DMAC channel number 
     */
    u32                         chn;
    /*! Source Peripheral attributes
     */
    TYPE_LL_DMAC_FLOW_ATTR      src;
    /*! Destination Peripheral attributes 
     */
    TYPE_LL_DMAC_FLOW_ATTR      dst;
    /*! DMAC flow control 
     */
    TYPE_ENUM_LL_DMAC_FC        flow_ctrl;
    /*! The width of each element of the DMA.
     */
    TYPE_ENUM_LL_DMAC_TR_WIDTH  element_per_width;
    /*! The number of data transmissions.  Must not be greater than 4095.
     */
    u32                         element_num;
} TYPE_LL_DMAC_CFG;

/**
  * @brief DMAC initialize stuctor.
  */
typedef struct __ll_dmac_init {
    u8  reserved;
} TYPE_LL_DMAC_INIT;

/**
  * @}
  */

/** @defgroup DMAC_LL_Interrupt DMAC LL Interrupt Handle function
  * @brief   DMAC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DMAC  
    Interrupt Handle function.

    how to use?

    The DMAC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the DMAC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup DMAC_LL_Inti_Cfg DMAC LL Initialization And Configuration
  * @brief    DMAC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DMAC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  ll_dmac_init
  * @param  p_dmac : pointer to the hardware DMAC_TypeDef
  * @param  p_init : pointer to the init stuct TYPE_LL_DMAC_INIT
  * @retval None
  */
void ll_dmac_init(DMAC_TypeDef * p_dmac, TYPE_LL_DMAC_INIT *p_init);

/**
  * @brief  ll_dmac_deinit
  * @param  p_dmac : pointer to the hardware DMAC_TypeDef
  * @retval None
  */
void ll_dmac_deinit(DMAC_TypeDef * p_dmac);

/**
  * @brief  DMAC channel interrupt configuration.
  * @param  p_dmac: DMAC module pointer.
  * @param  p_cfg :  DMAC interrupt configuration pointer.
  * @retval None
  */
void ll_dmac_irq_config(DMAC_TypeDef *p_dmac, TYPE_LL_DMAC_IRQ_CFG *p_cfg);

/**
  * @brief  DMAC module configuration function.
  * @param  p_dmac: DMAC module pointer.DMAC_TypeDef
  * @param  p_cfg :  DMAC configuration structure pointer.TYPE_LL_DMAC_CFG
  * @retval None
  */
void ll_dmac_config(DMAC_TypeDef *p_dmac, TYPE_LL_DMAC_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup DMAC_LL_Data_Transfers DMAC LL Data transfers functions
  * @brief    DMAC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DMAC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  Start a specific DMA channel.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval None
  */
void ll_dmac_start(DMAC_TypeDef *p_dmac, u32 chn);

/**
  * @brief  Disable a specific DMA channel.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval None
  */
void ll_dmac_stop(DMAC_TypeDef *p_dmac, u32 chn);

/**
  * @brief  Wait for the DMAC channel to end the operation.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval Returns true if the DMAC channel is normal and false if it
  *         represents an error in the DMAC channel.
  */
bool ll_dmac_wait_completed(DMAC_TypeDef *p_dmac, u32 chn);

/**
  * @brief  DMAC's gather function settings
  * @param  p_dmac: DMAC module pointer
  * @param  chn   : DMAC channel number
  * @param  sgc   : The amount of data continuously acquired
  * @param  sgi   : Interval of data
  * @retval None
  * @note   The amount of data that sgc and sgi together is taken as a unit.
  *         The sgi data will be discarded.
  */
void ll_dmac_src_gather_config(DMAC_TypeDef *p_dmac, u32 chn, u32 sgc, u32 sgi);

/**
  * @brief  Return the DMA xfered length when dma is complete or abort
  * @param  p_dmac: DMAC module pointer
  * @param  chn   : DMAC channel number
  * @retval DMA xfered length, 0 will set when both src&dst dma address both not change
  */
uint32_t ll_dma_get_xfer_len(DMAC_TypeDef *p_dmac, u8 chn);

/**
  * @brief  Whether a DMAC channel is idle.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval Returns true for idle and false for occupied.
  */
__STATIC_INLINE bool ll_dmac_is_channel_free(DMAC_TypeDef *p_dmac, u32 chn) {
    return p_dmac->ChEnRegL & BIT(chn) ? false : true;
}

/**
 * @brief  ll_dmac_xfer_interrupt_enable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_xfer_interrupt_enable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskTfrL = LL_DMAC_WRITE_EN(chn);
}

/**
 * @brief  ll_dmac_xfer_interrupt_disable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_xfer_interrupt_disable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskTfrL = LL_DMAC_WRITE_DIS(chn);
}

/**
 * @brief  LL_DMAC_CHECK_XFER_INTERRUPT_ENABLE
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval interrupt_enable 
 */
#define LL_DMAC_CHECK_XFER_INTERRUPT_ENABLE(p_dmac, chn)    ((p_dmac)->MaskTfrL & BIT(chn))

/**
 * @brief  LL_DMAC_GET_XFER_PENDING
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval pending 
 */
#define LL_DMAC_GET_XFER_PENDING(p_dmac, chn)               ((p_dmac)->RawTfrL & BIT(chn))

/**
 * @brief  ll_dmac_clear_xfer_pending
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_clear_xfer_pending(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->ClearTfrL = BIT(chn);
}

/**
 * @brief  ll_dmac_block_interrupt_enable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_block_interrupt_enable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskBlockL = LL_DMAC_WRITE_EN(chn);
}

/**
 * @brief  ll_dmac_block_interrupt_disable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_block_interrupt_disable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskBlockL = LL_DMAC_WRITE_DIS(chn);
}

/**
 * @brief  LL_DMAC_CHECK_BLOCK_INTERRUPT_ENABLE
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval interrupt_enable 
 */
#define LL_DMAC_CHECK_BLOCK_INTERRUPT_ENABLE(p_dmac, chn)   ((p_dmac)->MaskBlockL & BIT(chn))

/**
 * @brief  LL_DMAC_GET_BLOCK_PENDING
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval pending
 */
#define LL_DMAC_GET_BLOCK_PENDING(p_dmac, chn)              ((p_dmac)->RawBlockL & BIT(chn))

/**
 * @brief  ll_dmac_clear_block_pending
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_clear_block_pending(DMAC_TypeDef *p_dmac, u32 chn) {    
    p_dmac->ClearBlockL = BIT(chn);
}

/**
 * @brief  ll_dmac_src_xfer_interrupt_enable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_src_xfer_interrupt_enable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskSrcTranL = LL_DMAC_WRITE_EN(chn);
}

/**
 * @brief  ll_dmac_src_xfer_interrupt_disable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_src_xfer_interrupt_disable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskSrcTranL = LL_DMAC_WRITE_DIS(chn);
}

/**
 * @brief  LL_DMAC_CHECK_SRC_XFER_INTERRUPT_ENABLE
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval interrupt_enable 
 */
#define LL_DMAC_CHECK_SRC_XFER_INTERRUPT_ENABLE(p_dmac, chn)    ((p_dmac)->MaskSrcTranL & BIT(chn))

/**
 * @brief  LL_DMAC_GET_SRC_XFER_PENDING
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval pending
 */
#define LL_DMAC_GET_SRC_XFER_PENDING(p_dmac, chn)               ((p_dmac)->RawSrcTranL & BIT(chn))

/**
 * @brief  ll_dmac_clear_src_xfer_pending
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_clear_src_xfer_pending(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->ClearSrcTranL = BIT(chn);
}

/**
 * @brief  ll_dmac_dst_xfer_interrupt_enable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_dst_xfer_interrupt_enable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskDstTranL = LL_DMAC_WRITE_EN(chn);
}

/**
 * @brief  ll_dmac_dst_xfer_interrupt_disable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_dst_xfer_interrupt_disable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskDstTranL = LL_DMAC_WRITE_DIS(chn);
}

/**
 * @brief  LL_DMAC_CHECK_DST_XFER_INTERRUPT_ENABLE
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval interrupt_enable 
 */
#define LL_DMAC_CHECK_DST_XFER_INTERRUPT_ENABLE(p_dmac, chn)    ((p_dmac)->MaskDstTranL & BIT(chn))

/**
 * @brief  LL_DMAC_GET_DST_XFER_PENDING
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval pending
 */
#define LL_DMAC_GET_DST_XFER_PENDING(p_dmac, chn)               ((p_dmac)->RawDstTranL & BIT(chn))

/**
 * @brief  ll_dmac_clear_dst_xfer_pending
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_clear_dst_xfer_pending(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->ClearDstTranL = BIT(chn);
}

/**
 * @brief  ll_dmac_xfer_err_interrupt_enable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_xfer_err_interrupt_enable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskErrL = LL_DMAC_WRITE_EN(chn);
}

/**
 * @brief  ll_dmac_xfer_err_interrupt_disable
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_xfer_err_interrupt_disable(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->MaskErrL = LL_DMAC_WRITE_DIS(chn);
}

/**
 * @brief  LL_DMAC_CHECK_XFER_ERR_INTERRUPT_ENABLE
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval interrupt_enable 
 */
#define LL_DMAC_CHECK_XFER_ERR_INTERRUPT_ENABLE(p_dmac, chn)    ((p_dmac)->MaskErrL & BIT(chn))

/**
 * @brief  LL_DMAC_GET_XFER_ERR_PENDING
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval pending
 */
#define LL_DMAC_GET_XFER_ERR_PENDING(p_dmac, chn)               ((p_dmac)->RawErrL & BIT(chn))

/**
 * @brief  ll_dmac_clear_xfer_err_pending
 * @param  p_dmac: DMAC module pointer. DMAC_TypeDef
 * @param  chn   : DMAC channel number.
 * @retval none
 */
__STATIC_INLINE void ll_dmac_clear_xfer_err_pending(DMAC_TypeDef *p_dmac, u32 chn) {
    p_dmac->ClearErrL = BIT(chn);
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

#endif //__TX_PHE_LL_DMAC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
