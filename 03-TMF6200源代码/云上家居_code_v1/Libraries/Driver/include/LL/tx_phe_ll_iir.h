/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_iir.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the IIR LL firmware functions.
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
#ifndef __TX_PHE_LL_IIR_H
#define __TX_PHE_LL_IIR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup iir_interface_gr IIR Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup IIR_LL_Driver IIR LL Driver
  * @ingroup  iir_interface_gr
  * @brief Mainly the driver part of the IIR module, which includes \b IIR \b Register 
  * \b Constants, \b IIR \b Exported \b Constants, \b IIR \b Exported \b Struct, \b IIR
  * \b Data \b transfers \b functions, \b IIR \b Initialization \b and \b IIR \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 
/** @defgroup IIR_LL_Register_Constants IIR LL Register Constants
  * @ingroup  IIR_LL_Driver
  * @brief    IIR LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the IIR 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the IIR register, mainly for convenience. Understand the 
    configuration of the IIR.
    
@endverbatim
  *
  * @{
  */

/***** IIR_CH_ENA0 *****/
/*! IIR's channel enable bit, each 1 bit represents a channel, representing the 0th 
 *  to 31st channel. 
 */
#define LL_IIR_CH_ENA0(n)                         (((n)&0xFFFFFFFF) << 0)


/***** IIR_CPU_KST0 *****/
/*! The triggering of IIR is enabled. Each 1 bit represents the trigger of one channel, 
 *  representing the 0th to 31st channel. 
 */
#define LL_IIR_CPU_KST0(n)                        (((n)&0xFFFFFFFF) << 0)


/***** IIR_CFG_ADDR *****/
/*! The start address of the IIR configuration information storage, such as TXF6200 is 
 *  fixed in SRAM11 (0x20019000). 
 */
#define LL_IIR_CFG_ADDR(n)                        (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_ENA0 *****/
/*! The interrupt of the IIR's 0th to 31st channel is enabled. 
 */
#define LL_IIR_INT_ENA0(n)                        (((n)&0xFFFFFFFF) << 0)


/***** IIR_HALF_PND0 *****/
/*! When the output BUFF of the Nth channel of the IIR has filled half of the data, the 
 *  completion flag is displayed, representing the 0th to 31st channel, and the write 1 
 *  clear pending. 
 */
#define LL_IIR_HALF_PND0(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_FULL_PND0 *****/
/*! When the output BUFF of the Nth channel of the IIR is already filled with data, the 
 *  completion flag is displayed, representing the 0th to 31st channel, and the write 1 
 *  clear.
 */
#define LL_IIR_FULL_PND0(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_FILT_PND0 *****/
/*! When the Nth channel of the IIR completes the filtering, the completion flag is displayed, 
 *  representing the 0th to 31st channel, and the 1 clearing is performed. 
 */
#define LL_IIR_FILT_PND0(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_SRCL0 *****/
/*! When each bit represents the interrupt source selection of one channel, it represents 
 *  the 0th to 31st channel, and is used together with SRCH0, the format is {SRCH0, SRCL0}. 
 */
#define LL_IIR_INT_SRCL0(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_SRCH0 *****/
/*! When every 2 bits represents the interrupt source selection of one channel, it represents 
 *  the 0th to 31st channel and is used together with SRCH0 in the format of {SRCH0, SRCL0}. 
 */
#define LL_IIR_INT_SRCH0(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_DATA_OUT *****/
/*! When the IIR is only working all the way, the calculation result of the module is output. 
 */
#define LL_IIR_DATA_OUT(n)                        ((n)&0xFFFFFFFF) << 0)


/***** IIR_EVSYS_ENA0 *****/
/*! IIR's evsys channel enable bit, each 1 bit represents a channel, representing the 0th to 
 *  31st channel, used in conjunction with the event trigger, each event triggers an IIR, completes 
 *  an N channel filtering (mainly see Which way of this register is enabled). 
 */
#define LL_IIR_EVSYS_ENA0(n)                      (((n)&0xFFFFFFFF) << 0)


/***** IIR_CH_ENA1 *****/
/*! IIR's channel enable bit, each 1 bit represents a channel, representing the 32th to 63st channel. 
 */
#define LL_IIR_CH_ENA1(n)                         (((n)&0xFFFFFFFF) << 0)


/***** IIR_CPU_KST1 *****/
/*! The triggering of IIR is enabled. Each 1 bit represents the trigger of one channel, representing 
 *  the 32th to 63st channel. 
 */
#define LL_IIR_CPU_KST1(n)                        (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_ENA1 *****/
/*! The interrupt of the IIR's 32th to 63st channel is enabled. 
 */
#define LL_IIR_INT_ENA1(n)                        (((n)&0xFFFFFFFF) << 0)


/***** IIR_HALF_PND1 *****/
/*! When the output BUFF of the Nth channel of the IIR has filled half of the data, the 
 *  completion flag is displayed, representing the 32th to 63st channel, and the write 1 
 *  clear pending. 
 */
#define LL_IIR_HALF_PND1(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_FULL_PND1 *****/
/*! When the output BUFF of the Nth channel of the IIR is already filled with data, the 
 *  completion flag is displayed, representing the 32th to 63st channel, and the write 1 
 *  clear.
 */
#define LL_IIR_FULL_PND1(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_FILT_PND1 *****/
/*! When the Nth channel of the IIR completes the filtering, the completion flag is displayed, 
 *  representing the 32th to 63st channel, and the 1 clearing is performed. 
 */
#define LL_IIR_FILT_PND1(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_SRCL1 *****/
/*! When each bit represents the interrupt source selection of one channel, it represents 
 *  the 32th to 63st channel, and is used together with SRCH0, the format is {SRCH0, SRCL0}. 
 */
#define LL_IIR_INT_SRCL1(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_INT_SRCH1 *****/
/*! When every 2 bits represents the interrupt source selection of one channel, it represents 
 *  the 32th to 63st channel and is used together with SRCH0 in the format of {SRCH0, SRCL0}. 
 */
#define LL_IIR_INT_SRCH1(n)                       (((n)&0xFFFFFFFF) << 0)


/***** IIR_EVSYS_ENA1 *****/
/*! IIR's evsys channel enable bit, each 1 bit represents a channel, representing the 32th to 
 *  63st channel, used in conjunction with the event trigger, each event triggers an IIR, completes 
 *  an N channel filtering (mainly see Which way of this register is enabled). 
 */
#define LL_IIR_EVSYS_ENA1(n)                      (((n)&0xFFFFFFFF) << 0)

/**
  * @}
  */

/** @defgroup IIR_LL_Exported_Constants IIR LL Exported Constants
  * @ingroup  IIR_LL_Driver
  * @brief    IIR LL external constant definition
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
  * @brief Enumeration constant for IIR finished pending status
  */
typedef enum {
    /*! IIR idle flag macro definition.
     */
    LL_IIR_PNG_IDLE = 0,
    /*! IIR completes the filter flag macro definition.
     */
    LL_IIR_PNG_FILT,
    /*! IIR completes the definition of the half mark macro.
     */
    LL_IIR_PNG_HALF,
    /*! IIR completes the definition of the full mark macro.
     */
    LL_IIR_PNG_FULL,
} TYPE_ENUM_LL_IIR_PNG;

/**
  * @brief Enumeration constant for The IIR interrupt source triggers the selection
  */
typedef enum {
    /*! Each time the IIR is filtered, an interrupt is triggered.
     */
    LL_IIR_INTR_SRC_FILT = 0,
    /*! IIR triggers an interrupt when it calculates half of the output buffer.
     */
    LL_IIR_INTR_SRC_HALF,
    /*! IIR triggers an interrupt when it calculates that the output buffer is full
     */
    LL_IIR_INTR_SRC_FULL,
    /*! IIR triggers an interrupt when it calculates that the output buffer is full and half. 
     */
    LL_IIR_INTR_SRC_HALF_FULL,
} TYPE_ENUM_LL_IIR_INTR_SRC_SEL;


/**
  * @}
  */

/** @defgroup IIR_LL_Exported_Struct IIR LL Exported Struct
  * @ingroup  IIR_LL_Driver
  * @brief    IIR LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the IIR registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_iir_init. Function, you can configure the IIR module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief IIR interrupt low layer configuration structure
  */
typedef struct __ll_iir_irq_cfg {  
    /*! There are 8 sets of paths for each set of IIR, and chn is the one that 
     *  is selected for calculation, ranging from 0 to 7. 
     */
    u8                            chn;
    /*! Select the trigger source for the IIR module interrupt. 
     */
    TYPE_ENUM_LL_IIR_INTR_SRC_SEL intr_src_sel; 
} TYPE_LL_IIR_IRQ_CFG;

/**
  * @brief Channel configuration information, consistent with the definition
  *        of spec.
  */
typedef struct __ll_iir_param_cfg {
    /*! Output data downsampling count. This value represents a point written
     *  to the output buf for each number of points calculated.
     */
    u8  sample_rate;
    /*! Down-sampling status counter, initialized with a value of 0.
     */
    u8  sample_rate_sta;
    /*! DMA enable bit
     */
    u16 dma_en                  : 1,
        reserved_0              : 11,
    /*! The number of right-shifted output results, 5bit unsigned number. Here
     *  is the low 4bit.
     */
        odata_sft_bit_low       : 4;
    
    /*! The number of right-shifted output results, 5bit unsigned number. Here
     *  is the highest bit.
     */
    u32 odata_sft_bit_high      : 1,
        reserved_1              : 7,
    /*! Accumulator output right shift number of bits.
     */
        feedback_sft_bit        : 5,
        reserved_2              : 19;
        
    /*! IIR input data buf start address. All input IIR data requirements are
     *  16-bit signed numbers. Each IIR module has a total of 64 channels, which 
     *  must be 2Byte aligned. 
     *  @note IIR0's dma_src_addr can only be located in SRAM1.  
     *        IIR1's dma_src_addr can only be located in SRAM2.  
     *        IIR2's dma_src_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 iir0_ibuf[1024] __attribute__((section("SRAM1")));
     */
    u32 dma_src_addr;

    /*! IIR output data buf start address. All output IIR data requirements are
     *  16-bit signed numbers. Each IIR module has a total of 64 channels, which 
     *  must be 2Byte aligned. 
     *  @note IIR0's dma_dst_addr can only be located in SRAM1.  
     *        IIR1's dma_dst_addr can only be located in SRAM2.  
     *        IIR2's dma_dst_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 iir0_obuf[1024] __attribute__((section("SRAM1")));
     */
    u32 dma_dst_addr;
    /*! The IIR coefficient structure buf start address. Each IIR module has a
     *  total of 64 channels, which must be 2Byte aligned. 
     *  @note IIR0's coef_start_addr can only be located in SRAM14.  
     *        IIR1's coef_start_addr can only be located in SRAM15.  
     *        IIR2's coef_start_addr can only be located in SRAM16.  
     *        Statement example:  
     *        static TYPE_LL_IIR_COEFF_CFG iir0_coeff_cfg __attribute__((section("SRAM14")));
     */
    u32 coef_start_addr;
    /*! IIR input data buf length.
     */
    u16 dma_src_points;
    /*! IIR output data buf length.
     */
    u16 dma_dst_points;
    /*! IIR calculates the starting address of the fetched data. The address
     *  must be inside the IIR input buf.
     */
    u32 dma_src_index;
    /*! The starting address of the output of the IIR calculation result. The
     *  address must be inside IIR output buf.
     */
    u32 dma_dst_index;
} TYPE_LL_IIR_PARAM_CFG;

/**
  * @brief coefficient information, consistent with the definition of spec.  
  *        All IIR coefficients are signed 16-bit fixed-point numbers. The
  *        calculation of the coefficient can refer to digital biquad filter.
  * @note  The array of channel coefficients of the IIR0 module can only be
  *        placed in SRAM14.  
  *        The array of channel coefficients of the IIR1 module can only be
  *        placed in SRAM15.  
  *        The array of channel coefficients of the IIR2 module can only be
  *        placed in SRAM16.
  */
typedef struct __ll_iir_coeff_cfg {
    /*! The number of left-shifting IIR b coefficients. The valid range of this
     *  value is 0~16.
     */
    u16 scale      : 5,
        reserved_0 : 11;
    /*! IIR b0 coefficient
     */
    s16 b0_coef;
    /*! IIR b1 coefficient
     */
    s16 b1_coef;
    /*! IIR b2 coefficient
     */
    s16 b2_coef;
    /*! The IIR a1 coefficient is the standard calculated a1 coefficient multiplied
     *  by -1.
     */
    s16 a1_coef;
    /*! The IIR a2 coefficient is the standard calculated a2 coefficient multiplied
     *  by -1.
     */
    s16 a2_coef;
    u32 reserved_1;
    /*! IIR calculates historical value, only needs to be set to 0 at
     *  initialization.
     */
    s64 psum[2];
} TYPE_LL_IIR_COEFF_CFG;

/**
  * @brief This structure is used to configure the interrupt enable and the interrupt
  *        source selection for a particular channel of a particular IIR module.
  */
typedef struct __ll_iir_cfg {
    /*! The specific channel number of the IIR module. Each IIR module has 64 channels
     *  (0~63).
     */
    u8                            chn;
    /*! Enable the path between IIR and evsys. 
     */
    bool                          evsys_chn_en;
} TYPE_LL_IIR_CFG;

/**
  * @brief Low layer, specifies the IIR module initialization, used only for the reset 
  *        IIR module and configuration information start address.
  */
typedef struct __ll_iir_init {
    /*! The starting sram address of the IIR configuration parameter, which must 
     *  be 8Byte aligned. 
     *  @note IIR0's param_start_addr can only be located in SRAM11.  
     *        IIR1's param_start_addr can only be located in SRAM12.  
     *        IIR2's param_start_addr can only be located in SRAM13.  
     *        Statement example:  
     *        static TYPE_LL_IIR_PARAM_CFG iir_param[max_chn] 
     *                                     __attribute__((aligned(8), section("SRAM11")));  
     *        max_chn: max_chn is expressed as the maximum channel value using iir. 
     *                 For example, if you want to use channel 4 of IIR0 for calculation, 
     *                 max_chn=5, and the range of n is between 0 and 7.
     */
    u32 param_start_addr;
    /*! The length of the iir coefficient buffer.
     */
    u32 param_buf_len;
} TYPE_LL_IIR_INIT;

/**
  * @}
  */

/** @defgroup IIR_LL_Interrupt IIR LL Interrupt Handle function
  * @brief   IIR LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the IIR  
    Interrupt Handle function.

    how to use?

    The IIR interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the IIR in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup IIR_LL_Inti_Cfg IIR LL Initialization And Configuration
  * @brief    IIR LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the IIR data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  IIR module initialization function
  * @param  p_iir: Select the initialized DFTRAN group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_iir_init(IIR_TypeDef *p_iir, TYPE_LL_IIR_INIT *p_init);

/**
  * @brief  IIR module detele initialization function
  * @param  p_iir: Select the initialized IIR group pointer
  * @retval None
  */
void ll_iir_deinit(IIR_TypeDef *p_iir);

/**
  * @brief  IIR module interrupt configuration
  * @param  p_iir: Select the initialized IIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_iir_irq_config(IIR_TypeDef *p_iir, TYPE_LL_IIR_IRQ_CFG *p_cfg);

/**
  * @brief  IIR module configuration
  * @param  p_iir: Select the initialized IIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_iir_config(IIR_TypeDef *p_iir, TYPE_LL_IIR_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup IIR_LL_Data_Transfers IIR LL Data transfers functions
  * @brief    IIR LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the IIR data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  IIR module channel start function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_iir_start(IIR_TypeDef *p_iir, u64 chn_bits);
    
/**
  * @brief  IIR module channel stop function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_iir_stop(IIR_TypeDef *p_iir, u64 chn_bits);

/**
  * @brief  IIR check sample interrupt enable
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_IIR_CHECK_SAMPLE_INTERRUPT_ENABLE(p_iir, chn_bits)     (!((p_iir)->IIR_INT_SRCL0 & LL_IIR_INT_SRCL0(chn_bits)) && \
                                                                   !((p_iir)->IIR_INT_SRCH0 & LL_IIR_INT_SRCH0(chn_bits)) && \
                                                                   !((p_iir)->IIR_INT_SRCL1 & LL_IIR_INT_SRCL1(chn_bits)) && \
                                                                   !((p_iir)->IIR_INT_SRCH1 & LL_IIR_INT_SRCH1(chn_bits)))

/**
  * @brief  IIR check dma buffer half interrupt enable
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_IIR_CHECK_DMA_HALF_INTERRUPT_ENABLE(p_iir, chn_bits)   ((p_iir)->IIR_INT_SRCL0 & LL_IIR_INT_SRCL0(chn_bits) || \
                                                                   (p_iir)->IIR_INT_SRCL1 & LL_IIR_INT_SRCL1(chn_bits))

/**
  * @brief  IIR check dma buffer full interrupt enable
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_IIR_CHECK_DMA_FULL_INTERRUPT_ENABLE(p_iir, chn_bits)   ((p_iir)->IIR_INT_SRCH0 & LL_IIR_INT_SRCH0(chn_bits) || \
                                                                   (p_iir)->IIR_INT_SRCH1 & LL_IIR_INT_SRCH1(chn_bits))

/**
  * @brief  IIR module get filter pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bit : Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
#define LL_IIR_GET_SAMPLE_PENDING(p_iir, chn_bit)                 ((((u64)(p_iir)->IIR_FILT_PND1 << 32) | (p_iir)->IIR_FILT_PND0) & chn_bit)

/**
  * @brief  IIR module get dma buffer half pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bit : Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
#define LL_IIR_GET_DMA_HALF_PENDING(p_iir, chn_bit)               ((((u64)(p_iir)->IIR_HALF_PND1 << 32) | (p_iir)->IIR_HALF_PND0) & chn_bit)

/**
  * @brief  IIR module get dma buffer full pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bit : Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
#define LL_IIR_GET_DMA_FULL_PENDING(p_iir, chn_bit)               ((((u64)(p_iir)->IIR_FULL_PND1 << 32) | (p_iir)->IIR_FULL_PND0) & chn_bit)

/**
  * @brief  IIR module enable interrupt function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_iir_interrupt_enable(IIR_TypeDef *p_iir, u64 chn_bits) {
    p_iir->IIR_INT_ENA0 |= LL_IIR_INT_ENA0(chn_bits);
    p_iir->IIR_INT_ENA1 |= LL_IIR_INT_ENA1(chn_bits >> 32);
}

/**
  * @brief  IIR module disable interrupt function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_iir_interrupt_disable(IIR_TypeDef *p_iir, u64 chn_bits) {
    p_iir->IIR_INT_ENA0 &= ~(LL_IIR_INT_ENA0(chn_bits));
    p_iir->IIR_INT_ENA1 &= ~(LL_IIR_INT_ENA1(chn_bits >> 32));
}

/**
  * @brief  IIR get filter pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE u64 ll_iir_get_sample_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    u32 pnd1 = p_iir->IIR_FILT_PND1;
    u32 pnd0 = p_iir->IIR_FILT_PND0;
    return (((u64)pnd1 << 32) | pnd0) & chn_bits;
}


/**
  * @brief  IIR get half pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE u64 ll_iir_get_dma_half_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    u32 pnd1 = p_iir->IIR_HALF_PND1;
    u32 pnd0 = p_iir->IIR_HALF_PND0;
    return (((u64)pnd1 << 32) | pnd0) & chn_bits;
}

/**
  * @brief  IIR get full pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE u64 ll_iir_get_dma_full_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    u32 pnd1 = p_iir->IIR_FULL_PND1;
    u32 pnd0 = p_iir->IIR_FULL_PND0;
    return (((u64)pnd1 << 32) | pnd0) & chn_bits;
}

/**
  * @brief  IIR clear filter pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE void ll_iir_clear_sample_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    p_iir->IIR_FILT_PND0 |= LL_IIR_FILT_PND0(chn_bits);
    p_iir->IIR_FILT_PND1 |= LL_IIR_FILT_PND1(chn_bits >> 32);
}

/**
  * @brief  IIR clear half pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE void ll_iir_clear_dma_half_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    p_iir->IIR_HALF_PND0 |= LL_IIR_HALF_PND0(chn_bits);
    p_iir->IIR_HALF_PND1 |= LL_IIR_HALF_PND1(chn_bits >> 32);
}

/**
  * @brief  IIR clear full pending
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: The IIR channel number that was started.
  * @retval Returns the status value of png.
  */
__STATIC_INLINE void ll_iir_clear_dma_full_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    p_iir->IIR_FULL_PND0 |= LL_IIR_FULL_PND0(chn_bits);
    p_iir->IIR_FULL_PND1 |= LL_IIR_FULL_PND1(chn_bits >> 32);
}

/**
  * @brief  IIR wait for filter finished pending function
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_iir_wait_sample_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    if(chn_bits & 0xFFFFFFFF) {
        while(chn_bits != (p_iir->IIR_FILT_PND0 & chn_bits));
        p_iir->IIR_FILT_PND0 |= LL_IIR_FILT_PND0(chn_bits);
    } 
    if((chn_bits >> 32) & 0xFFFFFFFF) {
        while((chn_bits >> 32) != (p_iir->IIR_FILT_PND1 & (chn_bits >> 32)));
        p_iir->IIR_FILT_PND1 |= LL_IIR_FILT_PND1(chn_bits >> 32);
    }
}

/**
  * @brief  IIR wait for half finished pending function
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_iir_wait_dma_half_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    if(chn_bits & 0xFFFFFFFF) {
        while(chn_bits != (p_iir->IIR_HALF_PND0 & chn_bits));
        p_iir->IIR_HALF_PND0 |= LL_IIR_HALF_PND0(chn_bits);
    } 
    if((chn_bits >> 32) & 0xFFFFFFFF) {
        while((chn_bits >> 32) != (p_iir->IIR_HALF_PND1 & (chn_bits >> 32)));
        p_iir->IIR_HALF_PND1 |= LL_IIR_HALF_PND1(chn_bits >> 32);
    }
}

/**
  * @brief  IIR wait for full finished pending function
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_iir_wait_dma_full_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    if(chn_bits & 0xFFFFFFFF) {
        while(chn_bits != (p_iir->IIR_FULL_PND0 & chn_bits));
        p_iir->IIR_FULL_PND0 |= LL_IIR_FULL_PND0(chn_bits);
    }
    if((chn_bits >> 32) & 0xFFFFFFFF) {
        while((chn_bits >> 32) != (p_iir->IIR_FULL_PND1 & (chn_bits >> 32)));
        p_iir->IIR_FULL_PND1 |= LL_IIR_FULL_PND1(chn_bits >> 32);
    }
}

/**
  * @brief  IIR wait for filter or full finished pending function
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png(TYPE_ENUM_LL_IIR_PNG).
  */
__STATIC_INLINE TYPE_ENUM_LL_IIR_PNG ll_iir_wait_dma_half_full_pending(IIR_TypeDef *p_iir, u64 chn_bits) {
    TYPE_ENUM_LL_IIR_PNG sta = LL_IIR_PNG_IDLE;

    if(chn_bits & 0xFFFFFFFF) {
        while((chn_bits != (p_iir->IIR_HALF_PND0 & chn_bits)) &&
              (chn_bits != (p_iir->IIR_FULL_PND0 & chn_bits)));
        if(chn_bits == (p_iir->IIR_HALF_PND0 & chn_bits)) {
            p_iir->IIR_HALF_PND0 |= LL_IIR_HALF_PND0(chn_bits);
            sta = LL_IIR_PNG_HALF;
        }
        if(chn_bits == (p_iir->IIR_FULL_PND0 & chn_bits)) {
            p_iir->IIR_FULL_PND0 |= LL_IIR_FULL_PND0(chn_bits);
            sta = LL_IIR_PNG_FULL;
        }
    } 
    if((chn_bits >> 32) & 0xFFFFFFFF) {
        while(((chn_bits >> 32) != (p_iir->IIR_HALF_PND1 & (chn_bits >> 32))) &&
              ((chn_bits >> 32) != (p_iir->IIR_FULL_PND1 & (chn_bits >> 32))));
        if((chn_bits >> 32) == (p_iir->IIR_HALF_PND1 & (chn_bits >> 32))) {
            p_iir->IIR_HALF_PND1 |= LL_IIR_HALF_PND1(chn_bits >> 32);
            sta = LL_IIR_PNG_HALF;
        }
        if((chn_bits >> 32) == (p_iir->IIR_FULL_PND1 & (chn_bits >> 32))) {
            p_iir->IIR_FULL_PND1 |= LL_IIR_FULL_PND1(chn_bits >> 32);
            sta = LL_IIR_PNG_FULL;
        }
    }

    return sta;
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

#endif //__TX_PHE_LL_IIR_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
