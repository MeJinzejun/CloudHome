/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_fir.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the FIR LL firmware functions.
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
#ifndef __TX_PHE_LL_FIR_H
#define __TX_PHE_LL_FIR_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup fir_interface_gr FIR Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup FIR_LL_Driver FIR LL Driver
  * @ingroup  fir_interface_gr
  * @brief Mainly the driver part of the FIR module, which includes \b FIR \b Register 
  * \b Constants, \b FIR \b Exported \b Constants, \b FIR \b Exported \b Struct, \b FIR
  * \b Data \b transfers \b functions, \b FIR \b Initialization \b and \b FIR \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
    
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
    
/** @defgroup FIR_LL_Register_Constants FIR LL Register Constants
  * @ingroup  FIR_LL_Driver
  * @brief    FIR LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the FIR 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the FIR register, mainly for convenience. Understand the 
    configuration of the FIR.
    
@endverbatim
  *
  * @{
  */

/***** FIR_CH_ENA *****/
/*! FIR's channel enable bit, each 1 bit represents a channel, representing 
 *  the 0th to 7th channels. 
 */
#define LL_FIR_CH_ENA(n)                          (((n)&0xFF) << 0)


/***** FIR_CPU_KST *****/           
/*! The triggering of the FIR channel is enabled. Each bit represents a channel, 
 *  which represents the triggering of the channel from channel 0 to channel 7. 
 */
#define LL_FIR_CPU_KST(n)                         (((n)&0xFF) << 0)


/***** FIR_CFG_ADDR *****/        
/*! The starting address of the FIR configuration information.
 */
#define LL_FIR_CFG_ADDR(n)                        (((n)&0xFFFFFFFF) << 0)


/***** FIR_INT_ENA *****/       
/*! The FIR channel's interrupt trigger is enabled. Each bit represents a channel,
 *  which represents the trigger enable of the 0th to 7th channels. 
 */
#define LL_FIR_INT_ENA(n)                         (((n)&0xFF) << 0)


/***** FIR_HALF_PND *****/          
/*! When the output BUFF of the FIR channel of the FIR has filled half of the data, 
 *  the completion flag is written, and 1 is written to the register to clear the pending. 
 */
#define LL_FIR_HALF_PND(n)                        (((n)&0xFF) << 0)


/***** FIR_FULL_PND *****/               
/*! When the output BUFF of the FIR channel of the FIR has filled the data, the 
 *  completion flag is written, and 1 write to the register clears the pending. 
 */
#define LL_FIR_FULL_PND(n)                        (((n)&0xFF) << 0)


/***** FIR_FILT_PND *****/                   
/*! When the FIR of the FIR channel N completes filtering, the completion flag is 
 *  written, and a 1 write to the register clears the pending. 
 */
#define LL_FIR_FILT_PND(n)                        (((n)&0xFF) << 0)


/***** FIR_INT_SRCL *****/                
/*! When each bit represents the interrupt source selection of one channel, it 
 *  represents the 0th to 7th channel, and is used together with SRCH0 in the 
 *  format of{SRCH0, SRCL0}. 
 */
#define LL_FIR_INT_SRCL(n)                        (((n)&0xFF) << 0)


/***** FIR_INT_SRCH *****/             
/*! When each bit represents the interrupt source selection of one channel, it 
 *  represents the 0th to 7th channel, and is used together with SRCH0 in the 
 *  format of{SRCH0, SRCL0}. 
 */
#define LL_FIR_INT_SRCH(n)                        (((n)&0xFF) << 0)


/***** FIR_DATA_OUT *****/               
/*! After each calculation of the fir, the calculation saves the register.
 */
#define LL_FIR_DATA_OUT(n)                        (((n)&0xFFFFFFFF) << 0)


/***** FIR_EVSYS_ENA *****/              
/*! FIR's evsys channel enable bit, each 1 bit represents a channel, representing 
 *  the 0th to 7th channel, used in conjunction with the event trigger, each event 
 *  triggers an FIR, completes an N channel filtering (mainly see Which way of this 
 *  register is enabled). 
 */
#define LL_FIR_EVSYS_ENA(n)                       (((n)&0xFF) << 0)


/**
  * @}
  */

/** @defgroup FIR_LL_Exported_Constants FIR LL Exported Constants
  * @ingroup  FIR_LL_Driver
  * @brief    FIR LL external constant definition
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

/**
  * @brief Enumeration constant for FIR finished pending status
  */
typedef enum {
    /*! Fir idle flag macro definition.
     */
    LL_FIR_PNG_IDLE = 0,
    /*! Fir completes the filter flag macro definition.
     */
    LL_FIR_PNG_FILT,
    /*! Fir completes the definition of the half mark macro.
     */
    LL_FIR_PNG_HALF,
    /*! Fir completes the definition of the full mark macro.
     */
    LL_FIR_PNG_FULL,
} TYPE_ENUM_LL_FIR_PNG;

  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for The FIR interrupt source triggers the selection
  */
typedef enum {
    /*! Each time the fir is filtered, an interrupt is triggered.
     */
    LL_FIR_INTR_SRC_FILT = 0,
    /*! Fir triggers an interrupt when it calculates half of the output buffer.
     */
    LL_FIR_INTR_SRC_HALF,
    /*! Fir triggers an interrupt when it calculates that the output buffer is full
     */
    LL_FIR_INTR_SRC_FULL,
    /*! Fir triggers an interrupt when it calculates that the output buffer is full and half. 
     */
    LL_FIR_INTR_SRC_HALF_FULL,
} TYPE_ENUM_LL_FIR_INTR_SRC_SEL;

/**
  * @}
  */

/** @defgroup FIR_LL_Exported_Struct FIR LL Exported Struct
  * @ingroup  FIR_LL_Driver
  * @brief    FIR LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the FIR registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_fir_init. Function, you can configure the FIR module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief FIR interrupt low layer configuration structure
  */
typedef struct __ll_fir_irq_cfg {
    /*! There are 8 sets of paths for each set of FIR, and chn is the one that 
     *  is selected for calculation, ranging from 0 to 7. 
     */
    u8                            chn;
    /*! Select the trigger source for the FIR module interrupt. 
     */
    TYPE_ENUM_LL_FIR_INTR_SRC_SEL intr_src_sel; 
} TYPE_LL_FIR_IRQ_CFG;

/**
  * @brief FIR param low layer configuration structure
  */
typedef struct __ll_fir_param_cfg {  
    /*! The downsampling of the FIR, when the dma_en=1 is valid, 
     *  internally outputs the downsampled data to the SRAM. 
     */
    u8  sample_rate;
    /*! The position where the FIR's downsampling starts sampling. 
     *  The default value starts from 0. 
     */
    u8  sample_rate_index;
    /*! Enable the DMA mode of FIR, occupying 1 bit of space. This 
     *  space allocation is fixed and no change is allowed.
     */
    u16 dma_en              : 1,
    /*! DMA mode selection:
     *  0: Each time the trigger is triggered, the sample_rate sample 
     *     is jumped, which is generally used for CPU triggering.
     *  1: Only one sample is calculated for each trigger, which is 
     *     generally used for automatic triggering of the ADC. 
     */
        dma_mode            : 1,
    /*! Reserved bits occupy 10bits of space. This space allocation 
     *  is fixed and no changes are allowed. 
     */
        reserved            : 10,
    /*! The fixed-point number of output data occupies a space range 
     *  of 5 bits between 0 and 31, and odata_frac_width_l refers to a 
     *  low 4 bits. This space allocation is fixed and no change is allowed. 
     */
        odata_frac_width_l  : 4;

    /*! The fixed-point number of output data occupies a 5-bit space range 
     *  between 0 and 31, and odata_frac_width_h refers to a maximum of 1 bit. 
     *  This space allocation is fixed and no change is allowed. 
     */
    u16 odata_frac_width_h  : 1,
    /*! Reserved bits occupy 15bits of space. This space allocation 
     *  is fixed and no changes are allowed. 
     */
        reserved2           : 15;
    /*! FIR calculates the buffer length of the coefficient in units of 2 bytes.
     */
    u16 coef_len;

    /*! The FIR calculates the starting address of the input buffer, which must 
     *  be 8Byte aligned. 
     *  @note FIR0's dma_src_addr can only be located in SRAM1.  
     *        FIR1's dma_src_addr can only be located in SRAM2.  
     *        FIR2's dma_src_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 fir0_ibuf[1024] __attribute__((aligned(8), section("SRAM1"))) ;
     */
    u32 dma_src_addr;
    /*! The FIR calculates the starting address of the output buffer, which must 
     *  be 8Byte aligned. 
     *  @note FIR0's dma_dst_addr can only be located in SRAM1.  
     *        FIR1's dma_dst_addr can only be located in SRAM2.  
     *        FIR2's dma_dst_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 fir0_obuf[1024] __attribute__((aligned(8), section("SRAM1"))) ;
     */
    u32 dma_dst_addr;
    /*! The FIR calculates the starting address of the coefficient, which must 
     *  be 8Byte aligned. 
     *  @note FIR0's coef_start_addr can only be located in SRAM1.  
     *        FIR1's coef_start_addr can only be located in SRAM2.  
     *        FIR2's coef_start_addr can only be located in SRAM3.  
     *        Statement example:  
     *        static s16 fir0_coef[1024] __attribute__((aligned(8), section("SRAM1"))) ;
     */
    u32 coef_start_addr;
    
    /*! FIR input buffer length, the unit is 2 bytes.
     */
    u16 dma_src_points;
    /*! FIR output buffer length, the unit is 2 bytes.
     */
    u16 dma_dst_points;

    /*! The FIR input buffer starts the calculation of the starting position. 
     */
    u32 dma_src_index;
    /*! The FIR output buffer starts the calculation of the starting position. 
     */
    u32 dma_dst_index;
} TYPE_LL_FIR_PARAM_CFG;

/**
  * @brief FIR low layer configuration structure
  */
typedef struct __ll_fir_cfg {
    /*! There are 8 sets of paths for each set of FIR, and chn is the one that 
     *  is selected for calculation, ranging from 0 to 7. 
     */
    u8                            chn;
    /*! Enable the path between FIR and evsys. 
     */
    bool                          evsys_chn_en;
} TYPE_LL_FIR_CFG;

/**
  * @brief FIR low layer Initialization structure
  */
typedef struct __ll_fir_init {
    /*! The starting sram address of the FIR configuration parameter, which must 
     *  be 8Byte aligned. 
     *  @note FIR0's param_start_addr can only be located in SRAM0.  
     *        FIR1's param_start_addr can only be located in SRAM0.  
     *        FIR2's param_start_addr can only be located in SRAM0.  
     *        Statement example:  
     *        static TYPE_LL_FIR_PARAM_CFG fir_param[max_chn] 
     *                                     __attribute__((aligned(8), section("SRAM0")));  
     *        max_chn: max_chn is expressed as the maximum channel value using fir. 
     *                 For example, if you want to use channel 4 of FIR0 for calculation, 
     *                 max_chn=5, and the range of n is between 0 and 7.
     */
    u32 param_start_addr; 
    /*! The length of the fir coefficient buffer.
     */
    u32 param_buf_len;
} TYPE_LL_FIR_INIT;


/**
  * @}
  */

/** @defgroup FIR_LL_Interrupt FIR LL Interrupt Handle function
  * @brief   FIR LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FIR  
    Interrupt Handle function.

    how to use?

    The FIR interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the FIR in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup FIR_LL_Inti_Cfg FIR LL Initialization And Configuration
  * @brief    FIR LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FIR data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  FIR module initialization function
  * @param  p_fir : Select the initialized DFTRAN group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_fir_init(FIR_TypeDef *p_fir, TYPE_LL_FIR_INIT *p_init);

/**
  * @brief  FIR module detele initialization function
  * @param  p_fir: Select the initialized FIR group pointer
  * @retval None
  */
void ll_fir_deinit(FIR_TypeDef *p_fir);

/**
  * @brief  FIR module interrupt configuration
  * @param  p_fir: Select the initialized FIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_fir_irq_config(FIR_TypeDef *p_fir, TYPE_LL_FIR_IRQ_CFG *p_cfg);

/**
  * @brief  FIR module configuration
  * @param  p_fir: Select the initialized FIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_fir_config(FIR_TypeDef *p_fir, TYPE_LL_FIR_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup FIR_LL_Data_Transfers FIR LL Data transfers functions
  * @brief    FIR LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FIR data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  FIR module start function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_fir_start(FIR_TypeDef *p_fir, u8 chn_bits);

/**
  * @brief  FIR module stop function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_fir_stop(FIR_TypeDef *p_fir, u8 chn_bits);

/**
  * @brief  FIR check sample interrupt enable
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_CHECK_SAMPLE_INTERRUPT_ENABLE(p_fir, chn_bits)     (!((p_fir)->FIR_INT_SRCL & LL_FIR_INT_SRCL(chn_bits)) && \
                                                                   !((p_fir)->FIR_INT_SRCH & LL_FIR_INT_SRCH(chn_bits)))

/**
  * @brief  FIR check dma buffer half interrupt enable
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_CHECK_DMA_HALF_INTERRUPT_ENABLE(p_fir, chn_bits)   ((p_fir)->FIR_INT_SRCL & LL_FIR_INT_SRCL(chn_bits))

/**
  * @brief  FIR check dma buffer full interrupt enable
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_CHECK_DMA_FULL_INTERRUPT_ENABLE(p_fir, chn_bits)   ((p_fir)->FIR_INT_SRCH & LL_FIR_INT_SRCH(chn_bits))

/**
  * @brief  FIR get filter pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_GET_SAMPLE_PENDING(p_fir, chn_bits)          ((p_fir)->FIR_FILT_PND & chn_bits)

/**
  * @brief  FIR get half pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_GET_DMA_HALF_PENDING(p_fir, chn_bits)        ((p_fir)->FIR_HALF_PND & chn_bits)

/**
  * @brief  FIR get full pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png.
  */
#define LL_FIR_GET_DMA_FULL_PENDING(p_fir, chn_bits)        ((p_fir)->FIR_FULL_PND & chn_bits)

/**
  * @brief  FIR enable interrupt
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_fir_interrupt_enable(FIR_TypeDef *p_fir, u8 chn_bits) {
    p_fir->FIR_INT_ENA |= LL_FIR_INT_ENA(chn_bits);
}

/**
  * @brief  FIR disable interrupt
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_fir_interrupt_disable(FIR_TypeDef *p_fir, u8 chn_bits) {
    p_fir->FIR_INT_ENA &= ~(LL_FIR_INT_ENA(chn_bits));
}

/**
  * @brief  FIR clear filter pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_fir_clear_sample_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    p_fir->FIR_FILT_PND |= chn_bits;
}

/**
  * @brief  FIR clear half pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_fir_clear_dma_half_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    p_fir->FIR_HALF_PND |= chn_bits;
}

/**
  * @brief  FIR clear full pending
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_fir_clear_dma_full_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    p_fir->FIR_FULL_PND |= chn_bits;
}

/**
  * @brief  FIR wait for filter finished pending function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_fir_wait_sample_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    while(chn_bits != (p_fir->FIR_FILT_PND & chn_bits));
    p_fir->FIR_FILT_PND |= chn_bits;
}

/**
  * @brief  FIR wait for half finished pending function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_fir_wait_half_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    while(chn_bits != (p_fir->FIR_HALF_PND & chn_bits));
    p_fir->FIR_HALF_PND |= chn_bits;
}

/**
  * @brief  FIR wait for full finished pending function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None.
  */
__STATIC_INLINE void ll_fir_wait_full_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    while(chn_bits != (p_fir->FIR_FULL_PND & chn_bits));
    p_fir->FIR_FULL_PND |= chn_bits;
}

/**
  * @brief  FIR wait for filter or full finished pending function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval Returns the status value of png(TYPE_ENUM_LL_FIR_PNG).
  */
__STATIC_INLINE TYPE_ENUM_LL_FIR_PNG ll_fir_wait_half_full_pending(FIR_TypeDef *p_fir, u8 chn_bits) {
    TYPE_ENUM_LL_FIR_PNG sta = LL_FIR_PNG_IDLE;
    
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    while((chn_bits != (p_fir->FIR_HALF_PND & chn_bits)) &&
          (chn_bits != (p_fir->FIR_FULL_PND & chn_bits)));
    if(chn_bits == (p_fir->FIR_HALF_PND & chn_bits)) {
        p_fir->FIR_HALF_PND |= chn_bits;
        sta = LL_FIR_PNG_HALF;
    }
    if(chn_bits == (p_fir->FIR_FULL_PND & chn_bits)) {
        p_fir->FIR_FULL_PND |= chn_bits;
        sta = LL_FIR_PNG_FULL;
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

#endif //__TX_PHE_LL_FIR_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
