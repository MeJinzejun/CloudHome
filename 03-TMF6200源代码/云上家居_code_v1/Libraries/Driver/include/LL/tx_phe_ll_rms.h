/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_rms.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the RMS LL firmware functions.
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
#ifndef __TX_PHE_LL_RMS_H
#define __TX_PHE_LL_RMS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup rms_interface_gr RMS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup RMS_LL_Driver RMS LL Driver
  * @ingroup  rms_interface_gr
  * @brief Mainly the driver part of the RMS module, which includes \b RMS \b Register 
  * \b Constants, \b RMS \b Exported \b Constants, \b RMS \b Exported \b Struct, \b RMS
  * \b Data \b transfers \b functions, \b RMS \b Initialization \b and \b RMS \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup RMS_LL_Register_Constants RMS LL Register Constants
  * @ingroup  RMS_LL_Driver
  * @brief    RMS LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the RMS 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the RMS register, mainly for convenience. Understand the 
    configuration of the RMS.
    
@endverbatim
  *
  * @{
  */

/***** RMS_CON *****/
/*! The RMS calculation completes the flag.
 */
#define LL_RMS_CON_PEND                           (1UL << 8)
/*! The RMS enables the interrupt bit.

 */
#define LL_RMS_CON_IE                             (1UL << 4)
/*! RMS calculation mode selection bit.
 */
#define LL_RMS_CON_MODE(n)                        (((n)&0x7) << 1)
/*! RMS module enable bit.
 */
#define LL_RMS_CON_ENABLE                         (1UL << 0)


/***** RMS_LEN *****/
/*! The RMS calculates the length of the number of samples. */
#define LL_RMS_LEN(n)                             (((n)&0xFFFF) << 0)


/***** RMS_IN_FRAC_WIDTH *****/
/*! Only valid when LL_RMS_CON_MODE==011, this is LL_RMS_DATA_IN data fractional width.
 */
#define LL_RMS_IN_FRAC_WIDTH(n)                   (((n)&0x1F) << 0)


/***** RMS_OUT_FRAC_WIDTH *****/
/*! Only valid when LL_RMS_CON_MODE==011, this is LL_RMS_DATA_OUT data fractional width.
 *  The formula is: RMS_OUT_FRAC_WIDTH=(RMS_IN_FRAC_WIDTH+1)/2 
 */
#define LL_RMS_OUT_FRAC_WIDTH(n)                  (((n)&0x1F) << 0)


/***** RMS_DATA_IN *****/
/*! Only valid when LL_RMS_CON_MODE==011, this is the data we want to sqrt.
 */
#define LL_RMS_DATA_IN(n)                         (((n)&0xFFFFFFFF) << 0)


/***** RMS_DATA_OUTL *****/
/*! The RMS output data low 32bits, RMS_DATA_OUT[63:0] = { RMS_DATA_OUTH, RMS_DATA_OUTL}.
 */
#define LL_RMS_DATA_OUTL(n)                       (((n)&0xFFFFFFFF) << 0)


/***** RMS_DATA_OUTH *****/
/*! The RMS output data high 32bits, RMS_DATA_OUT[63:0] = { RMS_DATA_OUTH, RMS_DATA_OUTL}.
 */
#define LL_RMS_DATA_OUTH(n)                       (((n)&0xFFFFFFFF) << 0)


/***** RMS_DATA_IN_ADR *****/
/*! In RMS DMA mode, the start address of input buffer, 64 alignment.
 */
#define LL_RMS_DATA_IN_ADR(n)                     (((n)&0xFFFFFFFF) << 0)


/***** RMS_DATA_OUT_ADR *****/
/*! In RMS DMA mode, the start address of output buffer, 64 alignment.
 */
#define LL_RMS_DATA_OUT_ADR(n)                    (((n)&0xFFFFFFFF) << 0)

/**
  * @}
  */

/** @defgroup RMS_LL_Exported_Constants RMS LL Exported Constants
  * @ingroup  RMS_LL_Driver
  * @brief    RMS LL external constant definition
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
  * @brief Enumeration constant for The RMS mode selection
  */
typedef enum {
    /*! Read 32-bit data from memory, where the lower 16 bits are imaginary parts, 
     *  the upper 16 bits are real parts, and then the operation (imaginary part 
     *  square + real squared) opens the root number, saves the 16-bit result to 
     *  memory, and continuously calculates RMS_LEN times. 
     */
    LL_RMS_MODE_MOLDING = 0,
    /*! Read RMS_LEN from memory, then 16bits, squares and /RMS_LEN, then open the 
     *  root number, the final result is 16 bits, save to memory, is to find the 
     *  root mean square value, pay attention to this mode when RMS_LEN must meet 
     *  2 n power. 
     */
    LL_RMS_MODE_RMS,
    /*! Read RMS_LEN from memory so many 16bits, find the sum of squares, and finally 
     *  the result is 64 bits, save to memory. 
     */
    LL_RMS_MODE_SUM_SQUARE,
    /*! Find the root number of RMS0_DATA_IN and store the result in RMS0_DATA_OUT. 
     *  Perform calculations only once at a time.
     */
    LL_RMS_MODE_SQUARE_ROOT,
    /*! Read RMS_LEN from memory so many 16bits, sum, and finally result 64 bits, 
     *  save to memory. 
     */
    LL_RMS_MODE_SUM,
    /*! Read 32-bit data from memory, where the lower 16 bits are imaginary parts, 
     *  the upper 16 bits are real parts, and then the operation (imaginary part 
     *  square + real squared), the 32-bit result is saved to memory, and the RMS_LEN 
     *  is continuously operated as many times. 
     */
    LL_RMS_MODE_MOLE_SQUARE,
} TYPE_ENUM_LL_RMS_MODE;


/**
  * @}
  */

/** @defgroup RMS_LL_Exported_Struct RMS LL Exported Struct
  * @ingroup  RMS_LL_Driver
  * @brief    RMS LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the RMS registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_rms_init. Function, you can configure the RMS module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief RMS low layer configuration structure
  */
typedef struct __ll_rms_cfg {
    /*! RMS mode selection, a total of 6 calculation modes are supported. For specific 
     *  mode introduction, please refer to macro TYPE_ENUM_LL_RMS_MODE. 
     */
    TYPE_ENUM_LL_RMS_MODE mode;
    /*! Only valid when mode = LL_RMS_MODE_SQUARE_ROOT, this is input data fractional 
     *  width. The range of the parameter is between 0 and 31.
     */
    u8                    in_frac_width;
    /*! All modes except mode = LL_RMS_MODE_SQUARE_ROOT are DMA, and len indicates the 
     *  length of the DMA. 
     */
    u16                   dma_points;
    /*! The starting address of the DMA input buffer, the address is 8Byte aligned. 
     *  @note RMS0's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        RMS1's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        RMS2's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        Statement example:  
     *        static s16 rms_ibuf[1024] __attribute__((aligned(8),section("SRAM0")));
     */
    u32                   dma_src_addr;
    /*! The starting address of the DMA output buffer, the address is 8Byte aligned. 
     *  @note RMS0's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        RMS1's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        RMS2's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        Statement example:  
     *        static s16 rms_obuf[1024] __attribute__((aligned(8),section("SRAM0")));
     */
    u32                   dma_dst_addr;
} TYPE_LL_RMS_CFG;

/**
  * @brief RMS low layer Initialization structure
  */
typedef struct __ll_rms_init {
    u8 reserved;
} TYPE_LL_RMS_INIT;

/**
  * @}
  */

/** @defgroup RMS_LL_Interrupt RMS LL Interrupt Handle function
  * @brief   RMS LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the RMS  
    Interrupt Handle function.

    how to use?

    The RMS interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the RMS in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup RMS_LL_Inti_Cfg RMS LL Initialization And Configuration
  * @brief    RMS LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the RMS data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer RMS module initialization
  * @param  p_rms : The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_RMS_INIT)
  * @retval None
  */
void ll_rms_init(RMS_TypeDef *p_rms, TYPE_LL_RMS_INIT *p_init);

/**
  * @brief  Low layer RMS module detele initialization
  * @param  p_rms: The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @retval None
  */
void ll_rms_deinit(RMS_TypeDef *p_rms);

/**
  * @brief  Low layer RMS module configuration
  * @param  p_rms: The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_RMS_DMA_CFG)
  * @retval None
  */
void ll_rms_config(RMS_TypeDef *p_rms, TYPE_LL_RMS_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup RMS_LL_Data_Transfers RMS LL Data transfers functions
  * @brief    RMS LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the RMS data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  RMS module start function
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval None
  */
void ll_rms_start(RMS_TypeDef *p_rms);

/**
  * @brief  RMS module stop function
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval None
  */
void ll_rms_stop(RMS_TypeDef *p_rms);

/**
  * @brief  RMS get finished pending
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval pending value
  */
#define LL_RMS_GET_DONE_PENDING(p_rms)            ((p_rms)->CON & LL_RMS_CON_PEND)

/**
  * @brief  RMS clear finished pending
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval pending value
  */
__STATIC_INLINE void ll_rms_clear_done_pending(RMS_TypeDef *p_rms) {
    p_rms->CON |= LL_RMS_CON_PEND;
}

/**
  * @brief  RMS get output data frac width
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval pending value
  */
#define LL_RMS_GET_OUT_FRAC_WIDTH(p_rms)          ((p_rms)->OUT_FRAC_WIDTH)

/**
  * @brief  RMS Wait for the specified channel to finish computing
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval None
  */
__STATIC_INLINE void ll_rms_wait_done_pending(RMS_TypeDef *p_rms) {
    while(p_rms->CON & LL_RMS_CON_ENABLE);
    p_rms->CON |= LL_RMS_CON_PEND;
}

/**
  * @brief  low layer for setting input data in RMS square root mode.
  * @param  p_rms: Select the initialized RMS group pointer.
  * @param  data: The data value that needs to be set.
  * @retval None
  */
__STATIC_INLINE void ll_rms_set_square_root_indata(RMS_TypeDef *p_rms, u32 data) {
    p_rms->DATA_IN = data;
}

/**
  * @brief  low layer for returning data in RMS square root mode.
  * @param  p_rms: Select the initialized RMS group pointer.
  * @retval The calculation results in RMS square root mode.
  */
__STATIC_INLINE u64 ll_rms_return_square_root_indata(RMS_TypeDef *p_rms) {
    u32 data_outh;
    ll_rms_wait_done_pending(p_rms);
    data_outh = p_rms->DATA_OUTH;
    return (p_rms->DATA_OUTL + ((s64)data_outh << 32));
}

/**
  * @brief  low layer for getting data in RMS square root mode.
  * @param  p_rms   : Select the initialized RMS group pointer.
  * @param  result_h: The RMS calculation results in a high 32 bit.
  * @param  result_l: The RMS calculation results in a low 32 bit.
  * @retval None.
  */
__STATIC_INLINE void ll_rms_get_square_root_indata(RMS_TypeDef *p_rms, u32 *result_h, u32 *result_l) {
    ll_rms_wait_done_pending(p_rms);
    *result_h = p_rms->DATA_OUTH;
    *result_l = p_rms->DATA_OUTL;
}

/**
  * @brief  low layer for  RMS enable interrupt
  * @param  p_rms: Select the initialized RMS group pointer.
  * @retval None.
  */
__STATIC_INLINE void ll_rms_interrupt_enable(RMS_TypeDef *p_rms) {
    p_rms->CON |= LL_RMS_CON_IE;
}

/**
  * @brief  low layer for  RMS disable interrupt
  * @param  p_rms: Select the initialized RMS group pointer.
  * @retval None.
  */
__STATIC_INLINE void ll_rms_interrupt_disable(RMS_TypeDef *p_rms) {
    p_rms->CON &= ~(LL_RMS_CON_IE);
}

/**
  * @brief  RMS mode setting
  * @param  p_rms: Select the initialized RMS group pointer.
  * @param  mode : RMS mode
  * @retval None.
  */
__STATIC_INLINE void ll_rms_mode_set(RMS_TypeDef *p_rms,
                                     TYPE_ENUM_LL_RMS_MODE mode) {
    p_rms->CON &= ~LL_RMS_CON_MODE(0x07);
    p_rms->CON |= LL_RMS_CON_MODE(mode);
}

/**
  * @brief  RMS input data fractional width setting
  * @param  p_rms: Select the initialized RMS group pointer.
  * @param  width: input data fractional width
  * @retval None.
  */
__STATIC_INLINE void ll_rms_in_frac_width_set(RMS_TypeDef *p_rms, u8 width) {
    p_rms->IN_FRAC_WIDTH = LL_RMS_IN_FRAC_WIDTH(width);
}

/**
  * @brief  RMS length of the DMA setting
  * @param  p_rms : Select the initialized RMS group pointer.
  * @param  points: length of the DMA
  * @retval None.
  */
__STATIC_INLINE void ll_rms_dma_points_set(RMS_TypeDef *p_rms, u16 points) {
    p_rms->LEN = LL_RMS_LEN(points);
}

/**
  * @brief  RMS starting address of the DMA input buffer set
  * @param  p_rms: Select the initialized RMS group pointer.
  * @param  addr : The starting address of the DMA input buffer
  * @retval None.
  */
__STATIC_INLINE void ll_rms_dma_src_addr_set(RMS_TypeDef *p_rms, u32 addr) {
    p_rms->DATA_IN_ADR = LL_RMS_DATA_IN_ADR(addr);
}

/**
  * @brief  RMS starting address of the DMA output buffer set
  * @param  p_rms: Select the initialized RMS group pointer.
  * @param  addr : The starting address of the DMA output buffer
  * @retval None.
  */
__STATIC_INLINE void ll_rms_dma_dst_addr_set(RMS_TypeDef *p_rms, u32 addr) {
    p_rms->DATA_OUT_ADR = LL_RMS_DATA_OUT_ADR(addr);
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

#endif //__TX_PHE_LL_RMS_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
