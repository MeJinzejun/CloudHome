/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_arctan.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the ARCTAN LL firmware functions.
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
#ifndef __TX_PHE_LL_ARCTAN_H
#define __TX_PHE_LL_ARCTAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup arctan_interface_gr ARCTAN Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup ARCTAN_LL_Driver ARCTAN LL Driver
  * @ingroup  arctan_interface_gr
  * @brief Mainly the driver part of the ARCTAN module, which includes \b ARCTAN \b Register 
  * \b Constants, \b ARCTAN \b Exported \b Constants, \b ARCTAN \b Exported \b Struct, \b ARCTAN
  * \b Data \b transfers \b functions, \b ARCTAN \b Initialization \b and \b ARCTAN \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup ARCTAN_LL_Register_Constants ARCTAN LL Register Constants
  * @ingroup  ARCTAN_LL_Driver
  * @brief    ARCTAN LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the ARCTAN 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the ARCTAN register, mainly for convenience. Understand the 
    configuration of the ARCTAN.
    
@endverbatim
  *
  * @{
  */

/***** ARCTAN_CON *****/
/*! ARCTAN interrupt enable bit.
 */
#define LL_ARCTAN_CON_IE                          (1UL << 9)
/*! ARCTAN calculates the completion flag.
 */
#define LL_ARCTAN_CON_PEND                        (1UL << 8)
/*! ARCTAN dma mode enable bit.
 */
#define LL_ARCTAN_CON_DMA_EN                      (1UL << 2)
/*! ARCTAN data storage format selection.
 */
#define LL_ARCTAN_CON_SAVE_TOGETHER               (1UL << 1)
/*! The enable bit of the ARCTAN module.
 */
#define LL_ARCTAN_CON_ENABLE                      (1UL << 0)


/***** ARCTAN_DMA0_STADR *****/
/*! The starting address of the dma of the cos data in ARCTAN.
 */
#define LL_ARCTAN_DMA0_START(n)                   (((n)&0xFFFFFFFF) << 0)


/***** ARCTAN_DMA1_STADR *****/
/*! The starting address of the dma of the sin data in ARCTAN.
 */
#define LL_ARCTAN_DMA1_START(n)                   (((n)&0xFFFFFFFF) << 0)


/***** ARCTAN_DMA2_STADR *****/
/*! The starting address of the dma of the output data in ARCTAN.
 */
#define LL_ARCTAN_DMA2_START(n)                   (((n)&0xFFFFFFFF) << 0)


/***** ARCTAN_DMA_LEN *****/
/*! The length of the dma in ARCTAN.
 */
#define LL_ARCTAN_DMA_LEN(n)                      (((n)&0xFFFF) << 0)


/***** ARCTAN_IN *****/
/*! Data input of CPU mode cos in ARCTAN.
 */
#define LL_ARCTAN_IN_COS(n)                       (((n)&0xFFFF) << 16)
/*! Data input of CPU mode sin in ARCTAN.
 */
#define LL_ARCTAN_IN_SIN(n)                       (((n)&0xFFFF) << 0)


/***** ARCTAN_OUT *****/
/*! The result data is calculated in CPU mode in ARCTAN.
 */
#define LL_ARCTAN_OUT(n)                          (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup ARCTAN_LL_Exported_Constants ARCTAN LL Exported Constants
  * @ingroup  ARCTAN_LL_Driver
  * @brief    ARCTAN LL external constant definition
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
  * @brief Enumeration constant for ARCTAN mode selection.
  */
typedef enum {
    /*! In CPU mode, the user needs to call the function ll_arctan_set_input_data() 
     *  to input the calculated data. Once ll_arctan_start() is calculated once, it 
     *  is obtained by the function ll_arctan_get_result() or ll_arctan_return_result(). 
     */
    LL_ARCTAN_MODE_CPU = 0,
    /*! In the CPU mode, the user needs to configure cos_dma_src_addr, sin_dma_src_addr, 
     *  dma_dst_addr, and dma_len, and call ll_arctan_start() to calculate. 
     */
    LL_ARCTAN_MODE_DMA,
} TYPE_ENUM_LL_ARCTAN_MODE;

/**
  * @brief Enumeration constant for ARCTAN dma data selection.
  */
typedef enum {
    /*! Cos and sin saved in memory dividually, cos is stored in cos_dma_src_addr, sin is 
     *  stored in sin_dma_src_addr. 
     */
    LL_ARCTAN_DATA_MODE_SEPARATE = 0,
    /*! Cos and sin saved in memory together, stored in the format { cos[32:16], sin[15:0] }, 
     *  the user needs to store it in the space specified by cossin_dma_src_addr. 
     */
    LL_ARCTAN_DATA_MODE_TOGETHER,
} TYPE_ENUM_LL_ARCTAN_DATA_MODE;

/**
  * @}
  */

/** @defgroup ARCTAN_LL_Exported_Struct ARCTAN LL Exported Struct
  * @ingroup  ARCTAN_LL_Driver
  * @brief    ARCTAN LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the ARCTAN registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_arctan_init. Function, you can configure the ARCTAN module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief ARCTAN low layer configuration structure
  * @note  1. If cos=0, sin=0, then the module's calculation result is 0;  
  *        2. If cos=0, sin! =0, then the calculation result of the module is PI/2, which is 16384;  
  *        3. The calculation result range of the module is between 0~2PI, which is between 0 and 65535.  
  */
typedef struct __ll_arctan_cfg {
    /*! ARCTAN calculation mode, CPU mode and DMA mode, please refer to the comment 
     *  in enumeration TYPE_ENUM_LL_ARCTAN_MODE for details. 
     */
    TYPE_ENUM_LL_ARCTAN_MODE      work_mode;
    /*! ARCTAN DMA mode data storage format, please refer to the annotation in the 
     *  enumeration TYPE_ENUM_LL_ARCTAN_DATA. 
     */
    TYPE_ENUM_LL_ARCTAN_DATA_MODE mode;
    union {
        /*! mode = LL_ARCTAN_DATA_SEPARATE, The starting address of the cos buffer 
         *  is stored in cos_dma_src_addr, 8 byte aligned.The real part data is a 16-bit 
         *  signed number with 15 bit fixed point.
         *  @note ARCTAN0's cos_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM4/SRAM5.  
         *        ARCTAN1's cos_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM6/SRAM7.  
         *        ARCTAN2's cos_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM8/SRAM9.  
         *        Statement example:  
         *        static s16 arctan_cos_buf[1024] __attribute__((aligned(8),section("SRAM0")));
         */
        u32                       cos_dma_src_addr;
        /*! mode = LL_ARCTAN_DATA_TOGETHER, The starting address of the cos and sin buffer 
         *  is stored in cossin_dma_src_addr, stored in the format { cos[32:16], sin[15:0] },
         *  8 byte aligned. The data is a 16-bit signed number with 15 bit fixed point.
         *  @note ARCTAN0's cossin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM4/SRAM5.  
         *        ARCTAN1's cossin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM6/SRAM7.  
         *        ARCTAN2's cossin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM8/SRAM9.  
         *        Statement example:  
         *        static s16 arctan_cossin_buf[1024] __attribute__((aligned(8),section("SRAM1")));
         */
        u32                       cossin_dma_src_addr;
    };
    /*! The starting address of the sin buffer is stored in sin_dma_src_addr, 8 byte aligned.
     *  The data is a 16-bit signed number with 15 bit fixed point.
     *  @note ARCTAN0's sin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM4/SRAM5.  
     *        ARCTAN1's sin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM6/SRAM7.  
     *        ARCTAN2's sin_dma_src_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM8/SRAM9.  
     *        Statement example:  
     *        static s16 arctan_sin_buf[1024] __attribute__((aligned(8),section("SRAM1")));
     */
    u32                           sin_dma_src_addr;
    /*! The starting address of the output buffer is stored in dma_dst_addr, 8 byte aligned.
     *  The data is a 16-bit unsigned number with 15 bit fixed point.
     *  @note ARCTAN0's dma_dst_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM4/SRAM5.  
     *        ARCTAN1's dma_dst_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM6/SRAM7.  
     *        ARCTAN2's dma_dst_addr can only be located in SRAM1/SRAM2/SRAM3/SRAM8/SRAM9.  
     *        Statement example:  
     *        static u16 arctan_obuf[1024] __attribute__((aligned(8),section("SRAM1")));
     */
    u32                           dma_dst_addr;
    /*! The length of the dma is set. 
     */
    u32                           dma_points;
} TYPE_LL_ARCTAN_CFG;

/**
  * @brief ARCTAN low layer Initialization structure
  */
typedef struct __ll_arctan_init {
    u8 reserved;
} TYPE_LL_ARCTAN_INIT;

/**
  * @}
  */

/** @defgroup ARCTAN_LL_Interrupt ARCTAN LL Interrupt Handle function
  * @brief   ARCTAN LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the ARCTAN  
    Interrupt Handle function.

    how to use?

    The ARCTAN interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the ARCTAN in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup ARCTAN_LL_Inti_Cfg ARCTAN LL Initialization And Configuration
  * @brief    ARCTAN LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the ARCTAN data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  ARCTAN module initialization function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  p_init  : Module configuration structure pointer
  * @retval None
  */
void ll_arctan_init(ARCTAN_TypeDef *p_arctan, TYPE_LL_ARCTAN_INIT *p_init);

/**
  * @brief  ARCTAN module detele initialization function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_deinit(ARCTAN_TypeDef *p_arctan);

/**
  * @brief  ARCTAN module configuration function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_arctan_config(ARCTAN_TypeDef *p_arctan, TYPE_LL_ARCTAN_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup ARCTAN_LL_Data_Transfers ARCTAN LL Data transfers functions
  * @brief    ARCTAN LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the ARCTAN data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  ARCTAN module start function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_start(ARCTAN_TypeDef *p_arctan);

/**
  * @brief  ARCTAN module stop function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_stop(ARCTAN_TypeDef *p_arctan);

/**
  * @brief  ARCTAN get finished pending
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval pending value
  */
#define LL_ARCTAN_GET_DONE_PENDING(p_arctan)      ((p_arctan)->CON & LL_ARCTAN_CON_PEND)

/**
  * @brief  ARCTAN clear pending
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_arctan_clear_done_pending(ARCTAN_TypeDef *p_arctan) {
    p_arctan->CON |= LL_ARCTAN_CON_PEND;
}

/**
  * @brief  ARCTAN performs a single calculation.
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  cos_data: input cos data vaule
  * @param  sin_data: input sin data vaule
  * @retval Returns the result of ARCTAN calculation, 16bit.
  */
u16 ll_arctan_calculate_once(ARCTAN_TypeDef *p_arctan, s16 cos_data, s16 sin_data);

/**
  * @brief  ARCTAN set input data function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  cos_data: input cos data vaule
  * @param  sin_data: input sin data vaule
  * @retval None
  */
__STATIC_INLINE void ll_arctan_set_input_data(ARCTAN_TypeDef *p_arctan, u16 cos_data, u16 sin_data) {
    p_arctan->IN = LL_ARCTAN_IN_COS((u32)cos_data) | LL_ARCTAN_IN_SIN(sin_data);
}

/**
  * @brief  ARCTAN Wait for the specified channel to finish computing
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_arctan_wait_done_pending(ARCTAN_TypeDef *p_arctan) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_arctan->CON & LL_ARCTAN_CON_ENABLE);
    p_arctan->CON |= LL_ARCTAN_CON_PEND;
}

/**
  * @brief  Get the ARCTAN calculation result function.
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  p_result: Save the 16bit of the arctan calculation result.
  * @retval None
  */
__STATIC_INLINE void ll_arctan_get_result(ARCTAN_TypeDef *p_arctan, u16 *p_result) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_arctan->CON & LL_ARCTAN_CON_ENABLE);
    p_arctan->CON |= LL_ARCTAN_CON_PEND;
    
    *p_result = p_arctan->OUT;
}

/**
  * @brief  Returns the ARCTAN calculation result function.
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval Returns the result of ARCTAN calculation, 16bit.
  */
__STATIC_INLINE u16 ll_arctan_return_result(ARCTAN_TypeDef *p_arctan) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_arctan->CON & LL_ARCTAN_CON_ENABLE);
    p_arctan->CON |= LL_ARCTAN_CON_PEND;
    
    return p_arctan->OUT;
}

/**
  * @brief  ARCTAN enable interrupt
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_arctan_interrupt_enable(ARCTAN_TypeDef *p_arctan) {
    p_arctan->CON |= LL_ARCTAN_CON_IE;
}

/**
  * @brief  ARCTAN disable interrupt
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_arctan_interrupt_disable(ARCTAN_TypeDef *p_arctan) {
    p_arctan->CON &= ~(LL_ARCTAN_CON_IE);
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

#endif //__TX_PHE_LL_ARCTAN_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
