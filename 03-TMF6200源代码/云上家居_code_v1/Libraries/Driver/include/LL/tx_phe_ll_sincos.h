/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_sincos.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SINCOS LL firmware functions.
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
#ifndef __TX_PHE_LL_SINCOS_H
#define __TX_PHE_LL_SINCOS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup sincos_interface_gr SINCOS Driver
  * @ingroup    TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup SINCOS_LL_Driver SINCOS LL Driver
  * @ingroup  sincos_interface_gr
  * @brief Mainly the driver part of the SINCOS module, which includes \b SINCOS \b Register 
  * \b Constants, \b SINCOS \b Exported \b Constants, \b SINCOS \b Exported \b Struct, \b SINCOS
  * \b Data \b transfers \b functions, \b SINCOS \b Initialization \b and \b SINCOS \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup SINCOS_LL_Register_Constants SINCOS LL Register Constants
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the SINCOS 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the SINCOS register, mainly for convenience. Understand the 
    configuration of the SINCOS.
    
@endverbatim
  *
  * @{
  */

/***** SINCOS_CON *****/
/*! The SINCOS module calculates the completion flag.
 */
#define LL_SINCOS_PEND                      (1UL << 8)
/*! The SINCOS module interrupt enable bit.
 */
#define LL_SINCOS_IE                        (1UL << 5)
/*! Mode selection for SINCOS radians.
 */
#define LL_SINCOS_DMA_MODE                  (1UL << 4)
/*! The choice of SINCOS calculation mode.
 */
#define LL_SINCOS_DMA_EN                    (1UL << 3)
/*! SINCOS input buffer data format selection.
 */
#define LL_SINCOS_MODE(n)                   (((n)&0x3) << 1)
/*! The enable bit of the SINCOS module.
 */
#define LL_SINCOS_ENABLE                    (1UL << 0)


/***** SINCOS_LEN *****/
/*! The length of the buffer in SINCOS DMA mode.
 */
#define LL_SINCOS_LEN(n)                    (((n)&0xFFFF) << 0)


/***** SINCOS_STEP *****/
/*! In SINCOS DMA mode, the radians value is set to  auto increment,
 *  which sets the auto increment step of the radians value. 
 */
#define LL_SINCOS_STEP(n)                   (((n)&0xFFFF) << 0)


/***** SINCOS_DATA_IN *****/  
/*! Data input value in SINCOS CPU mode.
 */
#define LL_SINCOS_DATA_IN(n)                (((n)&0xFFFF) << 0)


/***** SINCOS_DATA_OUT *****/
/*! Data output value in SINCOS CPU mode.
 */
#define LL_SINCOS_DATA_OUT(n)               (((n)&0xFFFFFFFF) << 0)


/***** SINCOS_DATA_IN_ADR *****/   
/*! In SINCOS DMA mode, enter the starting address of the buffer.
 */
#define LL_SINCOS_DATA_IN_ADR(n)            (((n)&0xFFFFFFFF) << 0)


/***** SINCOS_DATA_OUT_ADR *****/
/*! In SINCOS DMA mode, the starting address of the output buffer.
 */
#define LL_SINCOS_DATA_OUT_ADR(n)           (((n)&0xFFFFFFFF) << 0)


/**
  * @}
  */

/** @defgroup SINCOS_LL_Exported_Constants SINCOS LL Exported Constants
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL external constant definition
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
  * @brief Enumeration constant for SINCOS radians value input mode selection.
  */
typedef enum {
    /*! The input radians of SICNOS are selected from the memory, 
     *  which is the memory space pointed to by dma_src_addr in
     *  TYPE_LL_SINCOS_CFG. 
     */
    LL_SINCOS_RADIAN_MEM = 0,
    /*! The input radians of SICNOS are selected to be incremented 
     * according to the step size of the step. 
     */
    LL_SINCOS_RADIAN_STEP_INC,
} TYPE_ENUM_LL_SINCOS_RADIAN;

/**
  * @brief Enumeration constant for SICNOS calculation mode selection.
  */
typedef enum {
    /*! When the SICNOS calculation mode is selected to use the CPU mode, 
     *  the input data is acquired from the data_in in the configuration 
     *  structure TYPE_LL_SINCOS_CFG, and is calculated only once. 
     */
    LL_SINCOS_MODE_CPU = 0,
    /*! When the SICNOS calculation mode is selected to use the DMA mode, 
     *  the input data is obtained from the start address of dma_src_addr 
     *  in the configuration structure TYPE_LL_SINCOS_CFG, and the space of 
     *  length len is obtained, and a total of len times are calculated. 
     */
    LL_SINCOS_MODE_DMA,
} TYPE_ENUM_LL_SINCOS_MODE;

/**
  * @brief Enumeration constant for SICNOS data format selection.
  */
typedef enum {
    /*! SINCOS input data 32Bit, the format is {sin1, sin0}, 
     *  each occupying 16Bit. 
     */
    LL_SINCOS_MODE_DATA_SIN = 0,
    /*! SINCOS input data 32Bit, the format is {cos1, cos0}, 
     *  each occupying 16Bit. 
     */
    LL_SINCOS_MODE_DATA_COS,
    /*! SINCOS input data 32Bit, the format is {cos0, sin0}, 
     *  each occupying 16Bit. 
     */
    LL_SINCOS_MODE_DATA_COSSIN,
} TYPE_ENUM_LL_SINCOS_MODE_DATA_FORMAT;

/**
  * @}
  */

/** @defgroup SINCOS_LL_Exported_Struct SINCOS LL Exported Struct
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the SINCOS registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_sincos_init. Function, you can configure the SINCOS module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief SINCOS low layer configuration structure
  */
typedef struct __ll_sincos_cfg {
    /*! Configure the SINCOS radians value input mode selection.
     *  Please refer to TYPE_ENUM_LL_SINCOS_RADIAN for the specific type.
     */
    TYPE_ENUM_LL_SINCOS_RADIAN           radian_sel;
    /*! Configure the calculation mode of SINCOS, CPU mode or DMA mode.
     *  Please refer to TYPE_ENUM_LL_SINCOS_MODE for the specific type.
     */
    TYPE_ENUM_LL_SINCOS_MODE             work_mode;
    /*! Configure the data format of the SINCOS input buffer.
     *  Please refer to TYPE_ENUM_LL_SINCOS_DATA_FORMAT for the specific type.
     */
    TYPE_ENUM_LL_SINCOS_MODE_DATA_FORMAT mode;
    
    /***** SINCOS CPU MODE *****/
    /*! Only used in cpu mode.  
     *  data_in = actual radian/PI*32768  
     *  Effective radians is between 0 and 2*PI (not including 2*PI).  
     *  @note The calculation result is equivalent to:  
     *        eg: int radians = data_in/32768.0*PI;  
     *            int sin_temp = (int)(sin(radians)*32768);
     */
    u16                                  data_in;
    
    /***** SINCOS DMA MODE *****/
    /*! Configure the length of the SINCOS DMA input buffer.
     */
    u16                                  dma_points;
    /*! Configure the step value of SINCOS.
     */
    u16                                  step;
    /*! SINCOS DMA start address of input buffer, 8 byte aligned. 
     *  The real part data is a 16-bit signed number with 15 bit
     *  fixed point.
     *  @note SINCOS0's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        SINCOS1's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        Statement example:  
     *        static s16 sincos_ibuf[512] __attribute__((aligned(8),section("SRAM0")));
     */
    u32                                  dma_src_addr;
    /*! SINCOS DMA start address of output buffer, 8 byte aligned. 
     *  The real part data is a 16-bit signed number with 15 bit
     *  fixed point.
     *  @note SINCOS0's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        SINCOS1's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  
     *        Statement example:  
     *        static s16 sincos_obuf[512*2] __attribute__((aligned(8),section("SRAM0")));
     */
    u32                                  dma_dst_addr;
} TYPE_LL_SINCOS_CFG;

/**
  * @brief SINCOS low layer Initialization structure
  */
typedef struct __ll_sincos_init {
    u8 reserved;
} TYPE_LL_SINCOS_INIT;

/**
  * @}
  */

/** @defgroup SINCOS_LL_Interrupt SINCOS LL Interrupt Handle function
  * @brief    SINCOS LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SINCOS  
    Interrupt Handle function.

    how to use?

    The SINCOS interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the SINCOS in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup SINCOS_LL_Inti_Cfg SINCOS LL Initialization And Configuration
  * @brief    SINCOS LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SINCOS data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  SINCOS module initialization function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  p_init  : Module configuration structure pointer
  * @retval None
  */
void ll_sincos_init(SINCOS_TypeDef *p_sincos, TYPE_LL_SINCOS_INIT *p_init);

/**
  * @brief  SINCOS module detele initialization function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_deinit(SINCOS_TypeDef *p_sincos);

/**
  * @brief  SINCOS module configuration function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_sincos_config(SINCOS_TypeDef *p_sincos, TYPE_LL_SINCOS_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup SINCOS_LL_Data_Transfers SINCOS LL Data transfers functions
  * @brief    SINCOS LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SINCOS data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  SINCOS module start function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_start(SINCOS_TypeDef *p_sincos);

/**
  * @brief  SINCOS module stop function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_stop(SINCOS_TypeDef *p_sincos);

/**
  * @brief  SINCOS get finished pending
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval pending value
  */
#define LL_SINCOS_GET_DONE_PENDING(p_sincos)      ((p_sincos)->CON & LL_SINCOS_PEND)

/**
  * @brief  SINCOS clear pending
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_sincos_clear_done_pending(SINCOS_TypeDef *p_sincos) {
    p_sincos->CON |= LL_SINCOS_PEND;
}

/**
  * @brief  SINCOS enable interrupt
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
__STATIC_INLINE void ll_sincos_interrupt_enable(SINCOS_TypeDef *p_sincos) {
    p_sincos->CON |= LL_SINCOS_IE;
}

/**
  * @brief  SINCOS disable interrupt
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
__STATIC_INLINE void ll_sincos_interrupt_disable(SINCOS_TypeDef *p_sincos) {
    p_sincos->CON &= ~(LL_SINCOS_IE);
}

/**
  * @brief  SINCOS set input data function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data: input data
  * @retval None
  */
__STATIC_INLINE void ll_sincos_set_input_data(SINCOS_TypeDef *p_sincos, u16 data) {
    p_sincos->DATA_IN = data;
}

/**
  * @brief  SINCOS Wait for the specified channel to finish computing
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
__STATIC_INLINE void ll_sincos_wait_done_pending(SINCOS_TypeDef *p_sincos) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_sincos->CON & LL_SINCOS_ENABLE);
    p_sincos->CON |= LL_SINCOS_PEND;
}

/**
  * @brief  Get the SINCOS calculation result function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  p_result_h: Save the high 16bit of the sincos calculation result.
  * @param  p_result_l: Save the low 16bit of the sincos calculation result.
  * @retval None
  */
__STATIC_INLINE void ll_sincos_get_result(SINCOS_TypeDef *p_sincos, s16 *p_result_h, s16 *p_result_l) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    u32 result_data = 0;
    while(p_sincos->CON & LL_SINCOS_ENABLE);
    p_sincos->CON |= LL_SINCOS_PEND;
    
    result_data = p_sincos->DATA_OUT;
    *p_result_h = result_data >> 16;
    *p_result_l = result_data & 0xFFFF;
}

/**
  * @brief  Returns the SINCOS calculation result function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval Returns the result of SINCOS calculation, 32bit.
  */
__STATIC_INLINE u32 ll_sincos_return_result(SINCOS_TypeDef *p_sincos) {
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_sincos->CON & LL_SINCOS_PEND);
    p_sincos->CON |= LL_SINCOS_PEND;
    
    return p_sincos->DATA_OUT;
}

/**
  * @brief  Sincos sin mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @retval The calculation result of sin.
  */
s16 ll_sincos_calculate_sin_once(SINCOS_TypeDef *p_sincos, u16 data);

/**
  * @brief  Sincos cos mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @retval The calculation result of sin.
  */
s16 ll_sincos_calculate_cos_once(SINCOS_TypeDef *p_sincos, u16 data);

/**
  * @brief  Sincos cos and sin mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @param  p_sin   : Calculate the saved value of the result sin.
  * @param  p_cos   : Calculate the saved value of the result cos.
  * @retval The calculation result of sin and cos.
  */
void ll_sincos_calculate_cossin_once(SINCOS_TypeDef *p_sincos, u16 data, s16 *p_sin, s16 *p_cos);

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

#endif //__TX_PHE_LL_SINCOS_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
