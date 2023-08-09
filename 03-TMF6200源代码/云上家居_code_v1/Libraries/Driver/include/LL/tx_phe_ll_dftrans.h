/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_dftrans.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DFTRANS LL firmware functions.
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
#ifndef __TX_PHE_LL_DFTRANS_H
#define __TX_PHE_LL_DFTRANS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup dftrans_interface_gr DFTRANS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup DFTRANS_LL_Driver DFTRANS LL Driver
  * @ingroup  dftrans_interface_gr
  * @brief Mainly the driver part of the DFTRANS module, which includes \b DFTRANS \b Register 
  * \b Constants, \b DFTRANS \b Exported \b Constants, \b DFTRANS \b Exported \b Struct, \b DFTRANS
  * \b Data \b transfers \b functions, \b DFTRANS \b Initialization \b and \b DFTRANS \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup DFTRANS_LL_Register_Constants DFTRANS LL Register Constants
  * @ingroup  DFTRANS_LL_Driver
  * @brief    DFTRANS LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the DFTRANS 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the DFTRANS register, mainly for convenience. Understand the 
    configuration of the DFTRANS.
    
@endverbatim
  *
  * @{
  */

/***** DFTRANS_CON *****/
/*! DFTRANS dma input data downsample select
 */
#define LL_DFTRAN_DOWNSAMPLE_SEL(n)               (((n)&0xF) << 10)
/*! DFTRANS interrupt enable
 */
#define LL_DFTRAN_IE                              (1UL << 9)
/*! DFTRANS Completion mark
 */
#define LL_DFTRAN_PEND                            (1UL << 8)
/*! (cos+/-j*sin) is an option to add or subtract
 */
#define LL_DFTRAN_ADD_MINUS                       (1UL << 6)
/*! The imaginary part of the DMA complex and the real part DMA exchange
 */
#define LL_DFTRAN_REAL_IMAG_EXCHANG               (1UL << 5)
/*! The imaginary part of the complex number of DMA is 0
 */
#define LL_DFTRAN_IMAG_ZERO                       (1UL << 4)
/*! DFTRANS module enable
 */
#define LL_DFTRAN_ENABLE                          (1UL << 0)


/***** DFTRANS_LEN *****/
/*! DFTRANS The length of the cyclic buffer that stores REAL and IMAG 
    data, in units of 16-bit data.
 */
#define LL_DFTRAN_LEN(n)                          (((n)&0xFFFF) << 0)


/***** DFTRANS_INDEX *****/
/*! DFTRANS The index of the circular buffer that stores REAL and IMAG 
    data, in units of 16-bit data. Indicates that the operation starts 
    to fetch data from here.
 */
#define LL_DFTRAN_INDEX(n)                        (((n)&0xFFFF) << 0)


/***** DFTRANS_STEP *****/
/*! The step of the angle of COS and SIN, the unit is pi. 32-bit unsigned 
    number, of which 31 are decimal places. The value range is [0, 2). 
    Calculation formula: (2*X/DFTRANS_LEN)*2^31
 */
#define LL_DFTRAN_STEP(n)                         (((n)&0xFFFFFFFF) << 0)


/***** DFTRANS_REAL_START *****/
/*! The real data DMA start address of the complex number. 
    Need 16bit alignment.
 */
#define LL_DFTRAN_REAL_START(n)                   (((n)&0xFFFFFFFF) << 0)


/***** DFTRANS_IMAG_START *****/
/*! The imaginary part of the imaginary data DMA start address. 
    Need 16bit alignment.
 */
#define LL_DFTRAN_IMAG_START(n)                   (((n)&0xFFFFFFFF) << 0)


/***** DFTRANS_OUT *****/
/*! The real part of the DFTRAN calculation results. 16-bit signed numbers, 
    15 of which are decimals.
 */
#define LL_DFTRAN_REAL_OUT(n)                     (((n)&0xFFFF) << 16)
/*! The imaginary part of the DFTRAN calculation result. 16-bit signed 
    numbers, 15 of which are decimals.
 */
#define LL_DFTRAN_IMAG_OUT(n)                     (((n)&0xFFFF) << 0)


/***** DFTRANS_DMA_LEN *****/
/*! The number of DMA points in Dftrans, ranging from 17 to 1024.
 */
#define LL_DFTRAN_DMA_LEN(n)                      (((n)&0x7FF) << 0)


/***** DFTRANS_NORMALIZED_COEF *****/
/*! Normalization coefficient. It is the end of the calculation step: 
    divided by the part of dma_len.The calculation formula is: 
    (1 /DFTRANS_DMA_LEN) * 2^19, the result is saturated to 32767.
 */
#define LL_DFTRAN_NORMALIZED_COEF(n)              (((n)&0x7FFF) << 0)


/***** DFTRANS_OUT_ADR *****/
/*! Save the register DFTRANS_OUT to the DMA start address of the SRAM. 
    Requires 64bit alignment. The output will take up 64 bits of space.
 */
#define LL_DFTRAN_OUT_START(n)                    (((n)&0xFFFFFFFF) << 0)


/**
  * @}
  */

/** @defgroup DFTRANS_LL_Exported_Constants DFTRANS LL Exported Constants
  * @ingroup  DFTRANS_LL_Driver
  * @brief    DFTRANS LL external constant definition
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
  * @brief Enumeration constant for DFTRAN downsample select
  */
typedef enum {
    /*! DFTRAN downsample select 1
     */
    LL_DFTRAN_DOWNSAMPLE_1 = 0,
    /*! DFTRAN downsample select 2
     */
    LL_DFTRAN_DOWNSAMPLE_2,
    /*! DFTRAN downsample select 3
     */
    LL_DFTRAN_DOWNSAMPLE_3,
    /*! DFTRAN downsample select 4
     */
    LL_DFTRAN_DOWNSAMPLE_4,
    /*! DFTRAN downsample select 5
     */
    LL_DFTRAN_DOWNSAMPLE_5,
    /*! DFTRAN downsample select 6
     */
    LL_DFTRAN_DOWNSAMPLE_6,
    /*! DFTRAN downsample select 7
     */
    LL_DFTRAN_DOWNSAMPLE_7,
    /*! DFTRAN downsample select 8
     */
    LL_DFTRAN_DOWNSAMPLE_8,
    /*! DFTRAN downsample select 9
     */
    LL_DFTRAN_DOWNSAMPLE_9,
    /*! DFTRAN downsample select 10
     */
    LL_DFTRAN_DOWNSAMPLE_10,
    /*! DFTRAN downsample select 11
     */
    LL_DFTRAN_DOWNSAMPLE_11,
    /*! DFTRAN downsample select 12
     */
    LL_DFTRAN_DOWNSAMPLE_12,
    /*! DFTRAN downsample select 13
     */
    LL_DFTRAN_DOWNSAMPLE_13,
    /*! DFTRAN downsample select 14
     */
    LL_DFTRAN_DOWNSAMPLE_14,
    /*! DFTRAN downsample select 15
     */
    LL_DFTRAN_DOWNSAMPLE_15,
    /*! DFTRAN downsample select 16
     */
    LL_DFTRAN_DOWNSAMPLE_16,
} TYPE_ENUM_LL_DFTRAN_DOWNSAMPLE;

/**
  * @brief Enumeration constant for DFTRAN imaginary select addition or subtraction 
  */
typedef enum {
    /*! DFTRAN imaginary select addition
     */
    LL_DFTRAN_ADD = 0,
    /*! DFTRAN imaginary select subtraction
     */
    LL_DFTRAN_MINUS,
} TYPE_ENUM_LL_DFTRAN_ADD_MIN;

/**
  * @brief Enumeration constant for DFTRAN real and imag exchang select
  */
typedef enum {
    /*! imaginary part of the DMA complex and the real part DMA no exchange 
     */
    LL_DFTRAN_R_I_EX_NO = 0,
    /*! imaginary part of the DMA complex and the real part DMA exchange
     */
    LL_DFTRAN_R_I_EX,
} TYPE_ENUM_LL_DFTRAN_R_I_EX;

/**
  * @brief Enumeration constant for DFTRAN imaginary value select
  */
typedef enum {
    /*! The imaginary part of the complex number of DMA is not 0
     */
    LL_DFTRAN_IMAG_NO_ZERO = 0,
    /*! The imaginary part of the complex number of DMA is 0
     */
    LL_DFTRAN_IMAG_IS_ZREO,
} TYPE_ENUM_LL_DFTRAN_IMAG;

/**
  * @}
  */

/** @defgroup DFTRANS_LL_Exported_Struct DFTRANS LL Exported Struct
  * @ingroup  DFTRANS_LL_Driver
  * @brief    DFTRANS LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the DFTRANS registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_dftrans_init. Function, you can configure the DFTRANS module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief DFTRAN low layer configuration structure
  */
typedef struct __ll_dftrans_cfg { 
    /*! Configure the second complex step increment direction
     *  @note Valid values are LL_DFTRAN_ADD and LL_DFTRAN_MINUS.
     */
    TYPE_ENUM_LL_DFTRAN_ADD_MIN    step_dir;
    /*! DMA real and imaginary address exchange.For example, the default
     *  real part of DFTRANS0 is placed in SRAM1 (ie, DFTRANS_REAL_STAR is
     *  in SRAM1) and the imaginary part is in SRAM2 (ie, DFTRANS_IMAG_STAR
     *  is in SRAM2). If the DMA exchanges real and imaginary addresses,
     *  then DFTRANS_REAL_STAR will be located in SRAM2 and
     *  DFTRANS_IMAG_STAR will be located in SRAM1.
     *  @note Valid values are LL_DFTRAN_R_I_EX_NO and LL_DFTRAN_R_I_EX.
     */
    TYPE_ENUM_LL_DFTRAN_R_I_EX     dma_real_img_swap;
    /*! If the value is DFTC_IMG_ALL_ZERO, then the DMA imaginary part is
     *  all 0.
     *  @note Valid values are LL_DFTRAN_IMAG and LL_DFTRAN_IMAG_ZREO.
     */
    TYPE_ENUM_LL_DFTRAN_IMAG       img_val_sel;
    /*! The number of points calculated by the dft module. The valid range
     *  is 16 to 1024; 
     */
    u16                            dft_points;
    /*! Len represents the cycle length of the DMA buffer. The value of len
     *  must not be less than the number of dft_points * downsample_sel.
     */    
    u16                            dma_points;
    /*! Index represents the subscript offset of the DMA buffer.           
     */
    u16                            index;
    /*! Step represents the radians step of the second complex cos and sin.
     *  The unit is PI.
     *  @note step = (2*n/dft_points)*2^31
     */
    u32                            step;
    /*! (a+b*j) DMA real part start address of complex number, 2 byte
     *  aligned. The real part data is a 16-bit signed number with 15 bit
     *  fixed point.
     *  @note DFTRANS0's real_dma_src_addr can only be located in SRAM1.  
     *        DFTRANS1's real_dma_src_addr can only be located in SRAM2.  
     *        DFTRANS2's real_dma_src_addr can only be located in SRAM3.  
     */
    u32                            real_dma_src_addr;
    /*! (a+b*j) DMA imaginary part start address of complex number, 2 byte
     *  aligned. The imaginary part data is a 16-bit signed number with 15
     *  bit fixed point.
     *  @note DFTRANS0's img_dma_src_addr can only be located in SRAM2.  
     *        DFTRANS1's img_dma_src_addr can only be located in SRAM3.  
     *        DFTRANS2's img_dma_src_addr can only be located in SRAM1. 
     */
    u32                            img_dma_src_addr;
    /*! Need to use an address to save the output(64bit size) in accordance with 64bit
     *  aligned variables.  
     *  @note DFTRANS0's dma_dst_addr can only be located in SRAM1.  
     *        DFTRANS1's dma_dst_addr can only be located in SRAM2.  
     *        DFTRANS2's dma_dst_addr can only be located in SRAM3. 
     */
    u32                            dma_dst_addr;
    /*! Index Steps for each increment. The valid range is 1 to 16.
     */
    TYPE_ENUM_LL_DFTRAN_DOWNSAMPLE downsample_sel;
} TYPE_LL_DFTRANS_CFG;

/**
  * @brief DFTRAN low layer Initialization structure
  */
typedef struct __ll_dftrans_init {
    u8 reserved;
} TYPE_LL_DFTRANS_INIT;

/**
  * @}
  */

/** @defgroup DFTRANS_LL_Interrupt DFTRANS LL Interrupt Handle function
  * @brief   DFTRANS LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DFTRANS  
    Interrupt Handle function.

    how to use?

    The DFTRANS interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the DFTRANS in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup DFTRANS_LL_Inti_Cfg DFTRANS LL Initialization And Configuration
  * @brief    DFTRANS LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DFTRANS data 
    Initialization and Configuration.
    
    formula:
    when ADD_MINUS = 0, 
        result = sum((a(n)+b(n)*j)*(cos(2*n*PI/N)+j*sin(2*n*PI/N)))/N, 
        n = 0,1,2,3,4....N-1
    when ADD_MINUS = 1, 
        result = sum((a(n)+b(n)*j)*(cos(2*n*PI/N)-j*sin(2*n*PI/N)))/N, 
        n = 0,1,2,3,4....N-1
        
    how to use?
    
    1. Declare the DFTRAN initialization structure TYPE_LL_DFTRANS_INIT, and call the 
       ll_dftran_init() function to reset the module and enable the module clock.
    1. Declare the configuration structure TYPE_LL_DFTRANS_CFG of DFTRAN and 
       configure it according to actual needs.
    2. Call the ll_dftran_config() function to initialize.
    3. Call function ll_dftran_start() to start DFTRAN work.
    4. Call function ll_dftran_wait() to wait for DFTRAN to complete the calculation, 
       or call LL_DFTRAN_GET_FINISHED_PEG() function to make PEG wait for judgment.
    5. Call ll_dftran_get_result() function to get the calculation result of DFTRAN.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief  DFTRAN module initialization function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  p_init  : Module configuration structure pointer
  * @retval None
  */
void ll_dftran_init(DFT_TypeDef *p_dftran, TYPE_LL_DFTRANS_INIT *p_init);

/**
  * @brief  DFTRAN module detele initialization function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_deinit(DFT_TypeDef *p_dftran);

/**
  * @brief  DFTRAN module configuration
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_dftran_config(DFT_TypeDef *p_dftran, TYPE_LL_DFTRANS_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup DFTRANS_LL_Data_Transfers DFTRANS LL Data transfers functions
  * @brief    DFTRANS LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DFTRANS data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  DFTRAN module start function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_start(DFT_TypeDef *p_dftran);

/**
  * @brief  DFTRAN module stop function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_stop(DFT_TypeDef *p_dftran);

/**
  * @brief  DFTRAN get finished pending
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval pending value
  */
#define LL_DFTRAN_GET_DONE_PENDING(p_dftran)      ((p_dftran)->CON & LL_DFTRAN_PEND)

/**
  * @brief  DFTRAN clear finished pending
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_dftran_clear_done_pending(DFT_TypeDef *p_dftran) { 
    p_dftran->CON |= LL_DFTRAN_PEND;
}

/**
  * @brief  DFTRAN enable interrupt
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_dftran_interrupt_enable(DFT_TypeDef *p_dftran) { 
    p_dftran->CON |= LL_DFTRAN_IE;
}

/**
  * @brief  DFTRAN disable interrupt
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_dftran_interrupt_disable(DFT_TypeDef *p_dftran) { 
    p_dftran->CON &= ~LL_DFTRAN_IE;
}

/**
  * @brief  DFTRAN Wait for the specified channel to finish computing
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
__STATIC_INLINE void ll_dftran_wait_done_pending(DFT_TypeDef *p_dftran) {    
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
    while(p_dftran->CON & LL_DFTRAN_ENABLE);
    p_dftran->CON |= LL_DFTRAN_PEND;
}

/**
  * @brief  Get DFTRAN module specific channel calculations
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  p_real  : The real part of the calculation. The real part data is a
  *                   16-bit signed number with 15 bit fixed point.
  * @param  p_imag  : The imaginary part of the result. The imaginary part data
  *                   is a 16-bit signed number with 15 bit fixed point.
  * @retval None
  */
__STATIC_INLINE void ll_dftran_get_result(DFT_TypeDef *p_dftran, s16 *p_real, s16 *p_imag) {
    u32 result = p_dftran->OUT;
    
    *p_real = (result >> 16) & 0xFFFF;
    *p_imag  = result & 0xFFFF;
}

/**
  * @brief  Set DFTRAN module specific channel INDEX
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  index   : 16bit index to be set, the index is small than dft_points
  * @retval None
  */
__STATIC_INLINE void ll_dftran_set_index(DFT_TypeDef *p_dftran, u16 index) {    
    p_dftran->INDEX = LL_DFTRAN_INDEX(index);
}

/**
  * @brief  Get DFTRAN module specific channel INDEX
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval Index
  */
#define LL_DFTRAN_GET_INDEX(p_dftran)             ((p_dftran)->INDEX & LL_DFTRAN_INDEX(0xFFFF))

/**
  * @brief  Set DFTRAN module specific channel real_dma_src_addr
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  addr    : 32bit addr \ref TYPE_LL_DFTRANS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_dftran_set_real_dma_src_addr(DFT_TypeDef *p_dftran, u32 addr) {
    p_dftran->REAL_STADR = LL_DFTRAN_REAL_START(addr);
}

/**
  * @brief  Set DFTRAN module specific channel img_dma_src_addr
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  addr    : 32bit addr \ref TYPE_LL_DFTRANS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_dftran_set_img_dma_src_addr(DFT_TypeDef *p_dftran, u32 addr) {
    p_dftran->IMAG_STADR = LL_DFTRAN_IMAG_START(addr);
}

/**
  * @brief  Set DFTRAN module specific channel dma_dst_addr
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  addr    : 32bit addr \ref TYPE_LL_DFTRANS_CFG
  * @retval None
  */
__STATIC_INLINE void ll_dftran_set_dma_dst_addr(DFT_TypeDef *p_dftran, u32 addr) {
    p_dftran->OUT_ADR = LL_DFTRAN_OUT_START(addr);
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

#endif //__TX_PHE_LL_DFTRANS_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
