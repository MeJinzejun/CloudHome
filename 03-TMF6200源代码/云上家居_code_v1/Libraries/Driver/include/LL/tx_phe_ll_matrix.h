/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_matrix.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the MATRIX LL firmware functions.
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
#ifndef __TX_PHE_LL_MATRIX_H
#define __TX_PHE_LL_MATRIX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup matrix_interface_gr MATRIX Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup MATRIX_LL_Driver MATRIX LL Driver
  * @ingroup  matrix_interface_gr
  * @brief Mainly the driver part of the MATRIX module, which includes \b MATRIX \b Register 
  * \b Constants, \b MATRIX \b Exported \b Constants, \b MATRIX \b Exported \b Struct, \b MATRIX
  * \b Data \b transfers \b functions, \b MATRIX \b Initialization \b and \b MATRIX \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup MATRIX_LL_Register_Constants MATRIX LL Register Constants
  * @ingroup  MATRIX_LL_Driver
  * @brief    MATRIX LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the MATRIX 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the MATRIX register, mainly for convenience. Understand the 
    configuration of the MATRIX.
    
@endverbatim
  *
  * @{
  */

/***** MATRIX_EN *****/
/*! MATRIX CHX_CFG_EN, Want to configure CH3_ENABLE,this bit must write1
 */
#define LL_MAXTRIX_CFG_EN_CH(n)                 (1UL << (8+(n)))
/*! MATRIX CHX_EN
 */
#define LL_MAXTRIX_ENABLE_CH(n)                 (0x101UL << (0+(n)))
#define LL_MAXTRIX_DISABLE_CH(n)                (0x100UL << (0+(n)))


/***** MATRIX_PENDING *****/
/*! MATRIX PENDING
 */
#define LL_MAXTRIX_PENDING_CH(n)                (1UL << (0+(n)))


/***** MATRIX_IE *****/
/*! MATRIX interrupt enable
 */
#define LL_MAXTRIX_INT_EN_CH(n)                 (1UL << (0+(n)))


/***** MATRIX_COEF_SEL *****/
/*! MATRIX coef select
 */
#define LL_MAXTRIX_INT_EN_CH(n)                 (1UL << (0+(n)))

/**
  * @}
  */

/** @defgroup MATRIX_LL_Exported_Constants MATRIX LL Exported Constants
  * @ingroup  MATRIX_LL_Driver
  * @brief    MATRIX LL external constant definition
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
/*! MATRIX channel max
 */
#define LL_MAXTRIX_CHN_MAX                      4

/**
  * @}
  */

/** @defgroup MATRIX_LL_Exported_Struct MATRIX LL Exported Struct
  * @ingroup  MATRIX_LL_Driver
  * @brief    MATRIX LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the MATRIX registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_matrix_init. Function, you can configure the MATRIX module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief The 3x3 matrix coefficients are all 16-bit signed numbers, of 
  *        which 15 bits are fixed points. As follows:
@verbatim 
           coef_00   coef_01   coef_02 
           coef_10   coef_11   coef_12 
           coef_20   coef_21   coef_22 
@endverbatim
  */
typedef struct __ll_matrix_3x3_cfg {
    /*! Hardware module supports storage of 2 sets of coefficients
     */
    u16 select;
    /*! coef_00
     */
    s16 coef_00;
    /*! coef_01
     */
    s16 coef_01;
    /*! coef_02
     */
    s16 coef_02;
    /*! coef_10
     */
    s16 coef_10;
    /*! coef_11
     */
    s16 coef_11;
    /*! coef_12
     */
    s16 coef_12;
    /*! coef_20
     */
    s16 coef_20;
    /*! coef_21
     */
    s16 coef_21;
    /*! coef_22
     */
    s16 coef_22;
} TYPE_LL_MATRIX_3X3_CFG;

/**
  * @brief The 3x1 input matrix coefficients are all 16-bit signed numbers,
  *        As follow:
@verbatim 
           coeff_00_in_addr[i] 
           coeff_10_in_addr[i] 
           coeff_20_in_addr[i] 
@endverbatim
  *       The 3x1 output matrix coefficients are all 16-bit signed numbers,
  *       As follow:
@verbatim 
          coeff_00_out_addr[i]
          coeff_10_out_addr[i]
          coeff_20_out_addr[i]
@endverbatim
  *       The matrix multiplication module requires that all DMA addresses 
  *       are 2 byte aligned.
  */
typedef struct __ll_matrix_calc_cfg {
    /*! Hardware supports 4-channel simultaneous operations.
     */
    u8  chn;
    /*! 3x3 matrix coefficient selection, only 2 options of 0 and 1.
     */
    u16 matrix_3x3_select;
    /*! DMA input and output buf subscript offset. You can only configure 
     *  the same offset for the time being.
     */
    u32 index;
    /*! DMA input and output buf cycle length. Can only be configured to the 
     *  same length for the time being.
     */
    u32 dma_points;
    /*! The fixed-point number of the 3x1 input matrix, valid range is 8~15.
     */
    u32 in_frac_width;
    /*! The fixed-point number of the 3x1 output matrix, valid range is 8~15.
     */
    u32 out_frac_width;
    /*! Address 2byte alignment, can only be the address in SRAM1.
     *  @note usage :  
     *        s16 matrix_ibuf0[256] __attribute__((aligned(2), section("SRAM1")));
     */
    u32 dma_src_addr_x;
    /*! Address 2byte alignment, can only be the address in SRAM2.
     */
    u32 dma_src_addr_y;
    /*! Address 2byte alignment, can only be the address in SRAM3.
     */    
    u32 dma_src_addr_z;
    /*! Address 2byte alignment, can only be the address in SRAM1.
     */
    u32 dma_dst_addr_x;
    /*! Address 2byte alignment, can only be the address in SRAM2.
     */
    u32 dma_dst_addr_y;
    /*! Address 2byte alignment, can only be the address in SRAM3.
     */
    u32 dma_dst_addr_z;
} TYPE_LL_MATRIX_CALC_CFG;

/**
  * @brief MATRIX low layer Initialization structure
  */
typedef struct __ll_matrix_init {
    u8 reserved;
} TYPE_LL_MATRIX_INIT;

/**
  * @}
  */

/** @defgroup MATRIX_LL_Interrupt MATRIX LL Interrupt Handle function
  * @brief   MATRIX LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MATRIX  
    Interrupt Handle function.

    how to use?

    The MATRIX interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the MATRIX in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup MATRIX_LL_Inti_Cfg MATRIX LL Initialization And Configuration
  * @brief    MATRIX LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MATRIX data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Matrix Multiplication Module Initialization
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  p_init   : pointer to the init struct TYPE_LL_MATRIX_INIT
  * @retval None
  */
void ll_matrix_init(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_INIT *p_init);

/**
  * @brief  Matrix Multiplication Module Initialization
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @retval None
  */
void ll_matrix_deinit(MATRIX_TypeDef *p_matrix);

/**
  * @brief  3x3 matrix configuration
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  p_cfg    : \ref TYPE_LL_MATRIX_3X3_CFG, Configure 3x3 matrix values and where they 
  *                    re stored.
  * @retval None
  */
void ll_matrix_3x3_coef_config(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_3X3_CFG *p_cfg);

/**
  * @brief  Matrix Multiplication
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  p_cfg    : \ref TYPE_LL_MATRIX_CALC_CFG, Configure matrix computing module.
  * @retval None
  */
void ll_matrix_config(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_CALC_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup MATRIX_LL_Data_Transfers MATRIX LL Data transfers functions
  * @brief    MATRIX LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the MATRIX data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  Start matrix module operation
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : Select the channel to enable
  * @retval None
  * @note   Each call to a function will only cause the module to perform a 
  *         matrix multiplication operation. That is, there will be only one
  *         output at a time.
  */
void ll_matrix_start(MATRIX_TypeDef *p_matrix, u8 chn);

/**
  * @brief  Wait for the completion of the matrix multiplication of a 
  *         particular channel
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : the channel waiting to be calculated.
  * @retval None
  */
void ll_matrix_wait_done_pending(MATRIX_TypeDef *p_matrix, u8 chn);

/**
  * @brief  stop matrix module operation
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : Select the channel to enable
  * @retval None
  * @note   Each call to a function will only cause the module to perform a 
  *         matrix multiplication operation. That is, there will be only one
  *         output at a time.
  */
void ll_matrix_stop(MATRIX_TypeDef *p_matrix, u8 chn);

/**
  * @brief  LL_MATRIX_CHECK_CHN_DONE_INTERRUPT_ENABLE
  * @param  p_matrix: Select the initialized MATRIX group pointer
  * @param  chn     : specify adc chn to set
  * @retval interrupt_enable
  * @note : user can use this function instead of ll_matrix_wait_done_pending(MATRIX_TypeDef *p_matrix, u8 chn);
  */
#define LL_MATRIX_CHECK_CHN_DONE_INTERRUPT_ENABLE(p_matrix, chn)    ((p_matrix)->MATRIX_IE & BIT(chn))

/**
  * @brief  MATRIX get finished pending
  * @param  p_matrix: Select the initialized MATRIX group pointer
  * @param  chn     : specify adc chn to set
  * @retval pending value
  * @note : user can use this function instead of ll_matrix_wait_done_pending(MATRIX_TypeDef *p_matrix, u8 chn);
  */
#define LL_MATRIX_GET_CHN_DONE_PENDING(p_matrix, chn)               ((p_matrix)->MATRIX_PEND & BIT(chn))

/**
  * @brief  MATRIX clear finished pending
  * @param  p_matrix: Select the initialized MATRIX group pointer
  * @param  chn     : specify adc chn to set
  * @retval None
  */
__STATIC_INLINE void ll_matrix_clear_chn_done_pending(MATRIX_TypeDef *p_matrix, u8 chn) {
    p_matrix->MATRIX_PEND |= BIT(chn);
}

/** 
  * @brief  MATRIX ll_matrix_interrupt_enable function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_interrupt_enable(MATRIX_TypeDef *p_matrix, u8 chn) {
    p_matrix->MATRIX_IE |= BIT(chn);
}

/** 
  * @brief  MATRIX ll_matrix_interrupt_disable function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_interrupt_disable(MATRIX_TypeDef *p_matrix, u8 chn) {
    p_matrix->MATRIX_IE &= ~(BIT(chn));
}

/** 
  * @brief  MATRIX ll_matrix_set_index function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @param  out_index: specify output buffer index to set
  * @param  in_index : specify input buffer index to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_set_index(MATRIX_TypeDef *p_matrix, u8 chn, u16 out_index, u16 in_index) {
    MATRIX_CH_TypeDef *p_matrix_ch = &p_matrix->CH[chn];
    p_matrix_ch->LEN  = (out_index << 16) | in_index;
}

/** 
  * @brief  MATRIX ll_matrix_get_index function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @param  out_index: specify output buffer index to set
  * @param  in_index : specify input buffer index to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_get_index(MATRIX_TypeDef *p_matrix, u8 chn, u16 *out_index, u16 *in_index) {
    MATRIX_CH_TypeDef *p_matrix_ch = &p_matrix->CH[chn];
    u32 index = p_matrix_ch->LEN;
    *in_index = index & 0xFFFF;
    *in_index = (index >> 16) & 0xFFFF;
}

/** 
  * @brief  MATRIX ll_matrix_set_dma_src_addr function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @param  addr_x   : specify X buffer addr to set
  * @param  addr_y   : specify Y buffer addr to set
  * @param  addr_z   : specify Z buffer addr to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_set_dma_src_addr(MATRIX_TypeDef *p_matrix, u8 chn, u16 addr_x, u16 addr_y, u16 addr_z) {
    MATRIX_CH_TypeDef *p_matrix_ch = &p_matrix->CH[chn];
    p_matrix_ch->DATAIN_STADR0   = addr_x;
    p_matrix_ch->DATAIN_STADR1   = addr_y;
    p_matrix_ch->DATAIN_STADR2   = addr_z;
}

/** 
  * @brief  MATRIX ll_matrix_set_dma_dst_addr function.
  * @param  p_matrix : pointer to the hardware MATRIX_TypeDef
  * @param  chn      : specify adc chn to set
  * @param  addr_x   : specify X buffer addr to set
  * @param  addr_y   : specify Y buffer addr to set
  * @param  addr_z   : specify Z buffer addr to set
  * @retval none
  */
__STATIC_INLINE void ll_matrix_set_dma_dst_addr(MATRIX_TypeDef *p_matrix, u8 chn, u16 addr_x, u16 addr_y, u16 addr_z) {
    MATRIX_CH_TypeDef *p_matrix_ch = &p_matrix->CH[chn];
    p_matrix_ch->DATAOUT_STADR0   = addr_x;
    p_matrix_ch->DATAOUT_STADR1   = addr_y;
    p_matrix_ch->DATAOUT_STADR2   = addr_z;
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

#endif //__TX_PHE_LL_MATRIX_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
