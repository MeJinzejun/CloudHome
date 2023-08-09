/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_hcc.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the HCC LL firmware functions.
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
#ifndef __TX_PHE_LL_HCC_H
#define __TX_PHE_LL_HCC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup hcc_interface_gr HCC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup HCC_LL_Driver HCC LL Driver
  * @ingroup  hcc_interface_gr
  * @brief Mainly the driver part of the HCC module, which includes \b HCC \b Register 
  * \b Constants, \b HCC \b Exported \b Constants, \b HCC \b Exported \b Struct, \b HCC
  * \b Data \b transfers \b functions, \b HCC \b Initialization \b and \b HCC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup HCC_LL_Register_Constants HCC LL Register Constants
  * @ingroup  HCC_LL_Driver
  * @brief    HCC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the HCC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the HCC register, mainly for convenience. Understand the 
    configuration of the HCC.
    
@endverbatim
  *
  * @{
  */

/***** CRC_CFG Register *****/
/*! HC done Pending, write 1 to clear
 */
#define LL_HCC_PENDING                     (1UL << 31)
/*! HC number = 2*Len
 */
#define LL_HCC_DMA_LEN(n)                  (((n)&0x3F) << 8)
/*! HC range calc enable
 */
#define LL_HCC_RAG_EN                      (1UL << 3)
/*! HC phase calc enable
 */
#define LL_HCC_PHS_EN                      (1UL << 2)
/*! HC interrupt enable
 */
#define LL_HCC_INT_EN                      (1UL << 1)
/*! HC Write 1 to start HC, auto clear
 */
#define LL_HCC_START                       (1UL << 0)


/**
  * @}
  */

/** @defgroup HCC_LL_Exported_Constants HCC LL Exported Constants
  * @ingroup  HCC_LL_Driver
  * @brief    HCC LL external constant definition
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
  * @}
  */

/** @defgroup HCC_LL_Exported_Struct HCC LL Exported Struct
  * @ingroup  HCC_LL_Driver
  * @brief    HCC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the HCC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_hcc_init. Function, you can configure the HCC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief Define the complex type, consistent with the HCC module  
  *        The imaginary part and the real part data are all 16bit signed numbers, among
  *        which there are 15bit fixed-point decimals.
  */
typedef struct __ll_hcc_complex_data {
    /*! Buffer image part, signed 
     */
    s16 img;
    /*! Buffer real part, signed 
     */
    s16 real;
} TYPE_LL_HCC_COMPLEX_DATA;

/**
  * @brief Defines the integer and fractional parts of the range, consistent
  *        with the HCC module.  
  *        The range data is a 12-bit unsigned number with 8 bits fixed-point
  *        decimals.
  */
typedef struct __ll_hcc_rag_split_coeff {
    /*! Decimal part 
     */
    u16 frac            : 8,
    /*! Integer part 
     */
        integer         : 4;
} TYPE_LL_HCC_RAG_SPLIT_COEFF;

/**
  * @brief Use union to define the range data type, consistent with the hcc
  *        module.
  */
typedef union __ll_hcc_range_data {
    /*! One overall 12bit data 
     */
    u16                         coeff : 12;
    /*! Range split coeff 
     */
    TYPE_LL_HCC_RAG_SPLIT_COEFF spilt_coeff;
} TYPE_LL_HCC_RANGE_DATA;

/**
  * @brief Define hcc configuration structure type
  */
typedef struct __ll_hcc_cfg {
    /*! FFT buf start address, requires 8byte alignment.  
     *  permission : SRAM0/4/5/6/7/8/9
     *  @note usage : 
     *        s16 fft_buf[256] __attribute__((aligned(8), section("SRAM0")));
     */
    u32  dma_fft_start_addr;
    /*! PHASE buf start address, requires 8byte alignment.  
     *  permission : SRAM0/10
     */
    u32  dma_phase_start_addr;
    /*! RANGE buf start address, requires 8byte alignment.  
     *  permission : SRAM0/10
     */
    u32  dma_range_start_addr;
    /*! HC number is the actual number of dma calculations. The HCC number must 
     *  be an even number.
     */
    u32  dma_points;
    /*! If range_enable is true, use a custom range coefficient. If range_enable
     *  is false, it means that there is no range coefficient.
     */
    bool range_enable;
    /*! If phase_enable is true, it means that you use a custom phase coefficient.
     *  If phase_enable is false, it means that there is no phase coefficient
     *  (corresponding to 0 in the imaginary part of the phase and 1<<15 in the
     *  real part).
     */
    bool phase_enable;
} TYPE_LL_HCC_CFG;

/**
  * @brief Define hcc init structure type
  */
typedef struct __ll_hcc_init {
    u8 reserved;
} TYPE_LL_HCC_INIT;


/**
  * @}
  */

/** @defgroup HCC_LL_Interrupt HCC LL Interrupt Handle function
  * @brief   HCC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the HCC  
    Interrupt Handle function.

    how to use?

    The HCC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the HCC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup HCC_LL_Inti_Cfg HCC LL Initialization And Configuration
  * @brief    HCC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the HCC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  hcc_init
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_HCC_INIT
  * @retval None
  */
void ll_hcc_init(HCC_TypeDef *p_hcc, TYPE_LL_HCC_INIT *p_init);

/**
  * @brief  hcc_deinit
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_deinit(HCC_TypeDef *p_hcc);

/**
  * @brief  ll_hcc_config
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @param  p_cfg: pointer to the init stuct TYPE_LL_HCC_CFG
  * @retval None
  */
void ll_hcc_config(HCC_TypeDef *p_hcc, TYPE_LL_HCC_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup HCC_LL_Data_Transfers HCC LL Data transfers functions
  * @brief    HCC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the HCC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  hcc_start
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_start(HCC_TypeDef *p_hcc);

/**
  * @brief  Wait for the module to finish computing
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_wait_done_pending(HCC_TypeDef *p_hcc);

/**
  * @brief  hcc stop
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_stop(HCC_TypeDef *p_hcc);

/**
  * @brief  Get the result of the calculation
  * @param  p_hcc   : pointer to the hardware HCC_TypeDef
  * @param  p_result: The complex pointer to save the result.The output results
  *                   are 16bit signed numbers, including 15bit fixed-point decimals
  * @retval None
  * @note
  *     HCC_RESULT_REAL = SUM(range*(fft_real*phs_real - fft_img*phs_img))
  *     HCC_RESULT_IMG= SUM(range*(fft_img*phs_real + fft_real*phs_img))
  */
void ll_hcc_get_result(HCC_TypeDef *p_hcc, TYPE_LL_HCC_COMPLEX_DATA *p_result);

/**
  * @brief  LL_HCC_CHECK_INTERRUPT_ENABLE
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval interrupt_enable
  */
#define LL_HCC_CHECK_INTERRUPT_ENABLE(p_hcc)    ((p_hcc)->HCC_CONTROL & LL_HCC_INT_EN)

/**
  * @brief  Get hcc pending
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval pending
  */
#define LL_HCC_GET_DONE_PENDING(p_hcc)          ((p_hcc)->HCC_CONTROL & LL_HCC_PENDING)

/**
  * @brief  clear hcc pending
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_clear_pending(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL |= LL_HCC_PENDING;
}

/**
  * @brief  ll_hcc_interrupt_enable
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_interrupt_enable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL |= LL_HCC_INT_EN;
}

/**
  * @brief  ll_hcc_interrupt_disable
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_interrupt_disable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL &= ~LL_HCC_INT_EN;
}

/**
  * @brief  HCC does not use the input range factor
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_range_disable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL &= ~LL_HCC_RAG_EN;
}

/**
  * @brief  HCC uses the input range factor
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_range_enable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL |= LL_HCC_RAG_EN;
}

/**
  * @brief  HCC does not use the input phase factor
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_phase_disable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL &= ~LL_HCC_PHS_EN;
}

/**
  * @brief  HCC uses the input phase factor
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_hcc_phase_enable(HCC_TypeDef *p_hcc) {
    p_hcc->HCC_CONTROL |= LL_HCC_PHS_EN;
}

/**
  * @brief  HCC sets the start address of the FFT coefficient
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @param  addr : start address of the FFT coefficient
  * @retval none
  */
__STATIC_INLINE void ll_hcc_fft_start_addr_set(HCC_TypeDef *p_hcc, u32 addr) {
    p_hcc->HCC_STADR_FFT = addr;
}

/**
  * @brief  HCC sets the start address of the phase coefficient
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @param  addr : start address of the phase coefficient
  * @retval none
  */
__STATIC_INLINE void ll_hcc_phase_start_addr_set(HCC_TypeDef *p_hcc, u32 addr) {
    p_hcc->HCC_STADR_PHS = addr;
}

/**
  * @brief  HCC sets the start address of the range coefficient
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @param  addr : start address of the range coefficient
  * @retval none
  */
__STATIC_INLINE void ll_hcc_range_start_addr_set(HCC_TypeDef *p_hcc, u32 addr) {
    p_hcc->HCC_STADR_RAG = addr;
}

/**
  * @brief  The HCC module sets the dma length
  * @param  p_hcc: pointer to the hardware HCC_TypeDef
  * @param  len  : dma length
  * @retval none
  */
__STATIC_INLINE void ll_hcc_dma_len_set(HCC_TypeDef *p_hcc, u32 len) {
    p_hcc->HCC_CONTROL &= ~LL_HCC_DMA_LEN(0x3F);
    p_hcc->HCC_CONTROL |= LL_HCC_DMA_LEN(len);
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

#endif //__TX_PHE_LL_HCC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
