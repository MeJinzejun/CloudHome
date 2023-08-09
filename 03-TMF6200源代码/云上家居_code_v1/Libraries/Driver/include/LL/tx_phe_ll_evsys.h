/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_evsys.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the EVSYS LL firmware functions.
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
#ifndef __TX_PHE_LL_EVSYS_H
#define __TX_PHE_LL_EVSYS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup evsys_interface_gr EVSYS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup EVSYS_LL_Driver EVSYS LL Driver
  * @ingroup  evsys_interface_gr
  * @brief Mainly the driver part of the EVSYS module, which includes \b EVSYS \b Register 
  * \b Constants, \b EVSYS \b Exported \b Constants, \b EVSYS \b Exported \b Struct, \b EVSYS
  * \b Data \b transfers \b functions, \b EVSYS \b Initialization \b and \b EVSYS \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 
/** @defgroup EVSYS_LL_Register_Constants EVSYS LL Register Constants
  * @ingroup  EVSYS_LL_Driver
  * @brief    EVSYS LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the EVSYS 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the EVSYS register, mainly for convenience. Understand the 
    configuration of the EVSYS.
    
@endverbatim
  *
  * @{
  */

/***** CH_ENA *****/
/*! evsys channel enable
 */
#define LL_EVSYS_CHN_ENABLE(n)                    (((n)&0xFFFF) << 0)


/***** CH_INT_ENA *****/
/*! evsys channel interrupt enable
 */
#define LL_EVSYS_CHN_INTR_ENABLE(n)               (((n)&0xFFFF) << 0)


/***** CPU_KST *****/
/*! evsys channel kick start
 */
#define LL_EVSYS_CHN_KICK_START_L(n)              (((n)&0xFFFF) << 0)
/*! evsys channel kick start
 */
#define LL_EVSYS_CHN_KICK_START_H(n)              (((n)&0xFFFF) << 16)


/***** CH_PND_CLR *****/
/*! evsys channel cpu clear pending
 */
#define LL_EVSYS_CHN_PND_CLR(n)                   (((n)&0xFFFF) << 0)


/***** CH_CPU_PND *****/
/*! evsys channel cpu pending
 */
#define LL_EVSYS_CHN_CPU_PND(n)                   (((n)&0xFFFF) << 0)


/***** CH_HW_PND *****/
/*! evsys channel hardware pending
 */
#define LL_EVSYS_CHN_HW_PND(n)                    (((n)&0xFFFF) << 0)


/***** CH_MODE *****/
/*! evsys channel mode select
 */
#define LL_EVSYS_CHN_MODE(n)                      (((n)&0xFFFF) << 0)


/***** SRC_CH_CON0 *****/
/*! evsys source channel num select(low chn: 0~31)
 */
#define LL_EVSYS_SRC_CHN_NUM0(n)                  (((n)&0xFFFFFFFF) << 0)


/***** SRC_CH_CON1 *****/
/*! evsys source channel counter set
 */
#define LL_EVSYS_SRC_CHN_CNT(n)                   (((n)&0xFFFF) << 16)
/*! evsys source channel num select(high chn: 32~43)
 */
#define LL_EVSYS_SRC_CHN_NUM1(n)                  (((n)&0x7FF) << 0)


/***** DST_CH_CON0 *****/
/*! evsys destination channel enable
 */
#define LL_EVSYS_DST_CHN_ENABLE(n)                (((n)&0xFFFFFFFF) << 0)

/**
  * @}
  */

/** @defgroup EVSYS_LL_Exported_Constants EVSYS LL Exported Constants
  * @ingroup  EVSYS_LL_Driver
  * @brief    EVSYS LL external constant definition
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
  * @brief Enumeration constant for The EVSYS source event selection
  */
typedef enum {
    /*! EVSYS source channel trigger signal selection(IIR0     ): 0
     */
    LL_EVSYS_SRC_EVENT_IIR0 = 0,
    /*! EVSYS source channel trigger signal selection(IIR1     ): 1
     */
    LL_EVSYS_SRC_EVENT_IIR1,
    /*! EVSYS source channel trigger signal selection(IIR2     ): 2
     */
    LL_EVSYS_SRC_EVENT_IIR2,
    /*! EVSYS source channel trigger signal selection(FIR0     ): 3
     */
    LL_EVSYS_SRC_EVENT_FIR0,
    /*! EVSYS source channel trigger signal selection(FIR1     ): 4
     */
    LL_EVSYS_SRC_EVENT_FIR1,
    /*! EVSYS source channel trigger signal selection(FIR2     ): 5
     */
    LL_EVSYS_SRC_EVENT_FIR2,
    /*! EVSYS source channel trigger signal selection(ARCTAN0  ): 6
     */
    LL_EVSYS_SRC_EVENT_ARCTAN0,
    /*! EVSYS source channel trigger signal selection(ARCTAN1  ): 7
     */
    LL_EVSYS_SRC_EVENT_ARCTAN1,
    /*! EVSYS source channel trigger signal selection(ARCTAN2  ): 8
     */
    LL_EVSYS_SRC_EVENT_ARCTAN2,
    /*! EVSYS source channel trigger signal selection(DFTTRANS0): 9
     */
    LL_EVSYS_SRC_EVENT_DFTTRANS0,
    /*! EVSYS source channel trigger signal selection(DFTTRANS1): 10
     */
    LL_EVSYS_SRC_EVENT_DFTTRANS1,
    /*! EVSYS source channel trigger signal selection(DFTTRANS2): 11
     */
    LL_EVSYS_SRC_EVENT_DFTTRANS2,
    /*! EVSYS source channel trigger signal selection(FFT0     ): 12
     */
    LL_EVSYS_SRC_EVENT_FFT0,
    /*! EVSYS source channel trigger signal selection(FFT1     ): 13
     */
    LL_EVSYS_SRC_EVENT_FFT1,
    /*! EVSYS source channel trigger signal selection(FFT2     ): 14
     */
    LL_EVSYS_SRC_EVENT_FFT2,
    /*! EVSYS source channel trigger signal selection(FFT0_DMA ): 15
     */
    LL_EVSYS_SRC_EVENT_FFT0_DMA,
    /*! EVSYS source channel trigger signal selection(FFT1_DMA ): 16
     */
    LL_EVSYS_SRC_EVENT_FFT1_DMA,
    /*! EVSYS source channel trigger signal selection(FFT2_DMA ): 17
     */
    LL_EVSYS_SRC_EVENT_FFT2_DMA,
    /*! EVSYS source channel trigger signal selection(MATRIX0  ): 18
     */
    LL_EVSYS_SRC_EVENT_MATRIX0,
    /*! EVSYS source channel trigger signal selection(MATRIX1  ): 19
     */
    LL_EVSYS_SRC_EVENT_MATRIX1,
    /*! EVSYS source channel trigger signal selection(MATRIX2  ): 20
     */
    LL_EVSYS_SRC_EVENT_MATRIX2,
    /*! EVSYS source channel trigger signal selection(MATRIX3  ): 21
     */
    LL_EVSYS_SRC_EVENT_MATRIX3,
    /*! EVSYS source channel trigger signal selection(RMS0     ): 22
     */
    LL_EVSYS_SRC_EVENT_RMS0,
    /*! EVSYS source channel trigger signal selection(RMS1     ): 23
     */
    LL_EVSYS_SRC_EVENT_RMS1,
    /*! EVSYS source channel trigger signal selection(RMS2     ): 24
     */
    LL_EVSYS_SRC_EVENT_RMS2,
    /*! EVSYS source channel trigger signal selection(SINCOS0  ): 25
     */
    LL_EVSYS_SRC_EVENT_SINCOS0,
    /*! EVSYS source channel trigger signal selection(SINCOS1  ): 26
     */
    LL_EVSYS_SRC_EVENT_SINCOS1,
    /*! EVSYS source channel trigger signal selection(DATA_DMA ): 27
     */
    LL_EVSYS_SRC_EVENT_DATA_DMA,
    /*! EVSYS source channel trigger signal selection(HCC      ): 28
     */
    LL_EVSYS_SRC_EVENT_HCC,
    /*! EVSYS source channel trigger signal selection(SARADC0  ): 29
     */
    LL_EVSYS_SRC_EVENT_SARADC0,
    /*! EVSYS source channel trigger signal selection(SARADC1  ): 30
     */
    LL_EVSYS_SRC_EVENT_SARADC1,
    /*! EVSYS source channel trigger signal selection(SARADC2  ): 31
     */
    LL_EVSYS_SRC_EVENT_SARADC2,
    /*! EVSYS source channel trigger signal selection(SARADC3  ): 32
     */
    LL_EVSYS_SRC_EVENT_SARADC3,
    /*! EVSYS source channel trigger signal selection(SARADC4  ): 33
     */
    LL_EVSYS_SRC_EVENT_SARADC4,
    /*! EVSYS source channel trigger signal selection(SARADC5  ): 34
     */
    LL_EVSYS_SRC_EVENT_SARADC5,
    /*! EVSYS source channel trigger signal selection(SARADC6  ): 35
     */
    LL_EVSYS_SRC_EVENT_SARADC6,
    /*! EVSYS source channel trigger signal selection(SARADC7  ): 36
     */
    LL_EVSYS_SRC_EVENT_SARADC7,
    /*! EVSYS source channel trigger signal selection(SARADC8  ): 37
     */
    LL_EVSYS_SRC_EVENT_SARADC8,
    /*! EVSYS source channel trigger signal selection(SARADC9  ): 38
     */
    LL_EVSYS_SRC_EVENT_SARADC9,
    /*! EVSYS source channel trigger signal selection(SARADC10 ): 39
     */
    LL_EVSYS_SRC_EVENT_SARADC10,
    /*! EVSYS source channel trigger signal selection(SARADC11 ): 40
     */
    LL_EVSYS_SRC_EVENT_SARADC11,
    /*! EVSYS source channel trigger signal selection(SARADC12 ): 41
     */
    LL_EVSYS_SRC_EVENT_SARADC12,
    /*! EVSYS source channel trigger signal selection(SARADC13 ): 42
     */
    LL_EVSYS_SRC_EVENT_SARADC13,
    /*! EVSYS source channel trigger signal selection(idle     ): 65
     */
    LL_EVSYS_SRC_EVEVT_IDLE = 65,
    /*! EVSYS source channel trigger signal selection(CPU kick ): 66
     */
    LL_EVSYS_SRC_EVEVT_CPU  = 66,
} TYPE_ENUM_LL_EVSYS_SRC_EVENT_SEL;

/**
  * @brief Enumeration constant for The EVSYS destination event selection
  */
typedef enum {
    /*! EVSYS destination channel trigger signal selection(IIR0     ): 0
     */
    LL_EVSYS_DST_EVENT_IIR0 = 0,
    /*! EVSYS destination channel trigger signal selection(IIR1     ): 1
     */
    LL_EVSYS_DST_EVENT_IIR1,
    /*! EVSYS destination channel trigger signal selection(IIR2     ): 2
     */
    LL_EVSYS_DST_EVENT_IIR2,
    /*! EVSYS destination channel trigger signal selection(FIR0     ): 3
     */
    LL_EVSYS_DST_EVENT_FIR0,
    /*! EVSYS destination channel trigger signal selection(FIR1     ): 4
     */
    LL_EVSYS_DST_EVENT_FIR1,
    /*! EVSYS destination channel trigger signal selection(FIR2     ): 5
     */
    LL_EVSYS_DST_EVENT_FIR2,
    /*! EVSYS destination channel trigger signal selection(ARCTAN0  ): 6
     */
    LL_EVSYS_DST_EVENT_ARCTAN0,
    /*! EVSYS destination channel trigger signal selection(ARCTAN1  ): 7
     */
    LL_EVSYS_DST_EVENT_ARCTAN1,
    /*! EVSYS destination channel trigger signal selection(ARCTAN2  ): 8
     */
    LL_EVSYS_DST_EVENT_ARCTAN2,
    /*! EVSYS destination channel trigger signal selection(DFTTRANS0): 9
     */
    LL_EVSYS_DST_EVENT_DFTTRANS0,
    /*! EVSYS destination channel trigger signal selection(DFTTRANS1): 10
     */
    LL_EVSYS_DST_EVENT_DFTTRANS1,
    /*! EVSYS destination channel trigger signal selection(DFTTRANS2): 11
     */
    LL_EVSYS_DST_EVENT_DFTTRANS2,
    /*! EVSYS destination channel trigger signal selection(FFT0     ): 12
     */
    LL_EVSYS_DST_EVENT_FFT0,
    /*! EVSYS destination channel trigger signal selection(FFT1     ): 13
     */
    LL_EVSYS_DST_EVENT_FFT1,
    /*! EVSYS destination channel trigger signal selection(FFT2     ): 14
     */
    LL_EVSYS_DST_EVENT_FFT2,
    /*! EVSYS destination channel trigger signal selection(FFT0_DMA ): 15
     */
    LL_EVSYS_DST_EVENT_FFT0_DMA,
    /*! EVSYS destination channel trigger signal selection(FFT1_DMA ): 16
     */
    LL_EVSYS_DST_EVENT_FFT1_DMA,
    /*! EVSYS destination channel trigger signal selection(FFT2_DMA ): 17
     */
    LL_EVSYS_DST_EVENT_FFT2_DMA,
    /*! EVSYS destination channel trigger signal selection(MATRIX0  ): 18
     */
    LL_EVSYS_DST_EVENT_MATRIX0,
    /*! EVSYS destination channel trigger signal selection(MATRIX1  ): 19
     */
    LL_EVSYS_DST_EVENT_MATRIX1,
    /*! EVSYS destination channel trigger signal selection(MATRIX2  ): 20
     */
    LL_EVSYS_DST_EVENT_MATRIX2,
    /*! EVSYS destination channel trigger signal selection(MATRIX3  ): 21
     */
    LL_EVSYS_DST_EVENT_MATRIX3,
    /*! EVSYS destination channel trigger signal selection(RMS0     ): 22
     */
    LL_EVSYS_DST_EVENT_RMS0,
    /*! EVSYS destination channel trigger signal selection(RMS1     ): 23
     */
    LL_EVSYS_DST_EVENT_RMS1,
    /*! EVSYS destination channel trigger signal selection(RMS2     ): 24
     */
    LL_EVSYS_DST_EVENT_RMS2,
    /*! EVSYS destination channel trigger signal selection(SINCOS0  ): 25
     */
    LL_EVSYS_DST_EVENT_SINCOS0,
    /*! EVSYS destination channel trigger signal selection(SINCOS1  ): 26
     */
    LL_EVSYS_DST_EVENT_SINCOS1,
    /*! EVSYS destination channel trigger signal selection(DATA_DMA ): 27
     */
    LL_EVSYS_DST_EVENT_DATA_DMA,
    /*! EVSYS destination channel trigger signal selection(HCC      ): 28
     */
    LL_EVSYS_DST_EVENT_HCC,
    /*! EVSYS destination channel trigger signal selection(idle     ): 65
     */
    LL_EVSYS_DST_EVENT_IDLE = 65,
} TYPE_ENUM_LL_EVSYS_DST_EVENT_SEL;

/**
  * @brief Enumeration constant for The EVSYS channel selection
  */
typedef enum {
    /*! EVSYS channel select: 0
     */
    LL_EVSYS_CHN_NUM0 = 0,
    /*! EVSYS channel select: 1
     */
    LL_EVSYS_CHN_NUM1,
    /*! EVSYS channel select: 2
     */
    LL_EVSYS_CHN_NUM2,
    /*! EVSYS channel select: 3
     */
    LL_EVSYS_CHN_NUM3,
    /*! EVSYS channel select: 4
     */
    LL_EVSYS_CHN_NUM4,
    /*! EVSYS channel select: 5
     */
    LL_EVSYS_CHN_NUM5,
    /*! EVSYS channel select: 6
     */
    LL_EVSYS_CHN_NUM6,
    /*! EVSYS channel select: 7
     */
    LL_EVSYS_CHN_NUM7,
    /*! EVSYS channel select: 8
     */
    LL_EVSYS_CHN_NUM8,
    /*! EVSYS channel select: 9
     */
    LL_EVSYS_CHN_NUM9,
    /*! EVSYS channel select: 10
     */
    LL_EVSYS_CHN_NUM10,
    /*! EVSYS channel select: 11
     */
    LL_EVSYS_CHN_NUM11,
    /*! EVSYS channel select: 12
     */
    LL_EVSYS_CHN_NUM12,
    /*! EVSYS channel select: 13
     */
    LL_EVSYS_CHN_NUM13,
    /*! EVSYS channel select: 14
     */
    LL_EVSYS_CHN_NUM14,
    /*! EVSYS channel select: 15
     */
    LL_EVSYS_CHN_NUM15,
} TYPE_ENUM_LL_EVSYS_CHN;

/**
  * @}
  */

/** @defgroup EVSYS_LL_Exported_Struct EVSYS LL Exported Struct
  * @ingroup  EVSYS_LL_Driver
  * @brief    EVSYS LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the EVSYS registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_evsys_init. Function, you can configure the EVSYS module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief EVSYS low layer configuration structure
  */
typedef struct _ll_evsys_cfg {
    /*! evsys channel num
     *  Please refer to the enumeration TYPE_ENUM_LL_EVSYS_CHN for details.
     */
    TYPE_ENUM_LL_EVSYS_CHN           chn;
    /*! source channel
     *  Please refer to the enumeration TYPE_ENUM_LL_EVSYS_EVENT_SEL for details.
     */
    TYPE_ENUM_LL_EVSYS_SRC_EVENT_SEL src_chn;
    /*! destination channel
     *  Please refer to the enumeration TYPE_ENUM_LL_EVSYS_EVENT_SEL for details.
     */
    TYPE_ENUM_LL_EVSYS_DST_EVENT_SEL dst_chn;
    /*! the source data is triggered once after count calculations
     */
    u16                              src_kick_cnt;
} TYPE_LL_EVSYS_CFG;

/**
  * @brief EVSYS low layer Initialization structure
  */
typedef struct _ll_evsys_init {
    u8 reserved;
} TYPE_LL_EVSYS_INIT;

/**
  * @}
  */

/** @defgroup EVSYS_LL_Interrupt EVSYS LL Interrupt Handle function
  * @brief   EVSYS LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EVSYS  
    Interrupt Handle function.

    how to use?

    The EVSYS interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the EVSYS in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup EVSYS_LL_Inti_Cfg EVSYS LL Initialization And Configuration
  * @brief    EVSYS LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EVSYS data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer EVSYS module initialization
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @param  p_init : Module configuration structure pointer(TYPE_LL_EVSYS_INIT)
  * @retval None
  */
void ll_evsys_init(EVSYS_TypeDef *p_evsys, TYPE_LL_EVSYS_INIT *p_init);

/**
  * @brief  Low layer EVSYS module detele initialization
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @retval None
  */
void ll_evsys_deinit(EVSYS_TypeDef *p_evsys);

/**
  * @brief  Low layer EVSYS module configuration
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @param  p_cfg  : Module configuration structure pointer(TYPE_LL_EVSYS_CFG)
  * @retval None
  */
void ll_evsys_config(EVSYS_TypeDef *p_evsys, TYPE_LL_EVSYS_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup EVSYS_LL_Data_Transfers EVSYS LL Data transfers functions
  * @brief    EVSYS LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EVSYS data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  EVSYS module start function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
void ll_evsys_start(EVSYS_TypeDef *p_evsys, u16 chn_bits);

/**
  * @brief  EVSYS module stop function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
void ll_evsys_stop(EVSYS_TypeDef *p_evsys, u16 chn_bits);

/**
  * @brief  EVSYS module get cpu pending function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval pending value
  */
#define LL_EVSYS_GET_CPU_PENDING(p_evsys, chn_bits)         ((p_evsys)->CH_CPU_PND & LL_EVSYS_CHN_CPU_PND(chn_bits))

/**
  * @brief  EVSYS module get hardware pending function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval pending value
  */
#define LL_EVSYS_GET_HARDWARE_PENDING(p_evsys, chn_bits)    ((p_evsys)->CH_HW_PND & LL_EVSYS_CHN_HW_PND(chn_bits))

/**
  * @brief  EVSYS module clear hardware and CPU pending function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval pending value
  */
__STATIC_INLINE void ll_evsys_clear_pending(EVSYS_TypeDef *p_evsys, u16 chn_bits) {
    p_evsys->CH_PND_CLR |= LL_EVSYS_CHN_PND_CLR(chn_bits);
}

/**
  * @brief  EVSYS module wait CPU pending function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval pending value
  */
__STATIC_INLINE void ll_evsys_wait_cpu_pending(EVSYS_TypeDef *p_evsys, u16 chn_bits) {
    while(chn_bits != (p_evsys->CH_CPU_PND & LL_EVSYS_CHN_CPU_PND(chn_bits)));
    p_evsys->CH_PND_CLR |= LL_EVSYS_CHN_PND_CLR(chn_bits);
}

/**
  * @brief  EVSYS module wait hardware pending function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval pending value
  */
__STATIC_INLINE void ll_evsys_wait_hardware_pending(EVSYS_TypeDef *p_evsys, u16 chn_bits) {
    while(chn_bits != (p_evsys->CH_HW_PND & LL_EVSYS_CHN_HW_PND(chn_bits)));
    p_evsys->CH_PND_CLR |= LL_EVSYS_CHN_PND_CLR(chn_bits);
}

/**
  * @brief  EVSYS module interrupt enable function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_evsys_interrupt_enable(EVSYS_TypeDef *p_evsys, u16 chn_bits) {
    p_evsys->CH_INT_ENA |= LL_EVSYS_CHN_INTR_ENABLE(chn_bits);
}

/**
  * @brief  EVSYS module interrupt disable function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_evsys_interrupt_disable(EVSYS_TypeDef *p_evsys, u16 chn_bits) {
    p_evsys->CH_INT_ENA &= ~(LL_EVSYS_CHN_INTR_ENABLE(chn_bits));
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

#endif //__TX_PHE_LL_EVSYS_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
