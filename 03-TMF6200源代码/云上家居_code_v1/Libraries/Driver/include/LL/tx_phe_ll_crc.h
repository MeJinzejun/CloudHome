/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_crc.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the CRC LL firmware functions.
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
#ifndef __TX_PHE_LL_CRC_H
#define __TX_PHE_LL_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup crc_interface_gr CRC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup CRC_LL_Driver CRC LL Driver
  * @ingroup  crc_interface_gr
  * @brief Mainly the driver part of the CRC module, which includes \b CRC \b Register 
  * \b Constants, \b CRC \b Exported \b Constants, \b CRC \b Exported \b Struct, \b CRC
  * \b Data \b transfers \b functions, \b CRC \b Initialization \b and \b CRC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup CRC_LL_Register_Constants CRC LL Register Constants
  * @ingroup  CRC_LL_Driver
  * @brief    CRC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the CRC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the CRC register, mainly for convenience. Understand the 
    configuration of the CRC.
    
@endverbatim
  *
  * @{
  */

/***** CRC_CFG Register *****/
/*! CRC interrupt enable 
 */
#define LL_CRC_INT_EN                          (1UL << 0)
/*! CRC shift bit direction 
 */
#define LL_CRC_BIT_ORDER_LEFT                  (1UL << 1)
#define LL_CRC_BIT_ORDER_RIGHT                 (0UL << 1)
/*! CRC POLY width: 5/7/8/16/32 
 */
#define LL_CRC_POLY_BITS(n)                    (((n)&0x3F) << 8)
/*! CRC DMA data wait clock every time 
 */
#define LL_CRC_DMAWAIT_CLOCK(n)                (((n)&0x7) << 16)


/***** CRC_KST Register *****/
/*! CRC pending clear 
 */
#define LL_CRC_DMA_PENDING_CLR                 (1UL << 0)


/***** CRC_STA Register *****/
/*! CRC pending 
 */
#define LL_CRC_DMA_PENDING                     (1UL << 0) 


/***** CRC_INIT Register *****/
/***** CRC_INV Register *****/
/***** CRC_POLY Register *****/
/***** CRC_DMA_ADDR Register *****/
/***** CRC_DMA_LEN Register *****/
/***** CRC_CRC_OUT Register *****/


/**
  * @}
  */

/** @defgroup CRC_LL_Exported_Constants CRC LL Exported Constants
  * @ingroup  CRC_LL_Driver
  * @brief    CRC LL external constant definition
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
  * @brief CRC poly shift dir enum
  */
typedef enum {
    /*! CRC poly shift bit direction : right
     */
    LL_CRC_POLY_SHIFT_RIGHT = 0,
    /*! CRC poly shift bit direction : left
     */
    LL_CRC_POLY_SHIFT_LEFT  = 1,
} TYPE_ENUM_LL_CRC_POLY_SHIFT_DIR;

/**
  * @brief CRC poly width enum
  */
typedef enum {
    /*! CRC poly bit width : 5
     */
    LL_CRC_POLY_5BIT  = 5,
    /*! CRC poly bit width : 7
     */
    LL_CRC_POLY_7BIT  = 7,
    /*! CRC poly bit width : 8
     */
    LL_CRC_POLY_8BIT  = 8,
    /*! CRC poly bit width : 16
     */
    LL_CRC_POLY_16BIT = 16,
    /*! CRC poly bit width : 32
     */
    LL_CRC_POLY_32BIT = 32,
} TYPE_ENUM_LL_CRC_POLY_WIDTH;


/***** LL API AND DRIVER API *****/


/**
  * @}
  */

/** @defgroup CRC_LL_Exported_Struct CRC LL Exported Struct
  * @ingroup  CRC_LL_Driver
  * @brief    CRC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the CRC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_crc_init. Function, you can configure the CRC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief CRC config struct
  */
typedef struct __ll_crc_cfg {
    /*! crc init.
     */
    u32                             init_val;
    /*! crc poly.
     */
    u32                             poly;
    /*! crc poly bit 5/7/8/16/32.
     */
    TYPE_ENUM_LL_CRC_POLY_WIDTH     poly_bits;
    /*! crc poly shift dir: 0 for right, 1 for left.
     */
    TYPE_ENUM_LL_CRC_POLY_SHIFT_DIR poly_shift_dir;
    /*! crc wait clocks (for CRC32 run speed limit to 32Mhz).
     */
    u32                             crc32_wait_clks;
    /*! crc out value invert.
     */
    u32                             out_invert;
} TYPE_LL_CRC_CFG;

/**
  * @brief CRC init struct
  */
typedef struct __ll_crc_init {
    u8  reserved;
} TYPE_LL_CRC_INIT;

/**
  * @}
  */

/** @defgroup CRC_LL_Interrupt CRC LL Interrupt Handle function
  * @brief   CRC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CRC  
    Interrupt Handle function.

    how to use?

    The CRC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the CRC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup CRC_LL_Inti_Cfg CRC LL Initialization And Configuration
  * @brief    CRC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CRC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  crc_init
  * @param  p_crc : pointer to the hardware CRC_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_CRC_INIT
  * @retval None
  */
void ll_crc_init(CRC_TypeDef *p_crc, TYPE_LL_CRC_INIT *p_init);

/**
  * @brief  crc_deinit
  * @param  p_crc : pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_deinit(CRC_TypeDef *p_crc);

/**
  * @brief  ll_crc_config & start crc calc
  * @param  p_crc : pointer to the hardware CRC_TypeDef
  * @param  p_cfg : pointer to the init stuct TYPE_LL_CRC_CFG
  * @retval None
  */
void ll_crc_config(CRC_TypeDef *p_crc, TYPE_LL_CRC_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup CRC_LL_Data_Transfers CRC LL Data transfers functions
  * @brief    CRC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CRC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  crc_start
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @param  addr : CRC calc start address 32bit align
  * @param  len  : CRC calc len , unit : 1byte
  * @note   The dma_addr is aligned in 4bytes and can only be located in
  *         SRAM0/SRAM10.  
  *         Statement example: tatic u8 buf[1024] __attribute__((aligned(4),section("SRAM0")));
  * @retval None
  */
void ll_crc_start(CRC_TypeDef *p_crc, u32 addr, u32 len);

/**
  * @brief  crc wait
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_wait_done_pending(CRC_TypeDef *p_crc);

/**
  * @brief  crc stop
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_stop(CRC_TypeDef *p_crc);

/**
  * @brief  Get crc result
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval crc result
  */
#define LL_CRC_GET_CHECK_VAL(p_crc)             ((p_crc)->CRC_OUT)

/**
  * @brief  LL_CRC_CHECK_INTERRUPT_ENABLE
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval interrupt_enable
  */
#define LL_CRC_CHECK_INTERRUPT_ENABLE(p_crc)    ((p_crc)->CRC_CFG & LL_CRC_INT_EN)

/**
  * @brief  Get crc pending
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval pending
  */
#define LL_CRC_GET_DONE_PENDING(p_crc)          ((p_crc)->CRC_STA & LL_CRC_DMA_PENDING)

/**
  * @brief  clear crc pending
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_crc_clear_done_pending(CRC_TypeDef *p_crc) {
    p_crc->CRC_KST = LL_CRC_DMA_PENDING_CLR;
}

/**
  * @brief  ll_crc_interrupt_enable
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_crc_interrupt_enable(CRC_TypeDef *p_crc) {
    p_crc->CRC_CFG |= LL_CRC_INT_EN;
}

/**
  * @brief  ll_crc_interrupt_disable
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval none
  */
__STATIC_INLINE void ll_crc_interrupt_disable(CRC_TypeDef *p_crc) {
    p_crc->CRC_CFG &= ~LL_CRC_INT_EN;
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

#endif //__TX_PHE_LL_CRC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
