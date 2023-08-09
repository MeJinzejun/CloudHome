/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_datadma.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DATADMA LL firmware functions.
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
#ifndef __TX_PHE_LL_DATADMA_H
#define __TX_PHE_LL_DATADMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup datadma_interface_gr DATADMA Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup DATADMA_LL_Driver DATADMA LL Driver
  * @ingroup  datadma_interface_gr
  * @brief Mainly the driver part of the DATADMA module, which includes \b DATADMA \b Register 
  * \b Constants, \b DATADMA \b Exported \b Constants, \b DATADMA \b Exported \b Struct, \b DATADMA
  * \b Data \b transfers \b functions, \b DATADMA \b Initialization \b and \b DATADMA \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup DATADMA_LL_Register_Constants DATADMA LL Register Constants
  * @ingroup  DATADMA_LL_Driver
  * @brief    DATADMA LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the DATADMA 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the DATADMA register, mainly for convenience. Understand the 
    configuration of the DATADMA.
    
@endverbatim
  *
  * @{
  */

/***** DATADMA_CON *****/
/*! Data dma interrupt enable bit
 */
#define LL_DATADMA_CON_IE                        (1UL << 9)
/*! Data dma finished pending bit
 */
#define LL_DATADMA_CON_PEND                      (1UL << 8)
/*! Data dma mode selection
 */
#define LL_DATADMA_MODE                          (1UL << 1)
/*! Data dma module enable bit
 */
#define LL_DATADMA_ENABLE                        (1UL << 0)


/***** DATADMA_SRC_BUF_STADR *****/
/*! The starting address of the input source data buffer of datadma.
 */
#define LL_DATADMA_SRC_BUF_STADR(n)              (((n)&0xFFFFFFFF) << 0)


/***** DATADMA_SRC_BUF_LEN *****/
/*! The length of the input source data buffer of datadma.
 */
#define LL_DATADMA_SRC_BUF_LEN(n)                (((n)&0xFFFF) << 0)


/***** DATADMA_DST_DMA_ADR *****/
/*! The starting address of the dma source data of datadma.
 */
#define LL_DATADMA_SRC_DMA_ADR(n)                (((n)&0xFFFFFFFF) << 0)


/***** DATADMA_DST_DMA_ADR *****/
/*! The starting address of the dma destination data of datadma.
 */
#define LL_DATADMA_DEST_DMA_ADR(n)               (((n)&0xFFFFFFFF) << 0)


/***** DATADMA_LEN *****/
/*! The dma length of the datadma.
 */
#define LL_DATADMA_LEN(n)                        (((n)&0xFFFF) << 0)


/**
  * @}
  */

/** @defgroup DATADMA_LL_Exported_Constants DATADMA LL Exported Constants
  * @ingroup  DATADMA_LL_Driver
  * @brief    DATADMA LL external constant definition
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
  * @brief Enumeration constant for DATADMA data mode selection
  */
typedef enum {                     
    /*! DMA source data address increases to buffer end and return to 
     *  LL_DATADMA_SRC_BUF_STADR, The buffer of the src_buf_addr 
     *  specified in the loop acquisition structure TYPE_LL_DATADMA_CFG 
     *  is defined as src_buf_len. 
     */
    LL_DATADMA_LOOP_GET_DATA = 0,  
    /*! Starting from the address of dma_src_addr specified in the 
     *  structure TYPE_LL_DATADMA_CFG, the value of the address increases 
     *  backward, and the end address specified by src_buf_addr does 
     *  not stop until the length of dma_len. 
     */
    LL_DATADMA_INC_GET_DATA,
} TYPE_ENUM_LL_DATADMA_MODE;

/**
  * @}
  */

/** @defgroup DATADMA_LL_Exported_Struct DATADMA LL Exported Struct
  * @ingroup  DATADMA_LL_Driver
  * @brief    DATADMA LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the DATADMA registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_datadma_init. Function, you can configure the DATADMA module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief DATADMA low layer configuration structure
  */
typedef struct __ll_datadma_cfg {
    /*! Datadma takes the mode selection of the data. There are two modes. For details, 
     *  please refer to enumeration TYPE_ENUM_LL_DATADMA_MODE. 
     */
    TYPE_ENUM_LL_DATADMA_MODE mode;
    /*! The starting address of the source data buffer. 
     *  @note The src_buf_addr is aligned in byte and can only be located in
     *        SRAM0~SRAM9 and SRAM10.  
     *        Statement example:  
     *          static u8 buf[1024] __attribute__((section("SRAM0")));
     */
    u32                       src_buf_addr;
    /*! The length of the source data buffer, 8bit is a count unit, Valid range is 0~65535. 
     */
    u16                       src_buf_len;
    /*! The starting address of dma starts from which address in the specified address 
     *  of dma_src_addr. 
     *  @note The dma_src_addr is aligned in byte and can only be located in
     *        SRAM0~SRAM9 and SRAM10.  
     *        Statement example:  
     *          static u8 buf[1024] __attribute__((section("SRAM0")));
     */
    u32                       dma_src_addr;
    /*! The starting address of dma starts from which address in the specified address 
     *  of dma_dst_addr. 
     *  @note The dma_dst_addr is aligned in byte and can only be located in
     *        SRAM0~SRAM9 and SRAM10.  
     *        Statement example:  
     *          static u8 buf[1024] __attribute__((section("SRAM0")));
     */
    u32                       dma_dst_addr;
    /*! The length of the dma, 8bit is a count unit, Valid range is 0~65535. 
     */
    u16                       dma_len;
} TYPE_LL_DATADMA_CFG;

/**
  * @brief DATADMA low layer Initialization structure
  */
typedef struct __ll_datadma_init {
    u8 reserved;
} TYPE_LL_DATADMA_INIT;

/**
  * @}
  */

/** @defgroup DATADMA_LL_Interrupt DATADMA LL Interrupt Handle function
  * @brief   DATADMA LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DATADMA  
    Interrupt Handle function.

    how to use?

    The DATADMA interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the DATADMA in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup DATADMA_LL_Inti_Cfg DATADMA LL Initialization And Configuration
  * @brief    DATADMA LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DATADMA data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  DATADMA module initialization function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  p_init   : Module configuration structure pointer
  * @retval None
  */
void ll_datadma_init(DATADMA_TypeDef *p_datadma, TYPE_LL_DATADMA_INIT *p_init);

/**
  * @brief  DATADMA module detele initialization function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_deinit(DATADMA_TypeDef *p_datadma);

/**
  * @brief  DATADMA module configuration function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  p_cfg    : Module configuration structure pointer
  * @retval None
  */
void ll_datadma_config(DATADMA_TypeDef *p_datadma, TYPE_LL_DATADMA_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup DATADMA_LL_Data_Transfers DATADMA LL Data transfers functions
  * @brief    DATADMA LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DATADMA data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  DATADMA module start function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_start(DATADMA_TypeDef *p_datadma);

/**
  * @brief  DATADMA module stop function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_stop(DATADMA_TypeDef *p_datadma);

/**
  * @brief  DATADMA get finished pending
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval pending value
  */
#define LL_DATADMA_GET_DONE_PENDING(p_datadma)    ((p_datadma)->CON & LL_DATADMA_CON_PEND)

/**
  * @brief  DATADMA clear pending
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_clear_done_pending(DATADMA_TypeDef *p_datadma) {
    p_datadma->CON |= LL_DATADMA_CON_PEND;
}

/**
  * @brief  DATADMA wait for the specified channel to finish computing
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_wait_done_pending(DATADMA_TypeDef *p_datadma) {
    while(p_datadma->CON & LL_DATADMA_ENABLE);
    p_datadma->CON |= LL_DATADMA_CON_PEND;
}

/**
  * @brief  DATADMA enable interrupt
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_interrupt_enable(DATADMA_TypeDef *p_datadma) {
    p_datadma->CON |= LL_DATADMA_CON_IE;
}

/**
  * @brief  DATADMA disable interrupt
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_interrupt_disable(DATADMA_TypeDef *p_datadma) {
    p_datadma->CON &= ~(LL_DATADMA_CON_IE);
}

/**
  * @brief  DATADMA dma length setting
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  len      : dma length
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_dma_len_set(DATADMA_TypeDef *p_datadma, u32 len) {
    p_datadma->DMA_LEN = LL_DATADMA_LEN(len);
}

/**
  * @brief  Datadma destination start address setting
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  addr     : destination start address
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_dma_dst_addr_set(DATADMA_TypeDef *p_datadma, u32 addr) {
    p_datadma->DEST_DMA_ADR = LL_DATADMA_DEST_DMA_ADR(addr);
}

/**
  * @brief  Datadma dma source start address setting
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  addr     : dma source start address
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_dma_src_addr_set(DATADMA_TypeDef *p_datadma, u32 addr) {
    p_datadma->SRC_DMA_ADR = LL_DATADMA_SRC_DMA_ADR(addr);
}

/**
  * @brief  Datadma sets the length of the source buf
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  len      : the length of the source buf
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_src_buf_len_set(DATADMA_TypeDef *p_datadma, u32 len) {
    p_datadma->SRC_BUF_LEN = LL_DATADMA_SRC_BUF_LEN(len);
}

/**
  * @brief  Datadma source buf start address setting
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  addr     : source buf start address
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_src_buf_addr_set(DATADMA_TypeDef *p_datadma, u32 addr) {
    p_datadma->SRC_BUF_STADR = LL_DATADMA_SRC_BUF_STADR(addr);
}

/**
  * @brief  Datadma setting mode
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  mode     : takes the mode selection of the data
  * @retval None.
  */
__STATIC_INLINE void ll_datadma_mode_set(DATADMA_TypeDef *p_datadma,
                                         TYPE_ENUM_LL_DATADMA_MODE mode) {
    if(mode == LL_DATADMA_LOOP_GET_DATA) {
        p_datadma->CON &= ~LL_DATADMA_MODE;
    } else {
        p_datadma->CON |= LL_DATADMA_MODE;
    }
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

#endif //__TX_PHE_LL_DATADMA_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
