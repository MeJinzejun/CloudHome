/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_datadma.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_datadma.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup datadma_interface_gr DATADMA Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup DATADMA_LL_Driver DATADMA LL Driver
  * @ingroup  datadma_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup DATADMA_LL_Interrupt DATADMA LL Interrupt Handle function
  * @ingroup  DATADMA_LL_Driver
  * @brief   DATADMA LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup DATADMA_LL_Inti_Cfg DATADMA LL Initialization And Configuration
  * @ingroup  DATADMA_LL_Driver
  * @brief    DATADMA LL Initialization And Configuration
  * @{
  */

/**
  * @brief  DATADMA module initialization function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  p_init   : Module configuration structure pointer
  * @retval None
  */
void ll_datadma_init(DATADMA_TypeDef *p_datadma, TYPE_LL_DATADMA_INIT *p_init)
{
    /* The system resets the DATADMA module, Enable the clock of the module. */
    memset(p_datadma, 0x0, sizeof(DATADMA_TypeDef));

    p_datadma->CON |= LL_DATADMA_CON_PEND;
}

/**
  * @brief  DATADMA module detele initialization function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_deinit(DATADMA_TypeDef *p_datadma)
{
    /* The system disable the DATADMA module, includes turning off the clock for the module. */
    
}

/**
  * @brief  DATADMA module configuration function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @param  p_cfg    : Module configuration structure pointer
  * @retval None
  */
void ll_datadma_config(DATADMA_TypeDef *p_datadma, TYPE_LL_DATADMA_CFG *p_cfg)
{
    u32 datadma_con = 0;
    
    TX_ASSERT_ADDR(p_cfg->src_buf_addr, p_cfg->src_buf_len,
                   SRAM_DATADMA_SRC_MASK, SRAM_DATADMA_SRC_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_len,
                   SRAM_DATADMA_SRC_MASK, SRAM_DATADMA_SRC_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_len,
                   SRAM_DATADMA_DST_MASK, SRAM_DATADMA_DST_ALIGN_MASK);
    
    datadma_con  = p_datadma->CON & LL_DATADMA_CON_IE;
    datadma_con |= p_cfg->mode ? LL_DATADMA_MODE : 0;
    
    p_datadma->CON           = datadma_con;
    p_datadma->SRC_BUF_STADR = LL_DATADMA_SRC_BUF_STADR(p_cfg->src_buf_addr);
    p_datadma->SRC_BUF_LEN   = LL_DATADMA_SRC_BUF_LEN(p_cfg->src_buf_len);
    p_datadma->SRC_DMA_ADR   = LL_DATADMA_SRC_DMA_ADR(p_cfg->dma_src_addr);
    p_datadma->DEST_DMA_ADR  = LL_DATADMA_DEST_DMA_ADR(p_cfg->dma_dst_addr);
    p_datadma->DMA_LEN       = LL_DATADMA_LEN(p_cfg->dma_len);
}

/**
  * @}
  */

/** @defgroup DATADMA_LL_Data_Transfers DATADMA LL Data transfers functions
  * @ingroup  DATADMA_LL_Driver
  * @brief    DATADMA LL Data transfers functions 
  * @{
  */

/**
  * @brief  DATADMA module start function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_start(DATADMA_TypeDef *p_datadma)
{
    p_datadma->CON |= LL_DATADMA_ENABLE;
}

/**
  * @brief  DATADMA module stop function
  * @param  p_datadma: Select the initialized DATADMA group pointer
  * @retval None
  */
void ll_datadma_stop(DATADMA_TypeDef *p_datadma)
{
    p_datadma->CON &= ~(LL_DATADMA_ENABLE);
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
