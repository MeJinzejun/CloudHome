/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_arctan.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_arctan.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup arctan_interface_gr ARCTAN Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup ARCTAN_LL_Driver ARCTAN LL Driver
  * @ingroup  arctan_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup ARCTAN_LL_Interrupt ARCTAN LL Interrupt Handle function
  * @ingroup  ARCTAN_LL_Driver
  * @brief   ARCTAN LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup ARCTAN_LL_Inti_Cfg ARCTAN LL Initialization And Configuration
  * @ingroup  ARCTAN_LL_Driver
  * @brief    ARCTAN LL Initialization And Configuration
  * @{
  */

/**
  * @brief  ARCTAN module initialization function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_arctan_init(ARCTAN_TypeDef *p_arctan, TYPE_LL_ARCTAN_INIT *p_init)
{
    /* The system resets the ARCTAN module, Enable the clock of the module. */
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
    
    memset(p_arctan, 0x0, sizeof(ARCTAN_TypeDef));
    p_arctan->CON |= LL_ARCTAN_CON_PEND;
}

/**
  * @brief  ARCTAN module detele initialization function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_deinit(ARCTAN_TypeDef *p_arctan)
{
    /* The system disable the ARCTAN module, includes turning off the clock for the module. */
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
}

/**
  * @brief  ARCTAN module configuration function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_arctan_config(ARCTAN_TypeDef *p_arctan, TYPE_LL_ARCTAN_CFG *p_cfg)
{
    u32 arctan_con = 0;
    
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
    
    if(p_cfg->work_mode == LL_ARCTAN_MODE_DMA) {
        if(p_arctan == ARCTAN0) {
            TX_ASSERT_ADDR(p_cfg->cos_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_0_MASK, SRAM_ARC_TRIANGLE_0_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->sin_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_0_MASK, SRAM_ARC_TRIANGLE_0_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_0_MASK, SRAM_ARC_TRIANGLE_0_ALIGN_MASK);
        } else if(p_arctan == ARCTAN1) {
            TX_ASSERT_ADDR(p_cfg->cos_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_1_MASK, SRAM_ARC_TRIANGLE_1_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->sin_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_1_MASK, SRAM_ARC_TRIANGLE_1_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_1_MASK, SRAM_ARC_TRIANGLE_1_ALIGN_MASK);
        } else if(p_arctan == ARCTAN2) {
            TX_ASSERT_ADDR(p_cfg->cos_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_2_MASK, SRAM_ARC_TRIANGLE_2_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->sin_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_2_MASK, SRAM_ARC_TRIANGLE_2_ALIGN_MASK);
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_ARC_TRIANGLE_2_MASK, SRAM_ARC_TRIANGLE_2_ALIGN_MASK);
        }
    }
    
    arctan_con  = p_arctan->CON & LL_ARCTAN_CON_IE;
    arctan_con |= p_cfg->work_mode ? LL_ARCTAN_CON_DMA_EN : 0;
    arctan_con |= p_cfg->mode ? LL_ARCTAN_CON_SAVE_TOGETHER : 0;
    
    p_arctan->CON        = arctan_con;
    p_arctan->DMA0_STADR = LL_ARCTAN_DMA0_START(p_cfg->cos_dma_src_addr);
    p_arctan->DMA1_STADR = LL_ARCTAN_DMA1_START(p_cfg->sin_dma_src_addr);
    p_arctan->DMA2_STADR = LL_ARCTAN_DMA2_START(p_cfg->dma_dst_addr);
    p_arctan->DMA_LEN    = LL_ARCTAN_DMA_LEN(p_cfg->dma_points);
}

/**
  * @}
  */

/** @defgroup ARCTAN_LL_Data_Transfers ARCTAN LL Data transfers functions
  * @ingroup  ARCTAN_LL_Driver
  * @brief    ARCTAN LL Data transfers functions 
  * @{
  */

/**
  * @brief  ARCTAN module start function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_start(ARCTAN_TypeDef *p_arctan)
{
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
    p_arctan->CON |= LL_ARCTAN_CON_ENABLE;
}

/**
  * @brief  ARCTAN module stop function
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @retval None
  */
void ll_arctan_stop(ARCTAN_TypeDef *p_arctan)
{
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
    p_arctan->CON &= ~(LL_ARCTAN_CON_ENABLE);
}

/**
  * @brief  Arctan performs a single calculation.
  * @param  p_arctan: Select the initialized ARCTAN group pointer
  * @param  cos_data: input cos data vaule
  * @param  sin_data: input sin data vaule
  * @retval Returns the result of ARCTAN calculation, 16bit.
  */
u16 ll_arctan_calculate_once(ARCTAN_TypeDef *p_arctan, s16 cos_data, s16 sin_data)
{
    TX_ASSERT((p_arctan == ARCTAN0) || (p_arctan == ARCTAN1) || (p_arctan == ARCTAN2));
    
    p_arctan->IN  = LL_ARCTAN_IN_COS((u32)cos_data) | LL_ARCTAN_IN_SIN(sin_data);
    p_arctan->CON = LL_ARCTAN_CON_ENABLE;
    
    ll_arctan_wait_done_pending(p_arctan);
    
    return (u16)p_arctan->OUT;
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
