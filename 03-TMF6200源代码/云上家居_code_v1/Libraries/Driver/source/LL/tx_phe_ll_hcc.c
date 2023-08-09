/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_hcc.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_hcc.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup hcc_interface_gr HCC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup HCC_LL_Driver HCC LL Driver
  * @ingroup  hcc_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup HCC_LL_Interrupt HCC LL Interrupt Handle function
  * @ingroup  HCC_LL_Driver
  * @brief   HCC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup HCC_LL_Inti_Cfg HCC LL Initialization And Configuration
  * @ingroup  HCC_LL_Driver
  * @brief    HCC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  hcc_init
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_HCC_INIT
  * @retval None
  */
void ll_hcc_init(HCC_TypeDef *p_hcc, TYPE_LL_HCC_INIT *p_init)
{
    TX_ASSERT(p_hcc == HCC);

    NONSENSE(p_init);
    /* reset hardware */
}

/**
  * @brief  hcc_deinit
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_deinit(HCC_TypeDef *p_hcc)
{
    TX_ASSERT(p_hcc == HCC);
    
}

/**
  * @brief  ll_hcc_config
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @param  p_cfg : pointer to the init stuct TYPE_LL_HCC_CFG
  * @retval None
  */
void ll_hcc_config(HCC_TypeDef *p_hcc, TYPE_LL_HCC_CFG *p_cfg)
{ 
    TX_ASSERT(p_hcc == HCC);
    TX_ASSERT(p_cfg->dma_points);
    TX_ASSERT(!(p_cfg->dma_points & 0x01));
    
    TX_ASSERT_ADDR(p_cfg->dma_fft_start_addr, (p_cfg->dma_points >> 1) * SRAM_BUF_UINT_4BYTE, 
                    SRAM_HC_FFT_MASK, SRAM_HC_FFT_ALIGN_MASK);
    if(p_cfg->phase_enable) {
        TX_ASSERT_ADDR(p_cfg->dma_phase_start_addr, (p_cfg->dma_points >> 1) * SRAM_BUF_UINT_4BYTE, 
                       SRAM_HC_PHS_MASK, SRAM_HC_PHS_ALIGN_MASK);
    }
    if(p_cfg->range_enable) {
        TX_ASSERT_ADDR(p_cfg->dma_range_start_addr, (p_cfg->dma_points >> 1) * SRAM_BUF_UINT_4BYTE, 
                       SRAM_HC_RANG_MASK, SRAM_HC_RANG_ALIGN_MASK);
    }
    p_hcc->HCC_STADR_FFT = p_cfg->dma_fft_start_addr;
    p_hcc->HCC_STADR_PHS = p_cfg->dma_phase_start_addr;
    p_hcc->HCC_STADR_RAG = p_cfg->dma_range_start_addr;
    p_hcc->HCC_CONTROL   = (p_hcc->HCC_CONTROL & LL_HCC_INT_EN)      |
                           (p_cfg->range_enable ? LL_HCC_RAG_EN : 0) |
                           (p_cfg->phase_enable ? LL_HCC_PHS_EN : 0) |
                           LL_HCC_DMA_LEN(p_cfg->dma_points >> 1)    |
                           LL_HCC_PENDING;
}

/**
  * @}
  */

/** @defgroup HCC_LL_Data_Transfers HCC LL Data transfers functions
  * @ingroup  HCC_LL_Driver
  * @brief    HCC LL Data transfers functions 
  * @{
  */


/**
  * @brief  hcc_start
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_start(HCC_TypeDef *p_hcc)
{       
    TX_ASSERT(p_hcc == HCC);
    
    p_hcc->HCC_CONTROL  |= LL_HCC_START;
}

/**
  * @brief  Wait for the module to finish computing
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_wait_done_pending(HCC_TypeDef *p_hcc)
{
    TX_ASSERT(p_hcc == HCC);
    while(((p_hcc->HCC_CONTROL & LL_HCC_PENDING) == 0));
    p_hcc->HCC_CONTROL |= LL_HCC_PENDING;
}

/**
  * @brief  hcc stop
  * @param  p_hcc : pointer to the hardware HCC_TypeDef
  * @retval None
  */
void ll_hcc_stop(HCC_TypeDef *p_hcc)
{       
    TX_ASSERT(p_hcc == HCC);
}

/**
  * @brief  Get the result of the calculation
  * @param  p_hcc   : pointer to the hardware HCC_TypeDef
  * @param  p_result: The complex pointer to save the result.The output results
  *                   are 16bit signed numbers, including 15bit fixed-point decimals
  * @retval None
  * @note
  *     HCC_RESULT_REAL = Σrange*(fft_real*phs_real – fft_img*phs_img)
  *     HCC_RESULT_IMG= Σrange*(fft_img*phs_real + fft_real*phs_img)
  */
void ll_hcc_get_result(HCC_TypeDef *p_hcc, TYPE_LL_HCC_COMPLEX_DATA *p_result)
{
    p_result->img  = p_hcc->HCC_RESULT & 0xFFFF;
    p_result->real = p_hcc->HCC_RESULT >> 16;
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
