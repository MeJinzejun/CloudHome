/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_iir.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the IIR LL firmware functions.
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
#include "tx_phe_ll_iir.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup iir_interface_gr IIR Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup IIR_LL_Driver IIR LL Driver
  * @ingroup  iir_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup IIR_LL_Interrupt IIR LL Interrupt Handle function
  * @ingroup  IIR_LL_Driver
  * @brief   IIR LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup IIR_LL_Inti_Cfg IIR LL Initialization And Configuration
  * @ingroup  IIR_LL_Driver
  * @brief    IIR LL Initialization And Configuration
  * @{
  */

/**
  * @brief  IIR module initialization function
  * @param  p_iir : Select the initialized DFTRAN group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_iir_init(IIR_TypeDef *p_iir, TYPE_LL_IIR_INIT *p_init)
{
    /* The system resets the FIR module, Enable the clock of the module. */
    TX_ASSERT((p_iir == IIR0) || (p_iir == IIR1) || (p_iir == IIR2));
    if(p_iir == IIR0) {
        TX_ASSERT_ADDR(p_init->param_start_addr, p_init->param_buf_len,
                       SRAM_IIR_CONFIG0_MASK, SRAM_IIR_CONFIG0_ALIGN_MASK);
    } else if(p_iir == IIR1) {
        TX_ASSERT_ADDR(p_init->param_start_addr, p_init->param_buf_len,
                       SRAM_IIR_CONFIG1_MASK, SRAM_IIR_CONFIG1_ALIGN_MASK);
    } else if(p_iir == IIR2) {
        TX_ASSERT_ADDR(p_init->param_start_addr, p_init->param_buf_len,
                       SRAM_IIR_CONFIG2_MASK, SRAM_IIR_CONFIG2_ALIGN_MASK);
    }
    
    p_iir->IIR_CFG_ADDR = LL_IIR_CFG_ADDR(p_init->param_start_addr);
}

/**
  * @brief  IIR module detele initialization function
  * @param  p_iir: Select the initialized IIR group pointer
  * @retval None
  */
void ll_iir_deinit(IIR_TypeDef *p_iir)
{
    /* The system disable the FIR module, includes turning off the clock for the module. */
    TX_ASSERT((p_iir == IIR0) || (p_iir == IIR1) || (p_iir == IIR2));
}

/**
  * @brief  IIR module interrupt configuration
  * @param  p_iir: Select the initialized IIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_iir_irq_config(IIR_TypeDef *p_iir, TYPE_LL_IIR_IRQ_CFG *p_cfg)
{
    if(p_cfg->chn < 31) {
        p_iir->IIR_INT_SRCL0  = (p_cfg->intr_src_sel & 0x01)                                 ?
                                (p_iir->IIR_INT_SRCL0 | LL_IIR_INT_SRCL0(BIT(p_cfg->chn)))   :
                                (p_iir->IIR_INT_SRCL0 & ~(LL_IIR_INT_SRCL0(BIT(p_cfg->chn))));
        p_iir->IIR_INT_SRCH0  = ((p_cfg->intr_src_sel >> 1) & 0x01)                          ?
                                (p_iir->IIR_INT_SRCH0 | LL_IIR_INT_SRCH0(BIT(p_cfg->chn)))   :
                                (p_iir->IIR_INT_SRCH0 & ~(LL_IIR_INT_SRCH0(BIT(p_cfg->chn))));
        p_iir->IIR_INT_ENA0  |= LL_IIR_INT_ENA0(BIT(p_cfg->chn));
    } else {
        p_iir->IIR_INT_SRCL1  = (p_cfg->intr_src_sel & 0x01)                                      ? 
                                (p_iir->IIR_INT_SRCL1 | LL_IIR_INT_SRCL1(BIT(p_cfg->chn - 32)))   : 
                                (p_iir->IIR_INT_SRCL1 & ~(LL_IIR_INT_SRCL1(BIT(p_cfg->chn - 32))));
        p_iir->IIR_INT_SRCH1  = ((p_cfg->intr_src_sel >> 1) & 0x01)                               ? 
                                (p_iir->IIR_INT_SRCH1 | LL_IIR_INT_SRCH1(BIT(p_cfg->chn - 32)))   : 
                                (p_iir->IIR_INT_SRCH1 & ~(LL_IIR_INT_SRCH1(BIT(p_cfg->chn - 32))));
        p_iir->IIR_INT_ENA1  |= LL_IIR_INT_ENA1(BIT(p_cfg->chn - 32));
    }
}

/**
  * @brief  IIR module configuration
  * @param  p_iir: Select the initialized IIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_iir_config(IIR_TypeDef *p_iir, TYPE_LL_IIR_CFG *p_cfg)
{
    TYPE_LL_IIR_PARAM_CFG *p_param_cfg = (TYPE_LL_IIR_PARAM_CFG *)p_iir->IIR_CFG_ADDR + p_cfg->chn;
    
    TX_ASSERT((p_iir == IIR0) || (p_iir == IIR1) || (p_iir == IIR2));
    TX_ASSERT(p_cfg->chn < 64);
    
    if(p_iir == IIR0) {
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, sizeof(TYPE_LL_IIR_COEFF_CFG),
                       SRAM_IIR_COEF0_MASK, SRAM_IIR_COEF0_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA0_MASK, SRAM_IIR_DATA0_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA0_MASK, SRAM_IIR_DATA0_ALIGN_MASK);
    } else if(p_iir == IIR1) {
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, sizeof(TYPE_LL_IIR_COEFF_CFG),
                       SRAM_IIR_COEF1_MASK, SRAM_IIR_COEF1_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA1_MASK, SRAM_IIR_DATA1_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA1_MASK, SRAM_IIR_DATA1_ALIGN_MASK);
    } else if(p_iir == IIR2) {
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, sizeof(TYPE_LL_IIR_COEFF_CFG),
                       SRAM_IIR_COEF2_MASK, SRAM_IIR_COEF2_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA2_MASK, SRAM_IIR_DATA2_ALIGN_MASK);
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_IIR_DATA2_MASK, SRAM_IIR_DATA2_ALIGN_MASK);
    }
    
    if(p_cfg->chn < 31) {
        p_iir->EVSYS_CH_ENA0 = p_cfg->evsys_chn_en                                          ?
                              (p_iir->EVSYS_CH_ENA0 | LL_IIR_EVSYS_ENA0(BIT(p_cfg->chn)))   :
                              (p_iir->EVSYS_CH_ENA0 & ~(LL_IIR_EVSYS_ENA0(BIT(p_cfg->chn))));
    } else {
        p_iir->EVSYS_CH_ENA1 = p_cfg->evsys_chn_en                                               ?
                              (p_iir->EVSYS_CH_ENA1 | LL_IIR_EVSYS_ENA1(BIT(p_cfg->chn - 32)))   :
                              (p_iir->EVSYS_CH_ENA1 & ~(LL_IIR_EVSYS_ENA1(BIT(p_cfg->chn - 32))));
    }
}

/**
  * @brief  IIR module multi-channel start function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_iir_start(IIR_TypeDef *p_iir, u64 chn_bits)
{
    TX_ASSERT((p_iir == IIR0) || (p_iir == IIR1) || (p_iir == IIR2));
    
    p_iir->IIR_CH_ENA0  |= LL_IIR_CH_ENA0(chn_bits);
    p_iir->IIR_CH_ENA1  |= LL_IIR_CH_ENA1(chn_bits >> 32);
    p_iir->IIR_CPU_KST0 |= LL_IIR_CPU_KST0(chn_bits);
    p_iir->IIR_CPU_KST1 |= LL_IIR_CPU_KST1(chn_bits >> 32);
}

/**
  * @brief  IIR module multi-channel stop function.
  * @param  p_iir   : Select the initialized IIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_iir_stop(IIR_TypeDef *p_iir, u64 chn_bits)
{
    TX_ASSERT((p_iir == IIR0) || (p_iir == IIR1) || (p_iir == IIR2));

    p_iir->IIR_CH_ENA0 &= ~(LL_IIR_CH_ENA0(chn_bits));
    p_iir->IIR_CH_ENA1 &= ~(LL_IIR_CH_ENA1(chn_bits >> 32));
}

/**
  * @}
  */

/** @defgroup IIR_LL_Data_Transfers IIR LL Data transfers functions
  * @ingroup  IIR_LL_Driver
  * @brief    IIR LL Data transfers functions 
  * @{
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

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
