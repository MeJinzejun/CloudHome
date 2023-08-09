/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_fir.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the FIR LL firmware functions.
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
#include "tx_phe_ll_fir.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup fir_interface_gr FIR Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup FIR_LL_Driver FIR LL Driver
  * @ingroup  fir_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup FIR_LL_Interrupt FIR LL Interrupt Handle function
  * @ingroup  FIR_LL_Driver
  * @brief   FIR LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup FIR_LL_Inti_Cfg FIR LL Initialization And Configuration
  * @ingroup  FIR_LL_Driver
  * @brief    FIR LL Initialization And Configuration
  * @{
  */

/**
  * @brief  FIR module initialization function
  * @param  p_fir: Select the initialized DFTRAN group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_fir_init(FIR_TypeDef *p_fir, TYPE_LL_FIR_INIT *p_init)
{
    /* The system resets the FIR module, Enable the clock of the module. */
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    /* FIR's param_start_addr can only be located in SRAM0. */
    TX_ASSERT_ADDR(p_init->param_start_addr, p_init->param_buf_len,
                   SRAM_FIR_CONFIG_MASK, SRAM_FIR_CONFIG_ALIGN_MASK);
    
    /* RESET FIR module */
    memset(p_fir, 0x0, sizeof(FIR_TypeDef));

    /* config fir param start address */
    p_fir->FIR_CFG_ADDR |= LL_FIR_CFG_ADDR(p_init->param_start_addr);
}

/**
  * @brief  FIR module detele initialization function
  * @param  p_fir: Select the initialized FIR group pointer
  * @retval None
  */
void ll_fir_deinit(FIR_TypeDef *p_fir)
{
    /* The system disable the FIR module, includes turning off the clock for the module. */
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
}

/**
  * @brief  FIR module interrupt configuration
  * @param  p_fir: Select the initialized FIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_fir_irq_config(FIR_TypeDef *p_fir, TYPE_LL_FIR_IRQ_CFG *p_cfg)
{
    /* Check the parameters of DFTRAN input DMA buffer address */
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    p_fir->FIR_INT_SRCL = (p_cfg->intr_src_sel & 0x01)                               ?
                          (p_fir->FIR_INT_SRCL | LL_FIR_INT_SRCL(BIT(p_cfg->chn)))   :
                          (p_fir->FIR_INT_SRCL & ~(LL_FIR_INT_SRCL(BIT(p_cfg->chn))));
    
    p_fir->FIR_INT_SRCH = ((p_cfg->intr_src_sel >> 1) & 0x01)                        ? 
                          (p_fir->FIR_INT_SRCH | LL_FIR_INT_SRCH(BIT(p_cfg->chn)))   : 
                          (p_fir->FIR_INT_SRCH & ~(LL_FIR_INT_SRCH(BIT(p_cfg->chn))));
    
    p_fir->FIR_INT_ENA |= LL_FIR_INT_ENA(BIT(p_cfg->chn));
}

/**
  * @brief  FIR module configuration
  * @param  p_fir: Select the initialized FIR group pointer
  * @param  p_cfg: Module configuration structure pointer
  * @retval None
  */
void ll_fir_config(FIR_TypeDef *p_fir, TYPE_LL_FIR_CFG *p_cfg)
{
    TYPE_LL_FIR_PARAM_CFG *p_param_cfg = (TYPE_LL_FIR_PARAM_CFG *)p_fir->FIR_CFG_ADDR + p_cfg->chn;
    
    /* Check the parameters of DFTRAN input DMA buffer address */
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    TX_ASSERT(p_fir->FIR_CFG_ADDR != 0);
    TX_ASSERT(p_cfg->chn < 8);
    
    if(p_fir == FIR0) {
        /* FIR0's coef_start_addr can only be located in SRAM1. */
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, p_param_cfg->coef_len * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR0_DATA_MASK, SRAM_FIR0_DATA_ALIGN_MASK);
        /* FIR0's dma_src_addr can only be located in SRAM1. */
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR0_DATA_MASK, SRAM_FIR0_DATA_ALIGN_MASK);
        /* FIR0's dma_dst_addr can only be located in SRAM1. */
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR0_DATA_MASK, SRAM_FIR0_DATA_ALIGN_MASK);
    } else if(p_fir == FIR1) {
        /* FIR1's coef_start_addr can only be located in SRAM2. */
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, p_param_cfg->coef_len * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR1_DATA_MASK, SRAM_FIR1_DATA_ALIGN_MASK);
        /* FIR1's dma_src_addr can only be located in SRAM2. */
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR1_DATA_MASK, SRAM_FIR1_DATA_ALIGN_MASK);
        /* FIR1's dma_dst_addr can only be located in SRAM2. */
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR1_DATA_MASK, SRAM_FIR1_DATA_ALIGN_MASK);
    } else if(p_fir == FIR2) {
        /* FIR2's coef_start_addr can only be located in SRAM3. */
        TX_ASSERT_ADDR(p_param_cfg->coef_start_addr, p_param_cfg->coef_len * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR2_DATA_MASK, SRAM_FIR2_DATA_ALIGN_MASK);
        /* FIR2's dma_src_addr can only be located in SRAM3. */
        TX_ASSERT_ADDR(p_param_cfg->dma_src_addr, p_param_cfg->dma_src_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR2_DATA_MASK, SRAM_FIR2_DATA_ALIGN_MASK);
        /* FIR2's dma_dst_addr can only be located in SRAM3. */
        TX_ASSERT_ADDR(p_param_cfg->dma_dst_addr, p_param_cfg->dma_dst_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FIR2_DATA_MASK, SRAM_FIR2_DATA_ALIGN_MASK);
    }
    
    /* Configuring the FIR register */
    p_fir->EVSYS_CH_ENA = p_cfg->evsys_chn_en                                         ? 
                          (p_fir->EVSYS_CH_ENA | LL_FIR_EVSYS_ENA(BIT(p_cfg->chn)))   : 
                          (p_fir->EVSYS_CH_ENA & ~(LL_FIR_EVSYS_ENA(BIT(p_cfg->chn))));
}

/**
  * @}
  */

/** @defgroup FIR_LL_Data_Transfers FIR LL Data transfers functions
  * @ingroup  FIR_LL_Driver
  * @brief    FIR LL Data transfers functions 
  * @{
  */

/**
  * @brief  FIR module start function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_fir_start(FIR_TypeDef *p_fir, u8 chn_bits)
{
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    p_fir->FIR_CH_ENA  |= LL_FIR_CH_ENA(chn_bits);
    p_fir->FIR_CPU_KST |= LL_FIR_CPU_KST(chn_bits);
}

/**
  * @brief  FIR module stop function
  * @param  p_fir   : Select the initialized FIR group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.  
  * @retval None
  */
void ll_fir_stop(FIR_TypeDef *p_fir, u8 chn_bits)
{
    TX_ASSERT((p_fir == FIR0) || (p_fir == FIR1) || (p_fir == FIR2));
    
    p_fir->FIR_CH_ENA &= ~(LL_FIR_CH_ENA(chn_bits));
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
