/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_fft.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the FFT LL firmware functions.
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
#include "tx_phe_ll_fft.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup fft_interface_gr FFT Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup FFT_LL_Driver FFT LL Driver
  * @ingroup  fft_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup FFT_LL_Interrupt FFT LL Interrupt Handle function
  * @ingroup  FFT_LL_Driver
  * @brief   FFT LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup FFT_LL_Inti_Cfg FFT LL Initialization And Configuration
  * @ingroup  FFT_LL_Driver
  * @brief    FFT LL Initialization And Configuration
  * @{
  */ 

/**
  * @brief  Low layer FFT module initialization
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_FFT_INIT)
  * @retval None
  */
void ll_fft_init(FFT_TypeDef *p_fft, TYPE_LL_FFT_INIT *p_init)
{
    /* The system resets the FFT module, Enable the clock of the module. */
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    /* Reset the registers of the FFT module. */
    memset(p_fft, 0x0, sizeof(FFT_TypeDef));
    p_fft->MAXIMUM = 11584;
}

/**
  * @brief  Low layer FFT module detele initialization
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_deinit(FFT_TypeDef *p_fft)
{
    /* The system disable the FFT module, includes turning off the clock for the module. */
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
}

/**
  * @brief  Low layer FFT module interrupt configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_IRQ_CFG)
  * @retval None
  */
void ll_fft_irq_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_IRQ_CFG *p_cfg)
{
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    p_fft->DMA_CON = p_cfg->dma_intr_en                             ?
                     (p_fft->DMA_CON | LL_FFT_DMA_CON_FFT_DMA_IE)   :
                     (p_fft->DMA_CON & (~LL_FFT_DMA_CON_FFT_DMA_IE));
    p_fft->CTRL    = p_cfg->intr_en                       ?
                     (p_fft->CTRL | LL_FFT_CTRL_IRQ_EN)   :
                     (p_fft->CTRL & (~LL_FFT_CTRL_IRQ_EN));
}

/**
  * @brief  Low layer FFT preprocess configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_PREPROCESS_CFG)
  * @retval None
  */
void ll_fft_preprocess_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_PREPROCESS_CFG *p_cfg)
{
    u32 fft_dma_con = 0;
    
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    /* Check the parameters of FFT input DMA buffer address */
    if(p_fft == FFT0) {
        /* FFT0's real_dma_src_addr can only be located in SRAM1. */
        TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FFT0_REAL_MASK, SRAM_FFT0_REAL_ALIGN_MASK);
        /* FFT0's img_dma_src_addr can only be located in SRAM2. */
        if(p_cfg->img_val_sel != LL_FFT_IMAG_INPUT_ZERO) {
            TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT0_IMAG_MASK, SRAM_FFT0_IMAG_ALIGN_MASK);
        }
        /* FFT0's window_dma_src_addr can only be located in SRAM0. */
        if(p_cfg->window_src == LL_FFT_WINDOW_SRC_MEM) {
            TX_ASSERT_ADDR(p_cfg->window_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT0_WINDOW_MASK, SRAM_FFT0_WINDOW_ALIGN_MASK);
        }
        /* FFT0's dma_dst_addr can only be located in SRAM4 and SRAM5. */
        TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_4BYTE,
                       SRAM_FFT0_REALIMAG_MASK, SRAM_FFT0_REALIMAG_ALIGN_MASK);
    } else if(p_fft == FFT1) {
        /* FFT1's real_dma_src_addr can only be located in SRAM2. */
        TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FFT1_REAL_MASK, SRAM_FFT1_REAL_ALIGN_MASK);
        /* FFT1's img_dma_src_addr can only be located in SRAM3. */
        if(p_cfg->img_val_sel != LL_FFT_IMAG_INPUT_ZERO) {
            TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT1_IMAG_MASK, SRAM_FFT1_IMAG_ALIGN_MASK);
        }
        /* FFT1's window_dma_src_addr can only be located in SRAM0. */
        if(p_cfg->window_src == LL_FFT_WINDOW_SRC_MEM) {
            TX_ASSERT_ADDR(p_cfg->window_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT1_WINDOW_MASK, SRAM_FFT1_WINDOW_ALIGN_MASK);
        }
        /* FFT1's dma_dst_addr can only be located in SRAM4 and SRAM5. */
        TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_4BYTE,
                       SRAM_FFT1_REALIMAG_MASK, SRAM_FFT1_REALIMAG_ALIGN_MASK);
    } else if(p_fft == FFT2) {
        /* FFT2's real_dma_src_addr can only be located in SRAM3. */
        TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                       SRAM_FFT2_REAL_MASK, SRAM_FFT2_REAL_ALIGN_MASK);
        /* FFT2's img_dma_src_addr can only be located in SRAM1. */
        if(p_cfg->img_val_sel != LL_FFT_IMAG_INPUT_ZERO) {
            TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT2_IMAG_MASK, SRAM_FFT2_IMAG_ALIGN_MASK);
        }
        /* FFT2's window_dma_src_addr can only be located in SRAM0. */
        if(p_cfg->window_src == LL_FFT_WINDOW_SRC_MEM) {
            TX_ASSERT_ADDR(p_cfg->window_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                           SRAM_FFT2_WINDOW_MASK, SRAM_FFT2_WINDOW_ALIGN_MASK);
        }
        /* FFT2's dma_dst_addr can only be located in SRAM4 and SRAM5. */
        TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_4BYTE,
                       SRAM_FFT2_REALIMAG_MASK, SRAM_FFT2_REALIMAG_ALIGN_MASK);
    }
    
    /* Calculation FFT DMA control */
    fft_dma_con  = p_fft->DMA_CON & LL_FFT_DMA_CON_FFT_DMA_IE;
    fft_dma_con |= LL_FFT_DMA_CON_DOWN_SAMPLE(p_cfg->downsample_sel);
    fft_dma_con |= p_cfg->real_img_swap ? LL_FFT_DMA_CON_REAL_IMAG_EXCHANGE : 0;
    fft_dma_con |= LL_FFT_DMA_CON_WINDOW_SRC(p_cfg->window_src);
    fft_dma_con |= LL_FFT_DMA_CON_IMAGE_SEL(p_cfg->img_val_sel);
    fft_dma_con |= LL_FFT_DMA_CON_FFT_MODE(p_cfg->point_sel);
    
    /* Configuring the FFT DMA register */
    p_fft->DMA_CON            = fft_dma_con;
    p_fft->REAL_STADR         = LL_FFT_REAL_STADR(p_cfg->real_dma_src_addr);
    p_fft->IMAG_STADR         = LL_FFT_IMAG_STADR(p_cfg->img_dma_src_addr);
    p_fft->WINDOW_STADR       = LL_FFT_WINDOW_STADR(p_cfg->window_dma_src_addr);
    p_fft->REALIMAG_OUT_STADR = LL_FFT_REALIMAG_OUT_STADR(p_cfg->dma_dst_addr);
    p_fft->INDEX              = LL_FFT_INDEX(p_cfg->index);
    p_fft->LEN                = LL_FFT_LEN(p_cfg->dma_points);
}

/**
  * @brief  Low layer FFT module configuration
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_FFT_CFG)
  * @retval None
  */
void ll_fft_config(FFT_TypeDef *p_fft, TYPE_LL_FFT_CFG *p_cfg)
{
    u32 fft_ctrl;
    
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    /* Calculation FFT control */
    fft_ctrl  = p_fft->CTRL & LL_FFT_CTRL_IRQ_EN;
    fft_ctrl |= p_cfg->shift_sel ? LL_FFT_CTRL_SHIFT_CTRL : 0;
    fft_ctrl |= p_cfg->mode ? LL_FFT_CTRL_IFFT_MODE : 0;
    fft_ctrl |= LL_FFT_CTRL_FFT_POINT_SEL(p_cfg->point_sel);
    
    /* Configuring the FFT register */
    p_fft->CTRL = fft_ctrl;
}

/**
  * @}
  */

/** @defgroup FFT_LL_Data_Transfers FFT LL Data transfers functions
  * @ingroup  FFT_LL_Driver
  * @brief    FFT LL Data transfers functions 
  * @{
  */

/**
  * @brief  Low layer FFT preprocess start function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_preprocess_start(FFT_TypeDef *p_fft) 
{
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    p_fft->DMA_CON |= LL_FFT_DMA_CON_ENABLE;
}

/**
  * @brief  Low layer FFT module start function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_start(FFT_TypeDef *p_fft) 
{
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    p_fft->CTRL |= LL_FFT_CTRL_FFT_EN;
    p_fft->KS    = LL_FFT_KS;
}

/**
  * @brief  Low layer FFT preprocess stop function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
void ll_fft_preprocess_stop(FFT_TypeDef *p_fft) 
{
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    p_fft->DMA_CON &= ~LL_FFT_DMA_CON_ENABLE;
}

/**
  * @brief  Low layer FFT module stop function
  * @param  p_fft: The structure pointer of the FFT group (FFT0, FFT1, FFT2) is selected.
  * @retval None
  */
 void ll_fft_stop(FFT_TypeDef *p_fft) 
{
    TX_ASSERT((p_fft == FFT0) || (p_fft == FFT1) || (p_fft == FFT2));
    
    p_fft->CTRL &= ~LL_FFT_CTRL_FFT_EN;
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
