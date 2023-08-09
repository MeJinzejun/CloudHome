/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_matrix.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the MATRIX LL firmware functions.
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
#include "tx_phe_ll_matrix.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup matrix_interface_gr MATRIX Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup MATRIX_LL_Driver MATRIX LL Driver
  * @ingroup  matrix_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup MATRIX_LL_Interrupt MATRIX LL Interrupt Handle function
  * @ingroup  MATRIX_LL_Driver
  * @brief   MATRIX LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup MATRIX_LL_Inti_Cfg MATRIX LL Initialization And Configuration
  * @ingroup  MATRIX_LL_Driver
  * @brief    MATRIX LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Matrix Multiplication Module Initialization
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  p_init  : pointer to the init struct TYPE_LL_MATRIX_INIT
  * @retval None
  */
void ll_matrix_init(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_INIT *p_init)
{
    NONSENSE(p_init);
    
    TX_ASSERT(p_matrix == MATRIX);
    
    /* disable all channels */
    p_matrix->MATRIX_EN   = 0x0F00;
    p_matrix->MATRIX_PEND = 0x000F;
    p_matrix->MATRIX_IE   = 0;
}

/**
  * @brief  Matrix Multiplication Module Initialization
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @retval None
  */
void ll_matrix_deinit(MATRIX_TypeDef *p_matrix)
{    
    TX_ASSERT(p_matrix == MATRIX);
    
    /* disable all channels                 */
    p_matrix->MATRIX_IE   = 0;
    p_matrix->MATRIX_EN   = 0x0F00;
    p_matrix->MATRIX_PEND = 0x000F;
}

/**
  * @brief  3x3 matrix configuration
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  p_cfg   : TYPE_MATRIX_3x3_CFG, Configure 3x3 matrix values and where they 
  *                    re stored.
  * @retval None
  */
void ll_matrix_3x3_coef_config(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_3X3_CFG *p_cfg)
{
    u32 chn = p_cfg->select;
    
    TX_ASSERT(p_matrix == MATRIX);

    p_matrix->COEF[chn][0] = p_cfg->coef_00;
    p_matrix->COEF[chn][1] = p_cfg->coef_01;
    p_matrix->COEF[chn][2] = p_cfg->coef_02;
    p_matrix->COEF[chn][3] = p_cfg->coef_10;
    p_matrix->COEF[chn][4] = p_cfg->coef_11;
    p_matrix->COEF[chn][5] = p_cfg->coef_12;
    p_matrix->COEF[chn][6] = p_cfg->coef_20;
    p_matrix->COEF[chn][7] = p_cfg->coef_21;
    p_matrix->COEF[chn][8] = p_cfg->coef_22;
}

/**
  * @brief  Matrix Multiplication
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  p_cfg   : TYPE_MATRIX_CALC_CFG, Configure matrix computing module.
  * @retval None
  */
void ll_matrix_config(MATRIX_TypeDef *p_matrix, TYPE_LL_MATRIX_CALC_CFG *p_cfg)
{
    TX_ASSERT(p_matrix == MATRIX);
    TX_ASSERT(p_cfg->chn < LL_MAXTRIX_CHN_MAX);
    TX_ASSERT(p_cfg->out_frac_width >= 8);
    TX_ASSERT(p_cfg->in_frac_width >= 8);
    TX_ASSERT(p_cfg->index < p_cfg->dma_points);
    
    TX_ASSERT_ADDR(p_cfg->dma_src_addr_x, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_X_MASK, SRAM_MATRIX_X_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_dst_addr_x, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_X_MASK, SRAM_MATRIX_X_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_src_addr_y, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_Y_MASK, SRAM_MATRIX_Y_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_dst_addr_y, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_Y_MASK, SRAM_MATRIX_Y_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_src_addr_z, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_Z_MASK, SRAM_MATRIX_Z_ALIGN_MASK);
    TX_ASSERT_ADDR(p_cfg->dma_dst_addr_z, (p_cfg->dma_points * SRAM_BUF_UINT_2BYTE),
                   SRAM_MATRIX_Z_MASK, SRAM_MATRIX_Z_ALIGN_MASK);

    MATRIX_CH_TypeDef *p_matrix_ch = &p_matrix->CH[p_cfg->chn];
    
    if(p_cfg->matrix_3x3_select == 1) {
        p_matrix->MATRIX_COEF_SEL |= BIT(p_cfg->chn);
    } else {
        p_matrix->MATRIX_COEF_SEL &= ~(BIT(p_cfg->chn));
    }
    
    p_matrix_ch->DATAIN_STADR0   = p_cfg->dma_src_addr_x;
    p_matrix_ch->DATAIN_STADR1   = p_cfg->dma_src_addr_y;
    p_matrix_ch->DATAIN_STADR2   = p_cfg->dma_src_addr_z;
    p_matrix_ch->DATAOUT_STADR0  = p_cfg->dma_dst_addr_x;
    p_matrix_ch->DATAOUT_STADR1  = p_cfg->dma_dst_addr_y;
    p_matrix_ch->DATAOUT_STADR2  = p_cfg->dma_dst_addr_z;
    p_matrix_ch->INDEX           = (p_cfg->index << 16) | p_cfg->index;
    p_matrix_ch->LEN             = (p_cfg->dma_points << 16) | p_cfg->dma_points;
    p_matrix_ch->OUT_FRAC_WIDTH  = p_cfg->out_frac_width - 8;
    p_matrix_ch->COEF_FRAC_WIDTH = p_cfg->in_frac_width - 8;
}

/**
  * @}
  */

/** @defgroup MATRIX_LL_Data_Transfers MATRIX LL Data transfers functions
  * @ingroup  MATRIX_LL_Driver
  * @brief    MATRIX LL Data transfers functions 
  * @{
  */

/**
  * @brief  Start matrix module operation
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  chn     : Select the channel to enable
  * @retval None
  * @note   Each call to a function will only cause the module to perform a 
  *         matrix multiplication operation. That is, there will be only one
  *         output at a time.
  * @note dmabuf is ringbuf, index will auto inc and reset to start at the end
  */
void ll_matrix_start(MATRIX_TypeDef *p_matrix, u8 chn)
{
    TX_ASSERT(p_matrix == MATRIX);
    
    p_matrix->MATRIX_EN = LL_MAXTRIX_ENABLE_CH(chn);
}

/**
  * @brief  Wait for the completion of the matrix multiplication of a 
  *         particular channel
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  chn     : the channel waiting to be calculated.
  * @retval None
  */
void ll_matrix_wait_done_pending(MATRIX_TypeDef *p_matrix, u8 chn)
{
    TX_ASSERT(p_matrix == MATRIX);
    
    /* After the calculation is completed, pending is set to 1 and enable 
     * is cleared. So the following 2 statements are equivalent
     */
//    while(0 == (p_matrix->MATRIX_PEND & BIT(chn))) {//Cannot be used after enable interrupt
//        p_matrix->MATRIX_PEND |= BIT(chn);
//    }
    while(p_matrix->MATRIX_EN & BIT(chn));
    p_matrix->MATRIX_PEND |= BIT(chn);
}

/**
  * @brief  stop matrix module operation
  * @param  p_matrix: pointer to the hardware MATRIX_TypeDef
  * @param  chn     : Select the channel to enable
  * @retval None
  * @note   Each call to a function will only cause the module to perform a 
  *         matrix multiplication operation. That is, there will be only one
  *         output at a time.
  */
void ll_matrix_stop(MATRIX_TypeDef *p_matrix, u8 chn)
{
    TX_ASSERT(p_matrix == MATRIX);
    
    p_matrix->MATRIX_EN = LL_MAXTRIX_DISABLE_CH(chn);
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
