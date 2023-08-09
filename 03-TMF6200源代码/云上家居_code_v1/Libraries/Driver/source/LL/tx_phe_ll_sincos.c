/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_sincos.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SINCOS LL firmware functions.
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
#include "tx_phe_ll_sincos.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup sincos_interface_gr SINCOS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup SINCOS_LL_Driver SINCOS LL Driver
  * @ingroup  sincos_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup SINCOS_LL_Interrupt SINCOS LL Interrupt Handle function
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup SINCOS_LL_Inti_Cfg SINCOS LL Initialization And Configuration
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL Initialization And Configuration
  * @{
  */

/**
  * @brief  SINCOS module initialization function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  p_init: Module configuration structure pointer
  * @retval None
  */
void ll_sincos_init(SINCOS_TypeDef *p_sincos, TYPE_LL_SINCOS_INIT *p_init)
{
    /* The system resets the SINCOS module, Enable the clock of the module. */
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    
    /* clear sincos pending */
    p_sincos->CON |= LL_SINCOS_PEND;
}

/**
  * @brief  SINCOS module detele initialization function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_deinit(SINCOS_TypeDef *p_sincos)
{
    /* The system disable the SINCOS module, includes turning off the clock for the module. */
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
}

/**
  * @brief  SINCOS module configuration function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_sincos_config(SINCOS_TypeDef *p_sincos, TYPE_LL_SINCOS_CFG *p_cfg)
{
    u32 sincos_con = 0;
    
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    
    /* Check the parameters of SINCOS input DMA buffer address */
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    if(p_cfg->work_mode == LL_SINCOS_MODE_DMA) {
        if(p_sincos == SINCOS0) {
            if(p_cfg->radian_sel == LL_SINCOS_RADIAN_MEM) {
                /* SINCOS0's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
                TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                               SRAM_SINCOS0_MASK, SRAM_SINCOS0_ALIGN_MASK);
            }
            /* SINCOS0's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
            if(p_cfg->mode == LL_SINCOS_MODE_DATA_COSSIN) {
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_4BYTE,
                               SRAM_SINCOS0_MASK, SRAM_SINCOS0_ALIGN_MASK);
            } else {
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                               SRAM_SINCOS0_MASK, SRAM_SINCOS0_ALIGN_MASK);
            }
        } else if(p_sincos == SINCOS1) {
            if(p_cfg->radian_sel == LL_SINCOS_RADIAN_MEM) {
                /* SINCOS1's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
                TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                               SRAM_SINCOS1_MASK, SRAM_SINCOS1_ALIGN_MASK);
            }
            
            /* SINCOS1's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
            if(p_cfg->mode == LL_SINCOS_MODE_DATA_COSSIN) {
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_4BYTE,
                               SRAM_SINCOS1_MASK, SRAM_SINCOS1_ALIGN_MASK);
            } else {
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE,
                               SRAM_SINCOS1_MASK, SRAM_SINCOS1_ALIGN_MASK);
            }
        }
    }
    
    /* Calculation SINCOS control */
    sincos_con  = p_sincos->CON & LL_SINCOS_IE;
    sincos_con |= p_cfg->radian_sel ? LL_SINCOS_DMA_MODE : 0;
    sincos_con |= p_cfg->work_mode ? LL_SINCOS_DMA_EN : 0;
    sincos_con |= LL_SINCOS_MODE(p_cfg->mode);
    
    /* Configuring the SINCOS register */
    p_sincos->CON          = sincos_con;
    p_sincos->LEN          = LL_SINCOS_LEN(p_cfg->dma_points);
    p_sincos->STEP         = LL_SINCOS_STEP(p_cfg->step);
    p_sincos->DATA_IN      = LL_SINCOS_DATA_IN(p_cfg->data_in);
    p_sincos->DATA_IN_ADR  = LL_SINCOS_DATA_IN_ADR(p_cfg->dma_src_addr);
    p_sincos->DATA_OUT_ADR = LL_SINCOS_DATA_OUT_ADR(p_cfg->dma_dst_addr);
}

/**
  * @}
  */

/** @defgroup SINCOS_LL_Data_Transfers SINCOS LL Data transfers functions
  * @ingroup  SINCOS_LL_Driver
  * @brief    SINCOS LL Data transfers functions 
  * @{
  */

/**
  * @brief  SINCOS module start function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_start(SINCOS_TypeDef *p_sincos)
{
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    p_sincos->CON |= LL_SINCOS_ENABLE;
}

/**
  * @brief  SINCOS module stop function
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @retval None
  */
void ll_sincos_stop(SINCOS_TypeDef *p_sincos)
{
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    p_sincos->CON &= ~LL_SINCOS_ENABLE;
}

/**
  * @brief  Sincos sin mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @retval The calculation result of sin.
  */
s16 ll_sincos_calculate_sin_once(SINCOS_TypeDef *p_sincos, u16 data)
{
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    
    p_sincos->CON     = LL_SINCOS_PEND;
    p_sincos->DATA_IN = data;
    p_sincos->CON     = LL_SINCOS_MODE(LL_SINCOS_MODE_DATA_SIN) | LL_SINCOS_ENABLE;
    
    while(p_sincos->CON & LL_SINCOS_ENABLE);
    p_sincos->CON |= LL_SINCOS_PEND;
    
    return p_sincos->DATA_OUT;
}

/**
  * @brief  Sincos cos mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @retval The calculation result of sin.
  */
s16 ll_sincos_calculate_cos_once(SINCOS_TypeDef *p_sincos, u16 data)
{
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    
    p_sincos->CON     = LL_SINCOS_PEND;
    p_sincos->DATA_IN = data;
    p_sincos->CON     = LL_SINCOS_MODE(LL_SINCOS_MODE_DATA_COS) | LL_SINCOS_ENABLE;
    
    while(p_sincos->CON & LL_SINCOS_ENABLE);
    p_sincos->CON |= LL_SINCOS_PEND;
    
    return p_sincos->DATA_OUT;
}

/**
  * @brief  Sincos cos and sin mode single point calculation function.
  * @param  p_sincos: Select the initialized SINCOS group pointer
  * @param  data    : input data
  * @param  p_sin   : Calculate the saved value of the result sin.
  * @param  p_cos   : Calculate the saved value of the result cos.
  * @retval The calculation result of sin and cos.
  */
void ll_sincos_calculate_cossin_once(SINCOS_TypeDef *p_sincos, u16 data, s16 *p_sin, s16 *p_cos)
{
    TX_ASSERT((p_sincos == SINCOS0) || (p_sincos == SINCOS1));
    
    p_sincos->CON     = LL_SINCOS_PEND;
    p_sincos->DATA_IN = data;
    p_sincos->CON     = LL_SINCOS_MODE(LL_SINCOS_MODE_DATA_COSSIN) | LL_SINCOS_ENABLE;
    
    while(p_sincos->CON & LL_SINCOS_ENABLE);
    p_sincos->CON |= LL_SINCOS_PEND;
    
    *p_sin = p_sincos->DATA_OUT;
    *p_cos = p_sincos->DATA_OUT >> 16;
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
