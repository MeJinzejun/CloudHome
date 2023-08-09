/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_crc.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_crc.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup crc_interface_gr CRC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup CRC_LL_Driver CRC LL Driver
  * @ingroup  crc_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup CRC_LL_Interrupt CRC LL Interrupt Handle function
  * @ingroup  CRC_LL_Driver
  * @brief   CRC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup CRC_LL_Inti_Cfg CRC LL Initialization And Configuration
  * @ingroup  CRC_LL_Driver
  * @brief    CRC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  crc_init
  * @param  p_crc : pointer to the hardware CRC_TypeDef
  * @param  p_cfg : pointer to the init stuct TYPE_LL_CRC_INIT
  * @retval None
  */
void ll_crc_init(CRC_TypeDef *p_crc, TYPE_LL_CRC_INIT *p_init)
{
    TX_ASSERT(p_crc == CRC);

    NONSENSE(p_init);
    /* reset hardware */
}

/**
  * @brief  crc_deinit
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_deinit(CRC_TypeDef *p_crc)
{
    TX_ASSERT(p_crc == CRC);
    
}

/**
  * @brief  ll_crc_config & start crc calc
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @param  p_cfg: pointer to the init stuct TYPE_LL_CRC_CFG
  * @retval None
  */
void ll_crc_config(CRC_TypeDef *p_crc, TYPE_LL_CRC_CFG *p_cfg)
{ 
    TX_ASSERT(p_crc == CRC);
    
    u32 crc_p_cfg_reg  = (p_crc->CRC_CFG & LL_CRC_INT_EN)            |
                         LL_CRC_POLY_BITS(p_cfg->poly_bits)          | 
                         LL_CRC_DMAWAIT_CLOCK(p_cfg->crc32_wait_clks);

    if(LL_CRC_POLY_SHIFT_RIGHT == p_cfg->poly_shift_dir) {
        crc_p_cfg_reg |= LL_CRC_BIT_ORDER_RIGHT;
    } else {
        crc_p_cfg_reg |= LL_CRC_BIT_ORDER_LEFT;
    }

    p_crc->CRC_INIT = p_cfg->init_val;
    p_crc->CRC_INV  = p_cfg->out_invert;
    p_crc->CRC_POLY = p_cfg->poly;
    p_crc->CRC_CFG  = crc_p_cfg_reg;
}

/**
  * @}
  */

/** @defgroup CRC_LL_Data_Transfers CRC LL Data transfers functions
  * @ingroup  CRC_LL_Driver
  * @brief    CRC LL Data transfers functions 
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
void ll_crc_start(CRC_TypeDef *p_crc, u32 addr, u32 len)
{
    TX_ASSERT(p_crc == CRC);
    TX_ASSERT(len > 0);
    TX_ASSERT_ADDR(addr, len, SRAM_CRC_DMA_MASK, SRAM_CRC_DMA_ALIGN_MASK);
    
    p_crc->DMA_ADDR = addr;
    p_crc->DMA_LEN  = len;
}

/**
  * @brief  crc wait
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_wait_done_pending(CRC_TypeDef *p_crc)
{
    TX_ASSERT(p_crc == CRC);

    while(!(p_crc->CRC_STA & LL_CRC_DMA_PENDING));
    p_crc->CRC_KST = LL_CRC_DMA_PENDING_CLR;
}

/**
  * @brief  crc stop
  * @param  p_crc: pointer to the hardware CRC_TypeDef
  * @retval None
  */
void ll_crc_stop(CRC_TypeDef *p_crc)
{
    TX_ASSERT(p_crc == CRC);
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
