/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_evsys.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the EVSYS LL firmware functions.
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
#include "tx_phe_ll_evsys.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup evsys_interface_gr EVSYS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup EVSYS_LL_Driver EVSYS LL Driver
  * @ingroup  evsys_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup EVSYS_LL_Interrupt EVSYS LL Interrupt Handle function
  * @ingroup  EVSYS_LL_Driver
  * @brief   EVSYS LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup EVSYS_LL_Inti_Cfg EVSYS LL Initialization And Configuration
  * @ingroup  EVSYS_LL_Driver
  * @brief    EVSYS LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Low layer EVSYS module initialization
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @param  p_init : Module configuration structure pointer(TYPE_LL_EVSYS_INIT)
  * @retval None
  */
void ll_evsys_init(EVSYS_TypeDef *p_evsys, TYPE_LL_EVSYS_INIT *p_init)
{
    /* The system resets the EVSYS module, Enable the clock of the module. */
    TX_ASSERT(p_evsys == EVSYS);
    
    memset(p_evsys, 0x0, sizeof(EVSYS_TypeDef));
    p_evsys->CH_PND_CLR = LL_EVSYS_CHN_PND_CLR(0xFFFF);
}

/**
  * @brief  Low layer EVSYS module detele initialization
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @retval None
  */
void ll_evsys_deinit(EVSYS_TypeDef *p_evsys)
{
    /* The system disable the EVSYS module, includes turning off the clock for the module. */
    TX_ASSERT(p_evsys == EVSYS);
}

/**
  * @brief  Low layer EVSYS module configuration
  * @param  p_evsys: The register structure pointer of the EVSYS.
  * @param  p_cfg  : Module configuration structure pointer(TYPE_LL_EVSYS_CFG)
  * @retval None
  */
void ll_evsys_config(EVSYS_TypeDef *p_evsys, TYPE_LL_EVSYS_CFG *p_cfg)
{
    u32 evsys_con1 = 0;
    
    TX_ASSERT(p_evsys == EVSYS);
    
    evsys_con1 |= LL_EVSYS_SRC_CHN_NUM1(BIT(p_cfg->src_chn - 32));
    evsys_con1 |= LL_EVSYS_SRC_CHN_CNT(p_cfg->src_kick_cnt);
    
    p_evsys->CH_MODE                 = 0;
    p_evsys->CH_ENA                 |= LL_EVSYS_CHN_ENABLE(BIT(p_cfg->chn));
    p_evsys->CH_SRC_CON0[p_cfg->chn] = LL_EVSYS_SRC_CHN_NUM0(BIT(p_cfg->src_chn));
    p_evsys->CH_SRC_CON1[p_cfg->chn] = evsys_con1;
    p_evsys->CH_DST_CON0[p_cfg->chn] = LL_EVSYS_DST_CHN_ENABLE(BIT(p_cfg->dst_chn));
}

/**
  * @}
  */

/** @defgroup EVSYS_LL_Data_Transfers EVSYS LL Data transfers functions
  * @ingroup  EVSYS_LL_Driver
  * @brief    EVSYS LL Data transfers functions 
  * @{
  */

/**
  * @brief  EVSYS module start function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
void ll_evsys_start(EVSYS_TypeDef *p_evsys, u16 chn_bits)
{
    TX_ASSERT(p_evsys == EVSYS);
    
    p_evsys->CH_ENA     |= LL_EVSYS_CHN_ENABLE(chn_bits);
    p_evsys->CH_CPU_KST |= LL_EVSYS_CHN_KICK_START_L(chn_bits) |
                           LL_EVSYS_CHN_KICK_START_H(chn_bits);
}

/**
  * @brief  EVSYS module stop function
  * @param  p_evsys : The register structure pointer of the EVSYS.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.
  * @retval None
  */
void ll_evsys_stop(EVSYS_TypeDef *p_evsys, u16 chn_bits)
{
    TX_ASSERT(p_evsys == EVSYS);
    
    p_evsys->CH_ENA &= ~(LL_EVSYS_CHN_ENABLE(chn_bits));
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
