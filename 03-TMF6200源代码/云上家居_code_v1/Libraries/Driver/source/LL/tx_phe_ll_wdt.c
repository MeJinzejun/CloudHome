/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_wdt.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the WDT LL firmware functions.
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
#include "tx_phe_ll_wdt.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup wdt_interface_gr WDT Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup WDT_LL_Driver WDT LL Driver
  * @ingroup  wdt_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup WDT_LL_Interrupt WDT LL Interrupt Handle function
  * @ingroup  WDT_LL_Driver
  * @brief   WDT LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup WDT_LL_Inti_Cfg WDT LL Initialization And Configuration
  * @ingroup  WDT_LL_Driver
  * @brief    WDT LL Initialization And Configuration
  * @{
  */

/**
  * @brief  wdt_init
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_WDT_INIT
  * @retval None
  */
void ll_wdt_init(WDT_TypeDef *p_wdt, TYPE_LL_WDT_INIT *p_init)
{
    TX_ASSERT(p_wdt == WDT);

    ll_cc_rst_softreset_release(LL_CC_RST_SOFTRST_M_WDT);

    /* watchdog reset enable */
    SYSCTRL_REG_OPT( 
        SYSCTRL->SYS_CON0 = (SYSCTRL->SYS_CON0 & ~(BIT(18)|BIT(19)|BIT(20))) | (BIT(20));
    ); 

    p_wdt->WDT_CR = 0;
    
    ll_wdt_clear_done_pending(p_wdt);
}

/**
  * @brief  wdt_deinit
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_deinit(WDT_TypeDef *p_wdt)
{
    TX_ASSERT(p_wdt == WDT);
    
    p_wdt->WDT_CR = 0;
    
    ll_wdt_clear_done_pending(p_wdt);

    
    ll_cc_rst_softreset(LL_CC_RST_SOFTRST_M_WDT);
}

/**
  * @brief  watchdog config
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @param  p_cfg : pointer to the init stuct TYPE_LL_WDT_CFG
  * @retval None
  */
void ll_wdt_config(WDT_TypeDef *p_wdt, TYPE_LL_WDT_CFG *p_cfg)
{ 
    TX_ASSERT(p_wdt == WDT);
    
    p_wdt->WDT_TORR = LL_WDT_TOP_INIT(p_cfg->timeout) | LL_WDT_TOP(p_cfg->timeout);

    p_wdt->WDT_CR   = LL_WDT_RMODE(p_cfg->mode) | LL_WDT_RPL(3);
}

/**
  * @}
  */

/** @defgroup WDT_LL_Data_Transfers WDT LL Data transfers functions
  * @ingroup  WDT_LL_Driver
  * @brief    WDT LL Data transfers functions 
  * @{
  */

/**
  * @brief  wdt_start
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_start(WDT_TypeDef *p_wdt)
{   
    TX_ASSERT(p_wdt == WDT);

    p_wdt->WDT_CR |= LL_WDT_EN;

    ll_wdt_feed(p_wdt);
}

/**
  * @brief  wdt stop
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_stop(WDT_TypeDef *p_wdt)
{     
    TX_ASSERT(p_wdt == WDT);

    /* wdt disable */
    
    p_wdt->WDT_CR &= ~LL_WDT_EN;
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
