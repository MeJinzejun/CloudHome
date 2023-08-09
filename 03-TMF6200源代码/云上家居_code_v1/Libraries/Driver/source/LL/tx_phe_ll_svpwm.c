/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_svpwm.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SVPWM LL firmware functions.
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
#include "tx_phe_ll_svpwm.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup svpwm_interface_gr SVPWM Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup SVPWM_LL_Driver SVPWM LL Driver
  * @ingroup  svpwm_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup SVPWM_LL_Interrupt SVPWM LL Interrupt Handle function
  * @ingroup  SVPWM_LL_Driver
  * @brief   SVPWM LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup SVPWM_LL_Inti_Cfg SVPWM LL Initialization And Configuration
  * @ingroup  SVPWM_LL_Driver
  * @brief    SVPWM LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Low layer SVPWM module initialization
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_SVPWM_INIT)
  * @retval None
  */
void ll_svpwm_init(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_INIT *p_init)
{
    /* The system resets the SVPWM module, Enable the clock of the module. */
    TX_ASSERT(p_svpwm == SVPWM);
    
    memset(p_svpwm, 0x0, sizeof(SVPWM_TypeDef));
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_LDP       |
                     LL_SVPWM_CON0_CLR_DERP      |
                     LL_SVPWM_CON0_CLR_MATCH_PNG |
                     LL_SVPWM_CON0_CLR_HWP       |
                     LL_SVPWM_CON0_CLR_SVPWMEN;
}

/**
  * @brief  Low layer SVPWM module detele initialization
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @retval None
  */
void ll_svpwm_deinit(SVPWM_TypeDef *p_svpwm)
{
    TX_ASSERT(p_svpwm == SVPWM);
    
    SYSCTRL_REG_OPT(
        SYSCTRL->PWM_KEY = GPIO_KEY_C | GPIO_KEY_B | GPIO_KEY_A;
        SYSCTRL->IO_MAP &= ~BIT(22)
    );
}

/**
  * @brief  Low layer SVPWM module interrupt configuration
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_SVPWM_IRQ_CFG)
  * @retval None
  */
void ll_svpwm_irq_config(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_IRQ_CFG *p_cfg)
{
    TX_ASSERT(p_svpwm == SVPWM);
    u32 svpwm_con0 = p_svpwm->CON0;
    
    svpwm_con0 = p_cfg->dataerr_intr_en                    ?
                 (svpwm_con0 | LL_SVPWM_CON0_DER_INT_EN)   :
                 (svpwm_con0 & ~(LL_SVPWM_CON0_DER_INT_EN));
    svpwm_con0 = p_cfg->match_intr_en                        ?
                 (svpwm_con0 | LL_SVPWM_CON0_MATCH_INT_EN)   :
                 (svpwm_con0 & ~(LL_SVPWM_CON0_MATCH_INT_EN));
    svpwm_con0 = p_cfg->hwp_intr_en                        ? 
                 (svpwm_con0 | LL_SVPWM_CON0_HWP_INT_EN)   :
                 (svpwm_con0 & ~(LL_SVPWM_CON0_HWP_INT_EN));
    svpwm_con0 = p_cfg->load_data_intr_en                       ?
                 (svpwm_con0 | LL_SVPWM_CON0_LOADDATA_INT_EN)   :
                 (svpwm_con0 & ~(LL_SVPWM_CON0_LOADDATA_INT_EN));
    
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 |= svpwm_con0;
    );
}

/**
  * @brief  Low layer SVPWM module configuration
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_SVPWM_CFG)
  * @retval None
  */
void ll_svpwm_config(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_CFG *p_cfg)
{
    TX_ASSERT(p_svpwm == SVPWM);
    TX_ASSERT(p_cfg->dead_time < 4096);
    TX_ASSERT(p_cfg->match_cnt < 131071);
    
    u32 svpwm_con = p_svpwm->CON0 & (
                    LL_SVPWM_CON0_DER_INT_EN      | 
                    LL_SVPWM_CON0_MATCH_INT_EN    |
                    LL_SVPWM_CON0_HWP_INT_EN      |
                    LL_SVPWM_CON0_LOADDATA_INT_EN);
    
    svpwm_con |= p_cfg->dead_time_en ? LL_SVPWM_CON0_DT_EN : 0;
    svpwm_con |= p_cfg->hwp_en ? LL_SVPWM_CON0_HWPRT_EN : 0;
    svpwm_con |= p_cfg->pwm_out_invt_en ? LL_SVPWM_CON0_PWMOUTINVT : 0;
    svpwm_con |= p_cfg->match_en ? LL_SVPWM_CON0_MATCH_EN : 0;
    svpwm_con |= p_cfg->dataerr_protect_en ? LL_SVPWM_CON0_DERP_EN : 0;
    svpwm_con |= p_cfg->mode_sel ? LL_SVPWM_CON0_LVL_SEL_THERE : 0;
    svpwm_con |= p_cfg->norm_factor_sel ? LL_SVPWM_CON0_NORM_FACTOR_SEL : 0;
    svpwm_con |= LL_SVPWM_CON0_FAULT_DEBC_TIME(p_cfg->fault_debc_sel);

    LL_SVPWM_REG_OPT(
        p_svpwm->CON1      = LL_SVPWM_CON1_SW_TIME(p_cfg->period);
        p_svpwm->CON2      = LL_SVPWM_CON2_DT_TIME(p_cfg->dead_time);
        p_svpwm->CON3      = LL_SVPWM_CON3_NORM_FACTOR(p_cfg->norm_factor);
        p_svpwm->CON4      = LL_SVPWM_CON4_FAULT_DECT_DIS(p_cfg->hwp_fault_dis);
        p_svpwm->CON5      = LL_SVPWM_CON5_FAULT_INVERT(p_cfg->hwp_fault_invert);
        p_svpwm->MATCH_CON = LL_SVPWM_MATCH_CON(p_cfg->match_cnt);
        p_svpwm->ADC_CON   = LL_SVPWM_ADC_AUTO_EN(p_cfg->adc_auto_chn);
        p_svpwm->REFA      = LL_SVPWM_REFA(p_cfg->phase_a_val);
        p_svpwm->REFB      = LL_SVPWM_REFB(p_cfg->phase_b_val);
        p_svpwm->REFC      = LL_SVPWM_REFC(p_cfg->phase_c_val);
        p_svpwm->CON0      = svpwm_con;
    );
    SYSCTRL_REG_OPT(
        SYSCTRL->PWM_KEY  = LL_SVPWM_KEY_VALUE;
        SYSCTRL->IO_MAP  |= BIT(22);
    );
}

/**
  * @}
  */

/** @defgroup SVPWM_LL_Data_Transfers SVPWM LL Data transfers functions
  * @ingroup  SVPWM_LL_Driver
  * @brief    SVPWM LL Data transfers functions 
  * @{
  */

/**
  * @brief  SVPWM module start function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
void ll_svpwm_start(SVPWM_TypeDef *p_svpwm)
{
    TX_ASSERT(p_svpwm == SVPWM);
    
    SVPWM->CON0 |= LL_SVPWM_CON0_M_EN;
}

/**
  * @brief  SVPWM module stop function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
void ll_svpwm_stop(SVPWM_TypeDef *p_svpwm)
{
    TX_ASSERT(p_svpwm == SVPWM);
    
    SVPWM->CON0 &= ~(LL_SVPWM_CON0_M_EN);
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
