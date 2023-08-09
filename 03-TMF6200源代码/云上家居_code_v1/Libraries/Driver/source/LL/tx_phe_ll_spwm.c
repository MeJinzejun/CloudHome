/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_spwm.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SPWM LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  *
  *
  *    The SPWM GPIO use as follows:
  *         |-------------------------------------------|
  *         |                   |         GPIO          |
  *         |-------------------------------------------|
  *         | spwm_a0           |         PB3           |
  *         | spwm_a1           |         PB2           |
  *         | spwm_a2           |         PB1           |
  *         | spwm_a3           |         PB0           |
  *         | spwm_b0           |         PB7           |
  *         | spwm_b1           |         PB6           |
  *         | spwm_b2           |         PB5           |
  *         | spwm_b3           |         PB4           |
  *         | spwm_c0           |         PB11          |
  *         | spwm_c1           |         PB10          |
  *         | spwm_c2           |         PB9           |
  *         | spwm_c3           |         PB8           |
  *         | fault_msb2lsb     | {PB15~PB12,PC11~PC0}. |
  *         |-------------------------------------------|
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_spwm.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup spwm_interface_gr SPWM Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup SPWM_LL_Driver SPWM LL Driver
  * @ingroup  spwm_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup SPWM_LL_Interrupt SPWM LL Interrupt Handle function
  * @ingroup  SPWM_LL_Driver
  * @brief   SPWM LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup SPWM_LL_Inti_Cfg SPWM LL Initialization And Configuration
  * @ingroup  SPWM_LL_Driver
  * @brief    SPWM LL Initialization And Configuration
  * @{
  */

/**
  * @brief  spwm_init
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_init : pointer to the init struct TYPE_LL_SPWM_INIT
  * @retval None
  */
void ll_spwm_init(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_INIT *p_init)
{
    TX_ASSERT(p_spwm == SPWM);

    /* reset hardware */
    ll_spwm_soft_reset();
    
    LL_SPWM_REG_OPT(p_spwm->SPWM_CON = LL_SPWM_CFG_EN | LL_SPWM_CLR_PWMEN);

    SYSCTRL_REG_OPT(
        SYSCTRL->IO_MAP &= ~BIT(12);
    );  
}

/**
  * @brief  spwm_deinit
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @retval None
  */
void ll_spwm_deinit(SPWM_TypeDef *p_spwm)
{
    TX_ASSERT(p_spwm == SPWM);
    
    ll_spwm_soft_reset();
    
    LL_SPWM_REG_OPT(p_spwm->SPWM_CON = LL_SPWM_CFG_EN | LL_SPWM_CLR_PWMEN);
    
    SYSCTRL_REG_OPT(
        SYSCTRL->IO_MAP &= ~BIT(12);
    );  
}

/**
  * @brief  spwm config
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_cfg  : pointer to the init struct TYPE_LL_SPWM_IRQ_CFG
  * @retval None
  */
void ll_spwm_irq_config(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_IRQ_CFG *p_cfg)
{
    TX_ASSERT(p_spwm == SPWM);
    
    if(p_cfg->match_intr_en) {
        p_spwm->SPWM_MATCH |= LL_SPWM_MATCH_INT_EN;
    } else {
        p_spwm->SPWM_MATCH &= ~LL_SPWM_MATCH_INT_EN;
    }

    if(p_cfg->fault_intr_en) {
        p_spwm->SPWM_CON |= LL_SPWM_HWP_INT_EN;
    } else {
        p_spwm->SPWM_CON &= ~LL_SPWM_HWP_INT_EN;
    }
    
    if(p_cfg->data_err_intr_en) {
        p_spwm->SPWM_CON |= LL_SPWM_DATAERR_INT_EN_ALL;
    } else {
        p_spwm->SPWM_CON &= ~LL_SPWM_DATAERR_INT_EN_ALL;
    }
    
    if(p_cfg->load_data0_intr_en) {
        p_spwm->SPWM_CNT0 |= LL_SPWM_LOAD_INT_EN;
    } else {
        p_spwm->SPWM_CNT0 &= ~LL_SPWM_LOAD_INT_EN;
    }
    
    if(p_cfg->load_data1_intr_en) {
        p_spwm->SPWM_CNT1 |= LL_SPWM_LOAD_INT_EN;
    } else {
        p_spwm->SPWM_CNT1 &= ~LL_SPWM_LOAD_INT_EN;
    }
    
    if(p_cfg->load_data2_intr_en) {
        p_spwm->SPWM_CNT2 |= LL_SPWM_LOAD_INT_EN;
    } else {
        p_spwm->SPWM_CNT2 &= ~LL_SPWM_LOAD_INT_EN;
    }
}

/**
  * @brief  spwm config
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_cfg : pointer to the init struct TYPE_LL_SPWM_CFG
  * @retval None
  */
void ll_spwm_config(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_CFG *p_cfg)
{ 
    TX_ASSERT(p_spwm == SPWM);
    
    u32 spwm_con, spwm_match, spwm_cnt1, spwm_cnt2, spwm_cnt3;
    
    p_spwm->SPWM_CON |= LL_SPWM_CFG_EN;

    spwm_match = p_spwm->SPWM_MATCH & LL_SPWM_MATCH_INT_EN;
    spwm_match |= LL_SPWM_MATCH_SEL(p_cfg->match_work_mode);
    spwm_match |= LL_SPWM_MATCH_VALUE(p_cfg->match_value);
    spwm_match |= LL_SPWM_MATCH_PENDING_CLR;

    spwm_cnt1 = p_spwm->SPWM_CNT0 & LL_SPWM_LOAD_INT_EN;
    spwm_cnt1 |= p_cfg->pwm_chn[0].cnt_dir ? LL_PWM_CNT_INI_DIR : 0;
    spwm_cnt1 |= p_cfg->pwm_chn[0].carrier_mode ? LL_PWM_CARRIER_MODE : 0;
    spwm_cnt1 |= p_cfg->pwm_chn[0].data_err_dis ? LL_SPWM_DATAERR_DIS : 0;
    spwm_cnt1 |= p_cfg->pwm_chn[0].deadtime_balance_en ? LL_SPWM_DEAD_BALANCE_MODE : 0;
    spwm_cnt1 |= p_cfg->pwm_chn[0].level_mode ? LL_SPWM_TWO_LEVEL : 0;    
    spwm_cnt1 |= LL_PWM_CNT_INI_VAL(p_cfg->pwm_chn[0].cnt_init_val);
    
    spwm_cnt2 = p_spwm->SPWM_CNT1 & LL_SPWM_LOAD_INT_EN;
    spwm_cnt2 |= p_cfg->pwm_chn[1].cnt_dir ? LL_PWM_CNT_INI_DIR : 0;
    spwm_cnt2 |= p_cfg->pwm_chn[1].carrier_mode ? LL_PWM_CARRIER_MODE : 0;
    spwm_cnt2 |= p_cfg->pwm_chn[1].data_err_dis ? LL_SPWM_DATAERR_DIS : 0;
    spwm_cnt2 |= p_cfg->pwm_chn[1].deadtime_balance_en ? LL_SPWM_DEAD_BALANCE_MODE : 0;
    spwm_cnt2 |= p_cfg->pwm_chn[1].level_mode ? LL_SPWM_TWO_LEVEL : 0;    
    spwm_cnt2 |= LL_PWM_CNT_INI_VAL(p_cfg->pwm_chn[1].cnt_init_val);

    spwm_cnt3 = p_spwm->SPWM_CNT2 & LL_SPWM_LOAD_INT_EN;
    spwm_cnt3 |= p_cfg->pwm_chn[2].cnt_dir ? LL_PWM_CNT_INI_DIR : 0;
    spwm_cnt3 |= p_cfg->pwm_chn[2].carrier_mode ? LL_PWM_CARRIER_MODE : 0;
    spwm_cnt3 |= p_cfg->pwm_chn[2].data_err_dis ? LL_SPWM_DATAERR_DIS : 0;
    spwm_cnt3 |= p_cfg->pwm_chn[2].deadtime_balance_en ? LL_SPWM_DEAD_BALANCE_MODE : 0;
    spwm_cnt3 |= p_cfg->pwm_chn[2].level_mode ? LL_SPWM_TWO_LEVEL : 0;    
    spwm_cnt3 |= LL_PWM_CNT_INI_VAL(p_cfg->pwm_chn[2].cnt_init_val);

    spwm_con  = p_spwm->SPWM_CON & (LL_SPWM_DATAERR_INT_EN_ALL | LL_SPWM_HWP_INT_EN);
    spwm_con |= p_cfg->fault_protect_en ? 0 : LL_SPWM_HWP_DIS;
    spwm_con |= p_cfg->output_invert_en ? LL_SPWM_PWM_INVERT : 0;
    spwm_con |= LL_SPWM_SEL(p_cfg->pwm_sel);
    spwm_con |= LL_SPWM_FAULT_DEBOUNCE(p_cfg->fault_debunce_time);
    spwm_con |= LL_SPWM_SYSERR_PROTECT_EN | LL_SPWM_LOAD_INT_EN_ALL;

    p_spwm->SPWM_DATAUSE0 = 0;
    p_spwm->SPWM_DATAUSE1 = 0;
    p_spwm->SPWM_DATAUSE2 = 0;
    p_spwm->SPWM_DATABUF0 = 0;
    p_spwm->SPWM_DATABUF1 = 0;
    p_spwm->SPWM_DATABUF2 = 0;
    
    p_spwm->SPWM_FAULT_INVERT = p_cfg->fault_invert;
    p_spwm->SPWM_FAULT_INFO = p_cfg->fault_disable;
    p_spwm->SPWM_PERIOD0 = p_cfg->period;
    if(p_cfg->dead_time) {
        p_spwm->SPWM_PERIOD1 = ((p_cfg->mini_time & 0xFFF) << 12) | 
                               (p_cfg->dead_time & 0xFFF) ;
    } else {
        p_spwm->SPWM_PERIOD1 = ((p_cfg->mini_time & 0xFFF) << 12) | 
                               (1 & 0xFFF) ;
    }
    
    p_spwm->SPWM_ADCC = p_cfg->match_kick_adc_chn_bits;
    p_spwm->SPWM_MATCH = spwm_match;
    p_spwm->SPWM_CNT0 = spwm_cnt1 | LL_SPWM_CLR_LOAD_PD;
    p_spwm->SPWM_CNT1 = spwm_cnt2 | LL_SPWM_CLR_LOAD_PD;
    p_spwm->SPWM_CNT2 = spwm_cnt3 | LL_SPWM_CLR_LOAD_PD;
    p_spwm->SPWM_CON = spwm_con                     | 
                       LL_SPWM_CLR_DATAERR_PD_ALL   | 
                       LL_SPWM_CLR_HWP_PD           | 
                       LL_SPWM_CLR_PWMEN;
    p_spwm->SPWM_CON &= ~LL_SPWM_CFG_EN;   
}

/**
  * @}
  */

/** @defgroup SPWM_LL_Data_Transfers SPWM LL Data transfers functions
  * @ingroup  SPWM_LL_Driver
  * @brief    SPWM LL Data transfers functions 
  * @{
  */

/**
  * @brief  spwm_start
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @retval None
  * @note   the unused io will free for GPIO
  */
void ll_spwm_start(SPWM_TypeDef *p_spwm)
{   
    TX_ASSERT(p_spwm == SPWM);

    if(LL_SPWM_GET_WORK_STA(p_spwm)) {
        return;
    }
    
    /* switch io to spwm */
    u32 spwm_key = 0;
    u8 spwm_abc_sel = 0;
    spwm_abc_sel = p_spwm->SPWM_CON & LL_SPWM_SEL(0x7);
    if(spwm_abc_sel & LL_SPWM_SEL(0x1)) {
        spwm_key |= PWM_KEY_A;
    } else {
        spwm_key |= GPIO_KEY_A;
    }
    if(spwm_abc_sel & LL_SPWM_SEL(0x2)) {
        spwm_key |= PWM_KEY_B;
    } else {
        spwm_key |= GPIO_KEY_B;
    }    
    if(spwm_abc_sel & LL_SPWM_SEL(0x4)) {
        spwm_key |= PWM_KEY_C;
    } else {
        spwm_key |= GPIO_KEY_C;
    }

    SYSCTRL_REG_OPT(
        SYSCTRL->PWM_KEY = spwm_key;
        SYSCTRL->IO_MAP |= BIT(12);
    );  
    
    /* spwm enable */
    p_spwm->SPWM_DATAUSE0 = 0;
    p_spwm->SPWM_DATAUSE1 = 0;
    p_spwm->SPWM_DATAUSE2 = 0;
    
    LL_SPWM_REG_OPT(
        p_spwm->SPWM_CON |= LL_SPWM_PWM_EN               | 
                            LL_SPWM_CLR_DATAERR_PD_ALL   | 
                            LL_SPWM_CLR_HWP_PD;
    );   
}

/**
  * @brief  spwm stop
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @retval None
  */
void ll_spwm_stop(SPWM_TypeDef *p_spwm)
{     
    TX_ASSERT(p_spwm == SPWM);

    /* spwm disable */
    LL_SPWM_REG_OPT(
        p_spwm->SPWM_CON |= LL_SPWM_CLR_PWMEN            | 
                            LL_SPWM_CLR_DATAERR_PD_ALL   | 
                            LL_SPWM_CLR_HWP_PD;
    );

    SYSCTRL->IO_MAP &= ~(BIT(12));
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
