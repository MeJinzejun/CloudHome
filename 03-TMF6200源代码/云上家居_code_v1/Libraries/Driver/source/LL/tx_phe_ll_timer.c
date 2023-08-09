/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_timer.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    08-04-2019
  * @brief   This file contains all the TIMER LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  * The correspondence table of IO_MAP is as follows:
  * ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  * ©¦       timer         ©¦      mode      ©¦  IO_MAP0  ©¦  IO_MAP1  ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PC12    ©¦    PD9    ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER0        ©¦ External clock ©¦   PA0     ©¦    PD4    ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA0     ©¦    PD4    ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PC13    ©¦    PD10   ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER1        ©¦ External clock ©¦   PA1     ©¦    PD5    ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA1     ©¦    PD5    ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PC14    ©¦    PD11   ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER2        ©¦ External clock ©¦   PA2     ©¦    PC15   ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA2     ©¦    PC15   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PC15    ©¦    PD12   ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER3        ©¦ External clock ©¦   PA3     ©¦    PC14   ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA3     ©¦    PC14   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PD6     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER4        ©¦ External clock ©¦   PA4     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA4     ©¦           ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PD7     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER5        ©¦ External clock ©¦   PA5     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PA5     ©¦           ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PD8     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER6        ©¦ External clock ©¦   PC13    ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PC13    ©¦           ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦      PWM       ©¦   PD9     ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦       TIMER7        ©¦ External clock ©¦   PC12    ©¦           ©¦
  * ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦                     ©¦     capture    ©¦   PC12    ©¦           ©¦
  * ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_timer.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup timer_interface_gr TIMER Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup TIMER_LL_Driver TIMER LL Driver
  * @ingroup  timer_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup TIMER_LL_Interrupt TIMER LL Interrupt Handle function
  * @ingroup  TIMER_LL_Driver
  * @brief   TIMER LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup TIMER_LL_Inti_Cfg TIMER LL Initialization And Configuration
  * @ingroup  TIMER_LL_Driver
  * @brief    TIMER LL Initialization And Configuration
  * @{
  */
  
/**
  * @brief  Low layer timer init function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_init : Configure the p_timer initialization structure
  * @retval None
  */
void ll_timer_init(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_INIT *p_init)
{
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
}

/**
  * @brief  Low layer timer delete init function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
void ll_timer_deinit(TIMER_TypeDef *p_timer)
{
    TYPE_ENUM_LL_TMR_GROUP group_tmr = LL_TMR_TIMER_GROUP0;
    
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
    
    if(p_timer == TIMER0) {
        group_tmr = LL_TMR_TIMER_GROUP0;
    } else if(p_timer == TIMER1) {
        group_tmr = LL_TMR_TIMER_GROUP1;
    } else if(p_timer == TIMER2) {
        group_tmr = LL_TMR_TIMER_GROUP2;
    } else if(p_timer == TIMER3) {
        group_tmr = LL_TMR_TIMER_GROUP3;
    } else if(p_timer == TIMER4) {
        group_tmr = LL_TMR_TIMER_GROUP0;
    } else if(p_timer == TIMER5) {
        group_tmr = LL_TMR_TIMER_GROUP1;
    } else if(p_timer == TIMER6) {
        group_tmr = LL_TMR_TIMER_GROUP2;
    } else if(p_timer == TIMER7) {
        group_tmr = LL_TMR_TIMER_GROUP3;
    }
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if((p_timer == TIMER0) ||
       (p_timer == TIMER1) ||
       (p_timer == TIMER2) ||
       (p_timer == TIMER3) ){
        //clear all io map of p_cfg->group_tmr in TIMER
        SYSCTRL->IO_MAP1 &= ~(BIT(2 * group_tmr)     |
                              BIT(2 * group_tmr + 1) |
                              BIT(2 * group_tmr + 8) |
                              BIT(2 * group_tmr + 9));
        
        /* disable IO_MAP0 debounce function. */
        SYSCTRL->SYS_CON3 &= ~BIT(group_tmr);
        if(group_tmr < LL_TMR_TIMER_GROUP2) {
            /* disable IO_MAP1 debounce function. */
            SYSCTRL->SYS_CON4 &= ~BIT(group_tmr + 4 + 16);
        } else {
            /* disable IO_MAP1 debounce function. */
            SYSCTRL->SYS_CON4 &= ~BIT(15 - group_tmr + 2);
        }
    } else if((p_timer == TIMER4) || 
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) ){
        //clear all io map of p_cfg->group_tmr in TIMER1
        SYSCTRL->IO_MAP1 &= ~(BIT(group_tmr + 16) |
                              BIT(group_tmr + 20));
        
        if(group_tmr < LL_TMR_TIMER_GROUP2) {
            /* disable IO_MAP0 debounce function. */
            SYSCTRL->SYS_CON3 &= ~BIT(group_tmr + 4);
        } else {
            /* disable IO_MAP0 debounce function. */
            SYSCTRL->SYS_CON4 &= ~BIT(13 - group_tmr + 2);
        }
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}
  
/**
  * @brief  Low layer timer io map init function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_cfg  : Configure the p_timer initialization structure
  * @retval None
  */
void ll_timer_io_map(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_CFG *p_cfg)
{
    TYPE_ENUM_LL_TMR_GROUP group_tmr = LL_TMR_TIMER_GROUP0;
    
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
    
    if((p_timer == TIMER4) ||
       (p_timer == TIMER5) ||
       (p_timer == TIMER6) ||
       (p_timer == TIMER7) ){
        TX_ASSERT(p_cfg->io_map < LL_TMR_IO_MAP1);
    }
       
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(p_timer == TIMER0) {
        group_tmr = LL_TMR_TIMER_GROUP0;
    } else if(p_timer == TIMER1) {
        group_tmr = LL_TMR_TIMER_GROUP1;
    } else if(p_timer == TIMER2) {
        group_tmr = LL_TMR_TIMER_GROUP2;
    } else if(p_timer == TIMER3) {
        group_tmr = LL_TMR_TIMER_GROUP3;
    } else if(p_timer == TIMER4) {
        group_tmr = LL_TMR_TIMER_GROUP0;
    } else if(p_timer == TIMER5) {
        group_tmr = LL_TMR_TIMER_GROUP1;
    } else if(p_timer == TIMER6) {
        group_tmr = LL_TMR_TIMER_GROUP2;
    } else if(p_timer == TIMER7) {
        group_tmr = LL_TMR_TIMER_GROUP3;
    }
    
    /* TIMER0 to TIMER3 IO_MAP */
    if((p_timer == TIMER0) ||
       (p_timer == TIMER1) ||
       (p_timer == TIMER2) ||
       (p_timer == TIMER3) ){
        
        //clear all io map of p_cfg->group_tmr in TIMER
        SYSCTRL->IO_MAP1 &= ~(BIT(2 * group_tmr)     | 
                              BIT(2 * group_tmr + 1) |
                              BIT(2 * group_tmr + 8) |
                              BIT(2 * group_tmr + 9));
        
        /* TIMER pwm mode IO_MAP */
        if(LL_TMR_MODE_SEL_PWM == p_cfg->mode_sel) {
            /* TIMER pwm map0 */
            if(p_cfg->io_map & LL_TMR_IO_MAP0) {
                SYSCTRL->IO_MAP1 |= BIT(2 * group_tmr + 8);
            }
            /* TIMER pwm map1 */
            if(p_cfg->io_map & LL_TMR_IO_MAP1) {
                SYSCTRL->IO_MAP1 |= BIT(2 * group_tmr + 9);
            }
        /* TIMER counter/capture mode IO_MAP */
        } else if((LL_TMR_MODE_SEL_COUNTER == p_cfg->mode_sel) || 
                 (LL_TMR_MODE_SEL_CAPTURE == p_cfg->mode_sel)){
            /* TIMER counter/capture map0 */
            if(p_cfg->io_map & LL_TMR_IO_MAP0) {
                SYSCTRL->IO_MAP1 |= BIT(2 * group_tmr);
                
                /* Enable the GPIOA(PA0,PA1,PA2,PA3) system synchronization function. */
                /* Select the system clock source. */
                SYSCTRL->CLK_CON0 &= ~BIT(17);
                SYSCTRL->CLK_CON0 |= BIT(18);
                /* Select the clock division ratio, the default value is 0. */
                SYSCTRL->CLK_CON3 &= ~0xFF;
                /* Enable debounce function. */
                SYSCTRL->SYS_CON3 |= BIT(group_tmr);
            }
            /* TIMER counter/capture map1 */
            if(p_cfg->io_map & LL_TMR_IO_MAP1) {
                SYSCTRL->IO_MAP1 |= BIT(2 * group_tmr + 1);
                
                if(group_tmr < LL_TMR_TIMER_GROUP2) {
                    /* Enable the GPIOD(PD4,PD5) system synchronization function. */
                    /* Select the system clock source. */
                    SYSCTRL->CLK_CON0 &= ~BIT(23);
                    SYSCTRL->CLK_CON0 |= BIT(24);
                    /* Select the clock division ratio, the default value is 0. */
                    SYSCTRL->CLK_CON3 &= ~0xFF000000;
                    /* Enable debounce function. */
                    SYSCTRL->SYS_CON4 |= BIT(group_tmr + 4 + 16);
                } else {
                    /* Enable the GPIOD(PC14,PC15) system synchronization function. */
                    /* Select the system clock source. */
                    SYSCTRL->CLK_CON0 &= ~BIT(21);
                    SYSCTRL->CLK_CON0 |= BIT(22);
                    /* Select the clock division ratio, the default value is 0. */
                    SYSCTRL->CLK_CON3 &= ~0xFF0000;
                    /* Enable debounce function. */
                    SYSCTRL->SYS_CON4 |= BIT(15 - group_tmr + 2);
                }
            }
        }
    /* TIMER4 to TIMER7 IO_MAP */
    } else if((p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) ){
        
        //clear all io map of p_cfg->group_tmr in TIMER1
        SYSCTRL->IO_MAP1 &= ~(BIT(group_tmr + 16) |
                              BIT(group_tmr + 20));
        
        /* TIMER1 pwm mode IO_MAP */
        if(LL_TMR_MODE_SEL_PWM == p_cfg->mode_sel) {
            if(p_cfg->io_map & LL_TMR_IO_MAP0) {
                SYSCTRL->IO_MAP1 |= BIT(group_tmr + 20);
            }
        /* TIMER1 counter/capture mode IO_MAP */
        } else if((LL_TMR_MODE_SEL_COUNTER == p_cfg->mode_sel) || 
                 (LL_TMR_MODE_SEL_CAPTURE == p_cfg->mode_sel)){
            if(p_cfg->io_map & LL_TMR_IO_MAP0) {
                SYSCTRL->IO_MAP1 |= BIT(group_tmr + 16);
                
                if(group_tmr < LL_TMR_TIMER_GROUP2) {
                    /* Enable the GPIOA(PA4,PA5) system synchronization function. */
                    /* Select the system clock source. */
                    SYSCTRL->CLK_CON0 &= ~BIT(17);
                    SYSCTRL->CLK_CON0 |= BIT(18);
                    /* Select the clock division ratio, the default value is 0. */
                    SYSCTRL->CLK_CON3 &= ~0xFF;
                    /* Enable debounce function. */
                    SYSCTRL->SYS_CON3 |= BIT(group_tmr + 4);
                } else {
                    /* Enable the GPIOD(PC12,PC13) system synchronization function. */
                    /* Select the system clock source. */
                    SYSCTRL->CLK_CON0 &= ~BIT(21);
                    SYSCTRL->CLK_CON0 |= BIT(22);
                    /* Select the clock division ratio, the default value is 0. */
                    SYSCTRL->CLK_CON3 &= ~0xFF0000;
                    /* Enable debounce function. */
                    SYSCTRL->SYS_CON4 |= BIT(13 - group_tmr + 2);
                }
            }
        }
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/**
  * @brief  Low layer timer interrupt config function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_cfg  : Configure the p_timer interrupt initialization structure
  * @retval None
  */
void ll_timer_irq_config(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_IRQ_CFG *p_cfg)
{
    u32 timer_con = p_timer->TMR_CON;
    
    timer_con = p_cfg->intr_en                         ? 
                (timer_con | LL_TMR_CON_TMR_INTR_EN)   : 
                (timer_con & (~LL_TMR_CON_TMR_INTR_EN));
    timer_con = p_cfg->capture_intr_en                 ? 
                (timer_con | LL_TMR_CON_CAP_INTR_EN)   : 
                (timer_con & (~LL_TMR_CON_CAP_INTR_EN));
    
    p_timer->TMR_CON = timer_con;
}

/**
  * @brief  Low layer timer init function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_cfg  : Configure the p_timer initialization structure
  * @retval None
  */
void ll_timer_config(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_CFG *p_cfg)
{
    u32 timer_con = p_timer->TMR_CON;
    
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
    
    /* Configure the IO_MAP of the timer */
    ll_timer_io_map(p_timer, p_cfg);
    
    /* Abstract conversion to TMR_CON using timer_con */
    timer_con |= LL_TMR_CON_TIMER_PNG | LL_TMR_CON_CAPTURE_PNG;
    timer_con |= LL_TMR_CON_PSC(p_cfg->prescaler);
    timer_con |= LL_TMR_CON_CAP_EDGE(p_cfg->edge_sel);
//  timer_con |= CON_MODE(p_cfg->mode_sel);
    timer_con |= LL_TMR_CON_INC_SRC(p_cfg->timer_src_sel);
    
    /* Configuring TIMER registers */
    p_timer->TMR_CNT = LL_TMR_TIMER_CNT(p_cfg->count);
    p_timer->TMR_PR  = LL_TMR_TIMER_PR(p_cfg->period);
    p_timer->TMR_PWM = LL_TMR_TIMER_PWM(p_cfg->duty);
    p_timer->TMR_CON = timer_con;
}

/**
  * @}
  */

/** @defgroup TIMER_LL_Data_Transfers TIMER LL Data transfers functions
  * @ingroup  TIMER_LL_Driver
  * @brief    TIMER LL Data transfers functions 
  * @{
  */

/**
  * @brief  Low layer timer start function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @param  mode_sel : Select to enable the timer mode.
  * @retval None
  */
void ll_timer_start(TIMER_TypeDef *p_timer, 
                    TYPE_ENUM_LL_TMR_MODE_SEL mode_sel)
{
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
    p_timer->TMR_CON |= LL_TMR_CON_MODE(mode_sel);
}

/**
  * @brief  Low layer timer disable function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
void ll_timer_stop(TIMER_TypeDef *p_timer)
{
    TX_ASSERT((p_timer == TIMER0) || 
              (p_timer == TIMER1) ||
              (p_timer == TIMER2) ||
              (p_timer == TIMER3) ||
              (p_timer == TIMER4) ||
              (p_timer == TIMER5) ||
              (p_timer == TIMER6) ||
              (p_timer == TIMER7) );
    p_timer->TMR_CON &= ~(LL_TMR_CON_MODE_DIS_MASK);
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
