/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_gpio.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the GPIO LL firmware functions.
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
#include "tx_phe_ll_gpio.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup gpio_interface_gr GPIO Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup GPIO_LL_Driver GPIO LL Driver
  * @ingroup  gpio_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup GPIO_LL_Interrupt GPIO LL Interrupt Handle function
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup GPIO_LL_Inti_Cfg GPIO LL Initialization And Configuration
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL Initialization And Configuration
  * @{
  */

/** 
  * @brief  Low layer set the gpio dir value.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @param  dir     : The dir to be set.
  * @retval None.
  * @note   GPIOB0 ~ GPIOB11 are the common outputs of the modules SPWM, SVPWM, ADTIMER and EPWM. 
  *         If you want to use GPIOB0 ~ GPIOB11 as general-purpose IO, you need to pay attention 
  *         to whether SPWM, SVPWM, ADTIMER and EPWM are already occupied (that is, other modules 
  *         have these The pin has been initialized. If it is occupied, it cannot be initialized 
  *         to general IO.
  */
void ll_gpio_dir(GPIO_TypeDef *p_gpio, u32 pin_bits, TYPE_ENUM_LL_GPIO_PIN_DIRECTION dir)
{
    /* Check the parameters */
    TX_ASSERT((p_gpio == GPIOA) || (p_gpio == GPIOB) || (p_gpio == GPIOC) || (p_gpio == GPIOD));
    
    if(LL_GPIO_PIN_INPUT == dir){
        p_gpio->DIR &= ~pin_bits;
    } else if(LL_GPIO_PIN_OUTPUT == dir){
        p_gpio->DIR |= pin_bits;
    }   
}

/** 
  * @brief  Low layer Set the gpio pin mode(input or output).
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @param  mode    : The PULL mode to be set.
  * @retval None.
  */
void ll_gpio_pull(GPIO_TypeDef *p_gpio, u32 pin_bits, TYPE_ENUM_LL_GPIO_PIN_PULL_MODE mode)
{
    /* Check the parameters */
    TX_ASSERT((p_gpio == GPIOA) || (p_gpio == GPIOB) || (p_gpio == GPIOC) || (p_gpio == GPIOD));
    
    if(LL_GPIO_OPENDRAIN_PULL_MASK & mode ){
        p_gpio->OD |= pin_bits;
    } else {
        p_gpio->OD &= ~pin_bits;
    }
    
    if(LL_GPIO_PULL_NONE & mode){
        p_gpio->PU0EN &= ~pin_bits;
        p_gpio->PD0EN &= ~pin_bits; 
    } else if(LL_GPIO_PULL_UP & mode){
        p_gpio->PU0EN |= pin_bits;
    } else if(LL_GPIO_PULL_DOWN & mode){
        p_gpio->PD0EN |= pin_bits;
    }
}

/** 
  * @brief  Enable the debounce function of GPIO.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @param  clk_sel : GPIO debounce clock source selection.
  * @param  div_val : The division ratio of the clock source.
  * @retval None.
  * @note   Time to debounce calculation: 1/clk_sel(The actual clock)*
  */
void ll_gpio_debounce_enable(GPIO_TypeDef *p_gpio, u16 pin_bits, TYPE_ENUM_DEBOUNCE_CLK_SEL clk_sel, u8 div_val)
{
    /* Check the parameters */
    TX_ASSERT((p_gpio == GPIOA) || (p_gpio == GPIOB) || (p_gpio == GPIOC) || (p_gpio == GPIOD));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(p_gpio == GPIOA) {
        /* Enable the GPIOA system synchronization function. */
        /* Select the system clock source. */
        SYSCTRL->CLK_CON0 &= ~(BIT(17)|BIT(18));
        SYSCTRL->CLK_CON0 |= clk_sel << 17;
        
        /* Select the clock division ratio. */
        SYSCTRL->CLK_CON3 &= ~0xFF;
        SYSCTRL->CLK_CON3 |= div_val;
        
        /* Enable debounce function. */
        SYSCTRL->SYS_CON3 |= pin_bits;
    } else if(p_gpio == GPIOB) {
        /* Enable the GPIOB system synchronization function. */
        /* Select the system clock source. */
        SYSCTRL->CLK_CON0 &= ~(BIT(19)|BIT(20));
        SYSCTRL->CLK_CON0 |= clk_sel << 19;
        
        /* Select the clock division ratio. */
        SYSCTRL->CLK_CON3 &= ~0xFF00;
        SYSCTRL->CLK_CON3 |= div_val << 8;
        
        /* Enable debounce function. */
        SYSCTRL->SYS_CON3 |= pin_bits << 16;
    } else if(p_gpio == GPIOC) {
        /* Enable the GPIOC system synchronization function. */
        /* Select the system clock source. */
        SYSCTRL->CLK_CON0 &= ~(BIT(21)|BIT(22));
        SYSCTRL->CLK_CON0 |= clk_sel << 21;
        
        /* Select the clock division ratio. */
        SYSCTRL->CLK_CON3 &= ~0xFF0000;
        SYSCTRL->CLK_CON3 |= div_val << 16;
        
        /* Enable debounce function. */
        SYSCTRL->SYS_CON4 |= pin_bits;
    } else if(p_gpio == GPIOD) {
        /* Enable the GPIOD system synchronization function. */
        /* Select the system clock source. */
        SYSCTRL->CLK_CON0 &= ~(BIT(23)|BIT(24));
        SYSCTRL->CLK_CON0 |= clk_sel << 23;
        
        /* Select the clock division ratio. */
        SYSCTRL->CLK_CON3 &= ~0xFF000000;
        SYSCTRL->CLK_CON3 |= div_val << 24;
        
        /* Enable debounce function. */
        SYSCTRL->SYS_CON4 |= pin_bits << 16;
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/** 
  * @brief  Disable the debounce function of GPIO.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @retval None.
  */
void ll_gpio_debounce_disable(GPIO_TypeDef *p_gpio, u16 pin_bits)
{
    /* Check the parameters */
    TX_ASSERT((p_gpio == GPIOA) || (p_gpio == GPIOB) || (p_gpio == GPIOC) || (p_gpio == GPIOD));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(p_gpio == GPIOA) {
        /* disable debounce function. */
        SYSCTRL->SYS_CON3 &= ~pin_bits;
    } else if(p_gpio == GPIOB) {
        /* disable debounce function. */
        SYSCTRL->SYS_CON3 &= ~(pin_bits << 16);
    } else if(p_gpio == GPIOC) {
        /* disable debounce function. */
        SYSCTRL->SYS_CON4 &= ~pin_bits;
    } else if(p_gpio == GPIOD) {
        /* disable debounce function. */
        SYSCTRL->SYS_CON4 &= ~(pin_bits << 16);
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/**
  * @}
  */

/** @defgroup GPIO_LL_Data_Transfers GPIO LL Data transfers functions
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL Data transfers functions 
  * @{
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

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
