/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_gpio.h
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TX_PHE_LL_GPIO_H
#define __TX_PHE_LL_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup gpio_interface_gr GPIO Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup GPIO_LL_Driver GPIO LL Driver
  * @ingroup  gpio_interface_gr
  * @brief Mainly the driver part of the GPIO module, which includes \b GPIO \b Register 
  * \b Constants, \b GPIO \b Exported \b Constants, \b GPIO \b Exported \b Struct, \b GPIO
  * \b Data \b transfers \b functions, \b GPIO \b Initialization \b and \b GPIO \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup GPIO_LL_Register_Constants GPIO LL Register Constants
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the GPIO 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the GPIO register, mainly for convenience. Understand the 
    configuration of the GPIO.
    
@endverbatim
  *
  * @{
  */



/**
  * @}
  */

/** @defgroup GPIO_LL_Exported_Constants GPIO LL Exported Constants
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the abstraction 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
  
/***** DRIVER API *****/
#define GPIOB_PWM_PIN_MASK              0x3FFF
#define GPIOB_KEY_VALUE                 ((GPIO_KEY_C ) | (GPIO_KEY_B ) | (GPIO_KEY_A ))


/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for GPIO pin direction select
  */
typedef enum {             
    /*! Set GPIO to input mode.
     */
    LL_GPIO_PIN_INPUT,
    /*! Set GPIO to output mode.
     */
    LL_GPIO_PIN_OUTPUT
} TYPE_ENUM_LL_GPIO_PIN_DIRECTION;

/**
  * @brief Enumeration constant for GPIO pin pull mode select
  */
typedef enum {
    /*! Set the GPIO pull mode to NO.
     */
    LL_GPIO_PULL_NONE           = 0x01,
    /*! Set the GPIO pull mode to pull up mode.
     */
    LL_GPIO_PULL_UP             = 0x02,
    /*! Set the GPIO pull mode to pull down mode.
     */
    LL_GPIO_PULL_DOWN           = 0x04,

    /*! Set the GPIO pull open-drain mode mask.
     */
    LL_GPIO_OPENDRAIN_PULL_MASK = 0x80,
    /*! Set the GPIO pull open-drain mode to NO.
     */
    LL_GPIO_OPENDRAIN_PULL_NONE = 0x81,
    /*! Set the GPIO pull open-drain mode to up mode.
     */
    LL_GPIO_OPENDRAIN_PULL_UP   = 0x82,
    /*! Set the GPIO pull open-drain mode to down mode.
     */
    LL_GPIO_OPENDRAIN_PULL_DOWN = 0x84,
} TYPE_ENUM_LL_GPIO_PIN_PULL_MODE;

/** 
  * @brief  GPIO Bit SET and Bit RESET enumeration 
  */ 
typedef enum{
    /*! BIT_SET means to let the GPIO output a high level.
     */
    LL_GPIO_RESET = 0x00,
    /*! BIT_RESET means to let the GPIO output a low level.
     */
    LL_GPIO_SET,
} TYPE_ENUM_LL_GPIO_BIT_ACTION;

/** 
  * @brief  GPIO debounce clock source selection.
  */
typedef enum {
    /*! Clock source selection for GPIO debounce: crystal oscillator clock 26M.
     */
    LL_GPIO_DEBOUNCE_CLK_26M = 0,
    /*! Clock source selection for GPIO debounce: Internal clock source 8M.
     */
    LL_GPIO_DEBOUNCE_CLK_RC8M,
    /*! Clock source selection for GPIO debounce: System clock.
     */
    LL_GPIO_DEBOUNCE_CLK_SYSCLK,
    /*! Clock source selection for GPIO debounce: Internal clock source 32K.
     */
    LL_GPIO_DEBOUNCE_CLK_RC32K,
} TYPE_ENUM_DEBOUNCE_CLK_SEL;

/**
  * @}
  */

/** @defgroup GPIO_LL_Exported_Struct GPIO LL Exported Struct
  * @ingroup  GPIO_LL_Driver
  * @brief    GPIO LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the GPIO registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_gpio_init. Function, you can configure the GPIO module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */



/**
  * @}
  */

/** @defgroup GPIO_LL_Interrupt GPIO LL Interrupt Handle function
  * @brief   GPIO LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the GPIO  
    Interrupt Handle function.

    how to use?

    The GPIO interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the GPIO in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup GPIO_LL_Inti_Cfg GPIO LL Initialization And Configuration
  * @brief    GPIO LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the GPIO data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/** 
  * @brief  Low layer set the gpio dir value.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @param  dir     : The dir to be set.
  * @retval None.
  */
void ll_gpio_dir(GPIO_TypeDef *p_gpio, u32 pin_bits, TYPE_ENUM_LL_GPIO_PIN_DIRECTION dir);

/** 
  * @brief  Low layer Set the gpio pin mode(input or output).
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @param  mode    : The PULL mode to be set.
  * @retval None.
  */
void ll_gpio_pull(GPIO_TypeDef *p_gpio, u32 pin_bits, TYPE_ENUM_LL_GPIO_PIN_PULL_MODE mode);

/**
  * @}
  */
  
/** @defgroup GPIO_LL_Data_Transfers GPIO LL Data transfers functions
  * @brief    GPIO LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the GPIO data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer set the specified output data port bit.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_bit_set(GPIO_TypeDef* p_gpio, u16 pin_bits) {
    p_gpio->BSRS = ((u32)pin_bits) << 16;
}

/**
  * @brief  Low layer reset the specified output data port bit.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_bit_reset(GPIO_TypeDef* p_gpio, u16 pin_bits) {
    p_gpio->BSRS = ((u32)pin_bits) & 0xFFFF;
}

/**
  * @brief  Low layer Reads the specified GPIO data port pin.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15).
  * @retval The input port pin value.
  */
__STATIC_INLINE TYPE_ENUM_LL_GPIO_BIT_ACTION ll_gpio_read_data_bit(GPIO_TypeDef* p_gpio, u16 pin_bits) {
    TYPE_ENUM_LL_GPIO_BIT_ACTION bit_status = LL_GPIO_RESET;
    
    if ((p_gpio->DR & pin_bits) != (u32)LL_GPIO_RESET) {
        bit_status = LL_GPIO_SET;
    } else {
        bit_status = LL_GPIO_RESET;
    }
    
    return bit_status;
}

/**
  * @brief  Low layer Reads the specified GPIO data port pin.
  * @param  p_gpio: where x can be (A..D) to select the GPIO peripheral.
  * @retval GPIO input data port value.
  */
__STATIC_INLINE u32 ll_gpio_read_data(GPIO_TypeDef* p_gpio) {
    return (p_gpio->DR);
}

/**
  * @brief  Low layer Write the specified output data port bit.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @param  pin_val : gpio set value.
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_write_data_bit(GPIO_TypeDef* p_gpio, u32 pin_bits, u8 pin_val){
    u32 bit_status = p_gpio->DR;
    
    if (pin_val) {
        bit_status |= pin_bits;
    } else {
        bit_status &= ~pin_bits;
    }
    p_gpio->DR = bit_status;
}

/**
  * @brief  Low layer Reads the specified GPIO output data port.
  * @param  p_gpio : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_val: gpio set value.
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_write_data(GPIO_TypeDef* p_gpio, u32 pin_val) {
    p_gpio->DR = pin_val;
}

/**
  * @brief  Low layer Gpio disable external interrupt function.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_interrupt_disable(GPIO_TypeDef* p_gpio, u32 pin_bits) {
    p_gpio->INTMASK &= ~pin_bits;
}

/**
  * @brief  Low layer Gpio enable external interrupt function.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval None.
  */
__STATIC_INLINE void ll_gpio_interrupt_enable(GPIO_TypeDef* p_gpio, u32 pin_bits) {
    p_gpio->INTMASK |= pin_bits;
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
void ll_gpio_debounce_enable(GPIO_TypeDef *p_gpio, u16 pin_bits, TYPE_ENUM_DEBOUNCE_CLK_SEL clk_sel, u8 div_val);

/** 
  * @brief  Disable the debounce function of GPIO.
  * @param  p_gpio  : where x can be (A..D) to select the GPIO peripheral.
  * @param  pin_bits: specifies the port bit to read.
  *                   This parameter can be gpio_pin_x where x can be (0..15). 
  * @retval None.
  */
void ll_gpio_debounce_disable(GPIO_TypeDef *p_gpio, u16 pin_bits);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif //__TX_PHE_LL_GPIO_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
