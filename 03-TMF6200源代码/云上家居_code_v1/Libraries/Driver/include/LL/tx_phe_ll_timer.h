/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_timer.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    08-04-2019
  * @brief   This file contains all the TIMER LL firmware functions.
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
#ifndef __TX_PHE_LL_TIMER_H
#define __TX_PHE_LL_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup timer_interface_gr TIMER Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup TIMER_LL_Driver TIMER LL Driver
  * @ingroup  timer_interface_gr
  * @brief Mainly the driver part of the TIMER module, which includes \b TIMER \b Register 
  * \b Constants, \b TIMER \b Exported \b Constants, \b TIMER \b Exported \b Struct, \b TIMER
  * \b Data \b transfers \b functions, \b TIMER \b Initialization \b and \b TIMER \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup TIMER_LL_Register_Constants TIMER LL Register Constants
  * @ingroup  TIMER_LL_Driver
  * @brief    TIMER LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in the TIMER register. In the 
    process of configuration, the macro definition can be directly called to configure 
    the TIMER register, mainly for convenience. Understand the configuration of the TIMER.
    
@endverbatim
  *
  * @{
  */

/***** TMRx_CON *****/
/*! RW, TIMERx pending, write 1 clear pending
 */
#define LL_TMR_CON_TIMER_PNG                (1UL << 15)
/*! RW, TIMERx capture pending, write 1 clear pending
 */
#define LL_TMR_CON_CAPTURE_PNG              (1UL << 14)
/*! RW, TIMERx timer mode interrupt enable
 */
#define LL_TMR_CON_TMR_INTR_EN              (1UL << 13)
/*! RW, TIMERx capture mode interrupt enable
 */
#define LL_TMR_CON_CAP_INTR_EN              (1UL << 12)
/*! RW, TIMERx timer's prescaler setting
 */
#define LL_TMR_CON_PSC(n)                   (((n)&0x7) << 8)
/*! RW, TIMERx capture mode edge select
 */
#define LL_TMR_CON_CAP_EDGE(n)              (((n)&0x3) << 6)
/*! RW, TIMERx mode select
 */
#define LL_TMR_CON_MODE(n)                  (((n)&0x3) << 4)
#define LL_TMR_CON_MODE_DIS_MASK            (0x03UL << 4)
/*! RW, TIMERx timer counter source select
 */
#define LL_TMR_CON_INC_SRC(n)               (((n)&0x7) << 0)


/***** TMRx_PR *****/
/*! RW, TIMERx Prescaler register
 */
#define LL_TMR_TIMER_PR(n)                  (((n)&0xFFFF) << 0)


/***** TMRx_CNT *****/
/*! RW, TIMERx Count register
 */
#define LL_TMR_TIMER_CNT(n)                 (((n)&0xFFFF) << 0)


/***** TMRx_PWM *****/
/*! RW, TIMERx Pulse width modulation
 */
#define LL_TMR_TIMER_PWM(n)                 (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup TIMER_LL_Exported_Constants TIMER LL Exported Constants
  * @ingroup  TIMER_LL_Driver
  * @brief    TIMER LL external constant definition
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


/***** LL API *****/
/**
  * @brief Enumeration constant for low layer TIMER Prescaler for timer
  */
typedef enum {
    /*! Timer clock source 0 prescaler
     */
    LL_TMR_TIMER_PSC_0 = 0,
    /*! Timer clock source 2 prescaler
     */
    LL_TMR_TIMER_PSC_2,
    /*! Timer clock source 4 prescaler
     */
    LL_TMR_TIMER_PSC_4,
    /*! Timer clock source 8 prescaler
     */
    LL_TMR_TIMER_PSC_8,
    /*! Timer clock source 16 prescaler
     */
    LL_TMR_TIMER_PSC_16,
    /*! Timer clock source 32 prescaler
     */
    LL_TMR_TIMER_PSC_32,
    /*! Timer clock source 64 prescaler
     */
    LL_TMR_TIMER_PSC_64,
    /*! Timer clock source 128 prescaler
     */
    LL_TMR_TIMER_PSC_128,
}TYPE_ENUM_LL_TMR_PSC;

/**
  * @brief Enumeration constant for low layer TIMER Trigger mode selection in timer's capture mode
  */
typedef enum {
    /*! Timer trigger source selection: RISING
     */
    LL_TMR_EDGE_SEL_RISING = 0,
    /*! Timer trigger source selection: FALLING
     */
    LL_TMR_EDGE_SEL_FALLING,
    /*! Timer trigger source selection: RISING and FALLING
     */
    LL_TMR_EDGE_SEL_RISING_FALLING,
}TYPE_ENUM_LL_TMR_CAP_EDGE;

/**
  * @brief Enumeration constant for low layer TIMER clock source
  */
typedef enum {
    /*! Timer clock source selection: pin rising
     */
    LL_TMR_SOURCE_PIN_RISING = 0,
    /*! Timer clock source selection: pin falling
     */
    LL_TMR_SOURCE_PIN_FALLING,
    /*! Timer clock source selection: half of 26M
     */
    LL_TMR_SOURCE_HALF_26M,
    /*! Timer clock source selection: half of 13K
     */
    LL_TMR_SOURCE_HALF_32K,
    /*! Timer clock source selection: the rising of sys clk
     */
    LL_TMR_SOURCE_SYS_RISING,
}TYPE_ENUM_LL_TMR_SRC_SEL;

  
/***** LL API AND DRIVER API *****/
/**
  * @brief Enumeration constant for low layer TIMER type of timer IO map
  */
typedef enum {
    /*! Select the pin of the timer io map none
     */
    LL_TMR_IO_MAP_NONE = 0,
    /*! Select the pin of the timer io map0
     */
    LL_TMR_IO_MAP0     = 0x01,
    /*! Select the pin of the timer io map1
     */
    LL_TMR_IO_MAP1     = 0x02,
}TYPE_ENUM_LL_TMR_IO_MAP;

/**
  * @brief Enumeration constant for low layer TIMER the pattern of the timer
  */
typedef enum {
    /*! Timer mode selection: disable
     */
    LL_TMR_MODE_SEL_DISABLE = 0,
    /*! Timer mode selection: counter mode
     */
    LL_TMR_MODE_SEL_COUNTER,
    /*! Timer mode selection: PWM mode
     */
    LL_TMR_MODE_SEL_PWM,
    /*! Timer mode selection: capture mode
     */
    LL_TMR_MODE_SEL_CAPTURE,
}TYPE_ENUM_LL_TMR_MODE_SEL;

/**
  * @brief Enumeration constant grouped for low layer TIMER 
  */
typedef enum {
    /*! Timer selection: group 0
     */
    LL_TMR_TIMER_GROUP0 = 0,
    /*! Timer selection: group 1
     */
    LL_TMR_TIMER_GROUP1,
    /*! Timer selection: group 2
     */
    LL_TMR_TIMER_GROUP2,
    /*! Timer selection: group 3
     */
    LL_TMR_TIMER_GROUP3,
}TYPE_ENUM_LL_TMR_GROUP;

/**
  * @}
  */

/** @defgroup TIMER_LL_Exported_Struct TIMER LL Exported Struct
  * @ingroup  TIMER_LL_Driver
  * @brief    TIMER LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the TIMER registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_timer_init. Function, you can configure the TIMER module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief Enumeration constant for low layer TIMER Configure the timer structure
  */
typedef struct __ll_timer_irq_cfg {
    /*! timer interrupt enable
     */
    bool             intr_en;
    /*! capture interrupt enable
     */
    bool             capture_intr_en;
} TYPE_LL_TIMER_IRQ_CFG;

/**
  * @brief Enumeration constant for low layer TIMER Configure the timer structure
  */
typedef struct __ll_timer_cfg {
    /*! count cycle value.
     *  Note: The TIMER period can be set to a maximum of 100ms, 
     *  and a period larger than 100ms cannot be supported. 
     *  For example: if you want 100ms, then period=1s/100ms=10.
     */
    u16                       period;
    /*! the initial value of count
     */
    u16                       count;
    /*! pwm value
     */
    u16                       duty;
    /*! prescaler value
     */
    TYPE_ENUM_LL_TMR_PSC      prescaler;
    /*! trigger source selection
     */
    TYPE_ENUM_LL_TMR_CAP_EDGE edge_sel;
    /*! timer mode selection
     */
    TYPE_ENUM_LL_TMR_MODE_SEL mode_sel;
    /*! timer clock source selection
     */
    TYPE_ENUM_LL_TMR_SRC_SEL  timer_src_sel;
    /*! IO map selection for timer
     *  @note The correspondence table of IO_MAP is as follows:
     *  ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
     *  ©¦       timer         ©¦      mode      ©¦    IO_MAP0   ©¦   IO_MAP1   ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PC12     ©¦     PD9     ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER0        ©¦ External clock ©¦     PA0      ©¦     PD4     ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA0      ©¦     PD4     ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PC13     ©¦     PD10    ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER1        ©¦ External clock ©¦     PA1      ©¦     PD5     ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA1      ©¦     PD5     ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PC14     ©¦     PD11    ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER2        ©¦ External clock ©¦     PA2      ©¦     PC15    ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA2      ©¦     PC15    ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PC15     ©¦     PD12    ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER3        ©¦ External clock ©¦     PA3      ©¦     PC14    ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA3      ©¦     PC14    ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PD6      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER4        ©¦ External clock ©¦     PA4      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA4      ©¦             ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PD7      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER5        ©¦ External clock ©¦     PA5      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PA5      ©¦             ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PD8      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER6        ©¦ External clock ©¦     PC13     ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PC13     ©¦             ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦      PWM       ©¦     PD9      ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦       TIMER7        ©¦ External clock ©¦     PC12     ©¦             ©¦
     *  ©¦                     ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦                     ©¦     capture    ©¦     PC12     ©¦             ©¦
     *  ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
     */
    TYPE_ENUM_LL_TMR_IO_MAP   io_map;
} TYPE_LL_TIMER_CFG;

/**
  * @brief Enumeration constant for low layer TIMER Configure the timer structure
  */
typedef struct __ll_timer_init { 
   u8 reserved;
} TYPE_LL_TIMER_INIT;

/**
  * @}
  */

/** @defgroup TIMER_LL_Interrupt TIMER LL Interrupt Handle function
  * @brief   TIMER LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the TIMER  
    Interrupt Handle function.

    how to use?

    The TIMER interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the TIMER in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup TIMER_LL_Inti_Cfg TIMER LL Initialization And Configuration
  * @brief    TIMER LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the TIMER data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer timer init function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_init : Configure the p_timer initialization structure
  * @retval None
  */
void ll_timer_init(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_INIT *p_init);

/**
  * @brief  Low layer timer delete init function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
void ll_timer_deinit(TIMER_TypeDef *p_timer);

/**
  * @brief  Low layer timer interrupt config function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_cfg  : Configure the p_timer interrupt initialization structure
  * @retval None
  */
void ll_timer_irq_config(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_IRQ_CFG *p_cfg);

/**
  * @brief  Low layer timer Configure function
  * @param  p_timer: Structure pointer set to TIMER0 to TIMER7
  * @param  p_init : Configure the p_timer initialization structure
  * @retval None
  */
void ll_timer_config(TIMER_TypeDef *p_timer, TYPE_LL_TIMER_CFG *p_init);

/**
  * @}
  */
  
/** @defgroup TIMER_LL_Data_Transfers TIMER LL Data transfers functions
  * @brief    TIMER LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the TIMER data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer timer start function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @param  mode_sel : Select to enable the timer mode.
  * @retval None
  */
void ll_timer_start(TIMER_TypeDef *p_timer,  
                    TYPE_ENUM_LL_TMR_MODE_SEL mode_sel);

/**
  * @brief  Low layer timer disable function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
void ll_timer_stop(TIMER_TypeDef *p_timer);

/**
  * @brief  Low layer timer check interrupt enable
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
#define LL_TIMER_CHECK_INTERRUPT_ENABLE(p_timer)                    ((p_timer)->TMR_CON & LL_TMR_CON_TMR_INTR_EN)

/**
  * @brief  Low layer timer check capture interrupt enable
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
#define LL_TIMER_CHECK_CAPTURE_INTERRUPT_ENABLE(p_timer)            ((p_timer)->TMR_CON & LL_TMR_CON_CAP_INTR_EN)

/**
  * @brief  Low layer timer get capture value function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
#define LL_TIMER_GET_CAPTURE_VALUE(p_timer)                         (LL_TMR_TIMER_PWM(p_timer->TMR_PWM))
  
/**
  * @brief  Low layer timer get interrupt pending function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
#define LL_TIMER_GET_PENDING(p_timer)                               ((p_timer)->TMR_CON & LL_TMR_CON_TIMER_PNG)

/**
  * @brief  Low layer timer get interrupt pending function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
#define LL_TIMER_GET_CAPTURE_PENDING(p_timer)                       ((p_timer)->TMR_CON & LL_TMR_CON_CAPTURE_PNG)

/**
  * @brief  Low layer timer clear interrupt pending function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_clear_interrupt_pending(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON |= LL_TMR_CON_TIMER_PNG;
}
                                                      
/**
  * @brief  Low layer timer clear capture interrupt pending function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_clear_capture_interrupt_pending(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON |= LL_TMR_CON_CAPTURE_PNG;
}

/**
  * @brief  Low layer timer enable interrupt function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_interrupt_enable(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON |= LL_TMR_CON_TMR_INTR_EN;
}

/**
  * @brief  Low layer timer enable capture interrupt function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_capture_interrupt_enable(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON |= LL_TMR_CON_CAP_INTR_EN;
}

/**
  * @brief  Low layer timer disable interrupt function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_interrupt_disable(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON &= ~LL_TMR_CON_TMR_INTR_EN;
}

/**
  * @brief  Low layer timer disable capture interrupt function
  * @param  p_timer  : Structure pointer set to TIMER0 to TIMER7
  * @retval None
  */
__STATIC_INLINE void ll_timer_capture_interrupt_disable(TIMER_TypeDef *p_timer) {
    p_timer->TMR_CON &= ~LL_TMR_CON_CAP_INTR_EN;
}

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

#endif //__TX_PHE_LL_TIMER_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
