/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_wdt.h
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TX_PHE_LL_WDT_H
#define __TX_PHE_LL_WDT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup wdt_interface_gr WDT Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup WDT_LL_Driver WDT LL Driver
  * @ingroup  wdt_interface_gr
  * @brief Mainly the driver part of the WDT module, which includes \b WDT \b Register 
  * \b Constants, \b WDT \b Exported \b Constants, \b WDT \b Exported \b Struct, \b WDT
  * \b Data \b transfers \b functions, \b WDT \b Initialization \b and \b WDT \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup WDT_LL_Register_Constants WDT LL Register Constants
  * @ingroup  WDT_LL_Driver
  * @brief    WDT LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the WDT 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the WDT register, mainly for convenience. Understand the 
    configuration of the WDT.
    
@endverbatim
  *
  * @{
  */


/***** WDT_CR Register *****/
/*! TYPE_ENUM_LL_WDT_RPL : Reset pulse length.  
 *  Writes have no effect when the configuration parameter WDT_HC_RPL is 1, making
 *  the register bits read-only. This is used to select the number of pclk cycles for which
 *  the system reset stays asserted. The range of values available is 2 to 256 pclk
 *  cycles.  
 *  000 - 2 pclk cycles  
 *  001 - 4 pclk cycles  
 *  010 - 8 pclk cycles  
 *  011 - 16 pclk cycles  
 *  100 - 32 pclk cycles  
 *  101 - 64 pclk cycles  
 *  110 - 128 pclk cycles  
 *  111 - 256 pclk cycles  
 *  @note:
 *      When WDT_SYNC_CLK_MOPE_ENABLE = 1, the total reset pulse
 *      length also includes the reset synchronization delay and the time
 *      taken for pclk to be made available. For details, refer to "System
 *      Resets" on page 35.
 */
#define LL_WDT_RPL(n)                             (((n)&0x7) << 2)
/*! Response mode: TYPE_ENUM_LL_WDT_RMODE
 *  Writes have no effect when the parameter WDT_HC_RMOD = 1, thus this register
 *  becomes read-only. Selects the output response generated to a timeout.  
 *  0 = Generate a system reset.  
 *  1 = First generate an interrupt and if it is not cleared by the time a second timeout
 *  occurs then generate a system reset
 */
#define LL_WDT_RMODE(n)                           (((n)&0x1) << 1)
/*! WDT enable.  
 *  When the configuration parameter WDT_ALWAYS_EN = 0, this bit can be set;
 *  otherwise, it is read-only. This bit is used to enable and disable the DW_apb_wdt.
 *  When disabled, the counter does not decrement. Thus, no interrupts or system
 *  resets are generated.  
 *  The DW_apb_wdt is used to prevent system lock-up. To prevent a software bug from
 *  disabling the DW_apb_wdt, once this bit has been enabled, it can be cleared only by
 *  a system reset.  
 *  0 = WDT disabled.  
 *  1 = WDT enabled.  
 */
#define LL_WDT_EN                                 (1UL << 0)


/***** WDT_TORR Register *****/
/*! Timeout period for initialization.
 *  Writes to these register bits have no effect when the configuration parameter
 *  WDT_HC_TOP = 1 or WDT_ALWAYS_EN = 1. Used to select the timeout period that
 *  the watchdog counter restarts from for the first counter restart (kick). This register
 *  should be written after reset and before the WDT is enabled.
 *  A change of the TOP_INIT is seen only once the WDT has been enabled, and any
 *  change after the first kick is not seen as subsequent kicks use the period specified by
 *  the TOP bits.
 *  The range of values is limited by the WDT_CNT_WIDTH. If TOP_INIT is programmed
 *  to select a range that is greater than the counter width, the timeout period is
 *  truncated to fit to the counter width. This affects only the non-user specified values as
 *  users are limited to these boundaries during configuration.  
 *  The range of values available for a 32-bit watchdog counter are:  
 *  Where i = TOP_INIT and  
 *  t = timeout period  
 *  For i = 0 to 15  
 *  if WDT_USE_FIX_TOP==1  
 *  t = 2(16 + i)  
 *  else  
 *  t = WDT_USER_TOP_INIT_(i)  
 *  Reset Value: Configuration parameter WDT_DFLT_TOP_INIT
 *  @note: These bits exist only when the configuration parameter WDT_DUAL_TOP =
 *  1, otherwise, they are fixed at zero.
 */
#define LL_WDT_TOP_INIT(n)                        (((n)&0xF) << 4)
/*! Timeout period.
 *  Writes have no effect when the configuration parameter WDT_HC_TOP = 1, thus
 *  making this register read-only. This field is used to select the timeout period from
 *  which the watchdog counter restarts. A change of the timeout period takes effect only
 *  after the next counter restart (kick).
 *  The range of values is limited by the WDT_CNT_WIDTH. If TOP is programmed to
 *  select a range that is greater than the counter width, the timeout period is truncated
 *  to fit to the counter width. This affects only the non-user specified values as users are
 *  limited to these boundaries during configuration.  
 *  The range of values available for a 32-bit watchdog counter are:  
 *  Where i = TOP and  
 *  t = timeout period  
 *  For i = 0 to 15  
 *  if WDT_USE_FIX_TOP==1  
 *  t = 2(16 + i)  
 *  else  
 *  t = WDT_USER_TOP_(i)  
 */
#define LL_WDT_TOP(n)                             (((n)&0xF) << 0)


/***** WDT_CCVR Register *****/
/*! This register, when read, is the current value of the internal
 *  counter. This value is read coherently when ever it is read, which
 *  is relevant when the APB_DATA_WIDTH is less than the counter
 *  width.
 */


/***** WDT_CRR Register *****/
/*! This register is used to restart the WDT counter. As a safety feature to prevent
 *  accidental restarts, the value 0x76 must be written. A restart also clears the WDT
 *  interrupt. Reading this register returns zero.
 */
#define LL_WDT_CNT_RESTART_KEY                    (((0x76)))


/***** WDT_STAT Register *****/
/*! This register shows the interrupt status of the WDT.  
 *  1 = Interrupt is active regardless of polarity.  
 *  0 = Interrupt is inactive.  
 */
#define LL_WDT_INT_PENDING                        (1UL << 0)


/***** WDT_EOI Register *****/
/*! Clears the watchdog interrupt. This can be used to clear the interrupt without restarting
 *  the watchdog counter. 
 */
#define LL_WDT_INT_PEND_CLR                       (1UL << 0)

/**
  * @}
  */

/** @defgroup WDT_LL_Exported_Constants WDT LL Exported Constants
  * @ingroup  WDT_LL_Driver
  * @brief    WDT LL external constant definition
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
  * @brief Enumeration constant for low layer WDT Reset pulse cycles
  */
typedef enum {
    /*! watchdog  Reset pulse cycles : 2
     */
    LL_WDT_RPL_2PCLK = 0,
    /*! watchdog  Reset pulse cycles : 4
     */
    LL_WDT_RPL_4PCLK,
    /*! watchdog  Reset pulse cycles : 8
     */
    LL_WDT_RPL_8PCLK,
    /*! watchdog  Reset pulse cycles : 16
     */
    LL_WDT_RPL_16PCLK,
    /*! watchdog  Reset pulse cycles : 32
     */
    LL_WDT_RPL_32PCLK,
    /*! watchdog  Reset pulse cycles : 64
     */
    LL_WDT_RPL_64PCLK,
    /*! watchdog  Reset pulse cycles : 128
     */
    LL_WDT_RPL_128PCLK,
    /*! watchdog  Reset pulse cycles : 256
     */
    LL_WDT_RPL_256PCLK,
} TYPE_ENUM_LL_WDT_RPL;

/**
  * @brief Enumeration constant for low layer WDT mode selector
  */
typedef enum {
    /*! watchdog response : Reset
     */
    LL_WDT_RMODE_RST = 0,
    /*! watchdog response : interrupt, then reset
     */
    LL_WDT_RMODE_INT_RST,
} TYPE_ENUM_LL_WDT_RMODE;

/*! watchdog timeout : 2^n(n > 1) s.
 *  @note : n > 1
 */
#define LL_WDT_IO_SECOND_2PWR(n)                       (((n)-1) & 0xF)


  
/***** LL API AND DRIVER API *****/



/**
  * @}
  */

/** @defgroup WDT_LL_Exported_Struct WDT LL Exported Struct
  * @ingroup  WDT_LL_Driver
  * @brief    WDT LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the WDT registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_wdt_init. Function, you can configure the WDT module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief WDT low layer config struct
  */
typedef struct __ll_wdt_cfg {
    /*! watchdog timeout period , ref LL_WDT_TOP_INIT  & LL_WDT_TOP, the clock is RC32KHZ
     */
    u8                      timeout;
    /*! watchdog mode, LL_WDT_RMOD(int_en) will be used
     */
    TYPE_ENUM_LL_WDT_RMODE  mode;
} TYPE_LL_WDT_CFG;

/**
  * @brief WDT low layer init struct
  */
typedef struct __ll_wdt_init {
    u8 reserved;
} TYPE_LL_WDT_INIT;

/**
  * @}
  */

/** @defgroup WDT_LL_Interrupt WDT LL Interrupt Handle function
  * @brief   WDT LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the WDT  
    Interrupt Handle function.

    how to use?

    The WDT interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the WDT in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup WDT_LL_Inti_Cfg WDT LL Initialization And Configuration
  * @brief    WDT LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the WDT data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  wdt_init
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_WDT_INIT
  * @retval None
  */
void ll_wdt_init(WDT_TypeDef *p_wdt, TYPE_LL_WDT_INIT *p_init);

/**
  * @brief  wdt_deinit
  * @param  p_wdt: pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_deinit(WDT_TypeDef *p_wdt);

/**
  * @brief  watchdog config
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @param  p_cfg : pointer to the init stuct TYPE_LL_WDT_CFG
  * @retval None
  */
void ll_wdt_config(WDT_TypeDef *p_wdt, TYPE_LL_WDT_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup WDT_LL_Data_Transfers WDT LL Data transfers functions
  * @brief    WDT LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the WDT data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  wdt_start
  * @param  p_wdt : pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_start(WDT_TypeDef *p_wdt);

/**
  * @brief  wdt stop
  * @param  p_wdt: pointer to the hardware WDT_TypeDef
  * @retval None
  */
void ll_wdt_stop(WDT_TypeDef *p_wdt);

/**
 * @brief  LL_WDT_GET_INTERRUPT_PENDING
 * @param  p_wdt: pointer to the hardware WDT_TypeDef *p_wdt
 * @retval pending
 */
#define LL_WDT_GET_PENDING(p_wdt)            (p_wdt->WDT_STAT & LL_WDT_INT_PENDING)

/**
  * @brief  ll_wdt_clear_interrupt_pending : use for clear interrupt pending 
  * @param  p_wdt: pointer to the hardware WDT_TypeDef *p_wdt
  * @retval None
  */
__STATIC_INLINE void ll_wdt_clear_done_pending(WDT_TypeDef *p_wdt) {
    p_wdt->WDT_EOI |= LL_WDT_INT_PEND_CLR;
}

/**
  * @brief  ll_wdt_feed : use for clear watchdog counter
  * @param  p_wdt: pointer to the hardware WDT_TypeDef *p_wdt
  * @retval None
  */
__STATIC_INLINE void ll_wdt_feed(WDT_TypeDef *p_wdt) {
    p_wdt->WDT_CRR = LL_WDT_CNT_RESTART_KEY;
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

#endif //__TX_PHE_LL_WDT_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
