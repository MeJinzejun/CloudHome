/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/tx_phe_system.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for Phoenix Connectivity line.
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripherals registers hardware
  *
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
#ifndef __TX_PHE_SYSTEM_H
#define __TX_PHE_SYSTEM_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup CMSIS
  * @{
  */

/** @defgroup   CONFIG_SETTING CONFIG_SETTING
  * @brief Various configuration settings
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "typedef.h"
#include "tx_phe.h"

/* Private define ------------------------------------------------------------*/
/*! SDK version : v1.0.9
 */
#define __SDK_VERSION                       0x010009UL
     
//#pragma import(__use_no_semihosting) 

/*! system assert : check param valid
 */
#define USE_ASSERT                         1

/*! DEBUG_LEVEL : debug level select : [none | error | warning | all]
 */
#define DEBUG_LEVEL                         DL_ALL

/*! module debug macro : user can add self-defined module debug switch like this
 *  usage : change XXX to user module
 */
#define XXX_DBG_EN                          1

/*! APP_PARAM_ST_SECTOR : app vip param sector sections(bigger is better, Balanced wear)
 *  usage : the system API auto save param in this section
 */
#define APP_PARAM_SECTORS                  20

/*! SARADC config : force SARADC not use trim infomation 
 */
#define __SARADC_TRIM_FORCE_OFF            0

/*! FADC config : force FADC not use trim infomation 
 */
#define __FADC_TRIM_FORCE_OFF              0

/**
  * @brief SYS_CLK define Max frequence value.
  */
#define SYS_CLK_MAX                                 82000000UL

/**
  * @brief __EPWM_FADC_ADPLL define Max frequence value.
  */
#define __EPWM_FADC_ADPLL_MAX                       360000000UL

/*********************************************************************************/
// system debug level : normal \ warning \ error \ none
/*********************************************************************************/
#define DL_NONE                             0
#define DL_ERR                              1
#define DL_WARNING                          2
#define DL_ALL                              3

#if (DL_NONE == DEBUG_LEVEL)
    #define DBG_ERR_EN                      0
    #define DBG_WAR_EN                      0
    #define DBG_NOR_EN                      0
#elif (DL_ERR == DEBUG_LEVEL)
    #define DBG_ERR_EN                      1
    #define DBG_WAR_EN                      0
    #define DBG_NOR_EN                      0
#elif (DL_WARNING == DEBUG_LEVEL)
    #define DBG_ERR_EN                      1
    #define DBG_WAR_EN                      1
    #define DBG_NOR_EN                      0
#elif (DL_ALL == DEBUG_LEVEL)
    #define DBG_ERR_EN                      1
    #define DBG_WAR_EN                      1
    #define DBG_NOR_EN                      1
#endif

/* Private macro -------------------------------------------------------------*/
#if (DBG_ERR_EN)
    #define DBG_ERR(fmt, args...)           printf("[err]");       printf(fmt, ##args)
    #define DBG_ERR_BUF(buf, len)           printf_array(buf, len)
#else
    #define DBG_ERR(...)
    #define DBG_ERR_BUF(buf, len)
#endif

#if (DBG_WAR_EN)
    #define DBG_WAR(fmt, args...)           printf("[warning]");   printf(fmt, ##args)
    #define DBG_WAR_BUF(buf, len)           printf_array(buf, len)
#else
    #define DBG_WAR(...)
    #define DBG_WAR_BUF(buf, len)
#endif

#if (DBG_NOR_EN)
    #define DBG_NOR(fmt, args...)           printf(fmt, ##args)
    #define DBG_NOR_BUF(buf, len)           printf_array(buf, len)
#else
    #define DBG_NOR(...)
    #define DBG_NOR_BUF(buf, len)
#endif

/*********************************************************************************/
// define each peripheral debug info
/*********************************************************************************/
#if (XXX_DBG_EN)
    #define XXX_DBG_NOR(fmt, args...)       DBG_NOR(fmt, ##args)
    #define XXX_DBG_NOR_BUF(buf, len)       DBG_NOR_BUF(buf, len)
#else
    #define XXX_DBG_NOR(...)
    #define XXX_DBG_NOR_BUF(buf, len)
#endif


/*********************************************************************************/
/*    firmware update                                                            */
/*********************************************************************************/
/**
  * @brief EFLASH firmware update cmd
  */
typedef enum {
    /*! erase all app code
     */    
    FIRMWARE_UPDATE_CMD_ERASE_APP,
    /*! erase special sectors in the buffer, use for "Incremental upgrade"
     */    
    FIRMWARE_UPDATE_CMD_ERASE_SECTORS,
    /*! program buffer data to special addr
     */    
    FIRMWARE_UPDATE_CMD_PROGRAM,
    /*! verify special addr data with buffer 
     */    
    FIRMWARE_UPDATE_CMD_VERIFY,
    /*! when execute schedule : erase->program->verify is passed, 
     *  use must kick cmd success to register new firmware.
     */
    FIRMWARE_UPDATE_CMD_REGISTER,
    /*! reboot to execute new app code 
     */    
    FIRMWARE_UPDATE_CMD_REBOOT,
} FIRMWARE_UPDATE_CMD;

/** APP_CODE_SIZE
  * this must same with scatter file & makecode.ini
  */
#define APP_CODE_SIZE                           (56 * 1024)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
extern void (* const __vector_handlers[])(void);
     
extern volatile u32 SysTickCnt;

extern __IO uint32_t SYS_CLK;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  system_critical_section_enter function
  * @retval None
  */
void system_critical_section_enter(void);

/**
  * @brief  system_critical_section_exit function
  * @retval None
  */
void system_critical_section_exit(void);

/**
  * @brief  Use the CPU to delay approximately us
  * @param  n : the number of us to delay
  * @retval None
  */
void delay_us(u32 n);

/**
  * @brief  Use the CPU to delay approximately ms
  * @param  n : the number of ms to delay
  * @retval None
  */
void delay_ms(u32 n);

/**
  * @brief  Setup the microcontroller system
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SYS_CLK variable.
  * @retval None
  * @note   This function should be used only after reset.
  */
void system_init(void);

/**
  * @brief  is_systick_expired.
  * @param  offset_ticks: current sys tick 
  * @param  Texpire: offset ticks
  * @retval return true or fault
  */
u32 is_systick_expired(s32 offset_ticks, s32 Texpire);

/**
  * @brief  system_tick_init.
  * @retval return true or fault
  */
u32 system_tick_init(void);

/**
  * @brief  get_system_tick.
  * @retval return the count of current systick
  */
u32 get_system_tick(void);

/**
  * @brief  sys_update_firmware_handle
  * @param  cmd    : FIRMWARE_UPDATE_CMD
  * @param  addr   : address , base 0.
  * @param  len    : length in unit 4Byte
  * @param  p_data : data buffer
  * @retval 1(success), 0(failed)
  */
bool sys_update_firmware_handle(FIRMWARE_UPDATE_CMD cmd, u32 *p_data, u32 addr, u32 len);

/**
  * @brief  dump_fault_state : save vip param & printf cpu fault state
  * @retval None
  * @note : User can save vip param at here, restore vip param at next power-up or reset
  */
void dump_fault_state(void);
    
/**
  * @brief  recover_fault_state : recover vip param
  * @retval None
  * @note : restore vip param at next power-up or reset
  */
void recover_fault_state(void);

/**
  * @brief  save vip param to eflash
  * @param  p_data: param buffer
  * @param  len   : length in unit 4Byte
  * @retval 1(success), 0(failed)
  * @note : User can save vip param at here, restore vip param at next power-up or reset
  */
bool sys_save_data_to_eflash(u32 *p_data, u32 len);

/**
  * @brief  recover vip param from eflash
  * @param  p_data: param buffer
  * @param  len   : length in unit 4Byte
  * @retval 1(success), 0(failed)
  * @note : if the last param is wrong, we will found the latest ok-param instead
  */
bool sys_recover_data_from_eflash(u32 *p_data, u32 len);


#ifdef __cplusplus
}
#endif

#endif //__TX_PHE_SYSTEM_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
