/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/tx_phe_debug.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DEBUG LL firmware functions.
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
#ifndef __TX_PHE_LL_DEBUG_H
#define __TX_PHE_LL_DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "tx_phe_ll_usart.h"
#include <string.h>
#include <stdio.h>
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup debug_interface_gr DEBUG Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup DEBUG_LL_Driver DEBUG LL Driver
  * @ingroup  debug_interface_gr
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup DEBUG_LL_Exported_Constants DEBUG LL Exported Constants
  * @ingroup  DEBUG_LL_Driver
  * @brief    DEBUG LL external constant definition
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

/**
  * @brief Debug interface type selection
  */
typedef enum {
    DEBUG_NO = 0,
    DEBUG_ITM,
    DEBUG_USART0,
    DEBUG_USART1,
    DEBUG_USART2,
} TYPE_ENUM_DEBUG_INTERFACE_SEL;
  
/**
  * @}
  */
  
  
/** @defgroup DEBUG_LL_Inti_Cfg DEBUG LL Initialization And Configuration
  * @brief    DEBUG LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DEBUG data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Configure the debug interface, mainly the ITM and USART modules.
  * @param  debug_interface: Debug interface type selection.
  * @retval None
  */
void debug_select_interface(TYPE_ENUM_DEBUG_INTERFACE_SEL debug_interface);

/**
  * @}
  */
  
/** @defgroup DEBUG_LL_Data_Transfers DEBUG LL Data transfers functions
  * @brief    DEBUG LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the DEBUG data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/** 
  * @brief  printf array.
  * @param  ptr: the address of the data which will send to RS485 TX pin.
  * @param  len: the len of data which will send.
  * @retval None.
  */
void printf_array(void *ptr, u16 len);

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

#endif //__TX_PHE_LL_DEBUG_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
