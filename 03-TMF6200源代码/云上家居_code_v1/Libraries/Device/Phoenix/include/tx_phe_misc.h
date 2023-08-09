/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/tx_phe_misc.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the functions prototypes for the miscellaneous
  *          firmware library functions (add-on to CMSIS functions).
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
#ifndef __TX_PHE_HAL_MISC_H
#define __TX_PHE_HAL_MISC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "core_cm3.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup MISC
  * @{
  */

/** @defgroup MISC_Exported_Types
  * @{
  */

/** 
  * @brief  NVIC Init Structure definition  
  */

typedef struct
{
  uint8_t NVIC_IRQChannel;                    /*!< Specifies the IRQ channel to be enabled or disabled.
                                                   This parameter can be a value of @ref IRQn_Type 
                                                   (For the complete HUGE-IC Devices IRQ Channels list, please
                                                    refer to HUGE-IC.h file) */

  uint8_t NVIC_IRQChannelPreemptionPriority;  /*!< Specifies the pre-emption priority for the IRQ channel
                                                   specified in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  uint8_t NVIC_IRQChannelSubPriority;         /*!< Specifies the subpriority level for the IRQ channel specified
                                                   in NVIC_IRQChannel. This parameter can be a value
                                                   between 0 and 15 as described in the table @ref NVIC_Priority_Table */

  FunctionalState NVIC_IRQChannelCmd;         /*!< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */   
} NVIC_InitTypeDef;
 
/**
  * @}
  */

/** @defgroup NVIC_Priority_Table 
  * @{
  */

/**
@code  
 The table below gives the allowed values of the pre-emption priority and subpriority according
 to the Priority Grouping configuration performed by NVIC_PriorityGroupConfig function
  ============================================================================================================================
    NVIC_PriorityGroup   | NVIC_IRQChannelPreemptionPriority | NVIC_IRQChannelSubPriority  | Description
  ============================================================================================================================
   NVIC_PriorityGroup_0  |                0                  |            0-7              |   0 bits for pre-emption priority
                         |                                   |                             |   3 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------
   NVIC_PriorityGroup_1  |                0-1                |            0-3              |   1 bits for pre-emption priority
                         |                                   |                             |   2 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_2  |                0-3                |            0-1              |   2 bits for pre-emption priority
                         |                                   |                             |   1 bits for subpriority
  ----------------------------------------------------------------------------------------------------------------------------    
   NVIC_PriorityGroup_3  |                0-7                |            0                |   3 bits for pre-emption priority
                         |                                   |                             |   0 bits for subpriority
  ============================================================================================================================
@endcode
*/

/**
  * @}
  */

/** @defgroup MISC_Exported_Constants
  * @{
  */

/** @defgroup Vector_Table_Base 
  * @{
  */

#define NVIC_VectTab_RAM                      ((u32)0x20000000)
#define NVIC_VectTab_FLASH                    ((u32)0x08000000)
#define IS_NVIC_VECTTAB(VECTTAB)              (((VECTTAB) == NVIC_VectTab_RAM) ||   \
                                              ((VECTTAB) == NVIC_VectTab_FLASH))
/**
  * @}
  */

/** @defgroup System_Low_Power 
  * @{
  */

#define NVIC_LP_SEVONPEND                     ((uint8_t)0x10)
#define NVIC_LP_SLEEPDEEP                     ((uint8_t)0x04)
#define NVIC_LP_SLEEPONEXIT                   ((uint8_t)0x02)
#define IS_NVIC_LP(LP)                        (((LP) == NVIC_LP_SEVONPEND) ||       \
                                              ((LP) == NVIC_LP_SLEEPDEEP) ||        \
                                              ((LP) == NVIC_LP_SLEEPONEXIT))
/**
  * @}
  */

/** @defgroup Preemption_Priority_Group 
  * @{
  */
/*! 0 bits for pre-emption priority 3 bits for subpriority
 */
#define NVIC_PriorityGroup_0                   ((u32)0x700)
/*! 1 bits for pre-emption priority 2 bits for subpriority
 */
#define NVIC_PriorityGroup_1                   ((u32)0x600)
/*! 2 bits for pre-emption priority 1 bits for subpriority
 */
#define NVIC_PriorityGroup_2                   ((u32)0x500)
/*! 3 bits for pre-emption priority 0 bits for subpriority
 */
#define NVIC_PriorityGroup_3                   ((u32)0x400)


#define IS_NVIC_PRIORITY_GROUP(GROUP)          (((GROUP) == NVIC_PriorityGroup_0) ||\
                                               ((GROUP) == NVIC_PriorityGroup_1) || \
                                               ((GROUP) == NVIC_PriorityGroup_2) || \
                                               ((GROUP) == NVIC_PriorityGroup_3))

#define IS_NVIC_PREEMPTION_PRIORITY(PRIORITY)  ((PRIORITY) < 0x10)

#define IS_NVIC_SUB_PRIORITY(PRIORITY)         ((PRIORITY) < 0x10)

#define IS_NVIC_OFFSET(OFFSET)                 ((OFFSET) < 0x0007FFFF)

/**
  * @}
  */

/** @defgroup SysTick_clock_source 
  * @{
  */

#define SysTick_CLKSource_HCLK_Div8            ((u32)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK                 ((u32)0x00000004)
#define IS_SYSTICK_CLK_SOURCE(SOURCE)          (((SOURCE) == SysTick_CLKSource_HCLK) || \
                                               ((SOURCE) == SysTick_CLKSource_HCLK_Div8))
/**
  * @}
  */

/**
  * @}
  */

/** @defgroup MISC_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Exported_Functions
  * @{
  */

void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup);
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);
void NVIC_EnableIRQ(IRQn_Type IRQn);
void NVIC_DisableIRQ(IRQn_Type IRQn);
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void NVIC_SystemLPConfig(uint8_t LowPowerMode, FunctionalState NewState);
void SysTick_CLKSourceConfig(u32 SysTick_CLKSource);
u32 SysTick_Config(u32 ticks);

#ifdef __cplusplus
}
#endif

#endif // __TX_PHE_HAL_MISC_H

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2017 HUGE-IC *****END OF FILE****/
