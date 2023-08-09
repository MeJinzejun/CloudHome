/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/source/tx_phe_misc.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"

/** @addtogroup TX_PHE_StdPeriph_Driver
  * @{
  */

/** @defgroup MISC 
  * @brief MISC driver modules
  * @{
  */

/** @defgroup MISC_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MISC_Private_Defines
  * @{
  */

#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)
/**
  * @}
  */

/** @defgroup MISC_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup MISC_Private_Functions
  * @{
  */

/**
  * @brief  Configures the priority grouping: pre-emption priority and subpriority.
  * @param  NVIC_PriorityGroup: specifies the priority grouping bits length. 
  *   This parameter can be one of the following values:
  *     @arg NVIC_PriorityGroup_0: 0 bits for pre-emption priority
  *                                3 bits for subpriority
  *     @arg NVIC_PriorityGroup_1: 1 bits for pre-emption priority
  *                                2 bits for subpriority
  *     @arg NVIC_PriorityGroup_2: 2 bits for pre-emption priority
  *                                1 bits for subpriority
  *     @arg NVIC_PriorityGroup_3: 3 bits for pre-emption priority
  *                                0 bits for subpriority
  * @retval None
  */
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup)
{
    /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
    SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup;
}

/**
  * @brief  Initializes the NVIC peripheral according to the specified
  *   parameters in the NVIC_InitStruct.
  * @param  NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure that contains
  *   the configuration information for the specified NVIC peripheral.
  * @retval None
  */
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
    u32 tmppriority = 0x00, tmppre = 0x00, tmpsub = (1<<__NVIC_PRIO_BITS)-1;
  
    if(NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE) {
        /* Compute the Corresponding IRQ Priority --------------------------------*/
        tmppriority = (0x700 - ((SCB->AIRCR) & (u32)0x700))>> 0x08;
        
        if(tmppriority >= __NVIC_PRIO_BITS) {
            tmppriority = (u32)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority
                               << (8-__NVIC_PRIO_BITS);
        } else {
            tmppre = __NVIC_PRIO_BITS - tmppriority;
            tmpsub = tmpsub >> tmppriority;

            tmppriority = (u32)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
            tmppriority |=  NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub;
            tmppriority = tmppriority << (8-__NVIC_PRIO_BITS);
        }

        NVIC->IP[NVIC_InitStruct->NVIC_IRQChannel] = tmppriority;
    
        /* Enable the Selected IRQ Channels --------------------------------------*/
        NVIC->ISER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
                        (u32)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (u8)0x1F);
    } else {
        /* Disable the Selected IRQ Channels -------------------------------------*/
        NVIC->ICER[NVIC_InitStruct->NVIC_IRQChannel >> 0x05] =
                        (u32)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (u8)0x1F);
    }
}

/**
  * @brief  Sets the vector table location and Offset.
  * @param  NVIC_VectTab: specifies if the vector table is in RAM or FLASH memory.
  *   This parameter can be one of the following values:
  *     @arg NVIC_VectTab_RAM
  *     @arg NVIC_VectTab_FLASH
  * @param  Offset: Vector Table base offset field. This value must be a multiple of 0x100.
  * @retval None
  */
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{ 
    SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80);
}

/**
  * @brief  Selects the condition for the system to enter low power mode.
  * @param  LowPowerMode: Specifies the new mode for the system to enter low power mode.
  *   This parameter can be one of the following values:
  *     @arg NVIC_LP_SEVONPEND
  *     @arg NVIC_LP_SLEEPDEEP
  *     @arg NVIC_LP_SLEEPONEXIT
  * @param  NewState: new state of LP condition. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState)
{
    if(NewState != DISABLE) {
        SCB->SCR |= LowPowerMode;
    } else {
        SCB->SCR &= (u32)(~(u32)LowPowerMode);
    }
}

/**
  * @brief  Configures the SysTick clock source.
  * @param  SysTick_CLKSource: specifies the SysTick clock source.
  *   This parameter can be one of the following values:
  *     @arg SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8 selected as SysTick clock source.
  *     @arg SysTick_CLKSource_HCLK: AHB clock selected as SysTick clock source.
  * @retval None
  */
void SysTick_CLKSourceConfig(u32 SysTick_CLKSource)
{
    if(SysTick_CLKSource == SysTick_CLKSource_HCLK) {
        SysTick->CTRL |= SysTick_CLKSource_HCLK;
    } else {
        SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
    }
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

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
