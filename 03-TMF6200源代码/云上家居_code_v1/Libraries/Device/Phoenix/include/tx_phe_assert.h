/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/tx_phe_assert.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *          This file contains the system clock configuration for TX_PHE devices,
  *          and is generated by the clock configuration tool.
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
#ifndef __TX_PHE_ASSERT_H
#define __TX_PHE_ASSERT_H

#ifdef __cplusplus
 extern "C" {
#endif
    
/** @addtogroup TX_PHE_ASSERT TX_PHE System
  * @{
  */
     
/** @addtogroup assert_interface_gr Assert 
  * @ingroup  TX_PHE_ASSERT
  * @{
  */ 

/** @defgroup ASSERT_Exported_Constants ASSERT Exported Constants
  * @ingroup  assert_interface_gr
  * @brief    ASSERT external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the low 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
    
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#if defined(TXF6200)
#include "txf6200_assert.h"
#elif  defined (TXF5200)
#include "txf5200_assert.h"
#else
#error "phoenix sdk target device error!!!"
#endif


/**
  * @}
  */
  
/** @defgroup ASSERT_Data_Handle ASSERT Data Handle functions
  * @ingroup  assert_interface_gr
  * @brief    ASSERT Data Handle functions 
  *
@verbatim   
  ===============================================================================
                            Data Handle functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the ASSERT data 
    Handle.
  
@endverbatim
  *
  * @{
  */

#if USE_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  p_file: pointer to the source file name
  * @param  line  : assert_param error line source number
  * @retval None
  */
void tx_assert_false(u8 *p_file, u32 line);

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  start_adr   : The starting address of the parameter.
  * @param  len         : The length of the buffer, the minimum unit is calculated as 1 byte.
  * @param  sram_mask   : The SRAM threshold supported by the module.
  * @param  align_mask  : Threshold value for buffer address alignment.
  * @param  p_file      : pointer to the source file name
  * @param  line        : assert_param error line source number
  * @retval None
  */
void tx_assert_address(u32  start_adr, 
                       u32  len, 
                       u32  sram_mask, 
                       u32  align_mask,
                       u8  *p_file, 
                       u32  line);

/**
  * @brief  Assert the effective value of the parameter
  * @param  expr: Enter the expression that asserts the parameter.
  * @retval None
  */
#define TX_ASSERT(expr)                           ((expr) ? (void)0U : tx_assert_false((uint8_t *)__FILE__, __LINE__))

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  start_adr: The starting address of the parameter.
  * @param  len: The length of the buffer, the minimum unit is calculated as 1 byte.
  * @param  sram_mask: The SRAM threshold supported by the module.
  * @param  align_mask: Threshold value for buffer address alignment.
  * @retval None
  */
#define TX_ASSERT_ADDR(start_adr, len, sram_mask, align_mask)    tx_assert_address(start_adr, len, sram_mask, align_mask, \
                                                                                   (uint8_t *)__FILE__, __LINE__)

#else

/**
  * @brief  Assert the effective value of the parameter
  * @param  expr: Enter the expression that asserts the parameter.
  * @retval None
  */
#define TX_ASSERT(expr)                                          ((void)0U)
    
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  start_adr: The starting address of the parameter.
  * @param  len: The length of the buffer, the minimum unit is calculated as 1 byte.
  * @param  sram_mask: The SRAM threshold supported by the module.
  * @param  align_mask: Threshold value for buffer address alignment.
  * @retval None
  */
#define TX_ASSERT_ADDR(start_adr, len, sram_mask, align_mask)    ((void)0U)

#endif /* USE_ASSERT */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __TX_PHE_ASSERT_H */


/************************ (C) COPYRIGHT TXMicroelectronics *****END OF FILE****/

