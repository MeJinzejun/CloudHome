/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/tx_phe.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-26-2018
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup PHOENIX
  * @{
  */

#ifndef __PHOENIX_H
#define __PHOENIX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "typedef.h"
#include "config.h"


#if defined(TXF6200)
#include "txf6200.h"
#elif  defined (TXF5200)
#include "txf5200.h"
#else
#error "phoenix sdk target device error!!!"
#endif


#ifdef __cplusplus
}
#endif

#endif /* __PHOENIX_H */

/**
  * @}
  */

  /**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
