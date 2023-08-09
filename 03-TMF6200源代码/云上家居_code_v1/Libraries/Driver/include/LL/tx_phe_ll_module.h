/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_module.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   Header file collection of various modules
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
#ifndef __TX_PHE_LL_MODULE_H
#define __TX_PHE_LL_MODULE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "tx_phe_misc.h"
#include "tx_phe_assert.h"
#include "tx_phe_system.h"
#include "tx_phe_debug.h"
     
#include "tx_phe_ll_gpio.h"
#include "tx_phe_ll_usart.h"
#include "tx_phe_ll_epwm.h"
#include "tx_phe_ll_saradc.h"
#include "tx_phe_ll_timer.h"
#include "tx_phe_ll_dftrans.h"
#include "tx_phe_ll_fft.h"
#include "tx_phe_ll_matrix.h"
#include "tx_phe_ll_sincos.h"
#include "tx_phe_ll_spwm.h"
#include "tx_phe_ll_eflash.h"
#include "tx_phe_ll_wdt.h"
#include "tx_phe_ll_fir.h"
#include "tx_phe_ll_rms.h"
#include "tx_phe_ll_datadma.h"
#include "tx_phe_ll_dmac.h"
#include "tx_phe_ll_crc.h"
#include "tx_phe_ll_hcc.h"
#include "tx_phe_ll_arctan.h"
#include "tx_phe_ll_iir.h"
#include "tx_phe_ll_spi.h"
#include "tx_phe_ll_fadc.h"
#include "tx_phe_ll_qei.h"
#include "tx_phe_ll_svpwm.h"
#include "tx_phe_ll_iic.h"
#include "tx_phe_ll_evsys.h"
#include "tx_phe_ll_can.h"
#include "tx_phe_ll_mac.h"
#include "tx_phe_ll_cc_rst.h"
     
//Protocol
#include "mb.h"
     
//net
//#include "net/include/uapi/linux/errno-base.h"
//#include "net/include/uapi/linux/ethtool.h"
//#include "net/include/uapi/linux/mii.h"
//#include "net/include/linux/mii.h"
//#include "net/include/linux/phy.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif //__TX_PHE_LL_MODULE_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
