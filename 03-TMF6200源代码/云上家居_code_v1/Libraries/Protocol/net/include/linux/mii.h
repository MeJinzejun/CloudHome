/**
  ******************************************************************************
  * @file    Libraries/Protocol/net/include/linux/mii.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    10-10-2018
  * @brief   definitions for MII-compatible transceivers
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
#ifndef __LINUX_MII_H__
#define __LINUX_MII_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "typedef.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @weakgroup NETWORK
  * @{
  */

/** @defgroup NETWORK_MII MII Register
  * @brief definitions for MII-compatible transceivers
  * @{
  */ 

/** @defgroup MII_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  A small helper function that translates ethtool advertisement
  *         settings to phy autonegotiation advertisements for the
  *         MII_ADVERTISE register.
  * @param  ethadv: the ethtool advertisement settings
  * @retval MII_ADVERTISE register value
  */
__STATIC_INLINE u32 ethtool_adv_to_mii_adv_t(u32 ethadv) {
    u32 result = 0;

    if(ethadv & ADVERTISED_10baseT_Half) {
        result |= ADVERTISE_10HALF;
    }
    if(ethadv & ADVERTISED_10baseT_Full) {
        result |= ADVERTISE_10FULL;
    }
    if(ethadv & ADVERTISED_100baseT_Half) {
        result |= ADVERTISE_100HALF;
    }
    if(ethadv & ADVERTISED_100baseT_Full) {
        result |= ADVERTISE_100FULL;
    }
    if(ethadv & ADVERTISED_Pause) {
        result |= ADVERTISE_PAUSE_CAP;
    }
    if(ethadv & ADVERTISED_Asym_Pause) {
        result |= ADVERTISE_PAUSE_ASYM;
    }

    return result;
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

#ifdef __cplusplus
}
#endif

#endif //__LINUX_MII_H__

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
