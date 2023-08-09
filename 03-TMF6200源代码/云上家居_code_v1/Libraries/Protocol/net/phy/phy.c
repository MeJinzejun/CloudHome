/**
  ******************************************************************************
  * @file    Libraries/Protocol/net/phy/phy.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    10-10-2018
  * @brief   Framework for configuring and reading PHY devices
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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define ARRAY_SIZE(x)                   (sizeof(x) / sizeof((x)[0]))
#define MAX_NUM_SETTINGS                ARRAY_SIZE(settings)
/* Private variables ---------------------------------------------------------*/
/**
  * @brief A structure for mapping a particular speed and duplex combination
  *        to a particular SUPPORTED and ADVERTISED value
  */
struct __phy_setting {
    int speed;
    int duplex;
    u32 setting;
};

/**
  * @brief A mapping of all SUPPORTED settings to speed/duplex
  */
static const struct __phy_setting settings[] = {
    {
        .speed = 10000,
        .duplex = DUPLEX_FULL,
        .setting = SUPPORTED_10000baseT_Full,
    },
    {
        .speed = SPEED_1000,
        .duplex = DUPLEX_FULL,
        .setting = SUPPORTED_1000baseT_Full,
    },
    {
        .speed = SPEED_1000,
        .duplex = DUPLEX_HALF,
        .setting = SUPPORTED_1000baseT_Half,
    },
    {
        .speed = SPEED_100,
        .duplex = DUPLEX_FULL,
        .setting = SUPPORTED_100baseT_Full,
    },
    {
        .speed = SPEED_100,
        .duplex = DUPLEX_HALF,
        .setting = SUPPORTED_100baseT_Half,
    },
    {
        .speed = SPEED_10,
        .duplex = DUPLEX_FULL,
        .setting = SUPPORTED_10baseT_Full,
    },
    {
        .speed = SPEED_10,
        .duplex = DUPLEX_HALF,
        .setting = SUPPORTED_10baseT_Half,
    },
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup NETWORK
  * @{
  */

/** @defgroup NETWORK_PHY PHY Framework
  * @brief    Framework for finding and configuring PHYs.  
  *           Also contains generic PHY driver
  * @{
  */ 
  
/**
  * @brief  return auto-negotiation status
  * @param  phydev : target phy_device struct
  * @retval auto-negotiation status
  * @note   Reads the status register and returns 0 either if 
  *         auto-negotiation is incomplete, or if there was an error.
  *         Returns BMSR_ANEGCOMPLETE if auto-negotiation is done.
  */
static inline int phy_aneg_done(TYPE_PHY_DEVICE *phydev)
{
    int retval;

    retval = phy_read(phydev, MII_BMSR);

    return (retval < 0) ? retval : (retval & BMSR_ANEGCOMPLETE);
}

/**
  * @brief  find a PHY settings array entry that matches speed & duplex
  * @param  speed  : speed to match
  * @param  duplex : duplex to match
  * @retval the index of that setting
  * @note   Searches the settings array for the setting which matches the
  *         desired speed and duplex, and returns the index of that setting.
  *         Returns the index of the last setting if none of the others
  *         match.
  */
static inline int phy_find_setting(int speed, int duplex)
{
    int idx = 0;

    while(idx < ARRAY_SIZE(settings) &&
          (settings[idx].speed != speed ||
          settings[idx].duplex != duplex)) {
        idx++;
    }
    return idx < MAX_NUM_SETTINGS ? idx : MAX_NUM_SETTINGS - 1;
}

/**
  * @brief  find a PHY setting that matches the requested features mask
  * @param  idx : The first index in settings[] to search
  * @retval the index of the first valid setting
  * @note   Returns the index of the first valid setting less than or equal
  *         to the one pointed to by idx, as determined by the mask in
  *         features. Returns the index of the last setting if nothing else
  *         matches.
  */
static inline int phy_find_valid(int idx, u32 features)
{
    while(idx < MAX_NUM_SETTINGS && !(settings[idx].setting & features)) {
        idx++;
    }

    return idx < MAX_NUM_SETTINGS ? idx : MAX_NUM_SETTINGS - 1;
}

/**
  * @brief  make sure the PHY is set to supported speed and duplex
  * @param  phydev : the target phy_device struct
  * @retval None
  * @note   Make sure the PHY is set to supported speeds and duplexes. Drop
  *         down by one in this order:  
  *         1000/FULL, 1000/HALF, 100/FULL, 100/HALF, 10/FULL, 10/HALF.
  */
static void phy_sanitize_settings(TYPE_PHY_DEVICE *phydev)
{
    u32 features = phydev->supported;
    int idx;

    /* Sanitize settings based on PHY capabilities */
    if ((features & SUPPORTED_Autoneg) == 0) {
        phydev->autoneg = AUTONEG_DISABLE;
    }
    idx = phy_find_valid(phy_find_setting(phydev->speed, phydev->duplex),
                         features);

    phydev->speed = settings[idx].speed;
    phydev->duplex = settings[idx].duplex;
}

/**
  * @brief  start auto-negotiation for this PHY device
  * @param  phydev : the phy_device struct
  * @retval Returning false means that the PHY does not support
  *         auto-negotiation.
  * @note   Sanitizes the settings (if we're not autonegotiating them), and
  *         then calls the driver's config_aneg function. If the PHYCONTROL
  *         Layer is operating, we change the state to reflect the beginning
  *         of Auto-negotiation or forcing.
  */
bool phy_start_aneg(TYPE_PHY_DEVICE *phydev)
{
    int err;

    if (AUTONEG_DISABLE == phydev->autoneg)
        phy_sanitize_settings(phydev);

    err = genphy_config_aneg(phydev);

    if (err < 0)
        return false;

    if (phydev->state != PHY_HALTED) {
        if (AUTONEG_ENABLE == phydev->autoneg) {
            phydev->state = PHY_AN;
//            phydev->link_timeout = PHY_AN_TIMEOUT;
        } else {
            phydev->state = PHY_FORCING;
//            phydev->link_timeout = PHY_FORCE_TIMEOUT;
        }
    }
    return true;
}

/**
  * @brief  start PHY state machine tracking
  * @param  phydev  : the phy_device struct
  * @param  handler : callback function for state change notifications
  * @retval None
  * @note   The PHY infrastructure can run a state machine which tracks
  *         whether the PHY is starting up, negotiating, etc. This function
  *         starts the timer which tracks the state of the PHY. If you want
  *         to be notified when the state changes, pass in the callback
  *         @handler, otherwise, pass NULL. If you want to maintain your own
  *         state machine, do not call this function.
  */
void phy_start_machine(TYPE_PHY_DEVICE *phydev,
                       void (*handler)(TYPE_PHY_PRIVATE *))
{
    phydev->adjust_state = handler;

//    schedule_delayed_work(&phydev->state_queue, HZ);
}

/**
  * @brief  start or restart a PHY device
  * @param  phydev : target phy_device struct
  * @retval None
  * @note   Indicates the attached device's readiness to handle PHY-related
  *         work. Used during startup to start the PHY, and after a call to
  *         phy_stop() to resume operation. Also used to indicate the MDIO
  *         bus has cleared an error condition.
  */
void phy_start(TYPE_PHY_DEVICE *phydev)
{
    switch(phydev->state) {
        case PHY_STARTING:
            phydev->state = PHY_PENDING;
            break;
        
        case PHY_READY:
            phydev->state = PHY_UP;
            break;
        
        case PHY_HALTED:
            phydev->state = PHY_RESUMING;
        
        default:
            break;
    }
}

/**
  * @brief  Handle the state machine
  * @param  phydev : target phy_device struct
  * @retval None
  */
void phy_state_machine(TYPE_PHY_DEVICE *phydev)
{
    int needs_aneg = 0;
    int err = 0;
    
    if(phydev->adjust_state) {
        phydev->adjust_state(phydev->priv);
    }
    switch(phydev->state) {
        case PHY_DOWN:
        case PHY_STARTING:
        case PHY_READY:
        case PHY_PENDING:
            break;
        
        case PHY_UP:
            needs_aneg = 1;
//            phydev->link_timeout = PHY_AN_TIMEOUT;
            break;
        
        case PHY_AN:
            err = genphy_read_status(phydev);

            if(err < 0) {
                break;
            }
            /* If the link is down, give up on
             * negotiation for now */
            if(!phydev->link) {
                phydev->state = PHY_NOLINK;
//                netif_carrier_off(phydev->priv);
                phydev->adjust_link(phydev->priv);
                break;
            }

            /* Check if negotiation is done.  Break
             * if there's an error */
            err = phy_aneg_done(phydev);
            if(err < 0) {
                break;
            }
            /* If AN is done, we're running */
            if(err > 0) {
                phydev->state = PHY_RUNNING;
//                netif_carrier_on(phydev->priv);
                phydev->adjust_link(phydev->priv);

            } else if(0 == phydev->link_timeout--) {
                needs_aneg = 1;
                /* If we have the magic_aneg bit,
                 * we try again */
//                if (phydev->drv->flags & PHY_HAS_MAGICANEG)
//                    break;
            }
            break;
            
        case PHY_NOLINK:
            err = genphy_read_status(phydev);

            if(err) {
                break;
            }
            if(phydev->link) {
                phydev->state = PHY_RUNNING;
//                netif_carrier_on(phydev->priv);
                phydev->adjust_link(phydev->priv);
            }
            break;
            
        case PHY_FORCING:
            err = genphy_update_link(phydev);

            if(err) {
                break;
            }
            if(phydev->link) {
                phydev->state = PHY_RUNNING;
//                netif_carrier_on(phydev->priv);
            } else {
                if (0 == phydev->link_timeout--)
                    needs_aneg = 1;
            }

            phydev->adjust_link(phydev->priv);
            break;
            
        case PHY_RUNNING:
            /* Only register a CHANGE if we are
             * polling */
            if(PHY_POLL == phydev->irq) {
                phydev->state = PHY_CHANGELINK;
            }
            break;
            
        case PHY_CHANGELINK:
            err = genphy_read_status(phydev);

            if(err) {
                break;
            }
            if(phydev->link) {
                phydev->state = PHY_RUNNING;
//                netif_carrier_on(phydev->priv);
            } else {
                phydev->state = PHY_NOLINK;
//                netif_carrier_off(phydev->priv);
            }
            phydev->adjust_link(phydev->priv);
//            if (PHY_POLL != phydev->irq)
//                err = phy_config_interrupt(phydev,
//                        PHY_INTERRUPT_ENABLED);
            break;
            
        case PHY_HALTED:
            if(phydev->link) {
                phydev->link = 0;
//                netif_carrier_off(phydev->priv);
                phydev->adjust_link(phydev->priv);
            }
            break;
            
        case PHY_RESUMING:
//            err = phy_clear_interrupt(phydev);
//            if (err)
//                break;

//            err = phy_config_interrupt(phydev,
//                    PHY_INTERRUPT_ENABLED);

//            if (err)
//                break;

//            if (AUTONEG_ENABLE == phydev->autoneg) {
//                err = phy_aneg_done(phydev);
//                if (err < 0)
//                    break;

//                /* err > 0 if AN is done.
//                 * Otherwise, it's 0, and we're
//                 * still waiting for AN */
//                if (err > 0) {
//                    err = phy_read_status(phydev);
//                    if (err)
//                        break;

//                    if (phydev->link) {
//                        phydev->state = PHY_RUNNING;
//                        netif_carrier_on(phydev->priv);
//                    } else
//                        phydev->state = PHY_NOLINK;
//                    phydev->adjust_link(phydev->priv);
//                } else {
//                    phydev->state = PHY_AN;
//                    phydev->link_timeout = PHY_AN_TIMEOUT;
//                }
//            } else {
//                err = phy_read_status(phydev);
//                if (err)
//                    break;

//                if (phydev->link) {
//                    phydev->state = PHY_RUNNING;
//                    netif_carrier_on(phydev->priv);
//                } else
//                    phydev->state = PHY_NOLINK;
//                phydev->adjust_link(phydev->priv);
//            }
            break;
            
            default:
                break;
    }

    if(needs_aneg) {
        err = phy_start_aneg(phydev);
    }
//    if (err < 0)
//        phy_error(phydev);

//    schedule_delayed_work(&phydev->state_queue, PHY_STATE_TIME * HZ);
}

/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
