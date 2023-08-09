/**
  ******************************************************************************
  * @file    Libraries/Protocol/phy/phy_device.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    10-10-2018
  * @brief   Framework for finding and configuring PHYs.  
  *          Also contains generic PHY driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @addtogroup NETWORK_PHY
  * @{
  */ 

/**
  * @brief  Initialize the phy_device structure.
  * @param  dev    : the phy_device struct
  * @param  bus    : the target MII bus
  * @param  addr   : PHY address on the MII bus
  * @param  phy_id : where to store the ID retrieved
  * @retval None
  */
void phy_device_create(TYPE_PHY_DEVICE *dev, TYPE_MII_BUS *bus, int addr, int phy_id)
{
//    dev->dev.release = phy_device_release;

    dev->speed     = 0;
    dev->duplex    = -1;
    dev->pause     = dev->asym_pause = 0;
    dev->link      = 1;
    dev->interface = PHY_INTERFACE_MODE_GMII;

    dev->autoneg   = AUTONEG_ENABLE;

    dev->addr      = addr;
    dev->phy_id    = phy_id;

    dev->bus       = bus;
//    dev->dev.parent = bus->parent;
//    dev->dev.bus = &mdio_bus_type;
//    dev->irq = bus->irq != NULL ? bus->irq[addr] : PHY_POLL;

    dev->state     = PHY_DOWN;

//    INIT_DELAYED_WORK(&dev->state_queue, phy_state_machine);

//    device_initialize(&dev->dev);
}

/**
  * @brief  reads the specified addr for its ID.
  * @param  bus    : the target MII bus
  * @param  addr   : PHY address on the MII bus
  * @param  phy_id : where to store the ID retrieved
  * @retval Returning 0 means success and returning other values means
  *         failure.
  * @note   In the case of a 802.3-c22 PHY, reads the ID registers of the
  *         PHY at @addr on the @bus, stores it in @phy_id and returns zero
  *         on success.
  */
static int get_phy_id(TYPE_MII_BUS *bus, int addr, u32 *phy_id)
{
    int phy_reg;

    /* Grab the bits from PHYIR1, and put them
     * in the upper half */
    phy_reg = bus->read(bus->p_gmac, addr, MII_PHYSID1);

    if(phy_reg < 0) {
        return -EIO;
    }
    *phy_id = (phy_reg & 0xffff) << 16;

    /* Grab the bits from PHYIR2, and put them in the lower half */
    phy_reg = bus->read(bus->p_gmac, addr, MII_PHYSID2);

    if(phy_reg < 0) {
        return -EIO;
    }
    *phy_id |= (phy_reg & 0xffff);

    return 0;
}

/**
  * @brief  reads the specified PHY device
  * @param  dev  : the phy_device struct
  * @param  bus  : the target MII bus
  * @param  addr : PHY address on the MII bus
  * @retval Return true to successfully initialize phy_device.
  */
bool get_phy_device(TYPE_PHY_DEVICE *dev, TYPE_MII_BUS *bus, int addr)
{
    u32 phy_id = 0;

    if(get_phy_id(bus, addr, &phy_id)) {
        return false;
    }

    /* If the phy_id is mostly Fs, there is no device there */
    if((phy_id & 0x1fffffff) == 0x1fffffff) {
        return false;
    }

    phy_device_create(dev, bus, addr, phy_id);
    
    return true;
}

/**
  * @brief  prepares the PHY layer to monitor link status
  * @param  phydev  : target phy_device struct
  * @param  handler : callback function for link status change notifications
  * @retval None
  * @note   Tells the PHY infrastructure to handle the gory details on
  *         monitoring link status (whether through polling or an interrupt),
  *         and to call back to the connected device driver when the link
  *         status changes. If you want to monitor your own link state,
  *         don't call this function.
  */
static void phy_prepare_link(TYPE_PHY_DEVICE *phydev,
                             void (*handler)(TYPE_PHY_PRIVATE *))
{
    phydev->adjust_link = handler;
}

static int genphy_config_init(TYPE_PHY_DEVICE *phydev)
{
    int val;
    u32 features;

    /* For now, I'll claim that the generic driver supports
     * all possible port types */
    features = (SUPPORTED_TP | SUPPORTED_MII | SUPPORTED_AUI |
               SUPPORTED_FIBRE | SUPPORTED_BNC);

    /* Do we support autonegotiation? */
    val = phy_read(phydev, MII_BMSR);

    if(val < 0) {
        return val;
    }

    if(val & BMSR_ANEGCAPABLE) {
        features |= SUPPORTED_Autoneg;
    }

    if(val & BMSR_100FULL) {
        features |= SUPPORTED_100baseT_Full;
    }
    if(val & BMSR_100HALF) {
        features |= SUPPORTED_100baseT_Half;
    }
    if(val & BMSR_10FULL) {
        features |= SUPPORTED_10baseT_Full;
    }
    if(val & BMSR_10HALF) {
        features |= SUPPORTED_10baseT_Half;
    }
    if(val & BMSR_ESTATEN) {
        val = phy_read(phydev, MII_ESTATUS);

        if(val < 0) {
            return val;
        }
        if(val & ESTATUS_1000_TFULL) {
            features |= SUPPORTED_1000baseT_Full;
        }
        if(val & ESTATUS_1000_THALF) {
            features |= SUPPORTED_1000baseT_Half;
        }
    }

    phydev->supported = features;
    phydev->advertising = features;

    return 0;
}

/**
  * @brief  attach a network device to a given PHY device pointer
  * @param  priv      : PHY private structure pointer.
  * @param  phydev    : Pointer to phy_device to attach
  * @param  flags     : PHY device's dev_flags
  * @param  interface : PHY device's interface
  * @retval always return true
  * @note   Called by drivers to attach to a particular PHY device. The
  *         phy_device is found, and properly hooked up to the phy_driver.
  *         If no driver is attached, then the genphy_driver is used. The
  *         phy_device is given a ptr to the attaching device, and given a
  *         callback for link status change. The phy_device is returned to
  *         the attaching driver.
  */
static bool phy_attach_direct(TYPE_PHY_PRIVATE *priv, TYPE_PHY_DEVICE *phydev,
                              u32 flags, TYPE_ENUM_PHY_INTERFACE interface)
{
    phydev->priv      = priv;
    priv->p_phydev    = phydev;

    phydev->dev_flags = flags;

    phydev->interface = interface;

    phydev->state     = PHY_READY;

    /* Do initial configuration here, now that
     * we have certain key parameters
     * (dev_flags and interface) */
    genphy_config_init(phydev);

    return true;
}

/**
  * @brief  connect an ethernet device to a specific phy_device
  * @param  priv      : PHY private structure pointer.
  * @param  phydev    : the pointer to the phy device
  * @param  handler   : callback function for state change notifications
  * @param  interface : PHY device's interface
  * @retval Returns true if the PHY connection is successful
  */
bool phy_connect_direct(TYPE_PHY_PRIVATE *priv, TYPE_PHY_DEVICE *phydev,
                        void (*handler)(TYPE_PHY_PRIVATE *),
                        TYPE_ENUM_PHY_INTERFACE interface)
{
    bool flag;

    flag = phy_attach_direct(priv, phydev, phydev->dev_flags, interface);
    if(flag == false) {
        return false;
    }
    phy_prepare_link(phydev, handler);
    phy_start_machine(phydev, NULL);
//    if (phydev->irq > 0)
//        phy_start_interrupts(phydev);

    return true;
}

/**
  * @brief  sanitize and advertise auto-negotiation parameters
  * @param  phydev : target phy_device struct
  * @retval Returns < 0 on error, 0 if the PHY's advertisement hasn't
  *         changed, and > 0 if it has changed.
  * @note   Writes MII_ADVERTISE with the appropriate values, after
  *         sanitizing the values to make sure we only advertise what is
  *         supported.  
  */
static int genphy_config_advert(TYPE_PHY_DEVICE *phydev)
{
    u32 advertise;
    int oldadv, adv;
    int changed = 0;

    /* Only allow advertising what
     * this PHY supports */
    phydev->advertising &= phydev->supported;
    advertise            = phydev->advertising;

    /* Setup standard advertisement */
    oldadv = adv = phy_read(phydev, MII_ADVERTISE);

    if(adv < 0) {
        return adv;
    }
    adv &= ~(ADVERTISE_ALL | ADVERTISE_100BASE4 | ADVERTISE_PAUSE_CAP |
             ADVERTISE_PAUSE_ASYM);
    adv |= ethtool_adv_to_mii_adv_t(advertise);

    if(adv != oldadv) {
        phy_write(phydev, MII_ADVERTISE, adv);
        changed = 1;
    }

    /* Configure gigabit if it's supported */
//    if (phydev->supported & (SUPPORTED_1000baseT_Half |
//                SUPPORTED_1000baseT_Full)) {
//        oldadv = adv = phy_read(phydev, MII_CTRL1000);

//        if (adv < 0)
//            return adv;

//        adv &= ~(ADVERTISE_1000FULL | ADVERTISE_1000HALF);
//        adv |= ethtool_adv_to_mii_ctrl1000_t(advertise);

//        if (adv != oldadv) {
//            err = phy_write(phydev, MII_CTRL1000, adv);

//            if (err < 0)
//                return err;
//            changed = 1;
//        }
//    }

    return changed;
}

/**
  * @brief  configures/forces speed/duplex from @phydev
  * @param  phydev : target phy_device struct
  * @retval None
  * @note   Configures MII_BMCR to force speed/duplex to the values in
  *         phydev. Assumes that the values are valid. Please see
  *         phy_sanitize_settings().
  */
static void genphy_setup_forced(TYPE_PHY_DEVICE *phydev)
{
    int ctl = 0;

    phydev->pause = phydev->asym_pause = 0;

    if(SPEED_1000 == phydev->speed) {
        ctl |= BMCR_SPEED1000;
    } else if(SPEED_100 == phydev->speed) {
        ctl |= BMCR_SPEED100;
    }
    if(DUPLEX_FULL == phydev->duplex) {
        ctl |= BMCR_FULLDPLX;
    }
    phy_write(phydev, MII_BMCR, ctl);
}

/**
  * @brief  Enable and Restart Autonegotiation
  * @param  phydev : target phy_device struct
  * @retval Returning 0 means the PHY supports auto-negotiation.
  */
int genphy_restart_aneg(TYPE_PHY_DEVICE *phydev)
{
    int ctl;

    ctl = phy_read(phydev, MII_BMCR);

    if(ctl < 0) {
        return ctl;
    }
    ctl |= (BMCR_ANENABLE | BMCR_ANRESTART);

    /* Don't isolate the PHY if we're negotiating */
    ctl &= ~(BMCR_ISOLATE);

    phy_write(phydev, MII_BMCR, ctl);

    return 0;
}

/**
  * @brief  restart auto-negotiation or write BMCR
  * @param  phydev : target phy_device struct
  * @retval Returns the auto-negotiation configuration result.
  * @note   If auto-negotiation is enabled, we configure the advertising,
  *         and then restart auto-negotiation. If it is not enabled, then we
  *         write the BMCR
  */
int genphy_config_aneg(TYPE_PHY_DEVICE *phydev)
{
    int result;

    if(AUTONEG_ENABLE != phydev->autoneg) {
        genphy_setup_forced(phydev);
        return 0;
    }

    result = genphy_config_advert(phydev);

    if(result < 0) {    /* error */
        return result;
    }

    if(result == 0) {
        /* Advertisement hasn't changed, but maybe aneg was never on to
         * begin with?  Or maybe phy was isolated? */
        int ctl = phy_read(phydev, MII_BMCR);

        if(ctl < 0) {
            return ctl;
        }
        if(!(ctl & BMCR_ANENABLE) || (ctl & BMCR_ISOLATE)) {
            result = 1; /* do restart aneg */
        }
    }

    /* Only restart aneg if we are advertising something different
     * than we were before.     */
    if(result > 0) {
        result = genphy_restart_aneg(phydev);
    }
    return result;
}

/**
  * @brief  update link status in @phydev
  * @param  phydev : target phy_device struct
  * @retval Returns the link status of the PHY.
  * @note   Update the value in phydev->link to reflect the current link
  *         value. In order to do this, we need to read the status register
  *         twice, keeping the second value.
  */
int genphy_update_link(TYPE_PHY_DEVICE *phydev)
{
    int status;

    /* Do a fake read */
    status = phy_read(phydev, MII_BMSR);

    if(status < 0) {
        return status;
    }
    /* Read link and autonegotiation status */
    status = phy_read(phydev, MII_BMSR);

    if(status < 0) {
        return status;
    }
    if((status & BMSR_LSTATUS) == 0) {
        phydev->link = 0;
    } else {
        phydev->link = 1;
    }
    return 0;
}

/**
  * @brief  check the link status and update current link state
  * @param  phydev : target phy_device struct
  * @retval Returns 0 when the PHY has no errors.
  * @note   Check the link, then figure out the current state by comparing
  *         what we advertise with what the link partner advertises. Start
  *         by checking the gigabit possibilities, then move on to 10/100.
  */
int genphy_read_status(TYPE_PHY_DEVICE *phydev)
{
    int adv;
    int err;
    int lpa;
    int lpagb = 0;

    /* Update the link, but return if there
     * was an error */
    err = genphy_update_link(phydev);
    if(err) {
        return err;
    }
    if(AUTONEG_ENABLE == phydev->autoneg) {
        if(phydev->supported & (SUPPORTED_1000baseT_Half |
                                SUPPORTED_1000baseT_Full)) {
            lpagb = phy_read(phydev, MII_STAT1000);

            if(lpagb < 0) {
                return lpagb;
            }
            adv = phy_read(phydev, MII_CTRL1000);

            if(adv < 0) {
                return adv;
            }
            lpagb &= adv << 2;
        }

        lpa = phy_read(phydev, MII_LPA);

        if(lpa < 0) {
            return lpa;
        }
        adv = phy_read(phydev, MII_ADVERTISE);

        if(adv < 0) {
            return adv;
        }
        lpa &= adv;

        phydev->speed = SPEED_10;
        phydev->duplex = DUPLEX_HALF;
        phydev->pause = phydev->asym_pause = 0;

        if(lpagb & (LPA_1000FULL | LPA_1000HALF)) {
            phydev->speed = SPEED_1000;
            
            if(lpagb & LPA_1000FULL) {
                phydev->duplex = DUPLEX_FULL;
            }
        } else if(lpa & (LPA_100FULL | LPA_100HALF)) {
            phydev->speed = SPEED_100;
            
            if(lpa & LPA_100FULL) {
                phydev->duplex = DUPLEX_FULL;
            }
        } else {
            if(lpa & LPA_10FULL) {
                phydev->duplex = DUPLEX_FULL;
            }
        }
        if(phydev->duplex == DUPLEX_FULL){
            phydev->pause = lpa & LPA_PAUSE_CAP ? 1 : 0;
            phydev->asym_pause = lpa & LPA_PAUSE_ASYM ? 1 : 0;
        }
    } else {
        int bmcr = phy_read(phydev, MII_BMCR);
        if(bmcr < 0) {
            return bmcr;
        }
        if(bmcr & BMCR_FULLDPLX) {
            phydev->duplex = DUPLEX_FULL;
        } else {
            phydev->duplex = DUPLEX_HALF;
        }
        if(bmcr & BMCR_SPEED1000) {
            phydev->speed = SPEED_1000;
        } else if(bmcr & BMCR_SPEED100) {
            phydev->speed = SPEED_100;
        } else {
            phydev->speed = SPEED_10;
        }
        phydev->pause = phydev->asym_pause = 0;
    }

    return 0;
}

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
