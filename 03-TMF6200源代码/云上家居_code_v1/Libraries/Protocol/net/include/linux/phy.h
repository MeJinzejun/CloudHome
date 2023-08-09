/**
  ******************************************************************************
  * @file    Libraries/Protocol/net/include/linux/phy.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    10-10-2018
  * @brief   Framework and drivers for configuring and reading different PHYs
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
#ifndef __PHY_H__
#define __PHY_H__

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

/** @weakgroup NETWORK_PHY
  * @{
  */

/** @defgroup NETWORK_PHY_Exported_Constants Exported Constants
  * @{
  */

/*
 * Set phydev->irq to PHY_POLL if interrupts are not supported,
 * or not desired for this PHY.  Set to PHY_IGNORE_INTERRUPT if
 * the attached driver handles the interrupt
 */
#define PHY_POLL                                -1
#define PHY_IGNORE_INTERRUPT                    -2

#define PHY_HAS_INTERRUPT                       0x00000001
#define PHY_HAS_MAGICANEG                       0x00000002

#define PHY_INIT_TIMEOUT                        100000
#define PHY_STATE_TIME                          1
#define PHY_FORCE_TIMEOUT                       10
#define PHY_AN_TIMEOUT                          10

#define PHY_MAX_ADDR                            32

/**
  * @}
  */

/** @defgroup NETWORK_PHY_Exported_Typedefs Exported Typedefs
  * @{
  */

/**
  *@brief Interface Mode definitions
  */
typedef enum {
    PHY_INTERFACE_MODE_NA,
    PHY_INTERFACE_MODE_MII,
    PHY_INTERFACE_MODE_GMII,
    PHY_INTERFACE_MODE_SGMII,
    PHY_INTERFACE_MODE_TBI,
    PHY_INTERFACE_MODE_RMII,
    PHY_INTERFACE_MODE_RGMII,
    PHY_INTERFACE_MODE_RGMII_ID,
    PHY_INTERFACE_MODE_RGMII_RXID,
    PHY_INTERFACE_MODE_RGMII_TXID,
    PHY_INTERFACE_MODE_RTBI,
    PHY_INTERFACE_MODE_SMII,
} TYPE_ENUM_PHY_INTERFACE;

/*
 * The Bus class for PHYs.  Devices which provide access to
 * PHYs should register using this structure
 */
typedef struct __mii_bus {
    GMAC_TypeDef *p_gmac;

    u16 (*read)(GMAC_TypeDef *p_gmac, u8 phy_id, u8 regnum);
    void (*write)(GMAC_TypeDef *p_gmac, u8 phy_id, u8 regnum, u16 val);
    int (*reset)(struct __mii_bus *bus);

    enum {
        MDIOBUS_ALLOCATED = 1,
        MDIOBUS_REGISTERED,
        MDIOBUS_UNREGISTERED,
        MDIOBUS_RELEASED,
    } state;

    /*
     * Pointer to an array of interrupts, each PHY's
     * interrupt at the index matching its address
     */
    int *irq;
} TYPE_MII_BUS;

/* PHY state machine states:
 *
 * DOWN: PHY device and driver are not ready for anything.  probe
 * should be called if and only if the PHY is in this state,
 * given that the PHY device exists.
 * - PHY driver probe function will, depending on the PHY, set
 * the state to STARTING or READY
 *
 * STARTING:  PHY device is coming up, and the ethernet driver is
 * not ready.  PHY drivers may set this in the probe function.
 * If they do, they are responsible for making sure the state is
 * eventually set to indicate whether the PHY is UP or READY,
 * depending on the state when the PHY is done starting up.
 * - PHY driver will set the state to READY
 * - start will set the state to PENDING
 *
 * READY: PHY is ready to send and receive packets, but the
 * controller is not.  By default, PHYs which do not implement
 * probe will be set to this state by phy_probe().  If the PHY
 * driver knows the PHY is ready, and the PHY state is STARTING,
 * then it sets this STATE.
 * - start will set the state to UP
 *
 * PENDING: PHY device is coming up, but the ethernet driver is
 * ready.  phy_start will set this state if the PHY state is
 * STARTING.
 * - PHY driver will set the state to UP when the PHY is ready
 *
 * UP: The PHY and attached device are ready to do work.
 * Interrupts should be started here.
 * - timer moves to AN
 *
 * AN: The PHY is currently negotiating the link state.  Link is
 * therefore down for now.  phy_timer will set this state when it
 * detects the state is UP.  config_aneg will set this state
 * whenever called with phydev->autoneg set to AUTONEG_ENABLE.
 * - If autonegotiation finishes, but there's no link, it sets
 *   the state to NOLINK.
 * - If aneg finishes with link, it sets the state to RUNNING,
 *   and calls adjust_link
 * - If autonegotiation did not finish after an arbitrary amount
 *   of time, autonegotiation should be tried again if the PHY
 *   supports "magic" autonegotiation (back to AN)
 * - If it didn't finish, and no magic_aneg, move to FORCING.
 *
 * NOLINK: PHY is up, but not currently plugged in.
 * - If the timer notes that the link comes back, we move to RUNNING
 * - config_aneg moves to AN
 * - phy_stop moves to HALTED
 *
 * FORCING: PHY is being configured with forced settings
 * - if link is up, move to RUNNING
 * - If link is down, we drop to the next highest setting, and
 *   retry (FORCING) after a timeout
 * - phy_stop moves to HALTED
 *
 * RUNNING: PHY is currently up, running, and possibly sending
 * and/or receiving packets
 * - timer will set CHANGELINK if we're polling (this ensures the
 *   link state is polled every other cycle of this state machine,
 *   which makes it every other second)
 * - irq will set CHANGELINK
 * - config_aneg will set AN
 * - phy_stop moves to HALTED
 *
 * CHANGELINK: PHY experienced a change in link state
 * - timer moves to RUNNING if link
 * - timer moves to NOLINK if the link is down
 * - phy_stop moves to HALTED
 *
 * HALTED: PHY is up, but no polling or interrupts are done. Or
 * PHY is in an error state.
 *
 * - phy_start moves to RESUMING
 *
 * RESUMING: PHY was halted, but now wants to run again.
 * - If we are forcing, or aneg is done, timer moves to RUNNING
 * - If aneg is not done, timer moves to AN
 * - phy_stop moves to HALTED
 */
typedef enum {
    PHY_DOWN            = 0,
    PHY_STARTING,
    PHY_READY,
    PHY_PENDING,
    PHY_UP,
    PHY_AN,
    PHY_RUNNING,
    PHY_NOLINK,
    PHY_FORCING,
    PHY_CHANGELINK,
    PHY_HALTED,
    PHY_RESUMING
} TYPE_ENUM_PHY_STATE;

typedef struct __type_phy_private {
    int               speed;
    int               duplex;
    /* The most recently read link state */
    int               link;
    
    struct phy_device *p_phydev;
} TYPE_PHY_PRIVATE;

/* phy_device: An instance of a PHY
 *
 * bus: Pointer to the bus this PHY is on
 * phy_id: UID for this device found during discovery
 * state: state of the PHY for management purposes
 * dev_flags: Device-specific flags used by the PHY driver.
 * addr: Bus address of PHY
 * link_timeout: The number of timer firings to wait before the
 * giving up on the current attempt at acquiring a link
 * irq: IRQ number of the PHY's interrupt (-1 if none)
 * adjust_link: Callback for the enet controller to respond to
 * changes in the link state.
 * adjust_state: Callback for the enet driver to respond to
 * changes in the state machine.
 *
 * speed, duplex, pause, supported, advertising, and
 * autoneg are used like in mii_if_info
 *
 * interrupts currently only supports enabled or disabled,
 * but could be changed in the future to support enabling
 * and disabling specific interrupts
 *
 * Contains some infrastructure for polling and interrupt
 * handling, as well as handling shifts in PHY hardware state
 */
typedef struct phy_device {
    TYPE_MII_BUS           *bus;
    u32                     phy_id;
    TYPE_ENUM_PHY_STATE     state;
    u32                     dev_flags;
    TYPE_ENUM_PHY_INTERFACE interface;

    /* Bus address of the PHY (0-31) */
    int                     addr;

    /*
     * forced speed & duplex (no autoneg)
     * partner speed & duplex & pause (autoneg)
     */
    int                     speed;
    int                     duplex;
    int                     pause;
    int                     asym_pause;

    /* The most recently read link state */
    int                     link;
    
    /* Union of PHY and Attached devices' supported modes */
    /* See mii.h for more info */
    u32                     supported;
    u32                     advertising;

    int                     autoneg;

    int                     link_timeout;

    /*
     * Interrupt number for this PHY
     * -1 means no interrupt
     */
    int                     irq;

    /* private data pointer */
    /* For use by PHYs to maintain extra state */
    TYPE_PHY_PRIVATE        *priv;

    void (*adjust_link)(TYPE_PHY_PRIVATE *priv);

    void (*adjust_state)(TYPE_PHY_PRIVATE *priv);
} TYPE_PHY_DEVICE;

/**
  * @}
  */

/** @defgroup NETWORK_PHY_Exported_Functions Exported Functions
  * @{
  */

/**
  * @brief  Convenience function for reading a given PHY register
  * @param  phydev : the phy_device struct
  * @param  reg    : register number to read
  * @retval Returns the data in the PHY register
  * @note   MUST NOT be called from interrupt context, because the bus
  *         read/write functions may wait for an interrupt to conclude the
  *         operation.
  */
static inline int phy_read(TYPE_PHY_DEVICE *phy, int reg) {
    return phy->bus->read(phy->bus->p_gmac, phy->addr, reg); 
}

/**
  * @brief  Convenience function for writing a given PHY register
  * @param  phydev : the phy_device struct
  * @param  reg    : register number to write
  * @param  val    : value to write to @reg
  * @retval None
  * @note    MUST NOT be called from interrupt context, because the bus
  *          read/write functions may wait for an interrupt to conclude the
  *          operation.
  */
static inline void phy_write(TYPE_PHY_DEVICE *phy, int reg, int val) {
    phy->bus->write(phy->bus->p_gmac, phy->addr, reg, val);
}

void phy_device_create(TYPE_PHY_DEVICE *dev, TYPE_MII_BUS *bus, int addr, int phy_id);
bool get_phy_device(TYPE_PHY_DEVICE *dev, TYPE_MII_BUS *bus, int addr);
bool phy_connect_direct(TYPE_PHY_PRIVATE *priv, TYPE_PHY_DEVICE *phydev,
                        void (*handler)(TYPE_PHY_PRIVATE *),
                        TYPE_ENUM_PHY_INTERFACE interface);
void phy_start(TYPE_PHY_DEVICE *phydev);
bool phy_start_aneg(TYPE_PHY_DEVICE *phydev);

int genphy_restart_aneg(TYPE_PHY_DEVICE *phydev);
int genphy_config_aneg(TYPE_PHY_DEVICE *phydev);
int genphy_update_link(TYPE_PHY_DEVICE *phydev);
int genphy_read_status(TYPE_PHY_DEVICE *phydev);
void phy_state_machine(TYPE_PHY_DEVICE *phydev);
void phy_start_machine(TYPE_PHY_DEVICE *phydev,
                       void (*handler)(TYPE_PHY_PRIVATE *));
/**
  * @}
  */ 

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif //__PHY_H__

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
