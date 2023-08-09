/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_mac.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    08-10-2018
  * @brief   This file contains all the MAC LL firmware functions.
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
#include "tx_phe_ll_mac.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup mac_interface_gr MAC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup MAC_LL_Driver MAC LL Driver
  * @ingroup  mac_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*! GMAC TX Descriptor Management Pointer
 */
static TYPE_LL_MAC_TX_DESCRIPTOR *__ll_mac_p_tx_descriptor;
/*! GMAC TX Descriptor Management Pointer
 */
static TYPE_LL_MAC_RX_DESCRIPTOR *__ll_mac_p_rx_descriptor;
/*! GMAC setup frame tx descriptor Pointer
 */
static TYPE_LL_MAC_TX_DESCRIPTOR *__ll_mac_p_setup_tx_descriptor;
/*! TX descriptor backup
 */
static TYPE_LL_MAC_TX_DESCRIPTOR __ll_mac_tx_descriptor_backup;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup MAC_LL_Interrupt MAC LL Interrupt Handle function
  * @ingroup  MAC_LL_Driver
  * @brief   MAC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup MAC_LL_Inti_Cfg MAC LL Initialization And Configuration
  * @ingroup  MAC_LL_Driver
  * @brief    MAC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  GMAC module initialization
  * @param  p_gmac: GMAC module pointer
  * @param  p_init: GMAC initialization structure pointer
  * @retval None
  */
void ll_mac_init(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_INIT *p_init)
{
    TYPE_ENUM_LL_MAC_CLKDIV_SEL clk_div = (TYPE_ENUM_LL_MAC_CLKDIV_SEL)0;
    
    TX_ASSERT(p_gmac == GMAC);
    
    TX_ASSERT_ADDR(p_init->tx_descriptor_start_addr, sizeof(TYPE_LL_MAC_TX_DESCRIPTOR),
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    TX_ASSERT_ADDR(p_init->rx_descriptor_start_addr, sizeof(TYPE_LL_MAC_RX_DESCRIPTOR),
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    
    /* GMAC clock selection */
    SYSCTRL_REG_OPT(SYSCTRL->SYS_CON0 |= BIT(29));
    
    /* io mapping */
    SYSCTRL_REG_OPT(SYSCTRL->IO_MAP |= BIT(7));
    
    /* reset gmac */
    LL_MAC_REG_OPT(GMAC->CSR0 = LL_MAC_CSR0_SWR_EN;);
    while(GMAC->CSR0 & LL_MAC_CSR0_SWR_EN) {
        __asm("nop");
    }

    /* use hardware mii management */
    LL_MAC_REG_OPT(p_gmac->CSR9 = 0x0000;);
    
    /* Calculating the division ratio */
    while((SYS_CLK / 8 >> clk_div) > 2500000) {
        clk_div++;
    }
    TX_ASSERT(clk_div <= LL_MAC_CLK_DIV_1024);
    
    /* Set the MDC clock to be less than 2.5Mhz */
    LL_MAC_REG_OPT(
        p_gmac->CSR10 = LL_MAC_CSR10_SB                                |
                        LL_MAC_CSR10_CLKDIV_SET(clk_div)               |
                        LL_MAC_CSR10_OPCODE_SET(LL_MAC_CLK_DIVIDER_SET);
    
    );
    while(p_gmac->CSR10 & LL_MAC_CSR10_SB) {
        __asm("nop");
    }
    
    /* PBL cannot be 0, the recommended setting is 16 */
    /* disable transmit automatic polling */
    LL_MAC_REG_OPT(
        p_gmac->CSR0 = LL_MAC_CSR0_TAP_SET(LL_MAC_TAP_DISABLE) |
                       LL_MAC_CSR0_PBL_SET(LL_MAC_PBL_16);
    );

    LL_MAC_REG_OPT(p_gmac->CSR3 = (u32)p_init->rx_descriptor_start_addr;);
    LL_MAC_REG_OPT(p_gmac->CSR4 = (u32)p_init->tx_descriptor_start_addr;);

    /* Save the starting descriptor pointer */
    __ll_mac_p_tx_descriptor = (TYPE_LL_MAC_TX_DESCRIPTOR *)p_init->tx_descriptor_start_addr;
    __ll_mac_p_rx_descriptor = (TYPE_LL_MAC_RX_DESCRIPTOR *)p_init->rx_descriptor_start_addr;
    __ll_mac_p_setup_tx_descriptor = __ll_mac_p_tx_descriptor;
    
    /* clear all pending */
    LL_MAC_REG_OPT(p_gmac->CSR5  = 0x00004DE7;);
    /* disable all interrupt */
    LL_MAC_REG_OPT(p_gmac->CSR7  = 0x00000000;);
    LL_MAC_REG_OPT(p_gmac->CSR11 = 0x00000000 | BIT(31););

    LL_MAC_REG_OPT(
        p_gmac->CSR6 = LL_MAC_CSR6_RU_DIS_EN | LL_MAC_CSR6_TXADR_UP_EN                        |
                       LL_MAC_CSR6_RXADR_UP_EN                                                |
                       (p_init->promiscuous_en ? (LL_MAC_CSR6_RA_EN | LL_MAC_CSR6_PR_EN) : 0) |
                       (p_init->pass_all_multicast_en ? LL_MAC_CSR6_PM_EN : 0)                |
                       (p_init->pass_bad_frame_en ? LL_MAC_CSR6_PB_EN : 0);
    );
    
    /* memset the backup tx descriptor */
    memset(&__ll_mac_tx_descriptor_backup, 0, sizeof(TYPE_LL_MAC_TX_DESCRIPTOR));
}

/**
  * @brief  GMAC module deinitialization
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_deinit(GMAC_TypeDef *p_gmac)
{
    TX_ASSERT(p_gmac == GMAC);
    
    /* reset gmac */
    LL_MAC_REG_OPT(GMAC->CSR0 = LL_MAC_CSR0_SWR_EN;);
    while(GMAC->CSR0 & LL_MAC_CSR0_SWR_EN) {
        __asm("nop");
    }
    
    /* release io */
    SYSCTRL_REG_OPT(SYSCTRL->IO_MAP &= ~BIT(7));
}

/**
  * @brief  GMAC interrupt enable setting
  * @param  p_gmac: GMAC module pointer
  * @param  p_cfg : GMAC interrupt enable control structure pointer
  * @retval None
  */
void ll_mac_irq_config(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_IRQ_CFG *p_cfg)
{
    TX_ASSERT(p_gmac == GMAC);
    
    p_gmac->CSR7 = LL_MAC_CSR7_NIE_EN | LL_MAC_CSR7_AIE_EN                      |
                   (p_cfg->rx_early_intr_en ? LL_MAC_CSR7_ERE_EN : 0)           |
                   (p_cfg->timer_overflow_intr_en ? LL_MAC_CSR7_GTE_EN : 0)     |
                   (p_cfg->tx_early_intr_en ? LL_MAC_CSR7_ETE_EN : 0)           |
                   (p_cfg->rx_stop_intr_en ? LL_MAC_CSR7_RSE_EN : 0)            |
                   (p_cfg->rx_buf_unavailable_intr_en ? LL_MAC_CSR7_RUE_EN : 0) |
                   (p_cfg->rx_intr_en ? LL_MAC_CSR7_RIE_EN : 0)                 |
                   (p_cfg->tx_underflow_intr_en ? LL_MAC_CSR7_UNE_EN : 0)       |
                   (p_cfg->tx_buf_unavilable_intr_en ? LL_MAC_CSR7_TUE_EN : 0)  |
                   (p_cfg->tx_stop_intr_en ? LL_MAC_CSR7_TSE_EN : 0)            |
                   (p_cfg->tx_intr_en ? LL_MAC_CSR7_TIE_EN : 0);
}

/**
  * @}
  */

/** @defgroup MAC_LL_Data_Transfers MAC LL Data transfers functions
  * @ingroup  MAC_LL_Driver
  * @brief    MAC LL Data transfers functions 
  * @{
  */

/**
  * @brief  Enable GMAC receiving function
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_start(GMAC_TypeDef *p_gmac)
{
    TX_ASSERT(p_gmac == GMAC);
    
    /* rx start */
    LL_MAC_REG_OPT(
        p_gmac->CSR6 |= LL_MAC_CSR6_ST_EN | LL_MAC_CSR6_SR_EN;
    );
    /* clear pending */
    LL_MAC_REG_OPT(
        p_gmac->CSR5 = LL_MAC_CSR5_TPS_PENDING | LL_MAC_CSR5_RPS_PENDING |
                       LL_MAC_CSR5_RU_PENDING;
    );
}

/**
  * @brief  Stop the GMAC module
  * @param  p_gmac: GMAC module pointer
  * @retval None
  */
void ll_mac_stop(GMAC_TypeDef *p_gmac)
{
    TX_ASSERT(p_gmac == GMAC);
    
    LL_MAC_REG_OPT(
        p_gmac->CSR6 &= ~(LL_MAC_CSR6_ST_EN | LL_MAC_CSR6_SR_EN);
    );
}

/**
  * @brief  The MII management interface sends data
  * @param  p_gmac  : GMAC module pointer
  * @param  phy_addr: PHY address
  * @param  reg_addr: register address
  * @param  data    : Data to send
  * @retval None
  * @note   Data format symbol IEEE802.3 clause 22
  */
void ll_mac_mdio_write(GMAC_TypeDef *p_gmac, u8 phy_addr, u8 reg_addr, u16 data)
{
    TX_ASSERT(p_gmac == GMAC);
    TX_ASSERT(phy_addr < 0x20);
    TX_ASSERT(reg_addr < 0x20);
    
    LL_MAC_REG_OPT(
        p_gmac->CSR10 = LL_MAC_CSR10_SB |
                        LL_MAC_CSR10_OPCODE_SET(LL_MAC_REG_WRITE) |
                        LL_MAC_CSR10_PHYADD_SET(phy_addr)         |
                        LL_MAC_CSR10_REGADD_SET(reg_addr)         |
                        LL_MAC_CSR10_DATA(data);
    );
    while(p_gmac->CSR10 & LL_MAC_CSR10_SB) {
        __asm("nop");
    }
}

/**
  * @brief  The MII management interface receives data
  * @param  p_gmac  : GMAC module pointer
  * @param  phy_addr: PHY address
  * @param  reg_addr: register address
  * @retval Return the read data
  * @note   Data format symbol IEEE802.3 clause 22
  */
u16 ll_mac_mdio_read(GMAC_TypeDef *p_gmac, u8 phy_addr, u8 reg_addr)
{
    TX_ASSERT(p_gmac == GMAC);
    TX_ASSERT(phy_addr < 0x20);
    TX_ASSERT(reg_addr < 0x20);
    
    LL_MAC_REG_OPT(
        p_gmac->CSR10 = LL_MAC_CSR10_SB                          |
                        LL_MAC_CSR10_OPCODE_SET(LL_MAC_REG_READ) |
                        LL_MAC_CSR10_PHYADD_SET(phy_addr)        |
                        LL_MAC_CSR10_REGADD_SET(reg_addr);
    );
    while(p_gmac->CSR10 & LL_MAC_CSR10_SB) {
        __asm("nop");
    }
    return LL_MAC_CSR10_DATA(p_gmac->CSR10);
}

/**
  * @brief  GMAC has received the frame?
  * @param  p_gmac: GMAC module pointer
  * @retval Returns true if received, false if none.
  */
bool ll_mac_has_received_frame(GMAC_TypeDef *p_gmac)
{
    TX_ASSERT(p_gmac == GMAC);
    
    if(__ll_mac_p_rx_descriptor->own) {
        return false;
    } else {
        return true;
    }
}

/**
  * @brief  GMAC allows to send a frame?
  * @param  p_gmac: GMAC module pointer
  * @retval Returning true means permission, returning false means that the
  *         TX descriptor has run out and cannot be sent.
  */
bool ll_mac_allow_send_frame(GMAC_TypeDef *p_gmac)
{
    TX_ASSERT(p_gmac == GMAC);
    
    if(__ll_mac_p_tx_descriptor->own) {
        return false;
    } else {
        return true;
    }
}

/**
  * @brief  GMAC sends a frame of data
  * @param  p_gmac        : GMAC module pointer
  * @param  p_tx_frame_ctl: tx frame structure pointer
  * @retval None
  */
void ll_mac_send_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_TX_FRAME_CTL *p_tx_frame_ctl)
{
    TX_ASSERT(p_gmac == GMAC);
    
    /* Recovery descriptor */
    if(__ll_mac_p_tx_descriptor->setup_packet) {
        memcpy(__ll_mac_p_tx_descriptor, &__ll_mac_tx_descriptor_backup,
               sizeof(TYPE_LL_MAC_TX_DESCRIPTOR));
    }
    
    TX_ASSERT_ADDR((u32)__ll_mac_p_tx_descriptor, sizeof(TYPE_LL_MAC_TX_DESCRIPTOR),
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    TX_ASSERT_ADDR(__ll_mac_p_tx_descriptor->tx_buf_addr_1, p_tx_frame_ctl->frame_len,
                   SRAM_GMAC_TXFIFO_MASK, SRAM_GMAC_TXFIFO_ALIGN_MASK);
    
    memcpy((void *)__ll_mac_p_tx_descriptor->tx_buf_addr_1,
            p_tx_frame_ctl->frame_buf,
            p_tx_frame_ctl->frame_len);
    __ll_mac_p_tx_descriptor->buf_1_size        = p_tx_frame_ctl->frame_len;
    __ll_mac_p_tx_descriptor->int_on_completion = 1;
    __ll_mac_p_tx_descriptor->own               = 1;
    /* Save the descriptor used by tx, which will be used later */
    p_tx_frame_ctl->p_tx_descriptor  = __ll_mac_p_tx_descriptor;
    /* The descriptor automatically points to the next descriptor */
    __ll_mac_p_tx_descriptor = __ll_mac_p_tx_descriptor->p_next;
    
    /* kick tx start */
    if(!(p_gmac->CSR0 & LL_MAC_CSR0_TAP_MASK)) {
        LL_MAC_REG_OPT (
            p_gmac->CSR5 = LL_MAC_CSR5_TU_PENDING | LL_MAC_CSR5_UNF_PENDING;
        );
        LL_MAC_REG_OPT(p_gmac->CSR1 = 0x00;);
    }
}

/**
  * @brief  GMAC setup frame transmission
  * @param  p_gmac           : GMAC module pointer
  * @param  p_setup_frame_ctl: setup frame structure pointer
  * @retval Returning true means that the setup frame is placed in the send
  *         queue. Returning false means that the previous setup frame was
  *         not sent.
  * @note   Wait for a setup frame to be sent before you can continue to
  *         send a setup frame.
  */
bool ll_mac_setup_send_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_SETUP_FRAME_CTL *p_setup_frame_ctl)
{
    TX_ASSERT(p_gmac == GMAC);
    TX_ASSERT_ADDR((u32)__ll_mac_p_setup_tx_descriptor, sizeof(TYPE_LL_MAC_TX_DESCRIPTOR),
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    TX_ASSERT_ADDR((u32)p_setup_frame_ctl->p_setup_frame, 192,
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    
    /* The last setup frame was not sent. */
    if(__ll_mac_p_setup_tx_descriptor->setup_packet && __ll_mac_p_setup_tx_descriptor->own) {
        return false;
    }

    /* backup */
    memcpy(&__ll_mac_tx_descriptor_backup, __ll_mac_p_tx_descriptor,
           sizeof(TYPE_LL_MAC_TX_DESCRIPTOR));
    
    /* interrupt on completion */
    __ll_mac_p_tx_descriptor->int_on_completion = 1;
    /* clear first&last descriptor */
    __ll_mac_p_tx_descriptor->first_descriptor  = 0;
    __ll_mac_p_tx_descriptor->last_descriptor   = 0;
    
    /* configurate destination address filter mode */
    switch(p_setup_frame_ctl->filter_mode) {
        case LL_MAC_PERFECT_FILTER:
            __ll_mac_p_tx_descriptor->filtering_type_1 = 0;
            __ll_mac_p_tx_descriptor->filtering_type_0 = 0;
            break;
        
        case LL_MAC_HASH_FILTER:
            __ll_mac_p_tx_descriptor->filtering_type_1 = 0;
            __ll_mac_p_tx_descriptor->filtering_type_0 = 1;
            break;
        
        case LL_MAC_INVERSE_FILTER:
            __ll_mac_p_tx_descriptor->filtering_type_1 = 1;
            __ll_mac_p_tx_descriptor->filtering_type_0 = 0;
            break;
        
        default:    //LL_MAC_HASH_ONLY_FILTER
            __ll_mac_p_tx_descriptor->filtering_type_1 = 1;
            __ll_mac_p_tx_descriptor->filtering_type_0 = 1;
            break;
    }

    /* setup packet */
    __ll_mac_p_tx_descriptor->setup_packet  = 1;
    /* size = 192byte */
    __ll_mac_p_tx_descriptor->buf_1_size    = 192;
    __ll_mac_p_tx_descriptor->tx_buf_addr_1 = (u32)p_setup_frame_ctl->p_setup_frame; 
    __ll_mac_p_tx_descriptor->own           = 1;
    
    /* Save the descriptor pointer for later query */
    p_setup_frame_ctl->p_tx_descriptor = __ll_mac_p_tx_descriptor;

    /* The descriptor automatically points to the next descriptor */
    __ll_mac_p_tx_descriptor = __ll_mac_p_tx_descriptor->p_next;
    
    /* kick tx start */
    if(!(p_gmac->CSR0 & LL_MAC_CSR0_TAP_MASK)) {
        LL_MAC_REG_OPT (
            p_gmac->CSR5 = LL_MAC_CSR5_TU_PENDING | LL_MAC_CSR5_UNF_PENDING;
        );
        LL_MAC_REG_OPT(p_gmac->CSR1 = 0x00;);
    }
    
    return true;
}

/**
  * @brief  GMAC receives a frame of data
  * @param  p_gmac        : GMAC module pointer
  * @param  p_rx_frame_ctl: rx frame structure pointer
  * @retval None
  */
void ll_mac_receive_frame(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_RX_FRAME_CTL *p_rx_frame_ctl)
{
    TX_ASSERT(p_gmac == GMAC);
    TX_ASSERT_ADDR((u32)__ll_mac_p_rx_descriptor, sizeof(TYPE_LL_MAC_RX_DESCRIPTOR),
                   SRAM_GMAC_DMA_MASK, SRAM_GMAC_DMA_ALIGN_MASK);
    TX_ASSERT_ADDR(__ll_mac_p_rx_descriptor->rx_buf_addr_1, __ll_mac_p_rx_descriptor->buf_1_size,
                   SRAM_GMAC_RXFIFO_MASK, SRAM_GMAC_RXFIFO_ALIGN_MASK);
    
    memcpy(p_rx_frame_ctl->frame_buf,
           (void *)__ll_mac_p_rx_descriptor->rx_buf_addr_1,
           __ll_mac_p_rx_descriptor->frame_len);
    p_rx_frame_ctl->frame_len = __ll_mac_p_rx_descriptor->frame_len;
    /* Save status information, will be used later */
    p_rx_frame_ctl->status = ((TYPE_LL_MAC_DESCRIPTOR *)__ll_mac_p_rx_descriptor)->reg0;   
    /* Release descriptor */
    __ll_mac_p_rx_descriptor->own = 1;
    /* The descriptor automatically points to the next descriptor */
    __ll_mac_p_rx_descriptor = __ll_mac_p_rx_descriptor->p_next;
}

/**
  * @brief  Get the status of the setup frame
  * @param  p_tx_frame_ctl: tx frame structure pointer
  * @retval tx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_setup_frame_status(TYPE_LL_MAC_SETUP_FRAME_CTL *p_setup_frame_ctl)
{
    if(p_setup_frame_ctl->p_tx_descriptor->setup_packet) {
        if(p_setup_frame_ctl->p_tx_descriptor->own) {
            return LL_MAC_FRAME_WAITTING_COMPLETE;
        }
    }   // setup_packet = 0, means the setup frame has been sent.
    return LL_MAC_FRAME_COMPLETE;
}

/**
  * @brief  Get the status of the send frame
  * @param  p_tx_frame_ctl: tx frame structure pointer
  * @retval tx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_tx_frame_status(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_TX_FRAME_CTL *p_tx_frame_ctl)
{
    TX_ASSERT(p_gmac == GMAC);
    
    if(p_tx_frame_ctl->p_tx_descriptor->own) {
        return LL_MAC_FRAME_WAITTING_COMPLETE;
    } else if(p_gmac->CSR6 & LL_MAC_CSR6_FD) {
        /* In full-duplex mode, only underflow error may occur in tx. */
        if(p_tx_frame_ctl->p_tx_descriptor->underflow_err) {
            return LL_MAC_FRAME_ERR;
        } else {
            return LL_MAC_FRAME_COMPLETE;
        }
    } else if(p_tx_frame_ctl->p_tx_descriptor->err_summary) {
        return LL_MAC_FRAME_ERR;
    } else {
        return LL_MAC_FRAME_COMPLETE;
    }
}

/**
  * @brief  Get the status of the received frame
  * @param  p_rx_frame_ctl: rx frame structure pointer
  * @retval rx frame status
  */
TYPE_ENUM_LL_MAC_FRAME_STATUS ll_mac_get_rx_frame_status(GMAC_TypeDef *p_gmac, TYPE_LL_MAC_RX_FRAME_CTL *p_rx_frame_ctl)
{
    TX_ASSERT(p_gmac == GMAC);
    
    if(p_rx_frame_ctl->status & BIT(15)) {
        return LL_MAC_FRAME_ERR;
    } else {
        return LL_MAC_FRAME_COMPLETE;
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

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
