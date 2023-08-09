/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_iic.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the IIC LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  * The correspondence table of IO_MAP is as follows:
  * ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  * ©¦           ©¦   LL_IIC_IO_MAP0  ©¦  LL_IIC_IO_SMBUS_MAP0  ©¦  LL_IIC_IO_SMBUS_MAP1  ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦           ©¦                   ©¦  iic_scl(PMBUS_CLK)/   ©¦  iic_scl(PMBUS_CLK)/   ©¦
  * ©¦  io name  ©¦  iic_scl/iic_sda  ©¦  iic_sda(PMBUS_DATA)/  ©¦  iic_sda(PMBUS_DATA)/  ©¦
  * ©¦           ©¦                   ©¦          PMBUS_CTRL/   ©¦          PMBUS_CTRL/   ©¦
  * ©¦           ©¦                   ©¦          PMBUS_ALERT   ©¦          PMBUS_ALERT   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦    IIC0   ©¦      PA6/PA7      ©¦     PA6/PA7/P4/PA5     ©¦   PB8/PB9/PB10/PB11    ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦    IIC1   ©¦      PA8/PA9      ©¦     PD0/PD1/PD2/PD3    ©¦                        ©¦
  * ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_iic.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup iic_interface_gr IIC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup IIC_LL_Driver IIC LL Driver
  * @ingroup  iic_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup IIC_LL_Interrupt IIC LL Interrupt Handle function
  * @ingroup  IIC_LL_Driver
  * @brief   IIC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup IIC_LL_Inti_Cfg IIC LL Initialization And Configuration
  * @ingroup  IIC_LL_Driver
  * @brief    IIC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Low layer IIC module initialization
  * @param  p_iic: The structure pointer of the IIC group (IIC0, IIC1) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_IIC_INIT)
  * @retval None
  */
void ll_iic_init(IIC_TypeDef *p_iic, TYPE_LL_IIC_INIT *p_init)
{
    /* The system resets the IIC module, Enable the clock of the module. */
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    memset(p_iic, 0x0, sizeof(IIC_TypeDef));
}

/**
  * @brief  Low layer IIC module detele initialization
  * @param  p_iic: The structure pointer of the IIC group (IIC0, IIC1) is selected.
  * @retval None
  */
void ll_iic_deinit(IIC_TypeDef *p_iic)
{
    /* The system disable the IIC module, includes turning off the clock for the module. */
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(p_iic == IIC0) {
        SYSCTRL->IO_MAP &= ~(BIT(0) | BIT(17) | BIT(18));
    }else if(p_iic == IIC1) {
        SYSCTRL->IO_MAP &= ~(BIT(1) | BIT(19));
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/**
  * @brief  Low layer IIC module interrupt configuration
  * @param  p_iic: The structure pointer of the IIC group (IIC0, IIC1) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_IIC_IRQ_CFG)
  * @retval None
  */
void ll_iic_irq_config(IIC_TypeDef *p_iic, TYPE_LL_IIC_IRQ_CFG *p_cfg)
{
    u32 intr_mask = 0x48FF;
    
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    
    intr_mask = p_cfg->rx_under_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_RX_UNDER)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RX_UNDER)));
    intr_mask = p_cfg->rx_over_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_RX_OVER)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RX_OVER)));
    intr_mask = p_cfg->rx_full_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_RX_FULL)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RX_FULL)));
    intr_mask = p_cfg->tx_over_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_TX_OVER)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_TX_OVER)));
    intr_mask = p_cfg->tx_empty_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_TX_EMPTY)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_TX_EMPTY)));
    intr_mask = p_cfg->read_req_intr_en                     ?
                (intr_mask | LL_IIC_INTR_MASK_M_RD_REQ)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RD_REQ)));
    intr_mask = p_cfg->tx_abrt_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_TX_ABRT)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_TX_ABRT)));
    intr_mask = p_cfg->rx_done_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_RX_DONE)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RX_DONE)));
    intr_mask = p_cfg->activity_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_ACTIVITY)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_ACTIVITY)));
    intr_mask = p_cfg->stop_det_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_STOP_DET)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_STOP_DET)));
    intr_mask = p_cfg->start_det_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_START_DET)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_START_DET)));
    intr_mask = p_cfg->gen_call_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_GEN_CALL)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_GEN_CALL)));
    intr_mask = p_cfg->restart_det_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_RESTART_DET)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_RESTART_DET)));
    intr_mask = p_cfg->master_on_hold_intr_en                    ?
                (intr_mask | LL_IIC_INTR_MASK_M_MST_ON_HOLD)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_MST_ON_HOLD)));
    intr_mask = p_cfg->scl_stuck_at_low_intr_en                       ?
                (intr_mask | LL_IIC_INTR_MASK_M_SCL_STUCK_AT_LOW)     :
                (intr_mask & (~(LL_IIC_INTR_MASK_M_SCL_STUCK_AT_LOW)));
    
    p_iic->INTR_MASK = LL_IIC_INTR_MASK(intr_mask);
}

/**
  * @brief  Low layer iic io map init function
  * @param  p_iic: The structure pointer of the IIC group (IIC0, IIC1) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_IIC_CFG)
  * @retval None
  */
void ll_iic_io_map(IIC_TypeDef *p_iic, TYPE_LL_IIC_CFG *p_cfg)
{
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    
    if(IIC1 == p_iic) {
        // IIC1 can only be set to LL_IIC_IO_MAP0 and LL_IIC_IO_SMBUS_MAP0.
        TX_ASSERT(p_cfg->io_map < LL_IIC_IO_SMBUS_MAP1);
    }
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    /* IIC0 IO_MAP */
    if(IIC0 == p_iic) {
        
        //clear all io map of IIC0
        SYSCTRL->IO_MAP &= ~(BIT(0) | BIT(17) | BIT(18));
        
        /* IIC0 map0 */
        if(p_cfg->io_map & LL_IIC_IO_MAP0) {
            SYSCTRL->IO_MAP |= BIT(0);
        }
        /* IIC0 Smbus map0 */
        if(p_cfg->io_map & LL_IIC_IO_SMBUS_MAP0) {
            SYSCTRL->IO_MAP |= BIT(17);
        }
        /* IIC0 Smbus map1 */
        if(p_cfg->io_map & LL_IIC_IO_SMBUS_MAP1) {
            SYSCTRL->IO_MAP |= BIT(18);
        }
    /* IIC1 IO_MAP */
    } else if(IIC1 == p_iic){
        
        //clear all io map of IIC1
        SYSCTRL->IO_MAP &= ~(BIT(1) | BIT(19));
        
        /* IIC1 map0 */
        if(p_cfg->io_map & LL_IIC_IO_MAP0) {
            SYSCTRL->IO_MAP |= BIT(1);
        }
        /* IIC1 Smbus map0 */
        if(p_cfg->io_map & LL_IIC_IO_SMBUS_MAP0) {
            SYSCTRL->IO_MAP |= BIT(19);
        }
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/**
  * @brief  Low layer IIC module configuration
  * @param  p_iic: The structure pointer of the IIC group (IIC0, IIC1) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_IIC_CFG)
  * @retval None
  */
void ll_iic_config(IIC_TypeDef *p_iic, TYPE_LL_IIC_CFG *p_cfg)
{
    u32 iic_con = 0;
    u32 iic_tar = 0;
    u32 baudrate_cnt = ll_cc_sys_clk_get()/p_cfg->baudrate/2;
    u32 ss_scl_hcnt = LL_IIC_IC_SS_SCL_HCNT(baudrate_cnt);
    u32 ss_scl_lcnt = LL_IIC_IC_SS_SCL_LCNT(baudrate_cnt);
    
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    
    ll_iic_io_map(p_iic, p_cfg);
    
    if(p_cfg->mode_sel == LL_IIC_MODE_MASTER) {
        iic_con |= LL_IIC_CON_MASTER_MODE;
        iic_con |= LL_IIC_CON_IC_SLAVE_DISABLE;
        iic_con |= p_cfg->addr_sel ? LL_IIC_CON_IC10BITADDR_MASTER : 0;
    } else if(p_cfg->mode_sel == LL_IIC_MODE_SLAVE) {
        iic_con &= ~LL_IIC_CON_MASTER_MODE;
        iic_con &= ~LL_IIC_CON_IC_SLAVE_DISABLE;
        iic_con |= p_cfg->addr_sel ? LL_IIC_CON_IC10BITADDR_SLAVE : 0;
    }
    
    if(!((!p_cfg->ss_scl_hcnt) && (!p_cfg->ss_scl_lcnt))) {
        ss_scl_hcnt = LL_IIC_IC_SS_SCL_HCNT(p_cfg->ss_scl_hcnt);
        ss_scl_lcnt = LL_IIC_IC_SS_SCL_LCNT(p_cfg->ss_scl_lcnt);
    }
    
    iic_con |= LL_IIC_CON_SPEED(LL_IIC_SPEED_STANDARD_MODE);
    iic_con |= p_cfg->restart_en ? LL_IIC_CON_IC_RESTART_EN : 0;
    iic_con |= p_cfg->stop_det_if_addr_en ? LL_IIC_CON_STOP_DET_IFADDRESSED : 0;
    iic_con |= p_cfg->tx_empty_ctrl ? LL_IIC_CON_TX_EMPTY_CTRL : 0;
    iic_con |= p_cfg->rx_fifo_full_hld_ctrl ? LL_IIC_CON_RX_FIFO_FULL_HLD_CTRL : 0;
    iic_con |= p_cfg->stop_det_if_master_active ? LL_IIC_CON_STOP_DEC_IF_MASTER_ACTINE : 0;
    iic_con |= p_cfg->bus_clear_feature_ctrl ? LL_IIC_CON_BUS_CLEAR_FEATURE_CTRL : 0;
    iic_con |= p_cfg->optional_sar_ctrl ? LL_IIC_CON_OPTION_SAR_CTRL : 0;

    iic_tar |= LL_IIC_TAR_IC_TAR(p_cfg->target_addr);
    
    p_iic->CON         = iic_con;
    p_iic->TAR         = iic_tar;
    p_iic->SAR         = LL_IIC_SAR_IC_SAR(p_cfg->slave_addr);
    p_iic->SS_SCL_HCNT = ss_scl_hcnt;
    p_iic->SS_SCL_LCNT = ss_scl_lcnt;
    p_iic->RX_TL       = LL_IIC_RX_TL(p_cfg->rx_threshold_level);
    p_iic->TX_TL       = LL_IIC_TX_TL(p_cfg->tx_threshold_level);
    p_iic->DMA_TDLR    = LL_IIC_DMA_TDLR(p_cfg->dma_tx_threshold_level);
    p_iic->DMA_RDLR    = LL_IIC_DMA_RDLR(p_cfg->dma_rx_threshold_level);
    p_iic->FS_SPKLEN   = 0x10;
    p_iic->SDA_HOLD    = (0x80<<16)|(0x80<<0);
}

/**
  * @}
  */

/** @defgroup IIC_LL_Data_Transfers IIC LL Data transfers functions
  * @ingroup  IIC_LL_Driver
  * @brief    IIC LL Data transfers functions 
  * @{
  */

/**
  * @brief  IIC module start function
  * @param  p_iic: Select the initialized IIC group pointer
  * @retval None
  */
void ll_iic_start(IIC_TypeDef *p_iic)
{
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    p_iic->ENABLE |= LL_IIC_ENABLE;
}

/**
  * @brief  IIC module DMA start function
  * @param  p_iic: Select the initialized IIC group pointer
  * @retval None
  */
void ll_iic_dma_start(IIC_TypeDef *p_iic)
{
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    p_iic->ENABLE |= LL_IIC_ENABLE;
    p_iic->DMA_CR  = LL_IIC_DMA_CR_TDMAE | LL_IIC_DMA_CR_RDMAE;
}

/**
  * @brief  IIC module stop function
  * @param  p_iic: Select the initialized IIC group pointer
  * @retval None
  */
void ll_iic_stop(IIC_TypeDef *p_iic)
{
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    p_iic->ENABLE &= ~(LL_IIC_ENABLE);
}

/**
  * @brief  IIC module DMA stop function
  * @param  p_iic: Select the initialized IIC group pointer
  * @retval None
  */
void ll_iic_dma_stop(IIC_TypeDef *p_iic)
{
    TX_ASSERT((p_iic == IIC0) || (p_iic == IIC1));
    p_iic->ENABLE &= ~(LL_IIC_ENABLE);
    p_iic->DMA_CR  = 0;
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
