/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_can.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the CAN LL firmware functions.
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
#include "tx_phe_ll_can.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup can_interface_gr CAN Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup CAN_LL_Driver CAN LL Driver
  * @ingroup  can_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup CAN_LL_Interrupt CAN LL Interrupt Handle function
  * @ingroup  CAN_LL_Driver
  * @brief   CAN LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup CAN_LL_Inti_Cfg CAN LL Initialization And Configuration
  * @ingroup  CAN_LL_Driver
  * @brief    CAN LL Initialization And Configuration
  * @{
  */
  
/**
  * @brief  Low layer CAN module initialization
  * @param  p_can : The register structure pointer of the CAN.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_CAN_INIT)
  * @retval None
  */
void ll_can_init(CAN_TypeDef *p_can, TYPE_LL_CAN_INIT *p_init)
{
    /* The system resets the CAN module, Enable the clock of the module. */
    TX_ASSERT(p_can == CAN);
    
    memset(p_can, 0x0, sizeof(CAN_TypeDef));
    
    p_can->RTIE  = LL_CAN_RTIE_RIE   |
                   LL_CAN_RTIE_ROIE  |
                   LL_CAN_RTIE_RFIE  |
                   LL_CAN_RTIE_RAFIE |
                   LL_CAN_RTIE_TPIE  |
                   LL_CAN_RTIE_TSIE  |
                   LL_CAN_RTIE_EIE;
    p_can->LIMIT = LL_CAN_LIMIT_AFWL(0x1) |
                   LL_CAN_LIMIT_EWL(0xB);
    
    LL_CAN_REG_OPT(
        p_can->BITTIME0 = LL_CAN_BITTIME_0_F_SJW(0x2)  |
                          LL_CAN_BITTIME_0_S_SEG_1(0x3);
        p_can->BITTIME1 = LL_CAN_BITTIME_1_F_SEG_2(0x2) |
                          LL_CAN_BITTIME_1_S_SEG_2(0x2);
        p_can->BITTIME2 = LL_CAN_BITTIME_2_F_SEG_1(0x3) |
                          LL_CAN_BITTIME_2_S_SJW(0x2);
        p_can->F_PRESC  = LL_CAN_F_PRESC(0x1);
        p_can->S_PRESC  = LL_CAN_S_PRESC(0x1);
        p_can->ACF_EN_0 = LL_CAN_ACF_EN_0(0x1);
        p_can->ACF_EN_1 = 0;
        p_can->TDC      = 0;
        p_can->ACFCTRL  = 0;
    );
}

/**
  * @brief  Low layer CAN module detele initialization
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_deinit(CAN_TypeDef *p_can)
{
    /* The system disable the CAN module, includes turning off the clock for the module. */
    TX_ASSERT(p_can == CAN);
    
    SYSCTRL_REG_OPT(
        /* can FD disalbe */
        SYSCTRL->SYS_CON0 &= ~BIT(21);
        SYSCTRL->IO_MAP   &= ~BIT(9);
    );
}

/**
  * @brief  Low layer CAN module interrupt configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module interrupt configuration structure pointer
  *                (TYPE_LL_CAN_IRQ_CFG)
  * @retval None
  */
void ll_can_irq_config(CAN_TypeDef *p_can, TYPE_LL_CAN_IRQ_CFG *p_cfg) 
{
    u8 can_rtie   = p_can->RTIE;
    u8 can_errint = p_can->ERRINT;
    
    TX_ASSERT(p_can == CAN);
    
    can_rtie = p_cfg->rx_intr_en              ?
               (can_rtie | LL_CAN_RTIE_RIE)   :
               (can_rtie & ~(LL_CAN_RTIE_RIE));
    can_rtie = p_cfg->rx_buf_overrun_intr_en   ?
               (can_rtie | LL_CAN_RTIE_ROIE)   :
               (can_rtie & ~(LL_CAN_RTIE_ROIE));
    can_rtie = p_cfg->rx_buf_full_intr_en      ?
               (can_rtie | LL_CAN_RTIE_RFIE)   :
               (can_rtie & ~(LL_CAN_RTIE_RFIE));
    can_rtie = p_cfg->rx_buf_almost_full_intr_en ?
               (can_rtie | LL_CAN_RTIE_RAFIE)    :
               (can_rtie & ~(LL_CAN_RTIE_RAFIE));
    can_rtie = p_cfg->tx_primary_intr_en       ?
               (can_rtie | LL_CAN_RTIE_TPIE)   :
               (can_rtie & ~(LL_CAN_RTIE_TPIE));
    can_rtie = p_cfg->tx_secondary_intr_en     ?
               (can_rtie | LL_CAN_RTIE_TSIE)   :
               (can_rtie & ~(LL_CAN_RTIE_TSIE));
    can_rtie = p_cfg->err_intr_en             ?
               (can_rtie | LL_CAN_RTIE_EIE)   :
               (can_rtie & ~(LL_CAN_RTIE_EIE));

    can_errint = p_cfg->err_passive_intr_en          ?
                 (can_errint | LL_CAN_ERRINT_EPIE)   :
                 (can_errint & ~(LL_CAN_ERRINT_EPIE));
    can_errint = p_cfg->arbitration_lost_intr_en     ?
                 (can_errint | LL_CAN_ERRINT_ALIE)   :
                 (can_errint & ~(LL_CAN_ERRINT_ALIE));
    can_errint = p_cfg->bus_err_intr_en              ?
                 (can_errint | LL_CAN_ERRINT_BEIE)   :
                 (can_errint & ~(LL_CAN_ERRINT_BEIE));

    p_can->RTIE   = can_rtie;
    p_can->ERRINT = can_errint;
}

/**
  * @brief  Low layer CAN module address filter configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module address filter configuration structure pointer
  *                (TYPE_LL_CAN_ADDR_FILT_CFG)
  * @retval None
  */
void ll_can_address_filter_config(CAN_TypeDef *p_can, TYPE_LL_CAN_ADDR_FILT_CFG *p_cfg) 
{
    TX_ASSERT(p_can == CAN);
    
    LL_CAN_REG_OPT(
        p_can->ACFCTRL = LL_CAN_ACFCTRL_ACFADR(p_cfg->chn);
        p_can->ACF     = LL_CAN_ACODE_X(p_cfg->acode_val);
        p_can->ACFCTRL = LL_CAN_ACFCTRL_SELMASK | LL_CAN_ACFCTRL_ACFADR(p_cfg->chn);
        p_can->ACF     = LL_CAN_AMASK_X(p_cfg->amask_val);
    );
    
    if(p_cfg->chn < 8) {
        p_can->ACF_EN_0 = p_cfg->enable                                            ?
                          (p_can->ACF_EN_0 | LL_CAN_ACF_EN_0(BIT(p_cfg->chn)))     :
                          (p_can->ACF_EN_0 & (~(LL_CAN_ACF_EN_0(BIT(p_cfg->chn)))));
    } else {
        p_can->ACF_EN_1 = p_cfg->enable                                                ?
                          (p_can->ACF_EN_1 | LL_CAN_ACF_EN_1(BIT(p_cfg->chn - 8)))     :
                          (p_can->ACF_EN_1 & (~(LL_CAN_ACF_EN_1(BIT(p_cfg->chn - 8)))));
    }
}

/**
  * @brief  Low layer CAN module configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_CAN_CFG)
  * @retval None
  */
void ll_can_config(CAN_TypeDef *p_can, TYPE_LL_CAN_CFG *p_cfg)
{
    u8 can_limit = 0;
    
    TX_ASSERT(p_can == CAN);
    
    can_limit = LL_CAN_LIMIT_AFWL(p_cfg->rx_almost_full_limit_sel) |
                LL_CAN_LIMIT_EWL(p_cfg->program_err_limit_sel);
    
    ll_gpio_pull(GPIOD, BIT(6) | BIT(7), LL_GPIO_PULL_UP);
    SYSCTRL_REG_OPT(
        /* can FD disalbe */
        SYSCTRL->SYS_CON0 |= BIT(21);
        SYSCTRL->IO_MAP   |= BIT(9);
    );
    
    p_can->LIMIT = can_limit;
    
    LL_CAN_REG_OPT(
        p_can->S_PRESC = LL_CAN_S_PRESC(p_cfg->baudrate);
    );
}

/**
  * @}
  */

/** @defgroup CAN_LL_Data_Transfers CAN LL Data transfers functions
  * @ingroup  CAN_LL_Driver
  * @brief    CAN LL Data transfers functions 
  * @{
  */

/**
  * @brief  CAN module start function
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_start(CAN_TypeDef *p_can)
{
    TX_ASSERT(p_can == CAN);
}

/**
  * @brief  CAN module stop function
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_stop(CAN_TypeDef *p_can)
{
    TX_ASSERT(p_can == CAN);
}

/**
  * @brief  CAN sends the function of the standard Primary Transmit Buffer.
  * @param  p_can        : The register structure pointer of the CAN.
  * @param  p_buf_format : The frame header format in which can can send data.
  * @param  p_data       : The header address pointer for sending data.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_PTB(CAN_TypeDef               *p_can, 
                              TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                              u32                       *p_data) 
{
    p_can->TCMD &= ~LL_CAN_TCMD_TBSEL;
    
    p_can->CANTBUF[0] = *((u32 *)p_buf_format);
    p_can->CANTBUF[1] = *(((u32 *)p_buf_format) + 1);

    for(int i=0, j=0; i<p_buf_format->data_len_code; i+=4, j++) {
        p_can->CANTBUF[2 + j] = *p_data++;
    }
    
    //p_can->TCTRL |= LL_CAN_TCTRL_TSNEXT;
    p_can->TCMD  |= LL_CAN_TCMD_TPE;
    while((p_can->TCMD & LL_CAN_TCMD_TPE) != 0);
}

/**
  * @brief  CAN sends a package of standard Secondary Transmit Buffer functions.
  * @param  p_can        : The register structure pointer of the CAN.
  * @param  p_buf_format : The frame header format in which can can send data.
  * @param  p_data       : The header address pointer for sending data.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_STB(CAN_TypeDef               *p_can, 
                              TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                              u32                       *p_data) 
{
    p_can->TCMD |= LL_CAN_TCMD_TBSEL;
    
    p_can->CANTBUF[0] = *((u32 *)p_buf_format);
    p_can->CANTBUF[1] = *(((u32 *)p_buf_format) + 1);
    
    for(int i=0, j=0; i<p_buf_format->data_len_code; i+=4, j++) {
        p_can->CANTBUF[2 + j] = *p_data++;
    }
    
    p_can->TCTRL |= LL_CAN_TCTRL_TSNEXT;
    p_can->TCMD  |= LL_CAN_TCMD_TSONE;
    while((p_can->TCMD & LL_CAN_TCMD_TSONE) != 0);
}

/**
  * @brief  CAN sends a multi-package standard Secondary Transmit Buffer function.
  * @param  p_can        : The register structure pointer of the CAN.
  * @param  p_buf_format : The frame header format in which can can send data.
  * @param  p_data       : The header address pointer for sending data.
  * @param  send_cnt     : The number of CAN packets.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_STB_most(CAN_TypeDef               *p_can, 
                                   TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                                   u32                       *p_data, 
                                   u32                        send_cnt) 
{
    p_can->TCMD |= LL_CAN_TCMD_TBSEL;
    
    for(int k=0; k<send_cnt; k++) {
        p_can->CANTBUF[0] = *((u32 *)p_buf_format);
        p_can->CANTBUF[1] = *(((u32 *)p_buf_format) + 1);

        for(int i=0, j=0; i<p_buf_format->data_len_code; i+=4, j++) {
            p_can->CANTBUF[2 + j] = *p_data++;
        }
        p_can->TCTRL |= LL_CAN_TCTRL_TSNEXT;
        
        if(p_can->RTIE & LL_CAN_RTIE_TSFF) {
            p_can->TCMD |= LL_CAN_TCMD_TSALL;
            while((p_can->TCMD & LL_CAN_TCMD_TSALL) != 0);
        }
    }
    
    p_can->TCMD |= LL_CAN_TCMD_TSALL;
    while((p_can->TCMD & LL_CAN_TCMD_TSALL) != 0);
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
