/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_usart.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the USART LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  * The correspondence table of IO_MAP is as follows:
  * ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  * ©¦           ©¦                 USART 232                 ©¦       USART 485      ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦           ©¦  LL_USART_IO_MAP0   ©¦  LL_USART_IO_MAP1   ©¦   LL_USART_IO_MAP0   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦  io name  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx/  ©¦
  * ©¦           ©¦                     ©¦                     ©¦  usart_re/usart_de   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦  USART0   ©¦      PA10/PA11      ©¦                     ©¦  PA10/PA11/PA9/PA8   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦  USART1   ©¦      PA12/PA13      ©¦                     ©¦  PA12/PA13/PA14/PA15 ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦  USART2   ©¦       PD2/PD3       ©¦      PA14/PA15      ©¦   PD2/PD3/PD1/PD0    ©¦
  * ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  *
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_system.h"
#include "tx_phe_ll_usart.h"

/** @addtogroup USART_LL_Driver USART LL Driver
  * @ingroup  usart_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup USART_LL_Interrupt USART LL Interrupt Handle function
  * @ingroup  USART_LL_Driver
  * @brief    USART LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup USART_LL_Inti_Cfg USART LL Initialization And Configuration
  * @ingroup  USART_LL_Driver
  * @brief    USART LL Initialization And Configuration
  * @{
  */
  
/** 
  * @brief  Low layer USART initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_init : USART initialization struct
  * @retval None.
  */
void ll_usart_init(USART_TypeDef *p_usart, TYPE_LL_USART_INIT *p_init)
{
    /* The system resets the USART module, Enable the clock of the module. */
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));
}

/** 
  * @brief  Low layer USART detele initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval None.
  */
void ll_usart_deinit(USART_TypeDef *p_usart)
{
    /* The system disable the USART module, includes turning off the clock for the module. */
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(USART0 == p_usart) {
        SYSCTRL->IO_MAP &= ~(BIT(4) | BIT(14));
    } else if(USART1 == p_usart) {
        SYSCTRL->IO_MAP &= ~(BIT(5) | BIT(15));
    } else if(USART2 == p_usart) {
        SYSCTRL->IO_MAP &= ~(BIT(6) | BIT(16) | BIT(24));
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/** 
  * @brief  Low layer USART interrupt initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart_irq_config(USART_TypeDef *p_usart, TYPE_LL_USART_IRQ_CFG *p_cfg)
{
    u32 usart_ier = 0;
    
    usart_ier |= p_cfg->program_thre_intr_en ? LL_USARTX_IER_PTIME : 0;
    usart_ier |= p_cfg->modem_status_intr_en ? LL_USARTX_IER_EDSSI : 0;
    usart_ier |= p_cfg->rx_line_status_en ? LL_USARTX_IER_ELSI : 0;
    usart_ier |= p_cfg->tx_empty_intr_en ? LL_USARTX_IER_ETBEI : 0;
    usart_ier |= p_cfg->rx_ready_intr_en ? LL_USARTX_IER_ERBFI : 0;
    
    p_usart->IER = LL_USARTX_IER_MASK(usart_ier);
}

/** 
  * @brief  Low layer USART 232 io mapping initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart232_io_map(USART_TypeDef *p_usart, TYPE_LL_USART_CFG *p_cfg)
{
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));
    
    if((USART0 == p_usart) || (USART1 == p_usart)) {
        // USART0 and USART1 can only be set to LL_TMR_IO_MAP1.
        TX_ASSERT(p_cfg->io_map < LL_USART_IO_MAP1);
    }
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    /* USART0 IO_MAP */
    if(USART0 == p_usart) {
        /* USART0 map0 */
        SYSCTRL->IO_MAP &= ~BIT(14);
        SYSCTRL->IO_MAP |= BIT(4);
        
    /* USART1 IO_MAP */
    } else if(USART1 == p_usart) {
        /* USART1 map0 */
        SYSCTRL->IO_MAP &= ~BIT(15);
        SYSCTRL->IO_MAP |= BIT(5);
        
    /* USART2 IO_MAP */
    } else if(USART2 == p_usart) {
        //clear all io map of USART2
        SYSCTRL->IO_MAP &= ~(BIT(6) | BIT(16) | BIT(24));

        /* USART2 map0 */
        if(p_cfg->io_map & LL_USART_IO_MAP0) {
            SYSCTRL->IO_MAP |= BIT(6);
        /* USART2 map1 */
        } else if(p_cfg->io_map & LL_USART_IO_MAP1) {
            SYSCTRL->IO_MAP |= BIT(24);
        }
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/** 
  * @brief  Low layer USART 485 io mapping initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval None.
  */
void ll_usart485_io_map(USART_TypeDef *p_usart)
{
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    /* USART0 IO_MAP */
    if(USART0 == p_usart) {
        SYSCTRL->IO_MAP &= ~BIT(4);
        SYSCTRL->IO_MAP |= BIT(14);
        
    /* USART1 IO_MAP */
    } else if(USART1 == p_usart) {
        //clear all io map of USART1
        SYSCTRL->IO_MAP &= ~BIT(5);
        SYSCTRL->IO_MAP |= BIT(15);
        
    /* USART2 IO_MAP */
    } else if(USART2 == p_usart) {
        //clear all io map of USART2
        SYSCTRL->IO_MAP &= ~(BIT(6) | BIT(24));
        SYSCTRL->IO_MAP |= BIT(16);
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}
  
/** 
  * @brief  Low layer USART 232 initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart232_config(USART_TypeDef *p_usart, TYPE_LL_USART_CFG *p_cfg)
{
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));

    u32 lcr = 0;
    u32 fcr = 0;

    ll_usart232_io_map(p_usart, p_cfg);

    lcr |= LL_USARTX_LCR_DLS(p_cfg->bit_width_sel);
    lcr |= p_cfg->stop_bit_sel ? LL_USARTX_LCR_STOP : 0;
    lcr |= LL_USARTX_LCR_PARITY(p_cfg->parity);
    if(p_cfg->stick_parity == LL_USART_STICK_PARITY_EN) {
        lcr |= LL_USARTX_LCR_SP | LL_USARTX_LCR_PEN;
    }

    fcr |= LL_USARTX_FCR_FIFOE | LL_USARTX_FCR_RFIFOR | LL_USARTX_FCR_XFIFOR;
    fcr |= LL_USARTX_FCR_TET(p_cfg->txfifo_trigger);
    fcr |= LL_USARTX_FCR_RCVR(p_cfg->rxfifo_trigger);
    
    p_usart->MCR  = 0x0;
    p_usart->LCR  = LL_USARTX_LCR_DLAB;
    p_usart->DLL  = LL_USARTX_DLL(p_cfg->baudrate);
    p_usart->DLH  = LL_USARTX_DLH(p_cfg->baudrate >> 8);
    p_usart->LCR &= ~LL_USARTX_LCR_DLAB;
    p_usart->LCR  = lcr;
    p_usart->FCR  = fcr;
}

/** 
  * @brief  Low layer USART 485 initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart485_config(USART_TypeDef *p_usart, TYPE_LL_RS485_CFG *p_cfg)
{
    TX_ASSERT((p_usart == USART0) || (p_usart == USART1) || (p_usart == USART2));
                
    ll_usart232_config(p_usart, &p_cfg->usart_cfg);
    
    ll_usart485_io_map(p_usart);

    p_usart->DET   = (u32)(p_cfg->de_deassertion_time << 16) | (p_cfg->de_assertion_time << 0);
    p_usart->TAT   = (u32)(p_cfg->re2de_turnaround_time << 16) | p_cfg->de2re_turnaround_time;

    p_usart->DE_EN = p_cfg->de_sig_en ? 1 : 0;
    p_usart->RE_EN = p_cfg->re_sig_en ? 1 : 0;

    p_usart->TCR   = (p_usart->TCR & ~0x3)                             |
                      LL_USARTX_TCR_RE_POL(p_cfg->re_sig_active_level) |
                      LL_USARTX_TCR_DE_POL(p_cfg->de_sig_active_level);
                   
    p_usart->TCR   = (p_usart->TCR & ~(LL_USARTX_TCR_XFER_MODE(3))) |
                      LL_USARTX_TCR_XFER_MODE(p_cfg->rs485_mode);

    p_usart->TCR  |= LL_USARTX_TCR_RS485_EN;
}

/**
  * @}
  */

/** @defgroup USART_LL_Data_Transfers USART LL Data transfers functions
  * @ingroup  USART_LL_Driver
  * @brief    USART LL Data transfers functions 
  * @{
  */



/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
