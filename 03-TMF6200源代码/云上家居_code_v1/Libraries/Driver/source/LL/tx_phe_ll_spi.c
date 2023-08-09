/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_spi.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the SPI LL firmware functions.
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
#include "tx_phe_ll_spi.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup spi_interface_gr SPI Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup SPI_LL_Driver SPI LL Driver
  * @ingroup  spi_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup SPI_LL_Interrupt SPI LL Interrupt Handle function
  * @ingroup  SPI_LL_Driver
  * @brief   SPI LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup SPI_LL_Inti_Cfg SPI LL Initialization And Configuration
  * @ingroup  SPI_LL_Driver
  * @brief    SPI LL Initialization And Configuration
  * @{
  */

/** 
  * @brief  Low layer SPI initialization function
  * @param  p_spi : SPI module pointer
  * @param  p_init: SPI initialization struct pointer
  * @retval None.
  */
void ll_spi_init(SPI_TypeDef *p_spi, TYPE_LL_SPI_INIT *p_init)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    
    /* reset */
    p_spi->CFG = 0x00000000;
    p_spi->CTL = 0x00000008;            //disable spi
    p_spi->STA = 0xFFFFFFFF;            //clear all pending
    
    /* IO initialization and mapping */
    ll_spi_io_map(p_spi, p_init->io_map);

    p_spi->CFG = LL_SPI_CFG_TDRQ_EN | LL_SPI_CFG_RDRQ_EN          |
                 LL_SPI_CFG_TX_FIFO_EN | LL_SPI_CFG_RX_FIFO_EN    |
                 LL_SPI_CFG_DIV_CNT(p_init->clk_div_cnt)          |
                 LL_SPI_CFG_DFS_SET(p_init->frame_size)           |
                 LL_SPI_CFG_SPI_MODE_SET(p_init->spi_mode)        |
                 LL_SPI_CFG_WIRE_MODE_SET(p_init->wire_mode)      |
                 (p_init->slave_mode_en ? LL_SPI_CFG_SLAVE_EN : 0);
}

/**
  * @brief  Low layer SPI io map init function
  * @param  p_spi : SPI module pointer
  * @param  io_map: SPI module IO mapping
  * @retval None
  */
void ll_spi_io_map(SPI_TypeDef *p_spi, TYPE_ENUM_LL_SPI_IO_MAP io_map)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    /* SPI IO map */
    if(p_spi == SPI0) {
        switch(io_map) {
            /* PA0 to PA5 */
            case LL_SPI0_IOMAP0:
                SYSCTRL->IO_MAP   |= BIT(2);
                SYSCTRL->IO_MAP   &= ~BIT(20);
                SYSCTRL->SYS_CON0 &= ~(BIT(24) | BIT(25));
                break;
            
            /* PA0 to PA3 */
            case LL_SPI0_IOMAP1:
                SYSCTRL->IO_MAP   |= BIT(20);
                SYSCTRL->IO_MAP   &= ~BIT(2);
                SYSCTRL->SYS_CON0 &= ~(BIT(24) | BIT(25));
                break;
            
            /* PA0/PD1/PD0/PA1 */
            case LL_SPI0_MCP_IOMAP0:
                SYSCTRL->SYS_CON0 |= BIT(24);
                SYSCTRL->IO_MAP   &= ~(BIT(2) | BIT(20));
                SYSCTRL->SYS_CON0 &= ~BIT(25);
                break;
            
            /* PA0/PD1/PD0/PA1/PA2/PD2 */
            case LL_SPI0_MCP_IOMAP1:
                SYSCTRL->SYS_CON0 |= BIT(25);
                SYSCTRL->IO_MAP   &= ~(BIT(2) | BIT(20));
                SYSCTRL->SYS_CON0 &= ~BIT(24);
                break;
            
            /* not exist */
            default:
                TX_ASSERT(0);
                break;
        }
    } else if(p_spi == SPI1) {
        switch(io_map) {
            /* PD0 to PD1 */
            case LL_SPI1_IOMAP0:
                SYSCTRL->IO_MAP |= BIT(3);
                SYSCTRL->IO_MAP &= ~BIT(21);
                break;
            
            /* PD0 to PD3 */
            case LL_SPI1_IOMAP1:
                SYSCTRL->IO_MAP |= BIT(21);
                SYSCTRL->IO_MAP &= ~BIT(3);
                break;
            
            /* not exist */
            default:
                TX_ASSERT(0);
                break;
        }
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
}

/**
  * @brief  SPI module detele initialization function
  * @param  p_spi: Select the initialized SPI module pointer
  * @retval None
  */
void ll_spi_deinit(SPI_TypeDef *p_spi)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    if(p_spi == SPI0) {
        SYSCTRL->IO_MAP   &= ~(BIT(2) | BIT(20));
        SYSCTRL->SYS_CON0 &= ~(BIT(24) | BIT(25));
    } else if(p_spi == SPI1) {
        SYSCTRL->IO_MAP &= ~(BIT(3) | BIT(21));
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();
    
    p_spi->CTL = 0x00000008;
    p_spi->CFG = 0x00000000;
    p_spi->STA = 0xFFFFFFFF;
}


/** 
  * @brief  Low layer SPI interrupt config function
  * @param  p_spi: SPI module pointer
  * @param  p_cfg: SPI Configuration struct pointer
  * @retval None.
  */
void ll_spi_irq_config(SPI_TypeDef *p_spi, TYPE_LL_SPI_IRQ_CFG *p_cfg)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    
    u32 reg = p_spi->CFG;
    
    reg &= ~(LL_SPI_CFG_WIRE_MODE_MASK | 
             LL_SPI_CFG_END_IRQ_EN     |
             LL_SPI_CFG_TIRQ_EN        | 
             LL_SPI_CFG_RIRQ_EN);
    reg |= (p_cfg->slave_cs_rising_intr_en ? LL_SPI_CFG_END_IRQ_EN : 0) |
           (p_cfg->tx_intr_en ? LL_SPI_CFG_TIRQ_EN : 0)                 |
           (p_cfg->rx_intr_en ? LL_SPI_CFG_RIRQ_EN : 0);
    
    p_spi->CFG = reg;
}

/**
  * @}
  */

/** @defgroup SPI_LL_Data_Transfers SPI LL Data transfers functions
  * @ingroup  SPI_LL_Driver
  * @brief    SPI LL Data transfers functions 
  * @{
  */

/**
  * @brief  The SPI module gets the current wire mode
  * @param  p_spi: SPI module pointer
  * @retval None
  */
TYPE_ENUM_LL_SPI_WIRE_MODE ll_spi_get_wire_mode(SPI_TypeDef *p_spi)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    
    return (TYPE_ENUM_LL_SPI_WIRE_MODE)(LL_SPI_CFG_WIRE_MODE_GET(p_spi->CFG));
}

/**
  * @brief  The SPI module set the current wire mode
  * @param  p_spi: SPI module pointer
  * @retval None
  */
void ll_spi_set_wire_mode(SPI_TypeDef *p_spi, TYPE_ENUM_LL_SPI_WIRE_MODE wire_mode_sel)
{
    TX_ASSERT((p_spi == SPI0) || (p_spi == SPI1));
    u32 spi_cfg;
    
    spi_cfg    = p_spi->CFG & (~LL_SPI_CFG_WIRE_MODE_MASK);
    p_spi->CFG = spi_cfg | LL_SPI_CFG_WIRE_MODE_SET(wire_mode_sel);
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
