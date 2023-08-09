/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_cc_rst.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the CC_RST LL firmware functions.
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
#ifndef __TX_PHE_LL_CC_RST_H
#define __TX_PHE_LL_CC_RST_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"

     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup cc_rst_interface_gr CC_RST Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 


/** globle clock trees
 *
 *                      WDT 
 *                       |
 *  RC32KHZ--------------|----------->|                       |---->APB0
 *                                    |-->SYS_CLK---->FCLK---->|
 *                      SYSPLL------->|                       |---->APB1|
 *                       ||           ||
 *  HXOSC HZ-------------||---------->||
 *                       ||           ||
 *  RC8MHZ-------------- ||---------->||
 *                       ||            |
 *                      ADCPLL-------->|-->EPWMCLK
 *                       |
 *                      FADC
 *
 */
 

/** @addtogroup CC_RST_LL_Driver CC_RST LL Driver
  * @ingroup  cc_rst_interface_gr
  * @brief Mainly the driver part of the CC_RST module, which includes \b CC_RST \b Register 
  * \b Constants, \b CC_RST \b Exported \b Constants, \b CC_RST \b Exported \b Struct, \b CC_RST
  * \b Data \b transfers \b functions, \b CC_RST \b Initialization \b and \b CC_RST \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
	 
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
	 
/** @defgroup CC_RST_LL_Register_Constants CC_RST LL Register Constants
  * @ingroup  CC_RST_LL_Driver
  * @brief    CC_RST LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the CC_RST 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the CC_RST register, mainly for convenience. Understand the 
    configuration of the CC_RST.
    
@endverbatim
  *
  * @{
  */

/***** SYS_CON0 *****/
/*! test out clock enable
 */
#define LL_CC_SYSCON0_TEST_OUT_EN                           (1UL << 3)

/***** SYS_CON1 *****/

/***** AIP_CON0 *****/
/*! The SYSPLL analog section is enabled.
 */
#define LL_CC_AIPCON0_SYSPLL_EN                             (1UL << 0)
/*! SYSPLL center frequency setting.
 */
#define LL_CC_AIPCON0_SYSPLL_BAND_SET(n)                    (((n)&0x03) << 1)
#define LL_CC_AIPCON0_SYSPLL_BAND_GET(n)                    (((n) >> 1) & 0x03)
/*! SYSPLL CP current setting.
 */
#define LL_CC_AIPCON0_SYSPLL_GCP_SET(n)                     (((n)&0x03) << 3)
/*! SYSPLL VCO frequency adjustment gain control.
 */
#define LL_CC_AIPCON0_SYSPLL_GVCO_SET(n)                    (((n)&0x03) << 5)
/*! SYSPLL test signal output selection.
 */
#define LL_CC_AIPCON0_SYSPLL_TEN_SET(n)                     (((n)&0x0F) << 8)
/*! The ADCPLL analog section is enabled.
 */
#define LL_CC_AIPCON0_ADCPLL_EN                             (1UL << 12)
/*! ADCPLL center frequency setting.
 */
#define LL_CC_AIPCON0_ADCPLL_BAND_SET(n)                    (((n)&0x03) << 13)
#define LL_CC_AIPCON0_ADCPLL_BAND_GET(n)                    (((n) >> 13) & 0x03)
/*! ADCPLL CP current setting.
 */
#define LL_CC_AIPCON0_ADCPLL_GCP_SET(n)                     (((n)&0x03) << 15)
/*! ADCPLL VCO frequency adjustment gain control.
 */
#define LL_CC_AIPCON0_ADCPLL_GVCO_SET(n)                    (((n)&0x03) << 17)
/*! ADCPLL test signal output selection.
 */
#define LL_CC_AIPCON0_ADCPLL_TEN_SET(n)                     (((n)&0x0F) << 20)
/*! SYSPLL reference clock selection
 */
#define LL_CC_AIPCON0_SYSPLL_LPFSEL_RC8M                    (1UL << 24)
/*! ADCPLL reference clock selection
 */
#define LL_CC_AIPCON0_ADCPLL_LPFSEL_RC8M                    (1UL << 26)

/***** AIP_CON1 *****/
/*! HXOSC enable
 */
#define LL_CC_AIPCON1_HXOSC_EN                              (1UL << 0)
/*! HXOSC resonant capacitor coarse adjustment
 */
#define LL_CC_AIPCON1_HXOSC_CS_SET(n)                       (((n)&0x07) << 1)
/*! HXOSC HXI port resonant capacitor fine tuning
 */
#define LL_CC_AIPCON1_HXOSC_CTI_SET(n)                      (((n)&0x07) << 4)
/*! HXOSC HXO port resonant capacitor fine tuning
 */
#define LL_CC_AIPCON1_HXOSC_CTO_SET(n)                      (((n)&0x07) << 8)
/*! HXOSC starting current setting
 */
#define LL_CC_AIPCON1_HXOSC_DR_SET(n)                       (((n)&0x07) << 12)
/*! Hysteresis enable for the HXOSC comparator
 */
#define LL_CC_AIPCON1_HXOSC_HY_EN                           (1UL << 15)
/*! RC 8M enable
 */
#define LL_CC_AIPCON1_RC8M_EN                               (1UL << 16)

/***** CLK_CON0 *****/
/*! system clock source select , reference to TYPE_ENUM_LL_CC_RST_SYSCLK_SRC
 */
#define LL_CC_RST_SYSCLK_SRC_SEL(n)                         (((n)&0x3) << 0)
#define LL_CC_RST_SYSCLK_SRC_GET(n)                         (((n) >> 0) & 0x03)
/*! Test clock output selection
 */
#define LL_CC_CLKCON0_TEST_CLK_SEL(n)                       (((n)&0x7) << 4)
#define LL_CC_CLKCON0_TEST_CLK_SEL_MASK                     (0x07UL << 4)
/*! system pll clock source select , reference to TYPE_ENUM_LL_CC_RST_EPWM_CLK_SRC
 */
#define LL_CC_RST_EPWM_CLK_SRC_SEL(n)                       (((n)&0x3) << 7)

/*! system pll clock source select , reference to TYPE_ENUM_LL_CC_RST_PLLCLK_SRC
 */
#define LL_CC_RST_SYSPLL_CLK_SRC_SEL(n)                     (((n)&0x3) << 11)
#define LL_CC_RST_SYSPLL_CLK_SRC_GET(n)                     (((n) >> 11) & 0x03)
/*! adc pll clock source select , reference to TYPE_ENUM_LL_CC_RST_PLLCLK_SRC
 */
#define LL_CC_RST_ADCPLL_CLK_SRC_SEL(n)                     (((n)&0x3) << 15)
#define LL_CC_RST_ADCPLL_CLK_SRC_GET(n)                     (((n) >> 15) & 0x03)

/*! gpio debunce clock source select , reference to TYPE_ENUM_LL_CC_RST_GPIO_DEBCLK_SRC
 */
#define LL_CC_RST_GPIOA_DEB_CLK_SRC_SEL(n)                  (((n)&0x3) << 17)
/*! gpio debunce clock source select , reference to TYPE_ENUM_LL_CC_RST_GPIO_DEBCLK_SRC
 */
#define LL_CC_RST_GPIOB_DEB_CLK_SRC_SEL(n)                  (((n)&0x3) << 19)
/*! gpio debunce clock source select , reference to TYPE_ENUM_LL_CC_RST_GPIO_DEBCLK_SRC
 */
#define LL_CC_RST_GPIOC_DEB_CLK_SRC_SEL(n)                  (((n)&0x3) << 21)
/*! gpio debunce clock source select , reference to TYPE_ENUM_LL_CC_RST_GPIO_DEBCLK_SRC
 */
#define LL_CC_RST_GPIOD_DEB_CLK_SRC_SEL(n)                  (((n)&0x3) << 23)


/***** CLK_CON1 *****/
/*! system clock divor , fclk/ahb_clk = sysclk/(divor + 1)
 */
#define LL_CC_RST_SYSCLK_DIV(n)                             (((n)&0xFF) << 0)
/*! apb0 clock divor , apb_clk = ahb_clk/(divor + 1)
 */
#define LL_CC_RST_APB0CLK_DIV(n)                            (((n)&0xFF) << 8)
/*! apb1 clock divor , apb_clk = ahb_clk/(divor + 1)
 */
#define LL_CC_RST_APB1CLK_DIV(n)                            (((n)&0xFF) << 16)


/***** CLK_CON2 *****/
/*! EPWM clock divor , epwm_clk = epwm_src_clk/(divor + 1)
 */
#define LL_CC_RST_EPWMCLK_DIV(n)                            (((n)&0xF) << 12)
/*! ADCPLL clock divor , adcpll_clk = adcpll_src_clk/(divor + 1)
 */
#define LL_CC_RST_ADCPLLCLK_DIV(n)                          (((n)&0xF) << 8)
/*! ADC clock divor , adc_clk = adcpll_clk/(divor + 1)
 */
#define LL_CC_RST_ADCCLK_DIV(n)                             (((n)&0xF) << 4)
/*! SYSPLL clock divor , syspll_clk = syspll_src_clk/(divor + 1)
 */
#define LL_CC_RST_SYSPLLCLK_DIV(n)                          (((n)&0xF) << 0)
#define LL_CC_RST_SYSPLLCLK_DIV_GET(n)                      (((n) >> 0) & 0x0F)


/***** CLK_CON3 *****/
/*! GPIO debunce clock divor , gpio_debunce_clk = gpio_src_clk/(divor + 1)
 */
#define LL_CC_RST_GPIOD_DEBCLK_DIV(n)                       (((n)&0xF) << 24)
/*! GPIO debunce clock divor , gpio_debunce_clk = gpio_src_clk/(divor + 1)
 */
#define LL_CC_RST_GPIOC_DEBCLK_DIV(n)                       (((n)&0xF) << 16)
/*! GPIO debunce clock divor , gpio_debunce_clk = gpio_src_clk/(divor + 1)
 */
#define LL_CC_RST_GPIOB_DEBCLK_DIV(n)                       (((n)&0xF) << 8)
/*! GPIO debunce clock divor , gpio_debunce_clk = gpio_src_clk/(divor + 1)
 */
#define LL_CC_RST_GPIOA_DEBCLK_DIV(n)                       (((n)&0xF) << 0)


/**
  * @}
  */

/** @defgroup CC_RST_LL_Exported_Constants CC_RST LL Exported Constants
  * @ingroup  CC_RST_LL_Driver
  * @brief    CC_RST LL external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the abstraction 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
  
/***** DRIVER API *****/



/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief define the PLL reference clock source
  */
typedef enum {
    /*! HXOSC.
     */
    LL_CC_RST_PLL_REFCLK_HXOSC   = 0,
    /*! RC8M.
     */
    LL_CC_RST_PLL_REFCLK_RC8M     = 1,
} TYPE_ENUM_LL_CC_RST_PLL_REFCLK;

/**
  * @brief define the PLL center frequnce
  */
typedef enum {
    /*! 312MHZ
     */
    LL_CC_RST_PLL_FREQ_312DIV4MHZ   = 0,
    /*! 396MHZ
     */
    LL_CC_RST_PLL_FREQ_396DIV4MHZ   = 1,
    /*! 466MHZ
     */
    LL_CC_RST_PLL_FREQ_466DIV4MHZ   = 2,
    /*! 520MHZ
     */
    LL_CC_RST_PLL_FREQ_520DIV4MHZ   = 3,
} TYPE_ENUM_LL_CC_RST_PLL_FREQ;

/**
  * @brief define the epwm clock source
  */
typedef enum {
    /*! RC8M.
     */
    LL_CC_RST_EPWM_CLK_RC8M     = 0,
    /*! HXOSC.
     */
    LL_CC_RST_EPWM_CLK_HXOSC    = 1,
    /*! SYSPLL
     */
    LL_CC_RST_EPWM_CLK_SYSPLL   = 2,
    /*! ADCPLL
     */
    LL_CC_RST_EPWM_CLK_ADCPLL   = 3,

} TYPE_ENUM_LL_CC_RST_EPWM_CLK_SRC;

/**
  * @brief define the pll clock source
  */
typedef enum {
    /*! OSC 26Mhz.
     */
    LL_CC_RST_PLLCLK_SRC_HXOSC     = 0,
    /*! RC 8Mhz.
     */
    LL_CC_RST_PLLCLK_SRC_RC8M      = 1,
    /*! no clock
     */
    LL_CC_RST_PLLCLK_SRC_NONE      = 2,
    /*! dft clock input
     */
    LL_CC_RST_PLLCLK_SRC_DFTCLK    = 3,

} TYPE_ENUM_LL_CC_RST_PLLCLK_SRC;

/**
  * @brief define the system clock source
  */
typedef enum {
    /*! RC32Khz.
     */
    LL_CC_RST_SYSCLK_SRC_RC32K          = 0,
    /*! OSC 26Mhz.
     */
    LL_CC_RST_SYSCLK_SRC_HXOSC          = 1,
    /*! PLL DIV
     */
    LL_CC_RST_SYSCLK_SRC_SYSPLL_DIV     = 2,
    /*! RC 8Mhz.
     */
    LL_CC_RST_SYSCLK_SRC_RC8M           = 3,

} TYPE_ENUM_LL_CC_RST_SYSCLK_SRC;

/**
  * @brief define the gpio debunce clock source
  */
typedef enum {
    /*! HXOSC.
     */
    LL_CC_RST_GPIO_DEBCLK_SRC_HXOSC    = 0,
    /*! RC8M.
     */
    LL_CC_RST_GPIO_DEBCLK_SRC_RC8M     = 1,
    /*! CPU CLK
     */
    LL_CC_RST_GPIO_DEBCLK_SRC_CPUCLK   = 1,
    /*! RC32K
     */
    LL_CC_RST_GPIO_DEBCLK_SRC_RC32K    = 3,
} TYPE_ENUM_LL_CC_RST_GPIO_DEBCLK_SRC;


/**
  * @brief define the APB0 peripheral modules
  */
typedef enum {
    /*! APB0 SPI0
     */
    LL_CC_RST_APB0_M_SPI0    = (27),
    /*! APB0 SPI1
     */
    LL_CC_RST_APB0_M_SPI1    = (26),
    /*! APB0 UART0
     */
    LL_CC_RST_APB0_M_UART0   = (23),
    /*! APB0 UART1
     */
    LL_CC_RST_APB0_M_UART1   = (22),
    /*! APB0 UART2
     */
    LL_CC_RST_APB0_M_UART2   = (21),
    /*! APB0 IIC0
     */
    LL_CC_RST_APB0_M_IIC0    = (20),
    /*! APB0 IIC1
     */
    LL_CC_RST_APB0_M_IIC1    = (19),
    /*! APB0 TMR1
     */
    LL_CC_RST_APB0_M_TMR1    = (12),

} TYPE_ENUM_LL_CC_RST_APB0_M;
    
/**
  * @brief define the APB1 peripheral modules
  */
typedef enum {
    /*! APB1 TMR0
     */
    LL_CC_RST_APB1_M_TMR0    = (11),
    /*! APB1 QEI
     */
    LL_CC_RST_APB1_M_QEI     = (15),
    /*! APB1 CRC
     */
    LL_CC_RST_APB1_M_CRC     = (17),

} TYPE_ENUM_LL_CC_RST_APB1_M;

/**
  * @brief define the APB2 peripheral modules
  */
typedef enum {
    /*! APB2 EFLASH , CLKCON4
     */
    LL_CC_RST_APB2_M_EFLASH  = (1),
    /*! APB2 SARADC, CLKCON4
     */
    LL_CC_RST_APB2_M_SARADC  = (4),
    /*! APB2 HCC, CLKCON5
     */
    LL_CC_RST_APB2_M_HCC     = (9),

} TYPE_ENUM_LL_CC_RST_APB2_M;

/**
  * @brief define the AHB0 peripheral modules
  */
typedef enum {
    /*! AHB0 GPIOA , CLKCON4
     */
    LL_CC_RST_AHB0_M_GPIOA  = (31),
    /*! AHB0 GPIOB , CLKCON4
     */
    LL_CC_RST_AHB0_M_GPIOB  = (30),
    /*! AHB0 GPIOC , CLKCON4
     */
    LL_CC_RST_AHB0_M_GPIOC  = (29),
    /*! AHB0 GPIOD , CLKCON4
     */
    LL_CC_RST_AHB0_M_GPIOD  = (28),
    /*! AHB0 DAC , CLKCON4
     */
    LL_CC_RST_AHB0_M_DAC    = (3),
    /*! AHB0 CAN , CLKCON4
     */
    LL_CC_RST_AHB0_M_CAN    = (13),
    
    /*! AHB0 DMAC , CLKCON5
     */
    LL_CC_RST_AHB0_M_DMAC   = (0),
    /*! AHB0 GMAC , CLKCON5
     */
    LL_CC_RST_AHB0_M_GMAC   = (8),
    /*! AHB0 ROM , CLKCON5
     */
    LL_CC_RST_AHB0_M_ROM    = (10),
    /*! AHB0 SRAM0 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM0  = (11),
    /*! AHB0 SRAM1 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM1  = (12),
    /*! AHB0 SRAM2 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM2  = (13),
    /*! AHB0 SRAM3 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM3  = (14),
    /*! AHB0 SRAM4 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM4  = (15),
    /*! AHB0 SRAM5 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM5  = (16),
    /*! AHB0 SRAM6 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM6  = (17),
    /*! AHB0 SRAM7 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM7  = (18),
    /*! AHB0 SRAM8 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM8  = (19),
    /*! AHB0 SRAM9 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM9  = (20),
    /*! AHB0 SRAM10 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM10 = (21),
    /*! AHB0 SRAM11 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM11 = (22),
    /*! AHB0 SRAM12 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM12 = (23),
    /*! AHB0 SRAM13 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM13 = (24),
    /*! AHB0 SRAM14 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM14 = (25),
    /*! AHB0 SRAM15 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM15 = (26),
    /*! AHB0 SRAM16 , CLKCON5
     */
    LL_CC_RST_AHB0_M_SRAM16 = (27),
    /*! AHB0 FADC , CLKCON5
     */
    LL_CC_RST_AHB0_M_FADC   = (28),

} TYPE_ENUM_LL_CC_RST_AHB0_M;

/**
  * @brief define the peripheral modules for soft reset
  */
typedef enum {
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_FPLL0  = (0),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_FPLL1  = (1),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_AHB0   = (2),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_AHB1   = (3),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_AHB2   = (4),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_APB0   = (5),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_APB1   = (6),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_EFLASH = (7),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_DMAC   = (8),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_IIC0   = (9),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_IIC1   = (10),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_SPI0   = (11),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_SPI1   = (12),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_UART0  = (13),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_UART1  = (14),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_UART2  = (15),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_ADVTMR = (16),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_PWRACE = (17),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_SARADC = (18),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_HCC    = (19),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_CRC    = (21),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_TMR    = (22),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_WDT    = (23),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_GPIO   = (24),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_CAN    = (25),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_EVSYS  = (26),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_QEI    = (27),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_GMAC   = (28),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_FADC   = (29),
    /*! soft reset peripheral
     */
    LL_CC_RST_SOFTRST_M_EPWM   = (30),

} TYPE_ENUM_LL_CC_RST_SOFTRST_M;

/**
  * @brief Test clock enumeration type
  */
typedef enum {
    LL_CC_TEST_OUT_OSC_26M          = 0x00,
    LL_CC_TEST_OUT_RC32K            = 0x01,
    LL_CC_TEST_OUT_SYSPLL_DIV_CLK   = 0x02,
    LL_CC_TEST_OUT_SYSPLL_DIV32_CLK = 0x03,
    LL_CC_TEST_OUT_RC8M             = 0x04,
    LL_CC_TEST_OUT_ADCPLL_DIV_CLK   = 0x06,
    LL_CC_TEST_OUT_ADCPLL_DIV32_CLK = 0x07,
} TYPE_ENUM_LL_CC_TEST_OUT;

/**
  * @}
  */

/** @defgroup CC_RST_LL_Exported_Struct CC_RST LL Exported Struct
  * @ingroup  CC_RST_LL_Driver
  * @brief    CC_RST LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the CC_RST registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_cc_rst_init. Function, you can configure the CC_RST module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */


/**
  * @}
  */

/** @defgroup CC_RST_LL_Interrupt CC_RST LL Interrupt Handle function
  * @brief   CC_RST LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CC_RST  
    Interrupt Handle function.

    how to use?

    The CC_RST interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the CC_RST in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */

/**
  * @brief System clock tree divider structure type
  */
typedef struct {
    /*! System clock division control
     *  @note system clock = SYS_CLK / (sys_clk_div+1);  
     *        The sys_clk_div must be less than 0xFF.
     */
    u8 sys_clk_div;
    /*! APB0 clock division control
     *  @note APB0 clock = system clock / (apb0_clk_div+1);  
     *        The apb0_clk_div must be less than 0xFF.
     */
    u8 apb0_clk_div;
    /*! APB1 clock division control
     *  @note APB1 clock = system clock / (apb1_clk_div+1);  
     *        The apb1_clk_div must be less than 0xFF.
     */
    u8 apb1_clk_div;
} TYPE_LL_CC_DIV_CFG;

/**
  * @}
  */
  
/** @defgroup CC_RST_LL_Inti_Cfg CC_RST LL Initialization And Configuration
  * @brief    CC_RST LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CC_RST data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Set the ADCPLL clock frequency.
  * @param  freq        : ADCPLL frequency
  * @param  pll_clk_src : the reference clock source for the ADCPLL
  * @retval None
  */
void ll_cc_adcpll_set(u32 freq, TYPE_ENUM_LL_CC_RST_PLLCLK_SRC pll_clk_src);

/**
  * @brief  Set SYSPLL
  * @param  sys_max_clk : maximum system clock frequency
  * @param  pll_clk_src : the reference clock source for SYSPLL
  * @retval None
  */
void ll_cc_syspll_set(u32 sys_max_clk, TYPE_ENUM_LL_CC_RST_PLLCLK_SRC pll_clk_src);

/**
  * @brief  SYS_CLK source config
  * @param  p_cfg : SYS_CLK configure structure pointer
  * @retval None
  */
void ll_cc_sysclk_select(TYPE_ENUM_LL_CC_RST_SYSCLK_SRC src);

/**
  * @brief  System clock tree divider setting
  * @param  p_cfg : Clock divider structure pointer
  * @retval None
  */
void ll_cc_clk_div_config(TYPE_LL_CC_DIV_CFG *p_cfg);

/**
  * @brief  Get the real system clock frequency
  * @retval Returns the system clock frequency in Hz
  */
u32 ll_cc_sys_clk_get(void);

/**
  * @brief  Get the oscillation frequency of the ADCPLL
  * @retval Returns the oscillation frequency of the ADCPLL in Hz
  */
u32 ll_cc_adc_pll_get(void);

/**
  * @brief  Test clock output function
  * @param  clk_sel : Select the clock to output
  * @retval None
  * @note   The clock will be output through PC0
  */
void ll_cc_test_clk_out(TYPE_ENUM_LL_CC_TEST_OUT clk_sel);

/**
  * @}
  */
  
/** @defgroup CC_RST_LL_Data_Transfers CC_RST LL Data transfers functions
  * @brief    CC_RST LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CC_RST data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  CC_RST soft reset peripheral 
  * @param  periph : TYPE_ENUM_LL_CC_RST_SOFTRST_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_softreset(TYPE_ENUM_LL_CC_RST_SOFTRST_M  periph) {
    SYSCTRL_REG_OPT(SYSCTRL->SYS_CON2 &= ~(BIT(periph)););
}

/**
  * @brief  CC_RST release soft reset peripheral 
  * @param  periph : TYPE_ENUM_LL_CC_RST_SOFTRST_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_softreset_release(TYPE_ENUM_LL_CC_RST_SOFTRST_M  periph) {
    SYSCTRL_REG_OPT(SYSCTRL->SYS_CON2 |= BIT(periph););
}

/**
  * @brief  CC_RST enable AHB0/APB0 CLK.
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahpb0_clk_enable(void) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 |= BIT(3););
}

/**
  * @brief  CC_RST disable AHB0/APB0 CLK.
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahpb0_clk_disable(void) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 &= ~(BIT(3)););
}

/**
  * @brief  CC_RST enable AHB1/APB1 CLK.
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahpb1_clk_enable(void) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 |= BIT(2););
}

/**
  * @brief  CC_RST disable AHB1/APB1 CLK.
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahpb1_clk_disable(void) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 &= ~(BIT(2)););
}

/**
  * @brief  CC_RST enable APB0 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB0_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb0_peripheral_clk_enable(TYPE_ENUM_LL_CC_RST_APB0_M periph) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 |= (BIT(periph)););
}

/**
  * @brief  CC_RST disable APB0 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB0_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb0_peripheral_clk_disable(TYPE_ENUM_LL_CC_RST_APB0_M periph) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 &= ~(BIT(periph)););
}

/**
  * @brief  CC_RST enable APB1 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB1_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb1_peripheral_clk_enable(TYPE_ENUM_LL_CC_RST_APB1_M periph) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 |= (BIT(periph)););
}

/**
  * @brief  CC_RST disable APB1 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB1_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb1_peripheral_clk_disable(TYPE_ENUM_LL_CC_RST_APB1_M periph) {
    SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 &= ~(BIT(periph)););
}

/**
  * @brief  CC_RST enable APB2 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB2_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb2_peripheral_clk_enable(TYPE_ENUM_LL_CC_RST_APB2_M periph) {
    if((LL_CC_RST_APB2_M_HCC == periph)) {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 |= (BIT(periph)););
    } else {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 |= (BIT(periph)););
    }
}

/**
  * @brief  CC_RST disable APB2 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_APB2_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_apb2_peripheral_clk_disable(TYPE_ENUM_LL_CC_RST_APB2_M periph) {
    if((LL_CC_RST_APB2_M_HCC == periph)) {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 &= ~(BIT(periph)););
    } else {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 &= ~(BIT(periph)););
    }
}

/**
  * @brief  CC_RST enable AHB0 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_AHB0_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahb0_peripheral_clk_enable(TYPE_ENUM_LL_CC_RST_AHB0_M periph) {
    if((LL_CC_RST_AHB0_M_GPIOA == periph) || 
       (LL_CC_RST_AHB0_M_GPIOB == periph) ||
       (LL_CC_RST_AHB0_M_GPIOC == periph) ||
       (LL_CC_RST_AHB0_M_GPIOD == periph) ||
       (LL_CC_RST_AHB0_M_DAC == periph)   ||
       (LL_CC_RST_AHB0_M_CAN == periph)) {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 |= (BIT(periph)););
    } else {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 |= (BIT(periph)););
    }
}

/**
  * @brief  CC_RST disable AHB0 peripheral CLK.
  * @param  periph : TYPE_ENUM_LL_CC_RST_AHB0_M
  * @retval None
  */
__STATIC_INLINE void ll_cc_rst_ahb0_peripheral_clk_disable(TYPE_ENUM_LL_CC_RST_AHB0_M periph) {
    if((LL_CC_RST_AHB0_M_GPIOA == periph) || 
       (LL_CC_RST_AHB0_M_GPIOB == periph) ||
       (LL_CC_RST_AHB0_M_GPIOC == periph) ||
       (LL_CC_RST_AHB0_M_GPIOD == periph) ||
       (LL_CC_RST_AHB0_M_DAC == periph)   ||
       (LL_CC_RST_AHB0_M_CAN == periph)) {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON4 &= ~(BIT(periph)););
    } else {
        SYSCTRL_REG_OPT(SYSCTRL->CLK_CON5 &= ~(BIT(periph)););
    }
}


/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

/**
  * @}
  */

/**
  * @}
  */

#endif //__TX_PHE_LL_CC_RST_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
