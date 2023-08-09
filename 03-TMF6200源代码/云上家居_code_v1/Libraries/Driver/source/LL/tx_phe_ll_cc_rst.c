/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_cc_rst.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_cc_rst.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup cc_rst_interface_gr CC_RST Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup CC_RST_LL_Driver CC_RST LL Driver
  * @ingroup  cc_rst_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup CC_RST_LL_Interrupt CC_RST LL Interrupt Handle function
  * @ingroup  CC_RST_LL_Driver
  * @brief   CC_RST LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup CC_RST_LL_Inti_Cfg CC_RST LL Initialization And Configuration
  * @ingroup  CC_RST_LL_Driver
  * @brief    CC_RST LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Delay5MsIn32khz
  * @param  count: the count which you want to delay.
  * @retval None
  */  
void ll_cc_rst_delay5ms_32khz(int count)
{
    volatile u32 loop;
    while(count--){
        for(loop = (32000/1000 * 5/6); loop!=0; loop--);
    }
}

/**
  *  PLL center frequnce table depend on TYPE_ENUM_LL_CC_RST_PLL_FREQ
  *
  */
static const u32 __ll_pll_center_freq[4] = {
    312000000,
    396000000,
    466000000,
    520000000,
};

/**
  * @brief  Set the ADCPLL clock frequency.
  * @param  freq        : ADCPLL frequency
  * @param  pll_clk_src : the reference clock source for the ADCPLL
  * @retval None
  */
void ll_cc_adcpll_set(u32 freq, TYPE_ENUM_LL_CC_RST_PLLCLK_SRC pll_clk_src)
{
    u32 diff, pll_ref_clk;
    u32 pll_center_freq = 0;
    u32 i;
    u64 pll_fixed_point;
    u32 diff_min = ~0;
    
    /* Find the closest center frequency */
    for(i=0; i<4; i++) {
        /* ABS */
        if(freq > __ll_pll_center_freq[i]) {
            diff = freq - __ll_pll_center_freq[i];
        } else {
            diff = __ll_pll_center_freq[i] - freq;
        }
        
        if(diff < diff_min) {
            diff_min        = diff;
            pll_center_freq = i;
        }
    }
    
    switch(pll_clk_src) {
        case LL_CC_RST_PLLCLK_SRC_HXOSC:
            pll_ref_clk = HXOSC_CLK;
            break;
        
        case LL_CC_RST_PLLCLK_SRC_RC8M:
            pll_ref_clk = RC8M_CLK;
            break;
        
        /* error */
        default :
            pll_ref_clk = 1000000;
            break;
    }
    /* Calculate the integer division and fractional division of the PLL */
    /* {PLL_INT, PLL_FRAC} = freq/4/pll_ref_clk */
    pll_fixed_point = ((u64)freq << (16-2))/pll_ref_clk;
    
    /* If the ADCPLL clock is now consistent with freq, there is no need to continue configuration */
    if(LL_CC_AIPCON0_ADCPLL_BAND_GET(SYSCTRL->AIP_CON0) == pll_center_freq) {
        if(FPLL1->FPLL1_INT == (pll_fixed_point >> 16)) {
            if(FPLL1->FPLL1_FRAC == (pll_fixed_point & 0xFFFF)) {
                if(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_ADCPLL_EN) {
                    if(LL_CC_RST_ADCPLL_CLK_SRC_GET(SYSCTRL->CLK_CON0) == pll_clk_src) {
                        switch(pll_clk_src) {
                            case LL_CC_RST_PLLCLK_SRC_HXOSC:
                                if(!(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_ADCPLL_LPFSEL_RC8M)) {
                                    if(SYSCTRL->AIP_CON1 & LL_CC_AIPCON1_HXOSC_EN) {
                                        return;
                                    }
                                }
                                break;
                                
                            case LL_CC_RST_PLLCLK_SRC_RC8M:
                                if(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_ADCPLL_LPFSEL_RC8M) {
                                    if(SYSCTRL->AIP_CON1 & LL_CC_AIPCON1_RC8M_EN) {
                                        return;
                                    }
                                }
                                break;
                                
                            default:
                                break;
                        }
                    }
                }
            }
        }
    }
    
    /* Set reference clock */
    switch (pll_clk_src) {
        case LL_CC_RST_PLLCLK_SRC_HXOSC:
            SYSCTRL_REG_OPT (
                SYSCTRL->CLK_CON0 &= ~LL_CC_RST_ADCPLL_CLK_SRC_SEL(3);
                SYSCTRL->AIP_CON0 &= ~LL_CC_AIPCON0_ADCPLL_LPFSEL_RC8M;
                SYSCTRL->AIP_CON1 |= LL_CC_AIPCON1_HXOSC_EN;
            );
            break;
        
        case LL_CC_RST_PLLCLK_SRC_RC8M:
            SYSCTRL_REG_OPT (
                SYSCTRL->AIP_CON1 |= LL_CC_AIPCON1_RC8M_EN;
                SYSCTRL->CLK_CON0 &= ~LL_CC_RST_ADCPLL_CLK_SRC_SEL(3);
                SYSCTRL->CLK_CON0 |= LL_CC_RST_ADCPLL_CLK_SRC_SEL(LL_CC_RST_PLLCLK_SRC_RC8M);
                SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_ADCPLL_LPFSEL_RC8M;
            );
            break;
        
        default:
            break;
    }
    
    SYSCTRL_REG_OPT (
        /* Set the PLL center frequency point */
        SYSCTRL->AIP_CON0 &= ~LL_CC_AIPCON0_ADCPLL_BAND_SET(0x03);
        SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_ADCPLL_BAND_SET(pll_center_freq);
        /* Set the integer division and fractional division of the PLL. */
        FPLL1->FPLL1_INT   = pll_fixed_point >> 16;
        FPLL1->FPLL1_FRAC  = pll_fixed_point & 0xFFFF;
        FPLL1->FPLL1_SSC   = 0x0;
        /* update */
        FPLL1->FPLL1_CON   = 0x00;
        FPLL1->FPLL1_CON   = 0x01;
        /* enable */
        SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_ADCPLL_EN;
    );
    /* Probably delayed by 600us */
    ll_cc_rst_delay5ms_32khz(150);
}

/**
  * @brief  Set SYSPLL
  * @param  sys_max_clk : maximum system clock frequency
  * @param  pll_clk_src : the reference clock source for SYSPLL
  * @retval None
  */
void ll_cc_syspll_set(u32 sys_max_clk, TYPE_ENUM_LL_CC_RST_PLLCLK_SRC pll_clk_src)
{
    u32 pll_div, pll_ref_clk;
    u32 quotient, remainder;
    u32 pll_center_freq = 0;
    u32 syspll_clk;
    u32 i;
    u64 pll_fixed_point;
    u32 diff_min = ~0;
    
    /* Look for the center frequency point with the smallest crossover error */
    for(i=0; i<4; i++) {
        /* The clock after the SYSPLL is divided by 2 will be divided by the div register */
        quotient  = (__ll_pll_center_freq[i]>>1) / sys_max_clk;
        remainder = (__ll_pll_center_freq[i]>>1) % sys_max_clk;
        
        /* If the system clock is small, the division ratio may be greater than 16. */
        if(quotient > 15) {
            quotient  = 15;
            remainder = __ll_pll_center_freq[i] - 15*sys_max_clk;
        } else if(quotient < 15) {
            /*
               If the remainder is greater than half of the dividend, the 
               value of sys_max_clk*(quotient+1) will be closer to the
               center frequency.
            */
            if(remainder > (sys_max_clk>>1)) {
                remainder = sys_max_clk - remainder;
                quotient++;
            }
        }
        
        if(remainder < diff_min) {
            diff_min        = remainder;
            pll_center_freq = i;
            pll_div         = quotient-1;
            syspll_clk      = quotient * sys_max_clk * 2;
        }
    }
    
    switch(pll_clk_src) {
        case LL_CC_RST_PLLCLK_SRC_HXOSC:
            pll_ref_clk = HXOSC_CLK;
            break; 
        
        case LL_CC_RST_PLLCLK_SRC_RC8M:
            pll_ref_clk = RC8M_CLK;
            break;
        
        /* error */
        default :
            pll_ref_clk = 1000000;
            break;
    }
    
    /* Calculate the integer division and fractional division of the PLL */
    /* {PLL_INT, PLL_FRAC} = syspll/4/pll_ref_clk */
    pll_fixed_point = ((u64)syspll_clk << (16-2))/pll_ref_clk;
    
    /* If the ADCPLL clock is now consistent with freq, there is no need to continue configuration */
    if(LL_CC_AIPCON0_SYSPLL_BAND_GET(SYSCTRL->AIP_CON0) == pll_center_freq) {
        if((FPLL0->FPLL0_INT == (pll_fixed_point >> 16)) && 
           (FPLL0->FPLL0_FRAC == (pll_fixed_point & 0xFFFF))) {
            if(LL_CC_RST_SYSCLK_SRC_GET(SYSCTRL->CLK_CON0) == LL_CC_RST_SYSCLK_SRC_SYSPLL_DIV) {
                if(LL_CC_RST_SYSPLLCLK_DIV_GET(SYSCTRL->CLK_CON2) == pll_div) {
                    if(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_SYSPLL_EN) {
                        if(LL_CC_RST_SYSPLL_CLK_SRC_GET(SYSCTRL->CLK_CON0) == pll_clk_src) {
                            switch(pll_clk_src) {
                                case LL_CC_RST_PLLCLK_SRC_HXOSC:
                                    if(SYSCTRL->AIP_CON1 & LL_CC_AIPCON1_HXOSC_EN) {
                                        if(!(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_SYSPLL_LPFSEL_RC8M)) {
                                            return;
                                        }
                                    }
                                    break;
                                    
                                case LL_CC_RST_PLLCLK_SRC_RC8M:
                                    if(SYSCTRL->AIP_CON0 & LL_CC_AIPCON0_SYSPLL_LPFSEL_RC8M) {
                                        if(SYSCTRL->AIP_CON1 & LL_CC_AIPCON1_RC8M_EN) {
                                            return;
                                        }
                                    }
                                    break;
                                    
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
        }
    }
    
    /* Set reference clock */
    switch(pll_clk_src) {
        case LL_CC_RST_PLLCLK_SRC_HXOSC:
            SYSCTRL_REG_OPT (
                SYSCTRL->CLK_CON0 &= ~LL_CC_RST_SYSPLL_CLK_SRC_SEL(3);
                SYSCTRL->AIP_CON0 &= ~LL_CC_AIPCON0_SYSPLL_LPFSEL_RC8M;
                SYSCTRL->AIP_CON1 |= LL_CC_AIPCON1_HXOSC_EN;
            );
            break;
        
        case LL_CC_RST_PLLCLK_SRC_RC8M:
            SYSCTRL_REG_OPT (
                SYSCTRL->AIP_CON1 |= LL_CC_AIPCON1_RC8M_EN;
                SYSCTRL->CLK_CON0 &= ~LL_CC_RST_SYSPLL_CLK_SRC_SEL(3);
                SYSCTRL->CLK_CON0 |= LL_CC_RST_SYSPLL_CLK_SRC_SEL(LL_CC_RST_PLLCLK_SRC_RC8M);
                SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_SYSPLL_LPFSEL_RC8M;
            );
            break;
        
        default:
            break;
    }
    
    SYSCTRL_REG_OPT (
        /* First switch the system clock to the RC 32KHz */
        SYSCTRL->CLK_CON0 = (SYSCTRL->CLK_CON0 & ~LL_CC_RST_SYSCLK_SRC_SEL(0x03)) |
                            LL_CC_RST_SYSCLK_SRC_SEL(LL_CC_RST_SYSCLK_SRC_RC32K);
        /* Set the PLL center frequency point */
        SYSCTRL->AIP_CON0 &= ~LL_CC_AIPCON0_SYSPLL_BAND_SET(0x03);
        SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_SYSPLL_BAND_SET(pll_center_freq);
        /* Set the PLL divider ratio */
        SYSCTRL->CLK_CON2 &= ~LL_CC_RST_SYSPLLCLK_DIV(0x0F);
        SYSCTRL->CLK_CON2 |= LL_CC_RST_SYSPLLCLK_DIV(pll_div);
        /* Set the integer division and fractional division of the PLL. */
        FPLL0->FPLL0_INT  = pll_fixed_point >> 16;
        FPLL0->FPLL0_FRAC = pll_fixed_point & 0xFFFF;
        FPLL0->FPLL0_SSC  = 0x0;
        /* update */
        FPLL0->FPLL0_CON  = 0x00;
        FPLL0->FPLL0_CON  = 0x01;
        /* enable */
        SYSCTRL->AIP_CON0 |= LL_CC_AIPCON0_SYSPLL_EN;

        /* delay 5ms */
        ll_cc_rst_delay5ms_32khz(1);

        /* The system clock is switched back to the PLL */
        SYSCTRL->CLK_CON0 = (SYSCTRL->CLK_CON0 & ~LL_CC_RST_SYSCLK_SRC_SEL(0x03)) |
                            LL_CC_RST_SYSCLK_SRC_SEL(LL_CC_RST_SYSCLK_SRC_SYSPLL_DIV);
    );
}

/**
  * @brief  SYSCLK source config
  * @param  p_cfg : SYSCLK configure structure pointer
  * @retval None
  */
void ll_cc_sysclk_select(TYPE_ENUM_LL_CC_RST_SYSCLK_SRC src)
{
    if(LL_CC_RST_SYSCLK_SRC_GET(SYSCTRL->CLK_CON0) != src) {
        SYSCTRL->CLK_CON0 = (SYSCTRL->CLK_CON0 & ~LL_CC_RST_SYSCLK_SRC_SEL(0x03)) |
            LL_CC_RST_SYSCLK_SRC_SEL(src);
    }
    
    SYS_CLK = ll_cc_sys_clk_get();
    
    extern void sys_peripheral_init(void);
    sys_peripheral_init();
}

/**
  * @brief  System clock tree divider setting
  * @param  p_cfg : Clock divider structure pointer
  * @retval None
  */
void ll_cc_clk_div_config(TYPE_LL_CC_DIV_CFG *p_cfg)
{
    u32 clkcon1;
    
    SYSCTRL_REG_OPT (
        clkcon1 = SYSCTRL->CLK_CON1;
    );
    
    clkcon1 &= ~(LL_CC_RST_SYSCLK_DIV(0xFF) | LL_CC_RST_APB0CLK_DIV(0xFF) |
                 LL_CC_RST_APB1CLK_DIV(0xFF));
    
    clkcon1 |= LL_CC_RST_SYSCLK_DIV((p_cfg->sys_clk_div == 0xFF) ? 0xFE : p_cfg->sys_clk_div)    |
               LL_CC_RST_APB0CLK_DIV((p_cfg->apb0_clk_div == 0xFF) ? 0xFE : p_cfg->apb0_clk_div) |
               LL_CC_RST_APB1CLK_DIV((p_cfg->apb1_clk_div == 0xFF) ? 0xFE : p_cfg->apb1_clk_div);
    
    SYSCTRL_REG_OPT (
        SYSCTRL->CLK_CON1 = clkcon1;
    );
}

/**
  * @brief  Get the real system clock frequency
  * @retval Returns the system clock frequency in Hz
  */
u32 ll_cc_sys_clk_get(void)
{
    u32 pll_ref_clk;
    u64 pll_fixed;
    u32 max_sys_clk;
    
    switch(LL_CC_RST_SYSCLK_SRC_GET(SYSCTRL->CLK_CON0)) {
        case LL_CC_RST_SYSCLK_SRC_RC32K:
            max_sys_clk = RC32K_CLK;
            break;
        case LL_CC_RST_SYSCLK_SRC_HXOSC:
            max_sys_clk = HXOSC_CLK;
            break;
        case LL_CC_RST_SYSCLK_SRC_SYSPLL_DIV:
            /* Select PLL reference clock frequency */
            switch(LL_CC_RST_SYSPLL_CLK_SRC_GET(SYSCTRL->CLK_CON0)) {
                case LL_CC_RST_PLL_REFCLK_HXOSC:
                    pll_ref_clk = HXOSC_CLK;
                    break;
                
                case LL_CC_RST_PLL_REFCLK_RC8M:
                    pll_ref_clk = RC8M_CLK;
                    break;
                
                default:
                    pll_ref_clk = 0;
                    break;
            }
            
            pll_fixed  = FPLL0->FPLL0_FRAC;
            pll_fixed |= (u64)FPLL0->FPLL0_INT<<16;
            /* Calculate the maximum system clock */
            max_sys_clk = pll_fixed * pll_ref_clk >> (16-1);
            break;
        case LL_CC_RST_SYSCLK_SRC_RC8M:
            max_sys_clk = RC8M_CLK;
            break;
    }
    
    /* sys clk = max_sys_clk / div */
    return max_sys_clk / (1 + LL_CC_RST_SYSPLLCLK_DIV_GET(SYSCTRL->CLK_CON2));
}

/**
  * @brief  Get the oscillation frequency of the ADCPLL
  * @retval Returns the oscillation frequency of the ADCPLL in Hz
  */
u32 ll_cc_adc_pll_get(void)
{
    u32 pll_ref_clk;
    u64 pll_fixed;
    
    /* Select PLL reference clock frequency */
    switch(LL_CC_RST_ADCPLL_CLK_SRC_GET(SYSCTRL->CLK_CON0)) {
        case LL_CC_RST_PLL_REFCLK_HXOSC:
            pll_ref_clk = HXOSC_CLK;
            break;
        
        case LL_CC_RST_PLL_REFCLK_RC8M:
            pll_ref_clk = RC8M_CLK;
            break;
        
        default:
            pll_ref_clk = 0;
            break;
    }
    
    pll_fixed  = FPLL1->FPLL1_FRAC;
    pll_fixed |= (u64)FPLL1->FPLL1_INT<<16;
    /* Calculate the frequency of the ADCPLL */
    return pll_fixed * pll_ref_clk >> (16-2);
}

/**
  * @brief  Test clock output function
  * @param  clk_sel : Select the clock to output
  * @retval None
  * @note   The clock will be output through PC0
  */
void ll_cc_test_clk_out(TYPE_ENUM_LL_CC_TEST_OUT clk_sel)
{
    ll_gpio_dir(GPIOC, BIT(0), LL_GPIO_PIN_OUTPUT);
    SYSCTRL_REG_OPT (
        SYSCTRL->CLK_CON0 &= ~LL_CC_CLKCON0_TEST_CLK_SEL(LL_CC_CLKCON0_TEST_CLK_SEL_MASK);
        SYSCTRL->CLK_CON0 |= LL_CC_CLKCON0_TEST_CLK_SEL(clk_sel);
        SYSCTRL->SYS_CON0 |= LL_CC_SYSCON0_TEST_OUT_EN;
    );
}

/**
  * @}
  */

/** @defgroup CC_RST_LL_Data_Transfers CC_RST LL Data transfers functions
  * @ingroup  CC_RST_LL_Driver
  * @brief    CC_RST LL Data transfers functions 
  * @{
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

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
