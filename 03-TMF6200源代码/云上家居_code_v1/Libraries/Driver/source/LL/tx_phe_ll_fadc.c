/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_fadc.c
  * @author  HUGE-IC Application Team
  * @version V1.0.2
  * @date    04-08-2019
  * @brief   This file contains all the FADC LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  * 1.The calculation formula for FADC is as follows:
  *     Yout = a*Xin + b
  *
  * Parameter Description:
  *     Yout: The results of the FADC acquisition can be obtained by 
  *           the function ll_fadc_get_result().
  *     a   : 32768/2/vref
  *     b   : -32768/2*vcm/vref
  *     Xin : Input data for FADC.
  *
  * them:
  *     vref: The default is 1.500V.
  *     vcm : The default is 1.500V.
  *
  * If the user wants to get the data conversion value collected by the FADC, 
  * the function ll_fadc_get_result_float() can be called, which returns the 
  * actual data and returns it as a float type.
  *
  *
  * 2.Correspondence table of fadc synchronous sampling path and SOC and adc channel:
  * ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  * ©¦ synchronous sampling chn ©¦    chn    ©¦                  adc_chn_sel                   ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             0            ©¦  0 and 1  ©¦ LL_FADC_CHNA_SARADC0 and LL_FADC_CHNA_SARADC13 ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             1            ©¦  2 and 3  ©¦ LL_FADC_CHNA_SARADC1 and LL_FADC_CHNA_SARADC12 ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             2            ©¦  4 and 5  ©¦ LL_FADC_CHNA_SARADC2 and LL_FADC_CHNA_SARADC11 ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             3            ©¦  6 and 7  ©¦ LL_FADC_CHNA_SARADC3 and LL_FADC_CHNA_SARADC10 ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             4            ©¦  8 and 9  ©¦ LL_FADC_CHNA_SARADC4 and LL_FADC_CHNA_SARADC9  ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             5            ©¦ 10 and 11 ©¦ LL_FADC_CHNA_SARADC5 and LL_FADC_CHNA_SARADC8  ©¦
  * ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  * ©¦             6            ©¦ 12 and 13 ©¦ LL_FADC_CHNA_SARADC6 and LL_FADC_CHNA_SARADC7  ©¦
  * ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  *
  * If you want to use the synchronous sampling function of fadc, you can follow the steps below.
  *
  * 1)The user needs to select the path of synchronous sampling according to the above table, and 
  *   correspondingly configure chn and adc_chn_sel in the TYPE_LL_FADC_CFG variable, and finally 
  *   call the function ll_fadc_config() to configure;
  *
  * 2)The path of the synchronous sampling is configured by calling the function ll_fadc_sync_sample_enable().
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_fadc.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup fadc_interface_gr FADC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup FADC_LL_Driver FADC LL Driver
  * @ingroup  fadc_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FADC_TRIM_EN                    0

static const u8 __ll_fadc_sfr_soc_timer[16] = {
    FADCSFRSOCTIMER0,  FADCSFRSOCTIMER1,  FADCSFRSOCTIMER2,  FADCSFRSOCTIMER3,
    FADCSFRSOCTIMER4,  FADCSFRSOCTIMER5,  FADCSFRSOCTIMER6,  FADCSFRSOCTIMER7,
    FADCSFRSOCTIMER8,  FADCSFRSOCTIMER9,  FADCSFRSOCTIMER10, FADCSFRSOCTIMER11,
    FADCSFRSOCTIMER12, FADCSFRSOCTIMER13, FADCSFRSOCTIMER14, FADCSFRSOCTIMER15
};

static const u8 __ll_fadc_sfr_soc_con[8] = {
    FADCSFRSOCCON0,  FADCSFRSOCCON1,  FADCSFRSOCCON2,  FADCSFRSOCCON3,
    FADCSFRSOCCON4,  FADCSFRSOCCON5,  FADCSFRSOCCON6,  FADCSFRSOCCON7,
};

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup FADC_LL_Interrupt FADC LL Interrupt Handle function
  * @ingroup  FADC_LL_Driver
  * @brief   FADC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup FADC_LL_Inti_Cfg FADC LL Initialization And Configuration
  * @ingroup  FADC_LL_Driver
  * @brief    FADC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  The software counts the values for the delay of the simulated path configuration.
  * @param  ms: The value of ms.
  * @retval None
  */
#define LL_FADC_DELAY_MS(ms)                   delay_ms(ms)

/**
  * @brief  The initialization function of the underlying fadc analog module.
  * @param  trim_en: Enable the function of the trim.
  * @retval None
  */
void ll_fadc_analog_init(u8 trim_en) 
{
    // analog setting
    ll_fadc_wr_sfr(FADCSFRANACON0, (
                   LL_FADC_ANA_CTRL_SEL(0) |      // 12     ana_ctrl_sel
                   LL_FADC_ANA_CMPBSEN(0)  |      // 11     ana_cmpbsen
                   LL_FADC_ANA_TVS2(0)     |      // 10     ana_tvs2
                   LL_FADC_ANA_TVS1(0)     |      // 9      ana_tvs1
                   LL_FADC_ANA_TVS0(0)     |      // 8      ana_tvs0
                   LL_FADC_ANA_CHSEL(0)    |      // 7      ana_chsel
                   LL_FADC_ANA_TRIM(0)     |      // 6      ana_trim
                   LL_FADC_ANA_ENVTS(0)    |      // 5      ana_envts
                   LL_FADC_ANA_ENSYNS(0)   |      // 4      ana_ensyns
                   LL_FADC_ANA_CMPEN(0)    |      // 3      ana_cmpen
                   LL_FADC_ANA_BUFFEN(1)   |      // 2      ana_buffen
                   LL_FADC_ANA_BIASEN(1)   |      // 1      ana_biasen
                   LL_FADC_ANA_ADCEN(0)           // 0      ana_adcen
                   ));
    LL_FADC_DELAY_MS(10); // real chip: 10ms

    ll_fadc_wr_sfr(FADCSFRANACON1, (
                   LL_FADC_ANA_SIOB(0)    |       // 29     ana_siob
                   LL_FADC_ANA_S2B(1)     |       // 28     ana_s2b
                   LL_FADC_ANA_S1B(1)     |       // 27     ana_s1b
                   LL_FADC_ANA_S0B(1)     |       // 26     ana_s0b
                   LL_FADC_ANA_VCMIBEN(1) |       // 25     ana_vcmiben
                   LL_FADC_ANA_ENSHTB(0)  |       // 24     ana_enshtb
                   LL_FADC_ANA_ENSHB(0)   |       // 23     ana_enshb
                   LL_FADC_ANA_ENS2DB(0)  |       // 16:22  ana_ens2db
                   LL_FADC_ANA_SIOA(0)    |       // 13     ana_sioa
                   LL_FADC_ANA_S2A(1)     |       // 12     ana_s2a
                   LL_FADC_ANA_S1A(1)     |       // 11     ana_s1a
                   LL_FADC_ANA_S0A(1)     |       // 10     ana_s0a
                   LL_FADC_ANA_VCMIAEN(1) |       // 9      ana_vcmiaen
                   LL_FADC_ANA_ENSHTA(0)  |       // 8      ana_enshta
                   LL_FADC_ANA_ENSHA(0)   |       // 7      ana_ensha
                   LL_FADC_ANA_ENS2DA(0)          // 0:6    ana_ens2da
                   ));
    LL_FADC_DELAY_MS(10); // real chip: 10ms

    ll_fadc_wr_sfr(FADCSFRANACON1, (
                   LL_FADC_ANA_SIOB(0)      |     // 29     ana_siob
                   LL_FADC_ANA_S2B(1)       |     // 28     ana_s2b
                   LL_FADC_ANA_S1B(1)       |     // 27     ana_s1b
                   LL_FADC_ANA_S0B(1)       |     // 26     ana_s0b
                   LL_FADC_ANA_VCMIBEN(1)   |     // 25     ana_vcmiben
                   LL_FADC_ANA_ENSHTB(0)    |     // 24     ana_enshtb
                   LL_FADC_ANA_ENSHB(0)     |     // 23     ana_enshb
                   LL_FADC_ANA_ENS2DB(0x7F) |     // 16:22  ana_ens2db
                   LL_FADC_ANA_SIOA(0)      |     // 13     ana_sioa
                   LL_FADC_ANA_S2A(1)       |     // 12     ana_s2a
                   LL_FADC_ANA_S1A(1)       |     // 11     ana_s1a
                   LL_FADC_ANA_S0A(1)       |     // 10     ana_s0a
                   LL_FADC_ANA_VCMIAEN(1)   |     // 9      ana_vcmiaen
                   LL_FADC_ANA_ENSHTA(0)    |     // 8      ana_enshta
                   LL_FADC_ANA_ENSHA(0)     |     // 7      ana_ensha
                   LL_FADC_ANA_ENS2DA(0x7F)       // 0:6    ana_ens2da
                   ));
    LL_FADC_DELAY_MS(2); // real chip: 2ms

    ll_fadc_wr_sfr(FADCSFRANACON1, (
                   LL_FADC_ANA_SIOB(1)      |     // 29     ana_siob
                   LL_FADC_ANA_S2B(1)       |     // 28     ana_s2b
                   LL_FADC_ANA_S1B(1)       |     // 27     ana_s1b
                   LL_FADC_ANA_S0B(1)       |     // 26     ana_s0b
                   LL_FADC_ANA_VCMIBEN(1)   |     // 25     ana_vcmiben
                   LL_FADC_ANA_ENSHTB(0)    |     // 24     ana_enshtb
                   LL_FADC_ANA_ENSHB(1)     |     // 23     ana_enshb
                   LL_FADC_ANA_ENS2DB(0x7F) |     // 16:22  ana_ens2db
                   LL_FADC_ANA_SIOA(1)      |     // 13     ana_sioa
                   LL_FADC_ANA_S2A(1)       |     // 12     ana_s2a
                   LL_FADC_ANA_S1A(1)       |     // 11     ana_s1a
                   LL_FADC_ANA_S0A(1)       |     // 10     ana_s0a
                   LL_FADC_ANA_VCMIAEN(1)   |     // 9      ana_vcmiaen
                   LL_FADC_ANA_ENSHTA(0)    |     // 8      ana_enshta
                   LL_FADC_ANA_ENSHA(1)     |     // 7      ana_ensha
                   LL_FADC_ANA_ENS2DA(0x7F)       // 0:6    ana_ens2da
                   ));
    LL_FADC_DELAY_MS(2); // real chip: 2ms

    ll_fadc_wr_sfr(FADCSFRANACON0, (
                   LL_FADC_ANA_CTRL_SEL(0) |      // 12     ana_ctrl_sel
                   LL_FADC_ANA_CMPBSEN(1)  |      // 11     ana_cmpbsen
                   LL_FADC_ANA_TVS2(0)     |      // 10     ana_tvs2
                   LL_FADC_ANA_TVS1(0)     |      // 9      ana_tvs1
                   LL_FADC_ANA_TVS0(0)     |      // 8      ana_tvs0
                   LL_FADC_ANA_CHSEL(0)    |      // 7      ana_chsel
                   LL_FADC_ANA_TRIM(0)     |      // 6      ana_trim
                   LL_FADC_ANA_ENVTS(0)    |      // 5      ana_envts
                   LL_FADC_ANA_ENSYNS(0)   |      // 4      ana_ensyns
                   LL_FADC_ANA_CMPEN(0)    |      // 3      ana_cmpen
                   LL_FADC_ANA_BUFFEN(1)   |      // 2      ana_buffen
                   LL_FADC_ANA_BIASEN(1)   |      // 1      ana_biasen
                   LL_FADC_ANA_ADCEN(0)           // 0      ana_adcen
                   ));
    LL_FADC_DELAY_MS(1); // real chip: 100us

    ll_fadc_wr_sfr(FADCSFRANACON0, (
                   LL_FADC_ANA_CTRL_SEL(1)   |    // 12     ana_ctrl_sel
                   LL_FADC_ANA_CMPBSEN(1)    |    // 11     ana_cmpbsen
                   LL_FADC_ANA_TVS2(1)       |    // 10     ana_tvs2
                   LL_FADC_ANA_TVS1(1)       |    // 9      ana_tvs1
                   LL_FADC_ANA_TVS0(0)       |    // 8      ana_tvs0
                   LL_FADC_ANA_CHSEL(0)      |    // 7      ana_chsel
                   LL_FADC_ANA_TRIM(trim_en) |    // 6      ana_trim
                   LL_FADC_ANA_ENVTS(1)      |    // 5      ana_envts
                   LL_FADC_ANA_ENSYNS(0)     |    // 4      ana_ensyns
                   LL_FADC_ANA_CMPEN(1)      |    // 3      ana_cmpen
                   LL_FADC_ANA_BUFFEN(1)     |    // 2      ana_buffen
                   LL_FADC_ANA_BIASEN(1)     |    // 1      ana_biasen
                   LL_FADC_ANA_ADCEN(1)           // 0      ana_adcen
                   ));
    LL_FADC_DELAY_MS(1); // real chip: 1ms
}

/**
  * @brief  The underlying fadc clock initialization function.
  * @param  adpll_div_val: Adc clock division ratio value.
  * @retval None
  */
void ll_fadc_clk_init(u32 adpll_val, u8 adpll_div_val)
{
    /* Select the clock source for adcpll156M. */
    SYSCTRL_REG_OPT(
        /* adc clk select pll */
        SYSCTRL->CLK_CON2 = (SYSCTRL->CLK_CON2 & (~LL_CC_RST_ADCCLK_DIV(0xF))) | 
                            LL_CC_RST_ADCCLK_DIV(adpll_div_val);
    );
}

/**
  * @brief  Low layer FADC module initialization
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_FADC_INIT)
  * @retval None
  */
void ll_fadc_init(FADC_TypeDef *p_fadc, TYPE_LL_FADC_INIT *p_init)
{
    u32 i        = 0;
    
    /* The system resets the FADC module, Enable the clock of the module. */
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(p_init->fadc_adpll_div_val <= 15);
    TX_ASSERT(p_init->soc_hpri_val <= 16);
    
    /* Initialize the clock of FADC. */
    ll_fadc_clk_init(__EPWM_FADC_ADPLL_VAL, p_init->fadc_adpll_div_val);
    
    /* Initialize the analog of FADC. */
#if FADC_TRIM_EN
    ll_fadc_analog_init(1);
#else
    ll_fadc_analog_init(0);
#endif
    
    /* You need to wait until the simulation of fadc is initialized before you can perform ll_fadc_rd_sfr(). */
    u32 soc_con8 = ll_fadc_rd_sfr(FADCSFRSOCCON8);
    
    /* ADC system clock domain data path enable signal. */
    p_fadc->FADCCON0 = LL_FADC_CON0_EN;
    
    /* FADCSFRSOCCONx(x=0..7) = {Reserved[31],trisel1[30:27],smp_win1[26:20],chsel1[19:16],
     *                           Reserved[15],trisel0[14:11],smp_win0[10:4] ,chsel0[3:0]}
     */
    /* config ADC SOC CH 0 and CH 1 */
    ll_fadc_wr_sfr(FADCSFRSOCCON0, (
                   LL_FADC_TRISEL1(0)  |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(1)   |          // 16:19  chsel1
                   LL_FADC_TRISEL0(1)  |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(0)              // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 2 and CH 3 */            
    ll_fadc_wr_sfr(FADCSFRSOCCON1, (              
                   LL_FADC_TRISEL1(2)  |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(3)   |          // 16:19  chsel1
                   LL_FADC_TRISEL0(3)  |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(2)              // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 4 and CH 5 */            
    ll_fadc_wr_sfr(FADCSFRSOCCON2, (              
                   LL_FADC_TRISEL1(4)  |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(5)   |          // 16:19  chsel1
                   LL_FADC_TRISEL0(5)  |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(4)              // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 6 and CH 7 */            
    ll_fadc_wr_sfr(FADCSFRSOCCON3, (              
                   LL_FADC_TRISEL1(6)  |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(7)   |          // 16:19  chsel1
                   LL_FADC_TRISEL0(7)  |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(6)              // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 8 and CH 9 */            
    ll_fadc_wr_sfr(FADCSFRSOCCON4, (              
                   LL_FADC_TRISEL1(8)  |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(9)   |          // 16:19  chsel1
                   LL_FADC_TRISEL0(9)  |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(8)              // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 10 and CH 11 */          
    ll_fadc_wr_sfr(FADCSFRSOCCON5, (              
                   LL_FADC_TRISEL1(10) |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(11)  |          // 16:19  chsel1
                   LL_FADC_TRISEL0(11) |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(10)             // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 12 and CH 13 */          
    ll_fadc_wr_sfr(FADCSFRSOCCON6, (              
                   LL_FADC_TRISEL1(12) |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(13)  |          // 16:19  chsel1
                   LL_FADC_TRISEL0(13) |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(12)             // 0:3    chsel1
                   ));                            
    /* config ADC SOC CH 14 and CH 15 */          
    ll_fadc_wr_sfr(FADCSFRSOCCON7, (              
                   LL_FADC_TRISEL1(14) |          // 30:27  trisel1
                   LL_FADC_SMP_WIN1(4) |          // 26:20  smp_win1
                   LL_FADC_CHSEL1(15)  |          // 16:19  chsel1
                   LL_FADC_TRISEL0(15) |          // 11:14  trisel1
                   LL_FADC_SMP_WIN0(4) |          // 4:10   smp_win1
                   LL_FADC_CHSEL0(14)             // 0:3    chsel1
                   ));
    
    /* FADCSFRSOCCON8 = {Reserved[31:29],soc_hpri[28:24],simulen[23:16],tri_en[15:0]} */
    ll_fadc_wr_sfr(FADCSFRSOCCON8, (
                   LL_FADC_SOC_HPRI(0) |          // 24:28  soc_hpri
                   LL_FADC_SIMULEN(0)  |          // 16:23  simulen
                   LL_FADC_TRI_EN(0)              // 0:15   tri_en
                   ));
                    
    ll_fadc_wr_sfr(FADCSFRCALIB0, (
                   LL_FADC_ERR_OFR(0)        |    // 30     err_ofr
                   LL_FADC_CALIB_FLAG(0)     |    // 29     calib_flag
                   LL_FADC_ERR_OFR_CLR(0)    |    // 28     err_ofr_clr
                   LL_FADC_CLR_CALIB_FLAG(0) |    // 27     clr_calib_flag
                   LL_FADC_CALIB_TRI(0)      |    // 26     calib_tri
                   LL_FADC_RES_CHECK_CNT(8)  |    // 22:25  res_check_cnt
                   LL_FADC_ERR_RANGE(4)      |    // 17:21  err_range
                   LL_FADC_ITER_TIME(63)     |    // 11:16  iter_time
                   LL_FADC_WSTENSIZE(16)     |    // 1:10   wstepsize
#if FADC_TRIM_EN
                   LL_FADC_CALIB_WMODE(0)         // 0      calib_wmode
#else
                   LL_FADC_CALIB_WMODE(1)         // 0      calib_wmode
#endif
                   ));

    //FADCSFRCALIB1 = {Reserved[31:9],calib_norm_outsel[8],max_ofr_con[7:0]}
    ll_fadc_wr_sfr(FADCSFRCALIB1, (
                   LL_FADC_CALIB_NORM_OUTSEL(0) | // 8      calib_norm_outsel
                   LL_FADC_MAX_OFR_CON(0)         // 0:7    max_ofr_con
                   ));

#if !__FADC_TRIM_FORCE_OFF
    // ideal calibration wcoef 
    TYPE_LL_EFLASH_CHIP_PARAM *p_chip_param = (TYPE_LL_EFLASH_CHIP_PARAM*)LL_EF_CHIP_PARAM_STADDR;
    if((p_chip_param->crc32_fsaradc_coef != 0xFFFFFFFF) && 
       (p_chip_param->fsaradc_coef[0] != 0xFFFFFFFF) ) {
        ll_fadc_wr_sfr(FADCSFRWCOEF0,  LL_FADC_WCOEF0(p_chip_param->fsaradc_coef[0]));
        ll_fadc_wr_sfr(FADCSFRWCOEF1,  LL_FADC_WCOEF1(p_chip_param->fsaradc_coef[1]));
        ll_fadc_wr_sfr(FADCSFRWCOEF2,  LL_FADC_WCOEF2(p_chip_param->fsaradc_coef[2]));
        ll_fadc_wr_sfr(FADCSFRWCOEF3,  LL_FADC_WCOEF3(p_chip_param->fsaradc_coef[3]));
        ll_fadc_wr_sfr(FADCSFRWCOEF4,  LL_FADC_WCOEF4(p_chip_param->fsaradc_coef[4]));
        ll_fadc_wr_sfr(FADCSFRWCOEF5,  LL_FADC_WCOEF5(p_chip_param->fsaradc_coef[5]));
        ll_fadc_wr_sfr(FADCSFRWCOEF6,  LL_FADC_WCOEF6(p_chip_param->fsaradc_coef[6]));
        ll_fadc_wr_sfr(FADCSFRWCOEF7,  LL_FADC_WCOEF7(p_chip_param->fsaradc_coef[7]));
        ll_fadc_wr_sfr(FADCSFRWCOEF8,  LL_FADC_WCOEF8(p_chip_param->fsaradc_coef[8]));
        ll_fadc_wr_sfr(FADCSFRWCOEF9,  LL_FADC_WCOEF9(p_chip_param->fsaradc_coef[9]));
        ll_fadc_wr_sfr(FADCSFRWCOEF10, LL_FADC_WCOEF10(p_chip_param->fsaradc_coef[10]));
        ll_fadc_wr_sfr(FADCSFRWCOEF11, LL_FADC_WCOEF11(p_chip_param->fsaradc_coef[11]));
        ll_fadc_wr_sfr(FADCSFRWCOEF12, LL_FADC_WCOEF12(p_chip_param->fsaradc_coef[12]));
        ll_fadc_wr_sfr(FADCSFRWCOEF13, LL_FADC_WCOEF13(p_chip_param->fsaradc_coef[13]));
        ll_fadc_wr_sfr(FADCSFRWCOEF14, LL_FADC_WCOEF14(p_chip_param->fsaradc_coef[14]));
        ll_fadc_wr_sfr(FADCSFRWCOEF15, LL_FADC_WCOEF15(p_chip_param->fsaradc_coef[15]));
        // ideal calibration gain & dc offset 
        for(i=0; i<16; i++) {
            p_fadc->FADCPPROC0CON[i] = (p_fadc->FADCPPROC0CON[i])        &
                                       (~(LL_FADC_DC_OFFSET(0xFFF)))     |
                                       LL_FADC_DC_OFFSET(p_chip_param->fadc_cab.dc_offset);

        
            p_fadc->FADCPPROC1CON[i] = (p_fadc->FADCPPROC1CON[i])                                     &
                                       (~(LL_FADC_QUANTIFY_COEF(0xFFFF) | LL_FADC_DECIMAL_WIDTH(7))) |
                                       LL_FADC_DECIMAL_WIDTH(1)                                       |
                                       LL_FADC_QUANTIFY_COEF(p_chip_param->fadc_cab.gain);
        }
    } else 
#endif
    {
        ll_fadc_wr_sfr(FADCSFRWCOEF0,  LL_FADC_WCOEF0(1727));
        ll_fadc_wr_sfr(FADCSFRWCOEF1,  LL_FADC_WCOEF1(3455));
        ll_fadc_wr_sfr(FADCSFRWCOEF2,  LL_FADC_WCOEF2(6289));
        ll_fadc_wr_sfr(FADCSFRWCOEF3,  LL_FADC_WCOEF3(11446));
        ll_fadc_wr_sfr(FADCSFRWCOEF4,  LL_FADC_WCOEF4(20831));
        ll_fadc_wr_sfr(FADCSFRWCOEF5,  LL_FADC_WCOEF5(37913));
        ll_fadc_wr_sfr(FADCSFRWCOEF6,  LL_FADC_WCOEF6(69003));
        ll_fadc_wr_sfr(FADCSFRWCOEF7,  LL_FADC_WCOEF7(125585));
        ll_fadc_wr_sfr(FADCSFRWCOEF8,  LL_FADC_WCOEF8(228565));
        ll_fadc_wr_sfr(FADCSFRWCOEF9,  LL_FADC_WCOEF9(415990));
        ll_fadc_wr_sfr(FADCSFRWCOEF10, LL_FADC_WCOEF10(757101));
        ll_fadc_wr_sfr(FADCSFRWCOEF11, LL_FADC_WCOEF11(1377925));
        ll_fadc_wr_sfr(FADCSFRWCOEF12, LL_FADC_WCOEF12(2507824));
        ll_fadc_wr_sfr(FADCSFRWCOEF13, LL_FADC_WCOEF13(4564240));
        ll_fadc_wr_sfr(FADCSFRWCOEF14, LL_FADC_WCOEF14(8306917));
        ll_fadc_wr_sfr(FADCSFRWCOEF15, LL_FADC_WCOEF15(15118589));
    }
    
    //FADCSFRSOCTIMER = {Reserved[31:19],timer_en[18],timer[17:0]};
    for(i=0; i<16; i++) {
        ll_fadc_wr_sfr(__ll_fadc_sfr_soc_timer[i], (
                       LL_FADC_TIMER_EN(1) |      // 18     timer_en
                       LL_FADC_TIMER(0)           // 0:17   timer
                       ));                                      
    }
   
    ll_fadc_wr_sfr(FADCSFRADCCON0, (
                   LL_FADC_DATINV_EN(0)       |   // 25     datinv_en
                   LL_FADC_DATFMT_SEL(0)      |   // 24     datfmt_sel
                   LL_FADC_SOC_ONDLY_TIME(13) |   // 17:23  soc_ondly_time
                   LL_FADC_CONVT_TIME(20)     |   // 11:16  convt_time
                   LL_FADC_STABLE_TIME(25)    |   // 6:10   stable_time
                   LL_FADC_POLLING_SLOTCON(0) |   // 2:5    polling_slotcon
                   LL_FADC_POLLING_MODE(0)    |   // 1      polling_mode
                   LL_FADC_M_EN(1)                // 0      m_en
                   ));
    
    /* Configure FADC conversion high priority. */
    soc_con8 &= ~LL_FADC_SOC_HPRI(0x1F);
    soc_con8 |= LL_FADC_SOC_HPRI(p_init->soc_hpri_val);
    
    ll_fadc_wr_sfr(FADCSFRSOCCON8, soc_con8);
}

/**
  * @brief  Low layer FADC module detele initialization
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @retval None
  */
void ll_fadc_deinit(FADC_TypeDef *p_fadc)
{
    /* The system disable the FADC module, includes turning off the clock for the module. */
    TX_ASSERT(p_fadc == FADC);
}

/**
  * @brief  Low layer FADC module irq configuration
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_cfg : Module irq configuration structure pointer(TYPE_FADC_IRQ_CFG)
  * @retval None
  */
void ll_fadc_irq_config(FADC_TypeDef *p_fadc, TYPE_FADC_IRQ_CFG *p_cfg)
{
    u32 fadc_int0 = p_fadc->FADCINT0;
    u32 fadc_int1 = p_fadc->FADCINT1;
    
    fadc_int0 = p_cfg->sample_intr_en                                      ? 
                (fadc_int0 | LL_FADC_SMP_DONE_INT_EN(BIT(p_cfg->chn)))     : 
                (fadc_int0 & (~(LL_FADC_SMP_DONE_INT_EN(BIT(p_cfg->chn)))));
              
    fadc_int1 = p_cfg->dma_half_intr_en                                    ? 
                (fadc_int1 | LL_FADC_DMA_HALF_INT_EN(BIT(p_cfg->chn)))     : 
                (fadc_int1 & (~(LL_FADC_DMA_HALF_INT_EN(BIT(p_cfg->chn)))));
    fadc_int1 = p_cfg->dma_full_intr_en                                    ? 
                (fadc_int1 | LL_FADC_DMA_FULL_INT_EN(BIT(p_cfg->chn)))     : 
                (fadc_int1 & (~(LL_FADC_DMA_FULL_INT_EN(BIT(p_cfg->chn)))));
    
    p_fadc->FADCINT0 = fadc_int0;
    p_fadc->FADCINT1 = fadc_int1;
}

/**
  * @brief  Low layer FADC module configuration
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_cfg : Module configuration structure pointer(TYPE_LL_FADC_DMA_CFG)
  * @retval None
  */
void ll_fadc_config(FADC_TypeDef *p_fadc, TYPE_LL_FADC_CFG *p_cfg)
{
    u32 dma_len    = 0;
    u32 pproc0_con = 0;
    u32 pproc1_con = 0;
    u32 timer_con  = 0;
    u32 soc_con    = 0;
    
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(p_cfg->chn <= 15);
    TX_ASSERT(p_cfg->dma_len < 8192);
    TX_ASSERT(p_cfg->dc_offset < 4096);
    TX_ASSERT(p_cfg->timer_period < 262143);
    if(p_cfg->dma_en) {
        TX_ASSERT_ADDR(p_cfg->dma_addr, p_cfg->dma_len,
                       SRAM_SARADC_DMA_MASK, SRAM_SARADC_DMA_ALIGN_MASK);
    }
               
    dma_len     = p_cfg->dma_en ? LL_FADC_DMA_EN : 0;
    dma_len    |= LL_FADC_DMA_LEN(p_cfg->dma_len);
               
    pproc0_con  = LL_FADC_DC_OFFSET(p_cfg->dc_offset);
    pproc0_con |= LL_FADC_DOMNSAMPLE_SEL(p_cfg->downsample_sel);
    pproc0_con |= LL_FADC_AVERAGE_SEL(p_cfg->average_sel);
    pproc0_con |= p_cfg->average_en ? LL_FADC_AVERAGE_EN : 0;
    
    pproc1_con  = LL_FADC_QUANTIFY_COEF(p_cfg->quantify_coef);
    pproc1_con |= LL_FADC_DECIMAL_WIDTH(p_cfg->decimal_width);
    pproc1_con |= p_cfg->quantify_en ? LL_FADC_QUANTIFY_EN : 0;
    
    timer_con   = p_cfg->timer_en ? LL_FADC_TIMER_EN(1) : 0;
    timer_con  |= LL_FADC_TIMER(p_cfg->timer_period);
    
    soc_con     = ll_fadc_rd_sfr(__ll_fadc_sfr_soc_con[p_cfg->chn / 2]);
    if(p_cfg->chn % 2) {
        soc_con &= ~(LL_FADC_TRISEL1(0xF) | LL_FADC_CHSEL1(0xF));
        soc_con |= LL_FADC_TRISEL1(p_cfg->trigger_sel);
        soc_con |= LL_FADC_CHSEL1(p_cfg->adc_chn_sel);
    } else {
        soc_con &= ~(LL_FADC_TRISEL0(0xF) | LL_FADC_CHSEL0(0xF));
        soc_con |= LL_FADC_TRISEL0(p_cfg->trigger_sel);
        soc_con |= LL_FADC_CHSEL0(p_cfg->adc_chn_sel);
    }
    
    p_fadc->FADCDMAADDR[p_cfg->chn]   = LL_FADC_DMA_ADDR(p_cfg->dma_addr);
    p_fadc->FADCDMALEN[p_cfg->chn]    = dma_len;
    p_fadc->FADCPPROC0CON[p_cfg->chn] = pproc0_con;
    p_fadc->FADCPPROC1CON[p_cfg->chn] = pproc1_con;

    ll_fadc_set_dc_offset(p_fadc, p_cfg->chn, p_cfg->dc_offset);

    if(p_cfg->quantify_en) {  
        ll_fadc_set_quantify(p_fadc, p_cfg->chn, p_cfg->quantify_coef, p_cfg->decimal_width);
    } else {
        ll_fadc_set_quantify(p_fadc, p_cfg->chn, 0x8000, 0);
    }
    
    ll_fadc_wr_sfr(__ll_fadc_sfr_soc_timer[p_cfg->chn], timer_con);
    ll_fadc_wr_sfr(__ll_fadc_sfr_soc_con[p_cfg->chn / 2], soc_con);
}

/**
  * @}
  */

/** @defgroup FADC_LL_Data_Transfers FADC LL Data transfers functions
  * @ingroup  FADC_LL_Driver
  * @brief    FADC LL Data transfers functions 
  * @{
  */

/**
  * @brief  FADC module start function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_start(FADC_TypeDef *p_fadc, u16 chn_bits)
{
    TX_ASSERT(p_fadc == FADC);
    
#if FADC_TRIM_EN
    ll_fadc_wr_sfr(FADCSFRCALIB0, (
                   ll_fadc_rd_sfr(FADCSFRCALIB0) | LL_FADC_CALIB_TRI(1) // 26     calib_tri
                   ));
#endif
    ll_fadc_wr_sfr(FADCSFRSOCCON8, 
                   (ll_fadc_rd_sfr(FADCSFRSOCCON8) | LL_FADC_TRI_EN(chn_bits)));
}

/**
  * @brief  FADC module CPU force kick function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_kick_one(FADC_TypeDef *p_fadc, u16 chn_bits)
{
    TX_ASSERT(p_fadc == FADC);
    
    ll_fadc_wr_sfr(FADCSFRSOCFLAG, 
                   (ll_fadc_rd_sfr(FADCSFRSOCFLAG) | LL_FADC_FC_TRI(chn_bits)));
}

/**
  * @brief  FADC module stop function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_stop(FADC_TypeDef *p_fadc, u16 chn_bits)
{
    TX_ASSERT(p_fadc == FADC);
    
    ll_fadc_wr_sfr(FADCSFRSOCCON8, 
                   (ll_fadc_rd_sfr(FADCSFRSOCCON8) & (~LL_FADC_TRI_EN(chn_bits))));
}

/**
  * @brief  Set the value of dc offset for fadc.
  * @param  p_fadc       : Select the initialized FADC group pointer
  * @param  chn          : Set the number of chn.
  * @param  dc_offset_val: The value of dc offset.
  * @retval None
  */
void ll_fadc_set_dc_offset(FADC_TypeDef *p_fadc, u16 chn, u16 dc_offset_val)
{
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(dc_offset_val < 4096);

    u16 new_offset  = dc_offset_val;
    
#if !__FADC_TRIM_FORCE_OFF
    TYPE_LL_EFLASH_CHIP_PARAM *p_chip_param = (TYPE_LL_EFLASH_CHIP_PARAM*)LL_EF_CHIP_PARAM_STADDR;
    if((p_chip_param->crc32_fsaradc_coef != 0xFFFFFFFF) && 
       (p_chip_param->fsaradc_coef[0] != 0xFFFFFFFF) ) {
        /* Bx([1,12,11]) = B0 + B1/A0 (a0:gain, b0:offset) */
        s16 add_offset;
        u32 decimal = (p_fadc->FADCPPROC1CON[chn] & LL_FADC_DECIMAL_WIDTH(0x7)) >> 1;
        add_offset = ((((s32)((s16)dc_offset_val << 4)) << (15-decimal)) / p_chip_param->fadc_cab.gain);
        new_offset = ((s16)(p_chip_param->fadc_cab.dc_offset << 0) + add_offset) >> 4;
    }
#endif
    p_fadc->FADCPPROC0CON[chn] = (p_fadc->FADCPPROC0CON[chn])       &
                                 (~(LL_FADC_DC_OFFSET(0xFFF)))      |
                                 LL_FADC_DC_OFFSET(new_offset);
}

/**
  * @brief  Set the value of quantify for fadc.
  * @param  p_fadc          : Select the initialized FADC group pointer
  * @param  chn             : Set the number of chn.
  * @param  quantify_val    : The value of quantify.
  * @param  decimal_width   : specify Quantify output data remain how many decimal width rang:[0,6]
  * @retval None
  */
void ll_fadc_set_quantify(FADC_TypeDef *p_fadc, u16 chn, u16 quantify_val, u8 decimal_width)
{
    TX_ASSERT(p_fadc == FADC);

    u32 new_gain    = quantify_val;
    u32 new_decimal = decimal_width;
    u32 quantify_en = 0;

#if !__FADC_TRIM_FORCE_OFF
    TYPE_LL_EFLASH_CHIP_PARAM *p_chip_param = (TYPE_LL_EFLASH_CHIP_PARAM*)LL_EF_CHIP_PARAM_STADDR;
    if((p_chip_param->crc32_fsaradc_coef != 0xFFFFFFFF) && 
       (p_chip_param->fsaradc_coef[0] != 0xFFFFFFFF) ) {
        /* Ax([1,16,14]) = A0 * A1 (a0:gain, b0:offset) */
        new_gain = ((u32)quantify_val * p_chip_param->fadc_cab.gain) >> 15;
        new_decimal += 1;
        quantify_en = LL_FADC_QUANTIFY_EN;
    }
#endif
    p_fadc->FADCPPROC1CON[chn] = (p_fadc->FADCPPROC1CON[chn])                                      & 
                                 (~(LL_FADC_QUANTIFY_COEF(0xFFFF)|LL_FADC_DECIMAL_WIDTH(0x7)))     |
                                 LL_FADC_QUANTIFY_COEF(new_gain)                                   |
                                 LL_FADC_DECIMAL_WIDTH(new_decimal)                                |
                                 quantify_en;
}

/**
  * @brief  Set the value of quantify for fadc.
  * @param  p_fadc      : Select the initialized FADC group pointer
  * @param  chn         : Set the number of chn.
  * @param  average_val : The value of quantify.
  * @retval None
  */
void ll_fadc_set_average(FADC_TypeDef             *p_fadc, 
                         u16                       chn, 
                         TYPE_ENUM_LL_FADC_AVERAGE average_val)
{
    TX_ASSERT(p_fadc == FADC);

    p_fadc->FADCPPROC0CON[chn] &= ~(LL_FADC_AVERAGE_SEL(0x03));
    p_fadc->FADCPPROC0CON[chn] |= LL_FADC_AVERAGE_SEL(average_val);
}

/**
  * @brief  Set the value of downsample for fadc.
  * @param  p_fadc        : Select the initialized FADC group pointer
  * @param  chn           : Set the number of chn.
  * @param  downsample_val: The value of downsample.
  * @retval None
  */
void ll_fadc_set_downsample(FADC_TypeDef                *p_fadc, 
                            u16                          chn, 
                            TYPE_ENUM_LL_FADC_DOWNSAMPLE downsample_val)
{
    TX_ASSERT(p_fadc == FADC);

    p_fadc->FADCPPROC0CON[chn] &= ~(LL_FADC_DOMNSAMPLE_SEL(0x07));
    p_fadc->FADCPPROC0CON[chn] |= LL_FADC_DOMNSAMPLE_SEL(downsample_val);
}

/**
  * @brief  Set the value of timer peroid for fadc.
  * @param  p_fadc          : Select the initialized FADC group pointer
  * @param  chn             : Set the number of chn.
  * @param  timer_period_val: The value of timer peroid.
  * @retval None
  */
void ll_fadc_set_timer_peroid(FADC_TypeDef *p_fadc, u16 chn, u32 timer_period_val)
{
    u32 timer_con  = ll_fadc_rd_sfr(__ll_fadc_sfr_soc_timer[chn]); 
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(timer_period_val < 262143);
    
    timer_con |= LL_FADC_TIMER(timer_period_val);
    ll_fadc_wr_sfr(__ll_fadc_sfr_soc_timer[chn], timer_con);
}

/**
  * @brief  Enable the timer module of the fadc.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : Set the number of chn.
  * @retval None
  */
void ll_fadc_timer_enable(FADC_TypeDef *p_fadc, u16 chn)
{
    u32 timer_con = ll_fadc_rd_sfr(__ll_fadc_sfr_soc_timer[chn]); 
    TX_ASSERT(p_fadc == FADC);
    
    timer_con |= LL_FADC_TIMER_EN(1);
    ll_fadc_wr_sfr(__ll_fadc_sfr_soc_timer[chn], timer_con);
}

/**
  * @brief  Disable the timer module of the fadc.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : Set the number of chn.
  * @retval None
  */
void ll_fadc_timer_disable(FADC_TypeDef *p_fadc, u16 chn)
{
    u32 timer_con = ll_fadc_rd_sfr(__ll_fadc_sfr_soc_timer[chn]); 
    TX_ASSERT(p_fadc == FADC);
    
    timer_con &= ~(LL_FADC_TIMER_EN(0));
    ll_fadc_wr_sfr(__ll_fadc_sfr_soc_timer[chn], timer_con);
}

/**
  * @brief  FADC module write SFR register function
  * @param  addr: The address of the SFR register.
  * @param  data: The value of the SFR register.
  * @retval None
  */
void ll_fadc_wr_sfr(u8 addr, u32 data)
{
    while(FADC->FADCACSCON & LL_FADC_ON);
    FADC->FADCACSDAT = data;
    FADC->FADCACSCON = LL_FADC_ACS_ADDR(addr) | LL_FADC_RW | LL_FADC_ON;
}

/**
  * @brief  FADC module read SFR register function
  * @param  addr: The address of the SFR register.
  * @retval The value of the SFR register.
  */
u32 ll_fadc_rd_sfr(u8 addr)
{
    while(FADC->FADCACSCON & LL_FADC_ON);
    FADC->FADCACSCON = (LL_FADC_ACS_ADDR(addr) & (~LL_FADC_RW)) | LL_FADC_ON;
    while(FADC->FADCACSCON & LL_FADC_ON);
    return FADC->FADCACSDAT;
}

/**
  * @brief  Low layer synchronous sampling enable function of fadc.
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  chn   : Set the number of synchronous sampling channel.
  * @retval None
  */
void ll_fadc_sync_sample_enable(FADC_TypeDef *p_fadc, u8 chn)
{
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(chn <= 6);
    
    u32 soc_con8 = ll_fadc_rd_sfr(FADCSFRSOCCON8);
    ll_fadc_wr_sfr(FADCSFRSOCCON8, soc_con8|LL_FADC_SIMULEN(BIT(chn)));
}

/**
  * @brief  Low layer synchronous sampling disable function of fadc.
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  chn   : Set the number of synchronous sampling channel.
  * @retval None
  */
void ll_fadc_sync_sample_disable(FADC_TypeDef *p_fadc, u8 chn)
{
    TX_ASSERT(p_fadc == FADC);
    TX_ASSERT(chn <= 6);
    
    u32 soc_con8 = ll_fadc_rd_sfr(FADCSFRSOCCON8);
    ll_fadc_wr_sfr(FADCSFRSOCCON8, soc_con8&(~(LL_FADC_SIMULEN(BIT(chn)))));
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
