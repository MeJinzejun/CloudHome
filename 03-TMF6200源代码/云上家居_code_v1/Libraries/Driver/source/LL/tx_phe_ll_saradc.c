/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_saradc.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    04-08-2019
  * @brief   This file contains all the SARADC LL firmware functions.
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
#include "tx_phe_ll_saradc.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup saradc_interface_gr SARADC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup SARADC_LL_Driver SARADC LL Driver
  * @ingroup  saradc_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup SARADC_LL_Interrupt SARADC LL Interrupt Handle function
  * @ingroup  SARADC_LL_Driver
  * @brief   SARADC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup SARADC_LL_Inti_Cfg SARADC LL Initialization And Configuration
  * @ingroup  SARADC_LL_Driver
  * @brief    SARADC LL Initialization And Configuration
  * @{
  */

/** 
  * @brief  SARADC init function. reset & enable saradc
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_init  : pointer to TYPE_LL_SARADDA_INIT SARADC init struct
  * @retval None.
  */
void ll_saradc_init(ADC_TypeDef *p_saradc, TYPE_LL_SARADDA_INIT *p_init)
{    
   TX_ASSERT(p_saradc == ADC);

   /* enable adc clk */
   ll_cc_rst_apb2_peripheral_clk_enable(LL_CC_RST_APB2_M_SARADC);

   /* saradc module reset release */
   ll_cc_rst_softreset_release(LL_CC_RST_SOFTRST_M_SARADC);

   p_saradc->SARADC_CON = 0; 
   p_saradc->SARADC_CON1 = 0; 
   p_saradc->SARADC_INT_CONTROL0 = 0;
   p_saradc->SARADC_INT_CONTROL1 = 0;
   p_saradc->SARADC_INT_CONTROL2 = 0;   
   p_saradc->SARADC_PENDING0_CLR = BIT(LL_SARADC_CHN_MAX)-1;
   p_saradc->SARADC_PENDING1_CLR = BIT(LL_SARADC_ADC_CHN_MAX)-1;
   p_saradc->SARADC_PENDING2_CLR = BIT(LL_SARADC_ADC_CHN_MAX)-1;
   
   p_saradc->SARADC_COM_ACFG = LL_SARADC_SARVREF_EN(0x3)         |
                               LL_SARADC_CLK_DIV(p_init->clk_div);

}

/** 
  * @brief  SARADC deinit function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @retval none
  */
void ll_saradc_deinit(ADC_TypeDef *p_saradc)
{
    TX_ASSERT(p_saradc == ADC);
    ll_saradc_stop(p_saradc, BIT(LL_SARADC_CHN_MAX)-1);

    /* saradc module reset */
    ll_cc_rst_softreset(LL_CC_RST_SOFTRST_M_SARADC);

    /* enable adc clk */
    ll_cc_rst_apb2_peripheral_clk_disable(LL_CC_RST_APB2_M_SARADC);

}  

/** 
  * @brief  SARADC irq config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : @TYPE_LL_SARADC_IRQ_CFG config interrupt enable
  * @retval None.
  */
u32 ll_saradc_irq_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_IRQ_CFG *p_cfg)
{
    TX_ASSERT(p_saradc == ADC);
    
    TX_ASSERT(p_cfg->chn < LL_SARADC_CHN_MAX);

    u32 chn_bit = BIT(p_cfg->chn);

    if(p_cfg->sample_intr_en){
        p_saradc->SARADC_INT_CONTROL0 |= chn_bit;
    } else {
        p_saradc->SARADC_INT_CONTROL0 &= ~chn_bit;
    }
    if(p_cfg->dma_half_intr_en){
        p_saradc->SARADC_INT_CONTROL1 |= chn_bit;
    } else {
        p_saradc->SARADC_INT_CONTROL1 &= ~chn_bit;
    }
    if(p_cfg->dma_full_intr_en){
        p_saradc->SARADC_INT_CONTROL2 |= chn_bit;
    } else {
        p_saradc->SARADC_INT_CONTROL2 &= ~chn_bit;
    }

    if(p_cfg->chn >= LL_SARADC_ADC_CHN_MAX){
        if(p_cfg->dac_intr_en){
            p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] |= LL_SARADC_DAC_INT_EN;
        } else {
            p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] &= ~LL_SARADC_DAC_INT_EN;
        }    
        
        if(p_cfg->cmp_intr_en){
            p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] |= LL_SARADC_CMP_INT_EN;
        } else {
            p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] &= ~LL_SARADC_CMP_INT_EN;
        }      
    }

    return 1;
}

/** 
  * @brief  SARADC ll_saradc_cmp_out_io_config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify chn to set: LL_SARADC_ADC_CHN_MAX~LL_SARADC_CHN_MAX
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_cmp_out_io_config(ADC_TypeDef *p_saradc, u8 chn)
{
    if((chn > LL_SARADC_CHN_MAX) || (chn < LL_SARADC_ADC_CHN_MAX)){
        return 0;
    }
    
    ll_gpio_dir(GPIOD, BIT(chn - LL_SARADC_ADC_CHN_MAX), LL_GPIO_PIN_OUTPUT);
    SYSCTRL_REG_OPT(
        SYSCTRL->SYS_CON0 |= BIT(chn - LL_SARADC_ADC_CHN_MAX + 4);
    );
    return 1;
}


/** 
  * @brief  SARADC config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify chn to set: LL_SARADC_ADC_CHN_MAX~LL_SARADC_CHN_MAX
  * @param  aux_sel : @TYPE_ENUM_LL_SARADDA_AUX_SEL
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_aux_in_io_config(ADC_TypeDef *p_saradc, u8 chn, TYPE_ENUM_LL_SARADDA_AUX_SEL aux_sel)
{
    if((chn > LL_SARADC_CHN_MAX) || (chn < LL_SARADC_ADC_CHN_MAX)){
        return 0;
    }
    
    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    switch(chn) {
        case 14: {
            switch(aux_sel) {
                case LL_SARADDA_AIN0: 
                    ll_gpio_dir(GPIOA, BIT(0), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(0), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(0);
                    break;
                
                case LL_SARADDA_AIN1: 
                    ll_gpio_dir(GPIOA, BIT(1), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(1), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(1);
                    break;
                
                case LL_SARADDA_AIN2: 
                    ll_gpio_dir(GPIOA, BIT(2), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(2), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(2);
                    break;
                
                case LL_SARADDA_AIN3: 
                    break;
                
                default:
                    break;
            }
            break;
        }
        case 15: {
            switch(aux_sel) {
                case LL_SARADDA_AIN0: 
                    ll_gpio_dir(GPIOA, BIT(4), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(4), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(4);
                    break;
                
                case LL_SARADDA_AIN1: 
                    ll_gpio_dir(GPIOA, BIT(5), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(5), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(5);
                    break;
                
                case LL_SARADDA_AIN2: 
                    ll_gpio_dir(GPIOA, BIT(6), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(6), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(6);
                    break;
                
                case LL_SARADDA_AIN3: 
                    break;
                
                default:
                    break;
            }
            break;
        }            
        case 16: {
            switch(aux_sel) {
                case LL_SARADDA_AIN0: 
                    ll_gpio_dir(GPIOA, BIT(8), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(8), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(8);
                    break;
                
                case LL_SARADDA_AIN1: 
                    ll_gpio_dir(GPIOA, BIT(9), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(9), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(9);
                    break;
                
                case LL_SARADDA_AIN2: 
                    ll_gpio_dir(GPIOA, BIT(10), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(10), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(10);
                    break;
                
                case LL_SARADDA_AIN3: 
                    break;
                
                default:
                    break;
            }
            break;
        }            
        case 17: {
            switch(aux_sel) {
                case LL_SARADDA_AIN0: 
                    ll_gpio_dir(GPIOA, BIT(12), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(12), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(12);
                    break;
                
                case LL_SARADDA_AIN1: 
                    ll_gpio_dir(GPIOA, BIT(13), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(13), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(13);
                    break;
                
                case LL_SARADDA_AIN2: 
                    ll_gpio_dir(GPIOA, BIT(14), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(14), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(14);
                    break;
                
                case LL_SARADDA_AIN3: 
                    ll_gpio_dir(GPIOA, BIT(15), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOA, BIT(15), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(15);
                    break;
                
                default:
                    break;
            }
            break;
        }            
        case 18: {
            switch(aux_sel) {
                case LL_SARADDA_AIN0: 
                    ll_gpio_dir(GPIOD, BIT(1), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOD, BIT(1), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(17);
                    break;
                
                case LL_SARADDA_AIN1: 
                    ll_gpio_dir(GPIOD, BIT(2), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOD, BIT(2), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(18);
                    break;
                
                case LL_SARADDA_AIN2: 
                    ll_gpio_dir(GPIOD, BIT(3), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOD, BIT(3), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(19);
                    break;
                
                case LL_SARADDA_AIN3: 
                    ll_gpio_dir(GPIOD, BIT(4), LL_GPIO_PIN_INPUT);
                    ll_gpio_pull(GPIOD, BIT(4), LL_GPIO_PULL_NONE);
                    SYSCTRL->SYS_CON7 |= BIT(20);
                    break;
                
                default:
                    break;
            }
            break;
        }            
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();

    return 1;
}


/** 
  * @brief  SARADC config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : @TYPE_LL_SARADDA_CFG config adda sample_rate, int, average, downsample, dc_offset, gain, 
                      quantify, dma, compare, dac, ect.
  * @retval None.
  */
u32 ll_saradc_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_CFG *p_cfg)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(p_cfg->chn < LL_SARADC_CHN_MAX);
    
    if(p_cfg->dma_len) {
        TX_ASSERT_ADDR(p_cfg->dma_addr, (p_cfg->dma_len),
                       SRAM_SARADC_DMA_MASK, SRAM_SARADC_DMA_ALIGN_MASK);
    }
    u32 chn_bit = BIT(p_cfg->chn);
    u32 temp;

    if(p_cfg->chn < LL_SARADC_ADC_CHN_MAX) {
        if(p_cfg->dma_len){
            p_saradc->SARADC_CON1 |= chn_bit;
            p_saradc->SARADC_DMASTADDR[p_cfg->chn] = p_cfg->dma_addr;
        } else {
            p_saradc->SARADC_CON1 &= ~chn_bit;
        }
        
        temp = LL_SARADC_QUANTIFY_COEF(p_cfg->quantify_coef)                       |
               LL_SARADC_QUANTIFY_DECIMAL_WIDTH(p_cfg->quantify_out_decimal_width) |
               (p_cfg->quantify_en ? LL_SARADC_QUANTIFY_EN : 0);
        p_saradc->SARADC_QUANTIFY_CON[p_cfg->chn] = temp;

        temp = LL_SARADC_DOWNSAMPLE(p_cfg->downsample)                             |
               LL_SARADC_DC_OFFSET(p_cfg->dc_offset)                               |
               LL_SARADC_GAIN(p_cfg->gain)                                         |
               LL_SARADC_AVERAGE(p_cfg->average)                                   |
               ((p_cfg->average == LL_SARADC_AVERAGE_1) ? 0 : LL_SARADC_AVERAGE_EN);
        p_saradc->SARADC_ACFG[p_cfg->chn] = temp | LL_SARADC_PGA_EN | LL_SARADC_ANALOG_EN;

        temp = LL_SARADC_CH_DMA_LEN(p_cfg->dma_len)                                       |
               LL_SARADC_CH_DIV(p_cfg->sample_rate_div)                                   |
               (p_cfg->adc_kick_by_extern_en ? LL_SARADC_CH_DIV_DIS : LL_SARADC_CH_DIV_EN);
        p_saradc->SARADC_CDIV_DMALEN[p_cfg->chn] = temp;

        ll_saradc_set_dc_offset(p_saradc, p_cfg->chn, p_cfg->dc_offset);
        if(p_cfg->quantify_en) {
            ll_saradc_set_quantify(p_saradc, p_cfg->chn, p_cfg->quantify_coef, p_cfg->quantify_out_decimal_width);
        } else {
            ll_saradc_set_quantify(p_saradc, p_cfg->chn, 0x8000, 0);
        }
        
    } else {
        ll_saradc_aux_in_io_config(p_saradc, p_cfg->chn, p_cfg->aux_sel);
    
        temp = LL_SARADC_DOWNSAMPLE(p_cfg->downsample)                             |
               LL_SARADC_DC_OFFSET(p_cfg->dc_offset)                               |
               LL_SARADC_AUX_EN(p_cfg->aux_sel)                                    |
               LL_SARADC_AVERAGE(p_cfg->average)                                   |
               ((p_cfg->average == LL_SARADC_AVERAGE_1) ? 0 : LL_SARADC_AVERAGE_EN);
        p_saradc->SARADC_ACFG[p_cfg->chn] = temp | LL_SARADC_ANALOG_EN;

        temp = LL_SARADC_CH_DIV(p_cfg->sample_rate_div)                                   |
               (p_cfg->adc_kick_by_extern_en ? LL_SARADC_CH_DIV_DIS : LL_SARADC_CH_DIV_EN);
        p_saradc->SARADC_CDIV_DMALEN[p_cfg->chn] = temp;
    }

    p_saradc->SARADC_START_POINT[p_cfg->chn] = p_cfg->start_point;
    
    return 1;
}

/** 
  * @brief  SARADC dac config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : @TYPE_LL_SARADC_CMP_CFG config
  * @retval None.
  */
u32 ll_saradc_cmp_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_CMP_CFG *p_cfg)
{
    TX_ASSERT((p_cfg->chn >= LL_SARADC_ADC_CHN_MAX) && (p_cfg->chn < LL_SARADC_CHN_MAX));

    u32 temp;
    
    ll_saradc_aux_in_io_config(p_saradc, p_cfg->chn, p_cfg->aux_sel);
    ll_saradc_cmp_out_io_config(p_saradc, p_cfg->chn);
           
    p_saradc->SARADC_CDIV_DMALEN[p_cfg->chn] = LL_SARADC_CH_DIV(0) | LL_SARADC_CH_DIV_EN;
    
    temp = ((p_cfg->cmp_des_sel == LL_SARADC_CMP_TO_IO_AND_CPU) ? 0 : 0)                                                       |
           ((p_cfg->cmp_des_type_sel_io & LL_SARADC_CMP_OUT_DEBUNCE) ? LL_SARADC_CMPTOIO_SEL(1) : LL_SARADC_CMPTOIO_SEL(0))    |
           ((p_cfg->cmp_io_out_invert_en) ? LL_SARADC_CMPTOIO_INV_EN : (0))                                                       |
           ((p_cfg->cmp_des_type_sel_cpu & LL_SARADC_CMP_OUT_DEBUNCE) ? LL_SARADC_CMPTOCPU_SEL(1) : LL_SARADC_CMPTOCPU_SEL(0)) |
           ((p_cfg->cmp_cpu_out_invert_en) ? LL_SARADC_CMPTOCPU_INV_EN : (0))                                                      |
           LL_SARADC_DAC_DONE_CLR;

    temp |= (p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] & (LL_SARADC_CMP_INT_EN));
    
    p_saradc->DACCMP_CON[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] = temp;
        
    p_saradc->DACCMP_DEBOUNCE[p_cfg->chn - LL_SARADC_ADC_CHN_MAX] = LL_SARADC_CMP_DEBOUNCE(p_cfg->cmp_out_debunce_cnt);
    
    temp = LL_SARADC_AUX_EN(p_cfg->aux_sel);
         
    p_saradc->SARADC_ACFG[p_cfg->chn] = temp;

    return 1;
}


/** 
  * @brief  SARADC dac config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : @TYPE_LL_SARADC_DAC_CFG config
  * @retval None.
  */
u32 ll_saradc_dac_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_DAC_CFG *p_cfg)
{
    TX_ASSERT((p_cfg->chn >= LL_SARADC_ADC_CHN_MAX) && (p_cfg->chn < LL_SARADC_CHN_MAX));

    //enable system key
    system_critical_section_enter();
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;
    
    switch(p_cfg->chn) {
        case 14:
            ll_gpio_dir(GPIOA, BIT(3), LL_GPIO_PIN_INPUT);
            ll_gpio_pull(GPIOA, BIT(3), LL_GPIO_PULL_NONE);
            SYSCTRL->SYS_CON7 |= BIT(3);
            break;
        
        case 15:
            ll_gpio_dir(GPIOA, BIT(7), LL_GPIO_PIN_INPUT);
            ll_gpio_pull(GPIOA, BIT(7), LL_GPIO_PULL_NONE);
            SYSCTRL->SYS_CON7 |= BIT(7);
            break;
        
        case 16:
            ll_gpio_dir(GPIOA, BIT(11), LL_GPIO_PIN_INPUT);
            ll_gpio_pull(GPIOA, BIT(11), LL_GPIO_PULL_NONE);
            SYSCTRL->SYS_CON7 |= BIT(11);
            break;
        
        case 17:
            ll_gpio_dir(GPIOD, BIT(0), LL_GPIO_PIN_INPUT);
            ll_gpio_pull(GPIOD, BIT(0), LL_GPIO_PULL_NONE);
            SYSCTRL->SYS_CON7 |= BIT(16);
            break;
        
        case 18:
            ll_gpio_dir(GPIOD, BIT(5), LL_GPIO_PIN_INPUT);
            ll_gpio_pull(GPIOD, BIT(5), LL_GPIO_PULL_NONE);
            SYSCTRL->SYS_CON7 |= BIT(21);
            break;
        
        default:
            break;
    }
    
    //disable system key
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL;
    system_critical_section_exit();

    p_saradc->SARADC_CDIV_DMALEN[p_cfg->chn] = LL_SARADC_CH_DIV(p_cfg->updata_data_rate_div)                              | 
                                               (p_cfg->dac_kick_by_extern_en ? LL_SARADC_CH_DIV_DIS : LL_SARADC_CH_DIV_EN);
    
    p_saradc->SARADC_START_POINT[p_cfg->chn] = p_cfg->start_point;
    
    return 1;
}



/**
  * @}
  */

/** @defgroup SARADC_LL_Data_Transfers SARADC LL Data transfers functions
  * @ingroup  SARADC_LL_Driver
  * @brief    SARADC LL Data transfers functions 
  * @{
  */

/** 
  * @brief  SARADC start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to start, use bit field.
  * @retval None.
  * @note dmabuf is ringbuf, index will auto inc and reset to start at the end
  */
void ll_saradc_start(ADC_TypeDef *p_saradc, u32 chn_bits)
{    
    TX_ASSERT(p_saradc == ADC);
    p_saradc->SARADC_CON |= (chn_bits & (BIT(LL_SARADC_CHN_MAX)-1));
}

/** 
  * @brief  SARADC stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to stop, use bit field.
  * @retval None.
  */
void ll_saradc_stop(ADC_TypeDef *p_saradc, u32 chn_bits)
{  
    TX_ASSERT(p_saradc == ADC);
    p_saradc->SARADC_CON &= ~(chn_bits & (BIT(LL_SARADC_CHN_MAX)-1));
}

/** 
  * @brief  SARADC dac start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_dac_start(ADC_TypeDef *p_saradc, u32 chn)
{    
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));
    p_saradc->SARADC_ACFG[chn] |= LL_SARADC_DAC_EN;
    
}

/** 
  * @brief  SARADC dac stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_dac_stop(ADC_TypeDef *p_saradc, u32 chn)
{    
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));
    p_saradc->SARADC_ACFG[chn] &= ~LL_SARADC_DAC_EN;
}

/** 
  * @brief  SARADC cmp start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_cmp_start(ADC_TypeDef *p_saradc, u32 chn)
{    
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));
    p_saradc->SARADC_ACFG[chn] |= LL_SARADC_CMP_EN;
    
}

/** 
  * @brief  SARADC dac stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_cmp_stop(ADC_TypeDef *p_saradc, u32 chn)
{    
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));
    p_saradc->SARADC_ACFG[chn] &= ~LL_SARADC_CMP_EN;
}

/** 
  * @brief  SARADC saradc_dc_offset_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  offset  : specify adc chn dc_offset[0:11] to set
  * @retval None.
  */
void ll_saradc_set_dc_offset(ADC_TypeDef *p_saradc, u32 chn, u16 offset)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);

    u16 new_offset  = offset;
    
#if !__SARADC_TRIM_FORCE_OFF
    s16 add_offset, gain;

    if(chn < LL_SARADC_ADC_CHN_MAX) {
        /* Bx([1,12,11]) = B0 + B1/A0 (a0:gain, b0:offset) */
        TYPE_LL_EFLASH_CHIP_PARAM *p_chip_param = (TYPE_LL_EFLASH_CHIP_PARAM*)LL_EF_CHIP_PARAM_STADDR;
        gain = p_chip_param->saradc_cab[chn].gain;
        if((p_chip_param->crc32_saradc_cab != 0xFFFFFFFF) &&
           (gain != (s16)0xFFFF) ) {
            u32 decimal = (p_saradc->SARADC_QUANTIFY_CON[chn] & LL_SARADC_QUANTIFY_DECIMAL_WIDTH(0x7)) >> 1;
            add_offset = ((((s32)((s16)offset << 4)) << (15-decimal)) / gain);
            new_offset = ((s16)(p_chip_param->saradc_cab[chn].dc_offset << 0) + add_offset) >> 4;
        }
    }
#endif    
    p_saradc->SARADC_ACFG[chn] = ((p_saradc->SARADC_ACFG[chn]       &
                                 (~(LL_SARADC_DC_OFFSET(0xFFFF)))) |
                                 LL_SARADC_DC_OFFSET(new_offset));
}

/** 
  * @brief  SARADC saradc_quantify_set function.
  * @param  p_saradc         : pointer to ADC_TypeDef SARADC controller 
  * @param  chn              : specify adc chn to set
  * @param  coef             : specify adc chn coef to set, rang [0:32767] means [0:1)
  * @param  out_decimal_width: specify Quantify output data remain how many decimal width
  * @retval None.
  */
void ll_saradc_set_quantify(ADC_TypeDef *p_saradc, u8 chn, u16 coef, u8 out_decimal_width)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);
    
    u16 new_coef    = coef;
    u16 new_decimal = out_decimal_width;
    u32 quantify_en = 0;
    
#if !__SARADC_TRIM_FORCE_OFF
    if(chn < LL_SARADC_ADC_CHN_MAX) {
        /* Ax([1,16,14]) = A0 * A1 (a0:gain, b0:offset) */
        TYPE_LL_EFLASH_CHIP_PARAM *p_chip_param = (TYPE_LL_EFLASH_CHIP_PARAM*)LL_EF_CHIP_PARAM_STADDR;
        s16 gain = p_chip_param->saradc_cab[chn].gain;
        if((p_chip_param->crc32_saradc_cab != 0xFFFFFFFF) &&
           (gain != (s16)0xFFFF) ) {
            new_coef = ((u32)coef * gain) >> 15;
            new_decimal += 1;
            quantify_en = LL_SARADC_QUANTIFY_EN;
        }
    }
#endif
    p_saradc->SARADC_QUANTIFY_CON[chn] = p_saradc->SARADC_QUANTIFY_CON[chn]                                            &
                                         (~(LL_SARADC_QUANTIFY_COEF(0xFFFF) | LL_SARADC_QUANTIFY_DECIMAL_WIDTH(0x7)))  |
                                         LL_SARADC_QUANTIFY_COEF(new_coef)                                             |
                                         LL_SARADC_QUANTIFY_DECIMAL_WIDTH(new_decimal)                                 |
                                         quantify_en;
}

/** 
  * @brief  SARADC saradc_average_times_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  average : specify average times @TYPE_ADC_AVERAGE_SEL
  * @retval None.
  */
void ll_saradc_set_average(ADC_TypeDef *p_saradc, u8 chn, TYPE_ENUM_LL_SARADC_AVERAGE_SEL average)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);

    if(LL_SARADC_AVERAGE_1 != average){
        p_saradc->SARADC_ACFG[chn] = (p_saradc->SARADC_ACFG[chn] & ~(LL_SARADC_AVERAGE(3)))  |
                                     LL_SARADC_AVERAGE(average)                              | 
                                     LL_SARADC_AVERAGE_EN;
    } else {
        p_saradc->SARADC_ACFG[chn] = p_saradc->SARADC_ACFG[chn] & (~LL_SARADC_AVERAGE_EN);
    }
}

/** 
  * @brief  SARADC saradc_down_sample_times_set function.
  * @param  p_saradc  : pointer to ADC_TypeDef SARADC controller 
  * @param  chn       : specify adc chn to set
  * @param  downsample: @TYPE_ADC_DOWN_SAMPLE_SEL specify down_sample times
  * @retval None.
  */
void ll_saradc_set_downsample(ADC_TypeDef *p_saradc, u8 chn, TYPE_ENUM_LL_SARADC_DOWNSAMPLE_SEL downsample)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);

    p_saradc->SARADC_ACFG[chn] = (p_saradc->SARADC_ACFG[chn] & ~(LL_SARADC_DOWNSAMPLE(7))) |
                                 LL_SARADC_DOWNSAMPLE(downsample);
}

/** 
  * @brief  SARADC saradc_sample_rate_get function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to get
  * @retval divor read from reg, sample_rate = sys_clk/divor
  */
u16 ll_saradc_get_sample_rate(ADC_TypeDef *p_saradc, u8 chn)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);

    return (u16)(p_saradc->SARADC_CDIV_DMALEN[chn] >> 16);
}

/** 
  * @brief  SARADC saradc_sample_rate_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  divor   : = sys_clk/sample_rate
  * @retval divor read from reg.
  */
u16 ll_saradc_set_sample_rate(ADC_TypeDef *p_saradc, u8 chn, u16 divor)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT(chn < LL_SARADC_CHN_MAX);

    p_saradc->SARADC_CDIV_DMALEN[chn] = ((u32)divor << 16)                          |
                                        (p_saradc->SARADC_CDIV_DMALEN[chn] & 0xFFFF);
    
    return (p_saradc->SARADC_CDIV_DMALEN[chn] >> 16);
}

/** 
  * @brief  SARADC ll_saradc_set_cmp_data function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  data0    : 12bit unsigned [0,3.3v] 
  * @param  data1    : 12bit unsigned [0,3.3v]
  * @retval none
  * @note   the compare use Hysteresis window control [dat_lit, dat_big] 
  *                                                      
  *             dat_lit<------^--------1
  *                   |       |
  *         0---------V------>dat_big
  */
void ll_saradc_set_cmp_data(ADC_TypeDef *p_saradc, u8 chn, u16 data0, u16 data1)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));

    if(data0 < data1) {
        p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA0(0xFFF))) |
                                                             LL_SARADC_CMP_DATA0(data1);
        p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA1(0xFFF))) |
                                                             LL_SARADC_CMP_DATA1(data0);
    
    } else {
        p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA0(0xFFF))) |
                                                             LL_SARADC_CMP_DATA0(data0);
        p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA1(0xFFF))) |
                                                             LL_SARADC_CMP_DATA1(data1);
    }
}

/** 
  * @brief  SARADC ll_saradc_set_dac_data function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  data    : 12bit valid
  * @retval none
  */
void ll_saradc_set_dac_data(ADC_TypeDef *p_saradc, u8 chn, s16 data)
{
    TX_ASSERT(p_saradc == ADC);
    TX_ASSERT((chn >= LL_SARADC_ADC_CHN_MAX) && (chn < LL_SARADC_CHN_MAX));

#if 0
    p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA0(0xFFF))) |
                                                         LL_SARADC_CMP_DATA0(data);
    p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = (p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] & ~(LL_SARADC_CMP_DATA1(0xFFF))) |
                                                         LL_SARADC_CMP_DATA1(data);
#else
    p_saradc->DACCMP_DATA[chn - LL_SARADC_ADC_CHN_MAX] = LL_SARADC_CMP_DATA0(data) | LL_SARADC_CMP_DATA1(data);
#endif
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
