/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/source/txf6200.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    08-04-2019
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *          This file contains the system clock configuration for Phoenix devices,
  *          and is generated by the clock configuration tool.
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

#if defined(TXF6200)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
  

/** @brief   Peripheral register structure definition
  * @{
  */

NO_INIT USED IIC_TypeDef      ___IIC0__    AT(IIC0_BASE);
NO_INIT USED IIC_TypeDef      ___IIC1__    AT(IIC1_BASE);
NO_INIT USED USART_TypeDef    ___USART0__  AT(USART0_BASE);
NO_INIT USED USART_TypeDef    ___USART1__  AT(USART1_BASE);
NO_INIT USED USART_TypeDef    ___USART2__  AT(USART2_BASE);
NO_INIT USED SPI_TypeDef      ___SPI0__    AT(SPI0_BASE);
NO_INIT USED SPI_TypeDef      ___SPI1__    AT(SPI1_BASE);
NO_INIT USED LVD_TypeDef      ___LVD__     AT(LVD_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER4__  AT(TIMER4_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER5__  AT(TIMER5_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER6__  AT(TIMER6_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER7__  AT(TIMER7_BASE);
NO_INIT USED HCC_TypeDef      ___HCC__     AT(HCC_BASE);
NO_INIT USED CAN_TypeDef      ___CAN__     AT(CAN_BASE);
NO_INIT USED QEI_TypeDef      ___QEI__     AT(QEI_BASE);
NO_INIT USED WDT_TypeDef      ___WDT__     AT(WDT_BASE);
NO_INIT USED WDT1_TypeDef     ___WDT1__    AT(WDT1_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER0__  AT(TIMER0_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER1__  AT(TIMER1_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER2__  AT(TIMER2_BASE);
NO_INIT USED TIMER_TypeDef    ___TIMER3__  AT(TIMER3_BASE);
NO_INIT USED EFLASH_TypeDef   ___EFLASH__  AT(EFLASH_BASE);
NO_INIT USED CRC_TypeDef      ___CRC__     AT(CRC_BASE);
NO_INIT USED EVSYS_TypeDef    ___EVSYS__   AT(EVSYS_BASE);
NO_INIT USED SVPWM_TypeDef    ___SVPWM__   AT(SVPWM_BASE);
NO_INIT USED FPLL0_TypeDef    ___FPLL0__   AT(FPLL0_BASE);
NO_INIT USED FPLL1_TypeDef    ___FPLL1__   AT(FPLL1_BASE);
NO_INIT USED SINCOS_TypeDef   ___SINCOS0__ AT(SINCOS0_BASE);
NO_INIT USED SINCOS_TypeDef   ___SINCOS1__ AT(SINCOS1_BASE);
NO_INIT USED RMS_TypeDef      ___RMS0__    AT(RMS0_BASE);
NO_INIT USED RMS_TypeDef      ___RMS1__    AT(RMS1_BASE);
NO_INIT USED RMS_TypeDef      ___RMS2__    AT(RMS2_BASE);
NO_INIT USED MATRIX_TypeDef   ___MATRIX__  AT(MATRIX_BASE);
NO_INIT USED SPWM_TypeDef     ___SPWM__    AT(SPWM_BASE);
NO_INIT USED FFT_TypeDef      ___FFT0__    AT(FFT0_BASE);
NO_INIT USED FFT_TypeDef      ___FFT1__    AT(FFT1_BASE);
NO_INIT USED FFT_TypeDef      ___FFT2__    AT(FFT2_BASE);
NO_INIT USED DFT_TypeDef      ___DFTRAN0__ AT(DFTRAN0_BASE);
NO_INIT USED DFT_TypeDef      ___DFTRAN1__ AT(DFTRAN1_BASE);
NO_INIT USED DFT_TypeDef      ___DFTRAN2__ AT(DFTRAN2_BASE);
NO_INIT USED ARCTAN_TypeDef   ___ARCTAN0__ AT(ARCTAN0_BASE);
NO_INIT USED ARCTAN_TypeDef   ___ARCTAN1__ AT(ARCTAN1_BASE);
NO_INIT USED ARCTAN_TypeDef   ___ARCTAN2__ AT(ARCTAN2_BASE);
NO_INIT USED DATADMA_TypeDef  ___DATADMA__ AT(DATADMA_BASE);
NO_INIT USED ADC_TypeDef      ___ADC__     AT(SARADC_BASE);
NO_INIT USED IIR_TypeDef      ___IIR0__    AT(IIR0_BASE);
NO_INIT USED IIR_TypeDef      ___IIR1__    AT(IIR1_BASE);
NO_INIT USED IIR_TypeDef      ___IIR2__    AT(IIR2_BASE);
NO_INIT USED FIR_TypeDef      ___FIR0__    AT(FIR0_BASE);
NO_INIT USED FIR_TypeDef      ___FIR1__    AT(FIR1_BASE);
NO_INIT USED FIR_TypeDef      ___FIR2__    AT(FIR2_BASE);
NO_INIT USED DMAC_TypeDef     ___DMAC__    AT(DMAC_BASE);
NO_INIT USED GMAC_TypeDef     ___GMAC__    AT(GMAC_BASE);
NO_INIT USED GPIO_TypeDef     ___GPIOA__   AT(GPIOA_BASE);
NO_INIT USED GPIO_TypeDef     ___GPIOB__   AT(GPIOB_BASE);
NO_INIT USED GPIO_TypeDef     ___GPIOC__   AT(GPIOC_BASE);
NO_INIT USED GPIO_TypeDef     ___GPIOD__   AT(GPIOD_BASE);
NO_INIT USED SYSCTRL_TypeDef  ___SYSCTRL__ AT(SYSCTRL_BASE);
NO_INIT USED ADVTMR_TypeDef   ___ADVTMR0__ AT(ADVTMR0_BASE);
NO_INIT USED EPWM_TypeDef     ___EPWM__    AT(EPWM_BASE);
NO_INIT USED FADC_TypeDef     ___FADC__    AT(FADC_BASE);

/**
  * @}
  */ 
#endif

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
