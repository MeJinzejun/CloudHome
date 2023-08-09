/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_fadc.h
  * @author  HUGE-IC Application Team
  * @version V1.0.2
  * @date    04-08-2019
  * @brief   This file contains all the FADC LL firmware functions.
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
#ifndef __TX_PHE_LL_FADC_H
#define __TX_PHE_LL_FADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup fadc_interface_gr FADC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup FADC_LL_Driver FADC LL Driver
  * @ingroup  fadc_interface_gr
  * @brief Mainly the driver part of the FADC module, which includes \b FADC \b Register 
  * \b Constants, \b FADC \b Exported \b Constants, \b FADC \b Exported \b Struct, \b FADC
  * \b Data \b transfers \b functions, \b FADC \b Initialization \b and \b FADC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 
/** @defgroup FADC_LL_Register_Constants FADC LL Register Constants
  * @ingroup  FADC_LL_Driver
  * @brief    FADC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the FADC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the FADC register, mainly for convenience. Understand the 
    configuration of the FADC.
    
@endverbatim
  *
  * @{
  */

/***** FADCCON0 *****/
/*! ADC system clock domain data path enable signal. 
 */
#define LL_FADC_CON0_EN                           (1UL << 0)


/***** FADCACSCON *****/
/*! Fadc special function register address. Sfr is in the adc high-speed clock domain,
 *  so it needs to synchronize the interface to operate. 
 */
#define LL_FADC_ACS_ADDR(n)                       (((n)&0x3F) << 2)
/*! The fadc special function register read and write operation selection bit.
 *  0: Read operation; 1: Write operation. 
 */
#define LL_FADC_RW                                (1UL << 1)
/*! Read and write work status bits. 0: Read and write idle; 1: Read and write busy. 
 */
#define LL_FADC_ON                                (1UL << 0)


/***** FADCACSDAT *****/
/*! The special register of the specified address is read and written according to the 
 *  value of acs_addr in FADCACSCON.
 *  Read operation: return the value of the corresponding special register;
 *  Write operation: Write data to a special register of the specified address. 
 */
#define LL_FADC_ACS_DATA(n)                       (((n)&0xFFFFFFFF) << 0)


/***** FADCINT0 *****/
/*! After the 0~15 ADC completes a sample conversion, the interrupt enable signal is 
 *  triggered, and each bit represents a path. 0: Off interrupt, 1: Open interrupt. 
 */
#define LL_FADC_SMP_DONE_INT_EN(n)                (((n)&0xFFFF) << 0)


/***** FADCINT1 *****/
/*! The 0~15 ADC DMA data FIFO is full, triggering the interrupt enable signal, and 
 *  each bit represents a path. 0: Off interrupt, 1: Open interrupt. 
 */
#define LL_FADC_DMA_FULL_INT_EN(n)                (((n)&0xFFFF) << 16)
/*! The 0~15 ADC DMA data FIFO is half full, triggering the interrupt enable signal, 
 *  and each bit represents a path. 0: Off interrupt, 1: Open interrupt. 
 */
#define LL_FADC_DMA_HALF_INT_EN(n)                (((n)&0xFFFF) << 0)


/***** FADCSMPFLAG *****/
/*! Clear the 0~15 smp_done_flag, write 1 clear flag, and each bit represents a path.
 */
#define LL_FADC_CLR_SMP_DONE_FLAG(n)              (((n)&0xFFFF) << 16)
/*! The 0~15 ADC completes the flag after a sample conversion, and each bit represents a path.
 */
#define LL_FADC_SMP_DONE_FLAG(n)                  (((n)&0xFFFF) << 0)


/***** FADCDMAFLAG0 *****/
/*! Clear the 0~15 smp_half_flag, write 1 clear flag, and each bit represents a path.
 */
#define LL_FADC_CLR_DMA_HALF_FLAG(n)              (((n)&0xFFFF) << 16)
/*! The 0~15 DMA data FIFO half flag bit. The size of the FIFO is determined by the 
 * register FADCDMALENn, and each bit represents a path. 
 */
#define LL_FADC_DMA_HALF_FLAG(n)                  (((n)&0xFFFF) << 0)


/***** FADCDMAFLAG1 *****/
/*! Clear the 0~15 smp_full_flag, write 1 clear flag, and each bit represents a path.
 */
#define LL_FADC_CLR_DMA_FULL_FLAG(n)              (((n)&0xFFFF) << 16)
/*! The 0~15 DMA data FIFO full flag bit. The size of the FIFO is determined by the 
 * register FADCDMALENn, and each bit represents a path. 
 */
#define LL_FADC_DMA_FULL_FLAG(n)                  (((n)&0xFFFF) << 0)


/***** FADCDMAADDR 0 ~ 15 *****/
/*! The Nth FADC DMA data FIFO start address, N means 0 ~ 15. 
 */
#define LL_FADC_DMA_ADDR(n)                       (((n)&0xFFFFFFFF) << 0)


/***** FADCDMALEN 0 ~ 15 *****/
/*! The Nth ADC data DMA is enabled. , N means 0 ~ 15.
 */
#define LL_FADC_DMA_EN                            (1UL << 13)
/*! The 0th ADC data DMA FIFO length, N means 0 ~ 15. 
 */
#define LL_FADC_DMA_LEN(n)                        (((n)&0x1FFF) << 0)


/***** FADCPPROC0CON 0 ~ 15 *****/
/*! Used for the 0th channel ADC data dc_offset elimination. The input data is 
 *  subtracted from this value as output data. 
 */
#define LL_FADC_DC_OFFSET(n)                      (((n)&0xFFF) << 6)
/*! Fadc sample extraction selection. 
 */
#define LL_FADC_DOMNSAMPLE_SEL(n)                 (((n)&0x7) << 3)
/*! Average selection of FADC data. 
 */
#define LL_FADC_AVERAGE_SEL(n)                    (((n)&0x3) << 1)
/*! The FADC data is enabled on average.
 */
#define LL_FADC_AVERAGE_EN                        (1UL << 0)


/***** FADCPPROC1CON 0 ~ 15 *****/
/*! The nth ADC data quantization coefficient. 
 */
#define LL_FADC_QUANTIFY_COEF(n)                  (((n)&0xFFFF) << 4)
/*! FADC data quantization bit width selection. 
 */
#define LL_FADC_DECIMAL_WIDTH(n)                  (((n)&0x7) << 1)
/*! FADC data quantization is enabled. 
 */
#define LL_FADC_QUANTIFY_EN                       (1UL << 0)


/***** FADCRES 0 ~ 7 *****/
/*! The conversion result of the first SOC. 
 */
#define LL_FADC_RES0(n)                           (((n)&0xFFFF) << 16)
/*! The conversion result of the 0th SOC.
 */
#define LL_FADC_RES1(n)                           (((n)&0xFFFF) << 0)


/***** FADCSFRANACON0 *****/                                    
/*! Select the ADC analog module part of the control bits using software 
 *  register control or hardware control. 
 */
#define LL_FADC_ANA_CTRL_SEL(i)                   (((i)&0x1) << 12)
/*!  */
#define LL_FADC_ANA_CMPBSEN(i)                    (((i)&0x1) << 11)
/*!  */
#define LL_FADC_ANA_TVS2(i)                       (((i)&0x1) << 10)
/*!  */
#define LL_FADC_ANA_TVS1(i)                       (((i)&0x1) << 9)
/*!  */
#define LL_FADC_ANA_TVS0(i)                       (((i)&0x1) << 8)
/*!  */
#define LL_FADC_ANA_CHSEL(i)                      (((i)&0x1) << 7)
/*!  */
#define LL_FADC_ANA_TRIM(i)                       (((i)&0x1) << 6)
/*!  */
#define LL_FADC_ANA_ENVTS(i)                      (((i)&0x1) << 5)
/*!  */
#define LL_FADC_ANA_ENSYNS(i)                     (((i)&0x1) << 4)
/*!  */
#define LL_FADC_ANA_CMPEN(i)                      (((i)&0x1) << 3)
/*!  */
#define LL_FADC_ANA_BUFFEN(i)                     (((i)&0x1) << 2)
/*!  */
#define LL_FADC_ANA_BIASEN(i)                     (((i)&0x1) << 1)
/*!  */
#define LL_FADC_ANA_ADCEN(i)                      (((i)&0x1) << 0)

 
/***** FADCSFRANACON1 *****/
/*!  */
#define LL_FADC_ANA_SIOB(i)                       (((i)&0x1) << 29)
/*!  */
#define LL_FADC_ANA_S2B(i)                        (((i)&0x1) << 28)
/*!  */
#define LL_FADC_ANA_S1B(i)                        (((i)&0x1) << 27)
/*!  */
#define LL_FADC_ANA_S0B(i)                        (((i)&0x1) << 26)
/*!  */
#define LL_FADC_ANA_VCMIBEN(i)                    (((i)&0x1) << 25)
/*!  */
#define LL_FADC_ANA_ENSHTB(i)                     (((i)&0x1) << 24)
/*!  */
#define LL_FADC_ANA_ENSHB(i)                      (((i)&0x1) << 23)
/*!  */
#define LL_FADC_ANA_ENS2DB(n)                     (((n)&0x7F) << 16)
/*!  */
#define LL_FADC_ANA_SIOA(i)                       (((i)&0x1) << 13)
/*!  */
#define LL_FADC_ANA_S2A(i)                        (((i)&0x1) << 12)
/*!  */
#define LL_FADC_ANA_S1A(i)                        (((i)&0x1) << 11)
/*!  */
#define LL_FADC_ANA_S0A(i)                        (((i)&0x1) << 10)
/*!  */
#define LL_FADC_ANA_VCMIAEN(i)                    (((i)&0x1) << 9)
/*!  */
#define LL_FADC_ANA_ENSHTA(i)                     (((i)&0x1) << 8)
/*!  */
#define LL_FADC_ANA_ENSHA(i)                      (((i)&0x1) << 7)
/*!  */
#define LL_FADC_ANA_ENS2DA(n)                     (((n)&0x7F) << 0)


/***** FADCSFRADCCON0 *****/
/*! Offset binary data is inverted.
 */
#define LL_FADC_DATINV_EN(i)                      (((i)&0x1) << 25)
/*! Adc data format selection.
 */
#define LL_FADC_DATFMT_SEL(i)                     (((i)&0x1) << 24)
/*! After the SOC is triggered, the channel switching delay.
 */
#define LL_FADC_SOC_ONDLY_TIME(n)                 (((n)&0x7F) << 17)
/*! The time required for a single ADC conversion, configuration  
 *  time = Tadc_clk* (convt_time+1). 
 */
#define LL_FADC_CONVT_TIME(n)                     (((n)&0x3F) << 11)
/*! The ADC simulation module starts up to normal settling time, configuration  
 *  time = Tadc_clk* (convt_time+1). 
 */
#define LL_FADC_STABLE_TIME(n)                    (((n)&0x1F) << 6)
/*! Polling mode polls the number of slots configured. 
 */
#define LL_FADC_POLLING_SLOTCON(n)                (((n)&0xF) << 2)
/*! Polling mode settings. 
 */
#define LL_FADC_POLLING_MODE(i)                   (((i)&0x1) << 1)
/*! The ADC conversion module is enabled. 
 */
#define LL_FADC_M_EN(i)                           (((i)&0x1) << 0)


/***** FADCSFRSOCCON 0 ~ 7 *****/
/*! The 2n+1th SOC trigger source selection.
 */ 
#define LL_FADC_TRISEL1(n)                        (((n)&0xF) << 27)
/*! The 2n+1th SOC sampling window time control. Configuration  
 *  time = Tadc_clk*(smp_win+1). 
 */ 
#define LL_FADC_SMP_WIN1(n)                       (((n)&0x7F) << 20)
/*! The analog input channel selection of the 2n+1th SOC.
 */
#define LL_FADC_CHSEL1(n)                         (((n)&0xF) << 16)
/*! The 2n SOC trigger source selection.
 */
#define LL_FADC_TRISEL0(n)                        (((n)&0xF) << 11)
/*! The 2nth SOC sampling window time control. Configuration  
 *  time = Tadc_clk*(smp_win+1). 
 */ 
#define LL_FADC_SMP_WIN0(n)                       (((n)&0x7F) << 4)
/*! The analog input channel selection of the 2nth SOC.
 */
#define LL_FADC_CHSEL0(n)                         (((n)&0xF) << 0)


/***** FADCSFRSOCCON8 *****/
/*! High priority SOC settings.
 */
#define LL_FADC_SOC_HPRI(n)                       (((n)&0x1F) << 24)
/*! Simultaneous sampling enable.
 */
#define LL_FADC_SIMULEN(n)                        (((n)&0xFF) << 16)
/*! The nth SOC trigger source is enabled. 
 */
#define LL_FADC_TRI_EN(n)                         (((n)&0xFFFF) << 0)


/***** FADCSFRSOCFLAG *****/
/*! CPU intervention forces the SOC to be triggered.
 */
#define LL_FADC_FC_TRI(n)                         (((n)&0xFFFF) << 16)
/*! The nth SOC is triggered and the conversion indication flag is not completed.
 */
#define LL_FADC_SOC_TRI(n)                        (((n)&0xFFFF) << 0)


/***** FADCSFRSOCOVFL *****/
/*! Clear the corresponding flag of soc_ovf and write 1 to clear.
 */
#define LL_FADC_CLR_SOC_OVF(n)                    (((n)&0xFFFF) << 16)
/*! The nth SOC is triggered and the conversion is not completed. At this time, 
 *  the SOC trigger is again caused, resulting in two conversion requests. Only 
 *  one data conversion occurs, and an overflow occurs. This flag is used to 
 *  indicate this situation.  
 *  0: no overflow  
 *  1: overflow occurs  
 */
#define LL_FADC_SOC_OVF(n)                        (((n)&0xFFFF) << 0)


/***** FADCSFRCALIB0 *****/
/*! The error estimation mode, the error of the final estimated parameter exceeds 
 *  the preset range, which is set to 1. 
 */
#define LL_FADC_ERR_OFR(i)                        (((i)&0x1) << 30)
/*! The error estimation mode estimates the completion flag.  
 *  0: Not completed  
 *  1: Completed  
 */
#define LL_FADC_CALIB_FLAG(i)                     (((i)&0x1) << 29)
/*! Clear the err_ofr flag and write 1 to clear.
 */
#define LL_FADC_ERR_OFR_CLR(i)                    (((i)&0x1) << 28)
/*! Clear the calib_flag flag and write 1 to clear.
 */
#define LL_FADC_CLR_CALIB_FLAG(i)                 (((i)&0x1) << 27)
/*! Error estimation mode work start signal,  
 *  0: Do not start  
 *  1: Start  
 */
#define LL_FADC_CALIB_TRI(i)                      (((i)&0x1) << 26)
/*! Estimated result check number control. 
 */
#define LL_FADC_RES_CHECK_CNT(n)                  (((n)&0xF) << 22)
/*! Error estimation mode, error estimation deviation control.
 */
#define LL_FADC_ERR_RANGE(n)                      (((n)&0x1F) << 17)
/*! The number of iterations is set.
 */
#define LL_FADC_ITER_TIME(n)                      (((n)&0x3F) << 11)
/*! Error estimation mode, iterative step selection, the default is 1/64.
 */
#define LL_FADC_WSTENSIZE(n)                      (((n)&0x3FF) << 1)
/*! ADC calibration module working mode selection,  
 *  0: error estimation mode  
 *  1: data calibration mode  
 */
#define LL_FADC_CALIB_WMODE(i)                    (((i)&0x1) << 0)


/***** FADCSFRCALIB1 *****/
/*! Data calibration mode, final data output selection,  
 *  0: Output the calibrated data  
 *  1: Output the data before calibration  
 */
#define LL_FADC_CALIB_NORM_OUTSEL(i)              (((i)&0x1) << 8)
/*! The error estimation mode presets the maximum value of the error of the 
 *  final estimated parameter. If the final estimated parameter exceeds the 
 *  preset value, it indicates that the error estimation result is not satisfactory. 
 */
#define LL_FADC_MAX_OFR_CON(n)                    (((n)&0xFF) << 0)


/***** FADCSFRWCOEF0 *****/
/*! Channel 0, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF0(n)                         (((n)&0x3FFF) << 0)


/***** FADCSFRWCOEF1 *****/
/*! Channel 1, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF1(n)                         (((n)&0x3FFF) << 0)


/***** FADCSFRWCOEF2 *****/
/*! Channel 2, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF2(n)                         (((n)&0x7FFF) << 0)


/***** FADCSFRWCOEF3 *****/
/*! Channel 3, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF3(n)                         (((n)&0x7FFF) << 0)


/***** FADCSFRWCOEF4 *****/
/*! Channel 4, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF4(n)                         (((n)&0xFFFF) << 0)


/***** FADCSFRWCOEF5 *****/
/*! Channel 5, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF5(n)                         (((n)&0x1FFFF) << 0)


/***** FADCSFRWCOEF6 *****/
/*! Channel 6, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF6(n)                         (((n)&0x3FFFF) << 0)


/***** FADCSFRWCOEF7 *****/
/*! Channel 7, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF7(n)                         (((n)&0x7FFFF) << 0)


/***** FADCSFRWCOEF8 *****/
/*! Channel 8, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF8(n)                         (((n)&0x7FFFF) << 0)


/***** FADCSFRWCOEF9 *****/
/*! Channel 9, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF9(n)                         (((n)&0xFFFFF) << 0)


/***** FADCSFRWCOEF10 *****/
/*! Channel 10, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF10(n)                        (((n)&0x1FFFFF) << 0)


/***** FADCSFRWCOEF11 *****/
/*! Channel 11, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF11(n)                        (((n)&0x3FFFFF) << 0)


/***** FADCSFRWCOEF12 *****/
/*! Channel 12, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF12(n)                        (((n)&0x7FFFFF) << 0)


/***** FADCSFRWCOEF13 *****/
/*! Channel 13, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF13(n)                        (((n)&0xFFFFFF) << 0)


/***** FADCSFRWCOEF14 *****/
/*! Channel 14, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF14(n)                        (((n)&0x1FFFFFF) << 0)


/***** FADCSFRWCOEF15 *****/
/*! Channel 15, error estimation mode, estimated parameters will be saved in this register 
 *  calibration mode, and the parameters obtained by the error estimation mode need to be 
 *  written to this register in advance. 
 */
#define LL_FADC_WCOEF15(n)                        (((n)&0x1FFFFFF) << 0)


/***** FADCSFRSOCTIMER 0 ~ 15 *****/
/*! Controls whether or not to generate using a counter.
 */
#define LL_FADC_TIMER_EN(i)                       (((i)&0x1) << 18)
/*! Generates a counter for the soc trigger source.
 */
#define LL_FADC_TIMER(n)                          (((n)&0x3FFFF) << 0)

/**
  * @}
  */

/** @defgroup FADC_LL_Exported_Constants FADC LL Exported Constants
  * @ingroup  FADC_LL_Driver
  * @brief    FADC LL external constant definition
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
/*! The default value of the FADC reference voltage. 
 */
#define LL_FADC_VREF_VAL                          (1.500)
/*! The default value of the FADC VCM voltage.
 */
#define LL_FADC_VCM_VAL                           (1.500)

/* Macro definition of the FADC high speed clock domain register. */
#define FADCSFRANACON0                            0x00
#define FADCSFRANACON1                            0x01
#define FADCSFRADCCON0                            0x02
#define FADCSFRSOCCON0                            0x03
#define FADCSFRSOCCON1                            0x04
#define FADCSFRSOCCON2                            0x05
#define FADCSFRSOCCON3                            0x06
#define FADCSFRSOCCON4                            0x07
#define FADCSFRSOCCON5                            0x08
#define FADCSFRSOCCON6                            0x09
#define FADCSFRSOCCON7                            0x0a
#define FADCSFRSOCCON8                            0x0b
#define FADCSFRSOCFLAG                            0x0c
#define FADCSFRSOCOVFL                            0x0d
#define FADCSFRCALIB0                             0x0e
#define FADCSFRCALIB1                             0x0f
#define FADCSFRWCOEF0                             0x10
#define FADCSFRWCOEF1                             0x11
#define FADCSFRWCOEF2                             0x12
#define FADCSFRWCOEF3                             0x13
#define FADCSFRWCOEF4                             0x14
#define FADCSFRWCOEF5                             0x15
#define FADCSFRWCOEF6                             0x16
#define FADCSFRWCOEF7                             0x17
#define FADCSFRWCOEF8                             0x18
#define FADCSFRWCOEF9                             0x19
#define FADCSFRWCOEF10                            0x1a
#define FADCSFRWCOEF11                            0x1b
#define FADCSFRWCOEF12                            0x1c
#define FADCSFRWCOEF13                            0x1d
#define FADCSFRWCOEF14                            0x1e
#define FADCSFRWCOEF15                            0x1f
#define FADCSFRSOCTIMER0                          0x20
#define FADCSFRSOCTIMER1                          0x21
#define FADCSFRSOCTIMER2                          0x22
#define FADCSFRSOCTIMER3                          0x23
#define FADCSFRSOCTIMER4                          0x24
#define FADCSFRSOCTIMER5                          0x25
#define FADCSFRSOCTIMER6                          0x26
#define FADCSFRSOCTIMER7                          0x27
#define FADCSFRSOCTIMER8                          0x28
#define FADCSFRSOCTIMER9                          0x29
#define FADCSFRSOCTIMER10                         0x2a
#define FADCSFRSOCTIMER11                         0x2b
#define FADCSFRSOCTIMER12                         0x2c
#define FADCSFRSOCTIMER13                         0x2d
#define FADCSFRSOCTIMER14                         0x2e
#define FADCSFRSOCTIMER15                         0x2f


/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for The FADC down sample selection
  */
typedef enum {
    /*! No extraction is performed and each sample is processed.
     */
    LL_FADC_DOWNSAMPLE_NONE = 0,
    /*! One out of every 2 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_2,
    /*! One out of every 3 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_3,
    /*! One out of every 4 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_4,
    /*! One out of every 5 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_5,
    /*! One out of every 6 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_6,
    /*! One out of every 7 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_7,
    /*! One out of every 8 sample values in the FADC is taken.
     */
    LL_FADC_DOWNSAMPLE_8,
} TYPE_ENUM_LL_FADC_DOWNSAMPLE;

/**
  * @brief Enumeration constant for The average selection of FADC data.
  */
typedef enum {
    /*! The 2 sample data were averaged.
     */
    LL_FADC_AVERAGE_2 = 0,
    /*! The 4 sample data were averaged.
     */
    LL_FADC_AVERAGE_4,
    /*! The 8 sample data were averaged.
     */
    LL_FADC_AVERAGE_8,
} TYPE_ENUM_LL_FADC_AVERAGE;

/**
  * @brief Enumeration constant for The FADC data quantization bit width selection.
  */
typedef enum {
    /*! The quantization bit width retains 0 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_0 = 0,
    /*! The quantization bit width retains 1 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_1,
    /*! The quantization bit width retains 2 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_2,
    /*! The quantization bit width retains 3 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_3,
    /*! The quantization bit width retains 4 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_4,
    /*! The quantization bit width retains 5 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_5,
    /*! The quantization bit width retains 6 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_6,
    /*! The quantization bit width retains 7 decimal places.
     */
    LL_FADC_DECIMAL_WIDTH_7,
} TYPE_ENUM_LL_FADC_DECI_W;

typedef enum {
    /*! Fadc analog pathway A group of SARADC0.
     */
    LL_FADC_CHNA_SARADC0 = 0,
    /*! Fadc analog pathway A group of SARADC1.
     */
    LL_FADC_CHNA_SARADC1,
    /*! Fadc analog pathway A group of SARADC2. 
     */
    LL_FADC_CHNA_SARADC2,
    /*! Fadc analog pathway A group of SARADC3. 
     */
    LL_FADC_CHNA_SARADC3,
    /*! Fadc analog pathway A group of SARADC4. 
     */
    LL_FADC_CHNA_SARADC4,
    /*! Fadc analog pathway A group of SARADC5. 
     */
    LL_FADC_CHNA_SARADC5,
    /*! Fadc analog pathway A group of SARADC6. 
     */
    LL_FADC_CHNA_SARADC6,        
    /*! Fadc analog pathway A group of pathway reserved. 
     */
    LL_FADC_CHNA_RESERVED,        
    /*! Fadc analog pathway B group of SARADC13. 
     */
    LL_FADC_CHNB_SARADC13,
    /*! Fadc analog pathway B group of SARADC12. 
     */
    LL_FADC_CHNB_SARADC12,
    /*! Fadc analog pathway B group of SARADC11. 
     */
    LL_FADC_CHNB_SARADC11,
    /*! Fadc analog pathway B group of SARADC10. 
     */
    LL_FADC_CHNB_SARADC10,
    /*! Fadc analog pathway B group of SARADC9. 
     */
    LL_FADC_CHNB_SARADC9,
    /*! Fadc analog pathway B group of SARADC8. 
     */
    LL_FADC_CHNB_SARADC8,
    /*! Fadc analog pathway B group of SARADC7. 
     */
    LL_FADC_CHNB_SARADC7,
    /*! Fadc analog pathway B group of pathway reserved. 
     */
    LL_FADC_CHNB_RESERVED,
} TYPE_ENUM_LL_FADC_ADC_SEL;


/**
  * @brief Enumeration constant for The Fadc trigger source selection.
  */
typedef enum {
    /*! The trigger source selection of fadc: no.
     */
    LL_FADC_TRI_NO = 0,
    /*! The trigger source selection of fadc: svpwm.
     */
    LL_FADC_TRI_SVPWM,
    /*! The trigger source selection of fadc: spwm and spwm0_a. 
     */
    LL_FADC_TRI_EPWM0_A_SPWM,
    /*! The trigger source selection of fadc: spwm1_a. 
     */
    LL_FADC_TRI_EPWM1_A,
    /*! The trigger source selection of fadc: spwm2_a. 
     */
    LL_FADC_TRI_EPWM2_A,
    /*! The trigger source selection of fadc: spwm3_a. 
     */
    LL_FADC_TRI_EPWM3_A,
    /*! The trigger source selection of fadc: spwm4_a. 
     */
    LL_FADC_TRI_EPWM4_A,
    /*! The trigger source selection of fadc: spwm5_a. 
     */
    LL_FADC_TRI_EPWM5_A,
    /*! The trigger source selection of fadc: spwm6_a. 
     */
    LL_FADC_TRI_EPWM6_A,
    /*! The trigger source selection of fadc: spwm0_b. 
     */
    LL_FADC_TRI_EPWM0_B,
    /*! The trigger source selection of fadc: spwm1_b. 
     */
    LL_FADC_TRI_EPWM1_B,
    /*! The trigger source selection of fadc: spwm2_b. 
     */
    LL_FADC_TRI_EPWM2_B,
    /*! The trigger source selection of fadc: spwm3_b. 
     */
    LL_FADC_TRI_EPWM3_B,
    /*! The trigger source selection of fadc: spwm4_b. 
     */
    LL_FADC_TRI_EPWM4_B,
    /*! The trigger source selection of fadc: spwm5_b. 
     */
    LL_FADC_TRI_EPWM5_B,
    /*! The trigger source selection of fadc: spwm6_b. 
     */
    LL_FADC_TRI_EPWM6_B,
} TYPE_ENUM_LL_FADC_TRI_SEL;

/**
  * @}
  */

/** @defgroup FADC_LL_Exported_Struct FADC LL Exported Struct
  * @ingroup  FADC_LL_Driver
  * @brief    FADC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the FADC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_fadc_init. Function, you can configure the FADC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief FADC low layer configuration structure
  */
typedef struct __ll_fadc_irq_cfg {
    /*! Chn is the one that is selected for calculation, ranging from 0 to 16. 
     */
    u8                           chn;
    /*! Enable an interrupt every time the sample is completed.
     */
    bool                         sample_intr_en;
    /*! Enables an interrupt when the DMA buffer reaches half.
     */
    bool                         dma_half_intr_en;
    /*! Enables an interrupt when the DMA buffer is full.
     */
    bool                         dma_full_intr_en;
} TYPE_FADC_IRQ_CFG;

/**
  * @brief FADC low layer configuration structure
  */
typedef struct __ll_fadc_cfg {
    /*! Chn is the one that is selected for calculation, ranging from 0 to 16. 
     */
    u8                           chn;
    /*! FADC DMA function enable bit.
     */
    bool                         dma_en;
    /*! FADC DMA length setting.
     */
    u16                          dma_len;
    /*! The starting address of the DMA input buffer, the address is 8Byte aligned. 
     *  @note FADC's dma_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3/SRAM10.  
     *        Statement example:
     *        static s16 fadc_buf[2048] __attribute__((aligned(8),section("SRAM0")));
     */
    u32                          dma_addr;
    /*! The ADC data dc_offset is eliminated and the value is subtracted from 
     *  the input data as output data. 
     */
    u16                          dc_offset;
    /*! For sample selection of FADC, please refer to enumeration TYPE_ENUM_LL_FADC_DOWNSMP. 
     */
    TYPE_ENUM_LL_FADC_DOWNSAMPLE downsample_sel;
    /*! FADC data average enable bit.
     */
    bool                         average_en;
    /*! Average selection of FADC data, please refer to enumeration TYPE_ENUM_LL_FADC_AVERAGE. 
     */
    TYPE_ENUM_LL_FADC_AVERAGE    average_sel;
    /*! FADC data quantization enable.
     */
    bool                         quantify_en;
    /*! Set the FADC data quantization coefficient, [1,16,15] stand for [0,1]. 
     */
    u16                          quantify_coef;
    /*! FADC data quantization bit width selection, please refer to enumeration TYPE_ENUM_LL_FADC_DECI_W. 
     */
    TYPE_ENUM_LL_FADC_DECI_W     decimal_width;
    
    /*! Fadc soc selects the external trigger source. For specific options, please refer to 
     *  the macro TYPE_ENUM_LL_FADC_TRI_SEL.
     *  @note The corresponding relationship is shown in the following table:
     *        ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
     *        ©¦ trigger_sel              ©¦   trigger module   ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_NO           ©¦        NO          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_SVPWM        ©¦       SVPWM        ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM0_A_SPWM ©¦  SPWM and EPWM0_a  ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM1_A      ©¦      EPWM1_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM2_A      ©¦      EPWM2_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM3_A      ©¦      EPWM3_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM4_A      ©¦      EPWM4_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM5_A      ©¦      EPWM5_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM6_A      ©¦      EPWM6_a       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM0_B      ©¦      EPWM0_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM1_B      ©¦      EPWM1_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM2_B      ©¦      EPWM2_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM3_B      ©¦      EPWM3_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM4_B      ©¦      EPWM4_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM5_B      ©¦      EPWM5_b       ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦ LL_FADC_TRI_EPWM6_B      ©¦      EPWM6_b       ©¦
     *        ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
     */
    TYPE_ENUM_LL_FADC_TRI_SEL    trigger_sel;
    /*! Select the analog path for fadc. There are a total of 14 analog channels, divided 
     *  into two A and B, each with 7 channels.
     *  @note The mapping relationship between the variable adc_chn_sel configuration item 
     *        and the actual chip's external pin map is as follows:
     *        ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
     *        ©¦  adc_chn_sel             ©¦ The external pin of the chip ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC0    ©¦         PIN_SARADC0          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC1    ©¦         PIN_SARADC1          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC2    ©¦         PIN_SARADC2          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC3    ©¦         PIN_SARADC3          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC4    ©¦         PIN_SARADC4          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC5    ©¦         PIN_SARADC5          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_SARADC6    ©¦         PIN_SARADC6          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNA_RESERVED   ©¦              NO              ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC13   ©¦         PIN_SARADC13         ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC12   ©¦         PIN_SARADC12         ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC11   ©¦         PIN_SARADC11         ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC10   ©¦         PIN_SARADC10         ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC9    ©¦         PIN_SARADC9          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC8    ©¦         PIN_SARADC8          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_SARADC7    ©¦         PIN_SARADC7          ©¦
     *        ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *        ©¦  LL_FADC_CHNB_RESERVED   ©¦              NO              ©¦
     *        ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
     */
    TYPE_ENUM_LL_FADC_ADC_SEL    adc_chn_sel;
    /*! Enable the timer inside the fadc. 
     */
    bool                         timer_en;
    /*! Configure the counting period of the timer inside the fadc. The timer is used to 
     *  trigger FADC, and the sampling rate is calculated as fadc_clk/timer_period.
     */
    u32                          timer_period;
} TYPE_LL_FADC_CFG;

/**
  * @brief FADC low layer Initialization structure
  */
typedef struct __ll_fadc_init {
    /*! Configure the division ratio of the FADC operating clock, Fadc and epwm share an adpll, 
     *  the default configuration is 180M.
     */
    u8  fadc_adpll_div_val;
    /*! FADC conversion high priority configuration.
     *  @note soc_hpri_val indicates that the priority of the configuration SOC0~SOC(soc_hpri_val-1) 
     *        is higher than SOCn~SOC15. If the configuration is 0, the default mode is used to 
     *        perform the round-robin conversion.
     */
    u8  soc_hpri_val;
} TYPE_LL_FADC_INIT;

/**
  * @}
  */

/** @defgroup FADC_LL_Interrupt FADC LL Interrupt Handle function
  * @brief   FADC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FADC  
    Interrupt Handle function.

    how to use?

    The FADC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the FADC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */


/**
  * @}
  */
  
/** @defgroup FADC_LL_Inti_Cfg FADC LL Initialization And Configuration
  * @brief    FADC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FADC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer FADC module initialization
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_FADC_INIT)
  * @retval None
  */
void ll_fadc_init(FADC_TypeDef *p_fadc, TYPE_LL_FADC_INIT *p_init);

/**
  * @brief  Low layer FADC module detele initialization
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @retval None
  */
void ll_fadc_deinit(FADC_TypeDef *p_fadc);

/**
  * @brief  Low layer FADC module irq configuration
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_cfg : Module irq configuration structure pointer(TYPE_FADC_IRQ_CFG)
  * @retval None
  */
void ll_fadc_irq_config(FADC_TypeDef *p_fadc, TYPE_FADC_IRQ_CFG *p_cfg);

/**
  * @brief  Low layer FADC module configuration
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  p_cfg : Module configuration structure pointer(TYPE_LL_FADC_DMA_CFG)
  * @retval None
  */
void ll_fadc_config(FADC_TypeDef *p_fadc, TYPE_LL_FADC_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup FADC_LL_Data_Transfers FADC LL Data transfers functions
  * @brief    FADC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the FADC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  FADC module start function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_start(FADC_TypeDef *p_fadc, u16 chn_bits);

/**
  * @brief  FADC module CPU force kick one time function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_kick_one(FADC_TypeDef *p_fadc, u16 chn_bits);

/**
  * @brief  FADC module stop function
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
void ll_fadc_stop(FADC_TypeDef *p_fadc, u16 chn_bits);

/**
  * @brief  Set the value of dc offset for fadc.
  * @param  p_fadc       : Select the initialized FADC group pointer
  * @param  chn          : Set the number of chn.
  * @param  dc_offset_val: The value of dc offset.
  * @retval None
  */
void ll_fadc_set_dc_offset(FADC_TypeDef *p_fadc, u16 chn, u16 dc_offset_val);

/**
  * @brief  Set the value of quantify for fadc.
  * @param  p_fadc          : Select the initialized FADC group pointer
  * @param  chn             : Set the number of chn.
  * @param  quantify_val    : The value of quantify.
  * @param  decimal_width   : specify Quantify output data remain how many decimal width rang:[0,6]
  * @retval None
  */
void ll_fadc_set_quantify(FADC_TypeDef *p_fadc, u16 chn, u16 quantify_val, u8 decimal_width);

/**
  * @brief  Set the value of quantify for fadc.
  * @param  p_fadc      : Select the initialized FADC group pointer
  * @param  chn         : Set the number of chn.
  * @param  average_val : The value of quantify.
  * @retval None
  */
void ll_fadc_set_average(FADC_TypeDef             *p_fadc, 
                         u16                       chn, 
                         TYPE_ENUM_LL_FADC_AVERAGE average_val);

/**
  * @brief  Set the value of downsample for fadc.
  * @param  p_fadc        : Select the initialized FADC group pointer
  * @param  chn           : Set the number of chn.
  * @param  downsample_val: The value of downsample.
  * @retval None
  */
void ll_fadc_set_downsample(FADC_TypeDef                *p_fadc, 
                            u16                          chn, 
                            TYPE_ENUM_LL_FADC_DOWNSAMPLE downsample_val);

/**
  * @brief  Set the value of timer peroid for fadc.
  * @param  p_fadc          : Select the initialized FADC group pointer
  * @param  chn             : Set the number of chn.
  * @param  timer_period_val: The value of timer peroid.
  * @retval None
  */
void ll_fadc_set_timer_peroid(FADC_TypeDef *p_fadc, u16 chn, u32 timer_period_val);

/**
  * @brief  Enable the timer module of the fadc.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : Set the number of chn.
  * @retval None
  */
void ll_fadc_timer_enable(FADC_TypeDef *p_fadc, u16 chn);

/**
  * @brief  Disable the timer module of the fadc.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : Set the number of chn.
  * @retval None
  */
void ll_fadc_timer_disable(FADC_TypeDef *p_fadc, u16 chn);

/**
  * @brief  Low layer synchronous sampling enable function of fadc.
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  chn   : Set the number of synchronous sampling channel.
  * @retval None
  */
void ll_fadc_sync_sample_enable(FADC_TypeDef *p_fadc, u8 chn);

/**
  * @brief  Low layer synchronous sampling disable function of fadc.
  * @param  p_fadc: The structure pointer of the FADC group (FADC0, FADC1, FADC2) is selected.
  * @param  chn   : Set the number of synchronous sampling channel.
  * @retval None
  */
void ll_fadc_sync_sample_disable(FADC_TypeDef *p_fadc, u8 chn);

/**
  * @brief  FADC module write SFR register function
  * @param  addr: The address of the SFR register.
  * @param  data: The value of the SFR register.
  * @retval None
  */
void ll_fadc_wr_sfr(u8 addr, u32 data);

/**
  * @brief  FADC module read SFR register function
  * @param  addr: The address of the SFR register.
  * @retval The value of the SFR register.
  */
u32 ll_fadc_rd_sfr(u8 addr);

/**
  * @brief  The FADC check sample interrupt_enable.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_CHECK_SAMPLE_INTERRUPT_ENABLE(p_fadc, chn_bits)   ((p_fadc)->FADCINT0 & LL_FADC_SMP_DONE_INT_EN(chn_bits))

/**
  * @brief  The FADC check dma buffer half interrupt_enable.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_CHECK_DMA_HALF_INTERRUPT_ENABLE(p_fadc, chn_bits) ((p_fadc)->FADCINT1 & LL_FADC_DMA_HALF_INT_EN(chn_bits))

/**
  * @brief  The FADC check dma buffer full interrupt_enable.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_CHECK_DMA_FULL_INTERRUPT_ENABLE(p_fadc, chn_bits) ((p_fadc)->FADCINT1 & LL_FADC_DMA_FULL_INT_EN(chn_bits))

/**
  * @brief  The FADC gets a flag for the completion of the sample.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_GET_SAMPLE_PENDING(p_fadc, chn_bits)              ((p_fadc)->FADCSMPFLAG & chn_bits)

/**
  * @brief  The FADC gets the flag that the sample buf is half done.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_GET_DMA_HALF_PENDING(p_fadc, chn_bits)            ((p_fadc)->FADCDMAFLAG0 & chn_bits)

/**
  * @brief  FADC gets the flag that the sample buffer is full.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_GET_DMA_FULL_PENDING(p_fadc, chn_bits)            ((p_fadc)->FADCDMAFLAG1 & chn_bits)

/**
  * @brief  FADC gets the sampling overflow flag.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
#define LL_FADC_GET_SOC_OVERFLOW(p_fadc, chn_bits)                (ll_fadc_rd_sfr(FADCSFRSOCOVFL) & chn_bits)

/**
  * @brief  FADC clears the flag for one sample completion.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_clear_sample_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCSMPFLAG |= LL_FADC_CLR_SMP_DONE_FLAG(chn_bits);
}

/**
  * @brief  FADC clears the flag that the sample buf is half done.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_clear_dma_half_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCDMAFLAG0 |= LL_FADC_CLR_DMA_HALF_FLAG(chn_bits);
}

/**
  * @brief  FADC clears the flag that the sample buffer is full.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_clear_dma_full_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCDMAFLAG1 |= LL_FADC_CLR_DMA_FULL_FLAG(chn_bits);
}

/**
  * @brief  Fadc enables a single interrupt to be completed.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_sample_interrupt_enable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT0 |= LL_FADC_SMP_DONE_INT_EN(chn_bits);
}

/**
  * @brief  Fadc disables a single interrupt to be completed.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_sample_interrupt_disable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT0 &= ~(LL_FADC_SMP_DONE_INT_EN(chn_bits));
}

/**
  * @brief  Fadc enables half of the buffer to be interrupted.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_dma_half_interrupt_enable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT1 |= LL_FADC_DMA_HALF_INT_EN(chn_bits);
}

/**
  * @brief  Fadc disables half of the buffer to be interrupted.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_dma_half_interrupt_disable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT1 &= ~(LL_FADC_DMA_HALF_INT_EN(chn_bits));
}

/**
  * @brief  The fadc enables the buffer to be full.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_dma_full_interrupt_enable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT1 |= LL_FADC_DMA_FULL_INT_EN(chn_bits);
}

/**
  * @brief  The fadc disables the buffer to be full.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_dma_full_interrupt_disable(FADC_TypeDef *p_fadc, u16 chn_bits) {
    p_fadc->FADCINT1 &= ~(LL_FADC_DMA_FULL_INT_EN(chn_bits));
}

/**
  * @brief  FADC clears the sample overflow flag.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_clear_soc_overflow(FADC_TypeDef *p_fadc, u16 chn_bits) {
    ll_fadc_wr_sfr(FADCSFRSOCOVFL, (ll_fadc_rd_sfr(FADCSFRSOCOVFL) | 
                   LL_FADC_CLR_SOC_OVF(chn_bits)));
}

/**
  * @brief  FADC waits for one sample to complete.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_wait_sample_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    while(chn_bits != (p_fadc->FADCSMPFLAG & chn_bits));
    p_fadc->FADCSMPFLAG |= LL_FADC_CLR_SMP_DONE_FLAG(chn_bits);
}

/**
  * @brief  The FADC waits for half of the sample buffer.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_wait_dma_half_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    while(chn_bits != (p_fadc->FADCDMAFLAG0 & chn_bits));
    p_fadc->FADCDMAFLAG0 |= LL_FADC_CLR_DMA_HALF_FLAG(chn_bits);
}

/**
  * @brief  FADC waits for the sample buffer to be full.
  * @param  p_fadc  : Select the initialized FADC group pointer
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT () 
  *                   function to assign values.
  * @retval None
  */
__STATIC_INLINE void ll_fadc_wait_dma_full_pending(FADC_TypeDef *p_fadc, u16 chn_bits) {
    while(chn_bits != (p_fadc->FADCDMAFLAG1 & chn_bits));
    p_fadc->FADCDMAFLAG1 |= LL_FADC_CLR_DMA_FULL_FLAG(chn_bits);
}

/**
  * @brief  The FADC takes the data of one sample.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : The number of channels to get the fadc data.
  * @retval None
  */
__STATIC_INLINE s16 ll_fadc_get_result(FADC_TypeDef *p_fadc, u8 chn) {
    TX_ASSERT(chn <= 15);
    
    return p_fadc->FADCRES[chn];
}

/**
  * @brief  The FADC gets the acquired data value and the float type returns.
  * @param  p_fadc: Select the initialized FADC group pointer
  * @param  chn   : The number of channels to get the fadc data.
  * @retval None
  */
//__STATIC_INLINE float ll_fadc_get_result_float(FADC_TypeDef *p_fadc, u8 chn) {
//	float fadc_result_f; 
//	TX_ASSERT(chn <= 15);
//	
//	fadc_result_f = (float)(p_fadc->FADCRES[chn])*LL_FADC_VREF_VAL/32768.0 + LL_FADC_VCM_VAL;
//	
//	return fadc_result_f;
//}
__STATIC_INLINE float ll_fadc_get_result_float(FADC_TypeDef *p_fadc, u8 chn) {
    float fadc_result_f; 
    TX_ASSERT(chn <= 15);
    
    fadc_result_f = (float)(p_fadc->FADCRES[chn])*3.3/32768.0 + 3;
    
    return fadc_result_f;
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

#endif //__TX_PHE_LL_FADC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
