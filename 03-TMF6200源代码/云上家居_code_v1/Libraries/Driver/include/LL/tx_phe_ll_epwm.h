/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_epwm.h
  * @author  HUGE-IC Application Team
  * @version V1.0.2
  * @date    09-05-2019
  * @brief   This file contains all the EPWM LL firmware functions.
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
#ifndef __TX_PHE_LL_EPWM_H
#define __TX_PHE_LL_EPWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "tx_phe_ll_cc_rst.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup epwm_interface_gr EPWM Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup EPWM_LL_Driver EPWM LL Driver
  * @ingroup  epwm_interface_gr
  * @brief Mainly the driver part of the EPWM module, which includes \b EPWM \b Register 
  * \b Constants, \b EPWM \b Exported \b Constants, \b EPWM \b Exported \b Struct, \b EPWM
  * \b Data \b transfers \b functions, \b EPWM \b Initialization \b and \b EPWM \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup EPWM_LL_Register_Constants EPWM LL Register Constants
  * @ingroup  EPWM_LL_Driver
  * @brief    EPWM LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the EPWM 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the EPWM register, mainly for convenience. Understand the 
    configuration of the EPWM.
    
@endverbatim
  *
  * @{
  */
  
/***** EPWM_TTCTL *****/
/*! Trip-Zone module interrupt pending
 */
#define LL_EPWM_TZINT_PNG(n)                     (((n)&0x7F) << 25)
/*! Event-Trigger module interrupt pending
 */
#define LL_EPWM_ETINT_PNG(n)                     (((n)&0x7F) << 18)
/*! Trip-Zone cpu software set value
 */
#define LL_EPWM_CPUTZ_ZERO                       (1UL << 10)
#define LL_EPWM_CPUTZ_ONE                        (0UL << 10)
/*! EPWM enable
 */
#define LL_EPWM_EN(n)                            (((n)&0x7F) << 0)


/***** EPWM_ADCSEL0 *****/
/*! Adc0 select epwm channel
 */
#define LL_EPWM_ADC_SEL0(n, i)                   (((n)&0x7) << (3*(i)))
/*! Adc1 select epwm channel
 */
#define LL_EPWM_ADC_SEL1(n, i)                   (((n)&0x7) << (3*(i)))


/***** EPWM_TBCLT *****/
/*! read TBCTR value
 */
#define LL_EPWM_TBCTR(n)                         (((n)&0xFFFF) << 16)
/*! read TBCTR value enable
 */
#define LL_EPWM_READ_TBCTR_EN                    (1UL << 10)
#define LL_EPWM_READ_TBCTR_DIS                   (0UL << 10)
/*! TBCTR dir
 */
#define LL_EPWM_TBCTR_DIR                        (1UL << 9)
/*! Input Synchronization Latched Status Bit
 */
#define LL_EPWM_SYNC_LATCHED                     (1UL << 8)
/*! Output Synchronization Latched Status Bit
 */
#define LL_EPWM_SYNCO_SEL(n)                     (((n)&0x7) << 5)
/*! Software Synchronization enable
 */
#define LL_EPWM_SWF_SYNC                         (1UL << 4)
/*! Phase Synchronization enable
 */
#define LL_EPWM_PHSEN                            (1UL << 3)
/*! TBCTR mode select
 */
#define LL_EPWM_CTRMODE(n)                       (((n)&0x3) << 1)
/*! Active Period Register Load From Shadow Register Select
 */
#define LL_EPWM_PRDLD_DIS                        (1UL << 0)


/***** EPWM_TBPRD_SD *****/
/*! EPWMB fraction value
 */
#define LL_EPWM_FRACB_VALUE(n)                   (((n)&0x7) << 21)
/*! EPWMB edge select
 */
#define LL_EPWM_FRACB_EDGE_NEG                   (1UL << 20)
#define LL_EPWM_FRACB_EDGE_POS                   (0UL << 20)
/*! EPWMA fraction value
 */
#define LL_EPWM_FRACA_VALUE(n)                   (((n)&0x7) << 17)
/*! EPWMA edge select.
 */
#define LL_EPWM_FRACA_EDGE_NEG                   (1UL << 16)
#define LL_EPWM_FRACA_EDGE_POS                   (0UL << 16)
/*! These bits determine the period of the time-base counter
 */
#define LL_EPWM_TBPRD(n)                         (((n)&0xFFFF) << 0)


/***** EPWM_TBPHASE *****/
/*! These bits set time-base counter direction in Phase Synchronization
 */
#define LL_EPWM_TBDIR_UP                         (1UL << 16)
#define LL_EPWM_TBDIR_DOWN                       (0UL << 16)
/*! These bits set time-base counter phase
 */
#define LL_EPWM_TBPHS(n)                         (((n)&0xFFFF) << 0)


/***** EPWM_CMPCTL *****/
/*! Counter-compare C (CMPC) Shadow Register Full Status Flag
 */
#define LL_EPWM_SHDW_C_FULL_PNG                  (1UL << 11)
/*! Counter-compare C (CMPC) Register Operating Mode0 Shadow mode
 */
#define LL_EPWM_SHDW_C_MODE                      (1UL << 10)
/*! Active Counter-Compare C (CMPC) Load From Shadow Select Mode
 */
#define LL_EPWM_LOAD_C_DATA_MODE(n)              (((n)&0x3) << 8)
/*! Counter-compare B (CMPB) Shadow Register Full Status Flag
 */
#define LL_EPWM_SHDW_B_FULL_PNG                  (1UL << 7)
/*! Counter-compare A (CMPA) Shadow Register Full Status Flag
 */
#define LL_EPWM_SHDW_A_FULL_PNG                  (1UL << 6)
/*! Counter-compare B (CMPB) Register Operating Mode0 Shadow mode
 */
#define LL_EPWM_SHDW_B_MODE                      (1UL << 5)
/*! Counter-compare A (CMPA) Register Operating Mode0 Shadow mode
 */
#define LL_EPWM_SHDW_A_MODE                      (1UL << 4)
/*! Active Counter-Compare B (CMPB) Load From Shadow Select Mode
 */
#define LL_EPWM_LOAD_B_DATA_MODE(n)              (((n)&0x3) << 2)
/*! Active Counter-Compare A (CMPA) Load From Shadow Select Mode
 */
#define LL_EPWM_LOAD_A_DATA_MODE(n)              (((n)&0x3) << 0)


/***** EPWM_CMPA_SD *****/
/*! CMPA used to access the counter-compare A value
 */
#define LL_EPWM_CPMA_VALUE(n)                    (((n)&0xFFFF) << 0)


/***** EPWM_CMPB_SD *****/
/*! CMPB used to access the counter-compare B value
 */
#define LL_EPWM_CPMB_VALUE(n)                    (((n)&0xFFFF) << 0)


/***** EPWM_CMPC_SD *****/
/*! CMPC used to access the counter-compare C value
 */
#define LL_EPWM_CPMC_VALUE(n)                    (((n)&0xFFFF) << 0)


/***** EPWM_AQCTLAB *****/
/*! Action when the counter equals the active CMPB register and the counter is decrementing
 */
#define LL_EPWM_CPMB_DEC_DONE_B(n)               (((n)&0x3) << 26)
/*! Action when the counter equals the active CMPB register and the counter is incrementing
 */
#define LL_EPWM_CPMB_INC_DONE_B(n)               (((n)&0x3) << 24)
/*! Action when the counter equals the active CMPA register and the counter is decrementing
 */
#define LL_EPWM_CPMA_DEC_DONE_B(n)               (((n)&0x3) << 22)
/*! Action when the counter equals the active CMPA register and the counter is incrementing
 */
#define LL_EPWM_CPMA_INC_DONE_B(n)               (((n)&0x3) << 20)
/*! Action when the counter equals the period
 */
#define LL_EPWM_PRD_DONE_B(n)                    (((n)&0x3) << 18)
/*! Action when counter equals zero
 */
#define LL_EPWM_ZERO_DONE_B(n)                   (((n)&0x3) << 16)
/*! Action when the counter equals the active CMPB register and the counter is decrementing
 */
#define LL_EPWM_CPMB_DEC_DONE_A(n)               (((n)&0x3) << 10)
/*! Action when the counter equals the active CMPB register and the counter is incrementing
 */
#define LL_EPWM_CPMB_INC_DONE_A(n)               (((n)&0x3) << 8)
/*! Action when the counter equals the active CMPA register and the counter is decrementing
 */
#define LL_EPWM_CPMA_DEC_DONE_A(n)               (((n)&0x3) << 6)
/*! Action when the counter equals the active CMPA register and the counter is incrementing
 */
#define LL_EPWM_CPMA_INC_DONE_A(n)               (((n)&0x3) << 4)
/*! Action when the counter equals the period
 */
#define LL_EPWM_PRD_DONE_A(n)                    (((n)&0x3) << 2)
/*! Action when counter equals zero
 */
#define LL_EPWM_ZERO_DONE_A(n)                   (((n)&0x3) << 0)


/***** EPWM_AQSFRC *****/
/*! AQCSFRC Active Register Reload From Shadow
 */
#define LL_EPWM_RLDCSF(n)                        (((n)&0x3) << 6)
/*! One-Time Software Forced Event on Output B
 */
#define LL_EPWM_OTSFB_EN                         (1UL << 5)
/*! Action when One-Time Software Force B Is invoked
 */
#define LL_EPWM_ACTSFB(n)                        (((n)&0x3) << 3)
/*! One-Time Software Forced Event on Output A
 */
#define LL_EPWM_OTSFA_EN                         (1UL << 2)
/*! Action When One-Time Software Force A Is Invoked
 */
#define LL_EPWM_ACTSFA(n)                        (((n)&0x3) << 0)


/***** EPWM_AQCSFRC *****/
/*! Continuous Software Force on Output B
 */
#define LL_EPWM_CSFB(n)                          (((n)&0x3) << 2)
/*! Continuous Software Force on Output A
 */
#define LL_EPWM_CSFA(n)                          (((n)&0x3) << 0)


/***** EPWM_DBCTL *****/
/*! Half Cycle Clocking Enable Bit
 */
#define LL_EPWM_HALF_CYCLE                       (1UL << 6)
/*! Dead Band Input Mode Control
 */
#define LL_EPWM_IN_MODE(n)                       (((n)&0x3) << 4)
/*! Polarity Select Control
 */
#define LL_EPWM_POLSEL(n)                        (((n)&0x3) << 2)
/*! Dead-band Output Mode Control
 */
#define LL_EPWM_OUT_MODE(n)                      (((n)&0x3) << 0)


/***** EPWM_DBDELAY *****/
/*! Falling Edge Delay Count. 10-bit counter
 */
#define LL_EPWM_FED(n)                           (((n)&0x3FF) << 16)
/*! Rising Edge Delay Count. 10-bit counter
 */
#define LL_EPWM_RED(n)                           (((n)&0x3FF) << 0)


/***** EPWM_ETCTL *****/
/*! Enable the ADC Start of Conversion B (EPWMxSOCB) Pulse
 */
#define LL_EPWM_SOCBEN                           (1UL << 30)
/*! EPWMxSOCB Selection Options
 */
#define LL_EPWM_SOCBSEL(n)                       (((n)&0xF) << 26)
/*! Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse
 */
#define LL_EPWM_SOCAEN                           (1UL << 25)
/*! EPWMxSOCA Selection Options
 */
#define LL_EPWM_SOCASEL(n)                       (((n)&0xF) << 21)
/*! Enable ePWM Interrupt (EPWMx_INT) Generation
 */
#define LL_EPWM_INTEN                            (1UL << 20)
/*! EPWM Interrupt (EPWMx_INT) Selection Options
 */
#define LL_EPWM_INTSEL(n)                        (((n)&0xF) << 16)
/*! EPWM ADC Start-of-Conversion B Event (EPWMxSOCB) Counter Register
 */
#define LL_EPWM_SOCBCNT(n)                       (((n)&0x3) << 14)
/*! EPWM ADC Start-of-Conversion B Event (EPWMxSOCB) Period Select
 */
#define LL_EPWM_SOCBPRD(n)                       (((n)&0x3) << 12)
/*! EPWM ADC Start-of-Conversion A Event (EPWMxSOCA) Counter Register
 */
#define LL_EPWM_SOCACNT(n)                       (((n)&0x3) << 10)
/*! EPWM ADC Start-of-Conversion A Event (EPWMxSOCA) Period Select
 */
#define LL_EPWM_SOCAPRD(n)                       (((n)&0x3) << 8)
/*! EPWM Interrupt Event (EPWMx_INT) Counter Register
 */
#define LL_EPWM_INTCNT(n)                        (((n)&0x3) << 2)
/*! EPWM Interrupt (EPWMx_INT) Period Select
 */
#define LL_EPWM_INTPRD(n)                        (((n)&0x3) << 0)


/***** EPWM_ETFLAG *****/
/*! SOCB Force Bit
 */
#define LL_EPWM_FRCSOCB_FLAG                     (1UL << 11)
/*! SOCA Force Bit
 */
#define LL_EPWM_FRCSOCA_FLAG                     (1UL << 10)
/*! INT Force Bit
 */
#define LL_EPWM_FRCINT_FLAG                      (1UL << 8)
/*! EPWM ADC Start-of-Conversion B (EPWMxSOCB) Flag Clear Bit
 */
#define LL_EPWM_CLRSOCB_FLAG                     (1UL << 7)
/*! EPWM ADC Start-of-Conversion A (EPWMxSOCA) Flag Clear Bit
 */
#define LL_EPWM_CLRSOCA_FLAG                     (1UL << 6)
/*! EPWM Interrupt (EPWMx_INT) Flag Clear Bit
 */
#define LL_EPWM_CLRINT_FLAG                      (1UL << 4)
/*! Latched ePWM ADC Start-of-Conversion B (EPWMxSOCB) Status Flag
 */
#define LL_EPWM_SOCB_FLAG                        (1UL << 3)
/*! Latched ePWM ADC Start-of-Conversion A (EPWMxSOCA) Status Flag
 */
#define LL_EPWM_SOCA_FLAG                        (1UL << 2)
/*! Latched ePWM Interrupt (EPWMx_INT) Status Flag
 */
#define LL_EPWM_INT_FLAG                         (1UL << 0)


/***** EPWM_DCCTL *****/
/*! DCAEVT2 Force Synchronization Signal Select
 */
#define LL_EPWM_AEVT2FRCSYNCSEL_ASYN             (1UL << 19)
#define LL_EPWM_AEVT2FRCSYNCSEL_SYNC             (0UL << 19)
/*! DCAEVT2 Source Signal Select
 */
#define LL_EPWM_AEVT2SRCSEL_FILT                 (1UL << 18)
#define LL_EPWM_AEVT2SRCSEL_EVT                  (0UL << 18)
/*! DCAEVT1 SYNC, Enable/Disable
 */
#define LL_EPWM_AEVT1SYNCE_EN                    (1UL << 17)
/*! DCAEVT1 SOC, Enable/Disable
 */
#define LL_EPWM_AEVT1SOCE_EN                     (1UL << 16)
/*! DCAEVT1 Force Synchronization Signal Select
 */
#define LL_EPWM_AEVT1FRCSYNCSEL_ASYN             (1UL << 15)
#define LL_EPWM_AEVT1FRCSYNCSEL_SYNC             (0UL << 15)
/*! DCAEVT1 Source Signal Select
 */
#define LL_EPWM_AEVT1SRCSEL_FILT                 (1UL << 14)
#define LL_EPWM_AEVT1SRCSEL_EVT                  (0UL << 14)
/*! DCBEVT2 Force Synchronization Signal Select
 */
#define LL_EPWM_BEVT2FRCSYNCSEL_ASYN             (1UL << 13)
#define LL_EPWM_BEVT2FRCSYNCSEL_SYNC             (0UL << 13)
/*! DCBEVT2 Source Signal Select
 */
#define LL_EPWM_BEVT2SRCSEL_FILT                 (1UL << 12)
#define LL_EPWM_BEVT2SRCSEL_EVT                  (0UL << 12)
/*! DCBEVT1 SYNC, Enable/Disable
 */
#define LL_EPWM_BEVT1SYNCE_EN                    (1UL << 11)
/*! DCBEVT1 SOC, Enable/Disable
 */
#define LL_EPWM_BEVT1SOCE_EN                     (1UL << 10)
/*! DCBEVT1 Force Synchronization Signal Select
 */
#define LL_EPWM_BEVT1FRCSYNCSEL_ASYN             (1UL << 9)
#define LL_EPWM_BEVT1FRCSYNCSEL_SYNC             (0UL << 9)
/*! DCBEVT1 Source Signal Select
 */
#define LL_EPWM_BEVT1SRCSEL_FILT                 (1UL << 8)
#define LL_EPWM_BEVT1SRCSEL_EVT                  (0UL << 8)
/*! Pulse Select For Blanking & Capture Alignment
 */
#define LL_EPWM_PULSESEL(n)                      (((n)&0x3) << 6)
/*! Blanking Window Inversion
 */
#define LL_EPWM_BLANKINV                         (1UL << 5)
/*! Blanking Window Enable/Disable
 */
#define LL_EPWM_BLANKE_EN                        (1UL << 4)
/*! Filter Block Signal Source Select
 */
#define LL_EPWM_FILTSRCSEL(n)                    (((n)&0x3) << 2)
/*! TBCTR Counter Capture Shadow Select Mode
 */
#define LL_EPWM_SHDWMODE                         (1UL << 1)
/*! TBCTR Counter Capture Enable/Disable
 */
#define LL_EPWM_CAPE_EN                          (1UL << 0)


/***** EPWM_DCTRIPSEL *****/
/*! Digital Compare Output B Event 2 Selection
 */
#define LL_EPWM_DCBEVT2(n)                       (((n)&0x7) << 25)
/*! Digital Compare Output B Event 1 Selection
 */
#define LL_EPWM_DCBEVT1(n)                       (((n)&0x7) << 22)
/*! Digital Compare Output A Event 2 Selection
 */
#define LL_EPWM_DCAEVT2(n)                       (((n)&0x7) << 19)
/*! Digital Compare Output A Event 1 Selection
 */
#define LL_EPWM_DCAEVT1(n)                       (((n)&0x7) << 16)
/*! Digital Compare B Low Input Select
 */
#define LL_EPWM_DCBLCOMPSEL(n)                   (((n)&0xF) << 12)
/*! Digital Compare B High Input Select
 */
#define LL_EPWM_DCBHCOMPSEL(n)                   (((n)&0xF) << 8)
/*! Digital Compare A Low Input Select Defines the source for the DCAL input
 */
#define LL_EPWM_DCALCOMPSEL(n)                   (((n)&0xF) << 4)
/*! Digital Compare A High Input Select
 */
#define LL_EPWM_DCAHCOMPSEL(n)                   (((n)&0xF) << 0)


/***** EPWM_DCCAP *****/
/*! Digital Compare Time-Base Counter Capture
 */
#define LL_EPWM_DCCAP(n)                         (((n)&0xFFFF) << 0)


/***** EPWM_BLANKOFFSET *****/
/*! Blanking Window Offset
 */
#define LL_EPWM_OFFSET(n)                        (((n)&0xFFFF) << 0)


/***** EPWM_WINWIDTH *****/
/*! Blanking Window Width
 */
#define LL_EPWM_WINDOW(n)                        (((n)&0xFFF) << 0)


/***** EPWM_TZCTL *****/
/*! Digital Compare Output B Event 1 Select
 */
#define LL_EPWM_TZ_DCBEVT1_EN                    (1UL << 31)
/*! Digital Compare Output A Event 1 Select
 */
#define LL_EPWM_TZ_DCAEVT1_EN                    (1UL << 30)
/*! Trip-zone 6 (TZ6) Select
 */
#define LL_EPWM_TZ_OSHT6_EN                      (1UL << 29)
/*! Trip-zone 5 (TZ5) Select
 */
#define LL_EPWM_TZ_OSHT5_EN                      (1UL << 28)
/*! Trip-zone 4 (TZ4) Select
 */
#define LL_EPWM_TZ_OSHT4_EN                      (1UL << 27)
/*! Trip-zone 3 (TZ3) Select
 */
#define LL_EPWM_TZ_OSHT3_EN                      (1UL << 26)
/*! Trip-zone 2 (TZ2) Select
 */
#define LL_EPWM_TZ_OSHT2_EN                      (1UL << 25)
/*! Trip-zone 1 (TZ1) Select
 */
#define LL_EPWM_TZ_OSHT1_EN                      (1UL << 24)
/*! Digital Compare Output B Event 2 Select
 */
#define LL_EPWM_TZ_DCBEVT2_EN                    (1UL << 23)
/*! Digital Compare Output A Event 2 Select
 */
#define LL_EPWM_TZ_DCAEVT2_EN                    (1UL << 22)
/*! Trip-zone 6 (TZ6) Select
 */
#define LL_EPWM_TZ_CBC6_EN                       (1UL << 21)
/*! Trip-zone 5 (TZ5) Select
 */
#define LL_EPWM_TZ_CBC5_EN                       (1UL << 20)
/*! Trip-zone 4 (TZ4) Select
 */
#define LL_EPWM_TZ_CBC4_EN                       (1UL << 19)
/*! Trip-zone 3 (TZ3) Select
 */
#define LL_EPWM_TZ_CBC3_EN                       (1UL << 18)
/*! Trip-zone 2 (TZ2) Select
 */
#define LL_EPWM_TZ_CBC2_EN                       (1UL << 17)
/*! Trip-zone 1 (TZ1) Select
 */
#define LL_EPWM_TZ_CBC1_EN                       (1UL << 16)
/*! Digital Compare Output B Event 2 Action On EPWMxB
 */
#define LL_EPWM_TZ_DCBEVT2(n)                    (((n)&0x3) << 10)
/*! Digital Compare Output A Event 2 Action On EPWMxA
 */
#define LL_EPWM_TZ_DCAEVT2(n)                    (((n)&0x3) << 8)
/*! Digital Compare Output B Event 1 Action On EPWMxB
 */
#define LL_EPWM_TZ_DCBEVT1(n)                    (((n)&0x3) << 6)
/*! Digital Compare Output A Event 1 Action On EPWMxA
 */
#define LL_EPWM_TZ_DCAEVT1(n)                    (((n)&0x3) << 4)
/*! When a trip event occurs the following action is taken on output EPWMxB
 */
#define LL_EPWM_TZ_TZB(n)                        (((n)&0x3) << 2)
/*! When a trip event occurs the following action is taken on output EPWMxA
 */
#define LL_EPWM_TZ_TZA(n)                        (((n)&0x3) << 0)


/***** EPWM_TZFLAG *****/
/*! Digital Comparator Output B Event 2 Interrupt Enable
 */
#define LL_EPWM_TZ_DCBEVT2INTE                   (1UL << 30)
/*! Digital Comparator Output B Event 1 Interrupt Enable
 */
#define LL_EPWM_TZ_DCBEVT1INTE                   (1UL << 29)
/*! Digital Comparator Output A Event 2 Interrupt Enable
 */
#define LL_EPWM_TZ_DCAEVT2INTE                   (1UL << 28)
/*! Digital Comparator Output A Event 1 Interrupt Enable
 */
#define LL_EPWM_TZ_DCAEVT1INTE                   (1UL << 27)
/*! Trip-zone One-Shot Interrupt Enable
 */
#define LL_EPWM_TZ_OSTINTE                       (1UL << 26)
/*! Trip-zone Cycle-by-Cycle Interrupt Enable
 */
#define LL_EPWM_TZ_CBCINTE                       (1UL << 25)
/*! Digital Comparator Output B Event 2 clear pending
 */
#define LL_EPWM_TZ_CLRDCBEVT2                    (1UL << 22)
/*! Digital Comparator Output B Event 1 clear pending
 */
#define LL_EPWM_TZ_CLRDCBEVT1                    (1UL << 21)
/*! Digital Comparator Output A Event 2 clear pending
 */
#define LL_EPWM_TZ_CLRDCAEVT2                    (1UL << 20)
/*! Digital Comparator Output A Event 1 clear pending
 */
#define LL_EPWM_TZ_CLRDCAEVT1                    (1UL << 19)
/*! Trip-zone One-Shot clear pending
 */
#define LL_EPWM_TZ_CLROST                        (1UL << 18)
/*! Trip-zone Cycle-by-Cycle clear pending
 */
#define LL_EPWM_TZ_CLRCBC                        (1UL << 17)
/*! interrupt clear pending
 */
#define LL_EPWM_TZ_CLRINT                        (1UL << 16)
/*! Force Flag for Digital Compare Output B Event 2
 */
#define LL_EPWM_TZ_FRCDCBEVT2                    (1UL << 14)
/*! Force Flag for Digital Compare Output B Event 1
 */
#define LL_EPWM_TZ_FRCDCBEVT1                    (1UL << 13)
/*! Force Flag for Digital Compare Output A Event 2
 */
#define LL_EPWM_TZ_FRCDCAEVT2                    (1UL << 12)
/*! Force Flag for Digital Compare Output A Event 1
 */
#define LL_EPWM_TZ_FRCDCAEVT1                    (1UL << 11)
/*! Force a One-Shot Trip Event via Software
 */
#define LL_EPWM_TZ_FRCOST                        (1UL << 10)
/*! Force a Cycle-by-Cycle Trip Event via Software
 */
#define LL_EPWM_TZ_FRCCBC                        (1UL << 9)
/*! Latched Status Flag for Digital Compare Output B Event 2
 */
#define LL_EPWM_TZ_DCBEVT2_FLAG                  (1UL << 6)
/*! Latched Status Flag for Digital Compare Output B Event 1
 */
#define LL_EPWM_TZ_DCBEVT1_FLAG                  (1UL << 5)
/*! Latched Status Flag for Digital Compare Output A Event 2
 */
#define LL_EPWM_TZ_DCAEVT2_FLAG                  (1UL << 4)
/*! Latched Status Flag for Digital Compare Output A Event 1
 */
#define LL_EPWM_TZ_DCAEVT1_FLAG                  (1UL << 3)
/*! Latched Status Flag for A One-Shot Trip Event
 */
#define LL_EPWM_TZ_OST                           (1UL << 2)
/*! Latched Status Flag for Cycle-By-Cycle Trip Event
 */
#define LL_EPWM_TZ_CBC                           (1UL << 1)
/*! Latched Trip Interrupt Status Flag
 */
#define LL_EPWM_TZ_INT                           (1UL << 0)


/**
  * @}
  */

/** @defgroup EPWM_LL_Exported_Constants EPWM LL Exported Constants
  * @ingroup  EPWM_LL_Driver
  * @brief    EPWM LL external constant definition
  *
@verbatim   
  ===============================================================================
                                Exported Constants
  ===============================================================================  
  
    Exported Constants mainly restricts the partial configuration of the low 
    layer by using the form of enumeration to facilitate the use and understanding of 
    the module configuration. For the specific enumeration meaning, please refer to 
    the annotation of each module.

@endverbatim
  *
  * @{
  */
  
/***** DRIVER API *****/
/**
  * @brief define for low layer EPWM channel enable mask
  */
#define LL_EPWM_EN_MASK                          (((u32)0x7F) << 0) 

/**
  * @brief define for low layer EPWM Maximum channel threshold
  */
#define LL_EPWM_MAX_CHANNEL                      7

/**
  * @brief define for low layer EPWM trigger adc Maximum channel threshold
  */
#define LL_EPWM_MAX_ADC_CHANNEL                  14

/**
  * @brief Enumeration constant for low layer EPWM channel mapping to adc channel.
  */
typedef enum {
    /*! Select the channel 0 channel of adc.
     */
    LL_EPWM_ADC_CHN0 = 0,
    /*! Select the channel 1 channel of adc.
     */
    LL_EPWM_ADC_CHN1,
    /*! Select the channel 2 channel of adc.
     */
    LL_EPWM_ADC_CHN2,
    /*! Select the channel 3 channel of adc.
     */
    LL_EPWM_ADC_CHN3,
    /*! Select the channel 4 channel of adc.
     */
    LL_EPWM_ADC_CHN4,
    /*! Select the channel 5 channel of adc.
     */
    LL_EPWM_ADC_CHN5,
    /*! Select the channel 6 channel of adc.
     */
    LL_EPWM_ADC_CHN6,
    /*! Select the channel 7 channel of adc.
     */
    LL_EPWM_ADC_CHN7,
    /*! Select the channel 8 channel of adc.
     */
    LL_EPWM_ADC_CHN8,
    /*! Select the channel 9 channel of adc.
     */
    LL_EPWM_ADC_CHN9,
    /*! Select the channel 10 channel of adc.
     */
    LL_EPWM_ADC_CHN10,
    /*! Select the channel 11 channel of adc.
     */
    LL_EPWM_ADC_CHN11,
    /*! Select the channel 12 channel of adc.
     */
    LL_EPWM_ADC_CHN12,
    /*! Select the channel 13 channel of adc.
     */
    LL_EPWM_ADC_CHN13,
} TYPE_LL_EPWM_CHANNEL;


/***** LL API *****/

/**
  * @brief Enumeration constant for low layer EPWM AB select
  */
typedef enum {
    /*! EPWM output selection both A end B
    */
    LL_EPWM_IO_MAP_SEL_AB = 0,
    /*! EPWM output selection A
     */
    LL_EPWM_IO_MAP_SEL_A,
    /*! EPWM output selection B
     */
    LL_EPWM_IO_MAP_SEL_B,
    /*! EPWM output selection NO
     */
    LL_EPWM_IO_MAP_SEL_NO,
} TYPE_ENUM_LL_EPWM_IO_MAP_SEL;

/**
  * @brief Enumeration constant for low layer EPWM TZ6 force value
  */
typedef enum {
    /*! Set EPWM force TZ6 to zero
     */
    LL_EPWM_FORCE_TZ6_0 = 0,
    /*! Set EPWM force TZ6 to one
     */
    LL_EPWM_FORCE_TZ6_1,
} TYPE_ENUM_LL_EPWM_TZ6_FORCE;

/**
  * @brief Enumeration constant for low layer EPWM channel select to trigger adc
  * @note  1¡¢SARADC channel_0~channel_6 can only select the channel trigger source of EPWM_A.
  *        2¡¢SARADC channel_7~channel_13 can only select the channel trigger source of EPWM_B.
  */
typedef enum {
    /*! Disable
     */
    LL_EPWM_DIS = 0,
    /*! EPWM0 channel select to trigger adc
     */
    LL_EWPM0_SOCAB,
    /*! EPWM1 channel select to trigger adc
     */
    LL_EWPM1_SOCAB,
    /*! EPWM2 channel select to trigger adc
     */
    LL_EWPM2_SOCAB,
    /*! EPWM3 channel select to trigger adc
     */
    LL_EWPM3_SOCAB,
    /*! EPWM4 channel select to trigger adc
     */
    LL_EWPM4_SOCAB,
    /*! EPWM5 channel select to trigger adc
     */
    LL_EWPM5_SOCAB,
    /*! EPWM6 channel select to trigger adc
     */
    LL_EWPM6_SOCAB,
} TYPE_ENUM_LL_EPWM_ADC_SEL;

/**
  * @brief Enumeration constant for low layer EPWM TB synco select
  */
typedef enum {
    /*! EPWM TB synco select sysn in
     */
    LL_EPWM_SYSNC_IN = 0,
    /*! EPWM TB synco select count equal zero
     */
    LL_EPWM_TB_CNT_EQUAL_ZERO,
    /*! EPWM TB synco select count equal cmpa
     */
    LL_EPWM_TB_CNT_EQUAL_COMPA,
    /*! EPWM TB synco select count equal cmpb
     */
    LL_EPWM_TB_CNT_EQUAL_COMPB,
    /*! EPWM TB synco select count equal NO
     */
    LL_EPWM_SEL_NO,
} TYPE_ENUM_LL_EPWM_SYSCO_SEL;

/**
  * @brief Enumeration constant for low layer EPWM TB counter mode
  */
typedef enum {
    /*! EPWM TB counter mode: up
     */
    LL_EPWM_MODE_UP = 0,
    /*! EPWM TB counter mode: down
     */
    LL_EPWM_MODE_DOWN,
    /*! EPWM TB counter mode: up and down
     */
    LL_EPWM_MODE_UP_DOWN,
    /*! EPWM TB counter mode: constant
     */
    LL_EPWM_MODE_CONSTANT,
} TYPE_ENUM_LL_EPWM_CNT_MODE;

/**
  * @brief Enumeration constant for low layer EPWM TB force pluse edge
  */
typedef enum {
    /*! EPWM TB force pluse edge: rising
     */
    LL_EPWM_TB_FRAC_EDGE_RISING = 0,
    /*! EPWM TB force pluse edge: falling
     */
    LL_EPWM_TB_FRAC_EDGE_FALLING,
} TYPE_ENUM_LL_EPWM_TB_FRAC_EDGE;

/**
  * @brief Enumeration constant for low layer EPWM TB force pluse cycle value
  */
typedef enum {
    /*! EPWM TB force pluse cycle value: DIS
     */
    LL_EPWM_TB_FRAC_PULSE_DIS = 0,
    /*! EPWM TB force pluse cycle value: ONE
     */
    LL_EPWM_TB_FRAC_PULSE_ONE,
    /*! EPWM TB force pluse cycle value: TWO
     */
    LL_EPWM_TB_FRAC_PULSE_TWO,
    /*! EPWM TB force pluse cycle value: THREE
     */
    LL_EPWM_TB_FRAC_PULSE_THREE,
    /*! EPWM TB force pluse cycle value: FOUR
     */
    LL_EPWM_TB_FRAC_PULSE_FOUR,
    /*! EPWM TB force pluse cycle value: FIVE
     */
    LL_EPWM_TB_FRAC_PULSE_FIVE,
    /*! EPWM TB force pluse cycle value: SIX
     */
    LL_EPWM_TB_FRAC_PULSE_SIX,
    /*! EPWM TB force pluse cycle value: SEVEN
     */
    LL_EPWM_TB_FRAC_PULSE_SEVEN,
} TYPE_ENUM_LL_EPWM_TB_FRAC_PULSE;

/**
  * @brief Enumeration constant for low layer EPWM CC shadow mode
  */
typedef enum {
    /*! EPWM CC shadow mode: shadow
     */
    LL_EPWM_MODE_SHADOW = 0,
    /*! EPWM CC shadow mode: immediate
     */
    LL_EPWM_MODE_IMMEDIATE,
} TYPE_ENUM_LL_EPWM_CC_SHDW_MODE;

/**
  * @brief Enumeration constant for low layer EPWM CC load mode
  */
typedef enum {
    /*! EPWM CC load mode: ZERO
     */
    LL_EPWM_LOAD_ON_CTR_ZERO = 0,
    /*! EPWM CC load mode: PRD
     */
    LL_EPWM_LOAD_ON_CTR_PRD,
    /*! EPWM CC load mode: ALL
     */
    LL_EPWM_LOAD_ON_CTR_ALL,
    /*! EPWM CC load mode: NO
     */
    LL_EPWM_LOAD_NO,
} TYPE_ENUM_LL_EPWM_CC_LOAD_MODE;

/**
  * @brief Enumeration constant for low layer EPWM AQ output mode
  */
typedef enum {
    /*! EPWM AQ output mode: NO
     */
    LL_EPWM_AQ_OUT_NO = 0,
    /*! EPWM AQ output mode: LOW
     */  
    LL_EPWM_AQ_OUT_LOW,
    /*! EPWM AQ output mode: HIGH
     */
    LL_EPWM_AQ_OUT_HIGH,
    /*! EPWM AQ output mode: TOGGLE
     */
    LL_EPWM_AQ_OUT_TOGGLE,
} TYPE_ENUM_LL_EPWM_AQ_OUTMODE;

/**
  * @brief Enumeration constant for low layer EPWM AQ software force output mode
  */
typedef enum {
    /*! EPWM AQ software force output mode: NO
     */
    LL_EPWM_SOFT_OUT_NO_EFFECT = 0,
    /*! EPWM AQ software force output mode: LOW
     */
    LL_EPWM_SOFT_OUT_LOW,
    /*! EPWM AQ software force output mode: HIGH
     */
    LL_EPWM_SOFT_OUT_HIGH,
    /*! EPWM AQ software force output mode: DISABLE 
     */
    LL_EPWM_SOFT_OUT_DISABLE,
} TYPE_ENUM_LL_EPWM_AQ_SW_FORCE;

/**
  * @brief Enumeration constant for low layer EPWM DB input edge mode
  */
typedef enum {                         
    /*! DB input edge mode: A both fall and rise   
     */
    LL_EPWM_DB_IN_A_BOTH_EDGE = 0,
    /*! DB input edge mode: A fall, B rise
     */
    LL_EPWM_DB_IN_A_FALL_B_RISE,
    /*! DB input edge mode: A rise, B fall
     */
    LL_EPWM_DB_IN_A_RISE_B_FALL,
    /*! DB input edge mode: B both fall and rise
     */
    LL_EPWM_DB_IN_B_BOTH_EDGE,
} TYPE_ENUM_LL_EPWM_DB_INMODE;

/**
  * @brief Enumeration constant for low layer EPWM DB output polsel mode
  */
typedef enum {
    /*! EPWM Both A and B inverted no
     */
    LL_EPWM_DB_BOTH_AB_NO = 0,
    /*! EPWM A inverted and B no
     */
    LL_EPWM_DB_A_INVERTED_B_NO,
    /*! EPWM A no and B inverted
     */
    LL_EPWM_DB_B_INVERTED_A_NO,
    /*! EPWM Both A and B inverted
     */
    LL_EPWM_DB_BOTH_AB_INVERTED,
} TYPE_ENUM_LL_EPWM_DB_POLSEL;

/**
  * @brief Enumeration constant for low layer EPWM DB output mode
  */
typedef enum {
    /*! DB output mode: both no
     */
    LL_EPWM_DB_OUT_BOTH_NO = 0,
    /*! DB output mode: falling
     */
    LL_EPWM_DB_OUT_FALL,
    /*! DB output mode: rising
     */
    LL_EPWM_DB_OUT_RISIE,
    /*! DB output mode: both falling and rising
     */
    LL_EPWM_DB_OUT_BOTH_FALL_RISE,
} TYPE_ENUM_LL_EPWM_DB_OUTMODE;

/**
  * @brief Enumeration constant for low layer EPWM ET input source select
  */
typedef enum {
    /*! DC B event1
     */
    LL_EPWM_ET_SOCSEL_DCBEVT1 = 0,
    /*! TB count zero
     */
    LL_EPWM_ET_SOCSEL_TBCTR_ZERO,
    /*! TB count period
     */
    LL_EPWM_ET_SOCSEL_TBCTR_PRD,
    /*! both TB count zero and period
     */
    LL_EPWM_ET_SOCSEL_BOTH_TBCTR,
    /*! count dir is up, and count to CMPA
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPA_INC,
    /*! count dir is down, and count to CMPA
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPA_DEC,
    /*! count dir is up, and count to CMPB
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPB_INC,
    /*! count dir is down, and count to CMPB
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPB_DEC,
    /*! count dir is up, and count to CMPC
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPC_INC,
    /*! count dir is down, and count to CMPC
     */
    LL_EPWM_ET_SOCSEL_TBCTR_CMPC_DEC,
} TYPE_ENUM_LL_EPWM_ET_SOCSEL;

/**
  * @brief Enumeration constant for low layer EPWM ET adc event count select
  */
typedef enum {
    /*! event trigger no
     */
    LL_EPWM_ET_EVENT_NO = 0,
    /*! event trigger every one times
     */
    LL_EPWM_ET_EVENT_ONE,
    /*! event trigger every two times
     */
    LL_EPWM_ET_EVENT_TWO,
    /*! event trigger every three times
     */
    LL_EPWM_ET_EVENT_THREE,
} TYPE_ENUM_LL_EPWM_ET_ADCEVENT;

/**
  * @brief Enumeration constant for low layer EPWM DC pulse select
  */
typedef enum {
    /*! DC pulse select: count period
     */
    LL_EPWM_PULSESEL_CTR_PRD = 0,
    /*! DC pulse select: count zero
     */
    LL_EPWM_PULSESEL_CTR_ZERO,
    /*! DC pulse select: no
     */
    LL_EPWM_PULSESEL_NO,
} TYPE_ENUM_LL_EPWM_DC_PULSESEL;

/**
  * @brief Enumeration constant for low layer EPWM DC filter sel
  */
typedef enum {
    /*! DC filter sel: dc A event1
     */
    LL_EPWM_FILTSRCSEL_DCAEVT1 = 0,
    /*! DC filter sel: dc A event2
     */
    LL_EPWM_FILTSRCSEL_DCAEVT2,
    /*! DC filter sel: dc B event1
     */
    LL_EPWM_FILTSRCSEL_DCBEVT1,
    /*! DC filter sel: dc B event2
     */
    LL_EPWM_FILTSRCSEL_DCBEVT2,
} TYPE_ENUM_LL_EPWM_DC_FILTSRCSEL;

/**
  * @brief Enumeration constant for low layer EPWM DC event output mode
  */
typedef enum {
    /*! DC event output mode: disable
     */
    LL_EPWM_EVT_OUT_DISABLE = 0,
    /*! DC event output mode: DCH low
     */
    LL_EPWM_EVT_OUT_DCH_LOW,
    /*! DC event output mode: DCH high
     */
    LL_EPWM_EVT_OUT_DCH_HIGH,
    /*! DC event output mode: DCL low
     */
    LL_EPWM_EVT_OUT_DCL_LOW,
    /*! DC event output mode: DCL high
     */
    LL_EPWM_EVT_OUT_DCL_HIGH,
    /*! DC event output mode: DCL high and DCH low 
     */
    LL_EPWM_EVT_OUT_DCL_HIGH_DCH_LOW,
    /*! DC event output mode: DCL low and DCH high 
     */
    LL_EPWM_EVT_OUT_DCL_LOW_DCH_HIGH,
} TYPE_ENUM_LL_EPWM_DC_EVT_OUT;

/**
  * @brief Enumeration constant for low layer EPWM DC input select
  */
typedef enum {
    /*! DC input select: TZ1
     */
    LL_EPWM_COMPSEL_TZ1_INPUT = 0,
    /*! DC input select: TZ2
     */
    LL_EPWM_COMPSEL_TZ2_INPUT,
    /*! DC input select: TZ3
     */
    LL_EPWM_COMPSEL_TZ3_INPUT,
    /*! DC input select: TZ4
     */
    LL_EPWM_COMPSEL_TZ4_INPUT,
    /*! DC input select: compare 1 output
     */
    LL_EPWM_COMPSEL_COMP1OUT_INPUT = 8,
    /*! DC input select: compare 2 output
     */
    LL_EPWM_COMPSEL_COMP2OUT_INPUT,
    /*! DC input select: compare 3 output
     */
    LL_EPWM_COMPSEL_COMP3OUT_INPUT,
    /*! DC input select: compare 4 output
     */
    LL_EPWM_COMPSEL_COMP4OUT_INPUT,
    /*! DC input select: compare 5 output
     */
    LL_EPWM_COMPSEL_COMP5OUT_INPUT,
} TYPE_ENUM_LL_EPWM_DC_COMPSEL;

/**
  * @brief Enumeration constant for low layer EPWM DC sync select
  */
typedef enum {
    /*! DC sync select: Synchronization signal
     */
    LL_EPWM_DC_SYNC_SIG = 0,
    /*! DC sync select: Asynchronous signal
     */
    LL_EPWM_DC_ASYNC_SIG,
} TYPE_ENUM_LL_EPWM_DC_SYNC_SEL;

/**
  * @brief Enumeration constant for low layer EPWM DC source event select
  */
typedef enum {
    /*! DC source event select: event signal
     */
    LL_EPWM_DC_EVT_SRC_EVT = 0,
    /*! DC source event select: event filter signal 
     */
    LL_EPWM_DC_EVT_SRC_EVTFILT,
} TYPE_ENUM_LL_EPWM_DC_EVT_SRC;

/**
  * @brief Enumeration constant for low layer EPWM TZ action select
  */
typedef enum {
    /*! TZ action select: high impedance
     */
    LL_EPWM_TZ_ACTION_HIGH_IMPEDANCE = 0, 
    /*! TZ action select: high
     */
    LL_EPWM_TZ_ACTION_HIGH,
    /*! TZ action select: low
     */
    LL_EPWM_TZ_ACTION_LOW,
    /*! TZ action select: disable
     */
    LL_EPWM_TZ_ACTION_DISABLE,
} TYPE_ENUM_LL_EPWM_TZ_ACTION;
  
/***** LL API AND DRIVER API *****/



/**
  * @}
  */

/** @defgroup EPWM_LL_Exported_Struct EPWM LL Exported Struct
  * @ingroup  EPWM_LL_Driver
  * @brief    EPWM LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the EPWM registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the low 
    layer and call hal_epwm_init. Function, you can configure the EPWM module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief configuration structure for low layer EPWM TB module struct
  */
typedef struct __ll_epwm_tb_cfg {
    /*! EPWM TB TBCTR Read Cnt Enable
     */
    bool                            read_counter_en;
    /*! EPWM TB Input Synchronization Latched Status Bit
     */
    bool                            sync_latched_clr;
    /*! EPWM TB Software Sync Enable
     */
    bool                            soft_sync_en;
    /*! EPWM TB Phase Sync Enable
     */
    bool                            phase_sync_en;
    /*! EPWM TB Active Period Register Load From Shadow Enable
     */
    bool                            shadow_load_dis;
    /*! EPWM TB Set Time-base Counter Direction Of The Selected EPWM
     */
    bool                            phase_sync_dir_up_en;
    /*! EPWM TB Sync Out Select Bit
     */
    TYPE_ENUM_LL_EPWM_SYSCO_SEL     sync_sel;
    /*! EPWM TB Counter Direction Status Bit
     */
    TYPE_ENUM_LL_EPWM_CNT_MODE      count_dir_mode;
    /*! EPWM TB EPWMB Fraction Value
     */
    TYPE_ENUM_LL_EPWM_TB_FRAC_PULSE frac_b_val;
    /*! EPWM TB EPWMB Edge Select
     */
    TYPE_ENUM_LL_EPWM_TB_FRAC_EDGE  frac_b_edge;
    /*! EPWM TB EPWMA Fraction Value
     */
    TYPE_ENUM_LL_EPWM_TB_FRAC_PULSE frac_a_val;
    /*! EPWM TB EPWMA Edge Select
     */
    TYPE_ENUM_LL_EPWM_TB_FRAC_EDGE  frac_a_edge;
    /*! EPWM TB Determine The Period Of The Time-base Counter
     */
    u16                             period_val;
    /*! EPWM TB Set Time-base Counter Phase Of The Selected EPWM
     */
    u16                             phase_val;
} TYPE_LL_EPWM_TB_CFG;

/**
  * @brief configuration structure for low layer EPWM CC module struct
  */
typedef struct __ll_epwm_cc_cfg {
    /*! EPWM CC Counter-compare A (CMPA) Operating Mode
     */
    TYPE_ENUM_LL_EPWM_CC_SHDW_MODE shadow_a_mode;
    /*! EPWM CC Counter-compare B (CMPB) Operating Mode
     */
    TYPE_ENUM_LL_EPWM_CC_SHDW_MODE shadow_b_mode;
    /*! EPWM CC Counter-compare C (CMPC) Operating Mode
    */
    TYPE_ENUM_LL_EPWM_CC_SHDW_MODE shadow_c_mode;
    /*! EPWM CC Active Counter-Compare A (CMPA) Load From Shadow Select Mode
     */
    TYPE_ENUM_LL_EPWM_CC_LOAD_MODE load_a_mode;
    /*! EPWM CC Active Counter-Compare B (CMPB) Load From Shadow Select Mode
     */
    TYPE_ENUM_LL_EPWM_CC_LOAD_MODE load_b_mode;
    /*! EPWM CC Active Counter-Compare C (CMPC) Load From Shadow Select Mode
     */
    TYPE_ENUM_LL_EPWM_CC_LOAD_MODE load_c_mode;
    /*! EPWM CC used to access the counter-compare A value
     *  Value should satisfy: 1 < cmp_a_val < tb.period_val
     */
    u16                            cmp_a_val;
    /*! EPWM CC used to access the counter-compare B value
     *  Value should satisfy: 1 < cmp_b_val < tb.period_val
     */
    u16                            cmp_b_val;
    /*! EPWM CC used to access the counter-compare C value
     *  Value should satisfy: 1 < cmp_c_val < tb.period_val
     */
    u16                            cmp_c_val;
} TYPE_LL_EPWM_CC_CFG;

/**
  * @brief configuration structure for low layer EPWM AQ module struct
  */
typedef struct __ll_epwm_aq_cfg {
    /*! EPWMB AQ Action when the TB cnt equals the active CMPB, the cnt is decrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_b_dec_b_out;
    /*! EPWMB AQ Action when the TB cnt equals the active CMPB, the cnt is incrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_b_inc_b_out;
    /*! EPWMB AQ Action when the TB cnt equals the active CMPA, the cnt is decrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_a_dec_b_out;
    /*! EPWMB AQ Action when the TB cnt equals the active CMPA, the cnt is incrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_a_inc_b_out;
    /*! EPWMB AQ Action when the counter equals the period
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  period_b_out;
    /*! EPWMB AQ Action when counter equals zero
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  zero_b_out;
    /*! EPWMA AQ Action when the TB cnt equals the active CMPB, the cnt is decrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_b_dec_a_out;
    /*! EPWMA AQ Action when the TB cnt equals the active CMPB, the cnt is incrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_b_inc_a_out;
    /*! EPWMA AQ Action when the TB cnt equals the active CMPA, the cnt is decrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_a_dec_a_out;
    /*! EPWMA AQ Action when the TB cnt equals the active CMPA, the cnt is incrementing 
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  cmp_a_inc_a_out;
    /*! EPWMA AQ Action when the counter equals the period
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  period_a_out;
    /*! EPWMA AQ Action when counter equals zero
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  zero_a_out;
    /*! EPWMB AQ One-Time Software Forced Event on Output B
     */
    bool                          soft_one_time_b_en;
    /*! EPWMB AQ Action when One-Time Software Force B Is invoked
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  soft_b_out;
    /*! EPWMA AQ One-Time Software Forced Event on Output A
     */
    bool                          soft_one_time_a_en;
    /*! EPWMA AQ Action When One-Time Software Force A Is Invoked
     */
    TYPE_ENUM_LL_EPWM_AQ_OUTMODE  soft_a_out;
    /*! EPWMB AQ Continuous Software Force on Output B
     */
    TYPE_ENUM_LL_EPWM_AQ_SW_FORCE contine_soft_b;
    /*! EPWMA AQ Continuous Software Force on Output A
     */
    TYPE_ENUM_LL_EPWM_AQ_SW_FORCE contine_soft_a;
} TYPE_LL_EPWM_AQ_CFG;

/**
  * @brief configuration structure for low layer EPWM DB module struct
  */
typedef struct __ll_epwm_db_cfg {
    /*! EPWM DB Half Cycle Clocking Enable Bit
     */
    bool                         half_cycle_en;
    /*! EPWM DB Dead Band Input Mode Control
     */
    TYPE_ENUM_LL_EPWM_DB_INMODE  in_mode;
    /*! EPWM DB Polarity Select Control
     */
    TYPE_ENUM_LL_EPWM_DB_POLSEL  pol_sel;
    /*! EPWM DB Dead-band Output Mode Control
     */
    TYPE_ENUM_LL_EPWM_DB_OUTMODE out_mode;
    /*! EPWM DB Falling Edge Delay Count. 10-bit counter
     */
    u16                          fall_cnt;
    /*! EPWM DB Rising Edge Delay Count. 10-bit counter
     */
    u16                          rise_cnt;
} TYPE_LL_EPWM_DB_CFG;

/**
  * @brief configuration structure for low layer EPWM ET module struct
  */
typedef struct __ll_epwm_et_cfg {
    /*! EWPM ET Enable the ADC Start of Conversion B (EPWMxSOCB) Pulse
     */
    bool                          soc_b_en;
    /*! EWPM ET EPWMxSOCB Selection Options
     */
    TYPE_ENUM_LL_EPWM_ET_SOCSEL   soc_b_sel;
    /*! EWPM ET Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse
     */
    bool                          soc_a_en;
    /*! EWPM ET EPWMxSOCA Selection Options
     */
    TYPE_ENUM_LL_EPWM_ET_SOCSEL   soc_a_sel;
    /*! EWPM ET Enable ePWM Interrupt (EPWMx_INT) Generation
     */
    bool                          intr_en;
    /*! EWPM ET Interrupt (EPWMx_INT) Selection Options
     */
    TYPE_ENUM_LL_EPWM_ET_SOCSEL   intr_sel;
    /*! EWPM ET ADC Start-of-Conversion B Event (EPWMxSOCB) Period
     */
    TYPE_ENUM_LL_EPWM_ET_ADCEVENT soc_b_event;
    /*! EWPM ET ADC Start-of-Conversion A Event (EPWMxSOCA) Period
     */
    TYPE_ENUM_LL_EPWM_ET_ADCEVENT soc_a_event;
    /*! EWPM ET Interrupt (EPWMx_INT) Period Select
     */
    TYPE_ENUM_LL_EPWM_ET_ADCEVENT intr_event;
} TYPE_LL_EPWM_ET_CFG;

/**
  * @brief configuration structure for low layer EPWM DC module struct
  */
typedef struct __ll_epwm_dc_cfg {
    /*! EPWM DC DCAEVT1 SYNC Enable
     */
    bool                            a_evt1_sync_en;
    /*! EPWM DC DCAEVT1 SOC Enable
     */
    bool                            a_evt1_soc_en;
    /*! EPWM DC DCAEVT2 Force Synchronization Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_SYNC_SEL   a_evt2_force_sync_sel;
    /*! EPWM DC DCAEVT2 Source Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_SRC    a_evt2_src_sel;
    /*! EPWM DC DCAEVT1 Force Synchronization Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_SYNC_SEL   a_evt1_force_sync_sel;
    /*! EPWM DC DCAEVT1 Source Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_SRC    a_evt1_src_sel;
    /*! EPWM DC DCBEVT1 SYNC Enable
     */
    bool                            b_evt1_sync_en;
    /*! EPWM DC DCBEVT1 SOC Enable
     */
    bool                            b_evt1_soc_en;
    /*! EPWM DC DCBEVT2 Force Synchronization Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_SYNC_SEL   b_evt2_force_sync_sel;
    /*! EPWM DC DCBEVT2 Source Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_SRC    b_evt2_src_sel;
    /*! EPWM DC DCBEVT1 Force Synchronization Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_SYNC_SEL   b_evt1_force_sync_sel;
    /*! EPWM DC DCBEVT1 Source Signal Select
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_SRC    b_evt1_src_sel;
    /*! EPWM DC Pulse Select For Blanking & Capture Alignment
     */
    TYPE_ENUM_LL_EPWM_DC_PULSESEL   pulse_sel;
    /*! EPWM DC Blanking Window Inversion
     */
    bool                            blank_invert;
    /*! EPWM DC Blanking Window Enable
     */
    bool                            blank_en;
    /*! EPWM DC Filter Block Signal Source Select
     */
    TYPE_ENUM_LL_EPWM_DC_FILTSRCSEL filt_src_sel;
    /*! EPWM DC TBCTR Counter Capture Shadow Select Mode
     */
    bool                            shadow_mode_dis;
    /*! EPWM DC TBCTR Counter Capture Enable
     */
    bool                            capture_en;

    /*! EPWM DC Digital Compare Output B Event 2 Selection
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_OUT    dc_b_evt2_out;
    /*! EPWM DC Digital Compare Output B Event 1 Selection
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_OUT    dc_b_evt1_out;
    /*! EPWM DC Digital Compare Output A Event 2 Selection
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_OUT    dc_a_evt2_out;
    /*! EPWM DC Digital Compare Output A Event 1 Selection
     */
    TYPE_ENUM_LL_EPWM_DC_EVT_OUT    dc_a_evt1_out;
    /*! EPWM DC Digital Compare B Low Input Select
     */
    TYPE_ENUM_LL_EPWM_DC_COMPSEL    dc_bl_cmp_in;
    /*! EPWM DC Digital Compare A High Input Select
     */
    TYPE_ENUM_LL_EPWM_DC_COMPSEL    dc_bh_cmp_in;
    /*! EPWM DC Digital Compare B Low Input Select
     */
    TYPE_ENUM_LL_EPWM_DC_COMPSEL    dc_al_cmp_in;
    /*! EPWM DC Digital Compare A High Input Select
     */
    TYPE_ENUM_LL_EPWM_DC_COMPSEL    dc_ah_cmp_in;
    /*! EPWM DC Digital Compare Time-Base Counter Capture
     */
    u16                             dc_capture_cnt;
    /*! EPWM DC Blanking Window Offset
     */
    u16                             blank_offset;
    /*! EPWM DC Blanking Window Width
     */
    u16                             window_width;
} TYPE_LL_EPWM_DC_CFG;

/**
  * @brief configuration structure for low layer EPWM TZ module struct
  */
typedef struct __ll_epwm_tz_cfg {
    /*! EPWM TZ Digital Compare Output B Event 1 Select
     */
    bool                        dc_b_evt1_en;
    /*! EPWM TZ Digital Compare Output A Event 1 Select
     */
    bool                        dc_a_evt1_en;
    /*! EPWM TZ Trip-zone 6 (TZ6) As one-shot Enable
     */
    bool                        tz6_as_trip_en;
    /*! EPWM TZ Trip-zone 5 (TZ5) As one-shot Enable
     */
    bool                        tz5_as_trip_en;
    /*! EPWM TZ Trip-zone 4 (TZ4) As one-shot Enable
     */
    bool                        tz4_as_trip_en;
    /*! EPWM TZ Trip-zone 3 (TZ3) As one-shot Enable
     */
    bool                        tz3_as_trip_en;
    /*! EPWM TZ Trip-zone 2 (TZ2) As one-shot Enable
     */
    bool                        tz2_as_trip_en;
    /*! EPWM TZ Trip-zone 1 (TZ1) As one-shot Enable
     */
    bool                        tz1_as_trip_en;
    
    /*! EPWM TZ Digital Compare Output B Event 2 Select
     */
    bool                        dc_b_evt2_en;
    /*! EPWM TZ Digital Compare Output A Event 2 Select
     */
    bool                        dc_a_evt2_en;
    /*! EPWM TZ Trip-zone 6 (TZ6) As cycle-by_cycle Enable
     */
    bool                        tz6_as_cbc_en;
    /*! EPWM TZ Trip-zone 5 (TZ5) As cycle-by_cycle Enable
     */
    bool                        tz5_as_cbc_en;       
    /*! EPWM TZ Trip-zone 4 (TZ4) As cycle-by_cycle Enable
     */
    bool                        tz4_as_cbc_en;
    /*! EPWM TZ Trip-zone 3 (TZ3) As cycle-by_cycle Enable
     */
    bool                        tz3_as_cbc_en;
    /*! EPWM TZ Trip-zone 2 (TZ2) As cycle-by_cycle Enable
     */
    bool                        tz2_as_cbc_en;
    /*! EPWM TZ Trip-zone 1 (TZ1) As cycle-by_cycle Enable
     */
    bool                        tz1_as_cbc_en;
    
    /*! EPWM TZ Digital Comparator Output B Event 2 Interrupt Enable
     */
    bool                        dc_b_evt2_intr_en;   
    /*! EPWM TZ Digital Comparator Output B Event 1 Interrupt Enable
     */
    bool                        dc_b_evt1_intr_en;
    /*! EPWM TZ Digital Comparator Output A Event 2 Interrupt Enable
     */
    bool                        dc_a_evt2_intr_en;
    /*! EPWM TZ Digital Comparator Output A Event 1 Interrupt Enable
     */
    bool                        dc_a_evt1_intr_en;
    /*! EPWM TZ Trip-zone One-Shot Interrupt Enable
     */
    bool                        tz_trip_intr_en;
    /*! EPWM TZ Trip-zone Cycle-by-Cycle Interrupt Enable
     */
    bool                        tz_cbc_intr_en;
    
    /*! EPWM TZ Digital Compare Output B Event 2 Action On EPWMxB
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION dc_b_evt2_action;
    /*! EPWM TZ Digital Compare Output B Event 1 Action On EPWMxB
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION dc_b_evt1_action;
    /*! EPWM TZ Digital Compare Output A Event 2 Action On EPWMxA
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION dc_a_evt2_action;
    /*! EPWM TZ Digital Compare Output A Event 1 Action On EPWMxA
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION dc_a_evt1_action;
    /*! EPWM TZ a trip event occurs the following action is taken on out EPWMxB
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION tz_b_action;
    /*! EPWM TZ a trip event occurs the following action is taken on out EPWMxA
     */
    TYPE_ENUM_LL_EPWM_TZ_ACTION tz_a_action;
} TYPE_LL_EPWM_TZ_CFG;

/**
  * @brief configuration structure for low layer EPWM channel struct
  */
typedef struct __ll_epwm_chn_cfg {
    /*! EPWM CHN Time-Base module config struct
     */
    TYPE_LL_EPWM_TB_CFG tb;
    /*! EPWM CHN Counter-Compare module config struct
     */
    TYPE_LL_EPWM_CC_CFG cc;
    /*! EPWM CHN Action-Qualifier module config struct
     */
    TYPE_LL_EPWM_AQ_CFG aq;
    /*! EPWM CHN Dead-Band module config struct
     */
    TYPE_LL_EPWM_DB_CFG db;
    /*! EPWM CHN Event-Trigger module config struct
     */
    TYPE_LL_EPWM_ET_CFG et;
    /*! EPWM CHN Digital Compare module config struct
     */
    TYPE_LL_EPWM_DC_CFG dc;
    /*! EPWM CHN Trip-Zone module config struct
     */
    TYPE_LL_EPWM_TZ_CFG tz;
} TYPE_LL_EPWM_CHN_CFG;

/**
  * @brief configuration structure for low layer EPWM module struct
  */ 
typedef struct __ll_epwm_cfg {       
    /*! Select the channel to be configured by EPWM
     *  @note The range of chn can only be between 0 and 6.
     */
    u8                            chn;
    /*! Select the EPWM output.
     */
    TYPE_ENUM_LL_EPWM_IO_MAP_SEL  io_map_sel;
    /*! EPWM Trip-Zone cpu kick value
     *  @note The TZ6 will refer to the software triggering the TZ behavior, 
     *        which triggers each channel.
     */     
    TYPE_ENUM_LL_EPWM_TZ6_FORCE   cpu_tz; 
    /*! EPWM channel 0 to channel 6 config struct
     */
    TYPE_LL_EPWM_CHN_CFG          module;
} TYPE_LL_EPWM_CFG; 

/**
  * @brief initialization structure for low layer EPWM module struct
  */ 
typedef struct __ll_epwm_init {
    /*! Configure the source select for EPWM 
     */
    TYPE_ENUM_LL_CC_RST_EPWM_CLK_SRC    src;
    /*! Configure the division ratio of the EPWM operating clock, Fadc and epwm share an adpll, 
     *  the default configuration is 180M.
     *  The epwm_adpll_div_val must be less than 15.
     */
    u8                                  epwm_adpll_div_val;
} TYPE_LL_EPWM_INIT; 

/**
  * @}
  */

/** @defgroup EPWM_LL_Interrupt EPWM LL Interrupt Handle function
  * @brief   EPWM LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EPWM  
    Interrupt Handle function.

    how to use?


@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup EPWM_LL_Inti_Cfg EPWM LL Initialization And Configuration
  * @brief    EPWM LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EPWM data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer EPWM initialization function
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  p_init: Low layer initialization variable
  * @retval None
  */
void ll_epwm_init(EPWM_TypeDef *p_epwm, TYPE_LL_EPWM_INIT *p_init);

/**
  * @brief  Low layer EPWM delete initialization function
  * @param  p_epwm: Structure pointer set to EPWM.
  * @retval None
  */
void ll_epwm_deinit(EPWM_TypeDef *p_epwm);

/**
  * @brief  Low layer EPWM Configuration function
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  p_cfg : Low layer initialization variable
  * @retval None
  */
void ll_epwm_config(EPWM_TypeDef *p_epwm, TYPE_LL_EPWM_CFG *p_cfg);

/**
  * @brief  Low layer Path selection between Saradc and epwm.
  * @param  p_epwm  : Select the initialized EPWM group pointer
  * @param  adc_chn : Select the configured adc path, range 0~14.
  * @param  epwm_sel: Select the trigger source of adc.
  * @retval None
  * @note   1.The EPWM trigger source selected by saradc0 ~ saradc6 can only be selected between epwm0_soca ~epwm6_soca.
  *         2.The EPWM trigger source selected by saradc7 ~ saradc13 can only be selected between epwm0_socb ~epwm6_socb.
  */
void ll_epwm_saradc_sel_config(EPWM_TypeDef *p_epwm, u8 adc_chn, TYPE_ENUM_LL_EPWM_ADC_SEL epwm_sel);

/**
  * @brief  Low layer Configure the map that triggers the signal between fadc and epwm.
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @retval None
  */
void ll_epwm_fadc_map_config(EPWM_TypeDef *p_epwm);

/**
  * @}
  */
  
/** @defgroup EPWM_LL_Data_Transfers EPWM LL Data transfers functions
  * @brief    EPWM LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EPWM data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer EPWM direct configuration start function
  * @param  p_epwm  : Structure pointer set to EPWM.
  * @param  chn_bits: Channel value with shift
  * @retval None
  * @note   This function is a direct configuration register, but 
  *         bit_chn must be assigned the form of BIT(chn).
  */
void ll_epwm_start(EPWM_TypeDef *p_epwm, u32 chn_bits);

/**
  * @brief  Low layer EPWM direct configuration stop function
  * @param  p_epwm  : Structure pointer set to EPWM.
  * @param  chn_bits: Channel value with shift
  * @param  a_action: When the EPWM is stopped, the safe level of the EPWM_A output is required.
  * @param  b_action: When the EPWM is stopped, the safe level of the EPWM_B output is required.
  * @retval None
  * @note   This function is a direct configuration register, but 
  *         bit_chn must be assigned the form of BIT(chn).
  */
void ll_epwm_stop(EPWM_TypeDef               *p_epwm, 
                  u32                         chn_bits, 
                  TYPE_ENUM_LL_EPWM_TZ_ACTION a_action,
                  TYPE_ENUM_LL_EPWM_TZ_ACTION b_action);

/**
  * @brief  Low layer EPWM check channel enable
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_CHANNEL_ENABLE(p_epwm, chn)                ((p_epwm)->EPWM_TTCTL & LL_EPWM_EN(BIT(chn)))

/**
  * @brief  Low layer EPWM check ET interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_ET_INTERRUPT_ENABLE(p_epwm, chn)           ((p_epwm)->CH[chn].EPWM_ETCTL & LL_EPWM_INTEN)

/**
  * @brief  Low layer EPWM check TZ dc b event2 interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_DCBEVT2_INTERRUPT_ENABLE(p_epwm, chn)   ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_DCBEVT2INTE)

/**
  * @brief  Low layer EPWM check TZ dc b event1 interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_DCBEVT1_INTERRUPT_ENABLE(p_epwm, chn)   ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_DCBEVT1INTE)

/**
  * @brief  Low layer EPWM check TZ dc a event2 interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_DCAEVT2_INTERRUPT_ENABLE(p_epwm, chn)   ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_DCAEVT2INTE)

/**
  * @brief  Low layer EPWM check TZ dc a event1 interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_DCAEVT1_INTERRUPT_ENABLE(p_epwm, chn)   ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_DCAEVT1INTE)

/**
  * @brief  Low layer EPWM check TZ one-shot interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_OST_INTERRUPT_ENABLE(p_epwm, chn)       ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_OSTINTE)

/**
  * @brief  Low layer EPWM check TZ cycle-by-cycle interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_CHECK_TZ_CBC_INTERRUPT_ENABLE(p_epwm, chn)       ((p_epwm)->CH[chn].EPWM_TZFLAG & LL_EPWM_TZ_CBCINTE)

/**
  * @brief  Low layer EPWM get ET interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_GET_ET_INTERRUPT_PENDING(p_epwm, chn)            ((p_epwm)->CH[chn].EPWM_ETFLAG)

/**
  * @brief  Low layer EPWM get TZ interrupt pending
  * @param  p_epwm: Structure pointer set to EPWM.
  * @param  chn   : Channel num of EPWM, Zreo to six
  * @retval None
  */
#define LL_EPWM_GET_TZ_INTERRUPT_PENDING(p_epwm, chn)            ((p_epwm)->CH[chn].EPWM_TZFLAG)

/**
  * @brief  Low layer EPWM clear ET interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_et_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_ETFLAG |= LL_EPWM_CLRINT_FLAG;
}

/**
  * @brief  Low layer EPWM clear TZ dc b event2 interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_dc_b_event2_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCBEVT2;
}

/**
  * @brief  Low layer EPWM clear TZ dc b event1 interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_dc_b_event1_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCBEVT1;
}

/**
  * @brief  Low layer EPWM clear TZ dc a event2 interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_dc_a_event2_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCAEVT2;
}

/**
  * @brief  Low layer EPWM clear TZ dc a event1 interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_dc_a_event1_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCAEVT1;
}

/**
  * @brief  Low layer EPWM clear TZ one-shot interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_one_shot_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLROST;
}

/**
  * @brief  Low layer EPWM clear TZ cycle-by-cycle interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_cycle_by_cycle_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRCBC;
}

/**
  * @brief  Low layer EPWM clear TZ interrupt pending
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_clear_tz_interrupt_pending(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRINT;
}

/**
  * @brief  Low layer EPWM disable TB software force synchronous
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tb_swf_sync_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TBCTL &= ~LL_EPWM_SWF_SYNC;
}

/**
  * @brief  Low layer EPWM ensable TB software force synchronous
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tb_soft_sync_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TBCTL |= LL_EPWM_SWF_SYNC;
}

/**
  * @brief  Low layer EPWM set period value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  period_val: period value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_period(u8 chn, u16 period_val) {
    EPWM->CH[chn].EPWM_TBPRD = LL_EPWM_TBPRD(period_val);
}

/**
  * @brief  Low layer EPWM set phase value
  * @param  chn      : Channel num of EPWM, Zreo to six
  * @param  phase_val: phase value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_phase(u8 chn, u16 phase_val) {
    EPWM->CH[chn].EPWM_TBPHASE = LL_EPWM_TBPHS(phase_val);
}

/**
  * @brief  Low layer EPWM set compare A value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  cmpa_value: compare A value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_a(u8 chn, u16 cmpa_value) {
    EPWM->CH[chn].EPWM_CMPA = LL_EPWM_CPMA_VALUE(cmpa_value);
}

/**
  * @brief  Low layer EPWM set compare B value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  cmpb_value: compare B value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_b(u8 chn, u16 cmpb_value) {
    EPWM->CH[chn].EPWM_CMPB = LL_EPWM_CPMB_VALUE(cmpb_value);
}

/**
  * @brief  Low layer EPWM set compare A and B value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  cmpa_value: compare A value
  * @param  cmpb_value: compare B value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_a_and_b(u8 chn, u16 cmpa_value, u16 cmpb_value) {
    TX_ASSERT((cmpa_value != 0)                                 && 
              (cmpb_value != 0)                                 && 
              (cmpa_value != (EPWM->CH[chn].EPWM_TBPRD&0xFFFF)) && 
              (cmpb_value != (EPWM->CH[chn].EPWM_TBPRD&0xFFFF)));
    
    if((cmpa_value == cmpb_value) && !(EPWM->CH[chn].EPWM_AQCTLAB & 0x0F000F00)) {
        EPWM->CH[chn].EPWM_CMPB = LL_EPWM_CPMB_VALUE(cmpb_value+1);
    } else {
        EPWM->CH[chn].EPWM_CMPB = LL_EPWM_CPMB_VALUE(cmpb_value);
    }
    EPWM->CH[chn].EPWM_CMPA = LL_EPWM_CPMA_VALUE(cmpa_value);
}

/**
  * @brief  Low layer EPWM set compare C value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  cmpc_value: compare C value
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_c(u8 chn, u16 cmpc_value) {
    EPWM->CH[chn].EPWM_CMPC = LL_EPWM_CPMC_VALUE(cmpc_value);
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare A by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled high.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_a_by_fixed_in_zero_high(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPA = LL_EPWM_CPMA_VALUE(((u32)period*fixed_percent)>>15);
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare B by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled high.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_b_by_fixed_in_zero_high(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPB = LL_EPWM_CPMB_VALUE(((u32)period*fixed_percent)>>15);
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare C by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled high.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_c_by_fixed_in_zero_high(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPC = LL_EPWM_CPMC_VALUE(((u32)period*fixed_percent)>>15);
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare A by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled low.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_a_by_fixed_in_zero_low(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPA = LL_EPWM_CPMA_VALUE(period - (((u32)period*fixed_percent)>>15));
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare B by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled low.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_b_by_fixed_in_zero_low(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPB = LL_EPWM_CPMB_VALUE(period - (((u32)period*fixed_percent)>>15));
}

/**
  * @brief  Low layer EPWM sets the duty ratio of compare C by the fixed point percentage,
  *         At count=0, the behavior of AQ is pulled low.
  * @param  chn          : Channel num of EPWM, Zreo to six
  * @param  fixed_percent: Fixed point percentage, 15bit fixed point value.
  * @param  period       : Period value.
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_compare_c_by_fixed_in_zero_low(u8 chn, u16 fixed_percent, u16 period) {
    EPWM->CH[chn].EPWM_CMPC = LL_EPWM_CPMC_VALUE(period - (((u32)period*fixed_percent)>>15));
}

/**
  * @brief  Low layer EPWM need to wait for the shadow register to be 
  *         in nofull state before updating cmpa value
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_wait_compare_a_shadow_nofull(u8 chn) {
    while(EPWM->CH[chn].EPWM_CMPCTL & LL_EPWM_SHDW_A_FULL_PNG);
}

/**
  * @brief  Low layer EPWM need to wait for the shadow register to be 
  *         in nofull state before updating cmpb value
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_wait_compare_b_shadow_nofull(u8 chn) {
    while(EPWM->CH[chn].EPWM_CMPCTL & LL_EPWM_SHDW_B_FULL_PNG);
}

/**
  * @brief  Low layer EPWM need to wait for the shadow register to be 
  *         in nofull state before updating cmpc value
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_wait_compare_c_shadow_nofull(u8 chn) {
    while(EPWM->CH[chn].EPWM_CMPCTL & LL_EPWM_SHDW_C_FULL_PNG);
}

/**
  * @brief  Low layer EPWM AQ enable One-Time Software Forced Event on Output A
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_a_aq_soft_force_one_enable(u8 chn) {
    EPWM->CH[chn].EPWM_AQSFRC |= LL_EPWM_OTSFA_EN;
}

/**
  * @brief  Low layer EPWM AQ enable One-Time Software Forced Event on Output B
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_b_aq_soft_force_one_enable(u8 chn) {
    EPWM->CH[chn].EPWM_AQSFRC |= LL_EPWM_OTSFB_EN;
}

/**
  * @brief  Low layer EPWM AQ enable Continuous Software Force on Output A
  * @param  chn    : Channel num of EPWM, Zreo to six
  * @param  force_a: Set the level mode of the output
  * @retval None
  */
__STATIC_INLINE void ll_epwm_a_aq_soft_continue_enable(u8 chn, 
                                                       TYPE_ENUM_LL_EPWM_AQ_SW_FORCE force_a) {
	uint32_t temp;
    temp = EPWM->CH[chn].EPWM_AQCSFRC;
    temp &= ~(LL_EPWM_CSFA(3UL));
    temp |= LL_EPWM_CSFA(force_a);
	EPWM->CH[chn].EPWM_AQCSFRC = temp;
}

/**
  * @brief  Low layer EPWM AQ disable Continuous Software Force on Output A
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_a_aq_soft_continue_disable(u8 chn) {
    EPWM->CH[chn].EPWM_AQCSFRC &= ~(LL_EPWM_CSFA(3UL));
}

/**
  * @brief  Low layer EPWM AQ enable Continuous Software Force on Output B
  * @param  chn    : Channel num of EPWM, Zreo to six
  * @param  force_b: Set the level mode of the output
  * @retval None
  */
__STATIC_INLINE void ll_epwm_b_aq_soft_continue_enable(u8 chn, 
                                                       TYPE_ENUM_LL_EPWM_AQ_SW_FORCE force_b) {
	uint32_t temp;
	temp = EPWM->CH[chn].EPWM_AQCSFRC;
    temp &= ~(LL_EPWM_CSFB(3UL));
    temp |= LL_EPWM_CSFB(force_b);
	EPWM->CH[chn].EPWM_AQCSFRC = temp;
}

/**
  * @brief  Low layer EPWM AQ enable Continuous Software Force on Output B
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_b_aq_soft_continue_disable(u8 chn) {
    EPWM->CH[chn].EPWM_AQCSFRC &= ~(LL_EPWM_CSFB(3UL));
}

/**
  * @brief  Low layer EPWM DC set Blanking Window Offset value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  offset_val: Set the value of the Blanking Window Offset
  * @retval None
  */
__STATIC_INLINE void ll_epwm_b_set_dc_blank_offset(u8 chn, u16 offset_val) {
    EPWM->CH[chn].EPWM_BLANKOFFSET = LL_EPWM_OFFSET(offset_val);
}

/**
  * @brief  Low layer EPWM DC set Blanking Window Width value
  * @param  chn       : Channel num of EPWM, Zreo to six
  * @param  window_val: Set the value of the Blanking Window Width
  * @retval None
  */
__STATIC_INLINE void ll_epwm_b_set_dc_window_width(u8 chn, u16 window_val) {
    EPWM->CH[chn].EPWM_WINWIDTH = LL_EPWM_WINDOW(window_val);
}

/**
  * @brief  Low layer EPWM set TZ6 to false
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_tz6_false(void) {
    EPWM->EPWM_TTCTL |= LL_EPWM_CPUTZ_ZERO;
}

/**
  * @brief  Low layer EPWM set TZ6 to true
  * @retval None
  */
__STATIC_INLINE void ll_epwm_set_tz6_true(void) {
    EPWM->EPWM_TTCTL &= ~LL_EPWM_CPUTZ_ZERO;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force a Cycle-by-Cycle Trip Event
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_cbc_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCCBC;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force a One-Shot Trip Event
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_ost_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCOST;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force Flag for Digital Compare Output A Event 1
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dca_evt1_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCDCAEVT1;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force Flag for Digital Compare Output A Event 2
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dca_evt2_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCDCAEVT2;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force Flag for Digital Compare Output B Event 1
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dcb_evt1_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCDCBEVT1;
}

/**
  * @brief  Low layer EPWM TZ enable Software Force Flag for Digital Compare Output B Event 2
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dcb_evt2_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_FRCDCBEVT2;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force a Cycle-by-Cycle Trip Event
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_cbc_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRCBC;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force a One-Shot Trip Event
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_ost_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLROST;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force Flag for Digital Compare Output A Event 1
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dca_evt1_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG = LL_EPWM_TZ_CLRDCAEVT1;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force Flag for Digital Compare Output A Event 2
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dca_evt2_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCAEVT2;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force Flag for Digital Compare Output B Event 1
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dcb_evt1_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCBEVT1;
}

/**
  * @brief  Low layer EPWM TZ disable Software Force Flag for Digital Compare Output B Event 2
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_force_dcb_evt2_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CLRDCBEVT2;
}

/**
  * @brief  Low layer EPWM enable ET interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_et_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_ETCTL |= LL_EPWM_INTEN;
}

/**
  * @brief  Low layer EPWM disable ET interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_et_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_ETCTL &= ~(LL_EPWM_INTEN);
}

/**
  * @brief  Low layer EPWM enable TZ dc b event2 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_b_event2_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_DCBEVT2INTE;
}

/**
  * @brief  Low layer EPWM disable TZ dc b event2 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_b_event2_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_DCBEVT2INTE);
}

/**
  * @brief  Low layer EPWM enable TZ dc b event1 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_b_event1_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_DCBEVT1INTE;
}

/**
  * @brief  Low layer EPWM disable TZ dc b event1 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_b_event1_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_DCBEVT1INTE);
}

/**
  * @brief  Low layer EPWM enable TZ dc a event2 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_a_event2_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_DCAEVT2INTE;
}

/**
  * @brief  Low layer EPWM disable TZ dc a event2 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_a_event2_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_DCAEVT2INTE);
}

/**
  * @brief  Low layer EPWM enable TZ dc a event1 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_a_event1_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_DCAEVT1INTE;
}

/**
  * @brief  Low layer EPWM disable TZ dc a event1 interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_dc_a_event1_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_DCAEVT1INTE);
}

/**
  * @brief  Low layer EPWM enable TZ One-Shot interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_one_shot_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_OSTINTE;
}

/**
  * @brief  Low layer EPWM disable TZ One-Shot interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_one_shot_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_OSTINTE);
}

/**
  * @brief  Low layer EPWM enable TZ Cycle-by-Cycle interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_cycle_by_cycle_interrupt_enable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG |= LL_EPWM_TZ_CBCINTE;
}

/**
  * @brief  Low layer EPWM disable TZ Cycle-by-Cycle interrupt
  * @param  chn: Channel num of EPWM, Zreo to six
  * @retval None
  */
__STATIC_INLINE void ll_epwm_tz_cycle_by_cycle_interrupt_disable(u8 chn) {
    EPWM->CH[chn].EPWM_TZFLAG &= ~(LL_EPWM_TZ_CBCINTE);
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

#endif //__TX_PHE_LL_EPWM_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
