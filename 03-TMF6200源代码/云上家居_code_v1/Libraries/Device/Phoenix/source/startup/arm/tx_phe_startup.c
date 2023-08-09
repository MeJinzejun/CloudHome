/**
  *****************************************************************************
  * @file    Libraries/Device/Phoenix/source/startup/arm/tx_phe_startup.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    08-04-2019
  * @brief   CMSIS Cortex-M3 Core Peripheral Access Layer Source File for 
  *          HUGE-IC Phoenix MCU
  ****************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  *
  ******************************************************************************
  */ 

#include "include.h"

/* Initialize segments */
#if defined(__CC_ARM)                /* KEIL Compiler */
extern uint32_t Image$$ARM_LIB_STACK$$ZI$$Limit;
extern void __main(void);

#elif defined(__GNUC__)              /* GCC Compiler */
extern uint32_t __StackTop;
extern uint32_t __etext;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __bss_extern_start__  WEAK;
extern uint32_t __bss_extern_end__ WEAK;

extern void uvisor_init(void);
extern void software_init_hook(void) __attribute__((weak));
extern void __libc_init_array(void);
extern int main(void);

#elif defined (__ICCARM__)           /* IAR Compiler */
void __iar_program_start(void);

#endif

/* Default empty handler */
void Default_Handler(void);

/* Reset handler */
void Reset_Handler(void);

/* Cortex-M3 core handlers */
WEAK_ALIAS_FUNC(NMI_Handler,                      Default_Handler)
WEAK_ALIAS_FUNC(HardFault_Handler,                Default_Handler)
WEAK_ALIAS_FUNC(MemManage_Handler,                Default_Handler)
WEAK_ALIAS_FUNC(BusFault_Handler,                 Default_Handler)
WEAK_ALIAS_FUNC(UsageFault_Handler,               Default_Handler)
WEAK_ALIAS_FUNC(SVC_Handler,                      Default_Handler)
WEAK_ALIAS_FUNC(DebugMon_Handler,                 Default_Handler)
WEAK_ALIAS_FUNC(PendSV_Handler,                   Default_Handler)
WEAK_ALIAS_FUNC(SysTick_Handler,                  Default_Handler)

/* Peripherals handlers */
WEAK_ALIAS_FUNC(IIC0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(IIC1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(USART0_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(USART1_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(USART2_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(SPI0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(SPI1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(HCC_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(GMAC_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(QEI_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(WDT_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(FADC_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(DMA0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(CAN_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(GPIOA_IRQHandler,                 Default_Handler)
WEAK_ALIAS_FUNC(GPIOB_IRQHandler,                 Default_Handler)
WEAK_ALIAS_FUNC(GPIOC_IRQHandler,                 Default_Handler)
WEAK_ALIAS_FUNC(GPIOD_IRQHandler,                 Default_Handler)
WEAK_ALIAS_FUNC(ADTIM0_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(SVPWM_IRQHandler,                 Default_Handler)
WEAK_ALIAS_FUNC(TIM0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM2_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM3_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(SARADC_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(RMS0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(RMS1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(RMS2_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(SINCOS0_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(SINCOS1_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(MATRIX_MULT_IRQHandler,           Default_Handler)
WEAK_ALIAS_FUNC(IIR0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(IIR1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(IIR2_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FIR0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FIR1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FIR2_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(LVD_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(SPWM_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FFT0_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FFT0_PREPROCESS_IRQHandler,       Default_Handler)
WEAK_ALIAS_FUNC(FFT1_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FFT1_PREPROCESS_IRQHandler,       Default_Handler)
WEAK_ALIAS_FUNC(FFT2_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(FFT2_PREPROCESS_IRQHandler,       Default_Handler)
WEAK_ALIAS_FUNC(EPWM_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(DFTRANS0_IRQHandler,              Default_Handler)
WEAK_ALIAS_FUNC(DFTRANS1_IRQHandler,              Default_Handler)
WEAK_ALIAS_FUNC(DFTRANS2_IRQHandler,              Default_Handler)
WEAK_ALIAS_FUNC(CRC_IRQHandler,                   Default_Handler)
WEAK_ALIAS_FUNC(ARCTAN0_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(ARCTAN1_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(ARCTAN2_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(DATADMA_IRQHandler,               Default_Handler)
WEAK_ALIAS_FUNC(WAKEUP_IRQHandler,                Default_Handler)
WEAK_ALIAS_FUNC(TIM4_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM5_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM6_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(TIM7_IRQHandler,                  Default_Handler)
WEAK_ALIAS_FUNC(EVSYS_IRQHandler,                 Default_Handler)

typedef void (*FUNC_IRQ)(void);

/* Vector table */
#if defined(__CC_ARM)           /* KEIL Compiler */
__attribute__((section("RESET")))
const FUNC_IRQ __vector_handlers[] = {
#elif defined(__GNUC__)         /* GCC Compiler */
__attribute__((section(".vector_table")))
const FUNC_IRQ __vector_handlers[] = {
#elif defined (__ICCARM__)      /* IAR Compiler */
extern uint32_t CSTACK$$Limit;
const FUNC_IRQ __vector_handlers[] @ ".intvec" = {
#endif

#if defined(__CC_ARM)           /* KEIL Compiler */
    (FUNC_IRQ)&Image$$ARM_LIB_STACK$$ZI$$Limit,
#elif defined(__GNUC__)         /* GCC Compiler */
    (FUNC_IRQ)&__StackTop,
#elif defined(__ICCARM__)       /* IAR Compiler */
    (FUNC_IRQ)&CSTACK$$Limit,
#endif

    Reset_Handler,              // Reset Handler
    NMI_Handler,                // NMI Handler
    HardFault_Handler,          // Hard Fault Handler
    MemManage_Handler,          // MPU Fault Handler
    BusFault_Handler,           // Bus Fault Handler
    UsageFault_Handler,         // Usage Fault Handler
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    0,                          // Reserved
    SVC_Handler,                // SVCall Handler
    DebugMon_Handler,           // Debug Monitor Handler
    0,                          // Reserved
    PendSV_Handler,             // PendSV Handler
    SysTick_Handler,            // SysTick Handler

    /* External Interrupts */
    IIC0_IRQHandler,            //0
    IIC1_IRQHandler,
    0,
    USART0_IRQHandler,
    USART1_IRQHandler,          //4
    USART2_IRQHandler,
    SPI0_IRQHandler, 
    SPI1_IRQHandler,
    HCC_IRQHandler,
    GMAC_IRQHandler,            //9
    QEI_IRQHandler,
    WDT_IRQHandler,
    FADC_IRQHandler,
    DMA0_IRQHandler,
    CAN_IRQHandler,             //14
    GPIOA_IRQHandler,
    GPIOB_IRQHandler,
    GPIOC_IRQHandler,
    GPIOD_IRQHandler,
    ADTIM0_IRQHandler,          //19
    0,
    SVPWM_IRQHandler,
    TIM0_IRQHandler,
    TIM1_IRQHandler,
    TIM2_IRQHandler,            //24
    TIM3_IRQHandler,
    0,
    SARADC_IRQHandler,
    RMS0_IRQHandler,
    RMS1_IRQHandler,            //29
    RMS2_IRQHandler,
    SINCOS0_IRQHandler,
    SINCOS1_IRQHandler,
    MATRIX_MULT_IRQHandler,
    IIR0_IRQHandler,            //34
    IIR1_IRQHandler,
    IIR2_IRQHandler,
    FIR0_IRQHandler,
    FIR1_IRQHandler,
    FIR2_IRQHandler,            //39
    SPWM_IRQHandler,
    FFT0_IRQHandler,
    FFT0_PREPROCESS_IRQHandler,
    FFT1_IRQHandler,
    FFT1_PREPROCESS_IRQHandler, //44
    FFT2_IRQHandler,
    FFT2_PREPROCESS_IRQHandler,
    EPWM_IRQHandler,
    LVD_IRQHandler,
    DFTRANS0_IRQHandler,        //49
    DFTRANS1_IRQHandler,
    DFTRANS2_IRQHandler,
    CRC_IRQHandler,
    ARCTAN0_IRQHandler,
    ARCTAN1_IRQHandler,         //54
    ARCTAN2_IRQHandler,         //55
    DATADMA_IRQHandler,         //56
    WAKEUP_IRQHandler,
    TIM4_IRQHandler,
    TIM5_IRQHandler,            //59
    TIM6_IRQHandler,
    TIM7_IRQHandler,
    EVSYS_IRQHandler,
};

extern void system_init(void);
extern int main(void);
extern int $Super$$main(void);
int $Sub$$main(void)
{
    system_init();
    return $Super$$main();
}

USED void Reset_Handler(void)
{
#if defined(__CC_ARM)           /* KEIL Compiler */
   __main();
#elif defined(__ICCARM__)       /* GCC Compiler */
    __iar_program_start();
#elif defined(__GNUC__)         /* IAR Compiler */
    uint32_t *src_ind = (uint32_t *) &__etext;
    uint32_t *dst_ind = (uint32_t *) &__data_start__;
    uint32_t *dst_end = (uint32_t *) &__data_end__;

    /* Move .data section from ROM to RAM */
    if(src_ind != dst_ind) {
        for(; dst_ind < dst_end;) {
            *dst_ind ++ = *src_ind ++;
        }
    }

    /* Initialize .bss section to zero */
    dst_ind = (uint32_t *) &__bss_start__;
    dst_end = (uint32_t *) &__bss_end__;
    if(dst_ind != dst_end) {
        for(; dst_ind < dst_end;) {
            *dst_ind ++ = 0;
        }
    }

    /* Initialize .bss.extern section to zero */
    dst_ind = (uint32_t *) &__bss_extern_start__;
    dst_end = (uint32_t *) &__bss_extern_end__;
    if(dst_ind != dst_end) {
        for(; dst_ind < dst_end;) {
            *dst_ind ++ = 0;
        }
    }

    //uvisor_init();

    if(software_init_hook) {
        /**
         * Give control to the RTOS via software_init_hook() which will also call __libc_init_array().
         * Assume software_init_hook() is defined in libraries/rtos/rtx/TARGET_CORTEX_M/RTX_CM_lib.h.
         */
        software_init_hook();
    } else {
        __libc_init_array();
        main();
    }
#endif
    /* Infinite loop */
    while(1);
}

void Default_Handler(void)
{
    while(1);
}

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
