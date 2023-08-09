/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/include/txf6200.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    08-04-2019
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer Header File.
  *          This file contains all the peripheral register's definitions, bits
  *          definitions and memory mapping for TXF6200 Connectivity line.
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripherals registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  *
  ******************************************************************************
  */ 

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup TXF6200
  * @{
  */

#ifndef __TXF6200_H
#define __TXF6200_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "typedef.h"
     
/* Macro Definitions */
#if defined(__CC_ARM)                /* KEIL Compiler */
    #pragma anon_unions
    #define WEAK            __attribute__ ((weak))
    #define ALIAS(f)        __attribute__ ((weak, alias(#f)))
    #define WEAK_ALIAS_FUNC(FUN, FUN_ALIAS) \
                void FUN(void) __attribute__ ((weak, alias(#FUN_ALIAS)));
    #define AT(n)           __attribute__((at(n)))
    #define NO_INIT         __attribute__((zero_init))
    #define USED            __attribute__((used))
#elif defined(__GNUC__)              /* GCC Compiler */
    #define WEAK            __attribute__ ((weak))
    #define ALIAS(f)        __attribute__ ((weak, alias(#f)))
    #define WEAK_ALIAS_FUNC(FUN, FUN_ALIAS) \
                void FUN(void) __attribute__ ((weak, alias(#FUN_ALIAS)));
    #define AT(n)           __attribute__((at(n)))
    #define NO_INIT         __attribute__((zero_init))
    #define USED            __attribute__((used))
#elif defined (__ICCARM__)           /* IAR Compiler */
    #define _WEAK_ALIAS_FUNC(FUN, FUN_ALIAS) weak __WEAK_ALIAS_FUNC(FUN, FUN_ALIAS)
    #define __WEAK_ALIAS_FUNC(FUN, FUN_ALIAS) FUN=FUN_ALIAS
    #define WEAK            _Pragma("weak")
    #define ALIAS(f)        _Pragma(_STRINGIFY(weak f))
    #define WEAK_ALIAS_FUNC(FUN, FUN_ALIAS) \
                void FUN(void) _Pragma(_STRINGIFY(_WEAK_ALIAS_FUNC(FUN, FUN_ALIAS)));
    #define AT(n)           @(n)
    #define NO_INIT         __no_init
    #define USED            __root
#endif

/**
  * @brief  set sysctal key valid & execute some code
  * @param  expression : code to execute
  * @retval None
  */
#define SYSCTRL_WRITE_ENABLE_KEY_VAL           (0x3fac87e4)
#define SYSCTRL_WRITE_DISABLE_KEY_VAL          (0x0)
  
#define SYSCTRL_REG_OPT(expression)                   \
do {                                                  \
    uint8_t ie = !__get_PRIMASK();                    \
    if(ie) __disable_irq();                           \
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_ENABLE_KEY_VAL;  \
    __ASM volatile ("nop");                           \
    expression;                                       \
    SYSCTRL->SYS_KEY = SYSCTRL_WRITE_DISABLE_KEY_VAL; \
    if(ie) __enable_irq();                            \
} while(0)

/** 
  * @brief  device defines  
  */
#define __CM3_REV                  0x0200U  /*!< Core Revision r2p0                           */
#define __MPU_PRESENT              0U       /*!< Other  devices does not provide an MPU       */
#define __NVIC_PRIO_BITS           3U       /*!<  uses 3 Bits for the Priority Levels         */
#define __Vendor_SysTickConfig     0U       /*!< Set to 1 if different SysTick Config is used */
     
/** @addtogroup bitband
  * @brief \@0x2000_0000 - 0x200F-FFFF
            @0x4000_0000 - 0x400F-FFFF
  * @{
  */
#define BITBAND_RAM(addr, bit) (*((uint32_t volatile*)(0x22000000u + (((uint32_t)&(addr) - (uint32_t)0x20000000u)<<5) + (((uint32_t)(bit))<<2))))
#define BITBAND_REG(addr, bit) (*((uint32_t volatile*)(0x42000000u + (((uint32_t)&(addr) - (uint32_t)0x40000000u)<<5) + (((uint32_t)(bit))<<2))))

/**
  * @}
  */

/** @addtogroup Peripheral_registers_structures
  * @{
  */
     
/**
 * @brief TXF6200 Interrupt Number Definition, according to the selected device
 */
typedef enum IRQn {
/******  Cortex-M3 Processor Exceptions Numbers ***********************************************/
    NonMaskableInt_IRQn         = -14,      /*!< 2 Non Maskable Interrupt                     */
    MemoryManagement_IRQn       = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt      */
    BusFault_IRQn               = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt              */
    UsageFault_IRQn             = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt            */
    SVCall_IRQn                 = -5,       /*!< 11 Cortex-M3 SV Call Interrupt               */
    DebugMonitor_IRQn           = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt         */
    PendSV_IRQn                 = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt               */
    SysTick_IRQn                = -1,       /*!< 15 Cortex-M3 System Tick Interrupt           */

/******  specific Interrupt Numbers ***********************************************************/
    IIC0_IRQn                   =  0,
    IIC1_IRQn                   =  1,
    AVDTMR0_UP_IRQn             =  2,
    USART0_IRQn                 =  3,
    USART1_IRQn                 =  4,
    USART2_IRQn                 =  5,
    SPI0_IRQn                   =  6,
    SPI1_IRQn                   =  7,
    HCC_IRQn                    =  8,
    GMAC_IRQn                   =  9,
    QEI_IRQn                    = 10,
    WDT_IRQn                    = 11,
    FADC_IRQn                   = 12,
    DMA0_IRQn                   = 13,
    CAN_IRQn                    = 14,
    GPIOA_IRQn                  = 15,
    GPIOB_IRQn                  = 16,
    GPIOC_IRQn                  = 17,
    GPIOD_IRQn                  = 18,
    AVDTMR0_CC_IRQn             = 19,
    AVDTMR0_TRG_IRQn            = 20,
    SVPWM_IRQn                  = 21,
    TIM0_IRQn                   = 22,
    TIM1_IRQn                   = 23,
    TIM2_IRQn                   = 24,
    TIM3_IRQn                   = 25,
    AVDTMR0_BRK_IRQn            = 26,
    SARADC_IRQn                 = 27,
    RMS0_IRQn                   = 28,
    RMS1_IRQn                   = 29,
    RMS2_IRQn                   = 30,
    SINCOS0_IRQn                = 31,
    SINCOS1_IRQn                = 32,
    MATRIX_MULT_IRQn            = 33,
    IIR0_IRQn                   = 34,
    IIR1_IRQn                   = 35,
    IIR2_IRQn                   = 36,
    FIR0_IRQn                   = 37,
    FIR1_IRQn                   = 38,
    FIR2_IRQn                   = 39,
    SPWM_IRQn                   = 40,
    FFT0_IRQn                   = 41,
    FFT0_PREPROCESS_IRQn        = 42,
    FFT1_IRQn                   = 43,
    FFT1_PREPROCESS_IRQn        = 44,
    FFT2_IRQn                   = 45,
    FFT2_PREPROCESS_IRQn        = 46,
    EPWM_IRQn                   = 47,
    LVD_IRQn                    = 48,
    DFTRANS0_IRQn               = 49,
    DFTRANS1_IRQn               = 50,
    DFTRANS2_IRQn               = 51,
    CRC_DMA_IRQn                = 52,
    ARCTAN0_IRQn                = 53,
    ARCTAN1_IRQn                = 54,
    ARCTAN2_IRQn                = 55,
    DATADMA_IRQn                = 56,
    WAKEPND_IRQn                = 57,
    TIM4_IRQn                   = 58,
    TIM5_IRQn                   = 59,
    TIM6_IRQn                   = 60,
    TIM7_IRQn                   = 61,
    EVSYS_IRQn                  = 62,
    AVDTMR1_BRK_IRQn            = 63,
    AVDTMR1_UP_IRQn             = 64,
    AVDTMR1_TRG_IRQn            = 65,
    AVDTMR1_CC_IRQn             = 66,
} IRQn_Type;

/**
  * @}
  */
#define ARM_MATH_CM3
#include "core_cm3.h" 

/** @addtogroup Peripheral_registers_structures
  * @{
  */

/**
  * @brief IIC/SMBUS (IP) 
  */

typedef struct {
    __IO uint32_t CON;
    __IO uint32_t TAR;
    __IO uint32_t SAR;
    __IO uint32_t HS_MADDR;
    
    __IO uint32_t DATA_CMD;
    __IO uint32_t SS_SCL_HCNT;
    __IO uint32_t SS_SCL_LCNT;
    __IO uint32_t FS_SCL_HCNT;
    
    __IO uint32_t FS_SCL_LCNT;
    __IO uint32_t HS_SCL_HCNT;
    __IO uint32_t HS_SCL_LCNT;
    __IO uint32_t INTR_STAT;
    
    __IO uint32_t INTR_MASK;
    __IO uint32_t RAW_INTR_STAT;
    __IO uint32_t RX_TL;
    __IO uint32_t TX_TL;
    
    __IO uint32_t CLR_INTR;
    __IO uint32_t CLR_RX_UNDER;
    __IO uint32_t CLR_RX_OVER;
    __IO uint32_t CLR_TX_OVER;
    
    __IO uint32_t CLR_RD_REQ;
    __IO uint32_t CLR_TX_ABRT;
    __IO uint32_t CLR_RX_DONE;
    __IO uint32_t CLR_ACTIVITY;
    
    __IO uint32_t CLR_STOP_DET;
    __IO uint32_t CLR_START_DET;
    __IO uint32_t CLR_GEN_CALL;
    __IO uint32_t ENABLE;
    
    __IO uint32_t STATUS;
    __IO uint32_t TXFLR;
    __IO uint32_t RXFLR;
    __IO uint32_t SDA_HOLD;
    __IO uint32_t TX_ABRT_SOURCE;
    __IO uint32_t SLV_DATA_NACK_ONLY;
    __IO uint32_t DMA_CR;
    __IO uint32_t DMA_TDLR;
    __IO uint32_t DMA_RDLR;
    __IO uint32_t SDA_SETUP;
    __IO uint32_t ACK_GENERAL_CALL;
    __IO uint32_t ENABLE_STATUS;
    __IO uint32_t FS_SPKLEN;
    __IO uint32_t HS_SPKLEN;
    __IO uint32_t CLR_RESTART_DET;
    __IO uint32_t SCL_STUCK_AT_LOW_TIMEOUT;
    __IO uint32_t SDA_STUCK_AT_LOW_TIMEOUT;
    __IO uint32_t CLR_SCL_STUCK_DET;
    __IO uint32_t DEVICE_ID;
    __IO uint32_t SMBUS_CLOCK_LOW_SEXT;
    __IO uint32_t SMBUS_CLOCK_LOW_MEXT;
    __IO uint32_t SMBUS_THIGH_MAX_IDLE_COUNT;
    __IO uint32_t SMBUS_INTR_STAT;
    __IO uint32_t SMBUS_INTR_MASK;
    __IO uint32_t SMBUS_INTR_RAW_STATUS;
    __IO uint32_t CLR_SMBUS_INTR;
    __IO uint32_t OPTIONAL_SAR;
    __IO uint32_t SMBUS_UDID_LSB;
} IIC_TypeDef;

/**
  * @brief USART(IP) 
  */
typedef struct {
    union {
        __IO uint32_t RBR;              /*!< USART Receive Buffer Register,                   RW, Address offset: 0x00 */ 
        __IO uint32_t THR;              /*!< USART Transmit Holding Register,                 RW, Address offset: 0x00 */
        __IO uint32_t DLL;              /*!< USART Divisor Latch Low,                         RW, Address offset: 0x00 */
    };
    union {
        __IO uint32_t IER;              /*!< USART Interrupt Enable Register,                 RW, Address offset: 0x04 */
        __IO uint32_t DLH;              /*!< USART Divisor Latch High,                        RW, Address offset: 0x04 */
    };
    union {
        __IO uint32_t FCR;              /*!< USART FIFO Control Register,                     RW, Address offset: 0x08 */
        __IO uint32_t IIR;              /*!< USART Interrupt Identity Register,               RW, Address offset: 0x08 */
    };

    __IO uint32_t LCR;                  /*!< USART Line Control Register,                     RW, Address offset: 0x0C */

    __IO uint32_t MCR;                  /*!< USART Modem Control Register,                    RW, Address offset: 0x10 */
    __IO uint32_t LSR;                  /*!< USART Line Status Register,                      RW, Address offset: 0x14 */
    __IO uint32_t MSR;                  /*!< USART Modem Status Register,                     RW, Address offset: 0x18 */
    __IO uint32_t SCR;                  /*!< USART Scratchpad Register,                       RW, Address offset: 0x1C */

    __IO uint32_t LPDLL;                /*!< USART Low Power Divisor Latch Low Register,      RW, Address offset: 0x20 */
    __IO uint32_t LPDLH;                /*!< USART Low Power Divisor Latch High Register,     RW, Address offset: 0x24 */
    __IO uint32_t FIFO[0x12];           /*!< USART Shadow Receive/Transmit FIFO Buf Register, RW, Address offset: 0x28 */

    __IO uint32_t FAR;                  /*!< USART FIFO Access Register,                      RW, Address offset: 0x70 */
    __IO uint32_t TFR;                  /*!< USART Transmit FIFO Read,                        RW, Address offset: 0x74 */
    __IO uint32_t RFW;                  /*!< USART Receive FIFO Write,                        RW, Address offset: 0x78 */
    __IO uint32_t USR;                  /*!< USART Status Register,                           RW, Address offset: 0x7C */

    __IO uint32_t TFL;                  /*!< USART Transmit FIFO Level,                       RW, Address offset: 0x80 */
    __IO uint32_t RFL;                  /*!< USART Receive FIFO Level,                        RW, Address offset: 0x84 */
    __IO uint32_t SRR;                  /*!< USART Software Reset Register,                   RW, Address offset: 0x88 */
    __IO uint32_t SRTS;                 /*!< USART Shadow Request to Send,                    RW, Address offset: 0x8C */

    __IO uint32_t SBCR;                 /*!< USART Shadow Break Control Register,             RW, Address offset: 0x90 */
    __IO uint32_t SDMAM;                /*!< USART Shadow DMA Mode,                           RW, Address offset: 0x94 */
    __IO uint32_t SFE;                  /*!< USART Shadow FIFO Enable,                        RW, Address offset: 0x98 */
    __IO uint32_t SRT;                  /*!< USART Shadow RCVR Trigger,                       RW, Address offset: 0x9C */

    __IO uint32_t STET;                 /*!< USART Shadow TX Empty Trigger,                   RW, Address offset: 0xA0 */
    __IO uint32_t HTX;                  /*!< USART Halt TX,                                   RW, Address offset: 0xA4 */
    __IO uint32_t DMASA;                /*!< USART DMA Software Acknowledge,                  RW, Address offset: 0xA8 */
    __IO uint32_t TCR;                  /*!< USART Transceiver Control Register,              RW, Address offset: 0xAC */

    __IO uint32_t DE_EN;                /*!< USART Driver Output Enable Register,             RW, Address offset: 0xB0 */
    __IO uint32_t RE_EN;                /*!< USART Receiver Output Enable Register,           RW, Address offset: 0xB4 */
    __IO uint32_t DET;                  /*!< USART Driver Output Enable Timing Register,      RW, Address offset: 0xB8 */
    __IO uint32_t TAT;                  /*!< USART TurnAround Timing Register,                RW, Address offset: 0xBC */

    __IO uint32_t DLF;                  /*!< USART Divisor Latch Fraction Register,           RW, Address offset: 0xC0 */
    __IO uint32_t RAR;                  /*!< USART Receive Address Register,                  RW, Address offset: 0xC4 */
    __IO uint32_t TAR;                  /*!< USART Transmit Address Register,                 RW, Address offset: 0xC8 */
    __IO uint32_t LCR_EXT;              /*!< USART Line Extended Control Register,            RW, Address offset: 0xCC */
} USART_TypeDef;


/**
  * @brief SPI (huge-ic)
  */
typedef struct {
    __IO uint32_t CFG;                  // 0x00
    __IO uint32_t CTL;                  // 0x04
    __IO uint32_t RX_BC;                // 0x08
    __IO uint32_t STA;                  // 0x0C
    __IO uint32_t WDATA;                // 0x10
    __IO uint32_t RDATA;                // 0x14
    __IO uint32_t DMA_TX_ADDR;          // 0x18
    __IO uint32_t DMA_RX_ADDR;          // 0x1C
    __IO uint32_t TX_BC;                // 0x20
    __IO uint32_t TX_START;             // 0x24
    __IO uint32_t RX_START;             // 0x28
} SPI_TypeDef;

/**
  * @brief LVD (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
} LVD_TypeDef;

/**
  * @brief Controller Area Network (IP)
  */
typedef struct {
    __IO uint32_t CANRBUF[0x12];        //0x00~0x47 Receive Buffer Registers
    __IO uint32_t CANTBUF[0x12];        //0x48~0x8f Transmit Buffer Registers
    __IO uint8_t  CFGSTA;               //0x90
    __IO uint8_t  TCMD;                 //0x91
    __IO uint8_t  TCTRL;                //0x92
    __IO uint8_t  RCTRL;                //0x93
                  
    __IO uint8_t  RTIE;                 //0x94
    __IO uint8_t  RTIF;                 //0x95
    __IO uint8_t  ERRINT;               //0x96
    __IO uint8_t  LIMIT;                //0x97
                  
    __IO uint8_t  BITTIME0;             //0x98
    __IO uint8_t  BITTIME1;             //0x99
    __IO uint8_t  BITTIME2;             //0x9a
    __IO uint8_t  reserved0;            //0x9b
                  
    __IO uint8_t  S_PRESC;              //0x9c
    __IO uint8_t  F_PRESC;              //0x9d
    __IO uint8_t  TDC;                  //0x9e
    __IO uint8_t  reserved1;            //0x9f
                  
    __IO uint8_t  EALCAP;               //0xa0
    __IO uint8_t  reserved2;            //0xa1
    __IO uint8_t  RECNT;                //0xa2
    __IO uint8_t  TECNT;                //0xa3
                  
    __IO uint8_t  ACFCTRL;              //0xa4
    __IO uint8_t  reserved3;            //0xa5
    __IO uint8_t  ACF_EN_0;             //0xa6
    __IO uint8_t  ACF_EN_1;             //0xa7
        
    __IO uint32_t ACF;                  //0xa8
    
    __IO uint32_t VER;
} CAN_TypeDef;

/**
  * @brief event system(huge-ic)
  */
typedef struct {
    __IO uint32_t CH_ENA;               /*!< EVSYS channel enable register,               RW,  Address offset: 0x00 */
    __IO uint32_t CH_INT_ENA;           /*!< EVSYS interrupt enable register,             RW,  Address offset: 0x04 */
    __IO uint32_t CH_CPU_KST;           /*!< EVSYS channel CPU kick start register,       RW,  Address offset: 0x08 */
         uint32_t RESERVED0;            /*!< EVSYS reservered,                            RW,  Address offset: 0x0c */
    __IO uint32_t CH_PND_CLR;           /*!< EVSYS channel pending clear register,        WO,  Address offset: 0x10 */
    __IO uint32_t CH_CPU_PND;           /*!< EVSYS channel CPU pending register,          RO,  Address offset: 0x14 */
    __IO uint32_t CH_HW_PND;            /*!< EVSYS channel hardware oending register,     RO,  Address offset: 0x18 */
    __IO uint32_t CH_MODE;              /*!< EVSYS channel mode select register,          RW,  Address offset: 0x1c */
    __IO uint32_t CH_SRC_CON0[16];      /*!< EVSYS source channel number register0,       RW,  Address offset: 0x20 */
    __IO uint32_t CH_SRC_CON1[16];      /*!< EVSYS source channel number register1,       RW,  Address offset: 0x60 */
    __IO uint32_t CH_DST_CON0[16];      /*!< EVSYS destination channel number register,   RW,  Address offset: 0xa0 */
} EVSYS_TypeDef;

/**
  * @brief svpwm(huge-ic)
  */

typedef struct
{
    __IO uint32_t CON0;                 /*!< SVPWM control register0,                    R/W, Address offset: 0x00  */
    __IO uint32_t CON1;                 /*!< SVPWM control register1,                    RW,  Address offset: 0x04  */
    __IO uint32_t CON2;                 /*!< SVPWM control register2,                    RW,  Address offset: 0x08  */
    __IO uint32_t CON3;                 /*!< SVPWM control register3,                    RW,  Address offset: 0x0c  */
    __IO uint32_t CON4;                 /*!< SVPWM control register4,                    RW,  Address offset: 0x10  */
    __IO uint32_t CON5;                 /*!< SVPWM control register5,                    RW,  Address offset: 0x14  */
                                                                                                                  
    __IO uint32_t REFA;                 /*!< SVPWM reference A register,                 RW,  Address offset: 0x18  */
    __IO uint32_t REFB;                 /*!< SVPWM reference B register,                 RW,  Address offset: 0x1c  */
    __IO uint32_t REFC;                 /*!< SVPWM reference C register,                 RW,  Address offset: 0x20  */
    __IO uint32_t STC0;                 /*!< SVPWM switching cycle register0,            RO,  Address offset: 0x24  */
    __IO uint32_t STC1;                 /*!< SVPWM switching cycle register1,            RO,  Address offset: 0x28  */
                                        
    __IO uint32_t MATCH_CON;            /*!< SVPWM counter Matching Control Registers,   RW,  Address offset: 0x2c  */
    __IO uint32_t ADC_CON;              /*!< SVPWM trigger ADC conversion Registers,     RW,  Address offset: 0x30  */
} SVPWM_TypeDef;


/**
  * @brief QEI (huge-ic)
  */
typedef struct {
    __IO uint32_t QEICON;
    __IO uint32_t DFLTCON;
    __IO uint32_t POSCNT;
    __IO uint32_t MAXCNT;
    __IO uint32_t QEIE;
    __IO uint32_t QEIFLAG;
    __IO uint32_t QEICLR;
    __IO uint32_t QEIO;
    __IO uint32_t QEI_TIMER_PERIOD;
    __IO uint32_t QEI_TIMER_CNT;
    __IO uint32_t QEI_TIMER_CNT_LATCH;
    __IO uint32_t QEI_ROTATE_PERIOD;
    __IO uint32_t QEI_ROTATE_CNT;
    __IO uint32_t QEI_ROTATE_CNT_LATCH;
    __IO uint32_t POSCNT_LATCH;
} QEI_TypeDef;

/**
  * @brief WDT (IP)
  */
typedef struct {
    __IO uint32_t WDT_CR;
    __IO uint32_t WDT_TORR;
    __IO uint32_t WDT_CCVR;
    __IO uint32_t WDT_CRR;
    __IO uint32_t WDT_STAT;
    __IO uint32_t WDT_EOI;
         uint32_t WDT_RSV[51];
    __IO uint32_t WDT_COMP_PARAMS_5;
    __IO uint32_t WDT_COMP_PARAMS_4;
    __IO uint32_t WDT_COMP_PARAMS_3;
    __IO uint32_t WDT_COMP_PARAMS_2;
    __IO uint32_t WDT_COMP_PARAMS_1;
    __IO uint32_t WDT_COMP_VERSION;
    __IO uint32_t WDT_COMP_TYPE;
} WDT_TypeDef;

/**
  * @brief WDT1 (Huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t KEY;    
} WDT1_TypeDef;

/**
  * @brief RTC (huge-ic)
  */
typedef struct {
    __IO uint32_t RTCCON;
    __IO uint32_t RTCCPND;
    __IO uint32_t RTCSECCNT;
    __IO uint32_t RTCDATA;
} RTC_TypeDef;

/**
  * @brief Normal TIMERs (huge-ic)
  */
typedef struct {
    __IO uint32_t TMR_CON;
    __IO uint32_t TMR_PR;
    __IO uint32_t TMR_CNT;
    __IO uint32_t TMR_PWM;
} TIMER_TypeDef;

/**
  * @brief Advanced-control timer (huge-ic) 
  */
typedef struct
{   
  __IO uint32_t CR1;                    /*!< TIM control register 1,                     RW, Address offset: 0x00 */
  __IO uint32_t CR2;                    /*!< TIM control register 2,                     RW, Address offset: 0x04 */
  __IO uint32_t SMCR;                   /*!< TIM slave Mode Control register,            RW, Address offset: 0x08 */
  __IO uint32_t DIER;                   /*!< TIM DMA/interrupt enable register,          RW, Address offset: 0x0C */
  __IO uint32_t SR;                     /*!< TIM status register,                        RW, Address offset: 0x10 */
  __IO uint32_t EGR;                    /*!< TIM event generation register,              RW, Address offset: 0x14 */
  __IO uint32_t CCMR1;                  /*!< TIM capture/compare mode register 1,        RW, Address offset: 0x18 */
  __IO uint32_t CCMR2;                  /*!< TIM capture/compare mode register 2,        RW, Address offset: 0x1C */
  __IO uint32_t CCER;                   /*!< TIM capture/compare enable register,        RW, Address offset: 0x20 */
  __IO uint32_t CNT;                    /*!< TIM counter register,                       RW, Address offset: 0x24 */
  __IO uint32_t PSC;                    /*!< TIM prescaler register,                     RW, Address offset: 0x28 */
  __IO uint32_t ARR;                    /*!< TIM auto-reload register,                   RW, Address offset: 0x2C */
  __IO uint32_t RCR;                    /*!< TIM repetition counter register,            RW, Address offset: 0x30 */
  __IO uint32_t CCR1;                   /*!< TIM capture/compare register 1,             RW, Address offset: 0x34 */    
  __IO uint32_t CCR2;                   /*!< TIM capture/compare register 2,             RW, Address offset: 0x38 */    
  __IO uint32_t CCR3;                   /*!< TIM capture/compare register 3,             RW, Address offset: 0x3C */
  __IO uint32_t CCR4;                   /*!< TIM capture/compare register 4,             RW, Address offset: 0x40 */
  __IO uint32_t BDTR;                   /*!< TIM break and dead-time register,           RW, Address offset: 0x44 */
  __IO uint32_t DCR;                    /*!< TIM DMA control register,                   RW, Address offset: 0x48 */
  __IO uint32_t DMAR;                   /*!< TIM DMA address for full transfer register, RW, Address offset: 0x4C */
  __IO uint32_t OR;                     /*!< TIM option register,                        RW, Address offset: 0x50 */
  __IO uint32_t CCMR3;                  /*!< TIM capture/compare mode register 3         RW, Address offset: 0x54 */
  __IO uint32_t CCR5;                   /*!< TIM capture/compare register 5              RW, Address offset: 0x58 */
  __IO uint32_t CCR6;                   /*!< TIM capture/compare register 6              RW, Address offset: 0x5C */
  __IO uint32_t AF1;                    /*!< TIM alternate function option register 1    RW, Address offset: 0x60 */
  __IO uint32_t AF2;                    /*!< TIM alternate function option register 2    RW, Address offset: 0x64 */
    
} ADVTMR_TypeDef;

/**
  * @brief HCC (huge-ic) 
  */
typedef struct {
    __IO uint32_t HCC_STADR_FFT;
    __IO uint32_t HCC_STADR_PHS;
    __IO uint32_t HCC_STADR_RAG;
    __IO uint32_t HCC_CONTROL;
    __IO uint32_t HCC_RESULT;
} HCC_TypeDef;

/**
  * @brief CRC (huge-ic)
  */
typedef struct {
    __IO uint32_t CRC_CFG;              // 0x0c
    __IO uint32_t CRC_INIT;             // 0x04
    __IO uint32_t CRC_INV;              // 0x08
    __IO uint32_t CRC_POLY;             // 0x0c 
    __IO uint32_t CRC_KST;              // 0x10
    __IO uint32_t CRC_STA;              // 0x14
         uint32_t RESERVED0;
    __IO uint32_t DMA_ADDR;             // 0x1c
    __IO uint32_t DMA_LEN;              // 0x20
    __IO uint32_t CRC_OUT;              // 0x24
} CRC_TypeDef;

/**
  * @brief Frac PLL 0 (huge-ic)
  */
typedef struct {
    __IO uint32_t FPLL0_CON;
    __IO uint32_t FPLL0_INT;
    __IO uint32_t FPLL0_FRAC;
    __IO uint32_t FPLL0_SSC;
} FPLL0_TypeDef;

/**
  * @brief Frac PLL 1 (huge-ic)
  */
typedef struct {
    __IO uint32_t FPLL1_CON;
    __IO uint32_t FPLL1_INT;
    __IO uint32_t FPLL1_FRAC;
    __IO uint32_t FPLL1_SSC;
} FPLL1_TypeDef;

/**
  * @brief SYS_CTRL (huge-ic)
  */
typedef struct {
    __IO uint32_t CLK_CON0;
    __IO uint32_t CLK_CON1;
    __IO uint32_t CLK_CON2;
    __IO uint32_t CLK_CON3;
    __IO uint32_t CLK_CON4;
    __IO uint32_t CLK_CON5;
    
    __IO uint32_t SYS_CON0;
    __IO uint32_t SYS_CON1;
    __IO uint32_t SYS_CON2;
    __IO uint32_t SYS_CON3;
    __IO uint32_t SYS_CON4;
    __IO uint32_t SYS_CON5;
    __IO uint32_t SYS_CON6;
    __IO uint32_t SYS_CON7;
    __IO uint32_t SYS_CON8;
    
    __IO uint32_t AIP_CON0;
    __IO uint32_t AIP_CON1;
    
    __IO uint32_t IO_MAP;
    __IO uint32_t IO_MAP1;
    
    __IO uint32_t PMUREG0;
    __IO uint32_t PMUREG4;
    
    __IO uint32_t PWM_KEY;
    __IO uint32_t SYS_KEY;
    
    __IO uint32_t DMA_ERR0;
    __IO uint32_t DMA_ERR1;
    
    __IO uint32_t HOSC_MNT;
    __IO uint32_t WKUP_CON0;
    __IO uint32_t LP_CON0;
    __IO uint32_t MBIST_CON0;
    __IO uint32_t MBIST_MISR;
    __IO uint32_t SPWM_SOFTRESET;       //SYSCTRL->SYS_KEY = 0x4c5de9b3
    __IO uint32_t CHIP_ID;
    __IO uint32_t MODE_REG;
} SYSCTRL_TypeDef;

/**
  * @brief eFlash controller (huge-ic)
  */
typedef struct {
    __IO uint32_t CTRLR0;               /*!< EFLASH control register,                     RW,  Address offset: 0x00           */
    __IO uint32_t KST;                  /*!< EFLASH kick start register,                  RO,  Address offset: 0x04           */
    __IO uint32_t DONE;                 /*!< EFLASH finish flag register,                 RO,  Address offset: 0x08           */
    __IO uint32_t RESERVED0;            /*!< EFLASH reserved0,                            RO,  Address offset: 0x0c           */
    __IO uint32_t PROG_ADDR;            /*!< EFLASH program address register,             RW,  Address offset: 0x10           */
    __IO uint32_t PROG_BYTE;            /*!< EFLASH program byte register,                RW,  Address offset: 0x14           */
    __IO uint32_t PROG_DATA;            /*!< EFLASH program data register,                RW,  Address offset: 0x18           */
         uint32_t RESERVED1;            /*!< EFLASH reserve1,                             RO,  Address offset: 0x1c           */
    __IO uint32_t ERASE_CTRL;           /*!< EFLASH erase register,                       R/W, Address offset: 0x20           */
         uint32_t RESERVED2[3];         /*!< EFLASH reserve2,                             RO,  Address offset: 0x24/0x28/0x2c */
    __IO uint32_t TIME_REG0;            /*!< EFLASH time register0,                       RW,  Address offset: 0x30           */
    __IO uint32_t TIME_REG1;            /*!< EFLASH time register1,                       RW,  Address offset: 0x34           */
    __IO uint32_t TIME_REG2;            /*!< EFLASH time register2,                       RW,  Address offset: 0x38           */
    __IO uint32_t TIME_REG3;            /*!< EFLASH time register3,                       RW,  Address offset: 0x3c           */
         uint32_t RESERVED3[4];         /*!< EFLASH reserve5,                             RO,  Address offset: 0x40-0x4c      */
    __IO uint32_t NVR_PASSWORD;         /*!< EFLASH NVR password register,                RW,  Address offset: 0x50           */
    __IO uint32_t MAIN_PASSWORD;        /*!< EFLASH MAIN password register,               RW,  Address offset: 0x54           */
    __IO uint32_t CRC_DMA;              /*!< EFLASH CRC DMA register,                     RW,  Address offset: 0x58           */
    __IO uint32_t CRC_OUT;              /*!< EFLASH CRC OUT register,                     RW,  Address offset: 0x5c           */
         uint32_t RESERVED4[4];         /*!< EFLASH reserve9,                             RO,  Address offset: 0x60-0x6c      */
    __IO uint32_t MODE_STA;             /*!< EFLASH mode status register,                 RO,  Address offset: 0x70           */
    __IO uint32_t PERMISSION0;          /*!< EFLASH NVR Hardware control permission,      RO,  Address offset: 0x74           */
    __IO uint32_t PERMISSION1;          /*!< EFLASH MAIN Hardware control permission,     RO,  Address offset: 0x78           */
} EFLASH_TypeDef;

/**
  * @brief GPIO controller (huge-ic)
  */
typedef struct {
    __IO uint32_t DR;                   /*!< GPIO port data register,                    RW,  Address offset: 0x00  */  
    __IO uint32_t DIR;                  /*!< GPIO port direct register,                  RW,  Address offset: 0x04  */ 
    __IO uint32_t INTMASK;              /*!< GPIO port interrupt mask register,          RW,  Address offset: 0x08  */ 
    __IO uint32_t PU0EN;                /*!< GPIO port pull up register,10K,             RW,  Address offset: 0x0C  */ 
         uint32_t RESERVED0;            /*!< GPIO port reserve0,                         RO,  Address offset: 0x10  */ 
         uint32_t RESERVED1;            /*!< GPIO port reserve1,                         RO,  Address offset: 0x14  */        
    __IO uint32_t PD0EN;                /*!< GPIO port pull down register,10K,           RW,  Address offset: 0x18  */ 
         uint32_t RESERVED2;            /*!< GPIO port reserve2,                         RO,  Address offset: 0x1C  */  
         uint32_t RESERVED3;            /*!< GPIO port reserve3,                         RO,  Address offset: 0x20  */   
    __IO uint32_t DS;                   /*!< GPIO port drive strong register,            RW,  Address offset: 0x24  */      
    __IO uint32_t HY;                   /*!< GPIO port hardware register,                RW,  Address offset: 0x28  */  
    __IO uint32_t OD;                   /*!< GPIO port open-drain register,              RW,  Address offset: 0x2c  */  
    __IO uint32_t SR;                   /*!< GPIO port slow rate register,               RW,  Address offset: 0x30  */   
    __IO uint32_t DIE;                  /*!< GPIO port digtial enable register,          RW,  Address offset: 0x34  */ 
    __IO uint32_t BSRS;                 /*!< GPIO port bit set/reset(H16/L16) register,  RW,  Address offset: 0x38  */ 
} GPIO_TypeDef;

/**
  * @brief DMAC controller (IP)
  */
typedef struct {
    __IO uint32_t SARL;
    __IO uint32_t SARH;
    __IO uint32_t DARL;
    __IO uint32_t DARH;
    __IO uint32_t FIFO0[2];             //LLPL,LLPH, NO USE
    __IO uint32_t CTLL;
    __IO uint32_t CTLH;
    __IO uint32_t FIFO1[8];             //SSTATL,SSTATH,DSTATL,DSTATH,SSTATARL,SSTATARH,DSTATARL,DSTATARH
    __IO uint32_t CFGL;
    __IO uint32_t CFGH;
    __IO uint32_t SGRL;
    __IO uint32_t SGRH;
    __IO uint32_t DSRL;
    __IO uint32_t DSRH;
} DMAC_CH_TypeDef;

/**
  * @brief DMAC
  */
typedef struct {
    DMAC_CH_TypeDef CH0;
    DMAC_CH_TypeDef CH1;
    DMAC_CH_TypeDef CH2;
    DMAC_CH_TypeDef CH3;
    __IO uint32_t   FIFO0[22*4];
    __IO uint32_t   RawTfrL;
    __IO uint32_t   RawTfrH;
    __IO uint32_t   RawBlockL;
    __IO uint32_t   RawBlockH;
    __IO uint32_t   RawSrcTranL;
    __IO uint32_t   RawSrcTranH;
    __IO uint32_t   RawDstTranL;
    __IO uint32_t   RawDstTranH;
    __IO uint32_t   RawErrL;
    __IO uint32_t   RawErrH;
    __IO uint32_t   FIFO1[5*2];
    __IO uint32_t   MaskTfrL;
    __IO uint32_t   MaskTfrH;
    __IO uint32_t   MaskBlockL;
    __IO uint32_t   MaskBlockH;
    __IO uint32_t   MaskSrcTranL;
    __IO uint32_t   MaskSrcTranH;
    __IO uint32_t   MaskDstTranL;
    __IO uint32_t   MaskDstTranH;
    __IO uint32_t   MaskErrL;
    __IO uint32_t   MaskErrH;
    __IO uint32_t   ClearTfrL;
    __IO uint32_t   ClearTfrH;
    __IO uint32_t   ClearBlockL;
    __IO uint32_t   ClearBlockH;
    __IO uint32_t   ClearSrcTranL;
    __IO uint32_t   ClearSrcTranH;
    __IO uint32_t   ClearDstTranL;
    __IO uint32_t   ClearDstTranH;
    __IO uint32_t   ClearErrL;
    __IO uint32_t   ClearErrH;
    __IO uint32_t   FIFO2[2];
    __IO uint32_t   ReqSrcRegL;
    __IO uint32_t   ReqSrcRegH;
    __IO uint32_t   ReqDstRegL;
    __IO uint32_t   ReqDstRegH;
    __IO uint32_t   SglReqSrcRegL;
    __IO uint32_t   SglReqSrcRegH;
    __IO uint32_t   SglReqDstRegL;
    __IO uint32_t   SglReqDstRegH;
    __IO uint32_t   LstSrcRegL;
    __IO uint32_t   LstSrcRegH;
    __IO uint32_t   LstDstRegL;
    __IO uint32_t   LstDstRegH;
    __IO uint32_t   DmaCfgRegL;
    __IO uint32_t   DmaCfgRegH;
    __IO uint32_t   ChEnRegL;
    __IO uint32_t   ChEnRegH;
    __IO uint32_t   FIFO3[2];
    __IO uint32_t   DmaTestRegL;
    __IO uint32_t   DmaTestRegH;
} DMAC_TypeDef;

/**
  * @brief Ethernet GMAC controller (IP modify)
  */
typedef struct {
    __IO uint32_t CSR0;
         uint32_t RESERVED0;
    __IO uint32_t CSR1;
         uint32_t RESERVED1;
    __IO uint32_t CSR2;
         uint32_t RESERVED2;
    __IO uint32_t CSR3;
         uint32_t RESERVED3;
    __IO uint32_t CSR4;
         uint32_t RESERVED4;
    __IO uint32_t CSR5;
         uint32_t RESERVED5;
    __IO uint32_t CSR6;
         uint32_t RESERVED6;
    __IO uint32_t CSR7;
         uint32_t RESERVED7;
    __IO uint32_t CSR8;
         uint32_t RESERVED8;
    __IO uint32_t CSR9;
         uint32_t RESERVED9;
    __IO uint32_t CSR10;
         uint32_t RESERVED10;
    __IO uint32_t CSR11;
         uint32_t RESERVED11;
} GMAC_TypeDef;

/** 
  * @brief sin&cos (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t LEN;
    __IO uint32_t STEP;
    __IO uint32_t DATA_IN;
    __IO uint32_t DATA_OUT;
    __IO uint32_t DATA_IN_ADR;
    __IO uint32_t DATA_OUT_ADR;
         uint32_t RESERVED[9];
} SINCOS_TypeDef;

/**
  * @brief RMS (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t LEN;
    __IO uint32_t IN_FRAC_WIDTH;
    __IO uint32_t OUT_FRAC_WIDTH;
    __IO uint32_t DATA_IN;
    __IO uint32_t DATA_OUTL;
    __IO uint32_t DATA_OUTH;
    __IO uint32_t DATA_IN_ADR;
    __IO uint32_t DATA_OUT_ADR;
         uint32_t RESERVED[7];
} RMS_TypeDef;

/**
  * @brief MATRIX channel typedef (huge-ic)
  */
typedef struct {
    __IO uint32_t DATAIN_STADR0;
    __IO uint32_t DATAIN_STADR1;
    __IO uint32_t DATAIN_STADR2;
    __IO uint32_t DATAOUT_STADR0;
    __IO uint32_t DATAOUT_STADR1;
    __IO uint32_t DATAOUT_STADR2;
    __IO uint32_t LEN;
    __IO uint32_t INDEX;
    __IO uint32_t COEF_FRAC_WIDTH;
    __IO uint32_t OUT_FRAC_WIDTH; 
} MATRIX_CH_TypeDef;

/**
  * @brief MATRIX (huge-ic)
  */
typedef struct {
    MATRIX_CH_TypeDef CH[4];
    __IO uint32_t     COEF[2][9];
    __IO uint32_t     MATRIX_COEF_SEL;
    __IO uint32_t     MATRIX_EN;
    __IO uint32_t     MATRIX_PEND;
    __IO uint32_t     MATRIX_IE;
         uint32_t     RESERVED[18];
} MATRIX_TypeDef;

/**
  * @brief SPWM (huge-ic)
  */
typedef struct {
    __IO uint32_t SPWM_CON;
    __IO uint32_t SPWM_PERIOD0;
    __IO uint32_t SPWM_DATAUSE0;
    __IO uint32_t SPWM_DATABUF0;
    __IO uint32_t SPWM_DATAUSE1;
    __IO uint32_t SPWM_DATABUF1;
    __IO uint32_t SPWM_DATAUSE2;
    __IO uint32_t SPWM_DATABUF2;
    __IO uint32_t SPWM_FAULT_INFO;
    __IO uint32_t SPWM_FAULT_INVERT;
    __IO uint32_t SPWM_PERIOD1;
    __IO uint32_t SPWM_MATCH;
    __IO uint32_t SPWM_ADCC;
    __IO uint32_t SPWM_CNT0;
    __IO uint32_t SPWM_CNT1;
    __IO uint32_t SPWM_CNT2;
} SPWM_TypeDef;


/**
  * @brief FFT (huge-ic)
  */
typedef struct {
    __IO uint32_t DMA_CON;
    __IO uint32_t REAL_STADR;
    __IO uint32_t IMAG_STADR;
    __IO uint32_t WINDOW_STADR;
    __IO uint32_t REALIMAG_OUT_STADR;
    __IO uint32_t INDEX;
    __IO uint32_t LEN;
    __IO uint32_t STADR;
    __IO uint32_t KS;
    __IO uint32_t CTRL;
    __IO uint32_t STAT; 
    __IO uint32_t MAXIMUM; 
         uint32_t RESERVED[4];
} FFT_TypeDef;

/**
  * @brief DFT (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t LEN;
    __IO uint32_t INDEX;
    __IO uint32_t STEP;
    __IO uint32_t REAL_STADR;
    __IO uint32_t IMAG_STADR;
    __IO uint32_t OUT;
    __IO uint32_t DMA_LEN;
    __IO uint32_t NORMALIZED_COEF;
    __IO uint32_t OUT_ADR;
         uint32_t RESERVED[6];
} DFT_TypeDef;

/**
  * @brief ARCTAN (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t DMA_LEN;
    __IO uint32_t DMA0_STADR;
    __IO uint32_t DMA1_STADR;
    __IO uint32_t DMA2_STADR;
    __IO uint32_t IN;
    __IO uint32_t OUT; 
         uint32_t RESERVED[9];
} ARCTAN_TypeDef;

/**
  * @brief DATADMA (huge-ic)
  */
typedef struct {
    __IO uint32_t CON;
    __IO uint32_t SRC_BUF_STADR;
    __IO uint32_t SRC_BUF_LEN;
    __IO uint32_t SRC_DMA_ADR;
    __IO uint32_t DEST_DMA_ADR;
    __IO uint32_t DMA_LEN; 
         uint32_t RESERVED[10];
} DATADMA_TypeDef;

/** 
  * @brief saradc (huge-ic)
  */
typedef struct {
    __IO uint32_t SARADC_CON;
         uint32_t RESERVED[4];
    __IO uint32_t SARADC_COM_ACFG;
    __IO uint32_t SARADC_CON1;
         uint32_t RESERVED0;
    __IO uint32_t SARADC_CDIV_DMALEN[19];
         uint32_t RESERVED1;
    __IO uint32_t SARADC_DMASTADDR[14];
         uint32_t RESERVED2;
         uint32_t RESERVED3;
    __IO uint32_t SARADC_ACFG[19];
         uint32_t RESERVED4;
    __IO  int16_t SARADC_DATA[19];
         uint16_t RESERVED5;
         uint32_t RESERVED6;
         uint32_t RESERVED7;
    __IO uint32_t DACCMP_CON[5];
         uint32_t RESERVED8;

    __IO uint32_t SARADC_QUANTIFY_CON[14];
         uint32_t RESERVED9[5];
    __IO uint32_t SARADC_START_POINT[19];
    __IO uint32_t SARADC_PENDING0;
    __IO uint32_t SARADC_PENDING1;
    __IO uint32_t SARADC_PENDING2;
    __IO uint32_t SARADC_PENDING0_CLR;
    __IO uint32_t SARADC_PENDING1_CLR;
    __IO uint32_t SARADC_PENDING2_CLR;
    __IO uint32_t SARADC_INT_CONTROL0;
    __IO uint32_t SARADC_INT_CONTROL1;
    __IO uint32_t SARADC_INT_CONTROL2;

    __IO uint32_t DACCMP_DEBOUNCE[5];
    __IO uint32_t DACCMP_DATA[5];
    __IO uint32_t SARADC_DMA_INDEX[14];
} ADC_TypeDef;

/** 
  * @brief iir (huge-ic)
  */
typedef struct {
    __IO uint32_t IIR_CH_ENA0;          // 0x00
    __IO uint32_t IIR_CPU_KST0;         // 0x04
    __IO uint32_t IIR_CFG_ADDR;         // 0x08
    __IO uint32_t IIR_INT_ENA0;         // 0x0c
    __IO uint32_t IIR_HALF_PND0;        // 0x10
    __IO uint32_t IIR_FULL_PND0;        // 0x14
    __IO uint32_t IIR_FILT_PND0;        // 0x18
    __IO uint32_t IIR_INT_SRCL0;        // 0x1c
    __IO uint32_t IIR_INT_SRCH0;        // 0x20
    __IO uint32_t IIR_DATA_OUT;         // 0x24
    __IO uint32_t EVSYS_CH_ENA0;        // 0x28
         uint32_t RESERVED0;            // 0x2c

    __IO uint32_t IIR_CH_ENA1;          // 0x30
    __IO uint32_t IIR_CPU_KST1;         // 0x34
         uint32_t RESERVED07;           // 0x38
    __IO uint32_t IIR_INT_ENA1;         // 0x3c
    __IO uint32_t IIR_HALF_PND1;        // 0x40
    __IO uint32_t IIR_FULL_PND1;        // 0x44
    __IO uint32_t IIR_FILT_PND1;        // 0x48
    __IO uint32_t IIR_INT_SRCL1;        // 0x4c
    __IO uint32_t IIR_INT_SRCH1;        // 0x50
         uint32_t RESERVED10;           // 0x54
    __IO uint32_t EVSYS_CH_ENA1;        // 0x58
         uint32_t RESERVED12;           // 0x5c
} IIR_TypeDef;  


/** 
  * @brief fir (huge-ic)
  */
typedef struct {
    __IO uint32_t FIR_CH_ENA;           // 0x00
    __IO uint32_t FIR_CPU_KST;          // 0x04
    __IO uint32_t FIR_CFG_ADDR;         // 0x08
    __IO uint32_t FIR_INT_ENA;          // 0x0c
    __IO uint32_t FIR_HALF_PND;         // 0x10
    __IO uint32_t FIR_FULL_PND;         // 0x14
    __IO uint32_t FIR_FILT_PND;         // 0x18
    __IO uint32_t FIR_INT_SRCL;         // 0x1c
    __IO uint32_t FIR_INT_SRCH;         // 0x20
    __IO uint32_t FIR_DATA_OUT;         // 0x24
    __IO uint32_t EVSYS_CH_ENA;         // 0x28
         uint32_t RESERVED02;           // 0x2c
} FIR_TypeDef;  

/** 
  * @brief EPWM (huge-ic)
  */
typedef struct {
     __IO uint32_t EPWM_TBCTL;          /*!< EPWM Time-Base Control Register,                            RW,  Address offset: 0x00  */
     __IO uint32_t EPWM_TBPRD;          /*!< EPWM Time-Base Period(Shadow Data) Register,                RW,  Address offset: 0x04  */
     __IO uint32_t EPWM_TBPHASE;        /*!< EPWM Phase Data Register,                                   RW,  Address offset: 0x08  */
     __IO uint32_t EPWM_CMPCTL;         /*!< EPWM Counter-Compare Control Register,                      RW,  Address offset: 0x0C  */
     __IO uint32_t EPWM_CMPA;           /*!< EPWM Counter-Compare A Register,                            RO,  Address offset: 0x10  */
     __IO uint32_t EPWM_CMPB;           /*!< EPWM Counter-Compare B Register,                            RO,  Address offset: 0x14  */
     __IO uint32_t EPWM_CMPC;           /*!< EPWM Counter-Compare C Register,                            RW,  Address offset: 0x18  */
     __IO uint32_t EPWM_AQCTLAB;        /*!< EPWM Action-Qualifier Control Register for Output A and B,  RO,  Address offset: 0x1C  */
     __IO uint32_t EPWM_AQSFRC;         /*!< EPWM Action-Qualifier Software Force Register,              RO,  Address offset: 0x20  */
     __IO uint32_t EPWM_AQCSFRC;        /*!< EPWM Action-Qualifier Continuous S/W Force Register Set,    RW,  Address offset: 0x24  */
     __IO uint32_t EPWM_DBCTL;          /*!< EPWM Dead-Band Generator Control Register,                  RW,  Address offset: 0x28  */
     __IO uint32_t EPWM_DBDELAY;        /*!< EPWM Dead-Band Generator Edge Delay Count Register          RW,  Address offset: 0x2c  */
     __IO uint32_t EPWM_ETCTL;          /*!< EPWM Event-Trigger Control Register,                        RW,  Address offset: 0x30  */
     __IO uint32_t EPWM_ETFLAG;         /*!< EPWM Event-Trigger Flag Register,                           RW,  Address offset: 0x34  */
     __IO uint32_t EPWM_DCCTL;          /*!< EPWM Digital Compare Control Register                       RW,  Address offset: 0x38  */
     __IO uint32_t EPWM_DCTRIPSEL;      /*!< EPWM Digital Compare Trip Select Register,                  RW,  Address offset: 0x24  */
     __IO uint32_t EPWM_DCCAP;          /*!< EPWM Digital Compare Capture Control Register,              RW,  Address offset: 0x28  */
     __IO uint32_t EPWM_BLANKOFFSET;    /*!< EPWM Blank Offset Data Register,                            RW,  Address offset: 0x2c  */
     __IO uint32_t EPWM_WINWIDTH;       /*!< EPWM Window Width Data Register,                            RW,  Address offset: 0x30  */
     __IO uint32_t EPWM_TZCTL;          /*!< EPWM Trip-Zone Control Register,                            RW,  Address offset: 0x34  */
     __IO uint32_t EPWM_TZFLAG;         /*!< EPWM Trip-Zone Flag Register,                               RW,  Address offset: 0x38  */
} EPWM_CH_TypeDef;

typedef struct {
     EPWM_CH_TypeDef CH[7];             /*!< EPWM Channel0 To Channel6 Control Config Register           RW,  Address offset: 0x00  */
     __IO uint32_t EPWM_TTCTL;          /*!< EPWM Control Config Register                                RW,  Address offset: 0x24C */
     __IO uint32_t EPWM_ADCSEL0;        /*!< EPWM Adc0 To Adc9 Select Register                           RW,  Address offset: 0x250 */
     __IO uint32_t EPWM_ADCSEL1;        /*!< EPWM Adca0 To Adc13 Select Register                         RW,  Address offset: 0x254 */
} EPWM_TypeDef;  

/** 
  * @brief fadc (huge-ic)
  */
typedef struct {
    __IO uint32_t FADCCON0;             // 0x00
    __IO uint32_t FADCACSCON;           // 0x04
    __IO uint32_t FADCACSDAT;           // 0x08
    __IO uint32_t FADCINT0;             // 0x0c
    __IO uint32_t FADCINT1;             // 0x10
    __IO uint32_t FADCSMPFLAG;          // 0x14
    __IO uint32_t FADCDMAFLAG0;         // 0x18
    __IO uint32_t FADCDMAFLAG1;         // 0x1c
    __IO uint32_t FADCDMAADDR[16];      // 0x20 ~ 0x5C
    __IO uint32_t FADCDMALEN[16];       // 0x60 ~ 0x9c
    __IO uint32_t FADCPPROC0CON[16];    // 0xa0 ~ 0xdc
    __IO uint32_t FADCPPROC1CON[16];    // 0xe0 ~ 0x11c
    __IO  int16_t FADCRES[16];          // 0x120 ~ 0x13c
} FADC_TypeDef;  

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup Peripheral_memory_map
  * @{
  */
/*! FLASH base address in the alias region */
#define FLASH_BASE              ((uint32_t)0x08000000)
/*! SRAM base address in the alias region */
#define SRAM_BASE               ((uint32_t)0x20000000)
/*! Peripheral base address in the alias region */
#define PERIPH_BASE             ((uint32_t)0x40000000)

//--------------Peripheral memory map------------------//
#define APB0_BASE               PERIPH_BASE
#define APB1_BASE               (PERIPH_BASE + 0x10000)
#define APB2_BASE               (PERIPH_BASE + 0x30000)

//--------------APB0 bus peris map---------------------//
#define IIC0_BASE               (APB0_BASE + 0x0000)
#define IIC1_BASE               (APB0_BASE + 0x1000)
#define USART0_BASE             (APB0_BASE + 0x3000)
#define USART1_BASE             (APB0_BASE + 0x4000)
#define USART2_BASE             (APB0_BASE + 0x5000)
#define SPI0_BASE               (APB0_BASE + 0x6000)
#define SPI1_BASE               (APB0_BASE + 0x7000)
#define LVD_BASE                (APB0_BASE + 0x9000)
#define ADVTMR0_BASE            (APB0_BASE + 0xA000)
#define TIMER4_BASE             (APB0_BASE + 0xD000)
#define TIMER5_BASE             (APB0_BASE + 0xD010)
#define TIMER6_BASE             (APB0_BASE + 0xD020)
#define TIMER7_BASE             (APB0_BASE + 0xD030)

//--------------APB1 bus peris map---------------------//
#define QEI_BASE                (APB1_BASE + 0x2000)
#define WDT_BASE                (APB1_BASE + 0x3000)
#define WDT1_BASE               (APB1_BASE + 0x4000)
#define TIMER0_BASE             (APB1_BASE + 0x5000)
#define TIMER1_BASE             (APB1_BASE + 0x5010)
#define TIMER2_BASE             (APB1_BASE + 0x5020)
#define TIMER3_BASE             (APB1_BASE + 0x5030)
#define CRC_BASE                (APB1_BASE + 0x7000)
#define FPLL0_BASE              (APB1_BASE + 0xC000)
#define FPLL1_BASE              (APB1_BASE + 0xD000)

//--------------APB2 bus peris map---------------------//
#define HCC_BASE                (APB2_BASE + 0x0000)
#define EFLASH_BASE             (APB2_BASE + 0x1000)
#define PWRACE_BASE             (APB2_BASE + 0x5000)
#define ADC_BASE                (APB2_BASE + 0x6000)
#define FADC_BASE               (APB2_BASE + 0x7000)

//--------------PWRACE Subsystem peris map-------------//
#define SINCOS0_BASE            (PWRACE_BASE + 0x000)
#define SINCOS1_BASE            (PWRACE_BASE + 0x040)
#define RMS0_BASE               (PWRACE_BASE + 0x080)
#define RMS1_BASE               (PWRACE_BASE + 0x0C0)
#define RMS2_BASE               (PWRACE_BASE + 0x100)
#define MATRIX_BASE             (PWRACE_BASE + 0x140)
#define SPWM_BASE               (PWRACE_BASE + 0x280)
#define FFT0_BASE               (PWRACE_BASE + 0x2C0)
#define FFT1_BASE               (PWRACE_BASE + 0x300)
#define FFT2_BASE               (PWRACE_BASE + 0x340)
#define DFTRAN0_BASE            (PWRACE_BASE + 0x380)
#define DFTRAN1_BASE            (PWRACE_BASE + 0x3C0)
#define DFTRAN2_BASE            (PWRACE_BASE + 0x400)
#define ARCTAN0_BASE            (PWRACE_BASE + 0x440)
#define ARCTAN1_BASE            (PWRACE_BASE + 0x480)
#define ARCTAN2_BASE            (PWRACE_BASE + 0x4C0)
#define DATADMA_BASE            (PWRACE_BASE + 0x500)
#define EVSYS_BASE              (APB2_BASE + 0x2000)
#define SVPWM_BASE              (APB2_BASE + 0x3000)
#define EPWM_BASE               (APB2_BASE + 0x8000)

//--------------ADC Subsystem peris map----------------//
#define SARADC_BASE             (ADC_BASE + 0x0000)
#define IIR0_BASE               (ADC_BASE + 0x0300 + 0x0000)
#define IIR1_BASE               (ADC_BASE + 0x0300 + 0x0060)
#define IIR2_BASE               (ADC_BASE + 0x0300 + 0x00c0)
#define FIR0_BASE               (ADC_BASE + 0x0300 + 0x0120)
#define FIR1_BASE               (ADC_BASE + 0x0300 + 0x0150)
#define FIR2_BASE               (ADC_BASE + 0x0300 + 0x0180)

//--------------AHB0 bus peris map---------------------//
#define AHB_BASE                (PERIPH_BASE + 0x20000)
#define DMAC_BASE               (AHB_BASE + 0x0000)
#define GMAC_BASE               (AHB_BASE + 0x1000)
#define GPIOA_BASE              (AHB_BASE + 0x2000)
#define GPIOB_BASE              (AHB_BASE + 0x4000)
#define GPIOC_BASE              (AHB_BASE + 0x5000)
#define GPIOD_BASE              (AHB_BASE + 0x7000)
#define SYSCTRL_BASE            (AHB_BASE + 0x6000)
#define CAN_BASE                (AHB_BASE + 0x9000)

/**
  * @}
  */

/** @addtogroup Peripheral_declaration
  * @{
  */
#define IIC0                    ((IIC_TypeDef     *) IIC0_BASE)
#define IIC1                    ((IIC_TypeDef     *) IIC1_BASE)
#define USART0                  ((USART_TypeDef   *) USART0_BASE)
#define USART1                  ((USART_TypeDef   *) USART1_BASE)
#define USART2                  ((USART_TypeDef   *) USART2_BASE)
#define SPI0                    ((SPI_TypeDef     *) SPI0_BASE)
#define SPI1                    ((SPI_TypeDef     *) SPI1_BASE)
#define LVD                     ((LVD_TypeDef     *) LVD_BASE)
#define TIMER4                  ((TIMER_TypeDef   *) TIMER4_BASE)
#define TIMER5                  ((TIMER_TypeDef   *) TIMER5_BASE)
#define TIMER6                  ((TIMER_TypeDef   *) TIMER6_BASE)
#define TIMER7                  ((TIMER_TypeDef   *) TIMER7_BASE)
#define HCC                     ((HCC_TypeDef     *) HCC_BASE)

#define CAN                     ((CAN_TypeDef     *) CAN_BASE)
#define QEI                     ((QEI_TypeDef     *) QEI_BASE)
#define WDT                     ((WDT_TypeDef     *) WDT_BASE)
#define WDT1                    ((WDT1_TypeDef    *) WDT1_BASE)
#define TIMER0                  ((TIMER_TypeDef   *) TIMER0_BASE)
#define TIMER1                  ((TIMER_TypeDef   *) TIMER1_BASE)
#define TIMER2                  ((TIMER_TypeDef   *) TIMER2_BASE)
#define TIMER3                  ((TIMER_TypeDef   *) TIMER3_BASE)
#define EFLASH                  ((EFLASH_TypeDef  *) EFLASH_BASE)
#define CRC                     ((CRC_TypeDef     *) CRC_BASE)
#define EVSYS                   ((EVSYS_TypeDef   *) EVSYS_BASE)
#define SVPWM                   ((SVPWM_TypeDef   *) SVPWM_BASE)
#define FPLL0                   ((FPLL0_TypeDef   *) FPLL0_BASE)
#define FPLL1                   ((FPLL1_TypeDef   *) FPLL1_BASE)

#define SINCOS0                 ((SINCOS_TypeDef  *) SINCOS0_BASE)
#define SINCOS1                 ((SINCOS_TypeDef  *) SINCOS1_BASE)
#define RMS0                    ((RMS_TypeDef     *) RMS0_BASE)
#define RMS1                    ((RMS_TypeDef     *) RMS1_BASE)
#define RMS2                    ((RMS_TypeDef     *) RMS2_BASE)
#define MATRIX                  ((MATRIX_TypeDef  *) MATRIX_BASE)
#define SPWM                    ((SPWM_TypeDef    *) SPWM_BASE)

#define FFT0                    ((FFT_TypeDef     *) FFT0_BASE)
#define FFT1                    ((FFT_TypeDef     *) FFT1_BASE)
#define FFT2                    ((FFT_TypeDef     *) FFT2_BASE)
#define FFT                     ((FFT_TypeDef     *) FFT0_BASE)

#define DFTRAN0                 ((DFT_TypeDef     *) DFTRAN0_BASE)
#define DFTRAN1                 ((DFT_TypeDef     *) DFTRAN1_BASE)
#define DFTRAN2                 ((DFT_TypeDef     *) DFTRAN2_BASE)
#define DFTRAN                  ((DFT_TypeDef     *) DFTRAN0_BASE)

#define ARCTAN0                 ((ARCTAN_TypeDef  *) ARCTAN0_BASE)
#define ARCTAN1                 ((ARCTAN_TypeDef  *) ARCTAN1_BASE)
#define ARCTAN2                 ((ARCTAN_TypeDef  *) ARCTAN2_BASE)
#define ARCTAN                  ((ARCTAN_TypeDef  *) ARCTAN0_BASE)

#define DATADMA                 ((DATADMA_TypeDef *) DATADMA_BASE)

#define ADC                     ((ADC_TypeDef     *) SARADC_BASE)
#define IIR0                    ((IIR_TypeDef     *) IIR0_BASE)
#define IIR1                    ((IIR_TypeDef     *) IIR1_BASE)
#define IIR2                    ((IIR_TypeDef     *) IIR2_BASE)
#define IIRX                    ((IIR_TypeDef     *) IIR0_BASE)
#define FIR0                    ((FIR_TypeDef     *) FIR0_BASE)
#define FIR1                    ((FIR_TypeDef     *) FIR1_BASE)
#define FIR2                    ((FIR_TypeDef     *) FIR2_BASE)
#define FIR                     ((FIR_TypeDef     *) FIR0_BASE)

#define DMAC                    ((DMAC_TypeDef    *) DMAC_BASE)
#define GMAC                    ((GMAC_TypeDef    *) GMAC_BASE)
#define GPIOA                   ((GPIO_TypeDef    *) GPIOA_BASE)
#define GPIOB                   ((GPIO_TypeDef    *) GPIOB_BASE)
#define GPIOC                   ((GPIO_TypeDef    *) GPIOC_BASE)
#define GPIOD                   ((GPIO_TypeDef    *) GPIOD_BASE)
#define SYSCTRL                 ((SYSCTRL_TypeDef *) SYSCTRL_BASE)

#define ADVTMR0                 ((ADVTMR_TypeDef  *) ADVTMR0_BASE)
#define EPWM                    ((EPWM_TypeDef    *) EPWM_BASE)
#define FADC                    ((FADC_TypeDef    *) FADC_BASE)

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/*! special key define : A(PB0-3), B(PB4-7), C(PB8-11) */
#define PWM_KEY_A               (0x51UL << 0)
#define PWM_KEY_B               (0xA1UL << 8)
#define PWM_KEY_C               (0x55UL << 16)
#define SVPWM_KEY_A             (0x54UL << 0)
#define SVPWM_KEY_B             (0xA4UL << 8)
#define SVPWM_KEY_C             (0x58UL << 16)
#define ADTTIM_KEY_A            (0x52UL << 0)
#define ADTTIM_KEY_B            (0xA2UL << 8)
#define ADTTIM_KEY_C            (0x56UL << 16)
#define GPIO_KEY_A              (0x53UL << 0)
#define GPIO_KEY_B              (0xA3UL << 8)
#define GPIO_KEY_C              (0x57UL << 16)


#ifdef __cplusplus
}
#endif

#endif /* __TXF6200_H */

/**
  * @}
  */

  /**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
