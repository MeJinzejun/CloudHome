/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_usart.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the USART LL firmware functions.
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
#ifndef __TX_PHE_LL_USART_H
#define __TX_PHE_LL_USART_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup usart_interface_gr USART Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 
     
/** @addtogroup USART_LL_Driver USART LL Driver
  * @ingroup  usart_interface_gr
  * @brief Mainly the driver part of the USART module, which includes \b USART \b Register 
  * \b Constants, \b USART \b Exported \b Constants, \b USART \b Exported \b Struct, \b USART
  * \b Data \b transfers \b functions, \b USART \b Initialization \b and \b USART \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup USART_LL_Register_Constants USART LL Register Constants
  * @ingroup  USART_LL_Driver
  * @brief    USART LL register constant table definition
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the USART 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the USART register, mainly for convenience. Understand the 
    configuration of the USART.
    
@endverbatim
  *
  * @{
  */

/*! RX/TX FIFO depth 16 byte
 */
#define FIFO_DEPTH_MASK                          (0xF)

/***** RBR(Receive Buffer Register) *****/
/*! Receive Buffer for 9 bit
 */
#define LL_USARTX_RBR_RX_9BIT(n)                 (((n)&0x1FF) << 0)
/*! Receive Buffer for 8 bit
 */
#define LL_USARTX_RBR_RX_8BIT(n)                 (((n)&0xFF) << 0)


/***** THR(Transmit Holding Register) *****/
/*! Transmit Buffer for 9 bit
 */
#define LL_USARTX_THR_TX_9BIT(n)                 (((n)&0x1FF) << 0)
/*! Transmit Buffer for 8 bit
 */
#define LL_USARTX_THR_TX_8BIT(n)                 (((n)&0xFF) << 0)


/***** DLH(Divisor Latch High) *****/
/*! Upper 8-bits of a 16-bit, Divisor Latch register that contains the baud 
 *  rate divisor for the USART
 */
#define LL_USARTX_DLH(n)                         (((n)&0xFF) << 0)


/***** DLL(Divisor Latch Low) *****/
/*! Lower 8-bits of a 16-bit, Divisor Latch register that contains the baud 
 *  rate divisor for the USART     
 */
#define LL_USARTX_DLL(n)                         (((n)&0xFF) << 0)


/***** IER(Interrupt Enable Register) *****/
/*! Programmable THRE Interrupt Mode Enable
 */
#define LL_USARTX_IER_PTIME                      (1UL << 7)
/*! Enable Modem Status Interrupt
 */
#define LL_USARTX_IER_EDSSI                      (1UL << 3)
/*! Enable Receiver Line Status Interrupt
 */
#define LL_USARTX_IER_ELSI                       (1UL << 2)
/*! Enable Transmit Holding Register Empty Interrupt
 */
#define LL_USARTX_IER_ETBEI                      (1UL << 1)
/*! Enable Received Data Available Interrupt
 */
#define LL_USARTX_IER_ERBFI                      (1UL << 0)
/*! Enable interrupt mask
 */
#define LL_USARTX_IER_MASK(n)                    (((n)&0xFF) << 0)


/***** IIR(Interrupt Identity Register) *****/
/*! FIFOs Enabled(RO)
 */
#define LL_USARTX_IIR_FIFOSE(n)                  (((n)&0x3) << 6)
/*! Interrupt ID(RO)
 */
#define LL_USARTX_IIR_IID(n)                     (((n)&0xF) << 0)


/***** FCR(FIFO Control Register) *****/
/*! Used to select the trigger level in the receiver FIFO at which the 
 *  Received Data Available Interrupt is generated.
 */
#define LL_USARTX_FCR_RCVR(n)                    (((n)&0x3) << 6)
/*! Used to select the trigger level in the Transmit FIFO at which the 
 *  Received Data Available Interrupt is generated
 */
#define LL_USARTX_FCR_TET(n)                     (((n)&0x3) << 4)
/*! DMA Mode
 */
#define LL_USARTX_FCR_DMAM                       (1UL << 3)
/*! This resets the control portion of the transmit FIFO and treats 
 *  the FIFO as empty
 */
#define LL_USARTX_FCR_XFIFOR                     (1UL << 2)
/*! This resets the control portion of the receive FIFO and treats 
 *  the FIFO as empty
 */
#define LL_USARTX_FCR_RFIFOR                     (1UL << 1)
/*! This enables/disables the transmit (XMIT) and receive (RCVR)FIFOs
 */
#define LL_USARTX_FCR_FIFOE                      (1UL << 0)


/***** LCR(Line Control Register) *****/
/*! Divisor Latch Access Bit
 */
#define LL_USARTX_LCR_DLAB                       (1UL << 7)
/*! This is used to cause a break condition to be transmitted to the 
 *  receiving device
 */
#define LL_USARTX_LCR_BC                         (1UL << 6)
/*! Stick Parity
 */
#define LL_USARTX_LCR_SP                         (1UL << 5)
/*! Even Parity Select
 */
#define LL_USARTX_LCR_EPS                        (1UL << 4)
/*! Parity Enable
 */
#define LL_USARTX_LCR_PEN                        (1UL << 3)

/*! Parity Enable And Parity Select
 */
#define LL_USARTX_LCR_PARITY(n)                  (((n)&0x3) << 3)
/*! Number of stop bits
 */
#define LL_USARTX_LCR_STOP                       (1UL << 2)
/*! Data Length Select
 */
#define LL_USARTX_LCR_DLS(n)                     (((n)&0x3) << 0)


/***** MCR(Modem Control Register) *****/
/*! SIR Mode Enable
 */
#define LL_USARTX_MCR_SIRE                       (1UL << 6)
/*! Auto Flow Control Enable
 */
#define LL_USARTX_MCR_AFCE                       (1UL << 5)
/*! Used to put the USART into a diagnostic mode for test purposes
 */
#define LL_USARTX_MCR_LB                         (1UL << 4)
/*! Used to directly control the user-designated Output2 (out2_n) output
 */
#define LL_USARTX_MCR_OUT2                       (1UL << 3)
/*! Used to directly control the user-designated Output1 (out1_n) output
 */
#define LL_USARTX_MCR_OUT1                       (1UL << 2)
/*! Used to directly control the Request to Send (rts_n) output
 */
#define LL_USARTX_MCR_RTS                        (1UL << 1)
/*! Used to directly control the Data Terminal Ready (dtr_n) output
 */
#define LL_USARTX_MCR_DTR                        (1UL << 0)


/***** LSR(Line Status Register) *****/
/*! Address Received bit
 */
#define LL_USARTX_LSR_ADDR_RCVD                  (1UL << 8)
/*! Receiver FIFO Error bit
 */
#define LL_USARTX_LSR_RFE                        (1UL << 7)
/*! Transmitter Empty bit
 */
#define LL_USARTX_LSR_TEMT                       (1UL << 6)
/*! Transmit Holding Register Empty bit
 */
#define LL_USARTX_LSR_THRE                       (1UL << 5)
/*! Break Interrupt bit
 */
#define LL_USARTX_LSR_BI                         (1UL << 4)
/*! Framing Error bit
 */
#define LL_USARTX_LSR_FE                         (1UL << 3)
/*! Parity Error bit
 */
#define LL_USARTX_LSR_PE                         (1UL << 2)
/*! Overrun error bit
 */
#define LL_USARTX_LSR_OE                         (1UL << 1)
/*! Data Ready bit
 */
#define LL_USARTX_LSR_DR                         (1UL << 0)


/***** MSR(Modem Status Register) *****/
/*! Used to indicate the current state of the modem control line dcd_n
 */
#define LL_USARTX_MSR_DCD                        (1UL << 7)
/*! Used to indicate the current state of the modem control line ri_n
 */
#define LL_USARTX_MSR_RI                         (1UL << 6)
/*! Used to indicate the current state of the modem control line dsr_n
 */
#define LL_USARTX_MSR_DSR                        (1UL << 5)
/*! Used to indicate the current state of the modem control line cts_n
 */
#define LL_USARTX_MSR_CTS                        (1UL << 4)
/*! Used to indicate that the modem control line dcd_n haschanged since 
 *  the last time the MSR was read
 */
#define LL_USARTX_MSR_DDCD                       (1UL << 3)
/*! Used to indicate that a change on the input ri_n has occurred since 
 *  the last time the MSR was read
 */
#define LL_USARTX_MSR_TERI                       (1UL << 2)
/*! sed to indicate that the modem control line dsr_n has changed since 
 *  the last time the MSR was read
 */
#define LL_USARTX_MSR_DDSR                       (1UL << 1)
/*! Used to indicate that the modem control line cts_n has changed since 
 *  the last time the MSR was read
 */
#define LL_USARTX_MSR_DCTS                       (1UL << 0)


/***** SCR(Scratchpad Register) *****/
/*! This register is for programmers to use as a temporary storage space
 */
#define LL_USARTX_SCR(n)                         (((n)&0xFF) << 0)


/***** LPDLL(Low Power Divisor Latch Low Register) *****/
/*! lower 8-bits of a 16-bit,It required for SIR Low Power (minimum pulse 
 *  width) detection at the receiver 
 */
#define LL_USARTX_LPDLL(n)                       (((n)&0xFF) << 0)


/***** LPDLL(Low Power Divisor Latch High Register) *****/
/*! upper 8-bits of a 16-bit,It required for SIR Low Power (minimum pulse 
 *  width) detection at the receiver 
 */
#define LL_USARTX_LPDLH(n)                       (((n)&0xFF) << 0)


/***** SRBR(Shadow Receive Buffer Register) *****/
/*! RX Buffer shadow register for the 9 BIT
 */
#define LL_USARTX_SRBR_RX_9BIT(n)                (((n)&0x1FF) << 0)
/*! RX Buffer shadow register for the 8 BIT
 */
#define LL_USARTX_SRBR_RX_8BIT(n)                (((n)&0xFF) << 0)


/***** STHR(Shadow Transmit Holding Register) *****/
/*! TX Buffer shadow register for the 9 BIT
 */
#define LL_USARTX_STHR_TX_9BIT(n)                (((n)&0x1FF) << 0)
/*! TX Buffer shadow register for the 8 BIT
 */
#define LL_USARTX_STHR_TX_8BIT(n)                (((n)&0xFF) << 0)


/***** FAR(FIFO Access Register) *****/
/*! This register is use to enable a FIFO access mode for testing
 */
#define LL_USARTX_FAR                            (1UL << 0)


/***** TFR(Transmit FIFO Read Register) *****/
/*! Transmit FIFO Read
 */
#define LL_USARTX_TFR                            (1UL << 0)


/***** RFW(Receive FIFO Write) *****/
/*! Receive FIFO Framing Error
 */
#define LL_USARTX_RFW_RFFE                       (1UL << 9)
/*! Receive FIFO Parity Error
 */
#define LL_USARTX_RFW_RFPE                       (1UL << 8)
/*! Receive FIFO Write Data
 */
#define LL_USARTX_RFW_RFWD(n)                    (((n)&0xFF) << 0)


/***** USR(USART Status Register) *****/
/*! Receive FIFO Full
 */
#define LL_USARTX_USR_RFF                        (1UL << 4)
/*! Receive FIFO Not Empty
 */
#define LL_USARTX_USR_RFNE                       (1UL << 3)
/*! Transmit FIFO Empty
 */
#define LL_USARTX_USR_TFE                        (1UL << 2)
/*! Transmit FIFO Not Full
 */
#define LL_USARTX_USR_TFNF                       (1UL << 1)
/*! USART Busy
 */
#define LL_USARTX_USR_BUSY                       (1UL << 0)


/***** TFL(Transmit FIFO Level) *****/
/*! Transmit FIFO Level
 */
#define LL_USARTX_TFL(n)                         (((n)&FIFO_DEPTH_MASK) << 0)

/***** RFL(Receive FIFO Level) *****/
/*! Receive FIFO Level
 */
#define LL_USARTX_RFL(n)                         (((n)&FIFO_DEPTH_MASK) << 0)


/***** SRR(Software Reset Register) *****/
/*! This is a shadow register for the XMIT FIFO Reset bit (FCR[2])
 */
#define LL_USARTX_SRR_XFR                        (1UL << 2)
/*! This is a shadow register for the RCVR FIFO Reset bit (FCR[1])
 */
#define LL_USARTX_SRR_RFR                        (1UL << 1)
/*! This asynchronously resets the DW_apb_usart and synchronously 
 *  removes the reset assertion
 */
#define LL_USARTX_SRR_UR                         (1UL << 0)


/***** SRTS(Shadow Request to Send) *****/
/*! Shadow Request to Send
 */
#define LL_USARTX_SRTS                           (1UL << 0)


/***** SBCR(Shadow Break Control Register) *****/
/*! Shadow Break Control Bit
 */
#define LL_USARTX_SBCR                           (1UL << 0)


/***** SDMAM(Shadow DMA Mode) *****/
/*! This is a shadow register for the DMA mode bit (FCR[3])
 */
#define LL_USARTX_SDMAM                          (1UL << 0)


/***** SFE(Shadow FIFO Enable) *****/
/*! This is a shadow register for the FIFO enable bit (FCR[0])
 */
#define LL_USARTX_SFE                            (1UL << 0)


/***** SRT(Shadow RCVR Trigger) *****/
/*! This is a shadow register for the RCVR trigger bits (FCR[7:6])
 */
#define LL_USARTX_SRT(n)                         (((n)&0x3) << 0)


/***** STET(Shadow TX Empty Trigger) *****/
/*! This is a shadow register for the TX empty trigger bits (FCR[5:4])
 */
#define LL_USARTX_STET(n)                        (((n)&0x3) << 0)


/***** HTX(Halt TX) *****/
/*! This register is use to halt transmissions for testing
 */
#define LL_USARTX_HTX                            (1UL << 0)


/***** DMASA(DMA Software Acknowledge) *****/
/*! This register is use to perform a DMA software acknowledge if a 
 *  transfer needs to be terminated due to an error condition
 */
#define LL_USARTX_DMASA                          (1UL << 0)


/***** TCR(Transceiver Control Register) *****/
/*! Transfer Mode
 */
#define LL_USARTX_TCR_XFER_MODE(n)               (((n)&0x3) << 3)
/*! Driver Enable Polarity
 */
#define LL_USARTX_TCR_DE_POL(n)                  (((n)&0x1) << 2)
/*! Receiver Enable Polarity
 */
#define LL_USARTX_TCR_RE_POL(n)                  (((n)&0x1) << 1)
/*! RS485 Transfer Enable
 */
#define LL_USARTX_TCR_RS485_EN                   (1UL << 0)


/***** DE_EN(Driver Output Enable Register) *****/
/*! Used to control assertion and de-assertion of 'de' signal.
 */
#define LL_USARTX_DE_EN                          (1UL << 0)


/***** RE_EN(Receiver Output Enable Register) *****/
/*! Used to control assertion and de-assertion of 're' signal.
 */
#define LL_USARTX_RE_EN                          (1UL << 0)


/***** DET(Driver Output Enable Timing Register) *****/
/*! Driver enable de-assertion time
 */
#define LL_USARTX_DET_DEDAT(n)                   (((n)&0xFF) << 16)
/*! Driver enable assertion time
 */
#define LL_USARTX_DET_DEAT(n)                    (((n)&0xFF) << 0)


/***** TAT(TurnAround Timing Register) *****/
/*! Receiver Enable to Driver Enable TurnAround time
 */
#define LL_USARTX_TAT_RE_TO_DE(n)                (((n)&0xFFFF) << 16)
/*! Driver Enable to Receiver Enable TurnAround time
 */
#define LL_USARTX_TAT_DE_TO_RE(n)                (((n)&0xFFFF) << 0)


/***** DLF(Divisor Latch Fraction Register) *****/
/*! Fractional part of divisor, The fractional value is added to integer 
 *  value set by DLH, DLL. Fractional value is determined by 
 *  (Divisor Fraction value)/(2^DLF_SIZE)
 */
#define LL_USARTX_DLF_DE_TO_RE(n)                (((n)&0xF) << 0)


/***** RAR(Receive Address Register) *****/
/*! This is an address matching register during receive mode
 */
#define LL_USARTX_RAR(n)                         (((n)&0xFF) << 0)


/***** TAR(Transmit Address Register) *****/
/*! This is an address matching register during transmit mode
 */
#define LL_USARTX_TAR(n)                         (((n)&0xFF) << 0)


/***** LCR_EXT(USART Status Register) *****/
/*! Transmit mode control bit
 */
#define LL_USARTX_LCR_TRANSMIT_MODE              (1UL << 3)
/*! Send address control bit
 */
#define LL_USARTX_LCR_SEND_ADDR                  (1UL << 2)
/*! Address Match Mode
 */
#define LL_USARTX_LCR_ADDR_MATCH                 (1UL << 1)
/*! Extension for DLS
 */
#define LL_USARTX_LCR_DLS_E                      (1UL << 0)


/***** CPR(USART Status Register) *****/
/*! Parameter FIFO_MODE
 */
#define LL_USARTX_CPR_FIFO_MODE                  (((n)&0xFF) << 16)
/*! Parameter DMA_EXTRA
 */
#define LL_USARTX_CPR_DMA_EXTRA                  (1UL << 13)
/*! Parameter ADD_ENCODED
 */
#define LL_USARTX_CPR_ADD_ENCODED                (1UL << 12)
/*! Parameter SHADOW
 */
#define LL_USARTX_CPR_SHADOW                     (1UL << 11)
/*! Parameter FIFO_STAT
 */
#define LL_USARTX_CPR_FIFO_STAT                  (1UL << 10)
/*! Parameter FIFO_ACCESS
 */
#define LL_USARTX_CPR_FIFO_ACCESS                (1UL << 9)
/*! Parameter NEW_FEAT
 */
#define LL_USARTX_CPR_NEW_FEAT                   (1UL << 8)
/*! Parameter SIR_LP_MODE
 */
#define LL_USARTX_CPR_SIR_LP_MODE                (1UL << 7)
/*! Parameter SIR_MODE
 */
#define LL_USARTX_CPR_SIR_MODE                   (1UL << 6)
/*! Parameter THRE_MODE
 */
#define LL_USARTX_CPR_THRE_MODE                  (1UL << 5)
/*! Parameter AFCE_MODE
 */
#define LL_USARTX_CPR_AFCE_MODE                  (1UL << 4)
/*! Parameter APB_DATA_WIDTH
 */
#define LL_USARTX_CPR_APB_DATA_WIDTH             (((n)&0x3) << 0)


/***** UCV(USART Component Version) *****/
/*! ASCII value for each number in the version
 */
#define LL_USARTX_UCV(n)                         (((n)&0xFFFFFFFF) << 0)


/***** CTR(Component Type Register) *****/
/*! This register contains the peripherals identification code
 */
#define LL_USARTX_CTR(n)                         (((n)&0xFFFFFFFF) << 0)


/**
  * @}
  */

/** @defgroup USART_LL_Exported_Constants USART LL Exported Constants
  * @ingroup  USART_LL_Driver
  * @brief    USART LL external constant definition
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

/**
  * @brief Enumeration constant for USART interrupt flag
  */
typedef enum {
    /*! Interrupt indicator modem status CTR, DSR, RI or DCD status changed
     */
    LL_USART_INTR_MODEM_STATUS   = 0x0,
    /*! Interrupt indicator No event/interrupt pending.
     */
    LL_USART_INTR_NO             = 0x1,
    /*! Interrupt indicator Transmit holding register empty or 
     *  TX FIFO at or below trigger threshold.
     */
    LL_USART_INTR_THR_EMPTY      = 0x2,
    /*! Interrupt indicator received data available Receive buffer register data
     *  available (non-FIFO mode) or RX FIFO trigger level reached.
     */
    LL_USART_INTR_RX_DATA_AVAIL  = 0x4,
    /*! Interrupt indicator Overrun/parity/framing error or break interrupt occurred.
     */
    LL_USART_INTR_RX_LINE_STATUS = 0x6,
    /*! Interrupt indicator Attempt to write to the LCR[7] while DW_apb_uart was busy (DLAB).
     */
    LL_USART_INTR_BUSY_DETECT    = 0x7,
    /*! Interrupt indicator No characters in or out of the receiver FIFO during the last four
     *  character times and there is at least one character in it during this time.
     */
    LL_USART_INTR_CHAR_TIMEOUT   = 0xC
} TYPE_ENUM_LL_USART_INTR_STATUS;

  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for USART data length
  */
typedef enum {
    /*! Usart word length select 5 bit
     */
    LL_USART_WORD_LENGTH_5B = 0,
    /*! Usart word length select 6 bit
     */
    LL_USART_WORD_LENGTH_6B,
    /*! Usart word length select 7 bit
     */
    LL_USART_WORD_LENGTH_7B,
    /*! Usart word length select 8 bit
     */
    LL_USART_WORD_LENGTH_8B,
} TYPE_ENUM_LL_USART_BIT_WIDTH_SEL;

/**
  * @brief Enumeration constant for USART stop bit select
  */
typedef enum {
    /*! Usart stop bit select 1 bit
     */
    LL_USART_STOP_1B = 0,
    /*! Usart stop bit select 2 bit
     */
    LL_USART_STOP_2B,
} TYPE_ENUM_LL_USART_STOP_BIT_SEL;

/**
  * @brief Enumeration constant for USART parity select
  */
typedef enum {
    /*! Usart no parity check
     */
    LL_USART_PARITY_NO = 0,
    /*! Usart odd parity
     */
    LL_USART_PARITY_ODD,
    /*! Reserver
     */
    LL_USART_RESVERS,
    /*! Usart enven parity
     */    
    LL_USART_PARITY_EVEN,
} TYPE_ENUM_LL_USART_PARITY_SEL;

/**
  * @brief Enumeration constant for USART stick parity feature
  */
typedef enum {
    /*! Usart stick parity disable
     */
    LL_USART_STICK_PARITY_DIS = 0,
    /*! Usart stick parity enable
     */
    LL_USART_STICK_PARITY_EN,
} TYPE_ENUM_LL_USART_STICK_PARITY;

/**
  * @brief Enumeration constant for USART TX FIFO trigger select
  */
typedef enum {
    /*! TX FIFO empty 
     */
    LL_USART_TXFIFO_TRIG_EMPTY = 0,
    /*! 2 characters in TX FIFO  
     */
    LL_USART_TXFIFO_TRIG_2CHAR,
    /*! 1/4 TX FIFO FULL trigger
     */
    LL_USART_TXFIFO_TRIG_QUSARTER,
    /*! 1/2 TX FIFO FULL trigger
     */
    LL_USART_TXFIFO_TRIG_HALF,
} TYPE_ENUM_LL_USART_TXFIFO_TRIG;

/**
  * @brief Enumeration constant for USART RX FIFO trigger select
  */
typedef enum {
    /*! 1 characters in RX FIFO  
     */
    LL_USART_RXFIFO_TRIG_1CHAR = 0,
    /*! 1/4 RX FIFO FULL 
     */
    LL_USART_RXFIFO_TRIG_QUSARTER,
    /*! 1/2 RX FIFO FULL 
     */
    LL_USART_RXFIFO_TRIG_HALF,
    /*! RX FIFO is 2 characters less than FULL 
     */
    LL_USART_RXFIFO_TRIG_2LESS_FULL,
} TYPE_ENUM_LL_USART_RXFIFO_TRIG;

/**
  * @brief Enumeration constant for USART interrupt source select
  */
typedef enum {
    /*! Interrupt trigger select rx ready
     */
    LL_USART_INTR_RX_READY      = 0x01,
    /*! Interrupt trigger select tx empty
     */
    LL_USART_INTR_TX_EMPTY      = 0x02,
    /*! Interrupt trigger select rx line status
     */
    LL_USART_INTR_RX_LINE_STAUS = 0x04,
    /*! Interrupt trigger select modem status
     */
    LL_USART_INTR_MODEM_STAUS   = 0x08,
    /*! Interrupt trigger select programmable THRE
     */
    LL_USART_INTR_PROG_THRE     = 0x80,
} TYPE_ENUM_LL_USART_INTR_SEL;

/**
  * @brief Enumeration constant for USART IO_MAP select
  * @note The correspondence table of IO_MAP is as follows:
  *  ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  *  ©¦           ©¦                 USART 232                 ©¦       USART 485      ©¦
  *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *  ©¦           ©¦  LL_USART_IO_MAP0   ©¦  LL_USART_IO_MAP1   ©¦   LL_USART_IO_MAP0   ©¦
  *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *  ©¦  io name  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx/  ©¦
  *  ©¦           ©¦                     ©¦                     ©¦  usart_re/usart_de   ©¦
  *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *  ©¦  USART0   ©¦      PA10/PA11      ©¦                     ©¦  PA10/PA11/PA9/PA8   ©¦
  *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *  ©¦  USART1   ©¦      PA12/PA13      ©¦                     ©¦  PA12/PA13/PA14/PA15 ©¦
  *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *  ©¦  USART2   ©¦       PD2/PD3       ©¦      PA14/PA15      ©¦   PD2/PD3/PD1/PD0    ©¦
  *  ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  */
typedef enum {
    /*! Select the pin of the USART io map0
     */
    LL_USART_IO_MAP0 = 0x01,
    /*! Select the pin of the USART io map1
     */
    LL_USART_IO_MAP1 = 0x02,
} TYPE_ENUM_LL_USART_IOMAP_SEL;

/**
  * @brief Enumeration constant for USART 485 mode select
  */
typedef enum {
    /*! MODE0:  
     * In this mode, transmit and receive can happen simultaneously. You can
     * enable DE_EN and RE_EN at any point of time. Turn around timing as
     * programmed in the TAT register is not applicable in this mode.
     */
    LL_USART485_MODE0 = 0x0,
    /*! MODE1:  
     * In this mode, DE and RE are mutually exclusive. The hardware considers
     * the turnaround timings that are programmed in the TAT register while
     * switching from RE to DE or from DE to RE. Ensure that either DE or RE is
     * expected to be enabled while programming.
     * For transmission, hardware waits if it is in the midst of receiving any
     * transfer, before it starts transmitting.
     */
    LL_USART485_MODE1 = 0x1,
    /*! MODE2:  
     * In this mode, DE and RE are mutually exclusive. Once DE_EN or RE_EN
     * is programed, 're' is enabled by default and DW_apb_uart controller will be
     * ready to receive. If the user programs the TX FIFO with data, then
     * DW_apb_uart, after ensuring no receive is in progress, disables the 're'
     * and enables the 'de' signal.
     * Once the TX FIFO becomes empty, the 're' signal gets enabled and the
     * 'de' signal will be disabled.
     * In this mode of operation, the hardware considers the turnaround timings
     * that are programmed in the TAT register while switching from RE to DE or
     * from DE to RE. In this mode, 'de' and 're' signals are strictly
     * complementary to each other.
     */
    LL_USART485_MODE2 = 0x2,
} TYPE_ENUM_LL_USART485_MODE;

/**
  * @}
  */

/** @defgroup USART_LL_Exported_Struct USART LL Exported Struct
  * @ingroup  USART_LL_Driver
  * @brief    USART LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the USART registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_usart_init. Function, you can configure the USART module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief USART low layer configuration structure
  */
typedef struct __ll_usart_irq_cfg {
    /*! Enable interrupt trigger select rx ready
     */
    bool rx_ready_intr_en;
    /*! Enable interrupt trigger select tx empty
     */
    bool tx_empty_intr_en;
    /*! Enable interrupt trigger select rx line status
     */
    bool rx_line_status_en;
    /*! Enable interrupt trigger select modem status
     */
    bool modem_status_intr_en;
    /*! Enable interrupt trigger select programmable THRE
     */
    bool program_thre_intr_en;
} TYPE_LL_USART_IRQ_CFG;

/**
  * @brief USART low layer configuration structure
  */
typedef struct __ll_usart_cfg {
    /***** usart *****/
    /*! USART stop bit selection
     */
    TYPE_ENUM_LL_USART_STOP_BIT_SEL  stop_bit_sel;
    /*! USART data length selection
     */
    TYPE_ENUM_LL_USART_BIT_WIDTH_SEL bit_width_sel;
    /*! USART parity selection
     */
    TYPE_ENUM_LL_USART_PARITY_SEL    parity;
    /*! USART stick parity feature
     *  @note Enable this feature will enable parity by force.
     */
    TYPE_ENUM_LL_USART_STICK_PARITY  stick_parity;

    /*! Trigger source selection for USART RX FIFO
     */
    TYPE_ENUM_LL_USART_RXFIFO_TRIG   rxfifo_trigger;
    /*! Trigger source selection for USART TX FIFO
     */
    TYPE_ENUM_LL_USART_TXFIFO_TRIG   txfifo_trigger;
    /*! USART baud rate
     */
    u16                              baudrate;
    /*! USART io_map selection : TYPE_ENUM_LL_USART_IOMAP_SEL
     *  @note The correspondence table of IO_MAP is as follows:
     *  ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
     *  ©¦           ©¦                 USART 232                 ©¦       USART 485      ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦           ©¦  LL_USART_IO_MAP0   ©¦  LL_USART_IO_MAP1   ©¦   LL_USART_IO_MAP0   ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦  io name  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx  ©¦  usart_rx/usart_tx/  ©¦
     *  ©¦           ©¦                     ©¦                     ©¦  usart_re/usart_de   ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦  USART0   ©¦      PA10/PA11      ©¦                     ©¦  PA10/PA11/PA9/PA8   ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦  USART1   ©¦      PA12/PA13      ©¦                     ©¦  PA12/PA13/PA14/PA15 ©¦
     *  ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
     *  ©¦  USART2   ©¦       PD2/PD3       ©¦      PA14/PA15      ©¦   PD2/PD3/PD1/PD0    ©¦
     *  ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
     */
    TYPE_ENUM_LL_USART_IOMAP_SEL     io_map;
} TYPE_LL_USART_CFG;

typedef struct __ll_rs485_cfg {
    /*! USART 485 MODE selection
     */
    TYPE_ENUM_LL_USART485_MODE rs485_mode;
    /*! USART RS485 Recieve signal output enable
     */
    bool                       re_sig_en;
    /*! USART RS485 Recieve signal active level
     */
    Polarity                   re_sig_active_level;
    /*! USART RS485 Transmit signal output enable
     */
    bool                       de_sig_en;
    /*! USART RS485 Transmit signal active level
     */
    Polarity                   de_sig_active_level;
    /*! USART RS485 Driver enable assertion time.  
     * This field controls the amount of time (in terms of number of serial clock
     * periods) between the assertion of rising edge of Driver output enable signal to
     * serial transmit enable. Any data in transmit buffer, will start on serial output
     * (sout) after the transmit enable.
     */
    u8                         de_assertion_time;
    /*! USART RS485 Driver enable de-assertion time.  
     * This field controls the amount of time (in terms of number of serial clock
     * periods) between the end of stop bit on the serial output (sout) to the falling
     * edge of Driver output enable signal.
     */
    u8                         de_deassertion_time;
    /*! USART RS485 Receiver Enable to Driver Enable TurnAround time
     * Turnaround time (in terms of serial clock) for RE de-assertion to DE assertion.
     * @note:
     *       If the DE assertion time in the DET register is 0, then the actual value is
     *          the programmed value + 3.  
     *       If the DE assertion time in the DET register is 1, then the actual value is
     *          the programmed value + 2.  
     *       If the DE assertion time in the DET register is greater than 1, then the
     *          actual value is the programmed value + 1.
     */
    u16                        re2de_turnaround_time;
    /*! USART RS485 Driver Enable to Receiver Enable TurnAround time
     * Turnaround time (in terms of serial clock) for DE de-assertion to RE assertion.
     * @note: The actual time is the programmed value + 1.
     */
    u16                        de2re_turnaround_time;
    /*! USART232 cfg
     */
    TYPE_LL_USART_CFG          usart_cfg;
} TYPE_LL_RS485_CFG;

/**
  * @brief USART low layer initialization structure
  */
typedef struct __ll_usart_init {
    u8 reserved;
} TYPE_LL_USART_INIT;

/**
  * @}
  */

/** @defgroup USART_LL_Interrupt USART LL Interrupt Handle function
  * @ingroup  USART_LL_Driver
  * @brief   USART LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the USART  
    Interrupt Handle function.

    how to use?

@endverbatim
  *
  * @{
  */


/**
  * @}
  */
  
/** @defgroup USART_LL_Inti_Cfg USART LL Initialization And Configuration
  * @ingroup  USART_LL_Driver
  * @brief    USART LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the USART data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/** 
  * @brief  Low layer USART initialization function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @param  p_init : USART initialization struct
  * @retval None.
  */
void ll_usart_init(USART_TypeDef *p_usart, TYPE_LL_USART_INIT *p_init);

/** 
  * @brief  Low layer USART detele initialization function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
void ll_usart_deinit(USART_TypeDef *p_usart);

/** 
  * @brief  Low layer USART interrupt initialization function
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart_irq_config(USART_TypeDef *p_usart, TYPE_LL_USART_IRQ_CFG *p_cfg);

/** 
  * @brief  low layer USART 232 initialization function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart232_config(USART_TypeDef *p_usart, TYPE_LL_USART_CFG *p_cfg);

/** 
  * @brief  Low layer USART 485 initialization function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @param  p_cfg  : USART Configuration struct
  * @retval None.
  */
void ll_usart485_config(USART_TypeDef *p_usart, TYPE_LL_RS485_CFG *p_cfg);


/**
  * @}
  */
  
/** @defgroup USART_LL_Data_Transfers USART LL Data transfers functions
  * @ingroup  USART_LL_Driver
  * @brief    USART LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the USART data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/** 
  * @brief  Get the status flag of the USART interrupt
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval status flag of the USART interrupt
  */
#define LL_USART_GET_INTR_STATUS(p_usart)                        ((p_usart)->IIR & LL_USARTX_IIR_IID(0xF)) 

/** 
  * @brief  Get the read ready state of the USART
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval the read ready state of the USART
  */
#define LL_USART_GET_LINE_DATA_READY(p_usart)                     ((p_usart)->LSR & LL_USARTX_LSR_DR)

/** 
  * @brief  Get the data in the USART
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  data   : the var to save result
  * @retval the data in the USART
  */
#define LL_USART_READ_DATA(p_usart, data)                         (data = (p_usart)->RBR)

/** 
  * @brief  Get the status of whether the TX FIFO in the USART is FULL
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval the status of the TX FIFO in the USART
  */
#define LL_USART_TXFIFO_NOT_FULL(p_usart)                         ((p_usart)->USR & LL_USARTX_USR_TFNF)

/** 
  * @brief  Send data to the USART
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  data   : the value to be write
  * @retval None.
  */
#define LL_USART_WRITE_DATA(p_usart, data)                        ((p_usart)->THR = data)

/** 
  * @brief  USART check rx data avail interrupt enable
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
#define LL_USART_CHECK_RX_DATA_AVAIL_INTERRUPT_ENABLE(p_usart)    ((p_usart)->IER & LL_USARTX_IER_ERBFI)

/** 
  * @brief  USART check rx line status interrupt enable
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
#define LL_USART_CHECK_RX_LINE_STATUS_INTERRUPT_ENABLE(p_usart)   ((p_usart)->IER & LL_USARTX_IER_ELSI)

/** 
  * @brief  USART check tx empty interrupt enable
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
#define LL_USART_CHECK_THR_EMPTY_INTERRUPT_ENABLE(p_usart)        ((p_usart)->IER & LL_USARTX_IER_ETBEI)

/** 
  * @brief  USART check modem status interrupt enable
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
#define LL_USART_CHECK_MODEM_STATUS_INTERRUPT_ENABLE(p_usart)     ((p_usart)->IER & LL_USARTX_IER_EDSSI)

/** 
  * @brief  USART check program thre interrupt enable
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
#define LL_USART_CHECK_PROGRAM_THRE_INTERRUPT_ENABLE(p_usart)     ((p_usart)->IER & LL_USARTX_IER_PTIME)

/** 
  * @brief  USART clear rx data avail pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_rx_data_avail_pending(USART_TypeDef *p_usart) {
    /* read USARTx->RBR to clear the interrupt of RX_DATA_AVAIL,  
     * But need to be implemented in the A function, you can call
     * LL_USART_READ_DATA() to clear.
     */
}

/** 
  * @brief  USART clear rx line status pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_rx_line_status_pending(USART_TypeDef *p_usart) {
    /* read USARTx->LSR to clear the interrupt of RX_LINE_STATUS */
    p_usart->LSR = p_usart->LSR;
}

/** 
  * @brief  USART clear thr empty pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_thr_empty_pending(USART_TypeDef *p_usart) {
    /* read USARTx->RBR or write USARTx->THR to clear the interrupt 
     * of THR_EMPTY, But need to be implemented in the A function, 
     * you can call LL_USART_READ_DATA() or LL_USART_WRITE_DATA to clear 
     */
}

/** 
  * @brief  USART clear busy detect pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_busy_detect_pending(USART_TypeDef *p_usart) {
    /* read USARTx->USR to clear the interrupt of BUSY_DETECT */
    p_usart->USR = p_usart->USR;
}

/** 
  * @brief  USART clear char timeout pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_char_timeout_pending(USART_TypeDef *p_usart) {
    /* read USARTx->RBR to clear the interrupt of CHAR_TIMEOUT,  
     * But need to be implemented in the A function, you can call
     * LL_USART_READ_DATA() to clear 
     */
}

/** 
  * @brief  USART clear modem status pending
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval result.
  */
__STATIC_INLINE void ll_usart_clear_modem_status_pending(USART_TypeDef *p_usart) {
    /* read USARTx->MSR to clear the interrupt of MODEM_STATUS */
    p_usart->MSR = p_usart->MSR;
}

/** 
  * @brief  Usart TX wait function.
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_wait_tx_pending(USART_TypeDef *p_usart) {
    /* wait tx fifo empty */
    while(0 == (p_usart->USR & LL_USARTX_USR_TFE));   
    while(0 == (p_usart->LSR & LL_USARTX_LSR_TEMT));
}

/** 
  * @brief  Usart put char function.
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @param  data   : data which will send to USART TX pin.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_putchar(USART_TypeDef *p_usart, u8 data) {
    while((p_usart->USR & LL_USARTX_USR_TFNF) == 0);
    p_usart->THR = data; 
}

/** 
  * @brief  Usart get char function.
  * @param  p_usart: The structure pointer of the USART group (USART0, USART1, USART2) is selected.
  * @retval The data received by the Usart
  */
__STATIC_INLINE u8 ll_usart_getchar(USART_TypeDef *p_usart) {
    while((p_usart->LSR & LL_USARTX_LSR_DR) == 0);
    return p_usart->RBR;
}

/** 
  * @brief  Low layer USART enable receive ready interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_rx_ready_interrupt_enable(USART_TypeDef *p_usart) {
    p_usart->IER |= LL_USARTX_IER_MASK(LL_USARTX_IER_ERBFI);
}

/** 
  * @brief  Low layer USART enable send empty interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_tx_empty_interrupt_enable(USART_TypeDef *p_usart) {
    p_usart->IER |= LL_USARTX_IER_MASK(LL_USARTX_IER_ETBEI);
}

/** 
  * @brief  Low layer USART enable receive line status interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_rx_line_status_interrupt_enable(USART_TypeDef *p_usart) {
    p_usart->IER |= LL_USARTX_IER_MASK(LL_USARTX_IER_ELSI);
}

/** 
  * @brief  Low layer USART enable modem status interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_modem_status_interrupt_enable(USART_TypeDef *p_usart) {
    p_usart->IER |= LL_USARTX_IER_MASK(LL_USARTX_IER_EDSSI);
}

/** 
  * @brief  Low layer USART enable program thre interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_program_thre_interrupt_enable(USART_TypeDef *p_usart) {
    p_usart->IER |= LL_USARTX_IER_MASK(LL_USARTX_IER_PTIME);
}

/** 
  * @brief  Low layer USART disable receive ready interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_rx_ready_interrupt_disable(USART_TypeDef *p_usart) {
    p_usart->IER &= ~(LL_USARTX_IER_MASK(LL_USARTX_IER_ERBFI));
}

/** 
  * @brief  Low layer USART disable send empty interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_tx_empty_interrupt_disable(USART_TypeDef *p_usart) {
    p_usart->IER &= ~(LL_USARTX_IER_MASK(LL_USARTX_IER_ETBEI));
}

/** 
  * @brief  Low layer USART disable receive line status interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_rx_line_status_interrupt_disable(USART_TypeDef *p_usart) {
    p_usart->IER &= ~(LL_USARTX_IER_MASK(LL_USARTX_IER_ELSI));
}

/** 
  * @brief  Low layer USART disable modem status interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_modem_status_interrupt_disable(USART_TypeDef *p_usart) {
    p_usart->IER &= ~(LL_USARTX_IER_MASK(LL_USARTX_IER_EDSSI));
}

/** 
  * @brief  Low layer USART disable program thre interrupt function
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  */
__STATIC_INLINE void ll_usart_program_thre_interrupt_disable(USART_TypeDef *p_usart) {
    p_usart->IER &= ~(LL_USARTX_IER_MASK(LL_USARTX_IER_PTIME));
}

/** 
  * @brief  Low layer USART Receive FIFO Level
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval Receive FIFO Level.
  */
#define LL_USART_GET_RX_FIFO_LEVEL(p_usart)     ((p_usart)->RFL)
  
/** 
  * @brief  Low layer USART Transmit FIFO Level
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval Transmit FIFO Level.
  */
#define LL_USART_GET_TX_FIFO_LEVEL(p_usart)     ((p_usart)->TFL)

/** 
  * @brief  Low layer USART 485 mode1 rx enable
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   This is only used at rs485 mode 1
  */
__STATIC_INLINE void ll_usart485_mode1_rx_enable(USART_TypeDef *p_usart) {
    p_usart->RE_EN = 1;
}

/** 
  * @brief  Low layer USART 485 mode1 rx disable
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   This is only used at rs485 mode 1
  */
__STATIC_INLINE void ll_usart485_mode1_rx_disable(USART_TypeDef *p_usart) {
    p_usart->RE_EN = 0;
}

/** 
  * @brief  Low layer USART 485 mode1 tx enable
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   This is only used at rs485 mode 1
  */
__STATIC_INLINE void ll_usart485_mode1_tx_enable(USART_TypeDef *p_usart) {
    p_usart->DE_EN = 1;
}

/** 
  * @brief  Low layer USART 485 mode1 tx disable
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   This is only used at rs485 mode 1
  */
__STATIC_INLINE void ll_usart485_mode1_tx_disable(USART_TypeDef *p_usart) {
    p_usart->DE_EN = 0;
}

/** 
  * @brief  USART Parity bit stick to low
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   Parity and Stick feature must be enabled before transfer.
  */
__STATIC_INLINE void ll_usart232_stick_parity_logic_0(USART_TypeDef *p_usart) {
    p_usart->LCR |= LL_USARTX_LCR_EPS;
}

/** 
  * @brief  USART Parity bit stick to high
  * @param  p_usart: where x can be 1, 2, 3 to select the UART.
  * @retval None.
  * @note   Parity and Stick feature must be enabled before transfer.
  */
__STATIC_INLINE void ll_usart232_stick_parity_logic_1(USART_TypeDef *p_usart) {
    p_usart->LCR &= ~LL_USARTX_LCR_EPS;
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

#endif //__TX_PHE_LL_USART_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
