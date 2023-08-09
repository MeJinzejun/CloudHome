/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_can.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the CAN LL firmware functions.
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
#ifndef __TX_PHE_LL_CAN_H
#define __TX_PHE_LL_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup can_interface_gr CAN Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup CAN_LL_Driver CAN LL Driver
  * @ingroup  can_interface_gr
  * @brief Mainly the driver part of the CAN module, which includes \b CAN \b Register 
  * \b Constants, \b CAN \b Exported \b Constants, \b CAN \b Exported \b Struct, \b CAN
  * \b Data \b transfers \b functions, \b CAN \b Initialization \b and \b CAN \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 
/** @defgroup CAN_LL_Register_Constants CAN LL Register Constants
  * @ingroup  CAN_LL_Driver
  * @brief    CAN LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the CAN 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the CAN register, mainly for convenience. Understand the 
    configuration of the CAN.
    
@endverbatim
  *
  * @{
  */

/***** CFG_STAT *****/
/*! RESET request bit  
 *  1 - The host controller performs a local reset of CAN-CTRL.  
 *  0 - no local reset of CAN-CTRL  
 *  The node configuration (BITTIME_x, x_PRESC, acceptance code and mask) can only
 *  be modified if RESET=1 
 */
#define LL_CAN_CFG_STA_RESET                      (1UL << 7)
/*! Loop Back Mode, External  
 *  0 - Disabled  
 *  1 - Enabled  
 *  LBME should not be enabled while a transmission is active. 
 */
#define LL_CAN_CFG_STA_LBME                       (1UL << 6)
/*! Loop Back Mode, Internal  
 *  0 - Disabled  
 *  1 - Enabled  
 *  LBMI should not be enabled while a transmission is active. 
 */
#define LL_CAN_CFG_STA_LBMI                       (1UL << 5)
/*! Transmission Primary Single Shot mode for PTB  
 *  0 - Disabled  
 *  1 - Enabled  
 */
#define LL_CAN_CFG_STA_TPSS                       (1UL << 4)
/*! Transmission Secondary Single Shot mode for STB  
 *  0 - Disabled  
 *  1 - Enabled  
 */
#define LL_CAN_CFG_STA_TSSS                       (1UL << 3)
/*! Reception ACTIVE (Receive Status bit)  
 *  1 - The controller is presently receiving data or a remote frame.  
 *  0 - No receive activity.   
 */
#define LL_CAN_CFG_STA_ARCTIVE                    (1UL << 2)
/*! Transmission ACTIVE (Transmit Status bit)  
 *  1 - The controller is presently transmitting data or a remote frame.  
 *  0 - No transmit activity.  
 */
#define LL_CAN_CFG_STA_TACTIVE                    (1UL << 1)
/*! Bus Off (Bus Status bit)  
 *  1 - The controller status is " bus off "  
 *  0 - The controller status is " bus on "  
 *  See detailed description in chap. 3.6.1
 */
#define LL_CAN_CFG_STA_BUSOFF                     (1UL << 0)


/***** TCMD *****/
/*! Transmit Buffer Select  
 *  Selects the transmit buffer to be loaded with a message. Use the TBUF 
 *  registers for  access. TBSEL needs to be stable all the time the TBUF 
 *  registers are written and when TSNEXT is set.  
 *  0 - PTB (high-priority buffer)  
 *  1 - STB (FIFO like)  
 */
#define LL_CAN_TCMD_TBSEL                         (1UL << 7)
/*! Listen Only Mode  
 *  0 - Disabled  
 *  1 - Enabled  
 *  LOM should not be enabled while a transmission is active. No transmission 
 *  can be started if LOM is enabled. 
 */
#define LL_CAN_TCMD_LOM                           (1UL << 6)
/*! Transceiver Standby Mode  
 *  0 - Disabled  
 *  1 - Enabled  
 *  This register bit is connected to the output signal stby which can be used 
 *  to control a standby mode of a transceiver.
 *  STBY cannot be set to 1 if TPE=1, TSONE=1 or TSALL=1. 
 */
#define LL_CAN_TCMD_STBY                          (1UL << 5)
/*! Transmit Primary Enable  
 *  1 - Transmission enable of the message of the high-priority PTB  
 *  0 - No transmission for the PTB  
 *  If TPE is set, the message from the PTB will be transmitted at the next possible 
 *  transmit position. A started transmission from the STB will be completed before, 
 *  but pending new messages are delayed until the PTB message has been transmitted. 
 */
#define LL_CAN_TCMD_TPE                           (1UL << 4)
/*! Transmit Primary Abort  
 *  1-  Aborts a transmission from TPB which has been requested by TPE=1 but not
 *      started yet.
 *      (The data bytes of the message remain in the PTB.)  
 *  0-  no abort  
 *  The bit has to be set by the host controller and will be reset by CAN-CTRL. 
 *  Setting TPA automatically de-asserts TPE. 
 */
#define LL_CAN_TCMD_TPA                           (1UL << 3)
/*! Transmit Secondary ONE frame  
 *  1-  Transmission enable of the oldest message in the STB.
 *      The message format is stored in the IDE buffer which belongs to this message.
 *      The controller starts the transmission as soon as the bus becomes vacant and
 *      no request of the PTB (bit TPE) is pending.  
 *  0-  No transmission for the STB.  
 *  TSONE stays set until the message has been transmitted successfully or it is aborted 
 *  using TSA. Only the host controller can set TSONE 
 */
#define LL_CAN_TCMD_TSONE                         (1UL << 2)
/*! Transmit Secondary ALL frames  
 *  1-  Transmission enable of all messages in the STB.
 *      The message format is stored in the appropriate IDE buffers.
 *      The controller starts the transmission as soon as the bus becomes vacant and
 *      no request of the PTB (bit TPE) is pending.  
 *  0-  No transmission for the STB.  
 *  TSALL stays set until all messages have been transmitted successfully or they are aborted 
 *  using TSA. Only the host controller can set TSALL 
 */
#define LL_CAN_TCMD_TSALL                         (1UL << 1)
/*! Transmit Secondary Abort  
 *  1-  Aborts a transmission from STB which has been requested but not started yet.
 *      For a TSONE transmission, only one frame is aborted while for a TSALL
 *      Transmission, all frames are aborted 
 *      One or all message slots will be released which updates TSSTAT. Because of
 *      the FIFO-like behavior of the STB all aborted messages are lost because they
 *      are not accessible any more.  
 *  0-  no abort  
 *  The bit has to be set by the host controller and will be reset by CAN-CTRL. Setting TSA, 
 *  automatically de-asserts TSONE or TSALL respectively
 */
#define LL_CAN_TCMD_TSA                           (1UL << 0)


/***** TCTRL *****/
/*! CAN FD ISO mode  
 *  0 - Bosch CAN FD (non-ISO) mode  
 *  1 - ISO CAN FD mode  
 *  This bit is only writeable if RESET=1.
 *  ISO CAN FD mode has a different CRC initialization value and an additional stuff bit count. 
 *  Both modes are incompatible and must not be mixed in one CAN network.
 *  This bit has no impact to CAN 2.0B. 
 */
#define LL_CAN_TCTRL_FD_ISO                       (1UL << 7)
/*! Transmit buffer Secondary NEXT  
 *  0 - no action  
 *  1 - STB slot filled, select next FIFO slot.  
 *  After all frame bytes are written to the TBUF registers, the host controller has to set 
 *  TSNEXT to signal that this slot has been filled. Then the CAN-CTRL core connects the TBUF 
 *  registers to the next FIFO slot. Once a slot is marked as filled a transmission can be 
 *  started using TSONE or TSALL 
 */
#define LL_CAN_TCTRL_TSNEXT                       (1UL << 6)
/*! Transmission Secondary STATus bits Number of filled message buffers (0 ~ 16). 
 */
#define LL_CAN_TCTRL_TSSTAT(n)                    (((n)&0x1F) << 0)


/***** RCTRL *****/
/*! Receive buffer OVerflow  
 *  1-  Overflow. At least one message is lost.  
 *  0-  No Overflow.  
 *  ROV is cleared by setting RREL=1. 
 */
#define LL_CAN_RCTRL_ROV                          (1UL << 5)
/*! Receive buffer RELease  
 *  The host controller acknowledges the emptying of the actual RB slot. Afterwards the 
 *  CAN-CTRL core points to the next RB slot. RSTAT gets updated.  
 *  1-  Release: The host has emptied the RB.  
 *  0-  No release  
 */
#define LL_CAN_RCTRL_RREL                         (1UL << 4)
/*! Receive buffer status  
 *  00 - empty  
 *  01 - > empty and < almost full (AFWL)  
 *  10 - >= almost full (programmable threshold by AFWL) but not full and no overflow  
 *  11 - full (stays set in case of overflow for overflow signaling see ROV)  
 */
#define LL_CAN_RCTRL_RSTAT(n)                     (((n)&0x3) << 0)


/***** RTIE *****/
/*! Receive Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_RIE                           (1UL << 7)
/*! RB Overrun Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled
 */
#define LL_CAN_RTIE_ROIE                          (1UL << 6)
/*! RB Full Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_RFIE                          (1UL << 5)
/*! RB Almost Full Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_RAFIE                         (1UL << 4)
/*! Transmission Primary Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_TPIE                          (1UL << 3)
/*! Transmission Secondary Interrupt Enable  
 *  0-  Disabled  
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_TSIE                          (1UL << 2)
/*! Error Interrupt Enable  
 *  0-  Disabled
 *  1-  Enabled  
 */
#define LL_CAN_RTIE_EIE                           (1UL << 1)
/*! Transmit Secondary buffer Full Flag  
 *  1 - The STB is filled with the maximal count of messages.  
 *  0 - The STB is not filled with the maximal count of messages  
 */
#define LL_CAN_RTIE_TSFF                          (1UL << 0)


/***** RTIF *****/
/*! Receive Interrupt Flag  
 *  1 - Data or a remote frame has been received and is available in the receive buffer.  
 *  0 - No frame has been received  
 */
#define LL_CAN_RTIF_RIF                           (1UL << 7)
/*! RB Overrun Interrupt Flag  
 *  1 - At least one received message has been overwritten in the RB.  
 *  0 - No RB overwritten.  
 *  In case of an overrun both ROIF and RFIF will be set.
 */
#define LL_CAN_RTIF_ROIF                          (1UL << 6)
/*! RB Full Interrupt Flag  
 *  1 - All RBs are full. If no RB will be released until the next valid message is received,
 *      the oldest message will be lost.  
 *  0 - The RB FIFO is not full   
 */
#define LL_CAN_RTIF_RFIF                          (1UL << 5)
/*! RB Almost Full Interrupt Flag  
 *  1 - number of filled RB slots >= AFWL_i  
 *  0 - number of filled RB slots < AFWL_i  
*/
#define LL_CAN_RTIF_RAFIF                         (1UL << 4)
/*! Transmission Primary Interrupt Flag  
 *  1 - The requested transmission of the PTB has been successfully completed.  
 *  0 - No transmission of the PTB has been completed.  
 */
#define LL_CAN_RTIF_TPIF                          (1UL << 3)
/*! Transmission Secondary Interrupt Flag  
 *  1 - The requested transmission of the STB has been successfully completed.  
 *  0 - No transmission of the STB has been completed successfully  
 */
#define LL_CAN_RTIF_TSIF                          (1UL << 2)
/*! Error Interrupt Flag  
 *  1 - The border of the error warning limit has been crossed in either direction,
 *      or the BUSOFF bit has been changed in either direction.  
 *  0 - There has been no change  
 */
#define LL_CAN_RTIF_EIF                           (1UL << 1)
/*! Abort Interrupt Flag  
 *  1 - After setting TPA or TSA the appropriated message(s) have been aborted.
 *      It is recommended to not set both TPA and TSA simultaneously because both
 *      source AIF.  
 *  0 - No abort has been executed.  
 *  The AIF does not have an associated enable register.
 *  See also chap. 3.6.6 for further information 
 */
#define LL_CAN_RTIF_AIF                           (1UL << 0)


/***** ERRINT *****/
/*! Error WARNing limit reached  
 *  1 - One of the error counters RECNT or TECNT is equal or bigger than EWL  
 *  0 - The values in both counters are less than EWL.  
 */
#define LL_CAN_ERRINT_EWARN                       (1UL << 7)
/*! Error Passive mode active  
 *  0 - not active (node is error active)  
 *  1 - active (node is error passive)  
 */
#define LL_CAN_ERRINT_EPASS                       (1UL << 6)
/*! Error Passive Interrupt Enable
 */
#define LL_CAN_ERRINT_EPIE                        (1UL << 5)
/*! Error Passive Interrupt Flag. EPIF will be activated if the error status changes 
 *  from error active to error passive or vice versa and if this interrupt is enabled 
 */
#define LL_CAN_ERRINT_EPIF                        (1UL << 4)
/*! Arbitration Lost Interrupt Enable
 */
#define LL_CAN_ERRINT_ALIE                        (1UL << 3)
/*! Arbitration Lost Interrupt Flag
 */
#define LL_CAN_ERRINT_ALIF                        (1UL << 2)
/*! Bus Error Interrupt Enable
 */
#define LL_CAN_ERRINT_BEIE                        (1UL << 1)
/*! Bus Error Interrupt Flag
 */
#define LL_CAN_ERRINT_BEIF                        (1UL << 0)


/***** BITTIME_0 *****/
/*! Synchronization Jump Width (fast speed)  
 *  The Synchronization Jump Width T_sjw = (SJW + 1) * TQ is the maximum time for 
 *  shortening or lengthening the Bit Time for resynchronization, where TQ is a time 
 *  quanta 
 */
#define LL_CAN_BITTIME_0_F_SJW(n)                 (((n)&0x3) << 6)
/*! Bit Timing Segment 1 (slow speed)  
 *  The sample point will be set to T_seg_1 = (Seg_1 + 2) * TQ after start of bit time.
 *  Seg_1=0 is meaningless and automatically treated as 1. 
 */
#define LL_CAN_BITTIME_0_S_SEG_1(n)               (((n)&0x3F) << 0)


/***** BITTIME_1 *****/
/*! Bit Timing Segment 2 (fast speed)  
 *  Time T_seg_2 = (Seg_2 + 1) * TQ after the sample point. Seg_2=0 is meaningless and
 *  automatically treated as 1 
 */
#define LL_CAN_BITTIME_1_F_SEG_2(n)               (((n)&0x7) << 5)
/*! Bit Timing Segment 2 (slow speed)  
 *  Time T_seg_2 = (Seg_2 + 1) * TQ after the sample point. Seg_2=0 is meaningless and
 *  automatically treated as 1 
 */
#define LL_CAN_BITTIME_1_S_SEG_2(n)               (((n)&0x1F) << 0)


/***** BITTIME_2 *****/
/*! Bit Timing Segment 1 (fast speed)  
 *  The sample point will be set to T_seg_1 = (Seg_1 + 2) * TQ after start of bit time.
 *  Seg_1=0 is meaningless and automatically treated as 1. 
 */
#define LL_CAN_BITTIME_2_F_SEG_1(n)               (((n)&0xF) << 4)
/*! Synchronization Jump Width (slow speed)  
 *  The Synchronization Jump Width T_sjw = (SJW + 1) * TQ is the maximum time for 
 *  shortening or lengthening the Bit Time for resynchronization, where TQ is a time 
 *  quanta. 
 */
#define LL_CAN_BITTIME_2_S_SJW(n)                 (((n)&0xF) << 0)


/***** S_PRESC  *****/
/*! Prescaler (slow and fast speed)  
 *  The prescaler divides the system clock to get the time quanta clock tq_clk.
 *  Valid range PRESC=[0x01, 0xff] results in divider values 2 to 256.
 *  PRESC=0 is forbidden and automatically treated as 1. (Necessary because the bit timing 
 *  logic of this core requires it.) 
 */
#define LL_CAN_S_PRESC(n)                         (((n)&0xFF) << 0)


/***** F_PRESC *****/
/*! Prescaler (slow and fast speed)  
 *  The prescaler divides the system clock to get the time quanta clock tq_clk.
 *  Valid range PRESC=[0x01, 0xff] results in divider values 2 to 256.
 *  PRESC=0 is forbidden and automatically treated as 1. (Necessary because the bit timing 
 *  logic of this core requires it.) 
 */
#define LL_CAN_F_PRESC(n)                         (((n)&0xFF) << 0)


/***** TDC *****/
/*! Transmitter Delay Compensation ENable  
 *  TDC will be activated during the data phase of a CAN FD frame if BRS is active if TDCEN=1.
 *  For more details about TDC see chap. 4.5 
 */
#define LL_CAN_TDC_TDCEN                          (1UL << 7)
/*! Secondary Sample Point OFFset  
 *  The transmitter delay plus SSPOFF defines the time of the secondary sample point for TDC.
 *  SSPOFF is given as a number of TQ. 
 */
#define LL_CAN_TDC_SSPOFF(n)                      (((n)&0x1F) << 0)


/***** LIMIT *****/
/*! receive buffer Almost Full Warning Limit
 */
#define LL_CAN_LIMIT_AFWL(n)                      (((n)&0xF) << 4)
/*! Programmable Error Warning Limit = (EWL+1)*8. Possible Limit values: 8, 16, ... 128.
 *  The value of EWL controls EIF.
 *  EWL needs to be transferred using CDC from host to CAN clock domain. During transfer 
 *  EWL register bits are write-locked for the host for a few clocks until CDC is complete 
 */
#define LL_CAN_LIMIT_EWL(n)                       (((n)&0xF) << 0)


/***** EALCAP *****/
/*! Kind Of ERror (Error code)  
 *  000 - no error  
 *  001 - BIT ERROR  
 *  010 - FORM ERROR  
 *  011 - STUFF ERROR  
 *  100 - ACKNOWLEDGEMENT ERROR  
 *  101 - CRC ERROR  
 *  110 - OTHER ERROR
 *        (dominant bits after own error flag, received active Error Flag too long,
 *        dominant bit during Passive-Error-Flag after ACK error)
 *  111 - not used  
 *  KOER is reset after a successful transmission or reception of a frame 
 */
#define LL_CAN_EALCAP_KOER(n)                     (((n)&0x7) << 5)
/*! Arbitration Lost Capture (bit position in the frame where the arbitration has been lost)
 */
#define LL_CAN_EALCAP_ALC(n)                      (((n)&0x1F) << 0)


/***** RECNT *****/
/*! Receive Error CouNT (number of errors during reception)  
 *  RECNT is incremented and decremented as defined in the CAN specification.
 *  RECNT does not overflow. RECNT signals 0xff = 255 as maximum value. See chap. 3.6.1 
 *  for more details about RECNT and the " bus off " state 
 */
#define LL_CAN_RECNT(n)                           (((n)&0xFF) << 0)


/***** TECNT *****/
/*! Receive Error CouNT (number of errors during reception)  
 *  RECNT is incremented and decremented as defined in the CAN specification.
 *  RECNT does not overflow. RECNT signals 0xff = 255 as maximum value. See chap. 3.6.1 
 *  for more details about RECNT and the " bus off " state 
 */
#define LL_CAN_TECNT(n)                           (((n)&0xFF) << 0)


/***** ACFCTRL *****/
/*! SELect acceptance MASK  
 *  0 - Registers ACF_x point to acceptance code  
 *  1 - Registers ACF_x point to acceptance mask  
 *  ACFADR selects one specific acceptance filter. (See Figure 3-4.) 
 */
#define LL_CAN_ACFCTRL_SELMASK                    (1UL << 5)
/*! acceptance filter address  
 *  ACFADR points to a specific acceptance filter. The selected filter is accessible using the 
 *  registers ACF_x. Bit SELMASK selects between acceptance code and mask for the 
 *  selected acceptance filter. (See Figure 3-4.)
 *  A value of ACFADR>NR_OF_ACF-1 is meaningless and automatically treated as value 
 *  NR_OF_ACF-1. (See chap. 0 and Figure 3-4 for details about NR_OF_ACF.) 
 */
#define LL_CAN_ACFCTRL_ACFADR(n)                  (((n)&0xF) << 0)


/***** ACODE_X *****/
/*! Acceptance CODE  
 *  1 - ACC bit value to compare with ID bit of the received message  
 *  0 - ACC bit value to compare with ID bit of the received message  
 *  ACODE_x(10:0) will be used for extended frames.  
 *  ACODE_x(28:0) will be used for extended frames.  
 *  Only filter 0 is affected by the power-on reset. All other filters stay uninitialized.
 *  See chap. 3.6.2 for further details 
 */
#define LL_CAN_ACODE_X(n)                         (((n)&0x1FFFFFFF) << 0)


/***** AMASK_X *****/
/*! Acceptance MASK  
 *  1 - acceptance check for these bits of receive identifier disabled  
 *  0 - acceptance check for these bits of receive identifier enable  
 *  AMASK_x(10:0) will be used for extended frames.  
 *  AMASK_x(28:0) will be used for extended frames.  
 *  Disabled bits result in accepting the message. Therefore the default configuration after 
 *  reset for filter 0 accepts all messages.
 *  Only filter 0 is affected by the power-on reset. All other filters stay uninitialized.
 *  See chap. 3.6.2 for further details 
 */
#define LL_CAN_AMASK_X(n)                         (((n)&0x1FFFFFFF) << 0)


/***** ACF_3 *****/
/*! Acceptance mask IDE bit check enable  
 *  1 - acceptance filter accepts either standard or extended as defined by AIDE  
 *  0 - acceptance filter accepts both standard or extended frames  
 *  Only filter 0 is affected by the power-on reset. All other filters stay uninitialized.
 */
#define LL_CAN_ACF_3_AIDEE                        (1UL << 1)
/*! Acceptance mask IDE bit value  
 *  If AIDEE=1 then:  
 *  1 - acceptance filter accepts only extended frames  
 *  0 - acceptance filter accepts only standard frames  
 *  Only filter 0 is affected by the power-on reset. All other filters stay uninitialized 
 */
#define LL_CAN_ACF_3_AIDE                         (1UL << 0)


/***** ACF_EN_0 *****/
/*! Acceptance filter Enable  
 *  1 - acceptance filter enabled  
 *  0 - acceptance filter disable  
 *  Each acceptance filter (AMASK / ACODE) can be individually enabled or disabled. Only 
 *  filter number 0 is enabled by default after hardware reset.
 *  Disabled filters reject a message. Only enabled filters can accept a message if the 
 *  appropriate AMASK / ACODE configuration matches.
 *  To accept all messages one filter x has to be enabled by setting AE_x=1, 
 *  AMASK_x=0xff and ACODE_x=0x00. This is the default configuration after hardware 
 *  reset for filter x=0 while all other filters are disabled 
 */
#define LL_CAN_ACF_EN_0(n)                        (((n)&0xFF) << 0)


/***** ACF_EN_1 *****/
/*! Acceptance filter Enable  
 *  1 - acceptance filter enabled  
 *  0 - acceptance filter disable  
 *  Each acceptance filter (AMASK / ACODE) can be individually enabled or disabled.
 *  Disabled filters reject a message. Only enabled filters can accept a message if the 
 *  appropriate AMASK / ACODE configuration matches 
 */
#define LL_CAN_ACF_EN_1(n)                        (((n)&0xFF) << 0)


/***** VER_0 *****/
/*! Version of CAN-CTRL, given as decimal value. VER_1 holds the major version and 
 *  VER_0 the minor version.  
 *  Example: version 5x16N00S00 is represented by VER_1=5 and VER_0=16. 
 */
#define LL_CAN_VER_0(n)                           (((n)&0xFFFF) << 0)


/***** VER_1 *****/
/*! Version of CAN-CTRL, given as decimal value. VER_1 holds the major version and 
 *  VER_0 the minor version.  
 *  Example: version 5x16N00S00 is represented by VER_1=5 and VER_0=16. 
 */
#define LL_CAN_VER_1(n)                           (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup CAN_LL_Exported_Constants CAN LL Exported Constants
  * @ingroup  CAN_LL_Driver
  * @brief    CAN LL external constant definition
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

/*! The address of can is 11 bits. 
 */
#define LL_CAN_BUF_ID_11BITS(n)                   (((n)&0x7FF) << 0)
/*! The address of can is 29 bits. 
 */
#define LL_CAN_BUF_ID_29BITS(n)                   (((n)&0x1FFFFFFF) << 0)

/*! The macro defines the configuration enable of the CAN.
 */
#define LL_CAN_REG_OPT(expression)        \
do{                                       \
    CAN->CFGSTA |= LL_CAN_CFG_STA_RESET;  \
    expression;                           \
    CAN->CFGSTA &= ~LL_CAN_CFG_STA_RESET; \
}while(0)


/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for CAN address filter channel selection.
  */
typedef enum {
    /*! CAN address filter channel select: channel 0. 
     */
    LL_CAN_ADDR_FILT_CHN_0 = 0,
    /*! CAN address filter channel select: channel 1. 
     */
    LL_CAN_ADDR_FILT_CHN_1,
    /*! CAN address filter channel select: channel 2. 
     */
    LL_CAN_ADDR_FILT_CHN_2,
    /*! CAN address filter channel select: channel 3. 
     */
    LL_CAN_ADDR_FILT_CHN_3,
    /*! CAN address filter channel select: channel 4. 
     */
    LL_CAN_ADDR_FILT_CHN_4,
    /*! CAN address filter channel select: channel 5. 
     */
    LL_CAN_ADDR_FILT_CHN_5,
    /*! CAN address filter channel select: channel 6. 
     */
    LL_CAN_ADDR_FILT_CHN_6,
    /*! CAN address filter channel select: channel 7. 
     */
    LL_CAN_ADDR_FILT_CHN_7,
    /*! CAN address filter channel select: channel 8. 
     */
    LL_CAN_ADDR_FILT_CHN_8,
    /*! CAN address filter channel select: channel 9. 
     */
    LL_CAN_ADDR_FILT_CHN_9,
    /*! CAN address filter channel select: channel 10. 
     */
    LL_CAN_ADDR_FILT_CHN_10,
    /*! CAN address filter channel select: channel 11. 
     */
    LL_CAN_ADDR_FILT_CHN_11,
    /*! CAN address filter channel select: channel 12. 
     */
    LL_CAN_ADDR_FILT_CHN_12,
    /*! CAN address filter channel select: channel 13. 
     */
    LL_CAN_ADDR_FILT_CHN_13,
    /*! CAN address filter channel select: channel 14. 
     */
    LL_CAN_ADDR_FILT_CHN_14,
    /*! CAN address filter channel select: channel 15. 
     */
    LL_CAN_ADDR_FILT_CHN_15,
} TYPE_ENUM_LL_CAN_ADDR_FILT_CHN;

/**
  * @brief Enumeration constant for CAN receive almost full warnning selection.
  */
typedef enum {
    /*! CAN receive almost full warnning select: 0 Byte, the default is 1. 
     */
    LL_CAN_RX_AFWL_SEL_0 = 0,
    /*! CAN receive almost full warnning select: 1 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_1,
    /*! CAN receive almost full warnning select: 2 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_2,
    /*! CAN receive almost full warnning select: 3 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_3,
    /*! CAN receive almost full warnning select: 4 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_4,
    /*! CAN receive almost full warnning select: 5 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_5,
    /*! CAN receive almost full warnning select: 6 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_6,
    /*! CAN receive almost full warnning select: 7 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_7,
    /*! CAN receive almost full warnning select: 8 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_8,
    /*! CAN receive almost full warnning select: 9 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_9,
    /*! CAN receive almost full warnning select: 10 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_10,
    /*! CAN receive almost full warnning select: 11 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_11,
    /*! CAN receive almost full warnning select: 12 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_12,
    /*! CAN receive almost full warnning select: 13 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_13,
    /*! CAN receive almost full warnning select: 14 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_14,
    /*! CAN receive almost full warnning select: 15 Byte. 
     */
    LL_CAN_RX_AFWL_SEL_15,
} TYPE_ENUM_LL_CAN_RX_AFWL_SEL;

/**
  * @brief Enumeration constant for CAN program error warnning selection.
  */
typedef enum {
    /*! CAN program error warnning select: Count up to 8 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_8 = 0,
    /*! CAN program error warnning select: Count up to 16 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_16,
    /*! CAN program error warnning select: Count up to 24 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_24,
    /*! CAN program error warnning select: Count up to 32 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_32,
    /*! CAN program error warnning select: Count up to 40 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_40,
    /*! CAN program error warnning select: Count up to 48 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_48,
    /*! CAN program error warnning select: Count up to 56 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_56,
    /*! CAN program error warnning select: Count up to 64 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_64,
    /*! CAN program error warnning select: Count up to 72 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_72,
    /*! CAN program error warnning select: Count up to 80 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_80,
    /*! CAN program error warnning select: Count up to 88 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_88,
    /*! CAN program error warnning select: Count up to 96 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_96,
    /*! CAN program error warnning select: Count up to 104 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_104,
    /*! CAN program error warnning select: Count up to 112 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_112,
    /*! CAN program error warnning select: Count up to 120 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_120,
    /*! CAN program error warnning select: Count up to 128 bytes. 
     */
    LL_CAN_PROG_ERRWL_SEL_128,
} TYPE_ENUM_LL_CAN_PROG_ERRWL_SEL;

/**
  * @}
  */

/** @defgroup CAN_LL_Exported_Struct CAN LL Exported Struct
  * @ingroup  CAN_LL_Driver
  * @brief    CAN LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the CAN registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_can_init. Function, you can configure the CAN module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief CAN interrupt low layer configuration structure
  */
typedef struct __ll_can_irq_cfg {
    /*! Receive interrupt enable 
     */
    bool rx_intr_en;
    /*! Receive buffer overrun interrupt enable 
     */
    bool rx_buf_overrun_intr_en;
    /*! Receive buffer full interrupt enable 
     */
    bool rx_buf_full_intr_en;
    /*! Receive buffer Almost Full Interrupt Enable 
     */
    bool rx_buf_almost_full_intr_en;
    /*! Transmission primary interrupt enable 
     */
    bool tx_primary_intr_en;
    /*! Transmission secondary interrupt enable 
     */
    bool tx_secondary_intr_en;
    /*! Error interrupt enable 
     */
    bool err_intr_en;
    /*! Error passive interrupt enable 
     */
    bool err_passive_intr_en;
    /*! Arbitration lost interrupt enable 
     */
    bool arbitration_lost_intr_en;
    /*! Bus error interrupt enable 
     */
    bool bus_err_intr_en;
} TYPE_LL_CAN_IRQ_CFG;

/**
  * @brief CAN buffer format low layer configuration structure
  */
typedef struct __ll_can_rx_buf_format {  
    /*! Standard/Extended iDentifier value 
     */
    u32 id                  : 29,
    /*! Reserved bit. 
     */
        reserved1           : 2,
    /*! Error State Indicator. This is a read-only status bit for RBUF and is not available 
     *  in TBUF. The protocol machine automatically embeds the correct value of ESI into 
     *  transmitted frames. ESI is only included in CAN FD frames and does not exist in CAN 
     *  2.0 frames. 
     */
        err_state_indicator : 1;
    /*! The Data Length Code (DLC) in RBUF and TBUF defines the length of the payload(the 
     *  number of payload bytes in a frame). 
     */
    u32 data_len_code     : 4,
    /*! Bit Rate Switch
     *  0: nominal / slow bit rate for the complete frame.
     *  1: switch to data / fast bit rate for the data payload and the CRC
     *  Only CAN FD frames can switch the bitrate. Therefore BRS is forced to 0 if EDL=0 
     */
        bit_rate_switch   : 1,
    /*! Extended Data Length
     *  0: CAN 2.0 frame (up to 8 bytes payload)
     *  1: CAN FD frame (up to 64 bytes payload) 
     */
        extended_data_len : 1,
    /*! Remote Transmission Request
     *  0: data frame
     *  1: remote frame 
     *  Only CAN 2.0 frames can be remote frames. There is no remote frame for CAN FD. 
     *  Therefore RTR is forced to 0 if EDL=1 in the TBUF.
     */
        remote_tx_req     : 1,
    /*! IDentifier Extension
     *  0: Standard Format: ID(10:0)
     *  1: Extended Format: ID(28:0)
     */
        id_extension      : 1,
    /*! Reserved bit. 
     */
        reserved2         : 24;
} TYPE_LL_CAN_RX_BUF_FORMAT;

/**
  * @brief CAN buffer format low layer configuration structure
  */
typedef struct __ll_can_tx_buf_format {  
    /*! Standard/Extended iDentifier value 
     */
    u32 id                : 29,
    /*! Reserved bit. 
     */
        reserved1         : 3;
    
    /*! The Data Length Code (DLC) in RBUF and TBUF defines the length of the payload(the 
     *  number of payload bytes in a frame). 
     */
    u32 data_len_code     : 4,
    /*! Bit Rate Switch  
     *  0: nominal / slow bit rate for the complete frame.  
     *  1: switch to data / fast bit rate for the data payload and the CRC  
     *  Only CAN FD frames can switch the bitrate. Therefore BRS is forced to 0 if EDL=0 
     */
        bit_rate_switch   : 1,
    /*! Extended Data Length  
     *  0: CAN 2.0 frame (up to 8 bytes payload)  
     *  1: CAN FD frame (up to 64 bytes payload)  
     */
        extended_data_len : 1,
    /*! Remote Transmission Request  
     *  0: data frame  
     *  1: remote frame  
     *  Only CAN 2.0 frames can be remote frames. There is no remote frame for CAN FD. 
     *  Therefore RTR is forced to 0 if EDL=1 in the TBUF.
     */
        remote_tx_req     : 1,
    /*! IDentifier Extension  
     *  0: Standard Format: ID(10:0)  
     *  1: Extended Format: ID(28:0)  
     */
        id_extension      : 1,
    /*! Reserved bit. 
     */
        reserved2         : 24;
} TYPE_LL_CAN_TX_BUF_FORMAT;

/**
  * @brief CAN address filter low layer configuration structure
  */
typedef struct __ll_can_addr_filt_cfg {
    /*! can address filter channel select
     */
    TYPE_ENUM_LL_CAN_ADDR_FILT_CHN chn;
    /*! channel enable bit
     */
    bool                           enable;
    /*! Acceptance CODE value  
     *  acceptance bit value to compare with ID bit of the received message.
     */
    u32                            acode_val;
    /*! Acceptance CODE value  
     *  acceptance check for these bits of receive identifier disabled.
     */
    u32                            amask_val;
} TYPE_LL_CAN_ADDR_FILT_CFG;

/**
  * @brief CAN low layer configuration structure
  */
typedef struct __ll_can_cfg {
    /*! Configure the baud rate of CAN.
     */
    u8                              baudrate;
    /*! Can receive the buffer almost full count condition selection.
     */
    TYPE_ENUM_LL_CAN_RX_AFWL_SEL    rx_almost_full_limit_sel;
    /*! Can send buffer, there is an error number warning selection.
     */
    TYPE_ENUM_LL_CAN_PROG_ERRWL_SEL program_err_limit_sel;
} TYPE_LL_CAN_CFG;

/**
  * @brief CAN low layer Initialization structure
  */
typedef struct __ll_can_init {
    u8 reserved;
} TYPE_LL_CAN_INIT;

/**
  * @}
  */

/** @defgroup CAN_LL_Interrupt CAN LL Interrupt Handle function
  * @brief   CAN LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CAN  
    Interrupt Handle function.

    how to use?

    The CAN interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the CAN in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup CAN_LL_Inti_Cfg CAN LL Initialization And Configuration
  * @brief    CAN LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CAN data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer CAN module initialization
  * @param  p_can : The register structure pointer of the CAN.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_CAN_INIT)
  * @retval None
  */
void ll_can_init(CAN_TypeDef *p_can, TYPE_LL_CAN_INIT *p_init);

/**
  * @brief  Low layer CAN module detele initialization
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_deinit(CAN_TypeDef *p_can);

/**
  * @brief  Low layer CAN module interrupt configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module interrupt configuration structure pointer
  *                (TYPE_LL_CAN_IRQ_CFG)
  * @retval None
  */
void ll_can_irq_config(CAN_TypeDef *p_can, TYPE_LL_CAN_IRQ_CFG *p_cfg) ;

/**
  * @brief  Low layer CAN module address filter configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module address filter configuration structure pointer
  *                (TYPE_LL_CAN_ADDR_FILT_CFG)
  * @retval None
  */
void ll_can_address_filter_config(CAN_TypeDef *p_can, TYPE_LL_CAN_ADDR_FILT_CFG *p_cfg);

/**
  * @brief  Low layer CAN module configuration
  * @param  p_can: The register structure pointer of the CAN.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_CAN_CFG)
  * @retval None
  */
void ll_can_config(CAN_TypeDef *p_can, TYPE_LL_CAN_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup CAN_LL_Data_Transfers CAN LL Data transfers functions
  * @brief    CAN LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the CAN data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  CAN module start function
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_start(CAN_TypeDef *p_can);

/**
  * @brief  CAN module stop function
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
void ll_can_stop(CAN_TypeDef *p_can);

/**
  * @brief  CAN sends the function of the standard Primary Transmit Buffer.
  * @param  p_can       : The register structure pointer of the CAN.
  * @param  p_buf_format: The frame header format in which can can send data.
  * @param  p_data      : The header address pointer for sending data.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_PTB(CAN_TypeDef               *p_can, 
                              TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                              u32                       *p_data);

/**
  * @brief  CAN sends a package of standard Secondary Transmit Buffer functions.
  * @param  p_can       : The register structure pointer of the CAN.
  * @param  p_buf_format: The frame header format in which can can send data.
  * @param  p_data      : The header address pointer for sending data.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_STB(CAN_TypeDef               *p_can, 
                              TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                              u32                       *p_data);

/**
  * @brief  CAN sends a multi-package standard Secondary Transmit Buffer function.
  * @param  p_can       : The register structure pointer of the CAN.
  * @param  p_buf_format: The frame header format in which can can send data.
  * @param  p_data      : The header address pointer for sending data.
  * @param  send_cnt    : The number of CAN packets.
  * @retval None
  * @note   The format of each frame of p_data must be aligned in 4 bytes.
  */
void ll_can_send_standard_STB_most(CAN_TypeDef               *p_can, 
                                   TYPE_LL_CAN_TX_BUF_FORMAT *p_buf_format, 
                                   u32                       *p_data, 
                                   u32                        send_cnt);

/**
  * @brief  Low layer CAN get Receive buffer status
  * @param  p_can  : The register structure pointer of the CAN.
  * @param  sta_val: Receive buffer status value:
  *                   00 - empty  
  *                   01 - > empty and < almost full (AFWL)  
  *                   10 - >= almost full (programmable threshold by AFWL) but not full and no overflow  
  *                   11 - full (stays set in case of overflow for overflow signaling see ROV)  
  * @retval status value
  */
#define LL_CAN_GET_RX_BUFF_STATUS(p_can, sta_val)                 ((p_can)->RCTRL & LL_CAN_RCTRL_RSTAT(sta_val))

/**
  * @brief  Low layer CAN get RTIF register value
  * @param  p_can: The register structure pointer of the CAN.
  * @retval register value
  */
#define LL_CAN_GET_RTIF_PENDING(p_can)                            ((p_can)->RTIF)

/**
  * @brief  Low layer CAN get ERRINT register value
  * @param  p_can: The register structure pointer of the CAN.
  * @retval register value
  */
#define LL_CAN_GET_ERRINT_PENDING(p_can)                          ((p_can)->ERRINT)

/**
  * @brief  Low layer CAN check Receive interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_RECEIVE_INTERRUPT_ENABLE(p_can)              ((p_can)->RTIE & LL_CAN_RTIE_RIE)

/**
  * @brief  Low layer CAN check Receive buffer over run interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_RECEIVE_OVERRUN_INTERRUPT_ENABLE(p_can)      ((p_can)->RTIE & LL_CAN_RTIE_ROIE)

/**
  * @brief  Low layer CAN check Receive buffer full interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_RECEIVE_BUFF_FULL_INTERRUPT_ENABLE(p_can)    ((p_can)->RTIE & LL_CAN_RTIE_RFIE)

/**
  * @brief  Low layer CAN check Receive buffer almost full interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_RECEIVE_ALMOST_FULL_INTERRUPT_ENABLE(p_can)  ((p_can)->RTIE & LL_CAN_RTIE_RAFIE)

/**
  * @brief  Low layer CAN check Transmission Primary interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_TRANS_PRIMARY_INTERRUPT_ENABLE(p_can)        ((p_can)->RTIE & LL_CAN_RTIE_TPIE)

/**
  * @brief  Low layer CAN check Transmission Secondary interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_TRANS_SECONDARY_INTERRUPT_ENABLE(p_can)      ((p_can)->RTIE & LL_CAN_RTIE_TSIE)

/**
  * @brief  Low layer CAN check error interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_ERR_INTERRUPT_ENABLE(p_can)                  ((p_can)->RTIE & LL_CAN_RTIE_EIE)

/**
  * @brief  Low layer CAN check abort interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_ERR_PASSIVE_INTERRUPT_ENABLE(p_can)          ((p_can)->ERRINT & LL_CAN_ERRINT_EPIE)

/**
  * @brief  Low layer CAN check error passive interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_ARBITRATION_LOST_INTERRUPT_ENABLE(p_can)     ((p_can)->ERRINT & LL_CAN_ERRINT_ALIE)

/**
  * @brief  Low layer CAN check Arbitration Lost interrupt enable
  * @param  p_can: The register structure pointer of the CAN.
  * @retval result
  */
#define LL_CAN_CHECK_BUS_ERR_INTERRUPT_ENABLE(p_can)              ((p_can)->ERRINT & LL_CAN_ERRINT_BEIE)

/**
  * @brief  Low layer CAN get Receive Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_RECEIVE_PENDING(p_can)                         ((p_can)->RTIF & LL_CAN_RTIF_RIF)

/**
  * @brief  Low layer CAN get Receive buffer Overrun Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_RECEIVE_OVERRUN_PENDING(p_can)                 ((p_can)->RTIF & LL_CAN_RTIF_ROIF)

/**
  * @brief  Low layer CAN get Receive buffer Full Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_RECEIVE_FULL_PENDING(p_can)                    ((p_can)->RTIF & LL_CAN_RTIF_RFIF)

/**
  * @brief  Low layer CAN get Receive buffer Almost Full Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_RECEIVE_ALMOST_FULL_PENDING(p_can)             ((p_can)->RTIF & LL_CAN_RTIF_RAFIF)

/**
  * @brief  Low layer CAN get Transmission Primary Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_TRANS_PRIMARY_PENDING(p_can)                   ((p_can)->RTIF & LL_CAN_RTIF_TPIF)

/**
  * @brief  Low layer CAN get Transmission Secondary Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_TRANS_SECONDARY_PENDING(p_can)                 ((p_can)->RTIF & LL_CAN_RTIF_TSIF)

/**
  * @brief  Low layer CAN get Error Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_ERR_PENDING(p_can)                             ((p_can)->RTIF & LL_CAN_RTIF_EIF)

/**
  * @brief  Low layer CAN get Abort Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_ABORT_PENDING(p_can)                           ((p_can)->RTIF & LL_CAN_RTIF_AIF)

/**
  * @brief  Low layer CAN get Error Passive Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_ERR_PASSIVE_PENDING(p_can)                     ((p_can)->ERRINT & LL_CAN_ERRINT_EPIF)

/**
  * @brief  Low layer CAN get Arbitration Lost Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_ARBITRATION_PENDING(p_can)                     ((p_can)->ERRINT & LL_CAN_ERRINT_ALIF)

/**
  * @brief  Low layer CAN get Bus Error Interrupt Flag
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
#define LL_CAN_GET_BUS_ERR_PENDING(p_can)                         ((p_can)->ERRINT & LL_CAN_ERRINT_BEIF)

/**
  * @brief  Low layer CAN clear receive buffer
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_clear_receive_buffer(CAN_TypeDef *p_can) {
    p_can->RCTRL |= LL_CAN_RCTRL_RREL;
}

/**
  * @brief  Low layer CAN enable receive interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_RIE;
}

/**
  * @brief  Low layer CAN disable receive interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_RIE);
}

/**
  * @brief  Low layer CAN enable receive buffer Overrun Interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_overrun_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_ROIE;
}

/**
  * @brief  Low layer CAN disable receive buffer Overrun Interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_overrun_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_ROIE);
}

/**
  * @brief  Low layer CAN enable receive buffer full interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_full_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_RFIE;
}

/**
  * @brief  Low layer CAN disable receive buffer full interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_full_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_RFIE);
}

/**
  * @brief  Low layer CAN enable receive buffer almost full interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_almost_full_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_RAFIE;
}

/**
  * @brief  Low layer CAN disable receive buffer almost full interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_receive_buf_almost_full_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_RAFIE);
}

/**
  * @brief  Low layer CAN enable transmission primary interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_transmission_primary_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_TPIE;
}

/**
  * @brief  Low layer CAN disable transmission primary interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_transmission_primary_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_TPIE);
}

/**
  * @brief  Low layer CAN enable transmission secondary interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_transmission_secondary_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_TSIE;
}

/**
  * @brief  Low layer CAN disable transmission secondary interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_transmission_secondary_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_TSIE);
}

/**
  * @brief  Low layer CAN enable error interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_error_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->RTIE |= LL_CAN_RTIE_EIE;
}

/**
  * @brief  Low layer CAN disable error interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_error_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->RTIE &= ~(LL_CAN_RTIE_EIE);
}

/**
  * @brief  Low layer CAN enable error passive interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_error_passive_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->ERRINT |= LL_CAN_ERRINT_EPIE;
}

/**
  * @brief  Low layer CAN disable error passive interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_error_passive_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->ERRINT &= ~(LL_CAN_ERRINT_EPIE);
}

/**
  * @brief  Low layer CAN enable arbitration lost interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_arbitration_lost_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->ERRINT |= LL_CAN_ERRINT_ALIE;
}

/**
  * @brief  Low layer CAN disable arbitration lost interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_arbitration_lost_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->ERRINT &= ~(LL_CAN_ERRINT_ALIE);
}

/**
  * @brief  Low layer CAN enable bus error interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_bus_error_interrupt_enable(CAN_TypeDef *p_can) {
    p_can->ERRINT |= LL_CAN_ERRINT_BEIE;
}

/**
  * @brief  Low layer CAN disable bus error interrupt
  * @param  p_can: The register structure pointer of the CAN.
  * @retval None
  */
__STATIC_INLINE void ll_can_bus_error_interrupt_disable(CAN_TypeDef *p_can) {
    p_can->ERRINT &= ~(LL_CAN_ERRINT_BEIE);
}

/**
  * @brief  Low layer CAN set slow speed mode bit time param
  * @param  p_can      : The register structure pointer of the CAN.
  * @param  s_seg_1_val: The value of bit timing segment 1 (slow speed).
  *                      Actual TQ value = s_seg_1_val + 2;
  * @param  s_seg_2_val: The value of bit timing segment 2 (slow speed).
  *                      Actual TQ value = s_seg_2_val + 1;
  * @param  s_sjw_val  : The value of synchronization jump width (slow speed).
  *                      Actual TQ value = s_sjw_val + 1;
  * @retval None
  */
__STATIC_INLINE void ll_can_set_slow_bit_time(CAN_TypeDef *p_can, 
                                              u8 s_seg_1_val, 
                                              u8 s_seg_2_val,
                                              u8 s_sjw_val) {
    TX_ASSERT(s_seg_1_val < 64);
    TX_ASSERT(s_seg_2_val < 32);
    TX_ASSERT(s_sjw_val < 16);

    LL_CAN_REG_OPT(
        p_can->BITTIME0 = (p_can->BITTIME0 & LL_CAN_BITTIME_0_F_SJW(0x3)) |
                          LL_CAN_BITTIME_0_S_SEG_1(s_seg_1_val);

        p_can->BITTIME1 = (p_can->BITTIME1 & LL_CAN_BITTIME_1_F_SEG_2(0x7)) |
                          LL_CAN_BITTIME_1_S_SEG_2(s_seg_2_val);

        p_can->BITTIME2 = (p_can->BITTIME2 & LL_CAN_BITTIME_2_F_SEG_1(0xF)) |
                          LL_CAN_BITTIME_2_S_SJW(s_sjw_val);
    );
}

#if 0
///**
//  * @brief  Low layer CAN set fast speed mode bit time param
//  * @param  p_can      : The register structure pointer of the CAN.
//  * @param  f_seg_1_val: The value of bit timing segment 1 (fast speed).
//  *                      Actual TQ value = f_seg_1_val + 2;
//  * @param  f_seg_2_val: The value of bit timing segment 2 (fast speed).
//  *                      Actual TQ value = f_seg_2_val + 1;
//  * @param  f_sjw_val  : The value of synchronization jump width (fast speed).
//  *                      Actual TQ value = f_seg_2_val + 1;
//  * @retval None
//  */
//__STATIC_INLINE void ll_can_set_fast_bit_time(CAN_TypeDef *p_can, 
//                                              u8 f_seg_1_val, 
//                                              u8 f_seg_2_val,
//                                              u8 f_sjw_val) {
//    TX_ASSERT(f_seg_1_val < 16);
//    TX_ASSERT(f_seg_2_val < 8);
//    TX_ASSERT(f_sjw_val < 4);

//    LL_CAN_REG_OPT(
//        p_can->BITTIME0 = (p_can->BITTIME0 & LL_CAN_BITTIME_0_S_SEG_1(0x3F)) |
//                          LL_CAN_BITTIME_0_F_SJW(f_seg_1_val);

//        p_can->BITTIME1 = (p_can->BITTIME1 & LL_CAN_BITTIME_1_S_SEG_2(0x1F)) |
//                          LL_CAN_BITTIME_1_F_SEG_2(f_seg_2_val);

//        p_can->BITTIME2 = (p_can->BITTIME2 & LL_CAN_BITTIME_2_S_SJW(0xF)) |
//                          LL_CAN_BITTIME_2_F_SEG_1(f_sjw_val);
//    );
//}
#endif 
/**
  * @brief  Low layer CAN set slow speed mode baudrate
  * @param  p_can     : The register structure pointer of the CAN.
  * @param  s_baudrate: The value of baudrate(slow speed).
  * @retval None
  */
__STATIC_INLINE void ll_can_set_slow_baudrate(CAN_TypeDef *p_can, u8 s_baudrate) {
    LL_CAN_REG_OPT(
        p_can->S_PRESC = LL_CAN_S_PRESC(s_baudrate);
    );
}

#if 0
///**
//  * @brief  Low layer CAN set fast speed mode baudrate
//  * @param  p_can     : The register structure pointer of the CAN.
//  * @param  s_baudrate: The value of baudrate(fast speed).
//  * @retval None
//  */
//__STATIC_INLINE void ll_can_set_fast_baudrate(CAN_TypeDef *p_can, u8 f_baudrate) {
//    LL_CAN_REG_OPT(
//        p_can->F_PRESC = LL_CAN_F_PRESC(f_baudrate);
//    );
//}
#endif

/**
  * @brief  Low layer CAN enable receive address filter 
  * @param  p_can   : The register structure pointer of the CAN.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_can_rx_addr_filt_enable(CAN_TypeDef *p_can, u16 chn_bits) {
    p_can->ACF_EN_0 |= LL_CAN_ACF_EN_0(chn_bits);
    p_can->ACF_EN_1 |= LL_CAN_ACF_EN_1(chn_bits >> 8);
}

/**
  * @brief  Low layer CAN disable receive address filter 
  * @param  p_can   : The register structure pointer of the CAN.
  * @param  chn_bits: Set the value of chn to be shifted, you can call the BIT() 
  *                   function to assign values.  
  * @retval None
  */
__STATIC_INLINE void ll_can_rx_addr_filt_disable(CAN_TypeDef *p_can, u16 chn_bits) {
    p_can->ACF_EN_0 &= ~(LL_CAN_ACF_EN_0(chn_bits));
    p_can->ACF_EN_1 &= ~(LL_CAN_ACF_EN_1(chn_bits >> 8));
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

#endif //__TX_PHE_LL_CAN_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
