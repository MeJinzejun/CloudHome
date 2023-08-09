/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_qei.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the QEI LL firmware functions.
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
#ifndef __TX_PHE_LL_QEI_H
#define __TX_PHE_LL_QEI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
#include "arm_math.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup qei_interface_gr QEI Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup QEI_LL_Driver QEI LL Driver
  * @ingroup  qei_interface_gr
  * @brief Mainly the driver part of the QEI module, which includes \b QEI \b Register 
  * \b Constants, \b QEI \b Exported \b Constants, \b QEI \b Exported \b Struct, \b QEI
  * \b Data \b transfers \b functions, \b QEI \b Initialization \b and \b QEI \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup QEI_LL_Register_Constants QEI LL Register Constants
  * @ingroup  QEI_LL_Driver
  * @brief    QEI LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the QEI 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the QEI register, mainly for convenience. Understand the 
    configuration of the QEI.
    
@endverbatim
  *
  * @{
  */

/***** QEICON *****/
/*! index match value use.
 */
#define LL_QEI_IMV_USE_A(n)                         (((n)&0x1) << 19)
/*! index match value use.
 */
#define LL_QEI_IMV_USE_B(n)                         (((n)&0x1) << 18)
/*! TYPE_ENUM_LL_QEI_ROTATE_SPEED_MODE
 */
#define LL_QEI_ROTATE_SPEED_MODE(n)                 (((n)&0x1) << 17)
/*! ROTATE_SPEED_EN.
 */
#define LL_QEI_ROTATE_SPEED_EN                      (1UL << 16)
/*! index match value .
 */
#define LL_QEI_IMV_A(n)                             (((n)&0x1) << 15)
/*! index match value .
 */
#define LL_QEI_IMV_B(n)                             (((n)&0x1) << 14)
/*! phase select TYPE_ENUM_LL_QEI_PHASE_SEL
 */
#define LL_QEI_PH_X2(n)                             (((n)&0x1) << 13)
/*! idle enable  .
 */
#define LL_QEI_IDL_EN                               (1UL << 12)
/*! direct up(1) or dn(0) :TYPE_ENUM_LL_QEI_DIRECTION 
 */
#define LL_QEI_UPDN                                 (1UL << 11)
/*! QEI mode :TYPE_ENUM_LL_QEI_MODE
 */
#define LL_QEI_MODE(n)                              (((n)&0x7) << 8)
/*! index match value select: 1:register, 0:hardware 
 */
#define LL_QEI_IMVS(n)                              (((n)&0x1) << 7)
/*! direct up(1) or dn(0) :TYPE_ENUM_LL_QEI_DIRECTION 
 */
#define LL_QEI_TG(n)                                (((n)&0x1) << 5)
/*! timer pre-scale :TYPE_ENUM_LL_QEI_TMR_PRESCALE 
 */
#define LL_QEI_TCKPS(n)                             (((n)&0x3) << 3)
/*! PULSE_RST_CNT enable  .
 */
#define LL_QEI_PULSE_RST_CNT_EN                     (1UL << 2)
/*! timer clock source :TYPE_ENUM_LL_QEI_TMR_CLK_SEL 
 */
#define LL_QEI_TQCS(n)                              (((n)&0x1) << 1)
/*! direct up or dn source :TYPE_ENUM_LL_QEI_UD_SRC_SEL 
 */
#define LL_QEI_UDSRC(n)                             (((n)&0x1) << 0)


/***** QEIO *****/
/*! index value .
 */
#define LL_QEI_IDX_VAL                              (1UL << 8)
/*! pos dir state output enable 
 */
#define LL_QEI_PSD_OUT_EN                           (1UL << 4)
/*! qei source A B swap  enable
 */
#define LL_QEI_SWAP_A_B_EN                          (1UL << 3)
/*! index SRC reverse enable .
 */
#define LL_QEI_IDX_INV_EN                           (1UL << 2)
/*! QEB SRC reverse enable .
 */
#define LL_QEI_QEB_INV_EN                           (1UL << 1)
/*! QEA SRC reverse enable .
 */
#define LL_QEI_QEA_INV_EN                           (1UL << 0)


/***** DFLTCON *****/
/*! digital filter enable 
 */
#define LL_QEI_DFLT_EN                              (1UL << 7)
/*! digital filter clock sel : TYPE_ENUM_LL_QEI_DFLT_PRESCALE 
 */
#define LL_QEI_DFLT_CLK(n)                          (((n)&0x3) << 4)


/***** QEIFLAG *****/
/*! dir change flag 
 */
#define LL_QEI_UPDN_INV_FLAG                        (1UL << 10)
/*! SPEED_CNT error flag 
 */
#define LL_QEI_SPEED_CNT_ERR                        (1UL << 9)
/*! SPEED_CNT overflow flag 
 */
#define LL_QEI_SPEED_CNT_OV                         (1UL << 8)
/*! timer error flag 
 */
#define LL_QEI_SPEED_TMR_ERR                        (1UL << 7)
/*! timer overflow flag 
 */
#define LL_QEI_SPEED_TMR_OV                         (1UL << 6)
/*! timer gate failing flag 
 */
#define LL_QEI_TG_FAILING                           (1UL << 5)
/*! poscnt==maxcnt flag 
 */
#define LL_QEI_POS_EQ_MAX                           (1UL << 4)
/*! COUNTER error flag 
 */
#define LL_QEI_CNT_ERR                              (1UL << 3)
/*! IDX_RST_CNT  flag 
 */
#define LL_QEI_IDX_RST_CNT                          (1UL << 2)
/*! poscnt==maxcnt _RST_CNT flag 
 */
#define LL_QEI_POS_EQ_MAX_RST_CNT                   (1UL << 1)
/*! phase error flag 
 */
#define LL_QEI_PHASE_ERR                            (1UL << 0)


/***** QEICLR *****/
/*! dir change flag clear 
 */
#define LL_QEI_UPDN_INV_FLAG_CLR                    (1UL << 10)
/*! SPEED_CNT error flag clear 
 */
#define LL_QEI_SPEED_CNT_ERR_CLR                    (1UL << 9)
/*! SPEED_CNT overflow flag clear 
 */
#define LL_QEI_SPEED_CNT_OV_CLR                     (1UL << 8)
/*! timer  error flag clear 
 */
#define LL_QEI_SPEED_TMR_ERR_CLR                    (1UL << 7)
/*! timer overflow flag clear 
 */
#define LL_QEI_SPEED_TMR_OV_CLR                     (1UL << 6)
/*! timer gate failing flag clear 
 */
#define LL_QEI_TG_FAILING_CLR                       (1UL << 5)
/*! poscnt==maxcnt flag clear 
 */
#define LL_QEI_POS_EQ_MAX_CLR                       (1UL << 4)
/*! COUNTER error flag clear  
 */
#define LL_QEI_CNT_ERR_CLR                          (1UL << 3)
/*! IDX_RST_CNT  flag clear 
 */
#define LL_QEI_IDX_RST_CNT_CLR                      (1UL << 2)
/*! poscnt==maxcnt _RST_CNT flag clear 
 */
#define LL_QEI_POS_EQ_MAX_RST_CNT_CLR               (1UL << 1)
/*! phase error flag clear 
 */
#define LL_QEI_PHASE_ERR_CLR                        (1UL << 0)


/***** QEIE *****/
/*! dir change flag interrupt 
 */
#define LL_QEI_UPDN_INV_FLAG_INT_EN                 (1UL << 10)
/*! SPEED_CNT error flag interrupt 
 */
#define LL_QEI_SPEED_CNT_ERR_INT_EN                 (1UL << 9)
/*! SPEED_CNT overflow flag interrupt 
 */
#define LL_QEI_SPEED_CNT_OV_INT_EN                  (1UL << 8)
/*! timer  error flag interrupt 
 */
#define LL_QEI_SPEED_TMR_ERR_INT_EN                 (1UL << 7)
/*! timer overflow flag interrupt 
 */
#define LL_QEI_SPEED_TMR_OV_INT_EN                  (1UL << 6)
/*! timer gate failing flag interrupt 
 */
#define LL_QEI_TG_FAILING_INT_EN                    (1UL << 5)
/*! poscnt==maxcnt flag interrupt 
 */
#define LL_QEI_POS_EQ_MAX_INT_EN                    (1UL << 4)
/*! COUNTER error flag interrupt 
 */
#define LL_QEI_CNT_ERR_INT_EN                       (1UL << 3)
/*! IDX_RST_CNT  flag interrupt 
 */
#define LL_QEI_IDX_RST_CNT_INT_EN                   (1UL << 2)
/*! poscnt==maxcnt _RST_CNT flag interrupt 
 */
#define LL_QEI_POS_EQ_MAX_RST_CNT_INT_EN            (1UL << 1)
/*! phase error flag interrupt 
 */
#define LL_QEI_PHASE_ERR_INT_EN                     (1UL << 0)


/***** POSCNT *****/
/***** MAXCNT *****/
/***** QEI_TIMER_PERIOD *****/
/***** QEI_TIMER_CNT *****/
/***** QEI_TIMER_CNT_LATCH *****/
/***** QEI_ROTATE_PERIOD *****/
/***** QEI_ROTATE_CNT *****/
/***** QEI_TIMER_ROTATE_LATCH *****/
/***** POSCNT_LATCH *****/

/**
  * @}
  */

/** @defgroup QEI_LL_Exported_Constants QEI LL Exported Constants
  * @ingroup  QEI_LL_Driver
  * @brief    QEI LL external constant definition
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
  * @brief QEI speed calculate mode enum
  */
typedef enum {
    /*! M MODE 
     */
    LL_QEI_ROTATE_SPEED_MODE_M = 0,
    /*! T MODE 
     */
    LL_QEI_ROTATE_SPEED_MODE_T = 1,
} TYPE_ENUM_LL_QEI_ROTATE_SPEED_MODE;

/**
  * @brief QEI phase select enum
  */
typedef enum {
    /*! QEA 
     */
    LL_QEI_PHASE_QEA = 0,
    /*! QEB 
     */
    LL_QEI_PHASE_QEB = 1,
} TYPE_ENUM_LL_QEI_PHASE_SEL;

/**
  * @brief QEI counter direction enum
  */
typedef enum {                      
    /*! DIRECTION_UP 
     */
    LL_QEI_DIRECTION_UP   = 1,
    /*! DIRECTION_DOWN 
     */
    LL_QEI_DIRECTION_DOWN = 0,
} TYPE_ENUM_LL_QEI_DIRECTION;

/**
  * @brief QEI work mode enum
  */
typedef enum {
    /*! mode none 
     */
    LL_QEI_MODE_DISABLE                      = 0,
    /*! mode timer 
     */
//    LL_QEI_MODE_TMR  = 1,
    /*! mode QE X2 , pos counter reset when 1'st index and match count 
     */
    LL_QEI_MODE_QEX2_RST_BY_1STIDX_AND_MATCH = 2,
    /*! mode QE X4 , pos counter reset when 1'st index and match count 
     */
    LL_QEI_MODE_QEX4_RST_BY_1STIDX_AND_MATCH = 3,
    /*! mode QE X2 , pos counter reset when index  
     */
    LL_QEI_MODE_QEX2_RST_BY_IDX              = 4,
    /*! mode QE X2 , pos counter reset when match count 
     */
    LL_QEI_MODE_QEX2_RST_BY_MATCH            = 5,
    /*! mode QE X4 , pos counter reset when index
     */
    LL_QEI_MODE_QEX4_RST_BY_IDX              = 6,
    /*! mode QE X4 , pos counter reset when match count 
     */
    LL_QEI_MODE_QEX4_RST_BY_MATCH            = 7,

} TYPE_ENUM_LL_QEI_MODE;
    
/**
  * @brief QEI timer prescale enum
  */
typedef enum {
    /*! timer pre-scale : 1:1 
     */
    LL_QEI_TMR_PRESCALE_0   = 0,
    /*! timer pre-scale : 1:8
     */
    LL_QEI_TMR_PRESCALE_8   = 1,
    /*! timer pre-scale : 1:64 
     */
    LL_QEI_TMR_PRESCALE_64  = 2,
    /*! timer pre-scale : 1:256 
     */
    LL_QEI_TMR_PRESCALE_256 = 3,

} TYPE_ENUM_LL_QEI_TMR_PRESCALE;

/**
  * @brief QEI timer clock enum
  */
typedef enum {
    /*! LL_QEI_TMR_CLK_SEL QEA 
     */
    LL_QEI_TMR_CLK_QEA      = 1,
    /*! LL_QEI_TMR_CLK_SEL internal clock 
     */
    LL_QEI_TMR_CLK_INTERNAL = 0,
} TYPE_ENUM_LL_QEI_TMR_CLK_SEL;

/**
  * @brief QEI up&down source enum
  */
typedef enum {
    /*! LL_QEI_UD_SRC QEB 
     */
    LL_QEI_UD_SRC_QEB    = 1,
    /*! LL_QEI_UD_SRC internal reg 
     */
    LL_QEI_UD_SRC_UD_REG = 0,
} TYPE_ENUM_LL_QEI_UD_SRC_SEL;

/**
  * @brief QEI digital filter prescale enum
  */
typedef enum {
    /*! QEA/QEB/INDX pre-scale : 1:1 
     */
    LL_QEI_DFLT_PRESCALE_0   = 0,
    /*! QEA/QEB/INDX pre-scale : 1:2 
     */
    LL_QEI_DFLT_PRESCALE_2   = 1,
    /*! QEA/QEB/INDX pre-scale : 1:4 
     */
    LL_QEI_DFLT_PRESCALE_4   = 2,
    /*! QEA/QEB/INDX pre-scale : 1:16 
     */
    LL_QEI_DFLT_PRESCALE_16  = 3,
    /*! QEA/QEB/INDX pre-scale : 1:32 
     */
    LL_QEI_DFLT_PRESCALE_32  = 4,
    /*! QEA/QEB/INDX pre-scale : 1:64 
     */
    LL_QEI_DFLT_PRESCALE_64  = 5,
    /*! QEA/QEB/INDX pre-scale : 1:128 
     */
    LL_QEI_DFLT_PRESCALE_128 = 6,
    /*! QEA/QEB/INDX pre-scale : 1:256 
     */
    LL_QEI_DFLT_PRESCALE_256 = 7,

} TYPE_ENUM_LL_QEI_DFLT_PRESCALE;

/**
  * @brief QEI polarity enum
  */
typedef enum {
    /*! LL_QEI_POLE invert 
     */
    LL_QEI_POLE_INV    = 1,
    /*! LL_QEI_POLE_NORMAL  
     */
    LL_QEI_POLE_NORMAL = 0,
} TYPE_ENUM_LL_QEI_POLE_SEL;

/**
  * @brief QEI index match value enum
  */
typedef enum {
    /*! LL_QEI index match value config by IMV reg
     */
    LL_QEI_IMV_REG      = 1,
    /*! LL_QEI index match value config by hardware
     */
    LL_QEI_IMV_HARDWARE = 0,
} TYPE_ENUM_LL_QEI_IMV_SEL;


/***** LL API AND DRIVER API *****/



/**
  * @}
  */

/** @defgroup QEI_LL_Exported_Struct QEI LL Exported Struct
  * @ingroup  QEI_LL_Driver
  * @brief    QEI LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the QEI registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_qei_init. Function, you can configure the QEI module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief configuration structure for low layer QEI interrupt
  */ 
typedef struct __ll_qei_irq_cfg {
    /*! Interrupt enable control.
     *  @note false represents disable interrupt, true represents enable interrupt.
     */
    /*! dir change flag interrupt
     */
    u16 updn_inv_intr_en               : 1,
    /*! SPEED_CNT error flag interrupt
     */
        speed_cnt_err_intr_en          : 1,
    /*! SPEED_CNT overflow flag interrupt
     */
        speed_cnt_ov_intr_en           : 1,
    /*! timer  error flag interrupt
     */
        speed_tmr_err_intr_en          : 1,
    /*! timer overflow flag interrupt
     */
        speed_tmr_ov_intr_en           : 1,
    /*! timer gate failing flag interrupt
     */
        speed_tmr_gate_failing_intr_en : 1,
    /*! poscnt==maxcnt flag interrupt
     */
        speed_pos_eq_max_intr_en       : 1,
    /*! COUNTER error flag interrupt
     */
        speed_pos_cnt_err_intr_en      : 1,
    /*! IDX_RST_CNT  flag interrupt
     */
        speed_idx_rst_cnt_intr_en      : 1,
    /*! poscnt==maxcnt _RST_CNT flag interrupt
     */
        speed_max_eq_rst_cnt_intr_en   : 1,
    /*! phase error flag interrupt
     */
        speed_phase_err_intr_en        : 1,
    /*! reserved
     */
        reserverd                      : 6;
    /*! digital filter enable
     */
} TYPE_LL_QEI_IRQ_CFG;

/**
  * @brief configuration structure for low layer QEI module struct
  */ 
typedef struct __ll_qei_cfg {
    /*! qei mode, timer mode is not support in current sdk
     */    
    TYPE_ENUM_LL_QEI_MODE               mode;
    /*! qei phase select, valid when QEIx2 mode
     */    
    TYPE_ENUM_LL_QEI_PHASE_SEL          phase;
    /*! digital filter enable
     */
    bool                                dflt_en;
    /*! digital filter pre-scale
     */
    TYPE_ENUM_LL_QEI_DFLT_PRESCALE      dflt_pre_scale;
    /*! max count = 4/2(mode) * no_lines_encoder
     */
    u16                                 max_count;
    /*! QEA Pole select
     */
    TYPE_ENUM_LL_QEI_POLE_SEL           qea_pole;
    /*! QEB Pole select
     */
    TYPE_ENUM_LL_QEI_POLE_SEL           qeb_pole;
    /*! INDEX Pole select
     */
    TYPE_ENUM_LL_QEI_POLE_SEL           idx_pole;
    /*! swap QEA & QEB
     */
    bool                                qea_qeb_swap_en;
    /*! updn direction output to IO enable
     */
    bool                                updn_dir_to_io_en;
    /*! INDEX Match Value select, when index pulsed && (qea == imv_qea) && (qeb == imv_qeb), the 
     *  pos counter will be reset( reg mode )
     */
    TYPE_ENUM_LL_QEI_IMV_SEL            imv_sel;
    /*! INDEX Match Value reg : 0/1
     */
    u8                                  imv_qea;
    /*! INDEX Match Value reg : 0/1
     */
    u8                                  imv_qeb;
    /*! speed calculate enable
     */
    bool                                speed_calc_en;
    /*! speed calculate mode 
     */
    TYPE_ENUM_LL_QEI_ROTATE_SPEED_MODE  speed_calc_mode;
    /*! speed calculate M-mode set : time, related to speed_calc_mode
     */
    u32                                 speed_calc_time;
    /*! speed calculate T-mode set : counter, related to speed_calc_mode
     */
    u32                                 speed_calc_counter;
} TYPE_LL_QEI_CFG;

/**
  * @brief QEI low layer Initialization structure
  */
typedef struct __ll_qei_init {
    u8 reserved;
} TYPE_LL_QEI_INIT;


/**
  * @}
  */

/** @defgroup QEI_LL_Interrupt QEI LL Interrupt Handle function
  * @brief   QEI LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the QEI  
    Interrupt Handle function.

    how to use?

    The QEI interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the QEI in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup QEI_LL_Inti_Cfg QEI LL Initialization And Configuration
  * @brief    QEI LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the QEI data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/**
  * @brief  Low layer QEI module initialization
  * @param  p_qei : The structure pointer of the QEI is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_QEI_INIT)
  * @retval None
  */
void ll_qei_init(QEI_TypeDef *p_qei, TYPE_LL_QEI_INIT *p_init);

/**
  * @brief  Low layer QEI module detele initialization
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @retval None
  */
void ll_qei_deinit(QEI_TypeDef *p_qei);

/**
  * @brief  Low layer QEI module configuration
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_QEI_IRQ_CFG)
  * @retval None
  */
void ll_qei_irq_config(QEI_TypeDef *p_qei, TYPE_LL_QEI_IRQ_CFG *p_cfg);

/**
  * @brief  Low layer QEI module configuration
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_QEI_CFG)
  * @retval None
  */
void ll_qei_config(QEI_TypeDef *p_qei, TYPE_LL_QEI_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup QEI_LL_Data_Transfers QEI LL Data transfers functions
  * @brief    QEI LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the QEI data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  QEI module start function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval None
  */
void ll_qei_start(QEI_TypeDef *p_qei);

/**
  * @brief  QEI module stop function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval None
  */
void ll_qei_stop(QEI_TypeDef *p_qei);

/**
  * @brief  QEI get Motor Mechanical Angle function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval angle [0, 1] for [0, 360] theta
  */
float32_t ll_qei_get_mech_angle(QEI_TypeDef *p_qei);

/**
  * @brief  QEI get Motor Mechanical Angle function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval angle [0, 32768] for [0, 360] theta
  */
s16 ll_qei_get_mech_angle_q15(QEI_TypeDef *p_qei);

/**
  * @brief QEI get Motor speed calc result function
  * @param p_qei   : Select the initialized QEI group pointer
  * @param pos_cnt : qea/qeb counters
  * @param time_cnt: apb1_clk counters
  * @retval true : ok, false : fail
  */
bool ll_qei_get_speed_result(QEI_TypeDef *p_qei, u32 *pos_cnt, u32 *time_cnt);

/**
  * @brief  ll_qei_updn_inv_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_updn_inv_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_UPDN_INV_FLAG_INT_EN;
}

/**
  * @brief  ll_qei_updn_inv_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_updn_inv_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_UPDN_INV_FLAG_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_UPDN_INV_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_UPDN_INV_INTERRUPT_ENABLE(p_qei)       ((p_qei)->QEIE & LL_QEI_UPDN_INV_FLAG_INT_EN)

/**
  * @brief  QEI get direction inverting pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_UPDN_INV_PENDING(p_qei)                  ((p_qei)->QEIFLAG & LL_QEI_UPDN_INV_FLAG)

/**
  * @brief  QEI clear direction inverting pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_updn_inv_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_UPDN_INV_FLAG_CLR;
}

/**
  * @brief  ll_qei_speed_cnt_err_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_cnt_err_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_SPEED_CNT_ERR_INT_EN;
}

/**
  * @brief  ll_qei_speed_cnt_err_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_cnt_err_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_SPEED_CNT_ERR_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_SPEED_CNT_ERR_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_SPEED_CNT_ERR_INTERRUPT_ENABLE(p_qei)  ((p_qei)->QEIE & LL_QEI_SPEED_CNT_ERR_INT_EN)

/**
  * @brief  QEI SPEED_CNT error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_SPEED_CNT_ERR_PENDING(p_qei)             ((p_qei)->QEIFLAG & LL_QEI_SPEED_CNT_ERR)

/**
  * @brief  QEI clear SPEED_CNT error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_speed_cnt_err_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_SPEED_CNT_ERR_CLR;
}

/**
  * @brief  ll_qei_speed_cnt_ov_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_cnt_ov_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_SPEED_CNT_OV_INT_EN;
}

/**
  * @brief  ll_qei_speed_cnt_ov_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_cnt_ov_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_SPEED_CNT_OV_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_SPEED_CNT_OV_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_SPEED_CNT_OV_INTERRUPT_ENABLE(p_qei)   ((p_qei)->QEIE & LL_QEI_SPEED_CNT_OV_INT_EN)

/**
  * @brief  QEI SPEED_CNT overflow pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_SPEED_CNT_OV_PENDING(p_qei)              ((p_qei)->QEIFLAG & LL_QEI_SPEED_CNT_OV)

/**
  * @brief  QEI clear SPEED_CNT overflow pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_speed_cnt_ov_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_SPEED_CNT_OV_CLR;
}

/**
  * @brief  ll_qei_speed_tmr_err_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_tmr_err_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_SPEED_TMR_ERR_INT_EN;
}

/**
  * @brief  ll_qei_speed_tmr_err_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_tmr_err_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_SPEED_TMR_ERR_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_TMR_ERR_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_TMR_ERR_INTERRUPT_ENABLE(p_qei)    ((p_qei)->QEIE & LL_QEI_SPEED_TMR_ERR_INT_EN)

/**
  * @brief  QEI timer  error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_SPEED_GET_TMR_ERR_PENDING(p_qei)         ((p_qei)->QEIFLAG & LL_QEI_SPEED_TMR_ERR)

/**
  * @brief  QEI clear timer  error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_speed_tmr_err_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_SPEED_TMR_ERR_CLR;
}

/**
  * @brief  ll_qei_speed_tmr_ov_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_tmr_ov_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_SPEED_TMR_OV_INT_EN;
}

/**
  * @brief  ll_qei_speed_tmr_ov_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_speed_tmr_ov_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_SPEED_TMR_OV_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_SPEED_TMR_OV_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_SPEED_TMR_OV_INTERRUPT_ENABLE(p_qei)   ((p_qei)->QEIE & LL_QEI_SPEED_TMR_OV_INT_EN)

/**
  * @brief  QEI timer overflow pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_SPEED_TMR_OV_PENDING(p_qei)              ((p_qei)->QEIFLAG & LL_QEI_SPEED_TMR_OV)

/**
  * @brief  QEI clear timer overflow pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_speed_tmr_ov_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_SPEED_TMR_OV_CLR;
}

/**
  * @brief  ll_qei_tg_failing_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_tg_failing_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_TG_FAILING_INT_EN;
}

/**
  * @brief  ll_qei_tg_failing_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_tg_failing_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_TG_FAILING_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_TG_FAILING_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_TG_FAILING_INTERRUPT_ENABLE(p_qei) ((p_qei)->QEIE & LL_QEI_TG_FAILING_INT_EN)

/**
  * @brief  QEI  timer gate failing pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_TG_FAILING_PENDING(p_qei)            ((p_qei)->QEIFLAG & LL_QEI_TG_FAILING)

/**
  * @brief  QEI clear timer gate failing pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_tg_failing_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_TG_FAILING_CLR;
}

/**
  * @brief  ll_qei_pos_eq_max_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_pos_eq_max_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_POS_EQ_MAX_INT_EN;
}

/**
  * @brief  ll_qei_pos_eq_max_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_pos_eq_max_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_POS_EQ_MAX_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_POS_EQ_MAX_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_POS_EQ_MAX_INTERRUPT_ENABLE(p_qei) ((p_qei)->QEIE & LL_QEI_POS_EQ_MAX_INT_EN)

/**
  * @brief  QEI  poscnt==maxcnt pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_POS_EQ_MAX_PENDING(p_qei)            ((p_qei)->QEIFLAG & LL_QEI_POS_EQ_MAX)

/**
  * @brief  QEI clear poscnt==maxcnt pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_pos_eq_max_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_POS_EQ_MAX_CLR;
}

/**
  * @brief  ll_qei_cnt_err_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_cnt_err_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_CNT_ERR_INT_EN;
}

/**
  * @brief  ll_qei_cnt_err_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_cnt_err_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_CNT_ERR_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_QEI_CNT_ERR_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_QEI_CNT_ERR_INTERRUPT_ENABLE(p_qei)    ((p_qei)->QEIE & LL_QEI_CNT_ERR_INT_EN)

/**
  * @brief  QEI  COUNTER error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_QEI_CNT_ERR_PENDING(p_qei)               ((p_qei)->QEIFLAG & LL_QEI_CNT_ERR)

/**
  * @brief  QEI clear COUNTER error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_qei_cnt_err_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_CNT_ERR_CLR;
}

/**
  * @brief  ll_qei_idx_rst_cnt_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_idx_rst_cnt_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_IDX_RST_CNT_INT_EN;
}

/**
  * @brief  ll_qei_idx_rst_cnt_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_idx_rst_cnt_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_IDX_RST_CNT_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_IDX_RST_CNT_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_IDX_RST_CNT_INTERRUPT_ENABLE(p_qei)    ((p_qei)->QEIE & LL_QEI_IDX_RST_CNT_INT_EN)

/**
  * @brief  QEI  IDX_RST_CNT pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_IDX_RST_CNT_PENDING(p_qei)               ((p_qei)->QEIFLAG & LL_QEI_IDX_RST_CNT)

/**
  * @brief  QEI clear IDX_RST_CNT pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_idx_rst_cnt_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_IDX_RST_CNT_CLR;
}

/**
  * @brief  ll_qei_pos_eq_max_rst_cnt_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_pos_eq_max_rst_cnt_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_POS_EQ_MAX_RST_CNT_INT_EN;
}

/**
  * @brief  ll_qei_pos_eq_max_rst_cnt_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_pos_eq_max_rst_cnt_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_POS_EQ_MAX_RST_CNT_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_POS_EQ_MAX_RST_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_POS_EQ_MAX_RST_INTERRUPT_ENABLE(p_qei) ((p_qei)->QEIE & LL_QEI_POS_EQ_MAX_RST_CNT_INT_EN)

/**
  * @brief  QEI  poscnt==maxcnt _RST_CNT pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_POS_EQ_MAX_RST_CNT_PENDING(p_qei)        ((p_qei)->QEIFLAG & LL_QEI_POS_EQ_MAX_RST_CNT)

/**
  * @brief  QEI clear poscnt==maxcnt _RST_CNT pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_pos_eq_max_rst_cnt_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_POS_EQ_MAX_RST_CNT_CLR;
}

/**
  * @brief  ll_qei_phase_err_interrupt_enable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_phase_err_interrupt_enable(QEI_TypeDef *p_qei) {
    p_qei->QEIE |= LL_QEI_PHASE_ERR_INT_EN;
}

/**
  * @brief  ll_qei_phase_err_interrupt_disable
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_phase_err_interrupt_disable(QEI_TypeDef *p_qei) {
    p_qei->QEIE &= ~LL_QEI_PHASE_ERR_INT_EN;
}

/**
  * @brief  LL_QEI_CHECK_PHASE_ERR_INTERRUPT_ENABLE
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval interrupt enable
  */
#define LL_QEI_CHECK_PHASE_ERR_INTERRUPT_ENABLE(p_qei) ((p_qei)->QEIE & LL_QEI_PHASE_ERR_INT_EN)

/**
  * @brief  QEI  phase error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_PHASE_ERR_PENDING(p_qei)           ((p_qei)->QEIFLAG & LL_QEI_PHASE_ERR)

/**
  * @brief  QEI clear phase error pending
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval none
  */
__STATIC_INLINE void ll_qei_clear_phase_err_pending(QEI_TypeDef *p_qei) {
    p_qei->QEICLR = LL_QEI_PHASE_ERR_CLR;
}

/**
  * @brief  QEI get idx state
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_IDX_STATE(p_qei)                   (((p_qei)->QEIO & LL_QEI_IDX_VAL))

/**
  * @brief  QEI get pos cnt
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_POS_CNT(p_qei)                     (((p_qei)->POSCNT))

/**
  * @brief  QEI get pos cnt LATCH
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_POS_CNT_LATCH(p_qei)               (((p_qei)->POSCNT_LATCH))

/**
  * @brief  QEI get MAX cnt
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_MAX_CNT(p_qei)                     (((p_qei)->MAXCNT))

/**
  * @brief  QEI get up & down direction 
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_DIR(p_qei)                         (((p_qei)->QEICON) & LL_QEI_UPDN)

/**
  * @brief  QEI get QEA value when index pulse reset pos_cnt
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_IDX_RST_QEA_POL(p_qei)             (((p_qei)->QEICON) & LL_QEI_IMV_USE_A)

/**
  * @brief  QEI get QEB value when index pulse reset pos_cnt
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval pending value
  */
#define LL_QEI_GET_IDX_RST_QEB_POL(p_qei)             (((p_qei)->QEICON) & LL_QEI_IMV_USE_B)

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

#endif //__TX_PHE_LL_QEI_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
