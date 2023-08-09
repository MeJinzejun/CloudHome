/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_spwm.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SPWM LL firmware functions.
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
#ifndef __TX_PHE_LL_SPWM_H
#define __TX_PHE_LL_SPWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup spwm_interface_gr SPWM Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup SPWM_LL_Driver SPWM LL Driver
  * @ingroup  spwm_interface_gr
  * @brief Mainly the driver part of the SPWM module, which includes \b SPWM \b Register 
  * \b Constants, \b SPWM \b Exported \b Constants, \b SPWM \b Exported \b Struct, \b SPWM
  * \b Data \b transfers \b functions, \b SPWM \b Initialization \b and \b SPWM \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup SPWM_LL_Register_Constants SPWM LL Register Constants
  * @ingroup  SPWM_LL_Driver
  * @brief    SPWM LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the SPWM 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the SPWM register, mainly for convenience. Understand the 
    configuration of the SPWM.
    
@endverbatim
  *
  * @{
  */
  
/***** SPWM_CON Register *****/
/*! auto-scale enable
 */
#define LL_SPWM_AUTO_SCALE_EN                      (1UL << 26)
/*! systerm error(osc stop or lvd) protect enable
 */
#define LL_SPWM_SYSERR_PROTECT_EN                  (1UL << 25)
/*! PWM configure SFR write enable. configure SFR Include:
 *  SPWM_CON[14:0]
 *  SPWM_PERIOD
 *  FAULT_INFO
 *  FAULT_INVERT
 *  1: CPU can write configure SFR.
 *  0: CPU can NOT write configure SFR .
 */
#define LL_SPWM_CFG_EN                             (1UL << 24)
/*! unit 64 clk
 */
#define LL_SPWM_FAULT_DEBOUNCE(n)                  (((n)&0xF) << 20)
/*! Clear data error pending
 */
#define LL_SPWM_CLR_DATAERR_PD_ALL                 (1UL << 19)
/*! Clear hardware protect pending
 */
#define LL_SPWM_CLR_HWP_PD                         (1UL << 17)
/*! Clear PWM enable
 */
#define LL_SPWM_CLR_PWMEN                          (1UL << 16)
/*! Data error pending
 */
#define LL_SPWM_DATAERR_PD_ALL                     (1UL << 15)
/*! Data error protect disable
 */
#define LL_SPWM_DATAERR_DIS_ALL                    (1UL << 14)
/*! PWM work : 1
 */
#define LL_SPWM_WORK_FLAG                          (1UL << 13)
/*! Data error interrupt enable
 */
#define LL_SPWM_DATAERR_INT_EN_ALL                 (1UL << 12)
/*! Hardware protect pending
 */
#define LL_SPWM_HWP_PD                             (1UL << 11)
/*! Hardware protect interrupt enable
 */
#define LL_SPWM_HWP_INT_EN                         (1UL << 10)
/*! Load data interrupt enable
 */
#define LL_SPWM_LOAD_INT_EN_ALL                    (1UL << 9)
/*! Hardware protect Disable
 */
#define LL_SPWM_HWP_DIS                            (1UL << 8)
/*! PWM Invert  
 *  1: Invert the PWM. Output 0 means IGBT ON.  
 *  0: Not invert the PWM. Output 1 means IGBT ON  
 */
#define LL_SPWM_PWM_INVERT                         (1UL << 7)
/*! Use PWM  
 *  1: Use this PWM  
 *  0: Not Use this PWM. The output always 0(PWM_INVERT==0) or 1(PWM_INVERT==1)  
 */
#define LL_SPWM_SEL(n)                             (((n)&0x7) << 1)
/*! SPWM enbale
 */
#define LL_SPWM_PWM_EN                             (1UL << 0)


/***** SPWM_PERIOD Register *****/
/*! PERIOD unit : 1 sys clk
 */
#define LL_SPWM_PWM_PERIOD(n)                      (((n)&0xFFFF) << 0) 


/***** SPWM_PERIOD1 Register *****/
 /*! MIN_TIME unit : 2 sys clk
  */
#define LL_SPWM_MIN_TIME(n)                        (((n)&0xFFF) << 24) 
/*! DEAD_TIME unit : 2 sys clk, must > 0
 */
#define LL_SPWM_DEAD_TIME(n)                       (((n)&0xFFF) << 16)


/***** SPWM_MATCH Register *****/
/*! SPWM match pending clear
 */
#define LL_SPWM_MATCH_PENDING_CLR                  (1UL << 20)
/*! SPWM match pending 
 */
#define LL_SPWM_MATCH_PENDING                      (1UL << 19)
/*! SPWM match interrupt enable
 */
#define LL_SPWM_MATCH_INT_EN                       (1UL << 18)
/*! Select which PWM to compare with MATCH_VALUE  
 *  0:Disable compare  
 *  1:PWM0  
 *  2:PWM1  
 *  3:PWM2  
 */
#define LL_SPWM_MATCH_SEL(n)                       (((n)&0x03) << 16)
/*! If PWM counter equal this value, the MATCH_PENDING will be assert.  
    This value range should be 0~ (PWM_PERIOD-1) 
*/
#define LL_SPWM_MATCH_VALUE(n)                     (((n)&0xFFFF) << 0) 


/***** SPWM_ADCC Register *****/
/*! ADC control enable. Select which ADC to control. adc:0~13  
 *  ADC_CONTROL_MODE select which mode to use.  
 *  1: Enable. If MATCH_PENDING change to 1, Start ADC convert at the same time  
 *  0: Disable  
 */
#define LL_SPWM_CONTROL_ADC_EN(n)                   (((n)&0x3FFF) << 0)


/***** SPWM_CNT0/1/2 Register *****/
/*! Two level mode  
 *  1: Two level mode  
 *  0: Three level mode  
 */
#define LL_SPWM_TWO_LEVEL                           (1UL << 22)
/*! Dead balance mode enable. This mode only use at PWM_CARRIER_MODE==0.  
 *  If PWM_CARRIER_MODE==1, this bit unused.  
 *  Recommend set this bit to 1.  
 *  1: Enable. Balance Dead Time whatever data is positive or negative, But it may made pulse time 
 *     small than minimum pulse(mean MIN_PULSE). 
 *  0: Disable  
 */
#define LL_SPWM_DEAD_BALANCE_MODE                   (1UL << 21)
/*! spwm clear load data pending
 */
#define LL_SPWM_CLR_LOAD_PD                         (1UL << 20)
/*! spwm load data pending
 */
#define LL_SPWM_LOAD_PD                             (1UL << 19)
/*! spwm load data interrupt enable
 */
#define LL_SPWM_LOAD_INT_EN                         (1UL << 18)
/*! PWM data error detect disable  
 *  1: Disable PWM data error detect  
 *  0: Enable PWM data error detect  
 */
#define LL_SPWM_DATAERR_DIS                         (1UL << 17)
/*! PWM triangle carrier mode  
 *  0: Mode0. Positive and negative number have same phase. Data will load to the register while 
 *     triangle carrier counter==0.  
 *  1: Mode1. Positive and negative number have different phase. Recommend use this mode. Data will 
 *  load to the register while triangle carrier counter==MAX.  
 */
#define LL_PWM_CARRIER_MODE                         (1UL << 16)
/*! PWM counter initial direction 
 *  0: PWM counter will increase if start
 *  1: PWM counter will decrease if start
 */
#define LL_PWM_CNT_INI_DIR                          (1UL << 15)
/*! PWM counter initial value 
 */
#define LL_PWM_CNT_INI_VAL(n)                       (((n)&0x7FFF) << 0)


/***** FAULT_INFO Register *****/
/*! Each Fault disable.  
 *  1: disable fault detect  
 *  0: enable fault detect  
 */
#define LL_SPWM_FAULT_DIS(n)                        (((n)&0xFFFF) << 0)
/*! There are 16 fault sources which are mapping to {PB28~PB31,PC0~PC11}.If the fault source is valid,
 *  this pending will set to one
 */
#define LL_SPWM_FAULT_PD(n)                         (((n)&0xFFFF) << 16)


/***** FAULT_INVERT Register *****/
/*! Invert the Fault source  
 *  1: The fault source is valid when it's 0.  
 *  0: The fault source is valid when it's 1.  
 */
#define LL_SPWM_FAULT_INVERT(n)                     (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup SPWM_LL_Exported_Constants SPWM LL Exported Constants
  * @ingroup  SPWM_LL_Driver
  * @brief    SPWM LL external constant definition
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
#define SPWM_DATA_MASK                          (0xFFFF)

/**
  * @brief SPWM match type.
  */
typedef enum {
    /*! spwm match mode disable
     */
    LL_SPWM_MM_DISABLE = 0,
    /*! spwm match mode sel spwm1
     */
    LL_SPWM_MM_SPWM1,
    /*! spwm match mode sel spwm2
     */
    LL_SPWM_MM_SPWM2,
    /*! spwm match mode sel spwm3
     */
    LL_SPWM_MM_SPWM3,
} TYPE_ENUM_LL_SPWM_MATCH_MODE;

/**
  * @brief SPWM channel group type.
  */
typedef enum {
    /*! spwm select none
     */
    LL_SPWM_SEL_NONE    = 0x0,
    /*! spwm1 valid
     */
    LL_SPWM_SEL_SPWM1   = 0x1,
    /*! spwm2 valid
     */
    LL_SPWM_SEL_SPWM2   = 0x2,
    /*! spwm3 valid
     */
    LL_SPWM_SEL_SPWM3   = 0x4,
    /*! spwm12 valid
     */
    LL_SPWM_SEL_SPWM12  = 0x3,
    /*! spwm13 valid
     */
    LL_SPWM_SEL_SPWM13  = 0x5,
    /*! spwm23 valid
     */
    LL_SPWM_SEL_SPWM23  = 0x6,
    /*! spwm123 valid
     */
    LL_SPWM_SEL_SPWM123 = 0x7,
} TYPE_ENUM_LL_SPWM_SEL;

/**
  * @brief SPWM counter dir type.
  */
typedef enum {
    /*! spwm counter increase
     */
    LL_SPWM_CNT_DIR_INC = 0,
    /*! spwm counter decrease
     */
    LL_SPWM_CNT_DIR_DEC = 1,
} TYPE_ENUM_LL_SPWM_CNT_DIR;

/**
  * @brief SPWM level mode type.
  */
typedef enum {
    /*! spwm level two
     */
    LL_SPWM_TWO_LEVEL_MODE   = 1,
    /*! spwm level three
     */
    LL_SPWM_THREE_LEVEL_MODE = 0,
} TYPE_ENUM_LL_SPWM_LEVEL_MODE;

/**
  * @brief SPWM carry mode type.
  */
typedef enum {
    /*! spwm carry at counter == 0
     */
    LL_SPWM_CARRIER_AT_CNT_0   = 0,
    /*! spwm carry at counter == max
     */
    LL_SPWM_CARRIER_AT_CNT_MAX = 1,
} TYPE_ENUM_LL_SPWM_CARRIER_MODE;


/***** LL API *****/


  
/***** LL API AND DRIVER API *****/



/**
  * @}
  */

/** @defgroup SPWM_LL_Exported_Struct SPWM LL Exported Struct
  * @ingroup  SPWM_LL_Driver
  * @brief    SPWM LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the SPWM registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_spwm_init. Function, you can configure the SPWM module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief SPWM interrupt configuration struct.
  */
typedef struct __ll_spwm_irq_cfg {
    /*! fault_intr_en
     */
    bool fault_intr_en;
    /*! match_intr_en 
     */
    bool match_intr_en;
    /*! data_err_intr_en
     */
    bool data_err_intr_en;
    /*! load_data0_intr_en
     */
    bool load_data0_intr_en;
    /*! load_data1_intr_en
     */
    bool load_data1_intr_en;
    /*! load_data2_intr_en
     */
    bool load_data2_intr_en;
} TYPE_LL_SPWM_IRQ_CFG;


/**
  * @brief SPWM counter A/B/C group configuration struct.
  */
typedef struct __ll_spwm_cnt_cfg {
    /*! cnt_init_val : 15bit value 
     */
    u16                             cnt_init_val;
    /*! cnt_dir : 0(inc), 1(dec), TYPE_ENUM_LL_SPWM_CNT_DIR
     */
    TYPE_ENUM_LL_SPWM_CNT_DIR       cnt_dir;
    /*! deadtime_balance_en : 1 (enable) 
     */
    bool                            deadtime_balance_en;
    /*! level_mode : 1 (Two level mode)  0(Three level mode) TYPE_ENUM_LL_SPWM_LEVEL_MODE
     */
    TYPE_ENUM_LL_SPWM_LEVEL_MODE    level_mode;
    /*! carrier_mode : 0(data load at cnt=0), 1(data load at cnt=max) 
     */
    TYPE_ENUM_LL_SPWM_CARRIER_MODE  carrier_mode;
    /*! data_err_dis : 1(disable data error protect) 
     */
    bool                            data_err_dis;
} SPWM_LL_CNT_CFG;

/**
  * @brief SPWM configuration struct.
  */
typedef struct __ll_spwm_cfg {
    /*! fault valid control : [15:0] set to 1 means '0' valid 
     *
     * @brief  SPWM io init function
     * @param  None
     * @retval None
     * @note   The GPIO use as follows:
     *         |-------------------------------------------|
     *         |                   |         GPIO          |
     *         |-------------------------------------------|
     *         | spwm_a0           |         PB3           |
     *         | spwm_a1           |         PB2           |
     *         | spwm_a2           |         PB1           |
     *         | spwm_a3           |         PB0           |
     *         | spwm_b0           |         PB7           |
     *         | spwm_b1           |         PB6           |
     *         | spwm_b2           |         PB5           |
     *         | spwm_b3           |         PB4           |
     *         | spwm_c0           |         PB11          |
     *         | spwm_c1           |         PB10          |
     *         | spwm_c2           |         PB9           |
     *         | spwm_c3           |         PB8           |
     *         | fault_msb2lsb     | {PB12~PB15,PC0~PC11}. |
     *         |-------------------------------------------|
     */
    u16                             fault_invert;
    /*! fault enable control : [15:0] set to 1 means disable fault detect 
     */
    u16                             fault_disable;
    /*! fault_debunce_time : [3:0] Unit is system clock*64 
     */
    u8                              fault_debunce_time;
    /*! fault_protect_en : 
     */
    bool                            fault_protect_en;
    /*! match_kick_adc_chn_bits : [13:0] means adc[13:0] 
     */
    u16                             match_kick_adc_chn_bits;
    /*! match_value : 0~ (PWM_PERIOD-1) 
     */
    u16                             match_value;
    /*! match_work_mode : TYPE_SPWM_MATCH_MODE 
     */
    TYPE_ENUM_LL_SPWM_MATCH_MODE    match_work_mode;
    /*! dead_time : [11:0] Unit is system clock*2 
     */
    u16                             dead_time;
    /*! mini_time : [11:0] Unit is system clock*2 
     */
    u16                             mini_time;
    /*! period :  sys_clk/(spwm_freq) [15:0] Unit is system clock 
     */
    u16                             period;
    /*! output_invert_en : [15:0] set to 1 means output '0' valid 
     */
    bool                            output_invert_en;
    /*! pwm_sel : TYPE_SPWM_SEL 
     */
    TYPE_ENUM_LL_SPWM_SEL           pwm_sel;
    /*! SPWM counter A/B/C group configuration struct.
     */
    SPWM_LL_CNT_CFG                 pwm_chn[3];
} TYPE_LL_SPWM_CFG;

/**
  * @brief initialization structure for low layer SPWM module 
  */ 
typedef struct __ll_spwm_init {
    u8  reserved;
} TYPE_LL_SPWM_INIT;

/**
  * @}
  */

/** @defgroup SPWM_LL_Interrupt SPWM LL Interrupt Handle function
  * @brief   SPWM LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPWM  
    Interrupt Handle function.

    how to use?

    The SPWM interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the SPWM in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup SPWM_LL_Inti_Cfg SPWM LL Initialization And Configuration
  * @brief    SPWM LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPWM data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */


/**
  * @brief  LL_SPWM_REG_OPT : use for config SPWM_CON register
  * @param  expression : pointer to the hardware SPWM_TypeDef
  * @retval None
  */
#define LL_SPWM_REG_OPT(expression)   \
do {                                  \
    SPWM->SPWM_CON |= LL_SPWM_CFG_EN; \
    __asm("NOP");                     \
    expression;                       \
    SPWM->SPWM_CON &= ~LL_SPWM_CFG_EN;\
} while(0)

/**
  * @brief  ll_spwm_soft_reset : use for reset spwm 
  * @retval None
  */
__STATIC_INLINE void ll_spwm_soft_reset(void) {
    SYSCTRL->SYS_KEY = 0x4c5de9b3;
    SYSCTRL->SPWM_SOFTRESET = 0;
    __ASM("NOP"); __ASM("NOP"); __ASM("NOP"); __ASM("NOP");
    SYSCTRL->SPWM_SOFTRESET = 1;
    SYSCTRL->SYS_KEY = 0;
}

/**
  * @brief  spwm_init
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_init : pointer to the init struct TYPE_LL_SPWM_INIT
  * @retval None
  */
void ll_spwm_init(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_INIT *p_init);

/**
  * @brief  spwm_deinit
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @retval None
  */
void ll_spwm_deinit(SPWM_TypeDef *p_spwm);

/**
  * @brief  spwm config
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_cfg  : pointer to the init struct TYPE_LL_SPWM_IRQ_CFG
  * @retval None
  */
void ll_spwm_irq_config(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_IRQ_CFG *p_cfg);

/**
  * @brief  spwm config
  * @param  p_spwm : pointer to the hardware SPWM_TypeDef
  * @param  p_cfg  : pointer to the init struct TYPE_LL_SPWM_CFG
  * @retval None
  */
void ll_spwm_config(SPWM_TypeDef *p_spwm, TYPE_LL_SPWM_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup SPWM_LL_Data_Transfers SPWM LL Data transfers functions
  * @brief    SPWM LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPWM data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  spwm_start
  * @param  p_spwm: pointer to the hardware SPWM_TypeDef
  * @retval None
  */
void ll_spwm_start(SPWM_TypeDef *p_spwm);

/**
  * @brief  spwm stop
  * @param  p_spwm: pointer to the hardware SPWM_TypeDef
  * @retval None
  */
void ll_spwm_stop(SPWM_TypeDef *p_spwm);

/**
 * @brief  LL_SPWM_GET_HARDFAULT
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval fault_info
 */
#define LL_SPWM_GET_HARDFAULT(p_spwm)             ((((p_spwm)->SPWM_FAULT_INFO >> 16) & 0xFFFF) & \
                                                   (~((p_spwm)->SPWM_FAULT_INFO >> 0) & 0xFFFF) )

/**
 * @brief  LL_SPWM_GET_WORK_STA
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval working state
 */
#define LL_SPWM_GET_WORK_STA(p_spwm)              ((p_spwm)->SPWM_CON & LL_SPWM_WORK_FLAG)

/**
 * @brief  ll_spwm_match_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_match_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_MATCH |= LL_SPWM_MATCH_INT_EN;
}

/**
 * @brief  ll_spwm_match_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_match_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_MATCH &= ~LL_SPWM_MATCH_INT_EN;
}

/**
 * @brief  LL_SPWM_CHECK_MATCH_INTERRUPT_ENABLE
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval interrupt_enable
 */
#define LL_SPWM_CHECK_MATCH_INTERRUPT_ENABLE(p_spwm)    ((p_spwm)->SPWM_MATCH & LL_SPWM_MATCH_INT_EN)

/**
 * @brief  LL_SPWM_GET_MATCH_PENDING
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval pending
 */
#define LL_SPWM_GET_MATCH_PENDING(p_spwm)               ((p_spwm)->SPWM_MATCH & LL_SPWM_MATCH_PENDING)

/**
 * @brief  ll_spwm_clear_match_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_match_pending(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_MATCH |= LL_SPWM_MATCH_PENDING_CLR;
}

/**
 * @brief  ll_spwm_hwp_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_hwp_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CON |= LL_SPWM_HWP_INT_EN;
}

/**
 * @brief  ll_spwm_hwp_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_hwp_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CON &= ~LL_SPWM_HWP_INT_EN;
}

/**
 * @brief  LL_SPWM_CHECK_HWP_INTERRUPT_ENABLE
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval interrupt_enable
 */
#define LL_SPWM_CHECK_HWP_INTERRUPT_ENABLE(p_spwm)  ((p_spwm)->SPWM_CON & LL_SPWM_HWP_INT_EN)

/**
 * @brief  LL_SPWM_GET_HWP_PENDING
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval pending
 */
#define LL_SPWM_GET_HWP_PENDING(p_spwm)             ((p_spwm)->SPWM_CON |= LL_SPWM_HWP_PD)

/**
 * @brief  ll_spwm_clear_hwp_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_hwp_pending(SPWM_TypeDef *p_spwm) {
    LL_SPWM_REG_OPT((p_spwm)->SPWM_CON |= LL_SPWM_CLR_HWP_PD);
}

/**
 * @brief  ll_spwm_dataerr_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_dataerr_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CON |= LL_SPWM_DATAERR_INT_EN_ALL;
}

/**
 * @brief  ll_spwm_dataerr_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_dataerr_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CON &= ~LL_SPWM_DATAERR_INT_EN_ALL;
}

/**
 * @brief  LL_SPWM_CHECK_DATAERR_INTERRUPT_ENABLE
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval interrupt_enable
 */
#define LL_SPWM_CHECK_DATAERR_INTERRUPT_ENABLE(p_spwm)  ((p_spwm)->SPWM_CON & LL_SPWM_DATAERR_INT_EN_ALL)

/**
 * @brief  LL_SPWM_GET_DATAERR_PENDING
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval pending
 */
#define LL_SPWM_GET_DATAERR_PENDING(p_spwm)             ((p_spwm)->SPWM_CON & LL_SPWM_DATAERR_PD_ALL)

/**
 * @brief  ll_spwm_clear_dataerr_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_dataerr_pending(SPWM_TypeDef *p_spwm) {
    LL_SPWM_REG_OPT((p_spwm)->SPWM_CON |= LL_SPWM_CLR_DATAERR_PD_ALL);
}

/**
 * @brief  ll_spwm_load_data0_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data0_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT0 |= LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  ll_spwm_load_data0_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data0_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT0 &= ~LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  ll_spwm_load_data1_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data1_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT1 |= LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  ll_spwm_load_data1_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data1_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT1 &= ~LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  ll_spwm_load_data2_interrupt_enable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data2_interrupt_enable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT2 |= LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  ll_spwm_load_data2_interrupt_disable
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_load_data2_interrupt_disable(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT2 &= ~LL_SPWM_LOAD_INT_EN;
}

/**
 * @brief  LL_SPWM_CHECK_LOAD_INTERRUPT_ENABLE
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @param  n     : 0/1/2 for spwm A/B/C
 * @retval interrupt_enable
 */
#define LL_SPWM_CHECK_LOAD_INTERRUPT_ENABLE(p_spwm, n)      ((p_spwm)->SPWM_CNT##n & LL_SPWM_LOAD_INT_EN)

/**
 * @brief  LL_SPWM_GET_LOAD_PENDING
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @param  n     : 0/1/2 for spwm A/B/C
 * @retval pending
 */
#define LL_SPWM_GET_LOAD_PENDING(p_spwm, n)                 ((p_spwm)->SPWM_CNT##n & LL_SPWM_LOAD_PD)

/**
 * @brief  ll_spwm_clear_load_data0_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_load_data0_pending(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT0 |= LL_SPWM_CLR_LOAD_PD;
}

/**
 * @brief  ll_spwm_clear_load_data1_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_load_data1_pending(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT1 |= LL_SPWM_CLR_LOAD_PD;
}

/**
 * @brief  ll_spwm_clear_load_data2_pending
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_load_data2_pending(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_CNT2 |= LL_SPWM_CLR_LOAD_PD;
}

/**
 * @brief  ll_spwm_fill_databuf0
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @param  dat   : spwm data
 * @retval none
 */
__STATIC_INLINE void ll_spwm_fill_databuf0(SPWM_TypeDef *p_spwm, s16 dat) {
    p_spwm->SPWM_DATABUF0 = dat;
}

/**
 * @brief  ll_spwm_fill_databuf1
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @param  dat   : spwm data
 * @retval none
 */
__STATIC_INLINE void ll_spwm_fill_databuf1(SPWM_TypeDef *p_spwm, s16 dat) {
    p_spwm->SPWM_DATABUF1 = dat;
}

/**
 * @brief  ll_spwm_fill_databuf2
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @param  dat   : spwm data
 * @retval none
 */
__STATIC_INLINE void ll_spwm_fill_databuf2(SPWM_TypeDef *p_spwm, s16 dat) {
    p_spwm->SPWM_DATABUF2 = dat;
}

/**
 * @brief  ll_spwm_clear_databuf
 * @param  p_spwm: pointer to the hardware SPWM_TypeDef
 * @retval none
 */
__STATIC_INLINE void ll_spwm_clear_databuf(SPWM_TypeDef *p_spwm) {
    p_spwm->SPWM_DATAUSE0 = 0;
    p_spwm->SPWM_DATAUSE1 = 0;
    p_spwm->SPWM_DATAUSE2 = 0;
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

#endif //__TX_PHE_LL_SPWM_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
