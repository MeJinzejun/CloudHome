/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_svpwm.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the SVPWM LL firmware functions.
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
#ifndef __TX_PHE_LL_SVPWM_H
#define __TX_PHE_LL_SVPWM_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup svpwm_interface_gr SVPWM Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup SVPWM_LL_Driver SVPWM LL Driver
  * @ingroup  svpwm_interface_gr
  * @brief Mainly the driver part of the SVPWM module, which includes \b SVPWM \b Register 
  * \b Constants, \b SVPWM \b Exported \b Constants, \b SVPWM \b Exported \b Struct, \b SVPWM
  * \b Data \b transfers \b functions, \b SVPWM \b Initialization \b and \b SVPWM \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 
/** @defgroup SVPWM_LL_Register_Constants SVPWM LL Register Constants
  * @ingroup  SVPWM_LL_Driver
  * @brief    SVPWM LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the SVPWM 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the SVPWM register, mainly for convenience. Understand the 
    configuration of the SVPWM.
    
@endverbatim
  *
  * @{
  */

/***** SVPWM CON0 *****/
/*! RW, When DATAERR_PND = 1, data error, whether to generate an interrupt.
 */
#define LL_SVPWM_CON0_DER_INT_EN                  (1UL << 27)
/*! RW, Svpwm counter match indication flag, whether to generate an interrupt
 */
#define LL_SVPWM_CON0_MATCH_INT_EN                (1UL << 26)
/*! RW, Clear svpwm counter match indicator flag.
 */
#define LL_SVPWM_CON0_CLR_MATCH_PNG               (1UL << 25)
/*! RW, TSvpwm counter match indicator flag. This flag is generated when the 
 *  internal svpwm counter matches the value of SVPWM_MATCHCON.
 */
#define LL_SVPWM_CON0_MATCH_PNG                   (1UL << 24)
/*! RW, Svpwm counter match output enable. Generates a match output pulse when 
 *  the internal svpwm counter matches the value of SVPWM_MATCHCON
 */
#define LL_SVPWM_CON0_MATCH_EN                    (1UL << 23)
/*! RW, The PWM output is inverted. The original output is 0, and the output is 1.
 */
#define LL_SVPWM_CON0_PWMOUTINVT                  (1UL << 22)
/*! RO, Svpwm working status query flag
 */
#define LL_SVPWM_CON0_SVPWM_ON                    (1UL << 21)
/*! RO, Data error indication flag
 */
#define LL_SVPWM_CON0_DATAERR_PND                 (1UL << 20)
/*! RO, Whether the input data has been loaded to indicate the flag
 */
#define LL_SVPWM_CON0_LOADDATA_PND                (1UL << 19)
/*! RW, Hardware protection flag. This indicator will only be valid when HWP_EN=1.
 */
#define LL_SVPWM_CON0_HWP_PND                     (1UL << 18)
/*! WO, Clear data error flag
 */
#define LL_SVPWM_CON0_CLR_DERP                    (1UL << 17)
/*! WO, Clear data load flag
 */
#define LL_SVPWM_CON0_CLR_LDP                     (1UL << 16)
/*! WO, Clear hardware protection flag
 */
#define LL_SVPWM_CON0_CLR_HWP                     (1UL << 15)
/*! WO, Software disables SVPWM function
 */
#define LL_SVPWM_CON0_CLR_SVPWMEN                 (1UL << 14)
/*! RW, SVPWM related control register write enable
 */
#define LL_SVPWM_CON0_SVPWM_CFG_EN                (1UL << 13)
/*! RW, Hardware protection time.When a hardware error occurs, the module
 *  automatically enters the hardware protection state and automatically
 *  shuts down the svpwm module (HWPRT_EN = 1 must be configured).
 *  time = FAULT_DEBC_TIME *64*Tclk.(Tclk is the module clock cycle)
 */
#define LL_SVPWM_CON0_FAULT_DEBC_TIME(n)          (((n)&0xF) << 9)
/*! RW, Whether to automatically turn off the SVPWM module after data error 
 *  (DATAERR_PND = 1)
 */
#define LL_SVPWM_CON0_DERP_EN                     (1UL << 7)
/*! RW, When hardware protection flag HWP_PND=1, whether interrupt is generated
 */
#define LL_SVPWM_CON0_HWP_INT_EN                  (1UL << 6)
/*! RW, When the input data usage indication flag LOADDATA_PND=1, whether an 
 *  interrupt is generated
 */
#define LL_SVPWM_CON0_LOADDATA_INT_EN             (1UL << 5)
/*! RW, Whether to start the hardware protection function
 */
#define LL_SVPWM_CON0_HWPRT_EN                    (1UL << 4)
/*! RW, Abc coordinate input data normalization factor selection
 */
#define LL_SVPWM_CON0_NORM_FACTOR_SEL             (1UL << 3)
/*! RW, Dead zone control enabled. After enabling, the dead time is configured 
 *  via register SVPWM_CON2 
 */
#define LL_SVPWM_CON0_DT_EN                       (1UL << 2)
/*! RW, Two-level, three-level SVPWM select signal
 */
#define LL_SVPWM_CON0_LVL_SEL_THERE               (1UL << 1)
/*! RW, Software opens SVPWM function
 */
#define LL_SVPWM_CON0_M_EN                        (1UL << 0)


/***** SVPEM CON1 *****/
/*! RW, SVPWM switching time. The configuration value should be 1/2 of the actual 
 *  switching time. SW_TIME = round(1/2*Tsw/Tclk).Tsw is the actual switching cycle 
 *  time and Tclk is the SVPWM operating clock cycle. This register needs to set 
 *  SVPWM_CFG_EN=1 before writing.
 */
#define LL_SVPWM_CON1_SW_TIME(n)                  (((n)&0xFFFF) << 0)


/***** SVPWM CON2 *****/
/*! RO, SVPWM dead time control time.DT_TIME = round (Tdt/Tclk), Tdt is the actual 
 *  dead time control time, and Tclk is the SVPWM duty cycle. This register needs 
 *  to be set after SVPWM_CFG_EN=1.
 */
#define LL_SVPWM_CON2_DT_TIME(n)                  (((n)&0xFFF) << 0)


/***** SVPWM CON3 *****/
/*! RW, Abc coordinates input data normalization factor. When NORM_FACTOR_SEL=1, 
 *  the configuration will not take effect.NORM_FACTOR = sqrt(3)*m, m needs to be 
 *  less than 0.866, scaling method is Q14  
 */
#define LL_SVPWM_CON3_NORM_FACTOR(n)              (((n)&0xFFFF) << 0)


/***** SVPWM CON4 *****/
/*! RW, Hardware error detection control signal
 */
#define LL_SVPWM_CON4_FAULT_DECT_DIS(n)           (((n)&0xFFFF) << 0)


/***** SVPWM CON5 *****/
/*! RW, When HWP_SIG[n] = 1 in the hardware error judgment of the HWP_SIG signal 
 *  inside the module, it indicates that an error has occurred
 */
#define LL_SVPWM_CON5_FAULT_INVERT(n)             (((n)&0xFFFF) << 0)


/***** SVPWM REFA *****/
/*! RW, Phase A input reference signal (abc coordinates)
 */
#define LL_SVPWM_REFA(n)                          (((n)&0xFFFF) << 0)


/***** SVPWM REFB *****/
/*! RW, Phase B input reference signal (abc coordinates)
 */
#define LL_SVPWM_REFB(n)                          (((n)&0xFFFF) << 0)


/***** SVPWM REFC *****/
/*! RW, Phase C input reference signal (abc coordinates)
 */
#define LL_SVPWM_REFC(n)                          (((n)&0xFFFF) << 0)


/***** SVPWM STC0 *****/
/*! RO, The operating time of Vector B in a switching cycle
 */
#define LL_SVPWM_STC_B(n)                         (((n)&0xFFFF) << 15)
/*! RO, The operating time of Vector A1 in a switching cycle
 */
#define LL_SVPWM_STC_A1(n)                        (((n)&0xFFFF) << 0)


/***** SVPWM STC1 *****/
/*! RO, The operating time of Vector A2 in a switching cycle
 */
#define LL_SVPWM_STC_A2(n)                        (((n)&0xFFFF) << 15)
/*! RO, The operating time of Vector C in a switching cycle
 */
#define LL_SVPWM_STC_C(n)                         (((n)&0xFFFF) << 0)


/***** SVPWM MATCH CON *****/
/*! RW, The control register that matches the svpwm internal counter. When the 
 *  svpwm internal counter is consistent with this register, a matching pulse 
 *  signal and corresponding flag bit and interrupt are output.
 */
#define LL_SVPWM_MATCH_CON(n)                     (((n)&0x1FFFF) << 0)


/***** SVPWM ADC COM *****/
/*! RW, SVPWM Triggers the ADC conversion enable signal. When the svpwm internal 
 *  counter coincides with SVPWM_MATCH_CON, a matched pulse signal is output for 
 *  triggering the ADC conversion
 */
#define LL_SVPWM_ADC_AUTO_EN(n)                   (((n)&0x3FFF) << 0)

/**
  * @}
  */

/** @defgroup SVPWM_LL_Exported_Constants SVPWM LL Exported Constants
  * @ingroup  SVPWM_LL_Driver
  * @brief    SVPWM LL external constant definition
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
/*! RO, SVPWM Output IO Enable Key(A:0x54, B:0xA4, C:0x58)
 */ 
#define LL_SVPWM_KEY_VALUE                        (SVPWM_KEY_C | SVPWM_KEY_B | SVPWM_KEY_A)

/*! The macro defines the configuration enable of the SVPWM.
 */
#define LL_SVPWM_REG_OPT(expression)            \
do{                                             \
    SVPWM->CON0 |= LL_SVPWM_CON0_SVPWM_CFG_EN;  \
    expression;                                 \
    SVPWM->CON0 &= ~LL_SVPWM_CON0_SVPWM_CFG_EN; \
}while(0)


/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief Enumeration constant for The SVPWM mode selection
  */
typedef enum {
    /*! svpwm Two-level mode
     */
    LL_SVPWM_MODE_LVL_SEL_TWO = 0,
    /*! svpwm Three-level mode
     */
    LL_SVPWM_MODE_LVL_SEL_THREE,
} TYPE_ENUM_LL_SVPWM_MODE_SEL;

/**
  * @brief Enumeration constant for The SVPWM fault debc selection
  */
typedef enum {
    /*! Set the hardware protection time and select 0.
     */
    LL_SVPWM_FAULT_DEBC_SEL0 = 0,
    /*! Set the hardware protection time and select 1.
     */
    LL_SVPWM_FAULT_DEBC_SEL1,
    /*! Set the hardware protection time and select 2.
     */
    LL_SVPWM_FAULT_DEBC_SEL2,
    /*! Set the hardware protection time and select 3.
     */
    LL_SVPWM_FAULT_DEBC_SEL3,
    /*! Set the hardware protection time and select 4.
     */
    LL_SVPWM_FAULT_DEBC_SEL4,
    /*! Set the hardware protection time and select 5.
     */
    LL_SVPWM_FAULT_DEBC_SEL5,
    /*! Set the hardware protection time and select 6.
     */
    LL_SVPWM_FAULT_DEBC_SEL6,
    /*! Set the hardware protection time and select 7.
     */
    LL_SVPWM_FAULT_DEBC_SEL7,
    /*! Set the hardware protection time and select 8.
     */
    LL_SVPWM_FAULT_DEBC_SEL8,
    /*! Set the hardware protection time and select 9.
     */
    LL_SVPWM_FAULT_DEBC_SEL9,
    /*! Set the hardware protection time and select 10.
     */
    LL_SVPWM_FAULT_DEBC_SEL10,
    /*! Set the hardware protection time and select 11.
     */
    LL_SVPWM_FAULT_DEBC_SEL11,
    /*! Set the hardware protection time and select 12.
     */
    LL_SVPWM_FAULT_DEBC_SEL12,
    /*! Set the hardware protection time and select 13.
     */
    LL_SVPWM_FAULT_DEBC_SEL13,
    /*! Set the hardware protection time and select 14.
     */
    LL_SVPWM_FAULT_DEBC_SEL14,
    /*! Set the hardware protection time and select 15.
     */
    LL_SVPWM_FAULT_DEBC_SEL15,
} TYPE_ENUM_LL_SVPWM_FAULT_DEBC_SEL;

/**
  * @brief Enumeration constant for The SVPWM norm factor selection
  */
typedef enum {
    /*! The abc coordinate input data normalization factor, choose 
     *  to use the built-in normalization factor. 
     */
    LL_SVPWM_NORM_FACTOR_SEL_INTERNAL = 0,
    /*! The abc coordinate enters the data normalization factor and 
     *  selects the user-configured normalization factor.
     */
    LL_SVPWM_NORM_FACTOR_SEL_USER,
} TYPE_ENUM_LL_SVPWM_NORM_FACTOR_SEL;

/**
  * @}
  */

/** @defgroup SVPWM_LL_Exported_Struct SVPWM LL Exported Struct
  * @ingroup  SVPWM_LL_Driver
  * @brief    SVPWM LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the SVPWM registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_svpwm_init. Function, you can configure the SVPWM module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */
  
/**
  * @brief SVPWM interrupt low layer configuration structure
  */
typedef struct _ll_svpwm_irq_cfg {
    /*! data err protect interrupt enable
     */
    bool dataerr_intr_en;
    /*! adc auto trigle match interrupt enable
     */
    bool match_intr_en;
    /*! hardware protect interrupt enable
     */
    bool hwp_intr_en;
    /*! load data interrupt enable
     */
    bool load_data_intr_en;
} TYPE_LL_SVPWM_IRQ_CFG;

/**
  * @brief SVPWM param low layer configuration structure
  */
typedef struct _ll_svpwm_cfg {
    /*! deal timer enable
     */
    bool                               dead_time_en;
    /*! hardware protect enable
     */
    bool                               hwp_en;
    /*! pwm out inverted
     */
    bool                               pwm_out_invt_en;
    /*! adc auto trigle match enable
     */
    bool                               match_en;
    /*! data err protect enable
     */
    bool                               dataerr_protect_en;
    /*! svpwm mode sel
     */
    TYPE_ENUM_LL_SVPWM_MODE_SEL        mode_sel;
    /*! ABC coordinate input data normalization factor selection
     */
    TYPE_ENUM_LL_SVPWM_NORM_FACTOR_SEL norm_factor_sel;
    /*! Hardware fault protection time
     */
    TYPE_ENUM_LL_SVPWM_FAULT_DEBC_SEL  fault_debc_sel;
    
    /*! SVPWM switching time. The configuration value should be 1/2 of the actual 
     *  switching time. SW_TIME = round(1/2*Tsw/Tclk).Tsw is the actual switching cycle 
     *  time and Tclk is the SVPWM operating clock cycle. This register needs to set 
     *  SVPWM_CFG_EN=1 before writing.
     */
    u16                                period;
    /*! SVPWM dead time control time. DT_TIME = round (Tdt/Tclk), Tdt is the actual 
     *  dead time control time, and Tclk is the SVPWM duty cycle. This register needs 
     *  to be set after SVPWM_CFG_EN=1, The range is between 0 and 4096.
     */
    u16                                dead_time;
    /*! ABC coordinates input data normalization factor. When NORM_FACTOR_SEL=1, 
     *  the configuration will not take effect.NORM_FACTOR = sqrt(3)*m, m needs to be 
     *  less than 0.866, scaling method is Q14  
     */
    u16                                norm_factor;
    /*! A control register that matches the svpwm internal counter. When the 
     *  svpwm internal counter matches the register, a matching pulse signal 
     *  and corresponding flag bits and interrupts are output. 
     */
    u32                                match_cnt;
    /*! Phase A inputs the reference signal (abc coordinates). 
     */
    u16                                phase_a_val;
    /*! Phase B inputs the reference signal (abc coordinates). 
     */
    u16                                phase_b_val;
    /*! Phase C inputs the reference signal (abc coordinates). 
     */
    u16                                phase_c_val;
    
    union {
        /*! hardware fault all channel disable
         */
        u16 hwp_fault_dis;
        struct {
            /*! hardware fault channel0 disable
             */
            u16 hwp_fault_dis0  : 1,
            /*! hardware fault channel1 disable
             */
                hwp_fault_dis1  : 1,
            /*! hardware fault channel2 disable
             */
                hwp_fault_dis2  : 1,
            /*! hardware fault channel3 disable
             */
                hwp_fault_dis3  : 1,
            /*! hardware fault channel4 disable
             */
                hwp_fault_dis4  : 1,
            /*! hardware fault channel5 disable
             */
                hwp_fault_dis5  : 1,
            /*! hardware fault channel6 disable
             */
                hwp_fault_dis6  : 1,
            /*! hardware fault channel7 disable
             */
                hwp_fault_dis7  : 1,
            /*! hardware fault channel8 disable
             */
                hwp_fault_dis8  : 1,
            /*! hardware fault channel9 disable
             */
                hwp_fault_dis9  : 1,
            /*! hardware fault channel10 disable
             */
                hwp_fault_dis10 : 1,
            /*! hardware fault channel11 disable
             */
                hwp_fault_dis11 : 1,
            /*! hardware fault channel12 disable
             */
                hwp_fault_dis12 : 1,
            /*! hardware fault channel13 disable
             */
                hwp_fault_dis13 : 1,
            /*! hardware fault channel14 disable
             */
                hwp_fault_dis14 : 1,
            /*! hardware fault channel15 disable
             */
                hwp_fault_dis15 : 1;
        };
    };

    union {
        /*! hardware fault all channel invert 
         */
        u16 hwp_fault_invert;
        struct {
            /*! hardware fault channel0  invert 
             */
            u16 hwp_fault_invert0  : 1,
            /*! hardware fault channel1  invert 
             */
                hwp_fault_invert1  : 1,
            /*! hardware fault channel2  invert 
             */
                hwp_fault_invert2  : 1,
            /*! hardware fault channel3  invert 
             */
                hwp_fault_invert3  : 1,
            /*! hardware fault channel4  invert 
             */
                hwp_fault_invert4  : 1,
            /*! hardware fault channel5  invert 
             */
                hwp_fault_invert5  : 1,
            /*! hardware fault channel6  invert 
             */
                hwp_fault_invert6  : 1,
            /*! hardware fault channel7  invert 
             */
                hwp_fault_invert7  : 1,
            /*! hardware fault channel8  invert 
             */
                hwp_fault_invert8  : 1,
            /*! hardware fault channel9  invert 
             */
                hwp_fault_invert9  : 1,
            /*! hardware fault channel10 invert 
             */
                hwp_fault_invert10 : 1,
            /*! hardware fault channel11 invert 
             */
                hwp_fault_invert11 : 1,
            /*! hardware fault channel12 invert 
             */
                hwp_fault_invert12 : 1,
            /*! hardware fault channel13 invert 
             */
                hwp_fault_invert13 : 1,
            /*! hardware fault channel14 invert 
             */
                hwp_fault_invert14 : 1,
            /*! hardware fault channel15 invert 
             */
                hwp_fault_invert15 : 1;
        };
    };

    union {
        /*! svpwm auto trigle adc all channel enable 
         */
        u16 adc_auto_chn;
        struct {
            /*! svpwm auto trigle adc channel0  enable 
             */
            u16 adc_auto_chn0  : 1,
            /*! svpwm auto trigle adc channel1  enable 
             */
                adc_auto_chn1  : 1,
            /*! svpwm auto trigle adc channel2  enable 
             */
                adc_auto_chn2  : 1,
            /*! svpwm auto trigle adc channel3  enable 
             */
                adc_auto_chn3  : 1,
            /*! svpwm auto trigle adc channel4  enable 
             */
                adc_auto_chn4  : 1,
            /*! svpwm auto trigle adc channel5  enable 
             */
                adc_auto_chn5  : 1,
            /*! svpwm auto trigle adc channel6  enable 
             */
                adc_auto_chn6  : 1,
            /*! svpwm auto trigle adc channel7  enable 
             */
                adc_auto_chn7  : 1,
            /*! svpwm auto trigle adc channel8  enable 
             */
                adc_auto_chn8  : 1,
            /*! svpwm auto trigle adc channel9  enable 
             */
                adc_auto_chn9  : 1,
            /*! svpwm auto trigle adc channel10 enable 
             */
                adc_auto_chn10 : 1,
            /*! svpwm auto trigle adc channel11 enable 
             */
                adc_auto_chn11 : 1,
            /*! svpwm auto trigle adc channel12 enable 
             */
                adc_auto_chn12 : 1,
            /*! svpwm auto trigle adc channel13 enable 
             */
                adc_auto_chn13 : 1;
        };
    };
} TYPE_LL_SVPWM_CFG;

/**
  * @brief SVPWM low layer Initialization structure
  */
typedef struct _ll_svpwm_init {
    u8 reserved;
} TYPE_LL_SVPWM_INIT;

/**
  * @}
  */

/** @defgroup SVPWM_LL_Interrupt SVPWM LL Interrupt Handle function
  * @brief   SVPWM LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SVPWM  
    Interrupt Handle function.

    how to use?

    The SVPWM interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the SVPWM in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup SVPWM_LL_Inti_Cfg SVPWM LL Initialization And Configuration
  * @brief    SVPWM LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SVPWM data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  Low layer SVPWM module initialization
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_SVPWM_INIT)
  * @retval None
  */
void ll_svpwm_init(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_INIT *p_init);

/**
  * @brief  Low layer SVPWM module detele initialization
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @retval None
  */
void ll_svpwm_deinit(SVPWM_TypeDef *p_svpwm);

/**
  * @brief  Low layer SVPWM module interrupt configuration
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_SVPWM_IRQ_CFG)
  * @retval None
  */
void ll_svpwm_irq_config(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_IRQ_CFG *p_cfg);

/**
  * @brief  Low layer SVPWM module configuration
  * @param  p_svpwm: The structure pointer of the SVPWM is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_SVPWM_DMA_CFG)
  * @retval None
  */
void ll_svpwm_config(SVPWM_TypeDef *p_svpwm, TYPE_LL_SVPWM_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup SVPWM_LL_Data_Transfers SVPWM LL Data transfers functions
  * @brief    SVPWM LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SVPWM data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  SVPWM module start function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
void ll_svpwm_start(SVPWM_TypeDef *p_svpwm);

/**
  * @brief  SVPWM module stop function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
void ll_svpwm_stop(SVPWM_TypeDef *p_svpwm);

/**
  * @brief  SVPWM module get pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval pending
  */
#define LL_SVPWM_GET_PENDING(p_svpwm)                             ((p_svpwm)->CON0)

/**
  * @brief  SVPWM module check match interrupt enable
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval pending
  */
#define LL_SVPWM_CHECK_MATCH_INTERRUPT_ENABLE(p_svpwm)            ((p_svpwm)->CON0 & LL_SVPWM_CON0_MATCH_INT_EN)

/**
  * @brief  SVPWM module check data error interrupt enable
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval pending
  */
#define LL_SVPWM_CHECK_DATA_ERR_INTERRUPT_ENABLE(p_svpwm)         ((p_svpwm)->CON0 & LL_SVPWM_CON0_DER_INT_EN)

/**
  * @brief  SVPWM module check load data interrupt enable
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval pending
  */
#define LL_SVPWM_CHECK_LOAD_DATA_INTERRUPT_ENABLE(p_svpwm)        ((p_svpwm)->CON0 & LL_SVPWM_CON0_LOADDATA_INT_EN)

/**
  * @brief  SVPWM module check hardware protect interrupt enable
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval pending
  */
#define LL_SVPWM_CHECK_HARDWARE_PROTECT_INTERRUPT_ENABLE(p_svpwm) ((p_svpwm)->CON0 & LL_SVPWM_CON0_HWP_INT_EN)

/**
  * @brief  SVPWM get match pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
#define LL_SVPWM_GET_MATCH_PENDING(p_svpwm)                       ((p_svpwm)->CON0 & LL_SVPWM_CON0_MATCH_PNG)

/**
  * @brief  SVPWM get data err pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
#define LL_SVPWM_GET_DATA_ERR_PENDING(p_svpwm)                    ((p_svpwm)->CON0 & LL_SVPWM_CON0_DATAERR_PND)

/**
  * @brief  SVPWM get load data pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
#define LL_SVPWM_GET_LOAD_DATA_PENDING(p_svpwm)                   ((p_svpwm)->CON0 & LL_SVPWM_CON0_LOADDATA_PND)

/**
  * @brief  SVPWM get hardware protect pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
#define LL_SVPWM_GET_HARDWARE_PROTECT_PENDING(p_svpwm)            ((p_svpwm)->CON0 & LL_SVPWM_CON0_HWP_PND)

/**
  * @brief  SVPWM clear match pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_clear_match_pending(SVPWM_TypeDef *p_svpwm) {
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_MATCH_PNG;
}

/**
  * @brief  SVPWM clear data err pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_clear_data_err_pending(SVPWM_TypeDef *p_svpwm) {
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_DERP;
}

/**
  * @brief  SVPWM clear load data pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_clear_load_data_pending(SVPWM_TypeDef *p_svpwm) {
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_LDP;
}

/**
  * @brief  SVPWM clear hardware protect pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_clear_hardware_protect_pending(SVPWM_TypeDef *p_svpwm) {
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_HWP;
}

/**
  * @brief  SVPWM wait load data pending
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_wait_load_data_pending(SVPWM_TypeDef *p_svpwm) {
    while(!(p_svpwm->CON0 & LL_SVPWM_CON0_LOADDATA_PND));
    p_svpwm->CON0 |= LL_SVPWM_CON0_CLR_LDP;
}

/**
  * @brief  SVPWM set phase switch time value function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @param  sw_val: Set the value of switch time.
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_set_switch_time_value(SVPWM_TypeDef *p_svpwm, u16 sw_val) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON1 = LL_SVPWM_CON1_SW_TIME(sw_val);
    );
}

/**
  * @brief  SVPWM set phase dead time time value function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @param  dt_val: Set the value of dead time.
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_set_dead_time_value(SVPWM_TypeDef *p_svpwm, u16 dt_val) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON2 = LL_SVPWM_CON2_DT_TIME(dt_val);
    );
}

/**
  * @brief  SVPWM set phase value function
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @param  phase_a_val: Set the value of phase A.
  * @param  phase_b_val: Set the value of phase B.
  * @param  phase_c_val: Set the value of phase C.
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_set_phase_value(SVPWM_TypeDef *p_svpwm, 
                                              u16 phase_a_val,
                                              u16 phase_b_val,
                                              u16 phase_c_val) {
    p_svpwm->REFA = LL_SVPWM_REFA(phase_a_val);
    p_svpwm->REFB = LL_SVPWM_REFB(phase_b_val);
    p_svpwm->REFC = LL_SVPWM_REFC(phase_c_val);
}

/**
  * @brief  SVPWM enable data err interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_data_err_interrupt_enable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 |= LL_SVPWM_CON0_DER_INT_EN;
    );
}

/**
  * @brief  SVPWM disable data err interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_data_err_interrupt_disable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 &= ~(LL_SVPWM_CON0_DER_INT_EN);
    );
}

/**
  * @brief  SVPWM enable match interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_match_interrupt_enable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 |= LL_SVPWM_CON0_MATCH_INT_EN;
    );
}

/**
  * @brief  SVPWM disable match interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_match_interrupt_disable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 &= ~(LL_SVPWM_CON0_MATCH_INT_EN);
    );
}

/**
  * @brief  SVPWM enable hardware protect interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_hardware_protect_interrupt_enable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 |= LL_SVPWM_CON0_HWP_INT_EN;
    );
}

/**
  * @brief  SVPWM disable hardware protect interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_hardware_protect_interrupt_disable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 &= ~(LL_SVPWM_CON0_HWP_INT_EN);
    );
}

/**
  * @brief  SVPWM enable load data interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_load_data_interrupt_enable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 |= LL_SVPWM_CON0_LOADDATA_INT_EN;
    );
}

/**
  * @brief  SVPWM disable load data interrupt
  * @param  p_svpwm: Select the initialized SVPWM group pointer
  * @retval None
  */
__STATIC_INLINE void ll_svpwm_load_data_interrupt_disable(SVPWM_TypeDef *p_svpwm) {
    LL_SVPWM_REG_OPT(
        p_svpwm->CON0 &= ~(LL_SVPWM_CON0_LOADDATA_INT_EN);
    );
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

#endif //__TX_PHE_LL_SVPWM_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
