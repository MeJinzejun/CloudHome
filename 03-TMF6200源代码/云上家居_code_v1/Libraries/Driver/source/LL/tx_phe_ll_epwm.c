/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_epwm.c
  * @author  HUGE-IC Application Team
  * @version V1.0.2
  * @date    09-05-2019
  * @brief   This file contains all the EPWM LL firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  * how to use?
  * 1. You need to declare a variable of TYPE_LL_EPWM_INIT and assign a value to the variable.
  * 2. Call ll_epwm_init() function to initialize EPWM.
  * 3. Call the ll_epwm_cfg_start() or ll_epwm_start() function to start the EPWM module work.
  *
  * Special Note:
  * 1. The GPIO corresponding to the external trigger signal of TZ is as follows:
  *    ©°©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ð©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©´
  *    ©¦  TZ_NUM  ©¦          GPIO          ©¦
  *    ©À©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©à©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©È
  *    ©¦   TZ1    ©¦          PC1           ©¦
  *    ©¦   TZ2    ©¦          PC2           ©¦
  *    ©¦   TZ3    ©¦          PC3           ©¦
  *    ©¦   TZ4    ©¦          PC4           ©¦
  *    ©¦   TZ5    ©¦   Crystal oscillator   ©¦
  *    ©¦   TZ6    ©¦       CPU force        ©¦
  *    ©¸©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©Ø©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¤©¼
  * 2. The GPIO pin corresponding to the EPWMxSYNCI signal in EPWM0 is GPIOC0.
  * 3. The priority of conflicting actions on the TZCTL register is as follows 
  *    (highest priority overrides lower priority):
  *    1). Output EPWMxA: TZA (highest) -> DCAEVT1 -> DCAEVT2 (lowest)
  *    2). Output EPWMxB: TZB (highest) -> DCBEVT1 -> DCBEVT2 (lowest)
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_epwm.h"

/** @addtogroup EPWM_LL_Driver EPWM LL Driver
  * @ingroup  epwm_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static TYPE_ENUM_LL_EPWM_IO_MAP_SEL epwm_io_map[8];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup EPWM_LL_Interrupt EPWM LL Interrupt Handle function
  * @ingroup  EPWM_LL_Driver
  * @brief   EPWM LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup EPWM_LL_Inti_Cfg EPWM LL Initialization And Configuration
  * @ingroup  EPWM_LL_Driver
  * @brief    EPWM LL Initialization And Configuration
  * @{
  */
  
/**
  * @brief  Low layer EPWM initialization function
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @param  p_init: Low layer initialization variable
  * @retval None
  */
void ll_epwm_init(EPWM_TypeDef *p_epwm, TYPE_LL_EPWM_INIT *p_init)
{
    /* The system resets the EPWM module, Enable the clock of the module. */
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    /* The epwm_adpll_div_val must be less than 15 */
    TX_ASSERT(p_init->epwm_adpll_div_val < 15);
    
    /* Select the clock source for adcpll156M. */
    // ll_cc_adcpll_set(__EPWM_FADC_ADPLL_VAL, LL_CC_RST_PLLCLK_SRC_HXOSC);
    SYSCTRL_REG_OPT(
        SYSCTRL->CLK_CON0 = ((SYSCTRL->CLK_CON0 & (~LL_CC_RST_EPWM_CLK_SRC_SEL(0x3))) | 
                             LL_CC_RST_EPWM_CLK_SRC_SEL(p_init->src));

        SYSCTRL->CLK_CON2 = (SYSCTRL->CLK_CON2 & (~LL_CC_RST_EPWMCLK_DIV(0xF))) | 
                             LL_CC_RST_EPWMCLK_DIV(p_init->epwm_adpll_div_val);
    );
    /* IO enable key initialization */
    SYSCTRL_REG_OPT(SYSCTRL->PWM_KEY = GPIO_KEY_C | GPIO_KEY_B | GPIO_KEY_A);
}

/**
  * @brief  Low layer EPWM delete initialization function
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @retval None
  */
void ll_epwm_deinit(EPWM_TypeDef *p_epwm)
{
    /* The system disable the EPWM module, includes turning off the clock for the module. */
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    
    SYSCTRL_REG_OPT(SYSCTRL->IO_MAP  &= ~0x7E000000; \
                    SYSCTRL->IO_MAP1 &= ~0xFF000000;
    );
}

/**
  * @brief  Low layer EPWM Configuration function
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @param  p_cfg : Low layer initialization variable
  * @retval None
  */
/**
  * @brief  Low layer EPWM Configuration function
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @param  p_cfg : Low layer initialization variable
  * @retval None
  */
void ll_epwm_config(EPWM_TypeDef *p_epwm, TYPE_LL_EPWM_CFG *p_cfg)
{
    u32 tt_ctl      = 0;
    u32 tb_ctl      = 0;
    u32 tb_prd      = 0;
    u32 tb_phase    = 0;
    u32 cc_ctl      = 0;
    u32 aq_ctlab    = 0;
    u32 aq_sfrc     = 0;
    u32 aq_csfrc    = 0;
    u32 db_ctl      = 0;
    u32 db_delay    = 0;
    u32 et_ctl      = 0;
    u32 dc_ctl      = 0;
    u32 dc_trip_sel = 0;
    u32 tz_ctl      = 0;
    u32 tz_flag     = 0;
    
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    TX_ASSERT(p_cfg->module.db.fall_cnt < 1024);
    TX_ASSERT(p_cfg->module.db.rise_cnt < 1024);
    TX_ASSERT(p_cfg->module.dc.window_width < 4096);
    TX_ASSERT(p_cfg->chn < LL_EPWM_MAX_CHANNEL);
    
    if((p_cfg->module.aq.cmp_a_dec_b_out != LL_EPWM_AQ_OUT_NO) ||
       (p_cfg->module.aq.cmp_a_inc_b_out != LL_EPWM_AQ_OUT_NO) ||
       (p_cfg->module.aq.cmp_a_dec_a_out != LL_EPWM_AQ_OUT_NO) ||
       (p_cfg->module.aq.cmp_a_inc_a_out != LL_EPWM_AQ_OUT_NO)) {
        /* Check the CMPA value */
        TX_ASSERT(p_cfg->module.cc.cmp_a_val > 1);
    }
    
    if((p_cfg->module.aq.cmp_b_dec_b_out != LL_EPWM_AQ_OUT_NO) || 
       (p_cfg->module.aq.cmp_b_inc_b_out != LL_EPWM_AQ_OUT_NO) ||
       (p_cfg->module.aq.cmp_b_dec_a_out != LL_EPWM_AQ_OUT_NO) ||
       (p_cfg->module.aq.cmp_b_inc_a_out != LL_EPWM_AQ_OUT_NO)) {
        /* Check the CMPB value */
        TX_ASSERT(p_cfg->module.cc.cmp_b_val > 1);
    }
    
    if(p_cfg->module.et.soc_a_en == true) {
        if((p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_INC) || 
          (p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_DEC)) {
            /* Check the CMPA value */
            TX_ASSERT(p_cfg->module.cc.cmp_a_val > 1);
        } else if((p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_INC) || 
                  (p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_DEC)) {
            /* Check the CMPB value */
            TX_ASSERT(p_cfg->module.cc.cmp_b_val > 1);
        } else if((p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_INC) || 
                  (p_cfg->module.et.soc_a_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_DEC)) {
            /* Check the CMPC value */
            TX_ASSERT(p_cfg->module.cc.cmp_c_val > 1);
        }
    }
    
    if(p_cfg->module.et.soc_b_en == true) {
        if((p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_INC) || 
          (p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_DEC)) {
            /* Check the CMPA value */
            TX_ASSERT(p_cfg->module.cc.cmp_a_val > 1);
        } else if((p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_INC) || 
                  (p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_DEC)) {
            /* Check the CMPB value */
            TX_ASSERT(p_cfg->module.cc.cmp_b_val > 1);
        } else if((p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_INC) || 
                  (p_cfg->module.et.soc_b_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_DEC)) {
            /* Check the CMPC value */
            TX_ASSERT(p_cfg->module.cc.cmp_c_val > 1);
        }
    }
    
    if(p_cfg->module.et.intr_en == true) {
        if((p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_INC) || 
          (p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPA_DEC)) {
            /* Check the CMPA value */
            TX_ASSERT(p_cfg->module.cc.cmp_a_val > 1);
        } else if((p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_INC) || 
                  (p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPB_DEC)) {
            /* Check the CMPB value */
            TX_ASSERT(p_cfg->module.cc.cmp_b_val > 1);
        } else if((p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_INC) || 
                  (p_cfg->module.et.intr_sel == LL_EPWM_ET_SOCSEL_TBCTR_CMPC_DEC)) {
            /* Check the CMPC value */
            TX_ASSERT(p_cfg->module.cc.cmp_c_val > 1);
        }
    }
    
    epwm_io_map[p_cfg->chn] = p_cfg->io_map_sel;

    /* EPWM_TTCTL config */
    tt_ctl      |= p_cfg->cpu_tz ? LL_EPWM_CPUTZ_ZERO : 0;
    
    /* EPWM_TBCTL config */
    tb_ctl      |= p_cfg->module.tb.read_counter_en ? LL_EPWM_READ_TBCTR_EN : 0;
    tb_ctl      |= p_cfg->module.tb.sync_latched_clr ? LL_EPWM_SYNC_LATCHED : 0;
    tb_ctl      |= LL_EPWM_SYNCO_SEL(p_cfg->module.tb.sync_sel);
    tb_ctl      |= p_cfg->module.tb.soft_sync_en ? LL_EPWM_SWF_SYNC : 0;
    tb_ctl      |= p_cfg->module.tb.phase_sync_en ? LL_EPWM_PHSEN : 0;
    tb_ctl      |= LL_EPWM_CTRMODE(p_cfg->module.tb.count_dir_mode);
    tb_ctl      |= p_cfg->module.tb.shadow_load_dis ? LL_EPWM_PRDLD_DIS : 0;
    
    /* EPWM_TBPRD config */
    tb_prd      |= LL_EPWM_FRACB_VALUE(p_cfg->module.tb.frac_b_val);
    tb_prd      |= p_cfg->module.tb.frac_b_edge ? LL_EPWM_FRACB_EDGE_NEG : 0;
    tb_prd      |= LL_EPWM_FRACA_VALUE(p_cfg->module.tb.frac_a_val);
    tb_prd      |= p_cfg->module.tb.frac_a_edge ? LL_EPWM_FRACA_EDGE_NEG : 0;
    tb_prd      |= LL_EPWM_TBPRD(p_cfg->module.tb.period_val);
    
    /* EPWM_TBPHASE config */
    tb_phase    |= p_cfg->module.tb.phase_sync_dir_up_en ? LL_EPWM_TBDIR_UP : 0;
    tb_phase    |= LL_EPWM_TBPHS(p_cfg->module.tb.phase_val);
             
    /* EPWM_CCCTL config */                     
    cc_ctl      |= p_cfg->module.cc.shadow_a_mode ? LL_EPWM_SHDW_A_MODE : 0;
    cc_ctl      |= LL_EPWM_LOAD_A_DATA_MODE(p_cfg->module.cc.load_a_mode);
    cc_ctl      |= p_cfg->module.cc.shadow_b_mode ? LL_EPWM_SHDW_B_MODE : 0;
    cc_ctl      |= LL_EPWM_LOAD_B_DATA_MODE(p_cfg->module.cc.load_b_mode);
    cc_ctl      |= p_cfg->module.cc.shadow_c_mode ? LL_EPWM_SHDW_C_MODE : 0;
    cc_ctl      |= LL_EPWM_LOAD_C_DATA_MODE(p_cfg->module.cc.load_c_mode);
             
    /* EPWM_AQCTLAB config */
    aq_ctlab    |= LL_EPWM_CPMB_DEC_DONE_B(p_cfg->module.aq.cmp_b_dec_b_out);
    aq_ctlab    |= LL_EPWM_CPMB_INC_DONE_B(p_cfg->module.aq.cmp_b_inc_b_out);
    aq_ctlab    |= LL_EPWM_CPMA_DEC_DONE_B(p_cfg->module.aq.cmp_a_dec_b_out);
    aq_ctlab    |= LL_EPWM_CPMA_INC_DONE_B(p_cfg->module.aq.cmp_a_inc_b_out);
    aq_ctlab    |= LL_EPWM_PRD_DONE_B(p_cfg->module.aq.period_b_out);
    aq_ctlab    |= LL_EPWM_ZERO_DONE_B(p_cfg->module.aq.zero_b_out);
    aq_ctlab    |= LL_EPWM_CPMB_DEC_DONE_A(p_cfg->module.aq.cmp_b_dec_a_out);
    aq_ctlab    |= LL_EPWM_CPMB_INC_DONE_A(p_cfg->module.aq.cmp_b_inc_a_out);
    aq_ctlab    |= LL_EPWM_CPMA_DEC_DONE_A(p_cfg->module.aq.cmp_a_dec_a_out);
    aq_ctlab    |= LL_EPWM_CPMA_INC_DONE_A(p_cfg->module.aq.cmp_a_inc_a_out);
    aq_ctlab    |= LL_EPWM_PRD_DONE_A(p_cfg->module.aq.period_a_out);     
    aq_ctlab    |= LL_EPWM_ZERO_DONE_A(p_cfg->module.aq.zero_a_out);    
         
    /* EPWM_AQSFRC config */
    aq_sfrc     |= p_cfg->module.aq.soft_one_time_b_en ? LL_EPWM_OTSFB_EN : 0;
    aq_sfrc     |= LL_EPWM_ACTSFB(p_cfg->module.aq.soft_b_out);
    aq_sfrc     |= p_cfg->module.aq.soft_one_time_a_en ? LL_EPWM_OTSFA_EN : 0;
    aq_sfrc     |= LL_EPWM_ACTSFA(p_cfg->module.aq.soft_a_out);
                
    /* EPWM_AQCSFRC config */
    aq_csfrc    |= LL_EPWM_CSFB(p_cfg->module.aq.contine_soft_b);
    aq_csfrc    |= LL_EPWM_CSFA(p_cfg->module.aq.contine_soft_a);
                
    /* EPWM_DBCTL config */
    db_ctl      |= p_cfg->module.db.half_cycle_en ? LL_EPWM_HALF_CYCLE : 0;
    db_ctl      |= LL_EPWM_IN_MODE(p_cfg->module.db.in_mode);
    db_ctl      |= LL_EPWM_POLSEL(p_cfg->module.db.pol_sel);
    db_ctl      |= LL_EPWM_OUT_MODE(p_cfg->module.db.out_mode);
                   
    /* EPWM_DBDELAY config */
    db_delay    |= LL_EPWM_FED(p_cfg->module.db.fall_cnt);
    db_delay    |= LL_EPWM_RED(p_cfg->module.db.rise_cnt);
                
    /* EPWM_ETCTL config */
    et_ctl      |= p_cfg->module.et.soc_b_en ? LL_EPWM_SOCBEN : 0;
    et_ctl      |= LL_EPWM_SOCBSEL(p_cfg->module.et.soc_b_sel);
    et_ctl      |= p_cfg->module.et.soc_a_en ? LL_EPWM_SOCAEN : 0;
    et_ctl      |= LL_EPWM_SOCASEL(p_cfg->module.et.soc_a_sel);
    et_ctl      |= p_cfg->module.et.intr_en ? LL_EPWM_INTEN : 0;
    et_ctl      |= LL_EPWM_INTSEL(p_cfg->module.et.intr_sel);
    et_ctl      |= LL_EPWM_SOCBPRD(p_cfg->module.et.soc_b_event);
    et_ctl      |= LL_EPWM_SOCAPRD(p_cfg->module.et.soc_a_event);
    et_ctl      |= LL_EPWM_INTPRD(p_cfg->module.et.intr_event);
                
    /* EPWM_DCCTL config */
    dc_ctl      |= p_cfg->module.dc.a_evt2_force_sync_sel ? LL_EPWM_AEVT2FRCSYNCSEL_ASYN : 0;
    dc_ctl      |= p_cfg->module.dc.a_evt2_src_sel ? LL_EPWM_AEVT2SRCSEL_FILT : 0;
    dc_ctl      |= p_cfg->module.dc.a_evt1_sync_en ? LL_EPWM_AEVT1SYNCE_EN : 0;
    dc_ctl      |= p_cfg->module.dc.a_evt1_soc_en ? LL_EPWM_AEVT1SOCE_EN : 0;
    dc_ctl      |= p_cfg->module.dc.a_evt1_force_sync_sel ? LL_EPWM_AEVT1FRCSYNCSEL_ASYN : 0;
    dc_ctl      |= p_cfg->module.dc.a_evt1_src_sel ? LL_EPWM_AEVT1SRCSEL_FILT : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt2_force_sync_sel ? LL_EPWM_BEVT2FRCSYNCSEL_ASYN : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt2_src_sel ? LL_EPWM_BEVT2SRCSEL_FILT : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt1_sync_en ? LL_EPWM_BEVT1SYNCE_EN : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt1_soc_en ? LL_EPWM_BEVT1SOCE_EN : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt1_force_sync_sel ? LL_EPWM_BEVT1FRCSYNCSEL_ASYN : 0;
    dc_ctl      |= p_cfg->module.dc.b_evt1_src_sel ? LL_EPWM_BEVT1SRCSEL_FILT : 0;
    dc_ctl      |= LL_EPWM_PULSESEL(p_cfg->module.dc.pulse_sel);
    dc_ctl      |= p_cfg->module.dc.blank_invert ? LL_EPWM_BLANKINV : 0;
    dc_ctl      |= p_cfg->module.dc.blank_en ? LL_EPWM_BLANKE_EN : 0;
    dc_ctl      |= LL_EPWM_FILTSRCSEL(p_cfg->module.dc.filt_src_sel);
    dc_ctl      |= p_cfg->module.dc.shadow_mode_dis ? LL_EPWM_SHDWMODE : 0;
    dc_ctl      |= p_cfg->module.dc.capture_en ? LL_EPWM_CAPE_EN : 0;
    
    /* EPWM_DCTRIP config */
    dc_trip_sel |= LL_EPWM_DCBEVT2(p_cfg->module.dc.dc_b_evt2_out);    
    dc_trip_sel |= LL_EPWM_DCBEVT1(p_cfg->module.dc.dc_b_evt1_out);    
    dc_trip_sel |= LL_EPWM_DCAEVT2(p_cfg->module.dc.dc_a_evt2_out);    
    dc_trip_sel |= LL_EPWM_DCAEVT1(p_cfg->module.dc.dc_a_evt1_out);    
    dc_trip_sel |= LL_EPWM_DCBLCOMPSEL(p_cfg->module.dc.dc_bl_cmp_in);
    dc_trip_sel |= LL_EPWM_DCBHCOMPSEL(p_cfg->module.dc.dc_bh_cmp_in);
    dc_trip_sel |= LL_EPWM_DCALCOMPSEL(p_cfg->module.dc.dc_al_cmp_in);
    dc_trip_sel |= LL_EPWM_DCAHCOMPSEL(p_cfg->module.dc.dc_ah_cmp_in);
    
    /* EPWM_TZCTL config */
    tz_ctl      |= p_cfg->module.tz.dc_b_evt1_en ? LL_EPWM_TZ_DCBEVT1_EN : 0;
    tz_ctl      |= p_cfg->module.tz.dc_a_evt1_en ? LL_EPWM_TZ_DCAEVT1_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz6_as_trip_en ? LL_EPWM_TZ_OSHT6_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz5_as_trip_en ? LL_EPWM_TZ_OSHT5_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz4_as_trip_en ? LL_EPWM_TZ_OSHT4_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz3_as_trip_en ? LL_EPWM_TZ_OSHT3_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz2_as_trip_en ? LL_EPWM_TZ_OSHT2_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz1_as_trip_en ? LL_EPWM_TZ_OSHT1_EN : 0;
    tz_ctl      |= p_cfg->module.tz.dc_b_evt2_en ? LL_EPWM_TZ_DCBEVT2_EN : 0;
    tz_ctl      |= p_cfg->module.tz.dc_a_evt2_en ? LL_EPWM_TZ_DCAEVT2_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz6_as_cbc_en ? LL_EPWM_TZ_CBC6_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz5_as_cbc_en ? LL_EPWM_TZ_CBC5_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz4_as_cbc_en ? LL_EPWM_TZ_CBC4_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz3_as_cbc_en ? LL_EPWM_TZ_CBC3_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz2_as_cbc_en ? LL_EPWM_TZ_CBC2_EN : 0;
    tz_ctl      |= p_cfg->module.tz.tz1_as_cbc_en ? LL_EPWM_TZ_CBC1_EN : 0;
    tz_ctl      |= LL_EPWM_TZ_DCBEVT2(p_cfg->module.tz.dc_b_evt2_action);
    tz_ctl      |= LL_EPWM_TZ_DCAEVT2(p_cfg->module.tz.dc_a_evt2_action);
    tz_ctl      |= LL_EPWM_TZ_DCBEVT1(p_cfg->module.tz.dc_b_evt1_action);
    tz_ctl      |= LL_EPWM_TZ_DCAEVT1(p_cfg->module.tz.dc_a_evt1_action);
    tz_ctl      |= LL_EPWM_TZ_TZB(p_cfg->module.tz.tz_b_action);    
    tz_ctl      |= LL_EPWM_TZ_TZA(p_cfg->module.tz.tz_a_action);    
    
    /* EPWM_TZFLAG config */
    tz_flag     |= p_cfg->module.tz.dc_b_evt2_intr_en ? LL_EPWM_TZ_DCBEVT2INTE : 0;
    tz_flag     |= p_cfg->module.tz.dc_b_evt1_intr_en ? LL_EPWM_TZ_DCBEVT1INTE : 0;
    tz_flag     |= p_cfg->module.tz.dc_a_evt2_intr_en ? LL_EPWM_TZ_DCAEVT2INTE : 0;
    tz_flag     |= p_cfg->module.tz.dc_a_evt1_intr_en ? LL_EPWM_TZ_DCAEVT1INTE : 0;
    tz_flag     |= p_cfg->module.tz.tz_trip_intr_en ? LL_EPWM_TZ_OSTINTE : 0;
    tz_flag     |= p_cfg->module.tz.tz_cbc_intr_en ? LL_EPWM_TZ_CBCINTE : 0;
    
    /* EPWM_TBCTL config */
    p_epwm->EPWM_TTCTL                     |= tt_ctl;
    
    /* TB module config */
    p_epwm->CH[p_cfg->chn].EPWM_TBCTL       = tb_ctl;
    p_epwm->CH[p_cfg->chn].EPWM_TBPRD       = tb_prd;
    p_epwm->CH[p_cfg->chn].EPWM_TBPHASE     = tb_phase;
       
    /* CC module config */               
    p_epwm->CH[p_cfg->chn].EPWM_CMPCTL      = cc_ctl;
    p_epwm->CH[p_cfg->chn].EPWM_CMPA        = LL_EPWM_CPMA_VALUE(p_cfg->module.cc.cmp_a_val);
    p_epwm->CH[p_cfg->chn].EPWM_CMPB        = LL_EPWM_CPMB_VALUE(p_cfg->module.cc.cmp_b_val);
    p_epwm->CH[p_cfg->chn].EPWM_CMPC        = LL_EPWM_CPMC_VALUE(p_cfg->module.cc.cmp_c_val);
                                  
    /* AQ module config */ 
    p_epwm->CH[p_cfg->chn].EPWM_AQCTLAB     = aq_ctlab;
    p_epwm->CH[p_cfg->chn].EPWM_AQSFRC      = aq_sfrc;
    p_epwm->CH[p_cfg->chn].EPWM_AQCSFRC     = aq_csfrc;
                                  
    /* DB module config */ 
    p_epwm->CH[p_cfg->chn].EPWM_DBCTL       = db_ctl;
    p_epwm->CH[p_cfg->chn].EPWM_DBDELAY     = db_delay;
                                  
    /* ET module config */ 
    p_epwm->CH[p_cfg->chn].EPWM_ETCTL       = et_ctl;
                                  
    /* DC module config */ 
    p_epwm->CH[p_cfg->chn].EPWM_DCCTL       = dc_ctl;
    p_epwm->CH[p_cfg->chn].EPWM_DCTRIPSEL   = dc_trip_sel;
    p_epwm->CH[p_cfg->chn].EPWM_DCCAP       = LL_EPWM_DCCAP(p_cfg->module.dc.dc_capture_cnt);
    p_epwm->CH[p_cfg->chn].EPWM_BLANKOFFSET = LL_EPWM_OFFSET(p_cfg->module.dc.blank_offset);
    p_epwm->CH[p_cfg->chn].EPWM_WINWIDTH    = LL_EPWM_WINDOW(p_cfg->module.dc.window_width);
    
    /* TZ module config */ 
    p_epwm->CH[p_cfg->chn].EPWM_TZCTL       = tz_ctl;
    p_epwm->CH[p_cfg->chn].EPWM_TZFLAG      = tz_flag;
}

/**
  * @brief  Low layer Path selection between Saradc and epwm.
  * @param  p_epwm  : Select the initialized EPWM group pointer
  * @param  adc_chn : Select the configured adc path, range 0~14.
  * @param  epwm_sel: Select the trigger source of adc.
  * @retval None
  * @note   1.The EPWM trigger source selected by saradc0 ~ saradc6 can only be selected between epwm0_soca ~epwm6_soca.
  *         2.The EPWM trigger source selected by saradc7 ~ saradc13 can only be selected between epwm0_socb ~epwm6_socb.
  */
void ll_epwm_saradc_sel_config(EPWM_TypeDef *p_epwm, u8 adc_chn, TYPE_ENUM_LL_EPWM_ADC_SEL epwm_sel)
{
    uint32_t temp;
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    TX_ASSERT(adc_chn < LL_EPWM_MAX_ADC_CHANNEL);
    
    if(adc_chn < 10) {
        temp = p_epwm->EPWM_ADCSEL0;
        temp &= ~(LL_EPWM_ADC_SEL0(7UL, adc_chn));
        temp |= LL_EPWM_ADC_SEL0(epwm_sel, adc_chn);
        p_epwm->EPWM_ADCSEL0 = temp;
    } else {
        temp = p_epwm->EPWM_ADCSEL1;
        temp &= ~(LL_EPWM_ADC_SEL1(7UL, adc_chn - 10));
        temp |= LL_EPWM_ADC_SEL1(epwm_sel, adc_chn - 10);
        p_epwm->EPWM_ADCSEL1 = temp;
    }
}

/**
  * @brief  Low layer Configure the map that triggers the signal between fadc and epwm.
  * @param  p_epwm: Select the initialized EPWM group pointer
  * @retval None
  */
void ll_epwm_fadc_map_config(EPWM_TypeDef *p_epwm)
{
    u32 epwm_adcsel0 = 0;
    u32 epwm_adcsel1 = 0;
    
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM0_SOCAB, LL_EPWM_ADC_CHN0);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM1_SOCAB, LL_EPWM_ADC_CHN1);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM2_SOCAB, LL_EPWM_ADC_CHN2);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM3_SOCAB, LL_EPWM_ADC_CHN3);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM4_SOCAB, LL_EPWM_ADC_CHN4);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM5_SOCAB, LL_EPWM_ADC_CHN5);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM6_SOCAB, LL_EPWM_ADC_CHN6);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM0_SOCAB, LL_EPWM_ADC_CHN7);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM1_SOCAB, LL_EPWM_ADC_CHN8);
    epwm_adcsel0 |= LL_EPWM_ADC_SEL0(LL_EWPM2_SOCAB, LL_EPWM_ADC_CHN9);
    
    epwm_adcsel1 |= LL_EPWM_ADC_SEL1(LL_EWPM3_SOCAB, LL_EPWM_ADC_CHN10 - 10);
    epwm_adcsel1 |= LL_EPWM_ADC_SEL1(LL_EWPM4_SOCAB, LL_EPWM_ADC_CHN11 - 10);
    epwm_adcsel1 |= LL_EPWM_ADC_SEL1(LL_EWPM5_SOCAB, LL_EPWM_ADC_CHN12 - 10);
    epwm_adcsel1 |= LL_EPWM_ADC_SEL1(LL_EWPM6_SOCAB, LL_EPWM_ADC_CHN13 - 10);
    
    p_epwm->EPWM_ADCSEL0 = epwm_adcsel0;
    p_epwm->EPWM_ADCSEL1 = epwm_adcsel1;
}

/**
  * @}
  */

/** @defgroup EPWM_LL_Data_Transfers EPWM LL Data transfers functions
  * @ingroup  EPWM_LL_Driver
  * @brief    EPWM LL Data transfers functions 
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
void ll_epwm_start(EPWM_TypeDef *p_epwm, u32 chn_bits)
{
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    u8 i = 0;
    u32 chn_bit_temp = chn_bits;
    
    SYSCTRL_REG_OPT(
        for(i=0; i<7; i++) {
            if(chn_bit_temp&0x01) {
                /* epwm io_map select and config */
                
                    switch(epwm_io_map[i]) {
                        
                        case LL_EPWM_IO_MAP_SEL_AB:
                            if(i < 3) {
                                SYSCTRL->IO_MAP |= BIT(25 + i*2);
                                SYSCTRL->IO_MAP |= BIT(26 + i*2);
                            } else {
                                SYSCTRL->IO_MAP1 |= BIT(24 + (i - 3)*2);
                                SYSCTRL->IO_MAP1 |= BIT(25 + (i - 3)*2);
                            }       
                            break;

                        case LL_EPWM_IO_MAP_SEL_A:
                            if(i < 3) {
                                SYSCTRL->IO_MAP |= BIT(25 + i*2);
                            } else {
                                SYSCTRL->IO_MAP1 |= BIT(24 + (i - 3)*2);
                            }       
                            break;

                        case LL_EPWM_IO_MAP_SEL_B:
                            if(i < 3) {
                                SYSCTRL->IO_MAP |= BIT(26 + i*2);
                            } else {
                                SYSCTRL->IO_MAP1 |= BIT(25 + (i - 3)*2);
                            }       
                            break;

                        case LL_EPWM_IO_MAP_SEL_NO:
                            break;
                        
                        default:
                            break;
                    }

                p_epwm->CH[i].EPWM_TZFLAG |= LL_EPWM_TZ_CLROST;
            }
            
            chn_bit_temp >>= 1;
        }
    );
    
    p_epwm->EPWM_TTCTL |= LL_EPWM_EN(chn_bits);
}

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
                  TYPE_ENUM_LL_EPWM_TZ_ACTION b_action)
{
    /* Check the parameters */
    TX_ASSERT(p_epwm == EPWM);
    u8 i = 0;
    u32 chn_bit_temp = chn_bits;
    uint32_t temp;
    for(i=0; i<7; i++) {
        if(chn_bit_temp&0x01) {
            temp   =  p_epwm->CH[i].EPWM_TZCTL;
            temp  &= ~(LL_EPWM_TZ_TZB(0x03) | LL_EPWM_TZ_TZA(0x03));
            temp  |= LL_EPWM_TZ_TZB(b_action) | LL_EPWM_TZ_TZA(a_action);
            p_epwm->CH[i].EPWM_TZCTL = temp;
            p_epwm->CH[i].EPWM_TZFLAG |= LL_EPWM_TZ_FRCOST;
        }
        chn_bit_temp >>= 1;
    }
    p_epwm->EPWM_TTCTL &= ~(LL_EPWM_EN(chn_bits));
}

/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
