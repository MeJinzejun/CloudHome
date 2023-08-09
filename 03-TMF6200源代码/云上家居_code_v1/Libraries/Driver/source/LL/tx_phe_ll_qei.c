/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_qei.c
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

/* Includes ------------------------------------------------------------------*/
#include "include.h"
#include "tx_phe_ll_qei.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup qei_interface_gr QEI Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup QEI_LL_Driver QEI LL Driver
  * @ingroup  qei_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup QEI_LL_Interrupt QEI LL Interrupt Handle function
  * @ingroup  QEI_LL_Driver
  * @brief   QEI LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup QEI_LL_Inti_Cfg QEI LL Initialization And Configuration
  * @ingroup  QEI_LL_Driver
  * @brief    QEI LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Low layer QEI module initialization
  * @param  p_qei : The structure pointer of the QEI is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_QEI_INIT)
  * @retval None
  */
void ll_qei_init(QEI_TypeDef *p_qei, TYPE_LL_QEI_INIT *p_init)
{
    TX_ASSERT((p_qei == QEI));

    /* enable clk */
    ll_cc_rst_apb1_peripheral_clk_enable(LL_CC_RST_APB1_M_QEI);
    
    /* module reset release */
    ll_cc_rst_softreset_release(LL_CC_RST_SOFTRST_M_QEI);
    
}

/**
  * @brief  Low layer QEI module detele initialization
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @retval None
  */
void ll_qei_deinit(QEI_TypeDef *p_qei)
{
    /* The system disable the QEI module, includes turning off the clock for the module. */
    TX_ASSERT((p_qei == QEI));
    
    /* module reset */
    ll_cc_rst_softreset(LL_CC_RST_SOFTRST_M_QEI);
    
    /* disable clk */
    ll_cc_rst_apb1_peripheral_clk_disable(LL_CC_RST_APB1_M_QEI);
    
    SYSCTRL_REG_OPT(
        SYSCTRL->IO_MAP &= ~BIT(8);
    );  
    
}

/**
  * @brief  Low layer QEI module configuration
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_QEI_IRQ_CFG)
  * @retval None
  */
void ll_qei_irq_config(QEI_TypeDef *p_qei, TYPE_LL_QEI_IRQ_CFG *p_cfg)
{
    TX_ASSERT((p_qei == QEI));
    
    p_qei->QEIE = ((p_cfg->updn_inv_intr_en) ? LL_QEI_UPDN_INV_FLAG_INT_EN : 0)                  |
                  ((p_cfg->speed_cnt_err_intr_en) ? LL_QEI_SPEED_CNT_ERR_INT_EN : 0)             |
                  ((p_cfg->speed_cnt_ov_intr_en) ? LL_QEI_SPEED_CNT_OV_INT_EN : 0)               |
                  ((p_cfg->speed_tmr_err_intr_en) ? LL_QEI_SPEED_TMR_ERR_INT_EN : 0)             |
                  ((p_cfg->speed_tmr_ov_intr_en) ? LL_QEI_SPEED_TMR_OV_INT_EN : 0)               |
                  ((p_cfg->speed_tmr_gate_failing_intr_en) ? LL_QEI_TG_FAILING_INT_EN : 0)       |
                  ((p_cfg->speed_pos_eq_max_intr_en) ? LL_QEI_POS_EQ_MAX_INT_EN : 0)             |
                  ((p_cfg->speed_pos_cnt_err_intr_en) ? LL_QEI_CNT_ERR_INT_EN : 0)               |
                  ((p_cfg->speed_idx_rst_cnt_intr_en) ? LL_QEI_IDX_RST_CNT_INT_EN : 0)           |
                  ((p_cfg->speed_max_eq_rst_cnt_intr_en) ? LL_QEI_POS_EQ_MAX_RST_CNT_INT_EN : 0) |
                  ((p_cfg->speed_phase_err_intr_en) ? LL_QEI_PHASE_ERR_INT_EN : 0);
    
    p_qei->QEICLR = 0xFFFF;
}

/**
  * @brief  Low layer QEI module configuration
  * @param  p_qei: The structure pointer of the QEI is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_QEI_CFG)
  * @retval None
  */
void ll_qei_config(QEI_TypeDef *p_qei, TYPE_LL_QEI_CFG *p_cfg)
{
    TX_ASSERT((p_qei == QEI));

    /* io config is neccessary */
    SYSCTRL_REG_OPT(SYSCTRL->IO_MAP |= BIT(8));

    p_qei->QEI_ROTATE_PERIOD = p_cfg->speed_calc_counter;
    p_qei->QEI_TIMER_PERIOD = p_cfg->speed_calc_time;
    
    p_qei->QEICLR = 0xffff;

    p_qei->QEIO = ((p_cfg->updn_dir_to_io_en) ? LL_QEI_PSD_OUT_EN : 0) |
                  ((p_cfg->qea_qeb_swap_en) ? LL_QEI_SWAP_A_B_EN : 0)  |
                  ((p_cfg->idx_pole) ? LL_QEI_IDX_INV_EN : 0)          |
                  ((p_cfg->qeb_pole) ? LL_QEI_QEB_INV_EN : 0)          |
                  ((p_cfg->qea_pole) ? LL_QEI_QEA_INV_EN : 0);

    p_qei->MAXCNT = p_cfg->max_count;
    p_qei->POSCNT = 0;
    
    p_qei->DFLTCON = ((p_cfg->dflt_en) ? LL_QEI_DFLT_EN : 0) |
                     LL_QEI_DFLT_CLK(p_cfg->dflt_pre_scale);
                   
    p_qei->QEICON =  ((p_cfg->speed_calc_en) ? LL_QEI_ROTATE_SPEED_EN : 0)                      |
                     LL_QEI_ROTATE_SPEED_MODE(p_cfg->speed_calc_mode)                           |
                     LL_QEI_IMV_B(p_cfg->imv_qeb)                                               |
                     LL_QEI_IMV_A(p_cfg->imv_qea)                                               |
                     LL_QEI_IMVS(p_cfg->imv_sel)                                                |
                     LL_QEI_MODE(p_cfg->mode)                                                   |
                     LL_QEI_PH_X2(p_cfg->phase)                                                 |
                     ((p_cfg->mode == LL_QEI_MODE_QEX2_RST_BY_IDX) ? LL_QEI_PULSE_RST_CNT_EN:0) |
                     ((p_cfg->mode == LL_QEI_MODE_QEX4_RST_BY_IDX) ? LL_QEI_PULSE_RST_CNT_EN:0);
}

/**
  * @}
  */

/** @defgroup QEI_LL_Data_Transfers QEI LL Data transfers functions
  * @ingroup  QEI_LL_Driver
  * @brief    QEI LL Data transfers functions 
  * @{
  */

/**
  * @brief  QEI module start function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval None
  */
void ll_qei_start(QEI_TypeDef *p_qei)
{
    TX_ASSERT((p_qei == QEI));
}

/**
  * @brief  QEI module stop function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval None
  */
void ll_qei_stop(QEI_TypeDef *p_qei)
{
    TX_ASSERT((p_qei == QEI));
}

/**
  * @brief  QEI get Motor Mechanical Angle function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval angle [0, 1] for [0, 360] theta
  */
float32_t ll_qei_get_mech_angle(QEI_TypeDef *p_qei)
{
    TX_ASSERT((p_qei == QEI));

    float32_t temp = (float32_t)p_qei->POSCNT;
    
    return temp / p_qei->MAXCNT;
}

/**
  * @brief  QEI get Motor Mechanical Angle function
  * @param  p_qei: Select the initialized QEI group pointer
  * @retval angle [0, 32768] for [0, 360] theta
  */
s16 ll_qei_get_mech_angle_q15(QEI_TypeDef *p_qei)
{
    TX_ASSERT((p_qei == QEI));
    
    u32 temp = p_qei->POSCNT * 32768;

    return temp / p_qei->MAXCNT;
}

/**
  * @brief QEI get Motor speed calc result function
  * @param p_qei   : Select the initialized QEI group pointer
  * @param pos_cnt : qea/qeb counters
  * @param time_cnt: apb1_clk counters
  * @retval true : ok, false : fail
  */
bool ll_qei_get_speed_result(QEI_TypeDef *p_qei, u32 *pos_cnt, u32 *time_cnt)
{
    TX_ASSERT((p_qei == QEI));
    if(LL_QEI_GET_SPEED_CNT_ERR_PENDING(p_qei) || LL_QEI_SPEED_GET_TMR_ERR_PENDING(p_qei)){
        ll_qei_clear_speed_cnt_err_pending(p_qei);
        ll_qei_clear_speed_tmr_err_pending(p_qei);
        return false;
    }
    //if(LL_QEI_GET_SPEED_CNT_OV_PENDING(p_qei) || LL_QEI_GET_SPEED_TMR_OV_PENDING(p_qei)){
        //ll_qei_clear_speed_cnt_ov_pending(p_qei);
        //ll_qei_clear_speed_tmr_ov_pending(p_qei);
        if(p_qei->QEICON & LL_QEI_ROTATE_SPEED_MODE(LL_QEI_ROTATE_SPEED_MODE_T)) {
            *pos_cnt = p_qei->QEI_ROTATE_PERIOD;
            *time_cnt = p_qei->QEI_TIMER_CNT_LATCH;
        } else {
            *pos_cnt = p_qei->QEI_ROTATE_CNT_LATCH;
            *time_cnt = p_qei->QEI_TIMER_PERIOD;
        }
        return true;
    //}
    //return false;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
