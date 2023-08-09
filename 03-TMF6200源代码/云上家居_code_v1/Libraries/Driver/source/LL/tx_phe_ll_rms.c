/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_rms.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the RMS LL RMSmware functions.
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
#include "tx_phe_ll_rms.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup rms_interface_gr RMS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup RMS_LL_Driver RMS LL Driver
  * @ingroup  rms_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup RMS_LL_Interrupt RMS LL Interrupt Handle function
  * @ingroup  RMS_LL_Driver
  * @brief   RMS LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup RMS_LL_Inti_Cfg RMS LL Initialization And Configuration
  * @ingroup  RMS_LL_Driver
  * @brief    RMS LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Low layer RMS module initialization
  * @param  p_rms: The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @param  p_init: Module configuration structure pointer(TYPE_LL_RMS_INIT)
  * @retval None
  */
void ll_rms_init(RMS_TypeDef *p_rms, TYPE_LL_RMS_INIT *p_init)
{
    /* The system resets the RMS module, Enable the clock of the module. */
    TX_ASSERT((p_rms == RMS0) || (p_rms == RMS1) || (p_rms == RMS2));
    
    memset(p_rms, 0x0, sizeof(RMS_TypeDef));
    p_rms->CON |= LL_RMS_CON_PEND;
}

/**
  * @brief  Low layer RMS module detele initialization
  * @param  p_rms: The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @retval None
  */
void ll_rms_deinit(RMS_TypeDef *p_rms)
{
    /* The system disable the RMS module, includes turning off the clock for the module. */
    TX_ASSERT((p_rms == RMS0) || (p_rms == RMS1) || (p_rms == RMS2));
}

/**
  * @brief  Low layer RMS module configuration
  * @param  p_rms: The structure pointer of the RMS group (RMS0, RMS1, RMS2) is selected.
  * @param  p_cfg: Module configuration structure pointer(TYPE_LL_RMS_CFG)
  * @retval None
  */
void ll_rms_config(RMS_TypeDef *p_rms, TYPE_LL_RMS_CFG *p_cfg)
{
    u32 rms_con   = 0;
    
#ifdef USE_ASSERT
    u8  data_uint = SRAM_BUF_UINT_2BYTE;
    
	NONSENSE(data_uint);
    
    TX_ASSERT((p_rms == RMS0) || (p_rms == RMS1) || (p_rms == RMS2));
    TX_ASSERT(p_cfg->in_frac_width < 32);
    
    if((p_cfg->mode == LL_RMS_MODE_MOLDING) || (p_cfg->mode == LL_RMS_MODE_MOLE_SQUARE)) {
        data_uint = SRAM_BUF_UINT_4BYTE;
    }
    /* Check the parameters of RMS input DMA buffer address */
    if(p_cfg->mode != LL_RMS_MODE_SQUARE_ROOT) {
        if(p_rms == RMS0) {
            /* RMS0's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  */
            TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS0_MASK, SRAM_RMS0_ALIGN_MASK);
            /* RMS0's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS0_MASK, SRAM_RMS0_ALIGN_MASK);
        } else if(p_rms == RMS1) {
            /* RMS1's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  */
            TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS1_MASK, SRAM_RMS1_ALIGN_MASK);
            /* RMS1's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS1_MASK, SRAM_RMS1_ALIGN_MASK);
        } else if(p_rms == RMS2) {
            /* RMS2's dma_src_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3.  */
            TX_ASSERT_ADDR(p_cfg->dma_src_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS2_MASK, SRAM_RMS2_ALIGN_MASK);
            /* RMS2's dma_dst_addr can only be located in SRAM0/SRAM1/SRAM2/SRAM3. */
            TX_ASSERT_ADDR(p_cfg->dma_dst_addr, p_cfg->dma_points * data_uint,
                           SRAM_RMS2_MASK, SRAM_RMS2_ALIGN_MASK);
        }
    }
#endif
    
    rms_con  = p_rms->CON & LL_RMS_CON_IE;
    rms_con |= LL_RMS_CON_MODE(p_cfg->mode);
    
    p_rms->CON           = rms_con;
    p_rms->LEN           = LL_RMS_LEN(p_cfg->dma_points);
    p_rms->IN_FRAC_WIDTH = LL_RMS_IN_FRAC_WIDTH(p_cfg->in_frac_width);
    p_rms->DATA_IN_ADR   = LL_RMS_DATA_IN_ADR(p_cfg->dma_src_addr);
    p_rms->DATA_OUT_ADR  = LL_RMS_DATA_OUT_ADR(p_cfg->dma_dst_addr);
}

/**
  * @}
  */

/** @defgroup RMS_LL_Data_Transfers RMS LL Data transfers functions
  * @ingroup  RMS_LL_Driver
  * @brief    RMS LL Data transfers functions 
  * @{
  */

/**
  * @brief  RMS module start function
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval None
  */
void ll_rms_start(RMS_TypeDef *p_rms)
{
    TX_ASSERT((p_rms == RMS0) || (p_rms == RMS1) || (p_rms == RMS2));
    p_rms->CON |= LL_RMS_CON_ENABLE;
}

/**
  * @brief  RMS module stop function
  * @param  p_rms: Select the initialized RMS group pointer
  * @retval None
  */
void ll_rms_stop(RMS_TypeDef *p_rms)
{
    TX_ASSERT((p_rms == RMS0) || (p_rms == RMS1) || (p_rms == RMS2));
    p_rms->CON &= ~LL_RMS_CON_ENABLE;
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
