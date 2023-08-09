
/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_dmac.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DMAC LL firmware functions.
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
#include "tx_phe_ll_dmac.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup dmac_interface_gr DMAC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup DMAC_LL_Driver DMAC LL Driver
  * @ingroup  dmac_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u32 __dmac_src_addr_bak[LL_MAX_DMAC_CHN];
static u32 __dmac_dst_addr_bak[LL_MAX_DMAC_CHN];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup DMAC_LL_Interrupt DMAC LL Interrupt Handle function
  * @ingroup  DMAC_LL_Driver
  * @brief   DMAC LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup DMAC_LL_Inti_Cfg DMAC LL Initialization And Configuration
  * @ingroup  DMAC_LL_Driver
  * @brief    DMAC LL Initialization And Configuration
  * @{
  */

/**
  * @brief  ll_dmac_init
  * @param  p_dmac: pointer to the hardware DMAC_TypeDef
  * @param  p_init: pointer to the init stuct TYPE_LL_DMAC_INIT
  * @retval None
  */
void ll_dmac_init(DMAC_TypeDef * p_dmac, TYPE_LL_DMAC_INIT *p_init)
{
    TX_ASSERT(p_dmac == DMAC);

    /* reset dmac */
    NONSENSE(p_init);

    p_dmac->ClearBlockL   = LL_MAX_DMAC_CHN_MASK;
    p_dmac->ClearDstTranL = LL_MAX_DMAC_CHN_MASK;
    p_dmac->ClearErrL     = LL_MAX_DMAC_CHN_MASK;
    p_dmac->ClearSrcTranL = LL_MAX_DMAC_CHN_MASK;
    p_dmac->ClearTfrL     = LL_MAX_DMAC_CHN_MASK;

    /* disable DAMC */
    p_dmac->DmaCfgRegL = 0;
}

/**
  * @brief  ll_dmac_deinit
  * @param  p_dmac: pointer to the hardware DMAC_TypeDef
  * @retval None
  */
void ll_dmac_deinit(DMAC_TypeDef * p_dmac)
{
    TX_ASSERT(p_dmac == DMAC);

    /* reset dmac */

    /* disable DAMC */
    p_dmac->DmaCfgRegL = 0;
}

/**
  * @brief  DMAC channel interrupt configuration.
  * @param  p_dmac: DMAC module pointer.
  * @param  p_cfg : DMAC interrupt configuration pointer.
  * @retval None
  */
void ll_dmac_irq_config(DMAC_TypeDef *p_dmac, TYPE_LL_DMAC_IRQ_CFG *p_cfg)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(p_cfg->chn < LL_MAX_DMAC_CHN);

    DMAC_CH_TypeDef *p_dmac_ch = &(p_dmac->CH0);
    u32 chn = p_cfg->chn;
    
    if(p_cfg->xfer_cplt_intr_dis) {
        p_dmac->MaskTfrL = LL_DMAC_WRITE_DIS(chn);
    } else {
        p_dmac->MaskTfrL = LL_DMAC_WRITE_EN(chn);
    }
    
    if(p_cfg->block_intr_dis) {
        p_dmac->MaskBlockL = LL_DMAC_WRITE_DIS(chn);
    } else {
        p_dmac->MaskBlockL = LL_DMAC_WRITE_EN(chn);
    }
    
    if(p_cfg->src_xfer_cplt_intr_dis) {
        p_dmac->MaskSrcTranL = LL_DMAC_WRITE_DIS(chn);
    } else {
        p_dmac->MaskSrcTranL = LL_DMAC_WRITE_EN(chn);
    }
    
    if(p_cfg->dst_xfer_cplt_intr_dis) {
        p_dmac->MaskDstTranL = LL_DMAC_WRITE_DIS(chn);
    } else {
        p_dmac->MaskDstTranL = LL_DMAC_WRITE_EN(chn);
    }
    
    if(p_cfg->xfer_err_intr_dis) {
        p_dmac->MaskErrL = LL_DMAC_WRITE_DIS(chn);
    } else {
        p_dmac->MaskErrL = LL_DMAC_WRITE_EN(chn);
    }

    /* clear pending */
    p_dmac->ClearBlockL   = BIT(chn);
    p_dmac->ClearDstTranL = BIT(chn);
    p_dmac->ClearErrL     = BIT(chn);
    p_dmac->ClearSrcTranL = BIT(chn);
    p_dmac->ClearTfrL     = BIT(chn);
    
    if(p_cfg->intr_en) {
        p_dmac_ch[chn].CTLL |= LL_DWC_CTLL_INT_EN;
    }
}

/**
  * @brief  DMAC module configuration function.
  * @param  p_dmac: DMAC module pointer.DMAC_TypeDef
  * @param  p_cfg : DMAC configuration structure pointer.TYPE_LL_DMAC_CFG
  * @retval None
  */
void ll_dmac_config(DMAC_TypeDef *p_dmac, TYPE_LL_DMAC_CFG *p_cfg)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(p_cfg->chn < LL_MAX_DMAC_CHN);
    
    DMAC_CH_TypeDef *p_dmac_ch = &(p_dmac->CH0);
    u32 chn = p_cfg->chn;

    /* Configure the source address */
    p_dmac_ch[chn].SARH = 0x0000;
    p_dmac_ch[chn].SARL = p_cfg->src.addr;
    __dmac_src_addr_bak[chn] = p_cfg->src.addr;

    /* Configure the destination address */
    p_dmac_ch[chn].DARH = 0x0000;
    p_dmac_ch[chn].DARL = p_cfg->dst.addr;
    __dmac_dst_addr_bak[chn] = p_cfg->dst.addr;

    /* Configure block size */
    p_dmac_ch[chn].CTLH = p_cfg->element_num;
    
    /* According to different flow control */
    /* configure different AHB master and flow control type. */
    switch(p_cfg->flow_ctrl) {
        case LL_DW_DMA_FC_D_M2M:
            TX_ASSERT_ADDR((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->src.addr - p_cfg->element_num) : p_cfg->src.addr,
                           ((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           SRAM_GPDMA_MASK, 
                           SRAM_GPDMA_ALIGN_MASK);
            TX_ASSERT_ADDR((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->dst.addr - p_cfg->element_num) : p_cfg->dst.addr,
                           ((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           SRAM_GPDMA_MASK, 
                           SRAM_GPDMA_ALIGN_MASK);
            p_dmac_ch[chn].CTLL = LL_DWC_CTLL_SMS(1) | LL_DWC_CTLL_DMS(1) |\
                                  LL_DWC_CTLL_FC(LL_DW_DMA_FC_D_M2M);
            break;
        
        case LL_DW_DMA_FC_D_M2P:
            TX_ASSERT_ADDR((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->src.addr - p_cfg->element_num) : p_cfg->src.addr,
                           ((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           SRAM_GPDMA_MASK, 
                           SRAM_GPDMA_ALIGN_MASK);
            p_dmac_ch[chn].CTLL = LL_DWC_CTLL_SMS(1) | LL_DWC_CTLL_DMS(0) |\
                                  LL_DWC_CTLL_FC(LL_DW_DMA_FC_D_M2P);
            break;
        
        case LL_DW_DMA_FC_D_P2M:
            TX_ASSERT_ADDR((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->dst.addr - p_cfg->element_num) : p_cfg->dst.addr,
                           ((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           SRAM_GPDMA_MASK, 
                           SRAM_GPDMA_ALIGN_MASK);
            p_dmac_ch[chn].CTLL = LL_DWC_CTLL_SMS(0) | LL_DWC_CTLL_DMS(1) |\
                                  LL_DWC_CTLL_FC(LL_DW_DMA_FC_D_P2M);
            break;
        
        case LL_DW_DMA_FC_D_P2P:
            p_dmac_ch[chn].CTLL = LL_DWC_CTLL_SMS(0) | LL_DWC_CTLL_DMS(0) |\
                                  LL_DWC_CTLL_FC(LL_DW_DMA_FC_D_P2P);
            break;
        
        case LL_DW_DMA_FC_D_M2M_EFLASH:
            TX_ASSERT_ADDR((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->src.addr - p_cfg->element_num) : p_cfg->src.addr,
                           ((p_cfg->src.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           ASSERT_EFLASH, 
                           SRAM_GPDMA_ALIGN_MASK);
            TX_ASSERT_ADDR((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_DEC) ? (p_cfg->dst.addr - p_cfg->element_num) : p_cfg->dst.addr,
                           ((p_cfg->dst.addr_dir == LL_DW_DMAC_ADDR_NO_CHANGE) ? 1 : p_cfg->element_num) * (1<<p_cfg->element_per_width), 
                           SRAM_GPDMA_MASK & (~ASSERT_EFLASH), 
                           SRAM_GPDMA_ALIGN_MASK);
            
            p_dmac_ch[chn].CTLL = LL_DWC_CTLL_SMS(0) | LL_DWC_CTLL_DMS(1) |\
                                  LL_DWC_CTLL_FC(LL_DW_DMA_FC_D_M2M);
            break;
            
        default :
            p_dmac_ch[chn].CTLL = 0;
            break;
    }
    
    /* MSIZE is fixed at 1byte. */
    /* Configure source & destination address how to change. */
    /* Configure source & destination TR_WIDTH. */
    p_dmac_ch[chn].CTLL |= LL_DWC_CTLL_SRC_MSIZE(LL_DW_DMAC_MSIZE_1)       |\
                           LL_DWC_CTLL_DST_MSIZE(LL_DW_DMAC_MSIZE_1)       |\
                           LL_DWC_CTLL_SRC_DIR(p_cfg->src.addr_dir)        |\
                           LL_DWC_CTLL_DST_DIR(p_cfg->dst.addr_dir)        |\
                           LL_DWC_CTLL_SRC_WIDTH(p_cfg->element_per_width) |\
                           LL_DWC_CTLL_DST_WIDTH(p_cfg->element_per_width);

                           /* Configure dma req channel */
    p_dmac_ch[chn].CFGH  = LL_DWC_CFGH_DST_PER(p_cfg->dst.interface_chn) |\
                           LL_DWC_CFGH_SRC_PER(p_cfg->src.interface_chn) |\
                           LL_DWC_CFGH_PROTCTL(1);

                           /* AMBA burst length no limited. */
    p_dmac_ch[chn].CFGL  = LL_DWC_CFGL_MAX_BURST(0);

    /* clear pending */
    p_dmac->ClearBlockL   = BIT(chn);
    p_dmac->ClearDstTranL = BIT(chn);
    p_dmac->ClearErrL     = BIT(chn);
    p_dmac->ClearSrcTranL = BIT(chn);
    p_dmac->ClearTfrL     = BIT(chn);
        
    /* enable DAMC */
    p_dmac->DmaCfgRegL    = LL_DW_CFG_DMA_EN;
}

/**
  * @}
  */

/** @defgroup DMAC_LL_Data_Transfers DMAC LL Data transfers functions
  * @ingroup  DMAC_LL_Driver
  * @brief    DMAC LL Data transfers functions 
  * @{
  */

/**
  * @brief  Start a specific DMA channel.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval None
  */
void ll_dmac_start(DMAC_TypeDef *p_dmac, u32 chn)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(chn < LL_MAX_DMAC_CHN);
    
    p_dmac->ChEnRegL = LL_DMAC_WRITE_EN(chn);
}

/**
  * @brief  Disable a specific DMA channel.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval None
  */
void ll_dmac_stop(DMAC_TypeDef *p_dmac, u32 chn)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(chn < LL_MAX_DMAC_CHN);
    
    p_dmac->ChEnRegL = LL_DMAC_WRITE_DIS(chn);
}

/**
  * @brief  Wait for the DMAC channel to end the operation.
  * @param  p_dmac: DMAC module pointer.
  * @param  chn   : DMAC channel number.
  * @retval Returns true if the DMAC channel is normal and false if it
  *         represents an error in the DMAC channel.
  */
bool ll_dmac_wait_completed(DMAC_TypeDef *p_dmac, u32 chn)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(chn < LL_MAX_DMAC_CHN);
    
    while(((p_dmac->RawTfrL & BIT(chn)) == 0) &&
          ((p_dmac->RawErrL & BIT(chn)) == 0));
    
    if((p_dmac->RawErrL & BIT(chn))) {
        return false;
    } else {
        return true;
    }
}

/**
  * @brief  DMAC's gather function settings
  * @param  p_dmac: DMAC module pointer
  * @param  chn   : DMAC channel number
  * @param  sgc   : The amount of data continuously acquired
  * @param  sgi   : Interval of data
  * @retval None
  * @note   The amount of data that sgc and sgi together is taken as a unit.
  *         The sgi data will be discarded.
  */
void ll_dmac_src_gather_config(DMAC_TypeDef *p_dmac, u32 chn, u32 sgc, u32 sgi)
{
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(chn < LL_MAX_DMAC_CHN);
    
    DMAC_CH_TypeDef *p_dmac_ch = &(p_dmac->CH0);
    p_dmac_ch[chn].CTLL |= LL_DWC_CTLL_S_GATH_EN;
    p_dmac_ch[chn].SGRL  = LL_DWC_SGR_SGC(sgc) | LL_DWC_SGR_SGI(sgi);
}

/**
  * @brief  Return the DMA xfered length when dma is complete or abort
  * @param  p_dmac: DMAC module pointer
  * @param  chn   : DMAC channel number
  * @retval DMA xfered length, 0 will set when both src&dst dma address both not change
  */
uint32_t ll_dma_get_xfer_len(DMAC_TypeDef *p_dmac, u8 chn)
{
    /* parameters Check */
    TX_ASSERT(p_dmac == DMAC);
    TX_ASSERT(chn < LL_MAX_DMAC_CHN);

    DMAC_CH_TypeDef *p_dmac_ch = &(p_dmac->CH0);

    u32 src_cnt, dst_cnt;

    src_cnt = p_dmac_ch[chn].SARL - __dmac_src_addr_bak[chn];
    dst_cnt = p_dmac_ch[chn].DARL - __dmac_dst_addr_bak[chn];
    if(src_cnt > dst_cnt) {
        return src_cnt;
    }
    return dst_cnt;
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
