/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_dftrans.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DFTRANS LL firmware functions.
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
#include "tx_phe_ll_dftrans.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup dftrans_interface_gr DFTRANS Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup DFTRANS_LL_Driver DFTRANS LL Driver
  * @ingroup  dftrans_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/*! Declare the calculation result buffer of DFTRAN0, 
 *  the address must be in SRAM1
 */
static u64 __ll_dftrans_0_out
       __attribute__((aligned(8),section("SRAM1")));

/*! Declare the calculation result buffer of DFTRAN1,
 *  the address must be in SRAM2
 */
static u64 __ll_dftrans_1_out
       __attribute__((aligned(8),section("SRAM2")));

/*! Declare the calculation result buffer of DFTRAN2,
 *  the address must be in SRAM3
 */
static u64 __ll_dftrans_2_out
       __attribute__((aligned(8),section("SRAM3")));

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup DFTRANS_LL_Interrupt DFTRANS LL Interrupt Handle function
  * @ingroup  DFTRANS_LL_Driver
  * @brief   DFTRANS LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup DFTRANS_LL_Inti_Cfg DFTRANS LL Initialization And Configuration
  * @ingroup  DFTRANS_LL_Driver
  * @brief    DFTRANS LL Initialization And Configuration
  * @{
  */
  
/**
  * @brief  DFTRAN module initialization function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  p_init  : Module configuration structure pointer
  * @retval None
  */
void ll_dftran_init(DFT_TypeDef *p_dftran, TYPE_LL_DFTRANS_INIT *p_init)
{
    /* The system resets the DFTRAN module, Enable the clock of the module. */
    TX_ASSERT((p_dftran == DFTRAN0) || (p_dftran == DFTRAN1) || (p_dftran == DFTRAN2));
}

/**
  * @brief  DFTRAN module detele initialization function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_deinit(DFT_TypeDef *p_dftran)
{
    /* The system disable the DFTRAN module, includes turning off the clock for the module. */
    TX_ASSERT((p_dftran == DFTRAN0) || (p_dftran == DFTRAN1) || (p_dftran == DFTRAN2));
}

/**
  * @brief  DFTRAN module configuration
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @param  p_cfg   : Module configuration structure pointer
  * @retval None
  */
void ll_dftran_config(DFT_TypeDef *p_dftran, TYPE_LL_DFTRANS_CFG *p_cfg)
{
    u32 normalized_coef = 0;
    u32 dftran_con      = 0;
    u32 dftran_out_addr = 0;
    u32 out_addr        = 0;
    
    TX_ASSERT((p_dftran == DFTRAN0) || (p_dftran == DFTRAN1) || (p_dftran == DFTRAN2));
    
    /* Check the parameters of DFTRAN input DMA buffer address */
    switch((u32)p_dftran) {
        case ((u32)DFTRAN0):
            /* DFTRANS0's real_dma_src_addr can only be located in SRAM1. */
            TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                           SRAM_DFT0_REAL_MASK, SRAM_DFT0_REAL_ALIGN_MASK);
            /* DFTRANS0's img_dma_src_addr can only be located in SRAM2. */
            if(p_cfg->img_val_sel == LL_DFTRAN_IMAG_NO_ZERO) {
                TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                               SRAM_DFT0_IMAG_MASK, SRAM_DFT0_IMAG_ALIGN_MASK);
            }
            if(p_cfg->dma_dst_addr != NULL) {
                /* DFTRANS0's dma_dst_addr can only be located in SRAM1. */
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, SRAM_BUF_UINT_8BYTE, 
                               SRAM_DFT0_REAL_MASK, SRAM_DFT0_OUT_ALIGN_MASK);
            }
            out_addr = (u32)&__ll_dftrans_0_out;
            break;
        
        case ((u32)DFTRAN1):
            /* DFTRANS1's real_dma_src_addr can only be located in SRAM2. */
            TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                           SRAM_DFT1_REAL_MASK, SRAM_DFT1_REAL_ALIGN_MASK);
            /* DFTRANS1's img_dma_src_addr can only be located in SRAM3. */
            if(p_cfg->img_val_sel == LL_DFTRAN_IMAG_NO_ZERO) {
                TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                               SRAM_DFT1_IMAG_MASK, SRAM_DFT1_IMAG_ALIGN_MASK);
            }
            if(p_cfg->dma_dst_addr != NULL) {
                /* DFTRANS1's dma_dst_addr can only be located in SRAM2. */
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, SRAM_BUF_UINT_8BYTE, 
                               SRAM_DFT1_REAL_MASK, SRAM_DFT1_OUT_ALIGN_MASK);
            }
            out_addr = (u32)&__ll_dftrans_1_out;
            break;
        
        case ((u32)DFTRAN2):
            /* DFTRANS2's real_dma_src_addr can only be located in SRAM3. */
            TX_ASSERT_ADDR(p_cfg->real_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                           SRAM_DFT2_REAL_MASK, SRAM_DFT2_REAL_ALIGN_MASK);
            /* DFTRANS2's img_dma_src_addr can only be located in SRAM1. */
            if(p_cfg->img_val_sel == LL_DFTRAN_IMAG_NO_ZERO) {
                TX_ASSERT_ADDR(p_cfg->img_dma_src_addr, p_cfg->dma_points * SRAM_BUF_UINT_2BYTE, 
                               SRAM_DFT2_IMAG_MASK, SRAM_DFT2_IMAG_ALIGN_MASK);
            }
            if(p_cfg->dma_dst_addr != NULL) {
                /* DFTRANS2's dma_dst_addr can only be located in SRAM3. */
                TX_ASSERT_ADDR(p_cfg->dma_dst_addr, (p_cfg->dma_dst_addr + SRAM_BUF_UINT_8BYTE), 
                               SRAM_DFT2_REAL_MASK, SRAM_DFT2_OUT_ALIGN_MASK);
            }
            out_addr = (u32)&__ll_dftrans_2_out;
            break;
        
        default:
            break;
    }
    
    /* Check the parameters */
    TX_ASSERT(p_cfg->dft_points <= 1024);
    
    /* Calculation DFTRAN Normalization coefficient */
    normalized_coef = (1 << 19) / p_cfg->dft_points;

    /* Calculation DFTRAN CON value */
    dftran_con      = p_dftran->CON & LL_DFTRAN_IE;
    dftran_con     |= LL_DFTRAN_DOWNSAMPLE_SEL(p_cfg->downsample_sel);
    dftran_con     |= p_cfg->step_dir ? LL_DFTRAN_ADD_MINUS : 0;
    dftran_con     |= p_cfg->dma_real_img_swap ? LL_DFTRAN_REAL_IMAG_EXCHANG : 0;
    dftran_con     |= p_cfg->img_val_sel ? LL_DFTRAN_IMAG_ZERO : 0;
    
    /* Calculation DFTRAN output bufffer address */
    dftran_out_addr = p_cfg->dma_dst_addr ? p_cfg->dma_dst_addr : out_addr;

    /* Configuring the DFTRAN register */
    p_dftran->CON             = dftran_con;
    p_dftran->LEN             = LL_DFTRAN_LEN(p_cfg->dma_points);
    p_dftran->INDEX           = LL_DFTRAN_INDEX(p_cfg->index);
    p_dftran->STEP            = LL_DFTRAN_STEP(p_cfg->step);
    p_dftran->REAL_STADR      = LL_DFTRAN_REAL_START(p_cfg->real_dma_src_addr);
    p_dftran->IMAG_STADR      = LL_DFTRAN_IMAG_START(p_cfg->img_dma_src_addr);
    p_dftran->DMA_LEN         = LL_DFTRAN_DMA_LEN(p_cfg->dft_points);
    p_dftran->NORMALIZED_COEF = (normalized_coef > 32767) ? 32767 : normalized_coef;
    p_dftran->OUT_ADR         = LL_DFTRAN_OUT_START(dftran_out_addr);
}

/**
  * @}
  */

/** @defgroup DFTRANS_LL_Data_Transfers DFTRANS LL Data transfers functions
  * @ingroup  DFTRANS_LL_Driver
  * @brief    DFTRANS LL Data transfers functions 
  * @{
  */

/**
  * @brief  DFTRAN module start function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_start(DFT_TypeDef *p_dftran)
{
    TX_ASSERT((p_dftran == DFTRAN0) || (p_dftran == DFTRAN1) || (p_dftran == DFTRAN2));
    p_dftran->CON |= LL_DFTRAN_ENABLE;
}

/**
  * @brief  DFTRAN module stop function
  * @param  p_dftran: Select the initialized DFTRAN group pointer
  * @retval None
  */
void ll_dftran_stop(DFT_TypeDef *p_dftran)
{
    TX_ASSERT((p_dftran == DFTRAN0) || (p_dftran == DFTRAN1) || (p_dftran == DFTRAN2));
    p_dftran->CON &= ~LL_DFTRAN_ENABLE;
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
