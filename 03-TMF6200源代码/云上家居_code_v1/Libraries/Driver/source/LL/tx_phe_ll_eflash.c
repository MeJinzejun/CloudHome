/**
  ******************************************************************************
  * @file    Libraries/Driver/source/LL/tx_phe_ll_eflash.c
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the EFLASH LL firmware functions.
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
#include "tx_phe_ll_eflash.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup eflash_interface_gr EFLASH Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup EFLASH_LL_Driver EFLASH LL Driver
  * @ingroup  eflash_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

  
/** @defgroup EFLASH_LL_Interrupt EFLASH LL Interrupt Handle function
  * @ingroup  EFLASH_LL_Driver
  * @brief   EFLASH LL Interrupt Handle function
  * @{
  */



/**
  * @}
  */

/** @defgroup EFLASH_LL_Inti_Cfg EFLASH LL Initialization And Configuration
  * @ingroup  EFLASH_LL_Driver
  * @brief    EFLASH LL Initialization And Configuration
  * @{
  */


/**
  * @brief  ll_eflash_init
  * @param  p_ef  : EFLASH_TypeDef pointer to eflash hardware register
  * @param  p_init: TYPE_LL_EFLASH_INIT pointer to eflash init stuct
  * @retval None
  * @note eflash clk 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_init(EFLASH_TypeDef *p_ef, TYPE_LL_EFLASH_INIT *p_init)
{
    TX_ASSERT(p_ef == EFLASH);
    ll_cc_rst_apb2_peripheral_clk_enable(LL_CC_RST_APB2_M_EFLASH);
    ll_cc_rst_softreset_release(LL_CC_RST_SOFTRST_M_EFLASH);
}

/**
  * @brief  ll_eflash_deinit
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  * @note eflash clk 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_deinit(EFLASH_TypeDef *p_ef)
{
    TX_ASSERT(p_ef == EFLASH);
    ll_cc_rst_softreset(LL_CC_RST_SOFTRST_M_EFLASH);
    ll_cc_rst_apb2_peripheral_clk_disable(LL_CC_RST_APB2_M_EFLASH);
}

/**
  * @brief  ll_eflash_config
  * @param  p_ef : EFLASH_TypeDef pointer to eflash hardware register
  * @param  p_cfg: TYPE_LL_EFLASH_CFG pointer to eflash init stuct
  * @retval None
  * @note eflash clk 29Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_config(EFLASH_TypeDef *p_ef, TYPE_LL_EFLASH_CFG *p_cfg)
{
    TX_ASSERT(p_ef == EFLASH);

    u32 read_cycles;
#if FPGA_EN
    read_cycles = (p_cfg->clk < 1000000) ? 0 : (((p_cfg->clk / 1000000) + 7)/8 - 1);
#else
    read_cycles = (p_cfg->clk < 1000000) ? 0 : (((p_cfg->clk / 1000000) + 28)/29 - 1);
#endif

    p_ef->TIME_REG0 = LL_EF_REG0_RC(read_cycles) |
                      LL_EF_REG0_RW(8)           |
                      LL_EF_REG0_ADH(1)          |
                      LL_EF_REG0_ADS(1)          |
                      LL_EF_REG0_PGH(1);

    p_ef->CTRLR0 = LL_EF_PG_CLK_SEL(LL_EF_PG_CLK_HXOSC)                        |    
                   (p_cfg->read_directly_en ? LL_EF_DIRCTLY_OUT : 0)             |
                   (p_cfg->cmd_cache_en ? LL_EF_BLK_REQ_MODE_EN : 0)             |
                   (LL_EF_CACHE_WRITEBACK_EN )                                   |
                   (p_cfg->data_endian_lit_en ? LL_EF_DATA_LITTLE_ENDIAN_EN : 0) |
                   (p_cfg->cache_en ? LL_EF_CACHE_EN : 0);

}

/**
  * @}
  */

/** @defgroup EFLASH_LL_Data_Transfers EFLASH LL Data transfers functions
  * @ingroup  EFLASH_LL_Driver
  * @brief    EFLASH LL Data transfers functions 
  * @{
  */
  
/**
  * @brief  ll_eflash_erase_chip
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  * @note eflash clk 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_erase_chip(EFLASH_TypeDef *p_ef)
{
    TX_ASSERT(p_ef == EFLASH);

    while(!(p_ef->DONE & LL_EF_CHIP_ERASE_DONE));
    
    ll_eflash_main_lock(p_ef, false);
    
    p_ef->ERASE_CTRL = LL_EF_CHIP_ERASE_KST;
}

/**
  * @brief  ll_eflash_erase_sector
  * @param  p_ef        : EFLASH_TypeDef pointer to eflash hardware register
  * @param  sector_index: 0 ~ 255, one sector is 512Byte
  * @retval None
  */
void ll_eflash_erase_sector(EFLASH_TypeDef *p_ef, u16 sector_index)
{
    TX_ASSERT(p_ef == EFLASH);
    TX_ASSERT(sector_index < LL_EF_SECTOR_NUMBERS);

    while((p_ef->DONE & LL_EF_SECT_ERASE_DONE) != LL_EF_SECT_ERASE_DONE);
    
    ll_eflash_main_lock(p_ef, false);
    
    p_ef->ERASE_CTRL = LL_EF_SECTOR_ERASE_KST | LL_EF_ERASE_SECTOR_ADDR(sector_index);
}

/**
  * @brief  ll_eflash_prog_word
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: Write data address in eflash, logical addr
  * @param  data: Write data to eflash
  * @retval None
  */
void ll_eflash_prog_word(EFLASH_TypeDef *p_ef, u32 addr, u32 data)
{
    TX_ASSERT(p_ef == EFLASH);
    TX_ASSERT(0 == (addr & 3));

    while(!(p_ef->DONE & LL_EF_PROG_DONE));
    
    ll_eflash_main_lock(p_ef, false);
    p_ef->PROG_ADDR = (addr + LL_EF_STADDR);
    p_ef->PROG_DATA = data;
}

/**
  * @brief  ll_eflash_erase_sector_nvr
  * @param  p_ef        : EFLASH_TypeDef pointer to eflash hardware register
  * @param  sector_index: LL_EF_NVR_SECTOR_NUMBERS, one sector is 512Byte
  * @retval None
  */
void ll_eflash_erase_sector_nvr(EFLASH_TypeDef *p_ef, u16 sector_index)
{
    TX_ASSERT(p_ef == EFLASH);
    TX_ASSERT(sector_index < LL_EF_NVR_SECTOR_NUMBERS);

    while((p_ef->DONE & LL_EF_SECT_ERASE_DONE) != LL_EF_SECT_ERASE_DONE);
    
    ll_eflash_nvr_lock(p_ef, false);
    
    p_ef->ERASE_CTRL = LL_EF_SECTOR_ERASE_KST | LL_EF_ERASE_SECTOR_ADDR(sector_index+LL_EF_SECTOR_NUMBERS);
}

/**
  * @brief  ll_eflash_prog_word
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: Write data address in NVR,logical addr
  * @param  data: Write data to eflash
  * @retval None
  */
void ll_eflash_prog_word_nvr(EFLASH_TypeDef *p_ef, u32 addr, u32 data)
{
    TX_ASSERT(p_ef == EFLASH);
    TX_ASSERT(0 == (addr & 3));
    TX_ASSERT( (addr < LL_EF_NVR_SECTOR_NUMBERS * LL_EF_SECTOR_SIZE) );

    while(!(p_ef->DONE & LL_EF_PROG_DONE));
    
    ll_eflash_nvr_lock(p_ef, false);
    p_ef->PROG_ADDR = (addr + LL_EF_NVR_STADDR);
    p_ef->PROG_DATA = data;
}

#if 0
/**
  * @brief  ll_eflash_crc32 : x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: 4byte unit
  * @param  len : 4byte unit
  * @retval crc32 result
  */
u32 ll_eflash_crc32(EFLASH_TypeDef *p_ef, u32 addr, u32 len)
{
    TX_ASSERT(p_ef == EFLASH);
    TX_ASSERT_ADDR(addr*SRAM_BUF_UINT_4BYTE, len*SRAM_BUF_UINT_4BYTE, 
                   ASSERT_EFLASH, SRAM_BUF_ALIGNMEMT_4BYTE_MASK);

    bool ef_cache_en = (bool)(p_ef->CTRLR0 & LL_EF_CACHE_EN);

    if(ef_cache_en) {
        ll_eflash_cache_disable(p_ef);
    }

#if 0    
    TX_ASSERT_ADDR(addr*SRAM_BUF_UINT_4BYTE,
                    len*SRAM_BUF_UINT_4BYTE,
                    ASSERT_EFLASH,
                    SRAM_BUF_ALIGNMEMT_4BYTE_MASK);
#endif

    p_ef->CRC_DMA = LL_EF_CRC_DMA_LEN(len) | LL_EF_CRC_DMA_ADDR(addr);
    p_ef->KST = LL_EF_CRC_DMA_KST | LL_EF_CRC_DMA_KST_EN;
    while(0 == (p_ef->DONE & LL_EF_CRC_DONE));

    if(ef_cache_en) {
        ll_eflash_cache_enable(p_ef);
    }
    return p_ef->CRC_OUT;
    
}
#endif

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
