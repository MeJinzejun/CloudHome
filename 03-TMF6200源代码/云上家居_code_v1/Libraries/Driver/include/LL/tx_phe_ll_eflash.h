/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_eflash.h
  * @author  HUGE-IC Application Team
  * @version V1.0.2
  * @date    04-08-2019
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TX_PHE_LL_EFLASH_H
#define __TX_PHE_LL_EFLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup eflash_interface_gr EFLASH Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup EFLASH_LL_Driver EFLASH LL Driver
  * @ingroup  eflash_interface_gr
  * @brief Mainly the driver part of the EFLASH module, which includes \b EFLASH \b Register 
  * \b Constants, \b EFLASH \b Exported \b Constants, \b EFLASH \b Exported \b Struct, \b EFLASH
  * \b Data \b transfers \b functions, \b EFLASH \b Initialization \b and \b EFLASH \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup EFLASH_LL_Register_Constants EFLASH LL Register Constants
  * @ingroup  EFLASH_LL_Driver
  * @brief    EFLASH LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the EFLASH 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the EFLASH register, mainly for convenience. Understand the 
    configuration of the EFLASH.
    
@endverbatim
  *
  * @{
  */


/***** CTRLR0 Register *****/
/*! eflash_program_clk_sel : TYPE_ENUM_LL_EF_PG_CLK_SEL
 */
#define LL_EF_PG_CLK_SEL(n)                     (((n)&0x3) << 16)
/*! directly output, not use prefetch & cache
 */
#define LL_EF_DIRCTLY_OUT                       (1UL << 7)
/*! Program/Erase Request buffer mode
 */
#define LL_EF_BLK_REQ_MODE_EN                   (1UL << 5)
/*! Program/Sector Erase auto write back to cache
 */
#define LL_EF_CACHE_WRITEBACK_EN                (1UL << 4)
/*! Little-endian mode 
 */
#define LL_EF_DATA_LITTLE_ENDIAN_EN             (1UL << 3)
/*! Prefetch Enable
 */
#define LL_EF_PREFETCH_EN                       (1UL << 2)
/*! cache enable
 */
#define LL_EF_CACHE_EN                          (1UL << 0)


/***** KST Register *****/
/*! CRC DMA Kick Start EN 
 */
#define LL_EF_CRC_DMA_KST_EN                    (1UL << 26)
/*! Cache Clear Kick Start EN
 */
#define LL_EF_CACHE_CLR_KST_EN                  (1UL << 20)
/*! Deep Standby Kick Start EN
 */
#define LL_EF_DSTANDBY_KST_EN                   (1UL << 19)
/*! Wakeup Kick Start EN
 */
#define LL_EF_WAKEUP_KST_EN                     (1UL << 18)
/*! CRC DMA Kick Start
 */
#define LL_EF_CRC_DMA_KST                       (1UL << 10)
/*! Cache Clear Kick Start 
 */
#define LL_EF_CACHE_CLR_KST                     (1UL << 4)
/*! Deep Standby Kick Start
 */
#define LL_EF_DSTANDBY_KST                      (1UL << 3)
/*! Wakeup Kick Start
 */
#define LL_EF_WAKEUP_KST                        (1UL << 2)


/***** DONE Register *****/
/*! CRC done flag
 */
#define LL_EF_CRC_DONE                          (1UL << 10)
/*! Chip erase error flag
 */
#define LL_EF_CHIP_ERASE_ERR                    (1UL << 9)
/*! Sector erase error flag 
 */
#define LL_EF_SECTOR_ERASE_ERR                  (1UL << 8)
/*! Program finished done flag 
 */
#define LL_EF_PROG_DONE                         (1UL << 6)
/*! Program FIFO Done flag 
 */
#define LL_EF_PROG_FIFO_EMPTY                   (1UL << 5)
/*! Cache Clear Done flag 
 */
#define LL_EF_CACHE_CLR_DONE                    (1UL << 4)
/*! Deep Standby Done flag 
 */
#define LL_EF_DSTANDBY_DONE                     (1UL << 3)
/*! Wakeup Done flag 
 */
#define LL_EF_WAKEUP_DONE                       (1UL << 2)
/*! Chip Erase Done flag 
 */
#define LL_EF_CHIP_ERASE_DONE                   (1UL << 1)
/*! Sector Erase Done flag 
 */
#define LL_EF_SECT_ERASE_DONE                   (1UL << 0)
/*! eflash busy flag 
 */
#define LL_EF_BUSY_MASK                         (LL_EF_SECT_ERASE_DONE  |\
                                                 LL_EF_CHIP_ERASE_DONE  |\
                                                 LL_EF_CACHE_CLR_DONE   |\
                                                 LL_EF_PROG_DONE        |\
                                                 LL_EF_PROG_FIFO_EMPTY)


/***** PROG_ADDR Register *****/
/*! Program Burst Mode 
 */
#define LL_EF_PROG_BURST_MODE_EN                (1UL << 30)
/*! Program Byte num 
 */
#define LL_EF_PROG_BYTE(n)                      (((n)&0x3) << 24)
/*! Program Address 
 */
#define LL_EF_PROG_BYTE_ADDR(n)                 (((n)&0xFFFFFF) << 0)


/***** PROG_DATA Register *****/
/*! Eflash programming data, need to configure the address before proceeding 
 */
#define LL_EF_PROG_DATA(n)                      (((n)&0xFFFFFFFF) << 0)


/***** ERASE_CTRL Register *****/
/*! Chip full erase trigger, write "1" trigger, you need to configure the password
 */
#define LL_EF_CHIP_ERASE_KST                    (1UL << 31)
/*! Sector erase trigger, write "1" trigger,need to configure the password
 */
#define LL_EF_SECTOR_ERASE_KST                  (1UL << 30)
/*! Erase sector selection, range: 0-259  
                                   0-255 is the sector of the main area, 
                                   256-259 is the area of the nvr 
*/
#define LL_EF_ERASE_SECTOR_ADDR(n)              (((n)&0x1FF) << 0)


/***** TIME_REG0 Register *****/
/*! WEb low to PROG2 high hold min time is 15ns
 */
#define LL_EF_REG0_PGH(n)                       (((n)&0xF) << 16)
/*! BYTE/Address/data setup min time is 15ns 
 */
#define LL_EF_REG0_ADS(n)                       (((n)&0xF) << 12)
/*! BYTE/Address/data hold min time is 15ns 
 */
#define LL_EF_REG0_ADH(n)                       (((n)&0xF) << 8)
/*! Latency to next operation after PROG/ERASE low min time is 100ns 
 */
#define LL_EF_REG0_RW(n)                        (((n)&0xF) << 4)
/*! Read Cycle Min Time is 25/30ns 
 */
#define LL_EF_REG0_RC(n)                        (((n)&0xF) << 0)


/***** TIME_REG1 Register *****/
/*! 1ms time configuration value, in units of 1us 
 */
#define LL_EF_REG1_MS(n)                        (((n)&0x7FF) << 8)
 /*! 1us time configuration value, the system default is 26MHz 
  */
#define LL_EF_REG1_US(n)                        (((n)&0xFF) << 0)


/***** CRC_DMA Register *****/
/*! unit: 4byte 
 */
#define LL_EF_CRC_DMA_LEN(n)                    (((n)&0xFFFF) << 16)
/*! word address, phyic addr 
 */
#define LL_EF_CRC_DMA_ADDR(n)                   (((n)&0xFFFF) << 0)


/* start of eflash security area , not opened */
/***** EFLASH_STA Register *****/
/*! sector erase enable 
 */
#define LL_EF_SECTOR_ERASE_EN                   (1UL << 10)
/*! 1:128kB, 0:124kB 
 */
#define LL_EF_128KB_MODE                        (1UL << 8)
/*! 1 means cfg is all '1' 
 */
#define LL_EF_MAIN_CFG_EMPTY                    (1UL << 7)
/*! 1 means cfg is right, CRC ok 
 */
#define LL_EF_MAIN_CFG_OK                       (1UL << 6)
/*! 1 means NVR3 UUID CRC OK 
 */
#define LL_EF_UUID_OK                           (1UL << 5)
/*! 1 means chip is protected, can't read eflash code 
 */
#define LL_EF_PROTECT_EN                        (1UL << 4)
/*! 0 means enter eflash write mode 
 */
#define LL_EF_WRITE_MODE_DIS                    (1UL << 3)
/*! 1 means enter eflash normal mode 
 */
#define LL_EF_NOR_MODE_EN                       (1UL << 2)
/*! 1 means enter eflash spec erase mode, erase main/nvr 
 */
#define LL_EF_SPEC_ERASE_MODE_EN                (1UL << 1)
/*! 1 means eflash NVR4 is all '1' 
 */
#define LL_EF_BLANK_CHIP_FLAG                   (1UL << 0)


/***** PERMISSION0 Register *****/
/*! eflash nvr0/1/2 write permisson : bit field for sector permission 
 */
#define LL_EF_NVR_PERMISSION(n)                 (((n)&0x7) << 0)


/***** PERMISSION1 Register *****/
/*! eflash main array write permisson : bit field for sector permission 
 */
#define LL_EF_MAIN_PERMISSION(n)                (((n)) << 0)


/**
  * @}
  */

/** @defgroup EFLASH_LL_Exported_Constants EFLASH LL Exported Constants
  * @ingroup  EFLASH_LL_Driver
  * @brief    EFLASH LL external constant definition
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
/*! eflash sector size 
 */
#define LL_EF_SECTOR_SIZE           512
/*! eflash sector numbers 
 */
#define LL_EF_SECTOR_NUMBERS        256
/*! eflash sector numbers 
 */
#define LL_EF_NVR_SECTOR_NUMBERS    3
/*! eflash base address 
 */
#define LL_EF_STADDR                0x08000000
/*! eflash end address 
 */
#define LL_EF_ENDADDR               (LL_EF_STADDR + LL_EF_SECTOR_NUMBERS*LL_EF_SECTOR_SIZE)
/*! eflash NVR base address 
 */
#define LL_EF_NVR_STADDR            0x08020000
/*! eflash NVR end address 
 */
#define LL_EF_NVR_ENDADDR           (LL_EF_NVR_STADDR + LL_EF_NVR_SECTOR_NUMBERS*LL_EF_SECTOR_SIZE)
/*! eflash chip infor & param base address 
 */
#define LL_EF_CHIP_PARAM_STADDR     0x08020600
/*! eflash password for main 
 */
#define LL_EF_MAIN_PASSWORD         0x20170230
/*! eflash password for nvr 
 */
#define LL_EF_NVR_PASSWORD          0x20150931



/***** LL API *****/


/***** LL API AND DRIVER API *****/

/**
  * @brief EFLASH proramming clock selection
  */
typedef enum {
    /*! EFLASH proram clk : rc 8Mhz 
     */
    LL_EF_PG_CLK_IRC8M   = 0,
    /*! EFLASH proram clk : sys pll 
     */
    LL_EF_PG_CLK_SYS_PLL = 1,
    /*! EFLASH proram clk : adc pll 
     */
    LL_EF_PG_CLK_ADC_PLL = 2,
    /*! EFLASH proram clk : osc 26Mhz
     */
    LL_EF_PG_CLK_HXOSC   = 3,
} TYPE_ENUM_LL_EF_PG_CLK_SEL;

/**
  * @}
  */

/** @defgroup EFLASH_LL_Exported_Struct EFLASH LL Exported Struct
  * @ingroup  EFLASH_LL_Driver
  * @brief    EFLASH LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the EFLASH registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_eflash_init. Function, you can configure the EFLASH module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief configuration structure for saradc/fsaradc calibration
  */
typedef struct __ll_eflash_adc_calibration {
    /*! gain = gain_act/2
     */
    u16 gain;
    /*! dc offset 
     */
    u16 dc_offset;
} TYPE_LL_EFLASH_ADC_CALIBRATION;

/**
  * @brief configuration structure for chip infor & param
  */
typedef struct __ll_eflash_chip_param {
    /*! uuid
     */
    u32                             uuid[3];
    /*! fadc_calibration 
     */
    TYPE_LL_EFLASH_ADC_CALIBRATION  fadc_cab;
    /*!  temperature sensor offset :
     *      floor((0.00499*25+1.34125)/(3.0/4096)) - ts_adc_reg_data 
     */
    u32                             temperature_offset;
    /*! chip id
     */
    u32                             chip_id;
    /*! 
     */
    u32                             reserved;
    /*! eflash size : unit sector(512B)
     */
    u16                             eflash_size;
    /*! sram size : unit sector(512B)
     */
    u16                             sram_size;
    /*! crc32
     */
    u32                             crc32;
    /*! fsaradc coef
     */
    u32                             fsaradc_coef[16];
    /*! crc32
     */
    u32                             crc32_fsaradc_coef;
    /*! saradc_calibration 
     */
    TYPE_LL_EFLASH_ADC_CALIBRATION  saradc_cab[14];
    /*! crc32
     */
    u32                             crc32_saradc_cab;
} TYPE_LL_EFLASH_CHIP_PARAM;

/**
  * @brief configuration structure for low layer eflash configure
  */
typedef struct __ll_eflash_cfg {
    /*! eflash program clk sel TYPE_ENUM_LL_EF_PG_CLK_SEL 
     */
    TYPE_ENUM_LL_EF_PG_CLK_SEL  pgclk_src_sel;
    /*! eflash clk 
     */
    u32                         clk;
    /*! eflash data little endian mode enable 
     */
    bool                        data_endian_lit_en;
    /*! eflash erase & program cmd cache enable 
     */
    bool                        cmd_cache_en;
    /*! eflash prefetch enable 
     */
    bool                        prefetch_en;
    /*! eflash cache enable 
     */
    bool                        cache_en;
    /*! read direct from eflash, skip cache & prefetch 
     */
    bool                        read_directly_en;
} TYPE_LL_EFLASH_CFG;

/**
  * @brief initialization structure for low layer EFLASH 
  */ 
typedef struct __ll_eflash_init {
    u8 reserved;
} TYPE_LL_EFLASH_INIT;

/**
  * @}
  */

/** @defgroup EFLASH_LL_Interrupt EFLASH LL Interrupt Handle function
  * @brief   EFLASH LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EFLASH  
    Interrupt Handle function.

    how to use?

    The EFLASH interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the EFLASH in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup EFLASH_LL_Inti_Cfg EFLASH LL Initialization And Configuration
  * @brief    EFLASH LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EFLASH data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/**
  * @brief  ll_eflash_init
  * @param  p_ef     : EFLASH_TypeDef pointer to eflash hardware register
  * @param  p_init   : TYPE_LL_EFLASH_INIT pointer to eflash init stuct
  * @retval None
  * @note eflash clk : 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_init(EFLASH_TypeDef *p_ef, TYPE_LL_EFLASH_INIT *p_init);

/**
  * @brief  ll_eflash_deinit
  * @param  p_ef     : EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  * @note eflash clk : 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_deinit(EFLASH_TypeDef *p_ef);

/**
  * @brief  ll_eflash_config
  * @param  p_ef     : EFLASH_TypeDef pointer to eflash hardware register
  * @param  p_cfg    : TYPE_LL_EFLASH_CFG pointer to eflash init stuct
  * @retval None
  * @note eflash clk : 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_config(EFLASH_TypeDef *p_ef, TYPE_LL_EFLASH_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup EFLASH_LL_Data_Transfers EFLASH LL Data transfers functions
  * @brief    EFLASH LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the EFLASH data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */

/**
  * @brief  ll_eflash_erase_chip
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  * @note eflash clk 26Mhz safty, but Need to be set to 8M on the FPGA
  */
void ll_eflash_erase_chip(EFLASH_TypeDef *p_ef);

/**
  * @brief  ll_eflash_erase_chip
  * @param  p_ef        : EFLASH_TypeDef pointer to eflash hardware register
  * @param  sector_index: 0 ~ 255, one sector is 512Byte
  * @retval None
  */
void ll_eflash_erase_sector(EFLASH_TypeDef *p_ef, u16 sector_index);

/**
  * @brief  ll_eflash_prog_word
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: Write data address in eflash, logical addr
  * @param  data: Write data to eflash
  * @retval None
  */
void ll_eflash_prog_word(EFLASH_TypeDef *p_ef, u32 addr, u32 data);

/**
  * @brief  ll_eflash_erase_sector_nvr
  * @param  p_ef        : EFLASH_TypeDef pointer to eflash hardware register
  * @param  sector_index: LL_EF_NVR_SECTOR_NUMBERS, one sector is 512Byte
  * @retval None
  */
void ll_eflash_erase_sector_nvr(EFLASH_TypeDef *p_ef, u16 sector_index);

/**
  * @brief  ll_eflash_prog_word
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: Write data address in NVR,logical addr
  * @param  data: Write data to eflash
  * @retval None
  */
void ll_eflash_prog_word_nvr(EFLASH_TypeDef *p_ef, u32 addr, u32 data);

#if 0
/**
  * @brief  ll_eflash_crc32 : x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  addr: 4byte unit
  * @param  len :  4byte unit
  * @retval crc32 result
  */
u32 ll_eflash_crc32(EFLASH_TypeDef *p_ef, u32 addr, u32 len);
#endif

/**
  * @brief  ll_eflash_main_lock
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  lock: 1=lock
  * @retval None
  */
__STATIC_INLINE void ll_eflash_main_lock(EFLASH_TypeDef *p_ef, bool lock) {
    p_ef->MAIN_PASSWORD = lock ? 0 :LL_EF_MAIN_PASSWORD;
}

/**
  * @brief  ll_eflash_nvr_lock
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @param  lock: 1=lock
  * @retval None
  */
__STATIC_INLINE void ll_eflash_nvr_lock(EFLASH_TypeDef *p_ef, bool lock) {
    p_ef->NVR_PASSWORD = lock ? 0 :LL_EF_NVR_PASSWORD;
}

/**
  * @brief  ll_eflash_clear_cache
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_clear_cache(EFLASH_TypeDef *p_ef) {
    p_ef->KST |= LL_EF_CACHE_CLR_KST | LL_EF_CACHE_CLR_KST_EN;
    while(0 == (p_ef->DONE & LL_EF_CACHE_CLR_DONE));
}

/**
  * @brief  ll_eflash_cache_enable
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_cache_enable(EFLASH_TypeDef *p_ef) {
    p_ef->CTRLR0 |= LL_EF_CACHE_EN;
}

/**
  * @brief  ll_eflash_cache_disable
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_cache_disable(EFLASH_TypeDef *p_ef) {
    p_ef->CTRLR0 &= ~LL_EF_CACHE_EN;
}

/**
  * @brief  ll_eflash_prefetch_enable
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_prefetch_enable(EFLASH_TypeDef *p_ef) {
    p_ef->CTRLR0 |= LL_EF_PREFETCH_EN;
}

/**
  * @brief  ll_eflash_prefetch_disable
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_prefetch_disable(EFLASH_TypeDef *p_ef) {
    p_ef->CTRLR0 &= ~LL_EF_PREFETCH_EN;
}

/**
  * @brief  ll_eflash_read_directly_enable
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_read_directly_enable(EFLASH_TypeDef *p_ef) {
    p_ef->CTRLR0 = (p_ef->CTRLR0 & ~(LL_EF_PREFETCH_EN | LL_EF_CACHE_EN)) | LL_EF_DIRCTLY_OUT;
}

/**
  * @brief  LL_EFLASH_DATA
  * @param  addr: EFLASH main phy address (start form 0)
  * @retval None
  */
#define LL_EFLASH_DATA(addr)                      (*((volatile u32 *)((addr)+LL_EF_STADDR)))

/**
  * @brief  LL_EFLASH_NVR_DATA
  * @param  addr: EFLASH main nvr address (start form 0)
  * @retval None
  */
#define LL_EFLASH_NVR_DATA(addr)                  (*((volatile u32 *)((addr)+LL_EF_NVR_STADDR)))

/**
  * @brief  LL_EFLASH_IS_BUSY
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
#define LL_EFLASH_IS_BUSY(p_ef)                   ((LL_EF_BUSY_MASK == (p_ef->DONE & LL_EF_BUSY_MASK)) ? 0 : 1)

/**
  * @brief  Wait for the eflash to complete
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
__STATIC_INLINE void ll_eflash_wait_busy(EFLASH_TypeDef *p_ef) {
    while(LL_EF_BUSY_MASK != (p_ef->DONE & LL_EF_BUSY_MASK));
}

/**
  * @brief  LL_EFLASH_IS_SECTOR_ERASE_BUSY
  * @param  p_ef: EFLASH_TypeDef pointer to eflash hardware register
  * @retval None
  */
#define LL_EFLASH_IS_SECTOR_ERASE_BUSY(p_ef)      (!(p_ef->DONE & LL_EF_SECT_ERASE_DONE))

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

#endif //__TX_PHE_LL_EFLASH_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
