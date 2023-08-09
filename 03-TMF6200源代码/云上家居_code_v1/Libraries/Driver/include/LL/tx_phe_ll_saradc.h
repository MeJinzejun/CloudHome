/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_saradc.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    04-08-2019
  * @brief   This file contains all the SARADC LL firmware functions.
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
#ifndef __TX_PHE_LL_SARADC_H
#define __TX_PHE_LL_SARADC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup saradc_interface_gr SARADC Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup SARADC_LL_Driver SARADC LL Driver
  * @ingroup  saradc_interface_gr
  * @brief Mainly the driver part of the SARADC module, which includes \b SARADC \b Register 
  * \b Constants, \b SARADC \b Exported \b Constants, \b SARADC \b Exported \b Struct, \b SARADC
  * \b Data \b transfers \b functions, \b SARADC \b Initialization \b and \b SARADC \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */
     
/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
     
/** @defgroup SARADC_LL_Register_Constants SARADC LL Register Constants
  * @ingroup  SARADC_LL_Driver
  * @brief    SARADC LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the SARADC 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the SARADC register, mainly for convenience. Understand the 
    configuration of the SARADC.
    
@endverbatim
  *
  * @{
  */
  
/***** SARADC_CON(Register) *****/
/*! ADC18~ADC0 enable control
 */
#define LL_SARADC_CH_EN(n)                  (1UL << ((n)))


/***** SARADC_CON1(Register) *****/
/*! ADC13~ADC0 DMA enable control
 */
#define LL_SARADC_CH_DMA_EN(n)              (1UL << ((n)))


/***** SARADC_PENDING0(Register) *****/
/*! ADC18-0 sample done pending
 */
#define LL_SARADC_CH_SAMPLE_DONE(n)         (1UL << ((n)))


/***** SARADC_PENDING1(Register) *****/
/*! ADC13-0 DMA half done pending
 */
#define LL_SARADC_CH_HALF_DONE(n)           (1UL << ((n)))


/***** SARADC_PENDING2(Register) *****/
/*! ADC13-0 DMA full done pending
 */
#define LL_SARADC_CH_FULL_DONE(n)           (1UL << ((n)))


/***** SARADC_PENDING0_CLR(Register) *****/
/*! ADC18-0 sample done pending clear
 */
#define LL_SARADC_SAMPLE_DONE_PD_CLR(n)     (1UL << ((n)))


/***** SARADC_PENDING1_CLR(Register) *****/
/*! ADC13-0 DMA half pending clear
 */
#define LL_SARADC_HALF_DMA_PD_CLR(n)        (1UL << ((n)))


/***** SARADC_PENDING2_CLR(Register) *****/
/*! ADC13-0 DMA full pending clear
 */
#define LL_SARADC_FULL_DMA_PD_CLR(n)        (1UL << ((n)))


/***** SARADC_INT_CONTROL0(Register) *****/
/*! ADC18~ADC0 sample done interrupt enable
 */
#define LL_SARADC_SAMPLE_DONE_INT_EN(n)     (1UL << ((n)))


/***** SARADC_INT_CONTROL1(Register) *****/
/*! ADC18~ADC0 DMA half interrupt enable
 */
#define LL_SARADC_HALF_DONE_INT_EN(n)       (1UL << ((n)))


/***** SARADC_INT_CONTROL2(Register) *****/
/*! ADC18~ADC0 DMA full interrupt enable
 */
#define LL_SARADC_FULL_DONE_INT_EN(n)       (1UL << ((n)))


/***** SARADC_CDIV_DMALEN[0 ~ 18](Register) ADC_CH_DMA_LEN is valid in ch[0-13] *****/
/*! Sample rate control.  
 *  If the system clock is K MHz:    sample rate =K MHz / CONVERT_DIV
 */
#define LL_SARADC_CH_DIV(n)                 (((n)&0x3FFFF) << 14)
/*! Convert counter enable
 */
#define LL_SARADC_CH_DIV_EN                 (1UL << 13)
#define LL_SARADC_CH_DIV_DIS                (0UL << 13)
/*! DMA length configure. The unit is byte
 */
#define LL_SARADC_CH_DMA_LEN(n)             (((n)&0x1FFF) << 0)


/***** SARADC_COM_ACFG(Register) *****/
/*! Analog clock divider control  
 *  ADC clock frequency = System clock frequency/CLOCK_DIV
 */
#define LL_SARADC_CLK_DIV(n)                (((n)&0xFF) << 0)
/*! Analog debug output select.  
 *  0000 : sar0_dbug_vdd  
 *  0001 : sar1_dbug_vdd  
 *  ...  
 *  1101 : sar13_dbug_vdd  
 */
#define LL_SARADC_DBG_SEL(n)                (((n)&0xF) << 8)
/*! Analog Vref enable.
 */
#define LL_SARADC_SARVREF_EN(n)             (((n)&0x3) << 12)
/*! Analog test data.
 */
#define LL_SARADC_K(n)                      (((n)&0xFFF) << 16)


/***** SARADC_ACFG0~13(Register) *****/
/*! Analog ADC enable
 */
#define LL_SARADC_ANALOG_EN                 (1UL << 0)
/*! K select. SARADC's sub_dac code selection bit  
 *  0: use SAR Logic output data  
 *  1: use SARADC_K[11:0]  
 */
#define LL_SARADC_KSEL                      (1UL << 1)
/*! PGA enable
 */
#define LL_SARADC_PGA_EN                    (1UL << 2)
/*! Analog PGA switch.  
    PGA offset trim signal;1'b1=disconnect PAD<-x->PGA, connect PAD<--->VREF_BUF
*/
#define LL_SARADC_ANALOG_PGA_SW             (1UL << 3)
/*! Analog timing setting. sample time selection bit  
 *  0: 2 clock cycle  
 *  1: 4 clock cycle  
 */
#define LL_SARADC_STIMESET                  (1UL << 4)
/*! Analog test select.  
 *  SARADC's analog debug signal selection:  
 *  ATSEL [3:2] be reserved  
 *  ATSEL [1:0]  
 *  00: No signal be pushed to Analog test PIN  
 *  01: DAC_OUT be pushed  to Analog test PIN  
 *  10: VREF_BUF be pushed to Analog test PIN  
 *  11:  AVDD_SAR be pushed to Analog test PIN  
 */
#define LL_SARADC_ATSEL(n)                  (((n)&0xF) << 5)
/*! PGA GAIN select.  
 *  3'b000=0.5x;  
 *  3'b001=1x;  
 *  3'b010=1.5x;  
 *  3'b011=2x;  
 *  3'b100=3x;  
 *  3'b101=4x;  
 *  3'b110=6x;  
 *  3'b111=8x;  
 */
#define LL_SARADC_GAIN(n)                   (((n)&0x7) << 9)
/*! adc average enable
 */
#define LL_SARADC_AVERAGE_EN                (1UL << 13)
/*! ADC sample data average option  
 *  00 : 2 sequential sample data averaging  
 *  01 : 4 sequential sample data averaging  
 *  10 : 8 sequential sample data averaging  
 *  11 : reserved  
 */
#define LL_SARADC_AVERAGE(n)                (((n)&0x3) << 14)
/*! adc sample_data minus this & shift left 4 bit save to ADC_DATA
 */
#define LL_SARADC_DC_OFFSET(n)              (((n)&0xFFF) << 16)
/*! ADC sample data down sample rate: div by (n+1)  
 *  000 : no down sample  
 *  001 : down sample 2  
 *  010 : down sample 3  
 *  011 : down sample 4  
 *  100 : down sample 5  
 *  101 : down sample 6  
 *  110 : down sample 7  
 *  111 : down sample 8  
 */
#define LL_SARADC_DOWNSAMPLE(n)             (((n)&0x7) << 28)


/***** SARADC_ACFG14~18(Register) *****/
/*! Analog ADC enable
 */
#define LL_SARADC_ANALOG_EN                 (1UL << 0)
/*! Analog DAC enable
 */
#define LL_SARADC_DAC_EN                    (1UL << 1)
/*! Analog CMP enable
 */
#define LL_SARADC_CMP_EN                    (1UL << 2)
/*! Analog set
 */
#define LL_SARADC_ANALOG_SET(n)             (((n)&0x7) << 3)
/*! adc or cmp mode input channel select
 */
#define LL_SARADC_AUX_EN(n)                 (((n)&0xF) << 6)
/*! adc average enable
 */
#define LL_SARADC_AVERAGE_EN                (1UL << 13)
/*! adc average select
 */
#define LL_SARADC_AVERAGE(n)                (((n)&0x3) << 14)
/*! adc dc offset set
 */
#define LL_SARADC_DC_OFFSET(n)              (((n)&0xFFF) << 16)
/*! adc sample times div by (n+1)
 */
#define LL_SARADC_DOWNSAMPLE(n)             (((n)&0x7) << 28)


/***** SARADC_QUANTIFY_CON0~13(Register) *****/
/*! adc quantify enable
 */
#define LL_SARADC_QUANTIFY_EN               (1UL << 0)
/*! Quantify output data remain how many decimal width  
 *  000 : 0 bit decimal width  
 *  ...  
 *  111 : 7 bit decimal width  
 */
#define LL_SARADC_QUANTIFY_DECIMAL_WIDTH(n) (((n)&0x7) << 1)
/*! Quantify coef,unsigned, 16bit data width, 15bit is decimal width
 */
#define LL_SARADC_QUANTIFY_COEF(n)          (((n)&0xFFFFUL) << 16)


/***** SARADC_START_POINT0~13(Register) *****/
/*! Convert counter coUnt to this value convert kick start 
 */
#define LL_SARADC_START_POINT(n)            (((n)&0x3FFFF) << 0)


/***** DACCMP_CON14~18(Register) *****/
/*! WO write 1 : clear DAC_DONE
 */
#define LL_SARADC_DAC_DONE_CLR                  (1UL << 28)
/*! RO DAC data output done
 */
#define LL_SARADC_DAC_DONE                      (1UL << 15)
/*! RW Comparator interrupt enable
 */
#define LL_SARADC_CMP_INT_EN                    (1UL << 14)
/*! RW DAC interrupt enable
 */
#define LL_SARADC_DAC_INT_EN                    (1UL << 13)
/*! RW CMP_TO_IO source invert enable
 */
#define LL_SARADC_CMPTOIO_INV_EN                (1UL << 12)
/*! RW CMP_TO_IO select : 0-Analog 1-Debounce
 */
#define LL_SARADC_CMPTOIO_SEL(n)                (((n)&0x1) << 11)
/*! RW Comparator output to CPU
 */
#define LL_SARADC_CMP_RESULT                    (1UL << 10)
/*! RW CMP_TO_CPU source invert enable
 */
#define LL_SARADC_CMPTOCPU_INV_EN               (1UL << 9)
/*! RW CMP_TO_CPU select : 0-Analog 1-Debounce
 */
#define LL_SARADC_CMPTOCPU_SEL(n)               (((n)&0x1) << 8)
/*! RW CMP_TO_DATA select 0-DATA0 1-DATA1
 */
#define LL_SARADC_CMP_DATA_SEL(n)               (((n)&0x1) << 7)


/***** DACCMP_DATA14~18(Register) *****/
/*! RW DATA output to DAC and CMP ananlog, when CMP_TO_DATA value is 0, output DACCMP_DATA0
 */
#define LL_SARADC_CMP_DATA0(n)                  (((n)&0xFFF) << 0)
/*! RW DATA output to DAC and CMP ananlog, when CMP_TO_DATA value is 1, output DACCMP_DATA1
 */
#define LL_SARADC_CMP_DATA1(n)                  (((n)&0xFFF) << 16)


/***** DACCMP_DEBOUNCE14~18(Register) *****/
/*! Comparator output debounce select
 */
#define LL_SARADC_CMP_DEBOUNCE(n)               (((n)&0xFFFF) << 0)

/**
  * @}
  */

/** @defgroup SARADC_LL_Exported_Constants SARADC LL Exported Constants
  * @ingroup  SARADC_LL_Driver
  * @brief    SARADC LL external constant definition
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
/*! saradc max channel counter
 */
#define LL_SARADC_CHN_MAX                         19
/*! saradc max adc channel counter
 * @note  saradc adc channel voltage range is [0, 3.0v]
 *        saradc adc data format[1, 16, 15]  
 */
#define LL_SARADC_ADC_CHN_MAX                     14
/*! saradc max dac channel counter
 * @note  saradc adc/dac/cmp channel voltage range is [0, 3.3v]
 *        saradc adc data format[1, 16, 15]  
 *        saradc dac data format[0, 12, 12]  
 *        saradc cmp data format[0, 12, 12]  
 */
#define LL_SARADC_DAC_CHN_MAX                     (LL_SARADC_CHN_MAX-LL_SARADC_ADC_CHN_MAX)


/***** LL API *****/

/**
  * @brief SARADC low layer sample data average times typedef
  */
typedef enum {
    /*! saradc average point select: 1
     */
    LL_SARADC_AVERAGE_1 = 3,
    /*! saradc average point select: 2
     */
    LL_SARADC_AVERAGE_2 = 0,
    /*! saradc average point select: 4
     */
    LL_SARADC_AVERAGE_4 = 1,
    /*! saradc average point select: 8
     */
    LL_SARADC_AVERAGE_8 = 2,
} TYPE_ENUM_LL_SARADC_AVERAGE_SEL;

/**
  * @brief SARADC low layer downsample times typedef
  */
typedef enum {
    /*! saradc down sample select: 1
     */
    LL_SARADC_DOWNSAMPLE_1 = 0,
    /*! saradc down sample select: 2
     */
    LL_SARADC_DOWNSAMPLE_2,
    /*! saradc down sample select: 3
     */
    LL_SARADC_DOWNSAMPLE_3,
    /*! saradc down sample select: 4
     */
    LL_SARADC_DOWNSAMPLE_4,
    /*! saradc down sample select: 5
     */
    LL_SARADC_DOWNSAMPLE_5,
    /*! saradc down sample select: 6
     */
    LL_SARADC_DOWNSAMPLE_6,
    /*! saradc down sample select: 7
     */
    LL_SARADC_DOWNSAMPLE_7,
    /*! saradc down sample select: 8
     */
    LL_SARADC_DOWNSAMPLE_8,
} TYPE_ENUM_LL_SARADC_DOWNSAMPLE_SEL;

/**
  * @brief SARADC low layer sample gain typedef
  */
typedef enum {
    /*! saradc gain select: 0.5
     */
    LL_SARADC_GAIN_0_5 = 0,
    /*! saradc gain select: 1.0
     */
    LL_SARADC_GAIN_1_0,
    /*! saradc gain select: 1.5
     */
    LL_SARADC_GAIN_1_5,
    /*! saradc gain select: 2.0
     */
    LL_SARADC_GAIN_2_0,
    /*! saradc gain select: 3.0
     */
    LL_SARADC_GAIN_3_0,
    /*! saradc gain select: 4.0
     */
    LL_SARADC_GAIN_4_0,
    /*! saradc gain select: 6.0
     */
    LL_SARADC_GAIN_6_0,
    /*! saradc gain select: 8.0
     */
    LL_SARADC_GAIN_8_0,
} TYPE_ENUM_LL_SARADC_GAIN_SEL;
    
/**
  * @brief SARADC low layer ad/compare source typedef
  */
typedef enum {
    /*! saradc compare select: AIN0 : PA0(ADC14)  PA4(ADC15)  PA8(ADC16)  PA12(ADC17)  PD1(ADC18)
     */
    LL_SARADDA_AIN0 = 1,
    /*! saradc compare select: AIN1 : PA1(ADC14)  PA5(ADC15)  PA9(ADC16)  PA13(ADC17)  PD2(ADC18)
     */
    LL_SARADDA_AIN1 = 2,
    /*! saradc compare select: AIN2 : PA2(ADC14)  PA6(ADC15)  PA10(ADC16) PA14(ADC17)  PD3(ADC18)
     */
    LL_SARADDA_AIN2 = 4,
    /*! saradc compare select: AIN3 :                                     PA15(ADC17)  PD4(ADC18)
     */
    LL_SARADDA_AIN3 = 8,
} TYPE_ENUM_LL_SARADDA_AUX_SEL;

/**
  * @brief SARADC low layer compare output typedef
  */
typedef enum {
    /*! saradc compare out select: analog
     */
    LL_SARADC_CMP_OUT_ANALOG =  0x00,
    /*! saradc compare out select: debunce
     */
    LL_SARADC_CMP_OUT_DEBUNCE = 0x01,
} TYPE_ENUM_LL_SARADDA_CMP_OUT_TYPE_SEL;

/**
  * @brief SARADC low layer compare destination typedef
  */
typedef enum {
    /*! saradc compare out des: io+cpu
     */
    LL_SARADC_CMP_TO_IO_AND_CPU = 0,
    /*! saradc compare out des: cpu
     */
    LL_SARADC_CMP_TO_CPU        = 1,
} TYPE_ENUM_LL_SARADDA_CMP_DST_SEL;

/**
  * @brief SARADC low layer compare DATA typedef
  */
typedef enum {
    /*! saradc compare source sel: data0
     */
    LL_SARADC_CMP_SRC_DAT0 = 0,
    /*! saradc compare source sel: data1
     */
    LL_SARADC_CMP_SRC_DAT1 = 1,
} TYPE_ENUM_LL_SARADDA_CMP_SRC_SEL;

/***** LL API AND DRIVER API *****/


/**
  * @}
  */

/** @defgroup SARADC_LL_Exported_Struct SARADC LL Exported Struct
  * @ingroup  SARADC_LL_Driver
  * @brief    SARADC LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the SARADC registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_saradc_init. Function, you can configure the SARADC module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief SARADC low layer adc irq_config struct
  */
typedef struct __ll_saradc_irq_cfg {
    /*! adc channel index : 0 ~ SARADC_CHN_CNT_MAX
     */
    u8   chn;
    /*! adc sample done interrupt enable
     */
    bool sample_intr_en;
    /*! adc dma_half interrupt enable
     */
    bool dma_half_intr_en;
    /*! adc dma_full interrupt enable
     */
    bool dma_full_intr_en;
    /*! dac mode interrupt enable
     */
    bool dac_intr_en;
    /*! cmp mode interrupt enable
     *  cmp interrupt usable only when adc & dac interrupt is disabled
     */
    bool cmp_intr_en;
} TYPE_LL_SARADC_IRQ_CFG ;

/**
  * @brief SARADC low layer adc config struct
  */
typedef struct __ll_saradc_cfg {
    /*! adc channel index : 0 ~ LL_SARADC_CHN_MAX
     */
    u8                                  chn;
    /*! average points: 0/2/4/8
     */
    TYPE_ENUM_LL_SARADC_AVERAGE_SEL     average; 
    /*! down_sample select: 1/2/3/4/5/6/7/8
     */
    TYPE_ENUM_LL_SARADC_DOWNSAMPLE_SEL  downsample;
    /*! sample_rate_div = SYS_CLK/sample_rate
     */
    u32                                 sample_rate_div;
    /*! adc chn gain setting  :TYPE_ENUM_SARADC_GAIN_SEL, use for 0 ~ LL_SARADC_ADC_CHN_MAX
     *  @note : use for channel 0 ~ LL_SARADC_ADC_CHN_MAX
     */
    TYPE_ENUM_LL_SARADC_GAIN_SEL        gain;
    /*! Convert kick start by external module (spwm,svpwm,epwm .ect) enable
     */    
    u8                                  adc_kick_by_extern_en;
    /*! adc chn dc offset, sample data = adc_inter_val- dc_offset
     */
    u16                                 dc_offset;
    /*! Convert counter coUnt to this value convert kick start
     */
    u32                                 start_point;
    /*! Quantify coef,unsigned, 16bit data width, 15bit is decimal width
     */
    u16                                 quantify_coef;
    /*! Quantify output data remain how many decimal width  
     *  000 : 0 bit decimal width  
     *  ...  
     *  111 : 7 bit decimal width  
     */    
    u16                                 quantify_out_decimal_width;
    /*! adc quantify enable
     *  @note usage :  
     *        use for channel 0 ~ LL_SARADC_ADC_CHN_MAX
     */
    u8                                  quantify_en;
    /*! adc dma start address: SRAM0/SRAM10/SRAM1/SRAM2/SRAM3  aligned(2)
     *  @note usage :  
     *        s16 saradc_buf[256] __attribute__((aligned(2), section("SRAM1")));
     *        use for channel LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     */
    u32                                 dma_addr;
    /*! adc dma length, unit byte, 12bit
     *  @note : use for channel LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     *          a zero dma_len means dma is disabled
     */
    u16                                 dma_len;
    /*! adc input channel select, use for channel LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     *  @note : use for channel LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     */
    TYPE_ENUM_LL_SARADDA_AUX_SEL        aux_sel;
    
} TYPE_LL_SARADC_CFG ;

/**
  * @brief SARADC low layer adc_dac config struct
  */
typedef struct __ll_saradc_dac_cfg {
    /*! adc channel index : LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     *  channel[14,18] IO : PA3/PA7/PA11/PD0/PD5
     */
    u8                                  chn;
    /*! dac_updata_data_rate_div = SYS_CLK/updata_data_rate
     */
    u32                                 updata_data_rate_div;
    /*! dac kick start by external module (spwm,svpwm,epwm .ect) enable
     */    
    bool                                dac_kick_by_extern_en;        
    /*! Convert counter count to this value convert kick start
     */
    u32                                 start_point;
} TYPE_LL_SARADC_DAC_CFG ;

/**
  * @brief SARADC low layer cmp config struct
  */
typedef struct __ll_saradc_cmp_cfg {
    /*! adc channel index : LL_SARADC_ADC_CHN_MAX ~ LL_SARADC_CHN_MAX
     */
    u8                                    chn;
    /*! cmp mode input channel select
     */
    TYPE_ENUM_LL_SARADDA_AUX_SEL          aux_sel;
    /*! Comparator output debounce select  
     *  0: debounce disable;  
     *  1: CMP level change must assert unless 2 clock;  
     *  ...  
     *  65535: CMP level change must assert unless 65536 clock  
     */
    u16                                   cmp_out_debunce_cnt;
    /*! cmp mode des select: cpu or cpu&io
     */
    TYPE_ENUM_LL_SARADDA_CMP_DST_SEL      cmp_des_sel;
    /*! cmp mode des_io config: anolog or debunce, both can be invert
     */
    TYPE_ENUM_LL_SARADDA_CMP_OUT_TYPE_SEL cmp_des_type_sel_io;
    /*! cmp mode des_cpu config: anolog or debunce, both can be invert
     */
    TYPE_ENUM_LL_SARADDA_CMP_OUT_TYPE_SEL cmp_des_type_sel_cpu;
    /*! cmp output be invert
     */
    bool                                  cmp_io_out_invert_en;
    /*! cmp output be invert
     */
    bool                                  cmp_cpu_out_invert_en;
} TYPE_LL_SARADC_CMP_CFG ;

/**
  * @brief SARADC low layer init struct
  */
typedef struct __ll_saradda_init {
    /*! adc clk : fpga_max 250000, IC_max:2500000 && must bigger than (18 * Max(sample_rate))
     *  clk_div = SYS_CLK / adc_clk, adc_clk is recommand to 2000000
     */
    u32     clk_div;
} TYPE_LL_SARADDA_INIT ;

/**
  * @}
  */

/** @defgroup SARADC_LL_Interrupt SARADC LL Interrupt Handle function
  * @brief   SARADC LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SARADC  
    Interrupt Handle function.

    how to use?

    The SARADC interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the SARADC in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */
  
/** 
  * @brief  SARADC interrupt handle function.
  * @param  None.
  * @retval None.
  */



/**
  * @}
  */
  
/** @defgroup SARADC_LL_Inti_Cfg SARADC LL Initialization And Configuration
  * @brief    SARADC LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SARADC data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */
  
/** 
  * @brief  SARADC init function. reset & enable saradc
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_init  : pointer to TYPE_LL_SARADDA_INIT SARADC init struct
  * @retval None.
  */
void ll_saradc_init(ADC_TypeDef *p_saradc, TYPE_LL_SARADDA_INIT *p_init);

/** 
  * @brief  SARADC deinit function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @retval none
  */
void ll_saradc_deinit(ADC_TypeDef *p_saradc);

/** 
  * @brief  SARADC irq config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : \ref TYPE_LL_SARADC_IRQ_CFG config interrupt enable
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_irq_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_IRQ_CFG *p_cfg);

/** 
  * @brief  SARADC config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : \ref TYPE_LL_SARADC_CFG config adda sample_rate, int, average, downsample, dc_offset, gain, 
                      quantify, dma, compare, dac, ect.
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_CFG *p_cfg);

/** 
  * @brief  SARADC dac config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : \ref TYPE_LL_SARADC_CFG config
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_dac_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_DAC_CFG *p_cfg);

/** 
  * @brief  SARADC dac config function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  p_cfg   : \ref TYPE_LL_SARADC_CMP_CFG config
  * @retval state   : 1(success), 0(failed).
  */
u32 ll_saradc_cmp_config(ADC_TypeDef *p_saradc, TYPE_LL_SARADC_CMP_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup SARADC_LL_Data_Transfers SARADC LL Data transfers functions
  * @brief    SARADC LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SARADC data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/** 
  * @brief  SARADC start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to start, use bit field.
  * @retval None.
  */
void ll_saradc_start(ADC_TypeDef *p_saradc, u32 chn_bits);

/** 
  * @brief  SARADC stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to stop, use bit field.
  * @retval None.
  */
void ll_saradc_stop(ADC_TypeDef *p_saradc, u32 chn_bits);

/** 
  * @brief  SARADC dac start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_dac_start(ADC_TypeDef *p_saradc, u32 chn);

/** 
  * @brief  SARADC dac stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_dac_stop(ADC_TypeDef *p_saradc, u32 chn);

/** 
  * @brief  SARADC cmp start function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_cmp_start(ADC_TypeDef *p_saradc, u32 chn);

/** 
  * @brief  SARADC dac stop function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn: specify adc chn to start
  * @retval None.
  */
void ll_saradc_cmp_stop(ADC_TypeDef *p_saradc, u32 chn);

/** 
  * @brief  SARADC saradc_dc_offset_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  offset  : specify adc chn dc_offset[0:11] to set
  * @retval None.
  */
void ll_saradc_set_dc_offset(ADC_TypeDef *p_saradc, u32 chn, u16 offset);

/** 
  * @brief  SARADC saradc_quantify_set function.
  * @param  p_saradc         : pointer to ADC_TypeDef SARADC controller 
  * @param  chn              : specify adc chn to set
  * @param  coef             : specify adc chn coef to set, rang [0:32767] means [0:1)
  * @param  out_decimal_width: specify Quantify output data remain how many decimal width rang:[0,6]
  * @retval None.
  */
void ll_saradc_set_quantify(ADC_TypeDef *p_saradc, u8 chn, u16 coef, u8 out_decimal_width);

/** 
  * @brief  SARADC saradc_average_times_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  average : specify average times \ref TYPE_ENUM_LL_SARADC_AVERAGE_SEL
  * @retval None.
  */
void ll_saradc_set_average(ADC_TypeDef *p_saradc, u8 chn, TYPE_ENUM_LL_SARADC_AVERAGE_SEL average);

/** 
  * @brief  SARADC saradc_down_sample_times_set function.
  * @param  p_saradc  : pointer to ADC_TypeDef SARADC controller 
  * @param  chn       : specify adc chn to set
  * @param  downsample: \ref TYPE_ENUM_LL_SARADC_DOWNSAMPLE_SEL specify down_sample times
  * @retval None.
  */
void ll_saradc_set_downsample(ADC_TypeDef *p_saradc, u8 chn, TYPE_ENUM_LL_SARADC_DOWNSAMPLE_SEL downsample);


/** 
  * @brief  SARADC saradc_sample_rate_get function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to get
  * @retval divor read from reg, sample_rate = sys_clk/divor
  */
u16 ll_saradc_get_sample_rate(ADC_TypeDef *p_saradc, u8 chn);

/** 
  * @brief  SARADC saradc_sample_rate_set function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  divor   : = sys_clk/sample_rate
  * @retval divor read from reg.
  */
u16 ll_saradc_set_sample_rate(ADC_TypeDef *p_saradc, u8 chn, u16 divor);

/** 
  * @brief  SARADC ll_saradc_sample_done_interrupt_enable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_sample_interrupt_enable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL0 |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_sample_done_interrupt_disable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_sample_interrupt_disable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL0 &= ~chn_bits;
}

/** 
  * @brief  LL_SARADC_CHECK_SAMPLE_INTERRUPT_ENABLE
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval interrupt_enable  
  */
#define LL_SARADC_CHECK_SAMPLE_INTERRUPT_ENABLE(p_saradc, chn_bits) ((p_saradc)->SARADC_INT_CONTROL0 & chn_bits)

/** 
  * @brief  SARADC LL_SARADC_GET_SAMPLE_PENDING function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval adc sample data.
  */
#define LL_SARADC_GET_SAMPLE_PENDING(p_saradc, chn_bits)            ((p_saradc)->SARADC_PENDING0 & chn_bits)

/** 
  * @brief  SARADC ll_saradc_clear_sample_pending function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_clear_sample_pending(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_PENDING0_CLR |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_dma_half_interrupt_enable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dma_half_interrupt_enable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL1 |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_dma_half_interrupt_disable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dma_half_interrupt_disable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL1 &= ~chn_bits;
}

/** 
  * @brief  LL_SARADC_CHECK_DMA_HALF_INTERRUPT_ENABLE
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval interrupt_enable  
  */
#define LL_SARADC_CHECK_DMA_HALF_INTERRUPT_ENABLE(p_saradc, chn_bits)   ((p_saradc)->SARADC_INT_CONTROL1 & chn_bits)

/** 
  * @brief  SARADC LL_SARADC_GET_DMA_HALF_PENDING function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval adc sample data.
  */
#define LL_SARADC_GET_DMA_HALF_PENDING(p_saradc, chn_bits)              ((p_saradc)->SARADC_PENDING1 & chn_bits)

/** 
  * @brief  SARADC ll_saradc_clear_dma_half_pending function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_clear_dma_half_pending(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_PENDING1_CLR |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_dma_full_interrupt_enable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dma_full_interrupt_enable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL2 |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_dma_full_interrupt_disable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dma_full_interrupt_disable(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_INT_CONTROL2 &= ~chn_bits;
}

/** 
  * @brief  LL_SARADC_CHECK_DMA_FULL_INTERRUPT_ENABLE
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn (bit field)
  * @retval interrupt_enable  
  */
#define LL_SARADC_CHECK_DMA_FULL_INTERRUPT_ENABLE(p_saradc, chn_bits)   ((p_saradc)->SARADC_INT_CONTROL2 & chn_bits)

/** 
  * @brief  SARADC LL_SARADC_GET_DMA_FULL_PENDING function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn_bits (bit field)
  * @retval adc sample data.
  */
#define LL_SARADC_GET_DMA_FULL_PENDING(p_saradc, chn_bits)              ((p_saradc)->SARADC_PENDING2 & chn_bits)

/** 
  * @brief  SARADC ll_saradc_clear_dma_full_pending function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn_bits: specify adc chn to set(bit field)
  * @retval none
  */
__STATIC_INLINE void ll_saradc_clear_dma_full_pending(ADC_TypeDef *p_saradc, u32 chn_bits) {
    p_saradc->SARADC_PENDING2_CLR |= chn_bits;
}

/** 
  * @brief  SARADC ll_saradc_set_cmp_data function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  data0    : 12bit unsigned [0,3.3v] 
  * @param  data1    : 12bit unsigned [0,3.3v]
  * @retval none
  * @note   the compare use Hysteresis window control [dat_lit, dat_big] 
  *                                                      
  *             dat_lit<------^--------1
  *                   |       |
  *         0---------V------>dat_big
  */
void ll_saradc_set_cmp_data(ADC_TypeDef *p_saradc, u8 chn, u16 data0, u16 data1);

/** 
  * @brief  SARADC ll_saradc_set_dac_data function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @param  data    : 12bit unsigned [0,3.3v] 
  */
void ll_saradc_set_dac_data(ADC_TypeDef *p_saradc, u8 chn, s16 data);

/** 
  * @brief  SARADC ll_saradc_dac_interrupt_enable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify dac chn to set , must > LL_SARADC_ADC_CHN_MAX
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dac_interrupt_enable(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] |=  LL_SARADC_DAC_INT_EN;
}

/** 
  * @brief  SARADC ll_saradc_dac_interrupt_disable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify dac chn to set,, must > LL_SARADC_ADC_CHN_MAX
  * @retval none
  */
__STATIC_INLINE void ll_saradc_dac_interrupt_disable(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] &= ~LL_SARADC_DAC_INT_EN;
}

/** 
  * @brief  LL_SARADC_CHECK_DMA_FULL_INTERRUPT_ENABLE
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @retval interrupt_enable  
  */
#define LL_SARADC_CHECK_DAC_INTERRUPT_ENABLE(p_saradc, chn)     ((p_saradc)->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] & LL_SARADC_DAC_INT_EN)

/** 
  * @brief  SARADC LL_SARADC_GET_COMPARE_PENDING function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn 
  * @retval adc sample data.
  */
#define LL_SARADC_GET_DAC_PENDING(p_saradc, chn)                ((p_saradc)->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] & LL_SARADC_DAC_DONE)

/** 
  * @brief  SARADC ll_saradc_clear_sample_pending function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn  
  * @retval none
  */
__STATIC_INLINE void ll_saradc_clear_dac_pending(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] |= LL_SARADC_DAC_DONE_CLR;
}

/** 
  * @brief  SARADC get sample_data function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to get
  * @retval adc sample data.
  */
#define LL_SARADC_GET_SAMPLE_DATA(p_saradc, chn)            ((p_saradc)->SARADC_DATA[chn])

/** 
  * @brief  SARADC get dma index function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to get
  * @retval adc sample data.
  */
#define LL_SARADC_GET_DMA_INDEX(p_saradc, chn)              ((p_saradc)->SARADC_DMA_INDEX[chn])

/** 
  * @brief  LL_SARADC_CHECK_CMP_INTERRUPT_ENABLE
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @retval interrupt_enable  
  */
#define LL_SARADC_CHECK_CMP_INTERRUPT_ENABLE(p_saradc, chn)     ((p_saradc)->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] & LL_SARADC_CMP_INT_EN)

/** 
  * @brief  SARADC LL_SARADC_GET_CMP_PENDING function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn 
  * @retval adc sample data.
  */
#define LL_SARADC_GET_CMP_CPU_VALUE(p_saradc, chn)              (((p_saradc)->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] & LL_SARADC_CMP_RESULT) ? 1 : 0)

/** 
  * @brief  SARADC ll_saradc_dma_full_interrupt_enable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @retval none
  */
__STATIC_INLINE void ll_saradc_compare_interrupt_enable(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] |=  LL_SARADC_CMP_INT_EN;
}

/** 
  * @brief  SARADC ll_saradc_dma_full_interrupt_disable function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn to set
  * @retval none
  */
__STATIC_INLINE void ll_saradc_compare_interrupt_disable(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] &= ~LL_SARADC_CMP_INT_EN;
}

#if 0
/** 
  * @brief  SARADC ll_saradc_clear_cmp_pending function.
  * @param  p_saradc: pointer to ADC_TypeDef SARADC controller 
  * @param  chn     : specify adc chn  
  * @retval none
  */
__STATIC_INLINE void ll_saradc_clear_cmp_pending(ADC_TypeDef *p_saradc, u32 chn) {
    p_saradc->DACCMP_CON[chn-LL_SARADC_ADC_CHN_MAX] |= LL_SARADC_DAC_DONE_CLR;
}
#endif

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

#endif //__TX_PHE_LL_SARADC_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
