/**
  ******************************************************************************
  * @file    Libraries/Driver/include/LL/tx_phe_ll_spi.h
  * @author  HUGE-IC Application Team
  * @version V1.0.1
  * @date    03-08-2018
  * @brief   This file contains all the SPI LL firmware functions.
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
#ifndef __TX_PHE_LL_SPI_H
#define __TX_PHE_LL_SPI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tx_phe.h"
     
/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
     
/** @addtogroup spi_interface_gr SPI Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */ 

/** @addtogroup SPI_LL_Driver SPI LL Driver
  * @ingroup  spi_interface_gr
  * @brief Mainly the driver part of the SPI module, which includes \b SPI \b Register 
  * \b Constants, \b SPI \b Exported \b Constants, \b SPI \b Exported \b Struct, \b SPI
  * \b Data \b transfers \b functions, \b SPI \b Initialization \b and \b SPI \b Configuration 
  * \b And \b Interrupt \b Handle \b function.
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/** @defgroup SPI_LL_Register_Constants SPI LL Register Constants
  * @ingroup  SPI_LL_Driver
  * @brief    SPI LL register constant table definition
  *
  *
@verbatim   
  ===============================================================================
                                Register Constants
  ===============================================================================  
  
    Register Constants mainly encapsulates each bit in each group in the SPI 
    register. In the process of configuration, the macro definition can be directly 
    called to configure the SPI register, mainly for convenience. Understand the 
    configuration of the SPI.
    
@endverbatim
  *
  * @{
  */

/***** CFG *****/
/*! The IRQ generated when the CS of the SPI slave changes from low to high.
 */
#define LL_SPI_CFG_END_IRQ_EN                       (1UL << 27)
/*! The number of clocks that the SPI data receives is delayed relative to CLK. 
 */
#define LL_SPI_CFG_RXSEL(n)                         (((n)&0x07) << 24)
/*! SPI Clock Divider. The frequency of the CLK is derived from the following
 *  equation:
 *          Fclk = APB_CLK/((DIV_CNT+1)*2)
 */
#define LL_SPI_CFG_DIV_CNT(n)                       (((n)&0xFF) << 16)
/*! SPI TX DRQ Enable, trigger DRQ when SPI TX FIFO is half empty. Just for 
 *  DMAC use.
 */
#define LL_SPI_CFG_TDRQ_EN                          (1UL << 15)
/*! SPI RX DRQ Enable, trigger DRQ when SPI RX FIFO is half full. Just for 
 *  DMAC use.
 */
#define LL_SPI_CFG_RDRQ_EN                          (1UL << 14)
/*! SPI TX IRQ Enable, trigger SPI TX IRQ when SPI TX FIFO is half empty.
 */
#define LL_SPI_CFG_TIRQ_EN                          (1UL << 13)
/*! SPI RX IRQ Enable, trigger SPI RX IRQ when SPI RX FIFO is half full.
 */
#define LL_SPI_CFG_RIRQ_EN                          (1UL << 12)
/*! SPI tx fifo enable. The tx function can only be used normally after tx
 *  fifo is enabled.
 */
#define LL_SPI_CFG_TX_FIFO_EN                       (1UL << 9)
/*! SPI rx fifo enable. The rx function can only be used normally after rx
 *  fifo is enabled.
 */
#define LL_SPI_CFG_RX_FIFO_EN                       (1UL << 8)
/*! SPI frame data format setting.
 */
#define LL_SPI_CFG_DFS_SET(n)                       (((n)&0x03) << 6)
/*! Wire mode setting of the SPI data line.
 */
#define LL_SPI_CFG_WIRE_MODE_SET(n)                 (((n)&0x03) << 4)
#define LL_SPI_CFG_WIRE_MODE_GET(n)                 (((n) >> 4) & 0x03)
#define LL_SPI_CFG_WIRE_MODE_MASK                   (0x03UL << 4)
/*! SPI mode setting
 */
#define LL_SPI_CFG_SPI_MODE_SET(n)                  (((n)&0x03) << 2)
/*! Enable the SPI slave
 */
#define LL_SPI_CFG_SLAVE_EN                         (1UL << 0)


/***** CTL *****/
/*! SPI CS pin control output, this bit is valid only in master mode
 */
#define LL_SPI_CTL_CS_HIGH                          (1UL << 3)
/*! Enable the SPI tx function.
 */
#define LL_SPI_CTL_TX_EN                            (1UL << 1)
/*! Enable the SPI rx function.
 */
#define LL_SPI_CTL_RX_EN                            (1UL << 0)


/***** STA *****/
/*! Trigger SPI Transfer End pending when SPI slave CS from low to high.
 *  Writing 1 to this bit will clear it, otherwise unchanged.
 */
#define LL_SPI_STA_END_PENDING                      (1UL << 10)
/*! Trigger SPI TX IRQ pending when SPI tx fifo is half empty. Writing 1 to
 *  this bit will clear it, otherwise unchanged.
 */
#define LL_SPI_STA_TX_IRQ_PENDING                   (1UL << 9)
/*! Trigger SPI RX IRQ pending when SPI rx fifo is half full. Writing 1 to
 *  this bit will clear it, otherwise unchanged.
 */
#define LL_SPI_STA_RX_IRQ_PENDING                   (1UL << 8)
/*! When spi tx fifo is full, the pending is 1, otherwise it is 0.
 */
#define LL_SPI_STA_TX_FIFO_FULL_PENDING             (1UL << 7)
/*! When spi tx fifo is empty, the pending is 1, otherwise it is 0.
 */
#define LL_SPI_STA_TX_FIFO_EMPTY_PENDING            (1UL << 6)
/*! When spi rx fifo is full, the pending is 1, otherwise it is 0.
 */
#define LL_SPI_STA_RX_FIFO_FULL_PENDING             (1UL << 5)
/*! When spi rx fifo is empty, the pending is 1, otherwise it is 0.
 */
#define LL_SPI_STA_RX_FIFO_EMPTY_PENDING            (1UL << 4)
/*! This bit set when SPI tx fifl is wrote overflow. Writing 1 to this bit
 *  will clear it, otherwise unchanged.
 */
#define LL_SPI_STA_TX_FIFO_ERR_PENDING              (1UL << 3)
/*! This bit set when SPI rx fifl is wrote overflow. Writing 1 to this bit
 *  will clear it, otherwise unchanged.
 */
#define LL_SPI_STA_RX_FIFO_ERR_PENDING              (1UL << 2)
/*! SPI busy status bit
 */
#define LL_SPI_STA_BUSY_PENDING                     (1UL << 0)

/**
  * @}
  */

/** @defgroup SPI_LL_Exported_Constants SPI LL Exported Constants
  * @ingroup  SPI_LL_Driver
  * @brief    SPI LL external constant definition
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



/***** LL API *****/


  
/***** LL API AND DRIVER API *****/

/**
  * @brief SPI data frame size enum type, consistent with the spec definition.  
  *        The spi data frame size represents the number of bits of data sent
  *        by the spi module each time.
  */
typedef enum {
    /*! The SPI module has a frame size of 8 bits.
     */
    LL_SPI_8_BIT                        = 0,
    /*! The SPI module has a frame size of 16 bits.
     */
    LL_SPI_16_BIT                       = 1,
    /*! The SPI module has a frame size of 24 bits, but the SPI module 
     *  performs data operations in word units.
     */
    LL_SPI_24_BIT                       = 2,
    /*! The SPI module has a frame size of 32 bits.
     */
    LL_SPI_32_BIT                       = 3,
} TYPE_ENUM_LL_SPI_FRAME_SIZE;

/**
  * @brief SPI wire mode enum type, consistent with the spec definition
  */
typedef enum {
    /*! The effective bus of the SPI module is CS, CLK, MOSI, MISO.
     */
    LL_SPI_NORMAL_MODE                  = 0,
    /*! The SPI module implements the MOSI and MISO functions with one IO.
     *  @note When the SPI module is used as the master, the active bus is CS,
     *        CLK, and MOSI. When the SPI module is used as the slave, the
     *        active bus is CS, CLK, and MISO.
     */
    LL_SPI_THREE_WIRE_MODE              = 1,
    /*! The active bus of the SPI module is CS, CLK, IO0, IO1. Among them IO0
     *  and IO1 are bidirectional IO.
     */
    LL_SPI_DUAL_MODE                    = 2,
    /*! The active bus of the SPI module is CS, CLK, IO0, IO1, IO2, IO3. Among
     *  them, IO0 to IO4 are bidirectional IO.
     */
    LL_SPI_QUAD_MODE                    = 3,
} TYPE_ENUM_LL_SPI_WIRE_MODE;

/**
  * @brief SPI mode enum type, consistent with the spec definition.
  * @note SPI_MODE_0 represents the first valid rising edge to start data acquisition.  
  *       SPI_MODE_1 represents the second valid falling edge to start data acquisition.  
  *       SPI_MODE_2 represents the first valid falling edge to start data acquisition.  
  *       SPI_MODE_3 represents the second valid rising edge to start data acquisition.
  */
typedef enum {
    /*! CPOL = 0, CPHA = 0
     */
    LL_SPI_MODE_0                       = 0,
    /*! CPOL = 0, CPHA = 1
     */
    LL_SPI_MODE_1                       = 1,
    /*! CPOL = 1, CPHA = 0
     */
    LL_SPI_MODE_2                       = 2,
    /*! CPOL = 1, CPHA = 1
     */
    LL_SPI_MODE_3                       = 3,
} TYPE_ENUM_LL_SPI_MODE;

/**
  * @brief SPI module IO mapping enumeration
  */
typedef enum {
    /*! Select the pin of the SPI0 io map0
     */
    LL_SPI0_IOMAP0,
    /*! Select the pin of the SPI0 io map1
     */
    LL_SPI0_IOMAP1,
    /*! Select the pin of the SPI0 MCP io map0
     */
    LL_SPI0_MCP_IOMAP0,
    /*! Select the pin of the SPI0 MCP io map1
     */
    LL_SPI0_MCP_IOMAP1,
    /*! Select the pin of the SPI1 io map0
     */
    LL_SPI1_IOMAP0,
    /*! Select the pin of the SPI1 io map1
     */
    LL_SPI1_IOMAP1,
} TYPE_ENUM_LL_SPI_IO_MAP;

/**
  * @}
  */

/** @defgroup SPI_LL_Exported_Struct SPI LL Exported Struct
  * @ingroup  SPI_LL_Driver
  * @brief    SPI LL external configuration structure definition
  *
@verbatim   
  ===============================================================================
                                Exported Struct
  ===============================================================================  

    Exported Struct mainly extracts the SPI registers from the API, and abstracts 
    the structure. As long as it implements the low coupling between the registers 
    and the registers, the user only needs to configure the structure of the abstraction 
    layer and call hal_spi_init. Function, you can configure the SPI module without 
    involving the configuration of the collective register.

@endverbatim
  *
  * @{
  */

/**
  * @brief SPI low layer initialization structure
  */
typedef struct __ll_spi_init {
    /*! SPI clock divider ratio.
     * @note SPI_CLK = APB_CLK/((clk_div_cnt+1)*2)
     */
    u32                         clk_div_cnt;
    /*! Configure the number of bits of data sent by the SPI module each time.
     */
    TYPE_ENUM_LL_SPI_FRAME_SIZE frame_size;
    /*! Configure the bus mode of the SPI module.
     */
    TYPE_ENUM_LL_SPI_MODE       spi_mode;
    /*! If the SPI module is acting as a slave, the flag needs to be set to 
     *  true.
     */
    bool                        slave_mode_en;
    /*! Configure the SPI module's wire mode.
     */
    TYPE_ENUM_LL_SPI_WIRE_MODE  wire_mode;
    /*! IO map selection for SPI
     *  @note The correspondence table of IO_MAP is as follows:
     *        |      | IO map enum        | CS/CLK/IO0/IO1/IO2/IO3  |
     *        | :--- | :----------------- | :---------------------- |
     *        | SPI0 | LL_SPI0_IOMAP0     | PA0/PA1/PA2/PA3/PA4/PA5 |
     *        | ^    | LL_SPI0_IOMAP1     | PA0/PA1/PA2/PA3         |
     *        | ^    | LL_SPI0_MCP_IOMAP0 | PA0/PD1/PD0/PA1         |
     *        | ^    | LL_SPI0_MCP_IOMAP1 | PA0/PD1/PD0/PA1/PA2/PD2 |
     *        | SPI1 | LL_SPI1_IOMAP0     | PD0/PD1/PD2/PD3/PD4/PD5 |
     *        | ^    | LL_SPI1_IOMAP1     | PD0/PD1/PD2/PD3         |
     */
    TYPE_ENUM_LL_SPI_IO_MAP     io_map;
} TYPE_LL_SPI_INIT;

/**
  * @brief SPI module interrupt low layer configuration structure
  */
typedef struct __ll_spi_irq_cfg {
    /*! The spi module tx interrupt enable.
     */
    bool                       tx_intr_en;
    /*! The spi module rx interrupt enable.
     */
    bool                       rx_intr_en;
    /*! The interrupt is enabled when the SPI slave CS goes from low to high. 
     */
    bool                       slave_cs_rising_intr_en;
} TYPE_LL_SPI_IRQ_CFG;

/**
  * @}
  */

/** @defgroup SPI_LL_Interrupt SPI LL Interrupt Handle function
  * @brief   SPI LL Interrupt Handle function
  *
@verbatim   
  ===============================================================================
                        Interrupt Handle function
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPI  
    Interrupt Handle function.

    how to use?

    The SPI interrupt handler uses a callback method that reserves the interface 
    to the user in the form of a callback function. The client needs to initialize 
    the callback function when initializing the SPI in order for the interrupt to 
    be processed normally. 
   
@endverbatim
  *
  * @{
  */



/**
  * @}
  */
  
/** @defgroup SPI_LL_Inti_Cfg SPI LL Initialization And Configuration
  * @brief    SPI LL Initialization And Configuration
  *
@verbatim   
  ===============================================================================
                        Initialization And Configuration
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPI data 
    Initialization and Configuration.
    
    how to use?

@endverbatim
  *
  * @{
  */

/** 
  * @brief  Low layer SPI initialization function
  * @param  p_spi : SPI module pointer
  * @param  p_init: SPI initialization struct pointer
  * @retval None.
  */
void ll_spi_init(SPI_TypeDef *p_spi, TYPE_LL_SPI_INIT *p_init);

/**
  * @brief  SPI module detele initialization function
  * @param  p_spi: Select the initialized SPI module pointer
  * @retval None
  */
void ll_spi_deinit(SPI_TypeDef *p_spi);

/**
  * @brief  Low layer SPI io map init function
  * @param  p_spi : SPI module pointer
  * @param  io_map: SPI module IO mapping
  * @retval None
  */
void ll_spi_io_map(SPI_TypeDef *p_spi, TYPE_ENUM_LL_SPI_IO_MAP io_map);

/** 
  * @brief  Low layer SPI interrupt config function
  * @param  p_spi: SPI module pointer
  * @param  p_cfg: SPI Configuration struct pointer
  * @retval None.
  */
void ll_spi_irq_config(SPI_TypeDef *p_spi, TYPE_LL_SPI_IRQ_CFG *p_cfg);

/**
  * @}
  */
  
/** @defgroup SPI_LL_Data_Transfers SPI LL Data transfers functions
  * @brief    SPI LL Data transfers functions 
  *
@verbatim   
  ===============================================================================
                            Data transfers functions
  ===============================================================================  

    This subsection provides a set of functions allowing to manage the SPI data 
    transfers and receive.
  
@endverbatim
  *
  * @{
  */
  
/**
  * @brief  The SPI module gets the current wire mode
  * @param  p_spi: SPI module pointer
  * @retval None
  */
TYPE_ENUM_LL_SPI_WIRE_MODE ll_spi_get_wire_mode(SPI_TypeDef *p_spi);

/**
  * @brief  The SPI module set the current wire mode
  * @param  p_spi        : SPI module pointer
  * @param  wire_mode_sel: SPI wire mode \ref TYPE_ENUM_LL_SPI_WIRE_MODE
  * @retval None
  */
void ll_spi_set_wire_mode(SPI_TypeDef *p_spi, TYPE_ENUM_LL_SPI_WIRE_MODE wire_mode_sel);

/**
  * @brief  The SPI write a data function.
  * @param  p_spi: SPI module pointer.
  * @param  data : The data sent.
  * @retval None
  */
#define LL_SPI_WRITE_DATA(p_spi, data)              ((p_spi)->WDATA = (data))

/**
  * @brief  The SPI read a data function.
  * @param  p_spi: SPI module pointer.
  * @retval The data read by the SPI.
  */
#define LL_SPI_READ_DATA(p_spi)                     ((p_spi)->RDATA)

/**
  * @brief  Pull down the chip select(CS) pins of the SPI module.
  * @param  p_spi: SPI module pointer.
  * @retval None
  * @note   Only valid in master mode.
  */
__STATIC_INLINE void ll_spi_set_cs(SPI_TypeDef *p_spi) {
    p_spi->CTL |= LL_SPI_CTL_CS_HIGH;
}

/**
  * @brief  Pull up the chip select(CS) pins of the SPI module.
  * @param  p_spi: SPI module pointer.
  * @retval None
  * @note   Only valid in master mode.
  */
__STATIC_INLINE void ll_spi_clear_cs(SPI_TypeDef *p_spi) {
    p_spi->CTL &= ~(LL_SPI_CTL_CS_HIGH);
}

/**
  * @brief  Check SPI RX interrupt enable
  * @param  p_spi: SPI module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_SPI_CHECK_RX_INTERRUPT_ENABLE(p_spi)     ((p_spi)->CFG & LL_SPI_CFG_RIRQ_EN)

/**
  * @brief  Check SPI TX interrupt enable
  * @param  p_spi: SPI module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_SPI_CHECK_TX_INTERRUPT_ENABLE(p_spi)     ((p_spi)->CFG & LL_SPI_CFG_TIRQ_EN)

/**
  * @brief  Check SPI END interrupt enable
  * @param  p_spi: SPI module pointer
  * @retval Returning 0 means disabling, and the other values are enabling
  */
#define LL_SPI_CHECK_END_INTERRUPT_ENABLE(p_spi)    ((p_spi)->CFG & LL_SPI_CFG_END_IRQ_EN)

/**
  * @brief  Whether the SPI slave communication ends
  * @param  p_spi: SPI module pointer
  * @retval Return 0 means SPI communication is not over, returning other
  *         values means SPI communication is completed.
  */
#define LL_SPI_GET_END_PENDING(p_spi)               ((p_spi)->STA & LL_SPI_STA_END_PENDING)

/**
  * @brief  SPI clears end pending.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_end_pending(SPI_TypeDef *p_spi) {
    (p_spi)->STA |= LL_SPI_STA_END_PENDING;
}

/**
  * @brief  Check SPI tx irq pending
  * @param  p_spi: SPI module pointer
  * @retval Return to SPI tx irq pending
  */
#define LL_SPI_GET_TX_IRQ_PENDING(p_spi)            ((p_spi)->STA & LL_SPI_STA_TX_IRQ_PENDING)

/**
  * @brief  clear SPI tx irq pending
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_tx_irq_pending(SPI_TypeDef *p_spi) {
    (p_spi)->STA |= LL_SPI_STA_TX_IRQ_PENDING;
}

/**
  * @brief  Check SPI rx irq pending
  * @param  p_spi: SPI module pointer
  * @retval Return to SPI rx irq pending
  */
#define LL_SPI_GET_RX_IRQ_PENDING(p_spi)            ((p_spi)->STA & LL_SPI_STA_RX_IRQ_PENDING)

/**
  * @brief  clear SPI rx irq pending
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_rx_irq_pending(SPI_TypeDef *p_spi) {
    (p_spi)->STA |= LL_SPI_STA_RX_IRQ_PENDING;
}

/**
  * @brief  Check SPI tx fifo full state
  * @param  p_spi: SPI module pointer
  * @retval Return SPI tx fifo full pending
  */
#define LL_SPI_GET_TX_FIFO_FULL_PENDING(p_spi)      ((p_spi)->STA & LL_SPI_STA_TX_FIFO_FULL_PENDING)

/**
  * @brief  Check SPI rx fifo empty state
  * @param  p_spi: SPI module pointer
  * @retval Return SPI rx fifo empty pending
  */
#define LL_SPI_GET_RX_FIFO_EMPTY_PENDING(p_spi)     ((p_spi)->STA & LL_SPI_STA_RX_FIFO_EMPTY_PENDING)

/**
  * @brief  Check SPI tx fifo err pending
  * @param  p_spi: SPI module pointer
  * @retval Return SPI tx fifo err pending
  */
#define LL_SPI_GET_TX_FIFO_ERR_PENDING(p_spi)       ((p_spi)->STA & LL_SPI_STA_TX_FIFO_ERR_PENDING)

/**
  * @brief  Clear SPI tx fifo err pending
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_tx_fifo_err_pending(SPI_TypeDef *p_spi) {
    (p_spi)->STA |= LL_SPI_STA_TX_FIFO_ERR_PENDING;
}

/**
  * @brief  Check SPI rx fifo err pending
  * @param  p_spi: SPI module pointer
  * @retval Return SPI rx fifo err pending
  */
#define LL_SPI_GET_RX_FIFO_ERR_PENDING(p_spi)       ((p_spi)->STA & LL_SPI_STA_RX_FIFO_ERR_PENDING)

/**
  * @brief  Clear SPI rx fifo err pending
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_clear_rx_fifo_err_pending(SPI_TypeDef *p_spi) {
    (p_spi)->STA |= LL_SPI_STA_RX_FIFO_ERR_PENDING;
}

/**
  * @brief  SPI module busy state function
  * @param  p_spi: SPI module pointer
  * @retval Return 0 means the SPI module is in the idle state, and returning
  *         other values means the SPI module is in the busy state.
  */
#define LL_SPI_GET_BUSY_PENDING(p_spi)              ((p_spi)->STA & LL_SPI_STA_BUSY_PENDING)

/**
  * @brief  SPI enable receive interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_receive_interrupt_enable(SPI_TypeDef *p_spi) {
    p_spi->CFG |= LL_SPI_CFG_RIRQ_EN;
}

/**
  * @brief  SPI disable receive interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_receive_interrupt_disable(SPI_TypeDef *p_spi) {
    p_spi->CFG &= ~(LL_SPI_CFG_RIRQ_EN);
}

/**
  * @brief  SPI enable send interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_send_interrupt_enable(SPI_TypeDef *p_spi) {
    p_spi->CFG |= LL_SPI_CFG_TIRQ_EN;
}

/**
  * @brief  SPI disable receive interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_send_interrupt_disable(SPI_TypeDef *p_spi) {
    p_spi->CFG &= ~(LL_SPI_CFG_TIRQ_EN);
}

/**
  * @brief  SPI enable receive interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_slave_cs_rising_interrupt_enable(SPI_TypeDef *p_spi) {
    p_spi->CFG |= LL_SPI_CFG_END_IRQ_EN;
}

/**
  * @brief  SPI disable receive interruption.
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_slave_cs_rising_interrupt_disable(SPI_TypeDef *p_spi) {
    p_spi->CFG &= ~(LL_SPI_CFG_END_IRQ_EN);
}

/**
  * @brief  Start the tx function of the spi module
  * @param  p_spi: SPI module pointer
  * @param  num:   The number of frames to send.
  * @retval None
  */
__STATIC_INLINE void ll_spi_start_tx(SPI_TypeDef *p_spi, u32 num)
{
    p_spi->CTL     |= LL_SPI_CTL_TX_EN; //Enable the SPI tx function
    p_spi->TX_BC    = num;              //Set the number of frames to send
    p_spi->TX_START = 0x0000;           //Start the TX function
}

/**
  * @brief  Start the rx function of the spi module
  * @param  p_spi: SPI module pointer
  * @param  num:   The number of frames to send.
  * @retval None
  */
__STATIC_INLINE void ll_spi_start_rx(SPI_TypeDef *p_spi, u32 num)
{
    p_spi->CTL     |= LL_SPI_CTL_RX_EN; //Enable the SPI rx function
    p_spi->RX_BC    = num;              //Set the number of frames to receive
    p_spi->RX_START = 0x0000;           //Start the RX function
}

/**
  * @brief  Disable the SPI module
  * @param  p_spi: SPI module pointer
  * @retval None
  */
__STATIC_INLINE void ll_spi_disable(SPI_TypeDef *p_spi)
{
    p_spi->CTL &= ~(LL_SPI_CTL_TX_EN | LL_SPI_CTL_RX_EN);
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

#endif //__TX_PHE_LL_SPI_H

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
