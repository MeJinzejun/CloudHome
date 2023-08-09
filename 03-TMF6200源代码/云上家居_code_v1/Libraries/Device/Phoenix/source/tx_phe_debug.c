/**
  ******************************************************************************
  * @file    Libraries/Device/Phoenix/source/tx_phe_debug.c
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   This file contains all the DEBUG LL firmware functions.
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
#include "tx_phe_debug.h"

/** @addtogroup TX_PHE_StdPeriph_Driver TX_PHE Driver
  * @{
  */
  
/** @defgroup debug_interface_gr DEBUG Driver
  * @ingroup  TX_PHE_StdPeriph_Driver
  * @{
  */

/** @addtogroup DEBUG_LL_Driver DEBUG LL Driver
  * @ingroup  debug_interface_gr
  * @{
  */
  
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#ifdef __GNUC__
    /* With GCC/RAISONANCE, small DBG_NOR (option LD Linker->Libraries->Small DBG_NOR
       set to 'Yes') calls __io_putchar() */
    #define PUTCHAR_PROTOTYPE           int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE           int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
    
#ifdef __GNUC__
    /* With GCC/RAISONANCE, small DBG_NOR (option LD Linker->Libraries->Small DBG_NOR
       set to 'Yes') calls __io_getchar() */
    #define GETCHAR_PROTOTYPE           int __io_getchar()
#else
    #define GETCHAR_PROTOTYPE           int fgetc(FILE *f) 
#endif /* __GNUC__ */
    
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined(__CC_ARM)                /* KEIL Compiler */
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; /* Add whatever you need here */ };
#endif

FILE __stdout;
FILE __stdin;
volatile int32_t ITM_RxBuffer;

static TYPE_ENUM_DEBUG_INTERFACE_SEL __debug_interface = DEBUG_NO;
static USART_TypeDef                 *__p_debug_usart   = USART0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup DEBUG_LL_Inti_Cfg DEBUG LL Initialization And Configuration
  * @ingroup  DEBUG_LL_Driver
  * @brief    DEBUG LL Initialization And Configuration
  * @{
  */

/**
  * @brief  Configure the debug interface, mainly the ITM and USART modules.
  * @param  debug_interface: Debug interface type selection.
  * @retval None
  */
void debug_select_interface(TYPE_ENUM_DEBUG_INTERFACE_SEL debug_interface)
{
    __debug_interface = debug_interface;
    
    if(debug_interface == DEBUG_USART0) {
        __p_debug_usart = USART0;
    } else if(debug_interface == DEBUG_USART1) {
        __p_debug_usart = USART1;
    } else if(debug_interface == DEBUG_USART2) {
        __p_debug_usart = USART2;
    }
}

/**
  * @}
  */

/** @defgroup DEBUG_LL_Data_Transfers DEBUG LL Data transfers functions
  * @ingroup  DEBUG_LL_Driver
  * @brief    DEBUG LL Data transfers functions 
  * @{
  */
    
/**
  * @brief  Get char, Retargets the C library DBG_NOR function to the USART or ITM.
  * @param  None
  * @retval None
  */
GETCHAR_PROTOTYPE
{
    int val = 0;
    
    if(__debug_interface > DEBUG_ITM) {
        val = ll_usart_getchar(__p_debug_usart);
    } else if(__debug_interface == DEBUG_ITM) {
        while (ITM_CheckChar() != 1) __NOP();
        val = ITM_ReceiveChar();
    }
    
    return val;
}
    
/**
  * @brief  Put char, Retargets the C library DBG_NOR function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
    if(__debug_interface > DEBUG_ITM) {
        ll_usart_putchar(__p_debug_usart, ch);
    } else if(__debug_interface == DEBUG_ITM) {
        ITM_SendChar(ch);
    }
    
    return ch;
}

/** 
  * @brief  printf array.
  * @param  ptr: the address of the data which will send to RS485 TX pin.
  * @param  len: the len of data which will send.
  * @retval None.
  */
void printf_array(void *ptr, u16 len)
{
    u8 cnt    = 0;
    u8 *p_ptr = (u8 *)ptr;
    
    while(len--) {
        cnt++;
        DBG_NOR("%02x ", *p_ptr);
        p_ptr++;
        
        if((cnt & 0x0f) == 0x00) {
            DBG_NOR("\r\n");
        }
    }
    
    if((cnt & 0x0f) != 0x0f) {
        DBG_NOR("\r\n");
    }
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
