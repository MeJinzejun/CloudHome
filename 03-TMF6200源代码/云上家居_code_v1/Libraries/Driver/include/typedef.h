/**
  ******************************************************************************
  * @file    Libraries/Driver/include/typedef.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   Type definition
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  *
  ******************************************************************************
  */ 

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup TYPEDEF
  * @{
  */

#ifndef __TYPEDEF_H
#define __TYPEDEF_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>


/** @addtogroup Exported_macro
  * @{
  */
#define BIT(a)                                  ((uint32_t)1<<(a))
     
#define BIT_64(a)                               ((uint64_t)1<<(a))

#define SET_BIT(REG, BIT)                       ((REG) |= (BIT))

#define CLEAR_BIT(REG, BIT)                     ((REG) &= ~(BIT))

#define READ_BIT(REG, BIT)                      ((REG) & (BIT))

#define CLEAR_REG(REG)                          ((REG) = (0x0))

#define WRITE_REG(REG, VAL)                     ((REG) = (VAL))

#define READ_REG(REG)                           ((REG))

#define MODIFY_REG(REG, CLEARMASK, SETMASK)     WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

#define NONSENSE(x) ((void)(x))

/**
  * @}
  */

/** @addtogroup Exported_types
  * @{
  */
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

/* Standard Peripheral Library old types (maintained for legacy purpose) */
typedef int64_t         s64;
typedef int32_t         s32;
typedef int16_t         s16;
typedef int8_t          s8;

typedef const int32_t   sc32;           /*!< Read Only */
typedef const int16_t   sc16;           /*!< Read Only */
typedef const int8_t    sc8;            /*!< Read Only */

typedef __IO int32_t    vs32;
typedef __IO int16_t    vs16;
typedef __IO int8_t     vs8;

typedef __I int32_t     vsc32;          /*!< Read Only */
typedef __I int16_t     vsc16;          /*!< Read Only */
typedef __I int8_t      vsc8;           /*!< Read Only */

typedef uint64_t        u64;
typedef uint32_t        u32;
typedef uint16_t        u16;
typedef uint8_t         u8;

typedef const uint32_t  uc32;           /*!< Read Only */
typedef const uint16_t  uc16;           /*!< Read Only */
typedef const uint8_t   uc8;            /*!< Read Only */

typedef __IO uint32_t   vu32;
typedef __IO uint16_t   vu16;
typedef __IO uint8_t    vu8;

typedef __I uint32_t    vuc32;          /*!< Read Only */
typedef __I uint16_t    vuc16;          /*!< Read Only */
typedef __I uint8_t     vuc8;           /*!< Read Only */

typedef enum {false = 0, true = 1, FALSE = 0, TRUE = 1} BOOL, bool;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;
typedef enum {HIGH = 1, LOW = 0} Polarity;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __TYPEDEF_H */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/
