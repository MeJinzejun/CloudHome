/**
  ******************************************************************************
  * @file    Libraries/Protocol/Modbus/functions/mbfuncuser.h
  * @author  HUGE-IC Application Team
  * @version V1.0.0
  * @date    01-08-2018
  * @brief   Main program body header file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2018 HUGE-IC</center></h2>
  *
  *
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MBFUNCUSER_H__
#define __MBFUNCUSER_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "typedef.h"
     
/** @addtogroup Template_Project
  * @{
  */

/** @addtogroup MBFUNCUSER
  * @{
  */ 

     
/** @defgroup MBFUNCUSER_Exported_Constants
  * @{
  */
     

/**
  * @}
  */


/** @defgroup MBFUNCUSER_Exported_Functions
  * @{
  */

/**
  * @brief  Format conversion of modbus data response, the format is 
  *         {command code (1 byte) + data length n (1 byte) + data (n byte)}.
  * @param  p_respond_frame_buf: The buffer address of the Modbus response data.
  * @param  p_respond_len: The address of the Modbus response data length.
  * @param  mb_respond_cmd: The command code for the Modbus response.
  * @param  p_mb_respond_data_buf: Modbus response data.
  * @param  mb_respond_data_len: The data length of the Modbus response.
  * @retval None
  * @note   1.The data format in the received pucFrame is 
  *           {command code (1 byte) + data length n (1 byte) + data (n byte)} or
  *           {command code (1 byte) + data (n byte)}.
  *         2.If function mb_respond_cmd_len_data_conversion() is called, the data format returned 
  *           in pucFrame is {command code (1 byte) + data length n (1 byte) + data (n byte)}.
  *         3.At the same time, you need to assign the variable p_respond_len to the length of the returned data.
  */
__STATIC_INLINE void mb_respond_cmd_len_data_conversion(UCHAR  *p_respond_frame_buf, 
                                                        USHORT *p_respond_len, 
                                                        u8      mb_respond_cmd, 
                                                        u8     *p_mb_respond_data_buf, 
                                                        u16     mb_respond_data_len) {
    /* Assign the return data. The format is 
     * {command code (1 byte) + data length n (1 byte) + data (n byte)}.
     */
    *(p_respond_frame_buf+0) = mb_respond_cmd;                                  //command code
    *(p_respond_frame_buf+1) = 0x03;                                            //data length
    memcpy(p_respond_frame_buf+2, p_mb_respond_data_buf, mb_respond_data_len);  //data
    
    /* Set the total length of the returned data.
     * length = command code (1 byte) + data length n (1 byte) + data (n byte).
     */
    *p_respond_len = mb_respond_data_len + 2;
}

/**
  * @brief  Format conversion of modbus data response, the format is {command code (1 byte) + data (n byte)}.
  * @param  p_respond_frame_buf: The buffer address of the Modbus response data.
  * @param  p_respond_len: The address of the Modbus response data length.
  * @param  mb_respond_cmd: The command code for the Modbus response.
  * @param  p_mb_respond_data_buf: Modbus response data.
  * @param  mb_respond_data_len: The data length of the Modbus response.
  * @retval None
  * @note   1.The data format in the received pucFrame is 
  *           {command code (1 byte) + data length n (1 byte) + data (n byte)} or
  *           {command code (1 byte) + data (n byte)}.
  *         2.If function mb_respond_cmd_len_data_conversion() is called, the data format returned 
  *           in pucFrame is {command code (1 byte) + data (n byte)}.
  *         3.At the same time, you need to assign the variable p_respond_len to the length of the returned data.
  */
__STATIC_INLINE void mb_respond_cmd_data_conversion(UCHAR  *p_respond_frame_buf, 
                                                        USHORT *p_respond_len, 
                                                        u8      mb_respond_cmd, 
                                                        u8     *p_mb_respond_data_buf, 
                                                        u16     mb_respond_data_len) {
    /* Assign the return data. The format is 
     * {command code (1 byte) + data (n byte)}.
     */
    *(p_respond_frame_buf+0) = mb_respond_cmd;                                  //command code
    memcpy(p_respond_frame_buf+1, p_mb_respond_data_buf, mb_respond_data_len);  //data
    
    /* Set the total length of the returned data.
     * length = command code (1 byte) + data (n byte).
     */
    *p_respond_len = mb_respond_data_len + 1;
}


/**
  * @}
  */ 


#ifdef __cplusplus
}
#endif

#endif //__MBFUNCUSER_H__

/**
  * @}
  */

/**
  * @}
  */

/*************************** (C) COPYRIGHT 2018 HUGE-IC ***** END OF FILE *****/

