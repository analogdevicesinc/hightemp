/***************************************************************************************
 * @file     driver_common.h
 * @version  V1.0
 * @date     31. March 2016
 *
 * @note
 * VORAGO Technologies 
 *
 * @note
 * Copyright (c) 2013-2016 VORAGO Technologies. 
 *
 * @par
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND BY 
 * ALL THE TERMS AND CONDITIONS OF THE VORAGO TECHNOLOGIES END USER LICENSE AGREEMENT. 
 * THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. VORAGO TECHNOLOGIES 
 * SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ****************************************************************************************/
#ifndef __DRIVER_COMMON_H
#define __DRIVER_COMMON_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define GPORTA  0
#define GPORTB  1

#define FUNSEL0 0
#define FUNSEL1 1
#define FUNSEL2 2
#define FUNSEL3 3

#define PROC_ID_TORCH 	0x040037E3
#define PROC_ID_ICEMAN 	0x040047E3

#define VOR_DRIVER_VERSION_MAJOR_MINOR(major,minor) (((major) << 8) | (minor))

/**
\brief Driver Version
*/
typedef struct _VOR_DRIVER_VERSION {
  uint16_t api;                         ///< API version
  uint16_t drv;                         ///< Driver version
} VOR_DRIVER_VERSION;

/* General return codes */
#define VOR_DRIVER_OK                 0 ///< Operation succeeded 
#define VOR_DRIVER_ERROR             -1 ///< Unspecified error
#define VOR_DRIVER_ERROR_BUSY        -2 ///< Driver is busy
#define VOR_DRIVER_ERROR_TIMEOUT     -3 ///< Timeout occurred
#define VOR_DRIVER_ERROR_UNSUPPORTED -4 ///< Operation not supported
#define VOR_DRIVER_ERROR_PARAMETER   -5 ///< Parameter error
#define VOR_DRIVER_ERROR_SPECIFIC    -6 ///< Start of driver specific errors 

uint32_t VOR_Get_ClockFreq(void); 
uint32_t VOR_Get_SysTime(void); 
void VOR_Sleep(uint32_t mtime);  
void VOR_GPIO_PinMux(uint32_t bank, uint32_t pin, uint32_t funsel); 

#endif /* __DRIVER_COMMON_H */
