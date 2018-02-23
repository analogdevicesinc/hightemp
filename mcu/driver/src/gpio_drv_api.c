/***************************************************************************************
 * @file     gpio_drv_api.c
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
#include "gpio_va108xx.h"
#include "driver_common.h"

#ifdef GPIO_DRIVER

extern GPIO_RESOURCES GPIO_Resources[MAX_GPIO_CNT];


#define GPIOA_FUN_DEF(pin)   \
  static int32_t VOR_GPIOA_##pin##_Initialize (VOR_GPIO_SignalEvent_t  cb_event) { \
	return VOR_GPIO_Initialize (cb_event, &GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_Uninitialize (void) { \
	return VOR_GPIO_Uninitialize(&GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_Read (void) { \
	return VOR_GPIO_Read(&GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_Write (uint32_t val) { \
	return VOR_GPIO_Write(val, &GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_ReadRaw (void) { \
	return VOR_GPIO_ReadRaw(&GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_WriteRaw (uint32_t val) { \
	return VOR_GPIO_WriteRaw(val, &GPIO_Resources[pin]);   \
  } \
  static int32_t VOR_GPIOA_##pin##_Control (uint32_t control, uint32_t arg) { \
	return VOR_GPIO_Control(control,arg, &GPIO_Resources[pin]);   \
  } \
  static VOR_GPIO_STATUS VOR_GPIOA_##pin##_GetStatus (void) { \
	return VOR_GPIO_GetStatus(&GPIO_Resources[pin]);   \
  } 

  #define GPIOB_FUN_DEF(pin)   \
  static int32_t VOR_GPIOB_##pin##_Initialize (VOR_GPIO_SignalEvent_t  cb_event) { \
	return VOR_GPIO_Initialize (cb_event, &GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_Uninitialize (void) { \
	return VOR_GPIO_Uninitialize(&GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_Read (void) { \
	return VOR_GPIO_Read(&GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_Write (uint32_t val) { \
	return VOR_GPIO_Write(val, &GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_ReadRaw (void) { \
	return VOR_GPIO_ReadRaw(&GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_WriteRaw (uint32_t val) { \
	return VOR_GPIO_WriteRaw(val, &GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static int32_t VOR_GPIOB_##pin##_Control (uint32_t control, uint32_t arg) { \
	return VOR_GPIO_Control(control,arg, &GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } \
  static VOR_GPIO_STATUS VOR_GPIOB_##pin##_GetStatus (void) { \
	return VOR_GPIO_GetStatus(&GPIO_Resources[MAX_GPIO_PORTA + pin]);   \
  } 
  
GPIOA_FUN_DEF(0)
GPIOA_FUN_DEF(1)
GPIOA_FUN_DEF(2)
GPIOA_FUN_DEF(3)
GPIOA_FUN_DEF(4)
GPIOA_FUN_DEF(5)
GPIOA_FUN_DEF(6)
GPIOA_FUN_DEF(7)
GPIOA_FUN_DEF(8)
GPIOA_FUN_DEF(9)
GPIOA_FUN_DEF(10)
GPIOA_FUN_DEF(11)
GPIOA_FUN_DEF(12)
GPIOA_FUN_DEF(13)
GPIOA_FUN_DEF(14)
GPIOA_FUN_DEF(15)
GPIOA_FUN_DEF(16)
GPIOA_FUN_DEF(17)
GPIOA_FUN_DEF(18)
GPIOA_FUN_DEF(19)
GPIOA_FUN_DEF(20)
GPIOA_FUN_DEF(21)
GPIOA_FUN_DEF(22)
GPIOA_FUN_DEF(23)
GPIOA_FUN_DEF(24)
GPIOA_FUN_DEF(25)
GPIOA_FUN_DEF(26)
GPIOA_FUN_DEF(27)
GPIOA_FUN_DEF(28)
GPIOA_FUN_DEF(29)
GPIOA_FUN_DEF(30)
GPIOA_FUN_DEF(31)
  
GPIOB_FUN_DEF(0)
GPIOB_FUN_DEF(1)
GPIOB_FUN_DEF(2)
GPIOB_FUN_DEF(3)
GPIOB_FUN_DEF(4)
GPIOB_FUN_DEF(5)
GPIOB_FUN_DEF(6)
GPIOB_FUN_DEF(7)
GPIOB_FUN_DEF(8)
GPIOB_FUN_DEF(9)
GPIOB_FUN_DEF(10)
GPIOB_FUN_DEF(11)
GPIOB_FUN_DEF(12)
GPIOB_FUN_DEF(13)
GPIOB_FUN_DEF(14)
GPIOB_FUN_DEF(15)
GPIOB_FUN_DEF(16)
GPIOB_FUN_DEF(17)
GPIOB_FUN_DEF(18)
GPIOB_FUN_DEF(19)
GPIOB_FUN_DEF(20)
GPIOB_FUN_DEF(21)
GPIOB_FUN_DEF(22)
GPIOB_FUN_DEF(23)


// GPIO Driver Control Block
VOR_DRIVER_GPIO Driver_GPIOA_0={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_0_Initialize, 
    VOR_GPIOA_0_Uninitialize, 
    VOR_GPIOA_0_Read,         
    VOR_GPIOA_0_Write,     
    VOR_GPIOA_0_ReadRaw,      
    VOR_GPIOA_0_WriteRaw,     
    VOR_GPIOA_0_Control,      
    VOR_GPIOA_0_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_1={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_1_Initialize, 
    VOR_GPIOA_1_Uninitialize,
    VOR_GPIOA_1_Read,         
    VOR_GPIOA_1_Write,        
    VOR_GPIOA_1_ReadRaw,      
    VOR_GPIOA_1_WriteRaw,     
    VOR_GPIOA_1_Control,      
    VOR_GPIOA_1_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_2={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_2_Initialize, 
    VOR_GPIOA_2_Uninitialize, 
    VOR_GPIOA_2_Read,         
    VOR_GPIOA_2_Write,        
    VOR_GPIOA_2_ReadRaw,      
    VOR_GPIOA_2_WriteRaw,     
    VOR_GPIOA_2_Control,      
    VOR_GPIOA_2_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_3={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_3_Initialize, 
    VOR_GPIOA_3_Uninitialize, 
    VOR_GPIOA_3_Read,         
    VOR_GPIOA_3_Write,        
    VOR_GPIOA_3_ReadRaw,      
    VOR_GPIOA_3_WriteRaw,     
    VOR_GPIOA_3_Control,      
    VOR_GPIOA_3_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_4={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_4_Initialize, 
    VOR_GPIOA_4_Uninitialize, 
    VOR_GPIOA_4_Read,         
    VOR_GPIOA_4_Write,        
    VOR_GPIOA_4_ReadRaw,      
    VOR_GPIOA_4_WriteRaw,     
    VOR_GPIOA_4_Control,      
    VOR_GPIOA_4_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_5={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_5_Initialize, 
    VOR_GPIOA_5_Uninitialize, 
    VOR_GPIOA_5_Read,         
    VOR_GPIOA_5_Write,        
    VOR_GPIOA_5_ReadRaw,      
    VOR_GPIOA_5_WriteRaw,     
    VOR_GPIOA_5_Control,      
    VOR_GPIOA_5_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_6={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_6_Initialize, 
    VOR_GPIOA_6_Uninitialize, 
    VOR_GPIOA_6_Read,         
    VOR_GPIOA_6_Write,        
    VOR_GPIOA_6_ReadRaw,      
    VOR_GPIOA_6_WriteRaw,     
    VOR_GPIOA_6_Control,      
    VOR_GPIOA_6_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_7={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_7_Initialize, 
    VOR_GPIOA_7_Uninitialize, 
    VOR_GPIOA_7_Read,         
    VOR_GPIOA_7_Write,        
    VOR_GPIOA_7_ReadRaw,      
    VOR_GPIOA_7_WriteRaw,     
    VOR_GPIOA_7_Control,      
    VOR_GPIOA_7_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_8={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_8_Initialize, 
    VOR_GPIOA_8_Uninitialize, 
    VOR_GPIOA_8_Read,         
    VOR_GPIOA_8_Write,        
    VOR_GPIOA_8_ReadRaw,      
    VOR_GPIOA_8_WriteRaw,     
    VOR_GPIOA_8_Control,      
    VOR_GPIOA_8_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_9={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_9_Initialize, 
    VOR_GPIOA_9_Uninitialize, 
    VOR_GPIOA_9_Read,         
    VOR_GPIOA_9_Write,        
    VOR_GPIOA_9_ReadRaw,      
    VOR_GPIOA_9_WriteRaw,     
    VOR_GPIOA_9_Control,      
    VOR_GPIOA_9_GetStatus};
		
VOR_DRIVER_GPIO Driver_GPIOA_10={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_10_Initialize, 
    VOR_GPIOA_10_Uninitialize, 
    VOR_GPIOA_10_Read,         
    VOR_GPIOA_10_Write,        
    VOR_GPIOA_10_ReadRaw,      
    VOR_GPIOA_10_WriteRaw,     
    VOR_GPIOA_10_Control,      
    VOR_GPIOA_10_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_11={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_11_Initialize, 
    VOR_GPIOA_11_Uninitialize, 
    VOR_GPIOA_11_Read,         
    VOR_GPIOA_11_Write,        
    VOR_GPIOA_11_ReadRaw,      
    VOR_GPIOA_11_WriteRaw,     
    VOR_GPIOA_11_Control,      
    VOR_GPIOA_11_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_12={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_12_Initialize, 
    VOR_GPIOA_12_Uninitialize, 
    VOR_GPIOA_12_Read,         
    VOR_GPIOA_12_Write,        
    VOR_GPIOA_12_ReadRaw,      
    VOR_GPIOA_12_WriteRaw,     
    VOR_GPIOA_12_Control,      
    VOR_GPIOA_12_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_13={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_13_Initialize, 
    VOR_GPIOA_13_Uninitialize, 
    VOR_GPIOA_13_Read,         
    VOR_GPIOA_13_Write,        
    VOR_GPIOA_13_ReadRaw,      
    VOR_GPIOA_13_WriteRaw,     
    VOR_GPIOA_13_Control,      
    VOR_GPIOA_13_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_14={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_14_Initialize, 
    VOR_GPIOA_14_Uninitialize, 
    VOR_GPIOA_14_Read,         
    VOR_GPIOA_14_Write,        
    VOR_GPIOA_14_ReadRaw,      
    VOR_GPIOA_14_WriteRaw,     
    VOR_GPIOA_14_Control,      
    VOR_GPIOA_14_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_15={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_15_Initialize, 
    VOR_GPIOA_15_Uninitialize, 
    VOR_GPIOA_15_Read,         
    VOR_GPIOA_15_Write,        
    VOR_GPIOA_15_ReadRaw,      
    VOR_GPIOA_15_WriteRaw,     
    VOR_GPIOA_15_Control,      
    VOR_GPIOA_15_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_16={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_16_Initialize, 
    VOR_GPIOA_16_Uninitialize, 
    VOR_GPIOA_16_Read,         
    VOR_GPIOA_16_Write,        
    VOR_GPIOA_16_ReadRaw,      
    VOR_GPIOA_16_WriteRaw,     
    VOR_GPIOA_16_Control,      
    VOR_GPIOA_16_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_17={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_17_Initialize, 
    VOR_GPIOA_17_Uninitialize, 
    VOR_GPIOA_17_Read,         
    VOR_GPIOA_17_Write,        
    VOR_GPIOA_17_ReadRaw,      
    VOR_GPIOA_17_WriteRaw,     
    VOR_GPIOA_17_Control,      
    VOR_GPIOA_17_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_18={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_18_Initialize, 
    VOR_GPIOA_18_Uninitialize, 
    VOR_GPIOA_18_Read,         
    VOR_GPIOA_18_Write,        
    VOR_GPIOA_18_ReadRaw,      
    VOR_GPIOA_18_WriteRaw,     
    VOR_GPIOA_18_Control,      
    VOR_GPIOA_18_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_19={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_19_Initialize, 
    VOR_GPIOA_19_Uninitialize, 
    VOR_GPIOA_19_Read,         
    VOR_GPIOA_19_Write,        
    VOR_GPIOA_19_ReadRaw,      
    VOR_GPIOA_19_WriteRaw,     
    VOR_GPIOA_19_Control,      
    VOR_GPIOA_19_GetStatus};
		
VOR_DRIVER_GPIO Driver_GPIOA_20={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_20_Initialize, 
    VOR_GPIOA_20_Uninitialize, 
    VOR_GPIOA_20_Read,         
    VOR_GPIOA_20_Write,        
    VOR_GPIOA_20_ReadRaw,      
    VOR_GPIOA_20_WriteRaw,     
    VOR_GPIOA_20_Control,      
    VOR_GPIOA_20_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_21={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_21_Initialize, 
    VOR_GPIOA_21_Uninitialize, 
    VOR_GPIOA_21_Read,         
    VOR_GPIOA_21_Write,        
    VOR_GPIOA_21_ReadRaw,      
    VOR_GPIOA_21_WriteRaw,     
    VOR_GPIOA_21_Control,      
    VOR_GPIOA_21_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_22={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_22_Initialize, 
    VOR_GPIOA_22_Uninitialize, 
    VOR_GPIOA_22_Read,         
    VOR_GPIOA_22_Write,        
    VOR_GPIOA_22_ReadRaw,      
    VOR_GPIOA_22_WriteRaw,     
    VOR_GPIOA_22_Control,      
    VOR_GPIOA_22_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_23={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_23_Initialize, 
    VOR_GPIOA_23_Uninitialize, 
    VOR_GPIOA_23_Read,         
    VOR_GPIOA_23_Write,        
    VOR_GPIOA_23_ReadRaw,      
    VOR_GPIOA_23_WriteRaw,     
    VOR_GPIOA_23_Control,      
    VOR_GPIOA_23_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_24={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_24_Initialize, 
    VOR_GPIOA_24_Uninitialize, 
    VOR_GPIOA_24_Read,         
    VOR_GPIOA_24_Write,        
    VOR_GPIOA_24_ReadRaw,      
    VOR_GPIOA_24_WriteRaw,     
    VOR_GPIOA_24_Control,      
    VOR_GPIOA_24_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_25={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_25_Initialize, 
    VOR_GPIOA_25_Uninitialize, 
    VOR_GPIOA_25_Read,         
    VOR_GPIOA_25_Write,        
    VOR_GPIOA_25_ReadRaw,      
    VOR_GPIOA_25_WriteRaw,     
    VOR_GPIOA_25_Control,      
    VOR_GPIOA_25_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_26={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_26_Initialize, 
    VOR_GPIOA_26_Uninitialize, 
    VOR_GPIOA_26_Read,         
    VOR_GPIOA_26_Write,        
    VOR_GPIOA_26_ReadRaw,      
    VOR_GPIOA_26_WriteRaw,     
    VOR_GPIOA_26_Control,      
    VOR_GPIOA_26_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_27={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_27_Initialize, 
    VOR_GPIOA_27_Uninitialize, 
    VOR_GPIOA_27_Read,         
    VOR_GPIOA_27_Write,        
    VOR_GPIOA_27_ReadRaw,      
    VOR_GPIOA_27_WriteRaw,     
    VOR_GPIOA_27_Control,      
    VOR_GPIOA_27_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_28={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_28_Initialize, 
    VOR_GPIOA_28_Uninitialize, 
    VOR_GPIOA_28_Read,         
    VOR_GPIOA_28_Write,        
    VOR_GPIOA_28_ReadRaw,      
    VOR_GPIOA_28_WriteRaw,     
    VOR_GPIOA_28_Control,      
    VOR_GPIOA_28_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_29={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_29_Initialize, 
    VOR_GPIOA_29_Uninitialize, 
    VOR_GPIOA_29_Read,         
    VOR_GPIOA_29_Write,        
    VOR_GPIOA_29_ReadRaw,      
    VOR_GPIOA_29_WriteRaw,     
    VOR_GPIOA_29_Control,      
    VOR_GPIOA_29_GetStatus};
		
VOR_DRIVER_GPIO Driver_GPIOA_30={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_30_Initialize, 
    VOR_GPIOA_30_Uninitialize, 
    VOR_GPIOA_30_Read,         
    VOR_GPIOA_30_Write,        
    VOR_GPIOA_30_ReadRaw,      
    VOR_GPIOA_30_WriteRaw,     
    VOR_GPIOA_30_Control,      
    VOR_GPIOA_30_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOA_31={
    VOR_GPIOx_GetVersion,
    VOR_GPIOA_31_Initialize, 
    VOR_GPIOA_31_Uninitialize, 
    VOR_GPIOA_31_Read,         
    VOR_GPIOA_31_Write,        
    VOR_GPIOA_31_ReadRaw,      
    VOR_GPIOA_31_WriteRaw,     
    VOR_GPIOA_31_Control,      
    VOR_GPIOA_31_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_0={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_0_Initialize, 
    VOR_GPIOB_0_Uninitialize, 
    VOR_GPIOB_0_Read,         
    VOR_GPIOB_0_Write,        
    VOR_GPIOB_0_ReadRaw,      
    VOR_GPIOB_0_WriteRaw,     
    VOR_GPIOB_0_Control,      
    VOR_GPIOB_0_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_1={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_1_Initialize, 
    VOR_GPIOB_1_Uninitialize, 
    VOR_GPIOB_1_Read,         
    VOR_GPIOB_1_Write,        
    VOR_GPIOB_1_ReadRaw,      
    VOR_GPIOB_1_WriteRaw,     
    VOR_GPIOB_1_Control,      
    VOR_GPIOB_1_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_2={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_2_Initialize, 
    VOR_GPIOB_2_Uninitialize, 
    VOR_GPIOB_2_Read,         
    VOR_GPIOB_2_Write,        
    VOR_GPIOB_2_ReadRaw,      
    VOR_GPIOB_2_WriteRaw,     
    VOR_GPIOB_2_Control,      
    VOR_GPIOB_2_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_3={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_3_Initialize, 
    VOR_GPIOB_3_Uninitialize, 
    VOR_GPIOB_3_Read,         
    VOR_GPIOB_3_Write,        
    VOR_GPIOB_3_ReadRaw,      
    VOR_GPIOB_3_WriteRaw,     
    VOR_GPIOB_3_Control,      
    VOR_GPIOB_3_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_4={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_4_Initialize, 
    VOR_GPIOB_4_Uninitialize, 
    VOR_GPIOB_4_Read,         
    VOR_GPIOB_4_Write,        
    VOR_GPIOB_4_ReadRaw,      
    VOR_GPIOB_4_WriteRaw,     
    VOR_GPIOB_4_Control,      
    VOR_GPIOB_4_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_5={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_5_Initialize, 
    VOR_GPIOB_5_Uninitialize, 
    VOR_GPIOB_5_Read,         
    VOR_GPIOB_5_Write,        
    VOR_GPIOB_5_ReadRaw,      
    VOR_GPIOB_5_WriteRaw,     
    VOR_GPIOB_5_Control,      
    VOR_GPIOB_5_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_6={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_6_Initialize, 
    VOR_GPIOB_6_Uninitialize, 
    VOR_GPIOB_6_Read,         
    VOR_GPIOB_6_Write,        
    VOR_GPIOB_6_ReadRaw,      
    VOR_GPIOB_6_WriteRaw,     
    VOR_GPIOB_6_Control,      
    VOR_GPIOB_6_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_7={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_7_Initialize, 
    VOR_GPIOB_7_Uninitialize, 
    VOR_GPIOB_7_Read,         
    VOR_GPIOB_7_Write,        
    VOR_GPIOB_7_ReadRaw,      
    VOR_GPIOB_7_WriteRaw,     
    VOR_GPIOB_7_Control,      
    VOR_GPIOB_7_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_8={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_8_Initialize, 
    VOR_GPIOB_8_Uninitialize, 
    VOR_GPIOB_8_Read,         
    VOR_GPIOB_8_Write,        
    VOR_GPIOB_8_ReadRaw,      
    VOR_GPIOB_8_WriteRaw,     
    VOR_GPIOB_8_Control,      
    VOR_GPIOB_8_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_9={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_9_Initialize, 
    VOR_GPIOB_9_Uninitialize, 
    VOR_GPIOB_9_Read,         
    VOR_GPIOB_9_Write,        
    VOR_GPIOB_9_ReadRaw,      
    VOR_GPIOB_9_WriteRaw,     
    VOR_GPIOB_9_Control,      
    VOR_GPIOB_9_GetStatus};
		
VOR_DRIVER_GPIO Driver_GPIOB_10={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_10_Initialize, 
    VOR_GPIOB_10_Uninitialize, 
    VOR_GPIOB_10_Read,         
    VOR_GPIOB_10_Write,        
    VOR_GPIOB_10_ReadRaw,      
    VOR_GPIOB_10_WriteRaw,     
    VOR_GPIOB_10_Control,      
    VOR_GPIOB_10_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_11={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_11_Initialize, 
    VOR_GPIOB_11_Uninitialize, 
    VOR_GPIOB_11_Read,         
    VOR_GPIOB_11_Write,        
    VOR_GPIOB_11_ReadRaw,      
    VOR_GPIOB_11_WriteRaw,     
    VOR_GPIOB_11_Control,      
    VOR_GPIOB_11_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_12={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_12_Initialize, 
    VOR_GPIOB_12_Uninitialize, 
    VOR_GPIOB_12_Read,         
    VOR_GPIOB_12_Write,        
    VOR_GPIOB_12_ReadRaw,      
    VOR_GPIOB_12_WriteRaw,     
    VOR_GPIOB_12_Control,      
    VOR_GPIOB_12_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_13={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_13_Initialize, 
    VOR_GPIOB_13_Uninitialize, 
    VOR_GPIOB_13_Read,         
    VOR_GPIOB_13_Write,        
    VOR_GPIOB_13_ReadRaw,      
    VOR_GPIOB_13_WriteRaw,     
    VOR_GPIOB_13_Control,      
    VOR_GPIOB_13_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_14={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_14_Initialize, 
    VOR_GPIOB_14_Uninitialize, 
    VOR_GPIOB_14_Read,         
    VOR_GPIOB_14_Write,        
    VOR_GPIOB_14_ReadRaw,      
    VOR_GPIOB_14_WriteRaw,     
    VOR_GPIOB_14_Control,      
    VOR_GPIOB_14_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_15={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_15_Initialize, 
    VOR_GPIOB_15_Uninitialize, 
    VOR_GPIOB_15_Read,         
    VOR_GPIOB_15_Write,        
    VOR_GPIOB_15_ReadRaw,      
    VOR_GPIOB_15_WriteRaw,     
    VOR_GPIOB_15_Control,      
    VOR_GPIOB_15_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_16={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_16_Initialize, 
    VOR_GPIOB_16_Uninitialize, 
    VOR_GPIOB_16_Read,         
    VOR_GPIOB_16_Write,        
    VOR_GPIOB_16_ReadRaw,      
    VOR_GPIOB_16_WriteRaw,     
    VOR_GPIOB_16_Control,      
    VOR_GPIOB_16_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_17={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_17_Initialize, 
    VOR_GPIOB_17_Uninitialize, 
    VOR_GPIOB_17_Read,         
    VOR_GPIOB_17_Write,        
    VOR_GPIOB_17_ReadRaw,      
    VOR_GPIOB_17_WriteRaw,     
    VOR_GPIOB_17_Control,      
    VOR_GPIOB_17_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_18={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_18_Initialize, 
    VOR_GPIOB_18_Uninitialize, 
    VOR_GPIOB_18_Read,         
    VOR_GPIOB_18_Write,        
    VOR_GPIOB_18_ReadRaw,      
    VOR_GPIOB_18_WriteRaw,     
    VOR_GPIOB_18_Control,      
    VOR_GPIOB_18_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_19={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_19_Initialize, 
    VOR_GPIOB_19_Uninitialize, 
    VOR_GPIOB_19_Read,         
    VOR_GPIOB_19_Write,        
    VOR_GPIOB_19_ReadRaw,      
    VOR_GPIOB_19_WriteRaw,     
    VOR_GPIOB_19_Control,      
    VOR_GPIOB_19_GetStatus};
		
VOR_DRIVER_GPIO Driver_GPIOB_20={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_20_Initialize, 
    VOR_GPIOB_20_Uninitialize, 
    VOR_GPIOB_20_Read,         
    VOR_GPIOB_20_Write,        
    VOR_GPIOB_20_ReadRaw,      
    VOR_GPIOB_20_WriteRaw,     
    VOR_GPIOB_20_Control,      
    VOR_GPIOB_20_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_21={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_21_Initialize, 
    VOR_GPIOB_21_Uninitialize, 
    VOR_GPIOB_21_Read,         
    VOR_GPIOB_21_Write,        
    VOR_GPIOB_21_ReadRaw,      
    VOR_GPIOB_21_WriteRaw,     
    VOR_GPIOB_21_Control,      
    VOR_GPIOB_21_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_22={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_22_Initialize, 
    VOR_GPIOB_22_Uninitialize, 
    VOR_GPIOB_22_Read,         
    VOR_GPIOB_22_Write,        
    VOR_GPIOB_22_ReadRaw,      
    VOR_GPIOB_22_WriteRaw,     
    VOR_GPIOB_22_Control,      
    VOR_GPIOB_22_GetStatus};

VOR_DRIVER_GPIO Driver_GPIOB_23={
    VOR_GPIOx_GetVersion,
    VOR_GPIOB_23_Initialize, 
    VOR_GPIOB_23_Uninitialize, 
    VOR_GPIOB_23_Read,         
    VOR_GPIOB_23_Write,        
    VOR_GPIOB_23_ReadRaw,      
    VOR_GPIOB_23_WriteRaw,     
    VOR_GPIOB_23_Control,      
    VOR_GPIOB_23_GetStatus};
#endif 

