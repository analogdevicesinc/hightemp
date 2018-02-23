/***************************************************************************************
 * @file     spi_drv_api.c
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
#include  "spi_va108xx.h"
#include  "driver_common.h"


extern VOR_SPI_RESOURCES SPI0_Resources;
extern VOR_SPI_RESOURCES SPI1_Resources;
extern VOR_SPI_RESOURCES SPI2_Resources;

/* SPI0 Driver wrapper functions */

static int32_t VOR_SPI0_Initialize (VOR_SPI_SignalEvent_t  cb_event) { 
	return VOR_SPI_Initialize (cb_event, &SPI0_Resources);   
}

static int32_t VOR_SPI0_Uninitialize (void) { 
	return VOR_SPI_Uninitialize (&SPI0_Resources);   
} 

static int32_t VOR_SPI0_Send (uint8_t *data, uint32_t num) { 
	return VOR_SPI_Send (data, num, &SPI0_Resources);   
} 

static int32_t VOR_SPI0_Receive (uint8_t *data, uint32_t num) { 
	return VOR_SPI_Receive (data, num, &SPI0_Resources);   
} 

static int32_t VOR_SPI0_Transfer (uint8_t *dataIn,uint8_t *dataOut,uint32_t num) { 
	return VOR_SPI_Transfer(dataIn,dataOut, num, &SPI0_Resources);   
} 

static int32_t VOR_SPI0_Control (uint32_t control, uint32_t arg) { 
	return VOR_SPI_Control (control, arg, &SPI0_Resources);   
} 

static VOR_SPI_STATUS VOR_SPI0_GetStatus (void) { 
	return VOR_SPI_GetStatus (&SPI0_Resources);   
} 

static uint32_t VOR_SPI0_GetTxCount (void) {
  return VOR_SPI_GetTxCount(&SPI0_Resources);
}

static uint32_t VOR_SPI0_GetRxCount (void) {
  return VOR_SPI_GetRxCount(&SPI0_Resources);
}

void VOR_SPI0_IRQHandler (void) {
   VOR_SPI_IrqHandler (&SPI0_Resources);
}

// SPI Driver Control Block
VOR_DRIVER_SPI Driver_SPI0={
	  VOR_SPIx_GetVersion,
    VOR_SPI0_Initialize, 
    VOR_SPI0_Uninitialize, 
    VOR_SPI0_Send,         
    VOR_SPI0_Receive, 
		VOR_SPI0_Transfer,
    VOR_SPI0_Control,      
    VOR_SPI0_GetStatus,
    VOR_SPI0_GetTxCount,
	  VOR_SPI0_GetRxCount
 };

/* SPI1 Driver wrapper functions */

static int32_t VOR_SPI1_Initialize (VOR_SPI_SignalEvent_t  cb_event) { 
	return VOR_SPI_Initialize (cb_event, &SPI1_Resources);   
}

static int32_t VOR_SPI1_Uninitialize (void) { 
	return VOR_SPI_Uninitialize (&SPI1_Resources);   
} 

static int32_t VOR_SPI1_Send ( uint8_t *data, uint32_t num) { 
	return VOR_SPI_Send (data, num, &SPI1_Resources);   
} 

static int32_t VOR_SPI1_Receive (uint8_t *data, uint32_t num) { 
	return VOR_SPI_Receive (data, num, &SPI1_Resources);   
} 

static int32_t VOR_SPI1_Transfer (uint8_t *dataIn,uint8_t *dataOut,uint32_t num) { 
	return VOR_SPI_Transfer(dataIn,dataOut, num, &SPI1_Resources);   
} 

static int32_t VOR_SPI1_Control (uint32_t control, uint32_t arg) { 
	return VOR_SPI_Control (control, arg, &SPI1_Resources);   
} 

static VOR_SPI_STATUS VOR_SPI1_GetStatus (void) { 
	return VOR_SPI_GetStatus (&SPI1_Resources);   
} 

void VOR_SPI1_IRQHandler (void) {
   VOR_SPI_IrqHandler (&SPI1_Resources);
}

static uint32_t VOR_SPI1_GetTxCount (void) {
  return VOR_SPI_GetTxCount(&SPI1_Resources);
}

static uint32_t VOR_SPI1_GetRxCount (void) {
  return VOR_SPI_GetRxCount(&SPI1_Resources);
}



// SPI1 Driver Control Block
VOR_DRIVER_SPI Driver_SPI1={
	  VOR_SPIx_GetVersion,
	  VOR_SPI1_Initialize, 
    VOR_SPI1_Uninitialize, 
    VOR_SPI1_Send,         
    VOR_SPI1_Receive,   
    VOR_SPI1_Transfer,
    VOR_SPI1_Control,      
    VOR_SPI1_GetStatus,
    VOR_SPI1_GetTxCount,
	  VOR_SPI1_GetRxCount
};

/* SPI2 Driver wrapper functions */

static int32_t VOR_SPI2_Initialize (VOR_SPI_SignalEvent_t  cb_event) { 
	return VOR_SPI_Initialize (cb_event, &SPI2_Resources);   
}

static int32_t VOR_SPI2_Uninitialize (void) { 
	return VOR_SPI_Uninitialize (&SPI2_Resources);   
} 

static int32_t VOR_SPI2_Send ( uint8_t *data, uint32_t num) { 
	return VOR_SPI_Send (data, num, &SPI2_Resources);   
} 

static int32_t VOR_SPI2_Receive (uint8_t *data, uint32_t num) { 
	return VOR_SPI_Receive (data, num, &SPI2_Resources);   
} 

static int32_t VOR_SPI2_Transfer (uint8_t *dataIn,uint8_t *dataOut,uint32_t num) { 
	return VOR_SPI_Transfer(dataIn,dataOut, num, &SPI2_Resources);   
} 

static int32_t VOR_SPI2_Control (uint32_t control, uint32_t arg) { 
	return VOR_SPI_Control (control, arg, &SPI2_Resources);   
} 

static VOR_SPI_STATUS VOR_SPI2_GetStatus (void) { 
	return VOR_SPI_GetStatus (&SPI2_Resources);   
} 

void VOR_SPI2_IRQHandler (void) {
   VOR_SPI_IrqHandler (&SPI2_Resources);
}

static uint32_t VOR_SPI2_GetTxCount (void) {
  return VOR_SPI_GetTxCount(&SPI2_Resources);
}

static uint32_t VOR_SPI2_GetRxCount (void) {
  return VOR_SPI_GetRxCount(&SPI2_Resources);
}


// SPI2 Driver Control Block
VOR_DRIVER_SPI Driver_SPI2 = {
	  VOR_SPIx_GetVersion,
	  VOR_SPI2_Initialize, 
    VOR_SPI2_Uninitialize, 
    VOR_SPI2_Send,         
    VOR_SPI2_Receive,   
    VOR_SPI2_Transfer,
    VOR_SPI2_Control,      
    VOR_SPI2_GetStatus,
    VOR_SPI2_GetTxCount,
	  VOR_SPI2_GetRxCount
};
