/***************************************************************************************
 * @file     i2c_drv_api.c
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
#include "i2c_va108xx.h"
#include "driver_common.h"

extern I2C_RESOURCES I2C0_Resources;
extern I2C_RESOURCES I2C1_Resources;


/* I2C0 Driver wrapper functions */
static int32_t VOR_I2C0_Initialize (VOR_I2C_SignalEvent_t cb_event) {
  return (VOR_I2C_Initialize (cb_event, &I2C0_Resources));
}
static int32_t VOR_I2C0_Uninitialize (void) {
  return (VOR_I2C_Uninitialize (&I2C0_Resources));
}
static int32_t VOR_I2C0_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (VOR_I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t VOR_I2C0_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (VOR_I2C_MasterReceive (addr, data, num, xfer_pending, &I2C0_Resources));
}
static int32_t VOR_I2C0_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (VOR_I2C_SlaveTransmit (data, num, &I2C0_Resources));
}
static int32_t VOR_I2C0_SlaveReceive (uint8_t *data, uint32_t num) {
  return (VOR_I2C_SlaveReceive (data, num, &I2C0_Resources));
}
static int32_t VOR_I2C0_GetDataCount (void) {
  return (VOR_I2C_GetDataCount (&I2C0_Resources));
}
static int32_t VOR_I2C0_Control (uint32_t control, uint32_t arg) {
  return (VOR_I2C_Control (control, arg, &I2C0_Resources));
}
static VOR_I2C_STATUS VOR_I2C0_GetStatus (void) {
  return (VOR_I2C_GetStatus (&I2C0_Resources));
}
void VOR_I2C0_MS_IRQHandler (void) {
  VOR_I2C_MasterHandler (&I2C0_Resources);
}
void VOR_I2C0_SL_IRQHandler (void) {
  VOR_I2C_SlaveHandler (&I2C0_Resources);
}

/* I2C0 Driver Control Block */
VOR_DRIVER_I2C Driver_I2C0 = {
  VOR_I2Cx_GetVersion,
  VOR_I2C0_Initialize,
  VOR_I2C0_Uninitialize,
  VOR_I2C0_MasterTransmit,
  VOR_I2C0_MasterReceive,
  VOR_I2C0_SlaveTransmit,
  VOR_I2C0_SlaveReceive,
  VOR_I2C0_GetDataCount,
  VOR_I2C0_Control,
  VOR_I2C0_GetStatus
};

/* I2C1 Driver wrapper functions */
static int32_t VOR_I2C1_Initialize (VOR_I2C_SignalEvent_t cb_event) {
  return (VOR_I2C_Initialize (cb_event, &I2C1_Resources));
}
static int32_t VOR_I2C1_Uninitialize (void) {
  return (VOR_I2C_Uninitialize (&I2C1_Resources));
}
static int32_t VOR_I2C1_MasterTransmit (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) {
  return (VOR_I2C_MasterTransmit (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t VOR_I2C1_MasterReceive (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending) {
  return (VOR_I2C_MasterReceive (addr, data, num, xfer_pending, &I2C1_Resources));
}
static int32_t VOR_I2C1_SlaveTransmit (const uint8_t *data, uint32_t num) {
  return (VOR_I2C_SlaveTransmit (data, num, &I2C1_Resources));
}
static int32_t VOR_I2C1_SlaveReceive (uint8_t *data, uint32_t num) {
  return (VOR_I2C_SlaveReceive (data, num, &I2C1_Resources));
}
static int32_t VOR_I2C1_GetDataCount (void) {
  return (VOR_I2C_GetDataCount (&I2C1_Resources));
}
static int32_t VOR_I2C1_Control (uint32_t control, uint32_t arg) {
  return (VOR_I2C_Control (control, arg, &I2C1_Resources));
}
static VOR_I2C_STATUS VOR_I2C1_GetStatus (void) {
  return (VOR_I2C_GetStatus (&I2C1_Resources));
}
void VOR_I2C1_MS_IRQHandler (void) {
  VOR_I2C_MasterHandler (&I2C1_Resources);
}
void VOR_I2C1_SL_IRQHandler (void) {
  VOR_I2C_SlaveHandler (&I2C1_Resources);
}

/* I2C1 Driver Control Block */
VOR_DRIVER_I2C Driver_I2C1 = {
  VOR_I2Cx_GetVersion,
  VOR_I2C1_Initialize,
  VOR_I2C1_Uninitialize,
  VOR_I2C1_MasterTransmit,
  VOR_I2C1_MasterReceive,
  VOR_I2C1_SlaveTransmit,
  VOR_I2C1_SlaveReceive,
  VOR_I2C1_GetDataCount,
  VOR_I2C1_Control,
  VOR_I2C1_GetStatus
};
