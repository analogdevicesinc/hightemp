/***************************************************************************************
 * @file     gpio_va108xx.c
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
#include "irq_va108xx.h"

#ifdef GPIO_DRIVER
GPIO_RESOURCES GPIO_Resources[MAX_GPIO_CNT] = {
{{GPIO_PORTA,0},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_0,{0}},
{{GPIO_PORTA,1},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_1,{0}},
{{GPIO_PORTA,2},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_2,{0}},
{{GPIO_PORTA,3},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_3,{0}},
{{GPIO_PORTA,4},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_4,{0}},
{{GPIO_PORTA,5},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_5,{0}},
{{GPIO_PORTA,6},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_6,{0}},
{{GPIO_PORTA,7},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_7,{0}},
{{GPIO_PORTA,8},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_8,{0}},
{{GPIO_PORTA,9},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_9,{0}},
{{GPIO_PORTA,10},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_10,{0}},
{{GPIO_PORTA,11},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_11,{0}},
{{GPIO_PORTA,12},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_12,{0}},
{{GPIO_PORTA,13},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_13,{0}},
{{GPIO_PORTA,14},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_14,{0}},
{{GPIO_PORTA,15},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_15,{0}},
{{GPIO_PORTA,16},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_16,{0}},
{{GPIO_PORTA,17},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_17,{0}},
{{GPIO_PORTA,18},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_18,{0}},
{{GPIO_PORTA,19},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_19,{0}},
{{GPIO_PORTA,20},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_20,{0}},
{{GPIO_PORTA,21},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_21,{0}},
{{GPIO_PORTA,22},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_22,{0}},
{{GPIO_PORTA,23},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_23,{0}},
{{GPIO_PORTA,24},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_24,{0}},
{{GPIO_PORTA,25},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_25,{0}},
{{GPIO_PORTA,26},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_26,{0}},
{{GPIO_PORTA,27},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_27,{0}},
{{GPIO_PORTA,28},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_28,{0}},
{{GPIO_PORTA,29},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_29,{0}},
{{GPIO_PORTA,30},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_30,{0}},
{{GPIO_PORTA,31},&VOR_GPIO->BANK[GPIO_PORTA],&Driver_GPIOA_31,{0}},
{{GPIO_PORTB,0},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_0,{0}},
{{GPIO_PORTB,1},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_1,{0}},
{{GPIO_PORTB,2},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_2,{0}},
{{GPIO_PORTB,3},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_3,{0}},
{{GPIO_PORTB,4},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_4,{0}},
{{GPIO_PORTB,5},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_5,{0}},
{{GPIO_PORTB,6},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_6,{0}},
{{GPIO_PORTB,7},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_7,{0}},
{{GPIO_PORTB,8},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_8,{0}},
{{GPIO_PORTB,9},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_9,{0}},
{{GPIO_PORTB,10},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_10,{0}},
{{GPIO_PORTB,11},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_11,{0}},
{{GPIO_PORTB,12},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_12,{0}},
{{GPIO_PORTB,13},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_13,{0}},
{{GPIO_PORTB,14},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_14,{0}},
{{GPIO_PORTB,15},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_15,{0}},
{{GPIO_PORTB,16},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_16,{0}},
{{GPIO_PORTB,17},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_17,{0}},
{{GPIO_PORTB,18},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_18,{0}},
{{GPIO_PORTB,19},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_19,{0}},
{{GPIO_PORTB,20},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_20,{0}},
{{GPIO_PORTB,21},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_21,{0}},
{{GPIO_PORTB,22},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_22,{0}},
{{GPIO_PORTB,23},&VOR_GPIO->BANK[GPIO_PORTB],&Driver_GPIOB_23,{0}}
};


/**
  function:          VOR_DRIVER_VERSION GPIOx_GetVersion (void)
  brief:       Get driver version.
  return       VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_GPIOx_GetVersion (void) {
  return gpio_driver_version;
}


/**
  function:    int32_t VOR_GPIO_Initialize (VOR_GPIO_SignalEvent_t  cb_event
                                         GPIO_RESOURCES         *GPIO)
  brief:       Initialize GPIO Interface.
  Input        cb_event  Pointer to  VOR_GPIO_SignalEvent
  Input        GPIO     Pointer to GPIO resources
  return       execution_status
*/
int32_t VOR_GPIO_Initialize (VOR_GPIO_SignalEvent_t  cb_event,
                                 GPIO_RESOURCES         *GPIO) {
  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;
	
  if(GPIO->info.cb_event != NULL)
	return VOR_GPIO_ERROR_MULTIPLEINIT; 
	
  GPIO->info.cb_event = cb_event;
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_Uninitialize (GPIO_RESOURCES *GPIO)
  brief:       De-initialize GPIO Interface.
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
int32_t VOR_GPIO_Uninitialize (GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;
	
  GPIO->info.cb_event = NULL;
  return VOR_DRIVER_OK;
}


/**
  function:    uint32_t VOR_GPIO_Read (GPIO_RESOURCES *GPIO)
  brief:       Read data from port pin
  Input:       GPIO     Pointer to GPIO resources
  return       pin value (0 or 1) or    execution_status

*/
int32_t VOR_GPIO_Read (GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  return ((GPIO->reg->DATAIN>>GPIO->gpioId.pin) & 0x1 );
}

/**
  function:    uint32_t VOR_GPIO_ReadRaw (GPIO_RESOURCES *GPIO)
  brief:       Read Raw data from port pin
  Input:       GPIO     Pointer to GPIO resources
  return       pin value (0 or 1) or    execution_status
*/
int32_t VOR_GPIO_ReadRaw (GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  return ((GPIO->reg->DATAINRAW>>GPIO->gpioId.pin) & 0x1 );
}


/**
  function:    int32_t VOR_GPIO_Write (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Write port pin
  Input:       val        Port pin value (0 or 1)
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
int32_t VOR_GPIO_Write (uint32_t val, GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->DATAOUT |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->DATAOUT &= ~(1UL << GPIO->gpioId.pin));
		
  return VOR_DRIVER_OK;
}

/**
  function:    void VOR_GPIO_WriteRaw (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Write data to port pin without mask
  Input:       val        Port pin value (0 or 1)
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
int32_t VOR_GPIO_WriteRaw (uint32_t val, GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->DATAOUTRAW |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->DATAOUTRAW &= ~(1UL << GPIO->gpioId.pin));
		
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_SetDir (uint32_t dir,
                                GPIO_RESOURCES *GPIO);
  brief:       Configure GPIO pin direction
  Input:       dir        VOR_GPIO_DIR_INPUT, VOR_GPIO_DIR_OUTPUT
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_SetDir (uint32_t dir,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  dir ? (GPIO->reg->DIR |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->DIR &= ~(1UL << GPIO->gpioId.pin));
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_SetOut (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO pin pull up
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_SetOut (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  if(val) {
	GPIO->reg->CLROUT &= ~(1UL << GPIO->gpioId.pin);
	GPIO->reg->TOGOUT &= ~(1UL << GPIO->gpioId.pin);
	GPIO->reg->SETOUT |=  (1UL << GPIO->gpioId.pin);
  } else {
	GPIO->reg->SETOUT &= ~(1UL << GPIO->gpioId.pin);
  }

  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_GPIO_ClrOut (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO pin pull down
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_ClrOut (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  if(val) {
	GPIO->reg->TOGOUT &= ~(1UL << GPIO->gpioId.pin);
	GPIO->reg->SETOUT &= ~(1UL << GPIO->gpioId.pin);
	GPIO->reg->CLROUT |=  (1UL << GPIO->gpioId.pin); 
  } else {
	GPIO->reg->CLROUT &= ~(1UL << GPIO->gpioId.pin);
  }
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_TogOut (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO pin Toggle output
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_TogOut (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  if(val) {
	GPIO->reg->CLROUT &= ~(1UL << GPIO->gpioId.pin); 
	GPIO->reg->SETOUT &= ~(1UL << GPIO->gpioId.pin);
	GPIO->reg->TOGOUT |=  (1UL << GPIO->gpioId.pin);
  } else {
	GPIO->reg->TOGOUT &= ~(1UL << GPIO->gpioId.pin);
  }
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_Mask (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO pin Mask
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_Mask (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->DATAMASK |= (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->DATAMASK &=~(1UL << GPIO->gpioId.pin));
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_Pulse (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO Pulse Mode
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_Pulse (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->PULSE |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->PULSE &= ~(1UL << GPIO->gpioId.pin));
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_PulseBase (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Set Base value for GPIO Pulse Mode
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_PulseBase (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->PULSEBASE |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->PULSEBASE &= ~(1UL << GPIO->gpioId.pin));

  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_Delay (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Set delay cycles for GPIO output
  Input:       val      VOR_GPIO_CONTROL_DELAY_0, VOR_GPIO_CONTROL_DELAY_1
                        VOR_GPIO_CONTROL_DELAY_2, VOR_GPIO_CONTROL_DELAY_3
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_Delay (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  switch(val) {
	case VOR_GPIO_CONTROL_DELAY_0:
		GPIO->reg->DELAY1  &= ~(1UL << GPIO->gpioId.pin);
		GPIO->reg->DELAY2  &= ~(1UL << GPIO->gpioId.pin);
		break;
	case VOR_GPIO_CONTROL_DELAY_1:
		GPIO->reg->DELAY1  |=  (1UL << GPIO->gpioId.pin);
		GPIO->reg->DELAY2  &= ~(1UL << GPIO->gpioId.pin);
		break;
	case VOR_GPIO_CONTROL_DELAY_2:
		GPIO->reg->DELAY1  &= ~(1UL << GPIO->gpioId.pin);
		GPIO->reg->DELAY2  |=  (1UL << GPIO->gpioId.pin);
		break;
	case VOR_GPIO_CONTROL_DELAY_3:
		GPIO->reg->DELAY1  |=  (1UL << GPIO->gpioId.pin);
		GPIO->reg->DELAY2  |=  (1UL << GPIO->gpioId.pin);
		break;
	default:
		return VOR_GPIO_ERROR_CTRL_ARG;
  };

  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_IrqTrg (uint32_t val,
                               GPIO_RESOURCES *GPIO);
  brief:       Set IRQ trigger for GPIO Interrupt
  Input:       val        VOR_GPIO_IRQ_TRG_LVL_H,  VOR_GPIO_IRQ_TRG_LVL_L
						   VOR_GPIO_IRQ_TRG_EDGE_LH, VOR_GPIO_IRQ_TRG_EDGE_HL
						   VOR_GPIO_IRQ_TRG_EDGE_ANY
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_IrqTrg (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

	
  switch(val) {
	case VOR_GPIO_IRQ_TRG_LVL_L:
		GPIO->reg->IRQ_SEN  |=  (1UL << GPIO->gpioId.pin);  // Level
		GPIO->reg->IRQ_EDGE &= ~(1UL << GPIO->gpioId.pin);  // Not Any edge
		GPIO->reg->IRQ_EVT  &= ~(1UL << GPIO->gpioId.pin);  // Low Level
		break;
	case VOR_GPIO_IRQ_TRG_LVL_H:
		GPIO->reg->IRQ_SEN  |=  (1UL << GPIO->gpioId.pin);  // Level
		GPIO->reg->IRQ_EDGE &= ~(1UL << GPIO->gpioId.pin);  // Not Any edge
		GPIO->reg->IRQ_EVT  |=  (1UL << GPIO->gpioId.pin);  // High Level
		break;
	case VOR_GPIO_IRQ_TRG_EDGE_LH:
		GPIO->reg->IRQ_SEN  &= ~(1UL << GPIO->gpioId.pin);  // Edge 
		GPIO->reg->IRQ_EDGE &= ~(1UL << GPIO->gpioId.pin);  // Not Any edge
		GPIO->reg->IRQ_EVT  |=  (1UL << GPIO->gpioId.pin);  // Low High Edge
		break;
	case VOR_GPIO_IRQ_TRG_EDGE_HL:
		GPIO->reg->IRQ_SEN  &= ~(1UL << GPIO->gpioId.pin);  // Edge 
		GPIO->reg->IRQ_EDGE &= ~(1UL << GPIO->gpioId.pin);  // Not Any edge
		GPIO->reg->IRQ_EVT  &= ~(1UL << GPIO->gpioId.pin);  // High Low Level
		break;
	case VOR_GPIO_IRQ_TRG_EDGE_ANY:
		GPIO->reg->IRQ_SEN  &= ~(1UL << GPIO->gpioId.pin);  // Edge 
		GPIO->reg->IRQ_EDGE |=  (1UL << GPIO->gpioId.pin);  // Any edge
		break;
	default:
		return VOR_GPIO_ERROR_CTRL_ARG;
  };

	GPIO->info.trg_mode = val;
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_GPIO_IrqEnable (uint32_t val,
                                GPIO_RESOURCES *GPIO);
  brief:       Enable/Disable GPIO Interrupt
  Input:       val        VOR_GPIO_CTRL_ENABLE, VOR_GPIO_CTRL_DISABLE
  Input:       GPIO     Pointer to GPIO resources
  return       execution_status
*/
static int32_t VOR_GPIO_IrqEnable (uint32_t val,GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  val ? (GPIO->reg->IRQ_ENB |=  (1UL << GPIO->gpioId.pin)) : \
        (GPIO->reg->IRQ_ENB &= ~(1UL << GPIO->gpioId.pin));

  // Set Enable status
  GPIO->info.status.irqenb = ((GPIO->reg->IRQ_ENB)>>GPIO->gpioId.pin)& 0x1;
  
  if(val) {
	if(GPIO->gpioId.port == GPIO_PORTA)
	  VOR_Enable_Irq(VOR_IRQ_GPIOA,GPIO->gpioId.pin);
    else if(GPIO->gpioId.port == GPIO_PORTB)
	  VOR_Enable_Irq(VOR_IRQ_GPIOB,GPIO->gpioId.pin);
  } else {
	if(GPIO->gpioId.port == GPIO_PORTA)
	  VOR_Disable_Irq(VOR_IRQ_GPIOA,GPIO->gpioId.pin);
    else if(GPIO->gpioId.port == GPIO_PORTB)
	  VOR_Disable_Irq(VOR_IRQ_GPIOB,GPIO->gpioId.pin);
  }
	
  return VOR_DRIVER_OK;
}



/**
  function:    int32_t VOR_GPIO_Control (uint32_t control, uint32_t arg)
  brief:       Control GPIO Interface.
  Input:       control  Operation
  Input:       arg      Argument of operation (optional)
  Input:       GPIO     Pointer to GPIO resources
  return       common  execution_status and driver specific  GPIO_execution_status
*/
int32_t VOR_GPIO_Control (uint32_t control, uint32_t arg, GPIO_RESOURCES *GPIO) {

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  switch(control) {
	case VOR_GPIO_CONTROL_DIR        : 	return VOR_GPIO_SetDir(arg,GPIO);
	case VOR_GPIO_CONTROL_SETOUT     : 	return VOR_GPIO_SetOut(arg,GPIO);
	case VOR_GPIO_CONTROL_CLROUT     : 	return VOR_GPIO_ClrOut(arg,GPIO);
	case VOR_GPIO_CONTROL_TOGOUT     : 	return VOR_GPIO_TogOut(arg,GPIO);
	case VOR_GPIO_CONTROL_MASK       : 	return VOR_GPIO_Mask(arg,GPIO);
	case VOR_GPIO_CONTROL_PULSE      : 	return VOR_GPIO_Pulse(arg,GPIO);
	case VOR_GPIO_CONTROL_PULSEBASE  : 	return VOR_GPIO_PulseBase(arg,GPIO);
	case VOR_GPIO_CONTROL_DELAY      : 	return VOR_GPIO_Delay(arg,GPIO);
	case VOR_GPIO_CONTROL_IRQ_TRG    : 	return VOR_GPIO_IrqTrg(arg,GPIO);
	case VOR_GPIO_CONTROL_IRQ_ENB    : 	return VOR_GPIO_IrqEnable(arg,GPIO);
	default                          :  return VOR_GPIO_ERROR_CTRL;
  };
}


/**
  function:   VOR_GPIO_STATUS VOR_GPIO_GetStatus (GPIO_RESOURCES *GPIO)
  brief:      Get GPIO status.
  Input:      GPIO     Pointer to GPIO resources
  return      GPIO status  VOR_GPIO_STATUS
*/
VOR_GPIO_STATUS VOR_GPIO_GetStatus (GPIO_RESOURCES *GPIO) {
  uint32_t status = 0;

  // Get Direction Status
  GPIO->info.status.direction = (((GPIO->reg->DIR) >> GPIO->gpioId.pin) & 0x1);
  // Get Setout Status
  GPIO->info.status.pullup    = (((GPIO->reg->SETOUT) >> GPIO->gpioId.pin) & 0x1);
  // Get Clrout Status
  GPIO->info.status.pulldown  = (((GPIO->reg->CLROUT) >> GPIO->gpioId.pin) & 0x1);
  // Get Toggle out Status
  GPIO->info.status.toggle    = (((GPIO->reg->TOGOUT) >> GPIO->gpioId.pin) & 0x1);
  // Get MASK Status
  GPIO->info.status.mask      = (((GPIO->reg->DATAMASK) >> GPIO->gpioId.pin) & 0x1);
  // Get Pulse mode Status
  GPIO->info.status.pulse     = (((GPIO->reg->PULSE) >> GPIO->gpioId.pin) & 0x1);
  // Get Pulse mode base
  GPIO->info.status.pulsebase = (((GPIO->reg->PULSEBASE) >> GPIO->gpioId.pin) & 0x1);
  // Get Delay Status
  status  = ((((GPIO->reg->DELAY1) >> GPIO->gpioId.pin) & 0x1));
  status |= ((((GPIO->reg->DELAY2) >> GPIO->gpioId.pin) & 0x1)   << 1);
  GPIO->info.status.delay     = status;
  // Get TrgEdge/Level Mode
  GPIO->info.status.trgmode   = (((GPIO->reg->IRQ_SEN) >> GPIO->gpioId.pin) & 0x1);
  // Get TrgEdge/Level
  GPIO->info.status.trgsig    = (((GPIO->reg->IRQ_EVT) >> GPIO->gpioId.pin) & 0x1);
  // Get Any Edge Trg Mode
  GPIO->info.status.trgany    = (((GPIO->reg->IRQ_EDGE) >> GPIO->gpioId.pin) & 0x1);
  // Get Interrupt Enable status
  GPIO->info.status.irqenb    = (((GPIO->reg->IRQ_ENB) >> GPIO->gpioId.pin) & 0x1);
  // Get Raw interrupt status
  GPIO->info.status.irqraw    = (((GPIO->reg->IRQ_RAW) >> GPIO->gpioId.pin) & 0x1);
  // Get Enabled interrupt status
  GPIO->info.status.irqend    = (((GPIO->reg->IRQ_END) >> GPIO->gpioId.pin) & 0x1);
 
  return GPIO->info.status;
}

/**
  function:   void VOR_GPIO_SignalEvent (uint32_t event,GPIO_RESOURCES *GPIO)
  brief:      Signal GPIO Events.
  Input:      event   GPIO_events notification mask
  return      none
*/
void VOR_GPIO_SignalEvent (uint32_t event,GPIO_RESOURCES *GPIO){

  if((GPIO == NULL) || (GPIO->reg ==NULL))
	return;

  if(GPIO->info.status.irqenb) {
    // Get Enabled interrupt status
    GPIO->info.status.irqend    = (((GPIO->reg->IRQ_END) >> GPIO->gpioId.pin) & 0x1);

		if( (GPIO->info.trg_mode == VOR_GPIO_IRQ_TRG_LVL_L) || ( GPIO->info.trg_mode == VOR_GPIO_IRQ_TRG_LVL_H)) {
		  GPIO->reg->IRQ_ENB &= ~GPIO->reg->IRQ_END;
		}
    if((GPIO->info.trg_mode == VOR_GPIO_IRQ_TRG_EDGE_LH) ||
  		( GPIO->info.trg_mode == VOR_GPIO_IRQ_TRG_EDGE_HL) ||
	    ( GPIO->info.trg_mode == VOR_GPIO_IRQ_TRG_EDGE_ANY)) {
				GPIO->info.status.edgeStatus = ((GPIO->reg->EDGE_STATUS >> GPIO->gpioId.pin) & 0x1);
				if(GPIO->info.status.edgeStatus)
					GPIO->info.cb_event(event);	
		  }
	  else {	
  	  if(GPIO->info.status.irqend && GPIO->info.cb_event)
  	    GPIO->info.cb_event(event);						 
    }
  }
}

/**
  function:   void VOR_GPIO_IRQHandler (void)
  brief:      GPIO Interrupt Handler.
  return      none
*/
void VOR_GPIO_IRQHandler (void) {
  int32_t i;
  for(i=0;i<MAX_GPIO_CNT;i++) {
	VOR_GPIO_SignalEvent(0,&GPIO_Resources[i]);
  }
}
#endif 
