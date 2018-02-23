/***************************************************************************************
 * @file     i2c_va108xx.c
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "i2c_va108xx.h"
#include "irq_va108xx.h"

/* I2C core clock (system_PA03CSA.c) */
extern uint32_t VOR_Get_ClockFreq (void);
extern uint32_t VOR_Get_SysTime(void);

 /* I2C0 Resources */
I2C_RESOURCES I2C0_Resources = {
  0,&VOR_I2C->BANK[0], &Driver_I2C0, {0},I2C0_CLK_EN_MASK
};

 /* I2C0 Resources */
I2C_RESOURCES I2C1_Resources = {
  1,&VOR_I2C->BANK[1], &Driver_I2C1, {0},I2C1_CLK_EN_MASK
};

/**
  function:    VOR_DRIVER_VERSION I2Cx_GetVersion (void)
  brief:       Get driver version.
  return:      VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_I2Cx_GetVersion (void) {
  return i2c_driver_version;
}


/**
  function:    int32_t VOR_I2C_Initialize (VOR_I2C_SignalEvent_t cb_event,
                                        I2C_RESOURCES         *i2c)
  brief:       Initialize I2C Interface.
  Input:       cb_event  Pointer to VOR_I2C_SignalEvent
  Input:       i2c   Pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_Initialize (VOR_I2C_SignalEvent_t cb_event, I2C_RESOURCES *i2c) {

  if((i2c == NULL) || (i2c->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;

  if (i2c->info.flags & I2C_FLAG_INIT) {
    /* Driver already initialized */
    return VOR_I2C_ERROR_MULTIPLEINIT;
  }
  i2c->info.flags = I2C_FLAG_INIT;

  /* Register driver callback function */
  i2c->info.cb_event = cb_event;

  /* Clear driver status */
  memset(&i2c->info.status, 0, sizeof(VOR_I2C_STATUS));

	// Switch-on clocks to I2C module 
	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |=  i2c->i2cClkEn; 
	
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Uninitialize (I2C_RESOURCES *i2c)
  brief:       De-initialize I2C Interface.
  Input:       i2c   Pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_Uninitialize (I2C_RESOURCES *i2c) {

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not initialized */
    return VOR_DRIVER_OK;
  }
  i2c->info.flags = 0;
  i2c->info.cb_event = NULL;

  VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &=  ~i2c->i2cClkEn; 
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Drv_Mode (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set I2C driver mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Drv_Mode (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.poll_mode = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Poll_Tout (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set I2C Polling Mode Time out.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Poll_Tout (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.poll_tout = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_busSpeed (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set I2C driver mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/

static int32_t VOR_I2C_busSpeed (uint32_t       arg, I2C_RESOURCES *i2c) {
  uint32_t clkscale= 0;
  switch (arg) {
	case VOR_I2C_BUS_SPEED_STANDARD :
				clkscale = 	((VOR_Get_ClockFreq() / I2C_STD_SPEED_CLK) / I2C_STD_SPEED_MULTI)-1;
				i2c->reg->CLKSCALE = clkscale;
				i2c->info.speed = arg; 
				break;
	case VOR_I2C_BUS_SPEED_FAST     : 
	      clkscale = 	((VOR_Get_ClockFreq() / I2C_FAST_SPEED_CLK) / I2C_FAST_SPEED_MULT)-1;
				i2c->reg->CLKSCALE = (clkscale | I2C_PERIPHERAL_CLKSCALE_FASTMODE_Msk);
	            i2c->info.speed = arg; 
				break;
	default                         : return VOR_DRIVER_ERROR_UNSUPPORTED;
  };
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_I2C_OwnAddr (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set I2C Own Slave address.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_OwnAddr (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.slv_addr = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Addr10Bit (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set I2C 7/10 bit address mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Addr10Bit (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.addr_mode = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_FifoClr (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       RX/TX Fifo Clear.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_FifoClr (uint32_t       arg, I2C_RESOURCES *i2c) {
	
  if(i2c->info.status.mode) {
	i2c->reg->FIFO_CLR     = (arg & 0x3);
	i2c->reg->FIFO_CLR     = 0;
  } else {
	i2c->reg->S0_FIFO_CLR = (arg & 0x3);
	i2c->reg->S0_FIFO_CLR = 0;
  }
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Loopback (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Loopback mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Loopback (uint32_t       arg, I2C_RESOURCES *i2c) {
	
	if(arg)
		i2c->reg->CTRL |= I2C_PERIPHERAL_CTRL_LOOPBACK_Msk;
	else
		i2c->reg->CTRL &= ~(I2C_PERIPHERAL_CTRL_LOOPBACK_Msk);
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Txfemd (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Transmit Empty FIFO Mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Txfemd (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.txfemd = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Rxffmd (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Receive Full FIFO Mode.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Rxffmd (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.rxffmd = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_AlgFilter (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Analog filter.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_AlgFilter (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.algfilter = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_DigFilter (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Digital filter.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_DigFilter (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.digfilter = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_RxFifoTrg (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Rx fifo trigger.
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_RxFifoTrg (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.rxfifotrg = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_TxFifoTrg (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Tx fifo trigger
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_TxFifoTrg (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.txfifotrg = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_ClkToLimit (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Low clock limit
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_ClkToLimit (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.clktolimit = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_TmConfig (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Set Timing configuration
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_TmConfig (uint32_t       arg, I2C_RESOURCES *i2c) {
  i2c->info.tmconfig.val = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_abort_transfer (uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Abort I2C transfer
  Input:       arg      argument of operation 
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_abort_transfer(uint32_t       arg, I2C_RESOURCES *i2c) {
  VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
  VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
	i2c->reg->IRQ_ENB = 0;
	i2c->reg->S0_IRQ_ENB = 0;
	// Clear FIFO
	i2c->reg->FIFO_CLR = 0xffffffff;
	i2c->reg->S0_FIFO_CLR = 0xffffffff;
	i2c->reg->CTRL    &=  ~I2C_PERIPHERAL_CTRL_ENABLE_Msk;
	i2c->reg->S0_CTRL &=  ~I2C_PERIPHERAL_S0_CTRL_ENABLE_Msk;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_Control (uint32_t       control,
                                     uint32_t       arg,
                                     I2C_RESOURCES *i2c)
  brief:       Control I2C Interface.
  Input:       control  operation
  Input:       arg      argument of operation (optional)
  Input:       i2c      pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_Control (uint32_t control, uint32_t arg, I2C_RESOURCES *i2c) {

  if((i2c == NULL) || (i2c->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;
  
  
  switch (control) {
	case VOR_I2C_DRV_POLL            : return VOR_I2C_Drv_Mode(arg,i2c);
	case VOR_I2C_POLL_TOUT           : return VOR_I2C_Poll_Tout(arg,i2c);
	case VOR_I2C_BUS_SPEED           : return VOR_I2C_busSpeed(arg,i2c);
	case VOR_I2C_OWN_ADDRESS         : return VOR_I2C_OwnAddr(arg,i2c);
	case VOR_I2C_ADDRESS_10BIT       : return VOR_I2C_Addr10Bit(arg,i2c);
	case VOR_I2C_FIFO_CLR            : return VOR_I2C_FifoClr(arg,i2c);
	case VOR_I2C_LOOPBACK            : return VOR_I2C_Loopback(arg,i2c);
	case VOR_I2C_ABORT_TRANSFER      : return VOR_I2C_abort_transfer(arg,i2c);
	case VOR_I2C_TXFEMD_MODE         : return VOR_I2C_Txfemd(arg,i2c);
	case VOR_I2C_RXFFMD_MODE         : return VOR_I2C_Rxffmd(arg,i2c);
	case VOR_I2C_ALGFILTER           : return VOR_I2C_AlgFilter(arg,i2c);
	case VOR_I2C_DIGFILTER           : return VOR_I2C_DigFilter(arg,i2c);
	case VOR_I2C_RXFIFOIRQTRG        : return VOR_I2C_RxFifoTrg(arg,i2c);
	case VOR_I2C_TXFIFOIRQTRG        : return VOR_I2C_TxFifoTrg(arg,i2c);
	case VOR_I2C_CLKTOLIMIT          : return VOR_I2C_ClkToLimit(arg,i2c);
	case VOR_I2C_TMCONFIG            : return VOR_I2C_TmConfig(arg,i2c);
	default                          : return VOR_I2C_ERROR_CTRL;
  }
}

/**
  function:    VOR_I2C_STATUS VOR_I2C_GetStatus (I2C_RESOURCES *I2C)
  brief:       Get I2C status.
  Input:       I2C     Pointer to I2C resources
  return:      I2C status VOR_I2C_STATUS
*/
VOR_I2C_STATUS VOR_I2C_GetStatus (I2C_RESOURCES *i2c) {

  if(i2c->info.status.mode == I2C_MODE_MASTER) {
    // Bus Idle Status
    i2c->info.status.i2cIdle   = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_I2CIDLE_Msk) ? 1: 0;
    // Controller Idle Status
    i2c->info.status.Idle      = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_IDLE_Msk)    ? 1: 0;
    // Controller Waiting Status
    i2c->info.status.waiting   = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_WAITING_Msk) ? 1: 0;
    // Controller Stalled Status
    i2c->info.status.stalled   = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_STALLED_Msk) ? 1: 0;
    // Lost arbitration Status
    i2c->info.status.arb_lost  = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_ARBLOST_Msk) ? 1: 0;
    // Address failed Status
    i2c->info.status.nackAddr  = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_NACKADDR_Msk)? 1: 0;
    // Receive negative Ack 
    i2c->info.status.nackData  = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_NACKDATA_Msk)? 1: 0;
    // Receive FIFO is not empty 
    i2c->info.status.rxNEmpty  = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_RXNEMPTY_Msk)? 1: 0;
    // Receive FIFO is full
    i2c->info.status.rxFull    = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_RXFULL_Msk)  ? 1: 0;
    // Receive FIFO >= Trigger level
    i2c->info.status.rxFifoTrg = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_RXTRIGGER_Msk)? 1: 0;
    // Transmit FIFO is empty 
    i2c->info.status.txEmpty   = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_TXEMPTY_Msk) ? 1: 0;
    // Transmit FIFO is not full 
    i2c->info.status.txNFull   = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_TXNFULL_Msk) ? 1: 0;
    // Transmit FIFO >= Trigger level 
    i2c->info.status.txFifoTrg = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_TXTRIGGER_Msk)? 1: 0;
    // Raw I2C Data value 
    i2c->info.status.rawSDA    = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_RAW_SDA_Msk) ? 1: 0;
    // Raw I2C Clock value 
    i2c->info.status.rawSCL    = (i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_RAW_SCL_Msk) ? 1: 0;
  }
 
  return i2c->info.status;
}


/**
  function:    int32_t VOR_I2C_Setup (uint32_t       addr,
                                      const uint8_t *data,
                                      uint32_t       num,
                                      I2C_RESOURCES *i2c)
  brief:       Set up I2C control
  Input:       addr          Slave address (7-bit or 10-bit)
  Input:       num           Number of data bytes to transmit
  Input:       i2c           Pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Setup (uint32_t       addr,
                                    uint32_t       num,
                                    I2C_RESOURCES *i2c) 
{
	uint32_t control ;
	
  // Enable Master for Slave Mode
  if(i2c->info.status.mode == I2C_MODE_MASTER) {
	  control = I2C_PERIPHERAL_CTRL_ENABLE_Msk;
		i2c->reg->S0_CTRL &=  ~I2C_PERIPHERAL_S0_CTRL_ENABLE_Msk;
  }else {
	  control = I2C_PERIPHERAL_S0_CTRL_ENABLE_Msk;
		i2c->reg->CTRL &=  ~I2C_PERIPHERAL_CTRL_ENABLE_Msk;
  }
	
  if(i2c->info.status.mode == I2C_MODE_MASTER) {
    // Configure TXFEMD
	  if(i2c->info.txfemd)	
			control |= ((i2c->info.txfemd & 1)<<I2C_PERIPHERAL_CTRL_TXFEMD_Pos);
	  // Configure RXFFMD
	  if(i2c->info.rxffmd)
  		control |= ((i2c->info.rxffmd & 1)<<I2C_PERIPHERAL_CTRL_RXFFMD_Pos);
	  // Configure ALGFILTER
	  if(i2c->info.algfilter)
		  control |= ((i2c->info.algfilter & 1)<<I2C_PERIPHERAL_CTRL_ALGFILTER_Pos);
	  // Configure DLGFILTER
	  if(i2c->info.digfilter)
  		control |= ((i2c->info.digfilter & 1)<<I2C_PERIPHERAL_CTRL_DLGFILTER_Pos);
	  // Configure Internal Loopback
	  if(i2c->info.loopback)
  		control |= ((i2c->info.loopback & 1)<<I2C_PERIPHERAL_CTRL_LOOPBACK_Pos);
		
		i2c->reg->CTRL = control;
  	// Configure I2C Timming 
	  if(i2c->info.tmconfig.val) {
		  i2c->reg->CTRL |= ((1)<<I2C_PERIPHERAL_CTRL_TMCONFIGENB_Pos);
		  i2c->reg->TMCONFIG = i2c->info.tmconfig.val;
		  i2c->reg->CTRL &= ~((1)<<I2C_PERIPHERAL_CTRL_TMCONFIGENB_Pos);
	  }
	  // Configure Number of bytes
	  i2c->reg->WORDS = num;
	  // Configure Address
	  i2c->reg->ADDRESS = 0;
	  i2c->reg->ADDRESS |= ((addr)<<I2C_ADDR_BITS);
	  // Configure Direction
	  if(i2c->info.status.direction)
		  i2c->reg->ADDRESS |= ((i2c->info.status.direction & 1)<<I2C_ADDR_DIR);
	  // Configure Address Mode
	  if(i2c->info.addr_mode)
  		i2c->reg->ADDRESS |= ((i2c->info.addr_mode & 1)<<I2C_ADDR_MODE);
  	// Configure Low clock limit
  	if(i2c->info.clktolimit)
		  i2c->reg->CLKTOLIMIT = i2c->info.clktolimit;
	  // Configure RX FIFO TRG
	  if(i2c->info.rxfifotrg)
  		i2c->reg->RXFIFOIRQTRG = i2c->info.rxfifotrg;
  	// Configure TX FIFO TRG
  	if(i2c->info.txfifotrg)
		  i2c->reg->TXFIFOIRQTRG = i2c->info.txfifotrg;
    // Clear TX Fifo
	  i2c->reg->FIFO_CLR = 0x3;
	  i2c->reg->FIFO_CLR = 0x0;
  } else {
	  // Configure TXFEMD
	  if(i2c->info.txfemd)	
		  control |= ((i2c->info.txfemd & 1)<<I2C_PERIPHERAL_S0_CTRL_TXFEMD_Pos);
	  // Configure RXFFMD
	  if(i2c->info.rxffmd)
  		control |= ((i2c->info.rxffmd & 1)<<I2C_PERIPHERAL_S0_CTRL_RXFFMD_Pos);
		
		i2c->reg->S0_CTRL = control;
  	// Configure Number of bytes
  	i2c->reg->S0_MAXWORDS = num;
  	// Configure Address
  	i2c->reg->S0_ADDRESS = 0;
  	i2c->reg->S0_ADDRESS |= ((i2c->info.slv_addr)<<I2C_ADDR_BITS);
  	// Configure Direction
	  if(i2c->info.status.direction)
  		i2c->reg->S0_ADDRESS |= ((i2c->info.status.direction & 1)<<I2C_ADDR_DIR);
  	// Configure Address Mode
  	if(i2c->info.addr_mode)
		  i2c->reg->S0_ADDRESS |= ((i2c->info.addr_mode & 1)<<I2C_ADDR_MODE);
	  // Configure RX FIFO TRG
	  if(i2c->info.rxfifotrg)
  		i2c->reg->S0_RXFIFOIRQTRG = i2c->info.rxfifotrg;
  	// Configure TX FIFO TRG
  	if(i2c->info.txfifotrg)
		  i2c->reg->S0_TXFIFOIRQTRG = i2c->info.txfifotrg;
    // Clear TX Fifo
  	i2c->reg->S0_FIFO_CLR = 0x3;
	  i2c->reg->S0_FIFO_CLR = 0x0;
  }	
  return VOR_DRIVER_OK;						
}

/**
  function:    int32_t VOR_I2C_Stop (I2C_RESOURCES *i2c)
  brief:       Stop I2C
  Input:       i2c           Pointer to I2C resources
  return:      execution_status
*/
static int32_t VOR_I2C_Stop (I2C_RESOURCES *i2c) 
{
	i2c->reg->CMD |= I2C_CMD_STOP;
	VOR_Sleep(1);
  i2c->reg->CTRL    = 0;
  i2c->reg->S0_CTRL = 0;
	
	
  if(i2c->info.status.mode == I2C_MODE_MASTER) {
 	// Configure Number of bytes
	i2c->reg->WORDS = 0;
	// Configure Address
	i2c->reg->ADDRESS = 0;
    // Clear TX Fifo
	i2c->reg->FIFO_CLR = 0x3;
	i2c->reg->FIFO_CLR = 0x0;

  } else {
	i2c->reg->S0_MAXWORDS = 0;
	// Configure Address
	i2c->reg->S0_ADDRESS = 0;
    // Clear TX Fifo
	i2c->reg->S0_FIFO_CLR = 0x3;
	i2c->reg->S0_FIFO_CLR = 0x0;
  }	
  return VOR_DRIVER_OK;						
}

/**
  function: int32_t VOR_I2C_MasterTransmit(uint32_t addr,
						 							 							   const uint8_t *data,
                                           uint32_t num,
																					 bool xfer_pending,
																					 I2C_RESOURCES *i2c)
  brief:	Start transmitting data as I2C Master.
  Input:	addr					Slave address (7-bit or 10-bit)
  Input: 	data					Pointer to buffer with data to transmit to I2C Slave
  Input:	num						Number of data bytes to transmit
  Input:	xfer_pending	Transfer operation is pending - Stop condition will not be generated
  Input:	i2c						Pointer to I2C resources
  return:	execution_status
*/
int32_t VOR_I2C_MasterTransmit(uint32_t addr,
                               const uint8_t *data,
                               uint32_t num,
                               bool xfer_pending,
                               I2C_RESOURCES *i2c) {
  int32_t i = 0;
  /*uint32_t poll_time, start_time;*/ 
	uint32_t status;
																	
  if (!data || (num > I2C_MAX_TXDATA)) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }
	
  i2c->reg->IRQ_ENB = 0;
  i2c->info.txidx = 0;

  if(i2c->info.poll_mode) {
		/* Set Master Mode */ 
  	i2c->info.status.mode = I2C_MODE_MASTER;
  	
		/* Set Direction Transmit */ 
  	i2c->info.status.direction = I2C_DIR_TRANSMIT;
  	i2c->reg->CTRL |= I2C_PERIPHERAL_CTRL_ENABLE_Msk;
		
	  /* Enable Start bit */ 
    VOR_I2C_Setup(addr,num,i2c);
	  i2c->reg->CMD |= I2C_CMD_START;

 		/* Get Current Time */ 
	  /* start_time = VOR_Get_SysTime();
	  poll_time  = start_time - i2c->info.poll_tout; */ 

	  while( (num !=0) /*&& (poll_time <= VOR_Get_SysTime())*/ ) {
			__NOP( ); 
  	  __NOP( ); 
	    __NOP( ); 
			
		  status = i2c->reg->STATUS;
		  if((!(status&I2C_PERIPHERAL_STATUS_IDLE_Msk) || (status &I2C_PERIPHERAL_STATUS_WAITING_Msk) || (status & I2C_PERIPHERAL_STATUS_STALLED_Msk)) ) {
  	    /* Check TX fifo status and Load Remaining Data */ 
	      if((status&I2C_PERIPHERAL_STATUS_TXNFULL_Msk)!=0) {
    	    i2c->reg->DATA=data[i++];
		      num--;			
					i2c->info.txidx++;
			    if((num==0) && (xfer_pending == 0))
    			  i2c->reg->CMD |= I2C_CMD_STOP;
	      }
  	  }
	    /* Check for Arbitration Error */ 
  	  if( status & I2C_PERIPHERAL_STATUS_ARBLOST_Msk ) {
	      VOR_I2C_Stop(i2c);
  		  return VOR_I2C_ERROR_ARBITRATION_LOST;
	    }
	    /* Check for Address Nack Error */ 
	    if(status & I2C_PERIPHERAL_STATUS_NACKADDR_Msk) {
	      VOR_I2C_Stop(i2c);
	      return VOR_I2C_ERROR_ADDRESS_NACK;
	    }
	    /* Check for Data Nack Error */ 
	    if(status & I2C_PERIPHERAL_STATUS_NACKDATA_Msk) {
	      VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_TRANSFER_INCOMPLETE;
	    }
	  }
		
	  if(num!=0) {
      VOR_I2C_Stop(i2c);
	    return VOR_I2C_ERROR_POLL_TOUT;
	  }
		
	  /* Wait for transaction to complete */ 
	  while( !(i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_TXEMPTY_Msk) /* && (poll_time <= VOR_Get_SysTime())*/ ) {
  	  __NOP( ); 
	    __NOP( ); 
	    /* Check for Data Nack Error */ 
	    if( i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_NACKDATA_Msk ) {
        VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_TRANSFER_INCOMPLETE;
	    }
	  }
		
	  if( !(i2c->reg->STATUS & I2C_PERIPHERAL_STATUS_TXEMPTY_Msk) ) {
      VOR_I2C_Stop(i2c);
	    return VOR_I2C_ERROR_POLL_TOUT;		  
	  } 
		
    if(!xfer_pending) 
	    VOR_I2C_Stop(i2c);
  
	} else {   /* Interrupt Mode */ 
    VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
	  i2c->reg->IRQ_CLR = 0xffffffff;
	  i2c->reg->IRQ_CLR = 0;
		
	  /* Set Master Mode */ 
	  i2c->info.status.mode = I2C_MODE_MASTER;
		
	  /* Set Direction Transmit */ 
	  i2c->info.status.direction = I2C_DIR_TRANSMIT;
		
	  /* Set Up Control parameters */ 
    VOR_I2C_Setup(addr,num,i2c);
	  memcpy(i2c->info.txdata,data,num);
	  i2c->info.txidx = 0;
	  i2c->info.rxidx = 0;
	  i2c->info.txcnt = num;
	  i2c->info.rxcnt = 0;
	  i2c->reg->CMD = I2C_CMD_START;
		
	  /* Enable Interrupts */ 
	  i2c->reg->IRQ_ENB = (I2C_PERIPHERAL_IRQ_END_IDLE_Msk | \
												 I2C_PERIPHERAL_IRQ_END_ARBLOST_Msk | \
												 I2C_PERIPHERAL_IRQ_END_NACKADDR_Msk | \
												 I2C_PERIPHERAL_IRQ_END_NACKDATA_Msk | \
												 I2C_PERIPHERAL_IRQ_END_CLKLOTO_Msk | \
												 I2C_PERIPHERAL_IRQ_END_TXREADY_Msk | \
												 I2C_PERIPHERAL_IRQ_END_TXEMPTY_Msk );
	  VOR_Enable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
  }
  return VOR_DRIVER_OK;
}
								

/**
  function:    int32_t VOR_I2C_MasterReceive (uint32_t       addr,
                                           uint8_t       *data,
                                           uint32_t       num,
                                           bool           xfer_pending,
                                           I2C_RESOURCES *i2c)
  brief:       Start receiving data as I2C Master.
  Input:       addr          Slave address (7-bit or 10-bit)
  Output:      data          Pointer to buffer for data to receive from I2C Slave
  Input:       num           Number of data bytes to receive
  Input:       xfer_pending  Transfer operation is pending - Stop condition will not be generated
  Input:       i2c           Pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_MasterReceive (uint32_t       addr,
                                   uint8_t       *data,
                                   uint32_t       num,
                                   bool           xfer_pending,
                                   I2C_RESOURCES *i2c) {

  int32_t i;
  // uint32_t poll_time, start_time;
	uint32_t status;
  if (!data || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }

  i2c->info.rxidx = 0;
  i2c->reg->IRQ_ENB = 0;
  if(i2c->info.poll_mode) {
	  // Set Master Mode
	  i2c->info.status.mode = I2C_MODE_MASTER;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_RECEIVE;
	  // Set Up Control parameters
    VOR_I2C_Setup(addr,num,i2c);
	  
    // Enable Start bit
		i2c->reg->CMD = I2C_CMD_START;

	  // Get Current Time
	  /*start_time = VOR_Get_SysTime();
	  poll_time  = start_time - i2c->info.poll_tout;*/ 
	  i =0;
	  // Load Remaining Data to TX FIFO
	  while((i<num) /*&& (poll_time <= VOR_Get_SysTime())*/) {
		  __NOP( ); 
		  __NOP( ); 
	    status = i2c->reg->STATUS;
      // Check TX fifo status and Load Remaining Data
		  if((!(status&I2C_PERIPHERAL_STATUS_IDLE_Msk) || (status &I2C_PERIPHERAL_STATUS_WAITING_Msk)|| (status & I2C_PERIPHERAL_STATUS_STALLED_Msk))  ) {
	      // Check RX fifo status 
	      if(i2c->reg->STATUS&I2C_PERIPHERAL_STATUS_RXNEMPTY_Msk) {
		      data[i++] = i2c->reg->DATA;
					i2c->info.rxidx++;
					if(i==1)
						i2c->reg->CMD = 0;
  			  if((i == num) && (xfer_pending == 0))
    			  i2c->reg->CMD = I2C_CMD_STOP;
				  continue;
	      }
	    }			
		  // Check for Arbitration Error
		  if(status & I2C_PERIPHERAL_STATUS_ARBLOST_Msk) {
  		  VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_ARBITRATION_LOST;
		  }
		  // Check for Address Nack Error
		  if(status & I2C_PERIPHERAL_STATUS_NACKADDR_Msk) {
  		  VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_ADDRESS_NACK;
		  }
		  // Check for Data Nack Error
		  if(status & I2C_PERIPHERAL_STATUS_NACKDATA_Msk) {
  		  VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_TRANSFER_INCOMPLETE;
		  }
	 
	  }
	  if(i<num) {
		  VOR_I2C_Stop(i2c);
		  return VOR_I2C_ERROR_POLL_TOUT;
	  } 
  } else {   // Interrupt Mode
    VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
	  i2c->reg->IRQ_CLR = 0xffffffff;
	  i2c->reg->IRQ_CLR = 0;
	  // Set Master Mode
	  i2c->info.status.mode = I2C_MODE_MASTER;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_RECEIVE;
	
	  // Set Up Control parameters
    VOR_I2C_Setup(addr,num,i2c);
	  i2c->info.rxdata = data;
	  i2c->info.txidx = 0;
	  i2c->info.rxidx = 0;
	  i2c->info.txcnt = 0;
	  i2c->info.rxcnt = num;
    // Enable Start bit
    i2c->reg->CMD = I2C_CMD_START;
	  
    // Enable Interrupts
	  i2c->reg->IRQ_ENB = (I2C_PERIPHERAL_IRQ_END_IDLE_Msk | \
	                    I2C_PERIPHERAL_IRQ_END_ARBLOST_Msk | \
				             I2C_PERIPHERAL_IRQ_END_NACKADDR_Msk | \
		                 I2C_PERIPHERAL_IRQ_END_NACKDATA_Msk | \
					            I2C_PERIPHERAL_IRQ_END_CLKLOTO_Msk | \
					            I2C_PERIPHERAL_IRQ_END_RXREADY_Msk | \
					             I2C_PERIPHERAL_IRQ_END_RXFULL_Msk );
			VOR_Enable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
  }
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_SlaveTransmit (const uint8_t *data,
                                           uint32_t       num,
                                           I2C_RESOURCES *i2c)
  brief:       Start transmitting data as I2C Slave.
  Input:       data  Pointer to buffer with data to transmit to I2C Master
  Input:       num   Number of data bytes to transmit
  Input:       i2c   Pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_SlaveTransmit (const uint8_t *data,
                                   uint32_t       num,
                                   I2C_RESOURCES *i2c) {

  int32_t i;
  uint32_t poll_time, start_time;
	uint32_t status, irq_raw;																			
  if (!data || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }

  i2c->reg->S0_IRQ_ENB = 0;
  i2c->reg->S0_IRQ_CLR = 0xffffffff;
  i2c->info.txidx = 0;
  if(i2c->info.poll_mode) {
	  // Set Slave Mode
	  i2c->info.status.mode = I2C_MODE_SLAVE;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_TRANSMIT;
	  // Set Up Control parameters
    VOR_I2C_Setup(NULL,num,i2c);
	  
	  // Load Slave TX FIFO
	  i =0;
		num++;
	  while((num !=0) && (i2c->reg->S0_STATUS&I2C_PERIPHERAL_S0_STATUS_TXNFULL_Msk)) {
		  i2c->reg->S0_DATA=data[i++];
			i2c->info.txidx++;
		  num--;
	  }
	  // Get Current Time
	  start_time = VOR_Get_SysTime();
	  poll_time  = start_time - i2c->info.poll_tout;
	  // Load Remaining Data to TX FIFO
	  while((num !=0) && (poll_time <= VOR_Get_SysTime())) {
  		__NOP( ); 
	  	__NOP( ); 
			
		  status = i2c->reg->S0_STATUS;
		  if((status & I2C_PERIPHERAL_S0_STATUS_IDLE_Msk) || (status & I2C_PERIPHERAL_S0_STATUS_WAITING_Msk) \
			             || (status & I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Msk)) {
										 
	      // Check TX fifo status and Load Remaining Data
	      if(status&I2C_PERIPHERAL_S0_STATUS_TXNFULL_Msk) {
		      i2c->reg->S0_DATA=data[i++];
					i2c->info.txidx++;
		      num--;			
		    }
      }			

      irq_raw = i2c->reg->S0_IRQ_RAW;
			// Check For Stop Condition
			if(irq_raw & I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Msk) 
				break;
    }
	  if(num!=0) {
		  VOR_I2C_Stop(i2c);
		  return VOR_I2C_ERROR_POLL_TOUT;
	  }
		else {
			// Wait for Transfer to complete
			while(!(i2c->reg->S0_STATUS & I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Msk) && (poll_time <= VOR_Get_SysTime())){
				__NOP( ); 
			  __NOP( );
			  if(i2c->reg->S0_IRQ_RAW & I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Msk) {
				   VOR_I2C_Stop(i2c);
			     return VOR_DRIVER_OK;
				}
			}
			if(!(i2c->reg->S0_STATUS & I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Msk)) {
		    VOR_I2C_Stop(i2c);
		    return VOR_I2C_ERROR_POLL_TOUT;
			}
		}
  } else {   // Interrupt Mode
    VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
	  i2c->reg->S0_IRQ_CLR = 0xffffffff;
	  i2c->reg->S0_IRQ_CLR = 0;
	  // Set Master Mode
	  i2c->info.status.mode = I2C_MODE_SLAVE;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_TRANSMIT;
	  // Set Up Control parameters
    VOR_I2C_Setup(NULL,num,i2c);
	  memcpy(i2c->info.txdata,data,num);
	  i2c->info.txidx = 0;
	  i2c->info.rxidx = 0;
	  i2c->info.txcnt = num;
	  i2c->info.rxcnt = 0;
	  // Load TX FIFO
	  i =0;
	  while((i < i2c->info.txcnt) && (i2c->reg->S0_STATUS&I2C_PERIPHERAL_S0_STATUS_TXNFULL_Msk)) {
  	  i2c->reg->S0_DATA=data[i++];
  	}
	  i2c->info.txidx = i;

	  // Enable Interrupts
	  i2c->reg->S0_IRQ_ENB = (I2C_PERIPHERAL_S0_IRQ_ENB_COMPLETED_Msk | \
	                             I2C_PERIPHERAL_S0_IRQ_ENB_IDLE_Msk | \
				               I2C_PERIPHERAL_S0_IRQ_ENB_ADDRESSMATCH_Msk | \
		                       I2C_PERIPHERAL_S0_IRQ_ENB_NACKDATA_Msk | \
					                I2C_PERIPHERAL_S0_IRQ_ENB_I2C_START_Msk | \
					                 I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Msk | \
					              I2C_PERIPHERAL_S0_IRQ_ENB_TXUNDERFLOW_Msk | \
						                I2C_PERIPHERAL_S0_IRQ_ENB_TXREADY_Msk | \
                 						I2C_PERIPHERAL_S0_IRQ_ENB_TXEMPTY_Msk );
		VOR_Enable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
  }
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_I2C_SlaveReceive (uint8_t       *data,
                                          uint32_t       num,
                                          I2C_RESOURCES *i2c)
  brief:       Start receiving data as I2C Slave.
  Output:      data  Pointer to buffer for data to receive from I2C Master
  Input:       num   Number of data bytes to receive
  Input:       i2c   Pointer to I2C resources
  return:      execution_status
*/
int32_t VOR_I2C_SlaveReceive (uint8_t       *data,
                                  uint32_t       num,
                                  I2C_RESOURCES *i2c) {
	
  int32_t i;
  uint32_t poll_time, start_time;
	uint32_t status, irq_raw;																		
																	
  if (!data || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }

  i2c->reg->S0_IRQ_ENB = 0;
  i2c->reg->S0_IRQ_CLR = 0xffffffff;
	i2c->info.rxidx = 0;
  if(i2c->info.poll_mode) {
	  // Set Master Mode
	  i2c->info.status.mode = I2C_MODE_SLAVE;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_RECEIVE;
	  // Set Up Control parameters
    VOR_I2C_Setup(NULL,num,i2c);
	  // Get Current Time
	  start_time = VOR_Get_SysTime();
	  poll_time  = start_time - i2c->info.poll_tout;
	  i =0;
		// Load Remaining Data to TX FIFO
	  while((i<(num)) && (poll_time <= VOR_Get_SysTime())) {
		  __NOP( ); 
		  __NOP( ); 
			status = i2c->reg->S0_STATUS;
		  if(!(status & I2C_PERIPHERAL_S0_STATUS_IDLE_Msk) || (status & I2C_PERIPHERAL_S0_STATUS_WAITING_Msk) \
			             || (status & I2C_PERIPHERAL_S0_STATUS_RXSTALLED_Msk)) {
										 
	      // Check TX fifo status and Load Remaining Data
	      if(status&I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Msk) {
		      data[i] = i2c->reg->S0_DATA;
		      i++;			
					i2c->info.rxidx++;
		    }		
				irq_raw = i2c->reg->S0_IRQ_RAW;
			  // Check For Stop Condition
			  if(irq_raw & I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Msk) 
				  break;
				
		  }
	  }
	  if(i<num) {
		  VOR_I2C_Stop(i2c);
		  return VOR_I2C_ERROR_POLL_TOUT;
	  }
  } else {   // Interrupt Mode
    VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
	  i2c->reg->S0_IRQ_CLR = 0xffffffff;
	  i2c->reg->S0_IRQ_CLR = 0;
	  // Set Slave Mode
	  i2c->info.status.mode = I2C_MODE_SLAVE;
	  // Set Direction Transmit
	  i2c->info.status.direction = I2C_DIR_RECEIVE;
	
	  // Set Up Control parameters
    VOR_I2C_Setup(NULL,num,i2c);
	  i2c->info.rxdata = data;
	  i2c->info.txidx = 0;
	  i2c->info.rxidx = 0;
	  i2c->info.txcnt = 0;
	  i2c->info.rxcnt = num;
	  
	  // Enable Interrupts
	  i2c->reg->S0_IRQ_ENB = (I2C_PERIPHERAL_S0_IRQ_ENB_COMPLETED_Msk | \
	                             I2C_PERIPHERAL_S0_IRQ_ENB_IDLE_Msk | \
				               I2C_PERIPHERAL_S0_IRQ_ENB_ADDRESSMATCH_Msk | \
		                       I2C_PERIPHERAL_S0_IRQ_ENB_NACKDATA_Msk | \
					                I2C_PERIPHERAL_S0_IRQ_ENB_I2C_START_Msk | \
					                 I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Msk | \
					               I2C_PERIPHERAL_S0_IRQ_ENB_RXOVERFLOW_Msk | \
					 	                I2C_PERIPHERAL_S0_IRQ_ENB_RXREADY_Msk | \
 				 		                 I2C_PERIPHERAL_S0_IRQ_ENB_RXFULL_Msk );
		VOR_Enable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);	  
  }
  return VOR_DRIVER_OK;
}



/**
  function:    void VOR_I2C_MasterHandler (I2C_RESOURCES *i2c)
  brief:       I2C Master state event handler.
  Input:       i2c  Pointer to I2C resources
  return:      I2C event notification flags
*/
void VOR_I2C_MasterHandler (I2C_RESOURCES *i2c) {
  uint32_t event  = 0;
  uint32_t irq_end;
	uint32_t status;
	
  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
	  i2c->reg->IRQ_ENB = 0;
		VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
    return;
  }
  
  status  = i2c->reg->STATUS;
  irq_end = i2c->reg->IRQ_END;
  if(irq_end & (I2C_PERIPHERAL_IRQ_END_TXREADY_Msk | I2C_PERIPHERAL_IRQ_END_TXEMPTY_Msk)) {
    if(status & (I2C_PERIPHERAL_STATUS_TXEMPTY_Msk | I2C_PERIPHERAL_STATUS_TXTRIGGER_Msk )){
	    if(i2c->info.txidx < i2c->info.txcnt) {
		    while(( i2c->info.txidx < i2c->info.txcnt) && (i2c->reg->STATUS&I2C_PERIPHERAL_STATUS_TXNFULL_Msk)) {
	        i2c->reg->DATA=i2c->info.txdata[i2c->info.txidx++];
			    if(i2c->info.txidx == i2c->info.txcnt) {
						event = VOR_I2C_EVENT_TRANSFER_DONE;
            if(i2c->info.cb_event)
	            i2c->info.cb_event(event);
						
					  i2c->reg->CMD = I2C_CMD_STOP;
					  i2c->reg->IRQ_ENB = 0;
						VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
					}
	      }
		    i2c->reg->IRQ_CLR = (irq_end & (I2C_PERIPHERAL_IRQ_END_TXREADY_Msk | I2C_PERIPHERAL_IRQ_END_TXEMPTY_Msk));
		    return;
	    }
	  }
  }
  
  if(irq_end & (I2C_PERIPHERAL_IRQ_END_RXREADY_Msk | I2C_PERIPHERAL_IRQ_END_RXFULL_Msk )) {
    if(status & (I2C_PERIPHERAL_STATUS_RXNEMPTY_Msk | I2C_PERIPHERAL_STATUS_RXFULL_Msk | I2C_PERIPHERAL_STATUS_RXTRIGGER_Msk )){
	    if((i2c->info.rxidx < i2c->info.rxcnt) && i2c->info.rxdata) {
		    while(( i2c->info.rxidx < i2c->info.rxcnt) && (i2c->reg->STATUS&I2C_PERIPHERAL_STATUS_RXNEMPTY_Msk)) {
		      i2c->info.rxdata[i2c->info.rxidx++] = i2c->reg->DATA;
			    if(i2c->info.rxidx == i2c->info.rxcnt) {
					  i2c->reg->CMD = I2C_CMD_STOP;
					  i2c->reg->IRQ_ENB = 0;
						VOR_Disable_Irq(VOR_IRQ_I2C_MS, i2c->i2cId);
						event = VOR_I2C_EVENT_TRANSFER_DONE;
            if(i2c->info.cb_event)
	            i2c->info.cb_event(event);
					}
	      }
		    i2c->reg->IRQ_CLR = (irq_end & (I2C_PERIPHERAL_IRQ_END_RXREADY_Msk | I2C_PERIPHERAL_IRQ_END_RXFULL_Msk));
		    return;
	    }
	  }
  }

  if(status & I2C_PERIPHERAL_STATUS_ARBLOST_Msk) 
	event |= (VOR_I2C_EVENT_ARBITRATION_LOST);
  
  if(status & I2C_PERIPHERAL_STATUS_NACKADDR_Msk)
	event |= (VOR_I2C_EVENT_ADDRESS_NACK);
  
  if(status & I2C_PERIPHERAL_STATUS_NACKDATA_Pos)
	event |= (VOR_I2C_EVENT_TRANSFER_INCOMPLETE);

  if((status & I2C_PERIPHERAL_STATUS_IDLE_Msk) && (i2c->info.txidx >= i2c->info.txcnt))
	event |= VOR_I2C_EVENT_TRANSFER_DONE;

  if((status & I2C_PERIPHERAL_STATUS_WAITING_Msk))
	event |= VOR_I2C_EVENT_TRANSFER_INCOMPLETE;
  
  if((status & I2C_PERIPHERAL_STATUS_STALLED_Msk))
	event |= VOR_I2C_EVENT_TRANSFER_INCOMPLETE;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_CLKLOTO_Msk)
	event |= VOR_I2C_EVENT_CLKLOTO;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_RXOVERFLOW_Msk)
	event |= VOR_I2C_EVENT_RXOVERFLOW;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_TXOVERFLOW_Msk)
	event |= VOR_I2C_EVENT_TXOVERFLOW;
	  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_TXREADY_Msk)
	event |= VOR_I2C_EVENT_TXREADY;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_RXREADY_Msk)
	event |= VOR_I2C_EVENT_RXREADY;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_TXEMPTY_Msk)
	event |= VOR_I2C_EVENT_TXEMPTY;
  
  if(irq_end & I2C_PERIPHERAL_IRQ_END_RXFULL_Msk)
	event |= VOR_I2C_EVENT_RXFULL;
  
  if(i2c->info.cb_event)
	  i2c->info.cb_event(event);
  
  i2c->reg->IRQ_CLR = irq_end;
  return;
}


/**
  function:    void VOR_I2C_SlaveHandler (I2C_RESOURCES *i2c)
  brief:       I2C Slave event handler.
  Input:       i2c  Pointer to I2C resources
  return:      I2C event notification flags
*/
void VOR_I2C_SlaveHandler (I2C_RESOURCES *i2c) {
  uint32_t event  = 0;
  uint32_t irq_end;
	uint32_t status;

  if (!(i2c->info.flags & I2C_FLAG_INIT)) {
    /* Driver not yet configured */
	  i2c->reg->IRQ_ENB = 0;
		VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
    return ;
  }
  status  = i2c->reg->S0_STATUS;
  irq_end = i2c->reg->S0_IRQ_END;
  
  // Check For Stop Condition
	if(irq_end & I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Msk) {
		if(i2c->info.txidx != i2c->info.txcnt) {
		  i2c->reg->IRQ_ENB = 0;
			VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
			event = VOR_I2C_EVENT_TRANSFER_INCOMPLETE;
      if(i2c->info.cb_event)
	       i2c->info.cb_event(event);
		}	  return;
	}

	
  if(irq_end & (I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Msk | I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Msk | I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Msk)) {
    if(status & (I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Msk | I2C_PERIPHERAL_S0_STATUS_TXTRIGGER_Msk | I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Msk |I2C_PERIPHERAL_S0_STATUS_WAITING_Msk | I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Msk |I2C_PERIPHERAL_S0_STATUS_WAITING_Msk  )){
	    if(i2c->info.txidx < i2c->info.txcnt) {
		    while(( i2c->info.txidx < i2c->info.txcnt) && (i2c->reg->S0_STATUS&I2C_PERIPHERAL_S0_STATUS_TXNFULL_Msk)) {
	        i2c->reg->S0_DATA=i2c->info.txdata[i2c->info.txidx++];
					if(i2c->info.txidx == i2c->info.txcnt) {
					  i2c->reg->IRQ_ENB = 0;
						VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
						event = VOR_I2C_EVENT_TRANSFER_DONE;
            if(i2c->info.cb_event)
	            i2c->info.cb_event(event);
					}
	      }
		
		    i2c->reg->S0_IRQ_CLR = (irq_end & (I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Msk | I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Msk | I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Msk ));
		    return;
	    }
	  }
  }

  
  if(irq_end & (I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Msk | I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Msk )) {
    if(status & (I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Msk | I2C_PERIPHERAL_S0_STATUS_RXFULL_Msk | I2C_PERIPHERAL_S0_STATUS_RXTRIGGER_Msk )){
	    if((i2c->info.rxidx < i2c->info.rxcnt) && i2c->info.rxdata) {
		    while(( i2c->info.rxidx < i2c->info.rxcnt) && (i2c->reg->S0_STATUS&I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Msk)) {
		      i2c->info.rxdata[i2c->info.rxidx++] = i2c->reg->S0_DATA;
					if(i2c->info.rxidx == i2c->info.rxcnt) {
					  i2c->reg->IRQ_ENB = 0;
						VOR_Disable_Irq(VOR_IRQ_I2C_SL, i2c->i2cId);
						event = VOR_I2C_EVENT_TRANSFER_DONE;
            if(i2c->info.cb_event)
	            i2c->info.cb_event(event);
					}
	      }
		    i2c->reg->S0_IRQ_CLR = (irq_end & (I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Msk | I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Msk));
		    return;
	    }
	  }
  }

  if(status & I2C_PERIPHERAL_S0_STATUS_NACKDATA_Msk)
	event |= (VOR_I2C_EVENT_TRANSFER_INCOMPLETE);

  if((status & I2C_PERIPHERAL_S0_STATUS_COMPLETED_Msk) && (i2c->info.txidx >= i2c->info.txcnt))
	event |= VOR_I2C_EVENT_TRANSFER_DONE;

  if((status & I2C_PERIPHERAL_S0_STATUS_WAITING_Msk))
	event |= VOR_I2C_EVENT_TRANSFER_INCOMPLETE;
  
  if((status & (I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Msk|I2C_PERIPHERAL_S0_STATUS_RXSTALLED_Msk)))
	event |= VOR_I2C_EVENT_TRANSFER_INCOMPLETE;
  
  if(irq_end & I2C_PERIPHERAL_S0_IRQ_END_RXOVERFLOW_Msk)
	event |= VOR_I2C_EVENT_RXOVERFLOW;
  
  if(irq_end & (I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Msk |I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Msk))
	event |= VOR_I2C_EVENT_TXREADY;
  
  if(irq_end & I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Msk)
	event |= VOR_I2C_EVENT_RXREADY;
  
  if(irq_end & I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Msk)
	event |= VOR_I2C_EVENT_TXEMPTY;
  
  if(irq_end & I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Msk)
	event |= VOR_I2C_EVENT_RXFULL;
  
  if(i2c->info.cb_event)
	i2c->info.cb_event(event);
  
  i2c->reg->S0_IRQ_CLR = irq_end;
  i2c->reg->S0_IRQ_CLR = 0;
  
  return;
}
	


/**
  function:    int32_t VOR_I2C_GetDataCount (I2C_RESOURCES *i2c)
  brief:       Get transferred data count.
  return:      number of data bytes transferred; 
*/
int32_t VOR_I2C_GetDataCount (I2C_RESOURCES *i2c) {
  if(i2c->info.status.direction == I2C_DIR_RECEIVE)
	return i2c->info.rxidx;
  else
	return i2c->info.txidx;
}



