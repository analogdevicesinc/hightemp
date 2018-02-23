/***************************************************************************************
 * @file     uart_va108xx.c
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
#include "uart_va108xx.h"
#include "driver_common.h"
#include "driver_uart.h"
#include "irq_va108xx.h"
#include "string.h"

#ifndef RTT_LOG

// Driver Version
static const VOR_DRIVER_VERSION uart_driver_version = { VOR_UART_API_VERSION, VOR_UART_DRV_VERSION };

// UART0
// UART0 Resources
UART_RESOURCES UART0_Resources = {
  0,
	&VOR_UART->BANK[0],
  &Driver_UART0,
	{0},
	UART0_CLK_EN_MASK
};

// UART1
// UART1 Resources
UART_RESOURCES UART1_Resources = {
	1,
	&VOR_UART->BANK[1],
  &Driver_UART1,
	{0},
	UART1_CLK_EN_MASK
};

extern uint32_t VOR_Get_ClockFreq (void);

// Local Function
/**
  function:    int32_t UART_SetBaudrate (uint32_t         baudrate,
                                          UART_RESOURCES *UART)
  brief:       Set baudrate dividers
  Input:       baudrate  UART baudrate
  Input:       UART     Pointer to UART resources)
  return:s
   -   0: function succeeded
   -  -1: function failed
*/
uint32_t UART_SetBaudrate (uint32_t  baudrate, UART_RESOURCES *UART)  
{
  uint32_t baudmode = 16, x, Register;
	x = (VOR_Get_ClockFreq()/(baudrate * baudmode))*128;
  Register = (x+1)/2;
	UART->reg->CLKSCALE = Register;
  UART->info.baudrate = baudrate;
  return 0;
}

/**
  function:    VOR_DRIVER_VERSION UARTx_GetVersion (void)
  brief:       Get driver version.
  return:      VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION UARTx_GetVersion (void) {
  return uart_driver_version;
}

/**
  function:    int32_t UART_Initialize (VOR_UART_SignalEvent_t  cb_event
                                         UART_RESOURCES         *UART)
  brief:       Initialize UART Interface.
  Input:       cb_event  Pointer to VOR_UART_SignalEvent
  Input:       UART     Pointer to UART resources
  return:      execution_status
*/
int32_t UART_Initialize (VOR_UART_SignalEvent_t  cb_event, UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	return VOR_DRIVER_ERROR_PARAMETER;
	
		UART->info.flags |= UART_FLAG_INIT; 
	
  if(UART->info.cb_event != NULL)
	return VOR_UART_ERROR_MULTIPLEINIT; 

	// Switch-on clocks to UART module 
	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |=  UART->uartClkEn; 
	
	UART->info.cb_event = cb_event;  
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t UART_Uninitialize (UART_RESOURCES *UART)
  brief:       De-initialize UART Interface.
  Input:       UART     Pointer to UART resources
  return:      execution_status
*/
int32_t UART_Uninitialize (UART_RESOURCES *UART) {

  if((UART == NULL) || (UART->reg ==NULL))
   	return VOR_DRIVER_ERROR_PARAMETER;
	
	UART->info.flags &= ~UART_FLAG_INIT;  

  UART->info.cb_event = NULL;
	// Switch-off clocks to UART module 
	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &=  ~UART->uartClkEn; 
  
	return VOR_DRIVER_OK;
}

/**
  function:    int32_t UART_Send (const void            *data,
                                         uint32_t         num,
                                         UART_RESOURCES *UART)
  brief:       Start sending data to UART transmitter.
  Input:       data  Pointer to buffer with data to send to UART transmitter
  Input:       num   Number of data items to send
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t UART_Send (uint8_t *data, uint32_t num, UART_RESOURCES *UART)
{	
		uint32_t count;
		// uint32_t poll_time, start_time;
		uint32_t txsts;
 		if (!data || (num > UART_MAX_TXDATA)) {
			/* Invalid parameters */
			return VOR_DRIVER_ERROR_PARAMETER;
		}

		if (!(UART->info.flags & UART_FLAG_INIT)) {
			/* Driver not yet configured */
			return VOR_DRIVER_ERROR;
		}

		VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);
		UART->reg->IRQ_ENB = 0;
    UART->info.txidx = 0;
	  UART->reg->ENABLE |= UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
	  UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		if(UART->info.poll_mode){
			
	  	/* start_time = VOR_Get_SysTime();
		  poll_time  = start_time - UART->info.poll_tout;*/ 
			count = 0;
		  while((num !=0) /*&& (poll_time <= VOR_Get_SysTime()) */ ) {
			  __NOP( ); 
			  __NOP( ); 
			  txsts = UART->reg->TXSTATUS;
			  if((txsts & UART_PERIPHERAL_TXSTATUS_WRRDY_Msk) && (~(txsts & UART_PERIPHERAL_TXSTATUS_TXBUSY_Msk))){
			       __NOP( ); 
			       __NOP( ); 
  			       UART->reg->DATA = data[count++];
					   UART->info.txidx++;
				     num--;
			  }
			  __NOP( ); 
			  __NOP( ); 
		  }
		  if(num!=0) {
		    UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
			  return VOR_UART_ERROR_POLL_TOUT;
		  }
	    // Wait for transaction to complete
	    while((UART->reg->TXSTATUS & UART_PERIPHERAL_TXSTATUS_WRBUSY_Msk) /*&& (poll_time <= VOR_Get_SysTime())*/) {
	      __NOP( ); 
	      __NOP( ); 
			  __NOP( ); 
			  __NOP( ); 
	     }			
			 // Stop Tranmiter
		   UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
		}
		else {   // Interrupt Mode
		  UART->reg->IRQ_CLR = 0xffffffff;
		  UART->reg->IRQ_CLR = 0;

		  memcpy(UART->info.txdata,data,num);
		  UART->info.txidx = 0;
		  UART->info.txcnt = num;
		  UART->info.rxidx = 0;
		  UART->info.rxcnt = 0;
		
		  VOR_Enable_Irq(VOR_IRQ_UART, UART->uart_id);
	    UART->reg->IRQ_ENB = (UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Msk | \
				 							 UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Msk | \
								        UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Msk | \
										      UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Msk );
	}
	return VOR_DRIVER_OK;
}

/**
  function:    int32_t UART_Receive (void            *data,
                                      uint32_t         num,
                                      UART_RESOURCES *UART)
  brief:       Start receiving data from UART receiver.
  Output:      data  Pointer to buffer for data to receive from UART receiver
  Input:       num   Number of data items to receive
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t UART_Receive (uint8_t *data, uint32_t num, UART_RESOURCES *UART) 
{
  int32_t count;
  uint32_t poll_time, start_time;
  if (!data || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }

  if (!(UART->info.flags & UART_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }

  VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);
  UART->reg->IRQ_ENB = 0;

  UART->reg->ENABLE |= UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
  UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
	UART->info.rxidx = 0;
  if(UART->info.poll_mode) {
	  
	  // Get Current Time
	  start_time = VOR_Get_SysTime();
	  poll_time  = start_time - UART->info.poll_tout;
	  count = 0;
	  while((count<num) && (poll_time <= VOR_Get_SysTime())) {
		  __NOP( ); 
		  __NOP( ); 
		  if((UART->reg->RXSTATUS & UART_PERIPHERAL_RXSTATUS_RDAVL_Msk) ) {
		    data[count++] = UART->reg->DATA;
		  } 
			
			if(UART->reg->RXSTATUS & UART_PERIPHERAL_RXSTATUS_RXPAR_Msk) {
        UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		    return VOR_UART_ERROR_PARITY;
			}

			if(UART->reg->RXSTATUS & UART_PERIPHERAL_RXSTATUS_RXFRM_Msk) {
        UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		    return VOR_UART_ERROR_FRAME;
			}
			
	  }
    UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		if(count<num){
		  return VOR_UART_ERROR_POLL_TOUT;
	  }
  } else {   // Interrupt Mode
		UART->reg->IRQ_CLR = 0xffffffff;
		UART->reg->IRQ_CLR = 0;

		UART->info.rxdata = data;
		UART->info.rxidx = 0;
		UART->info.rxcnt = num; 
	  UART->info.txidx = 0;
	  UART->info.txcnt = 0;
		 
		VOR_Enable_Irq(VOR_IRQ_UART, UART->uart_id);
		UART->reg->IRQ_ENB = (UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Msk | \
												UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Msk | \
												UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Msk );
   }		
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_UART_SetParity(uint32_t arg,UART_RESOURCES *UART)
  brief:       Set or clear the parity bit of UART. 
  Input:       arg   enable or disable of parity bit 
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_SetParity(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	 return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_PARITY_ENABLE:   UART->reg->CTRL |=  UART_PERIPHERAL_CTRL_PAREN_Msk;   break;
		case VOR_UART_CONTROL_PARITY_DISABLE:  UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_PAREN_Msk; break;
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_SelectParity(uint32_t arg,UART_RESOURCES *UART)
  brief:       Select odd or even parity of UART. 
  Input:       arg  Type of parity 
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_SelectParity(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	 return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_PARITY_EVEN:     UART->reg->CTRL |=  UART_PERIPHERAL_CTRL_PAREVEN_Msk;   break;
		case VOR_UART_CONTROL_PARITY_ODD:      UART->reg->CTRL   &= ~UART_PERIPHERAL_CTRL_PAREVEN_Msk; break;
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_ManualParity(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable or Disable Manual parity of UART. 
  Input:       arg  Manual parity set or clear
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_ManualParity(uint32_t arg,UART_RESOURCES *UART)
{
  if((UART == NULL) || (UART->reg ==NULL))
	 return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_MAN_PARITY_ENABLE:   UART->reg->CTRL |= UART_PERIPHERAL_CTRL_PARSTK_Msk; break;
		case VOR_UART_CONTROL_MAN_PARITY_DISABLE:  UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_PARSTK_Msk; break;
    default:                  return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_StopBit(uint32_t arg,UART_RESOURCES *UART)
  brief:       Select number of stop bits  
  Input:       arg  number of stop bits
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_StopBit(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{ 
		case VOR_UART_CONTROL_STOP_BIT_2:      UART->reg->CTRL |= UART_PERIPHERAL_CTRL_STOPBITS_Msk;   break;
    case VOR_UART_CONTROL_STOP_BIT_1:      UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_STOPBITS_Msk;	break;	
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_DataBits(uint32_t arg,UART_RESOURCES *UART)
  brief:       Select word size in UART frame
  Input:       arg  word size
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_DataBits(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_WORDSIZE_Msk;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_DATA_BITS_5: UART->reg->CTRL |= (0<<UART_PERIPHERAL_CTRL_WORDSIZE_Pos); break;
		case VOR_UART_CONTROL_DATA_BITS_6: UART->reg->CTRL |= (1<<UART_PERIPHERAL_CTRL_WORDSIZE_Pos); break;
		case VOR_UART_CONTROL_DATA_BITS_7: UART->reg->CTRL |= (2<<UART_PERIPHERAL_CTRL_WORDSIZE_Pos); break;
		case VOR_UART_CONTROL_DATA_BITS_8: UART->reg->CTRL |= (3<<UART_PERIPHERAL_CTRL_WORDSIZE_Pos); break;
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_LoopBack(uint32_t arg,UART_RESOURCES *UART)
  brief:       Loopback mode. When 1, then the Receiver input is
               connected to the Transmitter output and CTSn input is
               connected to RTSn output.
  Input:       arg  set or clear loop back
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_LoopBack(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_LOOP_BACK_ENABLE:		 UART->reg->CTRL |= UART_PERIPHERAL_CTRL_LOOPBACK_Msk; break;
		case VOR_UART_CONTROL_LOOP_BACK_DISABLE:		 UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_LOOPBACK_Msk; break;
    default:              		 return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_AutoCTS(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable auto CTS Mode. When enabled the Transmitter
               is paused if CTSn is high, and a transmit would
               normally start
  Input:       arg  select AutoCTS
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_AutoCTS(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_AUTO_CTS_ENABLE: 		UART->reg->CTRL |= UART_PERIPHERAL_CTRL_AUTOCTS_Msk; break;
		case VOR_UART_CONTROL_AUTO_CTS_DISABLE:	  UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_AUTOCTS_Msk; break;
    default:              	  return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_DEFRTS(uint32_t arg,UART_RESOURCES *UART)
  brief:       This specifies the value for the RTSn signal when AUTORTS is not enabled or when the Receiver is not enabled.
  Input:       arg  select DEFRTS
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_DEFRTS(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_DEFRTS_ENABLE:   UART->reg->CTRL |= UART_PERIPHERAL_CTRL_DEFRTS_Msk; break;
		case VOR_UART_CONTROL_DEFRTS_DISABLE:  UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_DEFRTS_Msk; break;
		default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_AutoRTS(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable or disable AutoRTS mode
  Input:       arg  select AutoRTS
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_AutoRTS(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_AUTORTS_ENABLE:  UART->reg->CTRL |= UART_PERIPHERAL_CTRL_AUTORTS_Msk; break;
		case VOR_UART_CONTROL_AUTORTS_DISABLE: UART->reg->CTRL &= ~UART_PERIPHERAL_CTRL_AUTORTS_Msk; break;
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_TX(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable or disable UART transmission
  Input:       arg  set or clear TX of UART
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_TX(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_TX_ENABLE:       UART->reg->ENABLE |= UART_PERIPHERAL_ENABLE_TXENABLE_Msk; break;
		case VOR_UART_CONTROL_TX_DISABLE:      UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk; break;
    default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_RX(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable or disable UART receiver
  Input:       arg  set or clear RX of UART
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_RX(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	switch(arg)
	{
		case VOR_UART_CONTROL_RX_ENABLE: 			UART->reg->ENABLE |= UART_PERIPHERAL_ENABLE_RXENABLE_Msk; break;
		case VOR_UART_CONTROL_RX_DISABLE:			UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk; break;
	  default:              return -1;
	}
	return 0;
}

/**
  function:    int32_t VOR_UART_Break(uint32_t arg,UART_RESOURCES *UART)
  brief:       The TXBREAK register provides a method of transmitting a break.
  Input:       arg  Specifies the break duration 
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_Break(int32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg >= 0 && arg<=0x7F)
	{
	  switch(arg)
	  {
	  	case VOR_UART_CONTROL_BREAK_CONTINOUS_COUNT:      UART->reg->TXBREAK   |=  (VOR_UART_CONTROL_BREAK_CONTINOUS_COUNT<<UART_BREAK_BIT);  break;
		  case VOR_UART_CONTROL_BREAK_RESET:                UART->reg->TXBREAK   &=  ~(VOR_UART_CONTROL_BREAK_CONTINOUS_COUNT<<UART_BREAK_BIT); break;
		  default :                        UART->reg->TXBREAK   &=  ~(VOR_UART_CONTROL_BREAK_CONTINOUS_COUNT<<UART_BREAK_BIT);                        
	                                	   UART->reg->TXBREAK   |=  (arg<<UART_BREAK_BIT);  break;
	   }
	return 0;
  }
  else 
	  return -1;
}

/**
  function:    int32_t VOR_UART_RXFIFOIRQTRG(uint32_t arg,UART_RESOURCES *UART)
  brief:       The RXFIFOIRQTRG register configures the RX FIFO half full level.
  Input:       arg  level RX of FIFO full
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_RXFIFOIRQTRG(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg<=RXFIFOIRQTRG_MASK)
	{
	 UART->reg->RXFIFOIRQTRG   &=  ~(RXFIFOIRQTRG_MASK<<UART_RXFIFOIRQTRG_BIT);
   UART->reg->RXFIFOIRQTRG   |=  (arg<<UART_RXFIFOIRQTRG_BIT);
	 return 0;
  }
	else
		return -1;		
}

/**
  function:    int32_t VOR_UART_TXFIFOIRQTRG(uint32_t arg,UART_RESOURCES *UART)
  brief:       The TXFIFOIRQTRG register configures the TX FIFO half full level.
  Input:       arg  level of TX FIFO full
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_TXFIFOIRQTRG(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg<=TXFIFOIRQTRG_MASK)
	{
	 UART->reg->TXFIFOIRQTRG   &=  ~(TXFIFOIRQTRG_MASK<<UART_TXFIFOIRQTRG_BIT);
   UART->reg->TXFIFOIRQTRG   |=  (arg<<UART_TXFIFOIRQTRG_BIT);
	 return 0;
  }else
		return -1;	
}

/**
  function:    int32_t VOR_UART_RXFIFORTSTRG(uint32_t arg,UART_RESOURCES *UART)
  brief:       The RXFIFORTSTRG register configures the TX FIFO level for AUTORTS mode.
  Input:       arg  level of RXFIFORTSTRG
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_RXFIFORTSTRG(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg<=TXFIFOIRQTRG_MASK)
	{
	 UART->reg->RXFIFORTSTRG   &=  ~(RXFIFORTSTRG_MASK<<UART_RXFIFORTSTRG_BIT);
   UART->reg->RXFIFORTSTRG   |=  (arg<<UART_RXFIFORTSTRG_BIT);
	 return 0;
  }else
		return -1;
}

/**
  function:    int32_t VOR_UART_ENB9BIT(uint32_t arg,UART_RESOURCES *UART)
  brief:       Enable or Disable 9-bit mode
  Input:       arg set or clear ENB9BIT
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_ENB9BIT(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
   
	switch(arg)
	 {
		 case VOR_UART_CONTROL_ENB9BIT_ENABLE:  UART->reg->ADDR9   |=  (1<<UART_ENB9BIT_BIT);  break;
		 case VOR_UART_CONTROL_ENB9BIT_DISABLE: UART->reg->ADDR9   &=  ~(1<<UART_ENB9BIT_BIT); break;
		 default:              return -1;
	 }
	 return 0;
}

/**
  function:    int32_t VOR_UART_ADDR9MSK(uint32_t arg,UART_RESOURCES *UART)
  brief:       The ADDR9MASK register provides the mask register for checking addresses in 9-bit mode.
  Input:       arg set or clear ADDR9MSK
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_ADDR9MSK(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg<=ADDR9MSK_MASK)
	{
	 UART->reg->ADDR9MASK   &=  ~(ADDR9MSK_MASK<<UART_ADDR9MSK_BIT);
   UART->reg->ADDR9MASK   |=  (arg<<UART_ADDR9MSK_BIT);
	 return 0;
  }else
		return -1;
}

/**
  function:    int32_t VOR_UART_FIFOCLR(uint32_t arg,UART_RESOURCES *UART)
  brief:       The FIFO_CLR register provides write access for clearing status conditions.
  Input:       arg clear FIFO
  Input:       UART Pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_FIFOCLR(uint32_t arg,UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
	 
	switch(arg)
	 {
		 case VOR_UART_CONTROL_UART_RXSTS_BIT:    UART->reg->FIFO_CLR   |=  UART_PERIPHERAL_FIFO_CLR_RXSTS_Msk;  break;
		 case VOR_UART_CONTROL_UART_TXSTS_BIT:    UART->reg->FIFO_CLR   |=  UART_PERIPHERAL_FIFO_CLR_TXSTS_Msk;  break;
	   case VOR_UART_CONTROL_UART_RXFIFO_BIT:   UART->reg->FIFO_CLR   |=  UART_PERIPHERAL_FIFO_CLR_RXFIFO_Msk; break;
		 case VOR_UART_CONTROL_UART_TXFIFO_BIT:   UART->reg->FIFO_CLR   |=  UART_PERIPHERAL_FIFO_CLR_TXFIFO_Msk; break;
		 default:                return -1;
	 }
	 return 0;
}

/**
  function:    int32_t VOR_UART_PollMode (uint32_t       arg, UART_RESOURCES *UART)
  brief:       Set UART driver mode.
  Input:       arg      argument of operation 
  Input:       UART      pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_PollMode (uint32_t       arg, UART_RESOURCES *UART) {
  UART->info.poll_mode = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_UART_PollTimeOut (uint32_t       arg, UART_RESOURCES *UART)
  brief:       Set UART Polling Mode Time out.
  Input:       arg      argument of operation 
  Input:       UART      pointer to UART resources
  return:      execution_status
*/
int32_t VOR_UART_PollTimeOut (uint32_t       arg, UART_RESOURCES *UART) {
  UART->info.poll_tout = arg;
  return VOR_DRIVER_OK;
}


int32_t VOR_UART_abort_send(uint32_t       arg, UART_RESOURCES *UART) {
  UART->reg->IRQ_ENB &=  ~(UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Msk | \
	  	 							 UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Msk | \
							        UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Msk | \
									      UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Msk );		  
	UART->reg->FIFO_CLR |= (UART_PERIPHERAL_FIFO_CLR_TXSTS_Msk | \
	                        UART_PERIPHERAL_FIFO_CLR_TXFIFO_Msk );
	// Stop Tranmiter
	UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
	if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_RXENABLE_Msk))
    VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);
	
  return VOR_DRIVER_OK;
}

int32_t VOR_UART_abort_recv(uint32_t       arg, UART_RESOURCES *UART) {
   UART->reg->IRQ_ENB &= ~(UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Msk | \
			 							UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Msk | \
											UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Msk );
	UART->reg->FIFO_CLR |= (UART_PERIPHERAL_FIFO_CLR_RXFIFO_Msk | \
	                        UART_PERIPHERAL_FIFO_CLR_RXSTS_Msk  );
 	 // Stop Receiver
   UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
   if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_TXENABLE_Msk))
     VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);				 
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t UART_Control (uint32_t          control,
                                      uint32_t          arg,
                                      UART_RESOURCES  *UART)
  brief:       Control UART Interface.
  Input:       control  Operation
  Input:       arg      Argument of operation (optional)
  Input:       UART    Pointer to UART resources
  return:      common execution_status and driver specific UART_execution_status
*/
int32_t UART_Control (uint32_t control, uint32_t arg, UART_RESOURCES *UART)
{
	if((UART == NULL) || (UART->reg ==NULL))
	  return VOR_DRIVER_ERROR_PARAMETER;
  
  switch(control)
   {
		case VOR_UART_CONTROL_PARITY_ENABLED:  return VOR_UART_SetParity(arg,UART);
		case VOR_UART_CONTROL_PARITY_SEL:      return VOR_UART_SelectParity(arg,UART);
		case VOR_UART_CONTROL_PARITY_MAN:      return VOR_UART_ManualParity(arg,UART);
		case VOR_UART_CONTROL_STOP_BIT:        return VOR_UART_StopBit(arg,UART);
		case VOR_UART_CONTROL_DATA_BITS:       return VOR_UART_DataBits(arg,UART);
		case VOR_UART_CONTROL_LOOPBACK:        return VOR_UART_LoopBack(arg,UART);
		case VOR_UART_CONTROL_AUTOCTS:         return VOR_UART_AutoCTS(arg,UART);
		case VOR_UART_CONTROL_DEFRTS:          return VOR_UART_DEFRTS(arg,UART);
		case VOR_UART_CONTROL_AUTORTS:         return VOR_UART_AutoRTS(arg,UART);
		case VOR_UART_CONTROL_TX:              return VOR_UART_TX(arg,UART);
		case VOR_UART_CONTROL_RX:              return VOR_UART_RX(arg,UART);
		case VOR_UART_CONTROL_BREAK:           return VOR_UART_Break(arg,UART);
		case VOR_UART_CONTROL_RXFIFOTRG:       return VOR_UART_RXFIFOIRQTRG(arg,UART);
		case VOR_UART_CONTROL_TXFIFOTRG:       return VOR_UART_TXFIFOIRQTRG(arg,UART);
		case VOR_UART_CONTROL_RXFIFORTSTRG:	   return VOR_UART_RXFIFORTSTRG(arg,UART);
		case VOR_UART_CONTROL_ENB9BIT:	       return VOR_UART_ENB9BIT(arg,UART);
		case VOR_UART_CONTROL_ADDR9MSK:	       return VOR_UART_ADDR9MSK(arg,UART); 
		case VOR_UART_CONTROL_FIFOCLR:	       return VOR_UART_FIFOCLR(arg,UART);
		case VOR_UART_CONTROL_POLL_MODE:       return VOR_UART_PollMode(arg,UART);
		case VOR_UART_CONTROL_POLL_TO:         return VOR_UART_PollTimeOut(arg,UART);
		case VOR_UART_CONTROL_BAUD:            return UART_SetBaudrate(arg,UART);
		case VOR_UART_ABORT_SEND:              return VOR_UART_abort_send(arg,UART);
		case VOR_UART_ABORT_RECV:              return VOR_UART_abort_recv(arg,UART);
		default:
			break;
   }
  return VOR_DRIVER_OK;
}

/**
  function:    VOR_UART_STATUS UART_GetStatus (UART_RESOURCES *UART)
  brief:       Get UART status.
  Input:       UART     Pointer to UART resources
  return:      UART status VOR_UART_STATUS
*/
VOR_UART_STATUS UART_GetStatus (UART_RESOURCES *UART) 
{
	UART->info.status.RX_RDAVL         =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RDAVL_Pos)   		&& 1;
	UART->info.status.RX_RDNFULL       =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RDNFULL_Pos) 		&& 1;
	UART->info.status.RX_BUSY          =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXBUSY_Pos)  		&& 1;
	UART->info.status.RX_TO            =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXTO_Pos)    		&& 1;
	UART->info.status.RX_OVR           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXOVR_Pos)  	 	&& 1;
	UART->info.status.RX_FRM           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXFRM_Pos) 			&& 1;
	UART->info.status.RX_PAR           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXPAR_Pos) 			&& 1;
	UART->info.status.RX_BRK           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXBRK_Pos) 			&& 1;
	UART->info.status.RX_BUSYBRK       =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXBUSYBRK_Pos)  && 1;
	UART->info.status.RX_ADDR9         =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXADDR9_Pos) 	  && 1;
	UART->info.status.RX_RTS           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_RXSTATUS_RXRTSN_Pos) 		&& 1;
	UART->info.status.TX_WRRDY         =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_TXSTATUS_WRRDY_Pos) 		  && 1;
	UART->info.status.TX_WRBUSY        =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_TXSTATUS_WRBUSY_Pos) 		&& 1;
	UART->info.status.TX_BUSY          =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_TXSTATUS_TXBUSY_Pos) 		&& 1;
	UART->info.status.TX_WRLOST        =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_TXSTATUS_WRLOST_Pos) 		&& 1;
	UART->info.status.TX_CTS           =      CHECK_BIT(UART->reg->RXSTATUS, UART_PERIPHERAL_TXSTATUS_TXCTSN_Pos) 		&& 1;
  return UART->info.status; 
}


/**
  function:    void UART_IRQHandler (UART_RESOURCES *UART)
  brief:       UART Interrupt handler.
  Input:       UART     Pointer to UART resources
*/
void UART_IRQHandler (UART_RESOURCES *UART) {
  uint32_t event  = 0;
  uint32_t rx_status, tx_status, irq_end;
  UART->reg->IRQ_CLR = UART->reg->IRQ_END;
	
	rx_status  = UART->reg->RXSTATUS;
  tx_status  = UART->reg->TXSTATUS;
	irq_end    = UART->reg->IRQ_END;

	// Check if Tx Is Completed. Turn off Interrupt
	if((irq_end & UART_PERIPHERAL_IRQ_END_IRQ_TX_EMPTY_Msk) &&  (UART->info.txidx >= UART->info.txcnt) && (UART->reg->ENABLE & UART_PERIPHERAL_ENABLE_TXENABLE_Msk )) {
	    UART->reg->IRQ_ENB &=  ~(UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Msk | \
				 							 UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Msk | \
								        UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Msk | \
										      UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Msk );		  
			// Stop Tranmiter
		  UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
		  if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_RXENABLE_Msk))
  		  VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);

		  if(UART->info.cb_event)
				UART->info.cb_event(VOR_UART_EVENT_TX_DONE);
			UART->reg->IRQ_CLR = irq_end;
			return;
	}
	
	// If Tx on progress
	if((irq_end & (UART_PERIPHERAL_IRQ_END_IRQ_TX_Msk | UART_PERIPHERAL_IRQ_END_IRQ_TX_EMPTY_Msk))&& (UART->reg->ENABLE & UART_PERIPHERAL_ENABLE_TXENABLE_Msk )) {
 	  while(( UART->info.txidx < UART->info.txcnt) && (UART->reg->TXSTATUS & UART_PERIPHERAL_TXSTATUS_WRRDY_Msk)) {
		  UART->reg->DATA = UART->info.txdata[UART->info.txidx++];
		}
	}
			 
	
	 if((irq_end & (UART_PERIPHERAL_IRQ_END_IRQ_RX_Msk )) && (UART->reg->ENABLE & UART_PERIPHERAL_ENABLE_RXENABLE_Msk ) ) {
	   while(( UART->info.rxidx < UART->info.rxcnt) && (UART->reg->RXSTATUS & UART_PERIPHERAL_RXSTATUS_RDAVL_Msk)) {
		   UART->info.rxdata[UART->info.rxidx++] = UART->reg->DATA;
			 if(UART->info.rxidx == UART->info.rxcnt) {
		     UART->reg->IRQ_ENB &= ~(UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Msk | \
					 							UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Msk | \
												UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Msk );
		   	 // Stop Receiver
		     UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		     if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_TXENABLE_Msk))
		       VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);				 
		     if(UART->info.cb_event)
				   UART->info.cb_event(VOR_UART_EVENT_RX_DONE);
			   UART->reg->IRQ_CLR = irq_end;
   			 return;			 
			 }
	   }
   }
	 
	if((UART->info.rxidx < UART->info.rxcnt) && (UART->reg->ENABLE &UART_PERIPHERAL_ENABLE_RXENABLE_Msk ) && (!UART->info.poll_mode)) {
		
    if((rx_status & UART_PERIPHERAL_RXSTATUS_RXOVR_Msk) && (UART->info.rxidx < UART->info.rxcnt))
  	  event |= VOR_UART_EVENT_RX_OVERFLOW;

    if((rx_status & UART_PERIPHERAL_RXSTATUS_RXFRM_Msk) && (UART->info.rxidx < UART->info.rxcnt))
  	  event |= VOR_UART_EVENT_RX_FRAMING_ERROR;
  
    if((rx_status & UART_PERIPHERAL_RXSTATUS_RXPAR_Msk) && (UART->info.rxidx < UART->info.rxcnt))
	    event |= VOR_UART_EVENT_RX_PARITY_ERROR;
  
    if(rx_status & UART_PERIPHERAL_RXSTATUS_RXBRK_Msk)
  	  event |= VOR_UART_EVENT_RX_BREAK;
  
    if(rx_status & UART_PERIPHERAL_RXSTATUS_RXTO_Msk)
  	  event |= VOR_UART_EVENT_RX_TIMEOUT;
		
		if(event) {
      UART->reg->IRQ_ENB &= ~(UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Msk | \
 	  	 							   UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Msk | \
		  										UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Msk );
		  // Stop Receiver
		  UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_RXENABLE_Msk;
		  if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_TXENABLE_Msk))
  		  VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);				 
		  if(UART->info.cb_event)
  		  UART->info.cb_event(event);
		}
	}
	if((UART->info.txidx < UART->info.txcnt) && (UART->reg->ENABLE &UART_PERIPHERAL_ENABLE_TXENABLE_Msk ) && (!UART->info.poll_mode)) {

    if(tx_status & UART_PERIPHERAL_TXSTATUS_WRLOST_Msk)
	    event |= VOR_UART_EVENT_TX_OVERFLOW;
	
	  if(irq_end & UART_PERIPHERAL_IRQ_END_IRQ_TX_CTS_Msk)
  	  event |= VOR_UART_EVENT_TX_CTS;

		if(event) {
      UART->reg->IRQ_ENB &=  ~(UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Msk | \
				 							 UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Msk | \
								        UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Msk | \
										      UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Msk );		  
		  // Stop Tranmiter
	    UART->reg->ENABLE &= ~UART_PERIPHERAL_ENABLE_TXENABLE_Msk;
	    if(!(UART->reg->ENABLE&UART_PERIPHERAL_ENABLE_RXENABLE_Msk))
   		  VOR_Disable_Irq(VOR_IRQ_UART, UART->uart_id);

  	  if(UART->info.cb_event)
	  		UART->info.cb_event(event);
		}
  }		
  
  UART->reg->IRQ_CLR = irq_end;
  
	
  return;
}


/**
  function:    int32_t VOR_UART_GetTxCount (UART_RESOURCES *UART)
  brief:       Get Tx data count.
  return:      number of data bytes transferred; 
*/
int32_t VOR_UART_GetTxCount (UART_RESOURCES *UART) {
	return UART->info.txidx;
}

/**
  function:    int32_t VOR_UART_GetRxCount (UART_RESOURCES *UART)
  brief:       Get Rx data count.
  return:      number of data bytes transferred; 
*/
int32_t VOR_UART_GetRxCount (UART_RESOURCES *UART) {
	return UART->info.rxidx;
}
#endif 
