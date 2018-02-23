/***************************************************************************************
 * @file     spi_va108xx.c
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

#include  "spi_va108xx.h"
#include  "irq_va108xx.h"


extern uint32_t VOR_Get_SysTime(void);
extern uint32_t VOR_Get_ClockFreq (void);

/* SPI0 Resources */
VOR_SPI_RESOURCES SPI0_Resources = {
  0, &VOR_SPI->BANK[0], &Driver_SPI0, {0},SPI0_CLK_EN_MASK
};

 /* SPI1 Resources */
VOR_SPI_RESOURCES SPI1_Resources = {
  1, &VOR_SPI->BANK[1], &Driver_SPI1, {0},SPI1_CLK_EN_MASK
};

/* SPI2 Resources */
VOR_SPI_RESOURCES SPI2_Resources = {
  2,  &VOR_SPI->BANK[2], &Driver_SPI2, {0},0
//  2,  &VOR_SPI->BANK[2], &Driver_SPI2, {0}
};

/**
  function:    VOR_DRIVER_VERSION VOR_SPIx_GetVersion (void)
  brief:       Get driver version.
  return:      VOR_DRIVER_VERSION
*/
VOR_DRIVER_VERSION VOR_SPIx_GetVersion(void)
{
	return(DriverVersion);
}

/*
function:    int32_t VOR_SPI_Initialize (VOR_SPI_SignalEvent_t cb_event,
																				 SPI_RESOURCES *SPI);
  brief:       Initialize SPI port
  Input:       cb_event    pointer to the VOR_SPI_SignalEvent callback function
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/
int32_t VOR_SPI_Initialize(VOR_SPI_SignalEvent_t cb_event, VOR_SPI_RESOURCES *SPI)
{
	if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	
  SPI->info.cb_event = cb_event;
	SPI->info.status.BUSY = 0x0;
	SPI->info.flags  = SPI_FLAG_INIT;
		
	// Switch-on clocks to SPI module 
	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE |=  SPI->spiClkEn; 
	SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;

  return VOR_DRIVER_OK;
}

/*
function:    int32_t VOR_SPI_Uninitialize (SPI_RESOURCES *SPI);
  brief:       Uninitialize SPI port
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/
int32_t VOR_SPI_Uninitialize(VOR_SPI_RESOURCES *SPI)
{
	
	if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;

	SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
	// Switch-off clocks to SPI module 
	VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE &=  ~SPI->spiClkEn; 
	
  return VOR_DRIVER_OK;
}


/**
  function:    int32_t VOR_SPI_Drv_Mode (uint32_t       arg,
                                     VOR_SPI_RESOURCES *SPI)
  brief:       Set SPI driver mode.
  Input:       arg      argument of operation 
  Input:       SPI       pointer to SPI resources
  return:      execution_status
*/
int32_t VOR_SPI_Drv_Mode (uint32_t arg, VOR_SPI_RESOURCES *SPI) {
  SPI->info.poll_mode = arg;
  return VOR_DRIVER_OK;
}

/**
  function:    int32_t VOR_SPI_Poll_Tout (uint32_t       arg,
                                     VOR_SPI_RESOURCES *SPI)
  brief:       Set SPI Polling Mode Time out.
  Input:       arg      argument of operation 
  Input:       SPI       pointer to SPI resources
  return:      execution_status
*/
int32_t VOR_SPI_Poll_Tout (uint32_t  arg, VOR_SPI_RESOURCES *SPI) {
	
  SPI->info.poll_tout = arg;
  
	return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_SET_LOOPBACKMODE (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set SPI as LOOPBACK mode 
  Input:       val     VOR_SPI_SET_LOOPBACKMODE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_LOOPBACKMODE(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
    
	if(val)
		SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_LBM_Msk;
	else
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_LBM_Msk;
  return VOR_DRIVER_OK;
}


/*
  function:    int32_t VOR_SPI_SET_SLAVEMODE (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set SPI as SLAVE mode 
  Input:       val     VOR_SPI_SET_SLAVEMODE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_SLAVEMODE(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	if(val)
		SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_MS_Msk;
	else
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_MS_Msk;
	
	SPI->info.msmode  = val;
	return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_CTRL_SLAVE_OP_DISABLEMODE (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set SPI as SLAVE_OP_DISABLEMODE 
  Input:       val     VOR_SPI_CTRL_SLAVE_OP_DISABLE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_CTRL_SLAVE_OP_DISABLEMODE(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	if(val)
		SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_SOD_Msk;
	else
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_SOD_Msk;
	  return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_CTRL_ENABLE_BLOCKMODE_OPR (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set SPI as Enable Block Mode. 
  Input:       val     VOR_SPI_CTRL_ENABLE_BLOCKMODE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_CTRL_ENABLE_BLOCKMODE_OPR(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
   
	if(val)
		SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_BYTEMODE_Msk;
	else
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_BYTEMODE_Msk;
	  return VOR_DRIVER_OK;
}
	

/*
  function:    int32_t VOR_SPI_SET_CTRL_SIZE_OPR (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set size of words for SPI frame 
  Input:       val     VOR_SPI_SET_CTRL_SIZE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_CTRL_SIZE_OPR(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	SPI->info.sizeofwords = val; 
	SPI->reg->CTRL0 |= ((val-1) & SPI_PERIPHERAL_CTRL0_SIZE_Msk) << SPI_PERIPHERAL_CTRL0_SIZE_Pos;

	return VOR_DRIVER_OK;
}
	
/*
  function:    int32_t VOR_SPI_SLAVE_SELECT_OPR (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set Chip Select for SPI frame 
  Input:       val     VOR_SPI_SET_CTRL_SIZE   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SLAVE_SELECT_OPR(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	SPI->reg->CTRL1 |= (val&0x7)<<SPI_PERIPHERAL_CTRL1_SS_Pos;
	return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_SET_TX_FIFOTRG (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set TX FIFO TRG 
  Input:       val     FIFO Level   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_TX_FIFOTRG(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	SPI->reg->TXFIFOIRQTRG = val;
	return VOR_DRIVER_OK;
}
	
/*
  function:    int32_t VOR_SPI_SET_RX_FIFOTRG (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set RX FIFO TRG 
  Input:       val     FIFO Level   
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_RX_FIFOTRG(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	SPI->reg->RXFIFOIRQTRG = val;
	return VOR_DRIVER_OK;
}


/*
  function:    int32_t VOR_SPI_SET_Clock_Mode (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set Clock Polarity and phase 
  Input:       val     Clock Mode
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_Clock_Mode(uint32_t val, VOR_SPI_RESOURCES *SPI) {

  if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	SPI->reg->CTRL0 &= ~(SPI_PERIPHERAL_CTRL0_SPO_Msk|SPI_PERIPHERAL_CTRL0_SPH_Msk);
	SPI->reg->CTRL0 |= (val & 0x3)<<SPI_PERIPHERAL_CTRL0_SPO_Pos;
	return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_SET_Clock_Rate (uint32_t val,
                                SPI_RESOURCES *SPI);
  brief:       Set SPI Clock rate 
  Input:       val     Clock rate
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_SET_Clock_Rate(uint32_t val, VOR_SPI_RESOURCES *SPI) {

	uint32_t prescale, spidiv;
  
	if((SPI == NULL) || (SPI->reg == NULL) || (val == 0))
		return VOR_DRIVER_ERROR_PARAMETER;
			
	if( val >= VOR_Get_ClockFreq() ) {
		prescale = 1;
		spidiv = 0; 
	} else {
		prescale = 2;
		spidiv = (VOR_Get_ClockFreq()/(prescale*val)) - 1; 
	}
	SPI->reg->CLKPRESCALE = prescale; 
	SPI->reg->CTRL0 &= ~(SPI_PERIPHERAL_CTRL0_SCRDV_Msk);
	SPI->reg->CTRL0 |= (spidiv)<<SPI_PERIPHERAL_CTRL0_SCRDV_Pos;
	
	return VOR_DRIVER_OK;
}


int32_t VOR_SPI_abort_transfer(uint32_t arg, VOR_SPI_RESOURCES *SPI) {
  VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	SPI->reg->IRQ_ENB = 0;
	// Clear FIFO
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk;
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk;
  SPI->info.transfer = 0;
  return VOR_DRIVER_OK;
}

int32_t VOR_SPI_ENABLE_INTERFACE(uint32_t arg, VOR_SPI_RESOURCES *SPI) {
	if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	
	if(arg == VOR_SPI_CTRL_ENABLE)
		SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
	else
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
	return VOR_DRIVER_OK;
}

/*
  function:    int32_t VOR_SPI_Control (uint32_t control, uint32_t arg
                                SPI_RESOURCES *SPI);
  brief:       Set SPI control with Control Code and Args 
  Input:       val      Control code 
  Input:       val      Control args 
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_Control(uint32_t control, uint32_t arg, VOR_SPI_RESOURCES *SPI)
{
	
		if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
		
    switch (control)
    {
      case VOR_SPI_DRV_POLL:
				  VOR_SPI_Drv_Mode(arg,SPI);
				  break;
			case VOR_SPI_POLL_TOUT:
				  VOR_SPI_Poll_Tout(arg,SPI);
				  break;
			case VOR_SPI_CTRL_MS:
				  VOR_SPI_SET_SLAVEMODE(arg,SPI);
				  break;
			case VOR_SPI_CTRL_SOD:
				  VOR_SPI_CTRL_SLAVE_OP_DISABLEMODE(arg,SPI);
				  break;
			case VOR_SPI_CTRL_CLKMODE:
				  VOR_SPI_SET_Clock_Mode(arg,SPI);
				  break;
			case VOR_SPI_CTRL_CLKRATE:
				  VOR_SPI_SET_Clock_Rate(arg,SPI);
				  break;
			case VOR_SPI_CTRL_TX_FIFO_TRG:
				  VOR_SPI_SET_TX_FIFOTRG(arg,SPI);
				  break;
			case VOR_SPI_CTRL_RX_FIFO_TRG:
				  VOR_SPI_SET_RX_FIFOTRG(arg,SPI);
				  break;
			case VOR_SPI_CTRL_LOOPBACK :				//SPI Loopback mode
					VOR_SPI_SET_LOOPBACKMODE(arg,SPI);
					break;
      case VOR_SPI_CTRL_BLOCKMODE:										// SPI Blocked mode 
					VOR_SPI_CTRL_ENABLE_BLOCKMODE_OPR(arg,SPI);
					break;
		  case VOR_SPI_CTRL_CS        :                	// Control Slave Select
			   VOR_SPI_SLAVE_SELECT_OPR(arg,SPI);
          break;
  		case VOR_SPI_CTRL_DATA_WIDTH :								// Set Size of words for data width
         VOR_SPI_SET_CTRL_SIZE_OPR(arg,SPI);
					break;
			case VOR_SPI_ABORT_TRANSFER:
				 VOR_SPI_abort_transfer(arg,SPI);
			case VOR_SPI_CTRL_INTERFACE: 
				 VOR_SPI_ENABLE_INTERFACE(arg, SPI); 	
			default:
         return VOR_SPI_ERROR_CTRL;
    }
		
		return VOR_DRIVER_OK;
}

/*
The function VOR_SPI_GetStatus returns the current SPI interface status.
function:    int32_t VOR_SPI_GetStatus (SPI_RESOURCES *SPI);
  brief:       Get SPI status  
  Input:       SPI     Pointer to SPI resources
  return:      VOR_SPI_STATUS
*/
VOR_SPI_STATUS VOR_SPI_GetStatus(VOR_SPI_RESOURCES *SPI)
{

	//Transmit FIFIO Empty Status
	SPI->info.status.TFE = ((SPI->reg->STATUS) & (SPIC_STATUS_TFE_Msk) ? 1:0);
	
	//Transmit FIFO Not Full Status
	SPI->info.status.TNF = ((SPI->reg->STATUS) & (SPIC_STATUS_TNF_Msk) ? 1:0);
	
	//Receive FIFO Not Empty Status
	SPI->info.status.RNE = ((SPI->reg->STATUS) & (SPIC_STATUS_RNE_Msk) ? 1:0);
	
	//Receive FIFO Full Status
	SPI->info.status.RFF = ((SPI->reg->STATUS) & (SPIC_STATUS_RFF_Msk) ? 1:0);
	
	//Transmit/Receive Busy Status
	SPI->info.status.BUSY = ((SPI->reg->STATUS) & (SPIC_STATUS_BUSY_Msk) ? 1:0);
	
	//Indicates that the next Receive FIFO Data word is the first received byte in BLOCKMODE.
	SPI->info.status.RXDATAFIRST = ((SPI->reg->STATUS) & (SPIC_STATUS_RXDATAFIRST_Msk) ? 1:0);
	
	//Receive FIFO is above or equals the trigger level Status
	SPI->info.status.RXTRIGGER = ((SPI->reg->STATUS) & (SPIC_STATUS_RXTRIGGER_Msk) ? 1:0);
	
	//Transmit FIFO is below the trigger level Status
	SPI->info.status.TXTRIGGER = ((SPI->reg->STATUS) & (SPIC_STATUS_TXTRIGGER_Msk) ? 1:0);
	
	return(SPI->info.status);
} 

/* 
function:    int32_t VOR_SPI_Send (uint8_t *data, uint32_t num,
                                   SPI_RESOURCES *SPI);
  brief:       Send data from other port
  Input:       data    data need to send to other port   
  Input:       arg     number of data bytes to send  
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/

int32_t VOR_SPI_Send(uint8_t *data, uint32_t num, VOR_SPI_RESOURCES *SPI)
{
	// uint32_t poll_time, start_time;
	
	if((SPI == NULL) || (SPI->reg == NULL))
		return VOR_DRIVER_ERROR_PARAMETER;
	
	if ((data == NULL) || (num == 0))         
		return VOR_DRIVER_ERROR_PARAMETER;
	
	SPI->reg->IRQ_ENB = 0;
	  
	// Clear TX FIFO
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk;
	SPI->info.transfer = 0;
  if(SPI->info.poll_mode) {
		// Get Current Time
		/* start_time = VOR_Get_SysTime();
		poll_time  = start_time - SPI->info.poll_tout;*/
    SPI->info.txidx = 0;
		// Enable SPI
    // SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		// num++;
		while( (num != 0) /*&& (poll_time <= VOR_Get_SysTime())*/  ) {
	    if(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_TNF_Msk) { 
			  SPI->reg->DATA = data[SPI->info.txidx++];
				num--; 
			}
		}	 
		
		if(num!=0) {
			// Disable SPI
			SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
			return VOR_SPI_ERROR_POLL_TOUT;
		}
		// Wait for transfer to complete
		// while(!(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_TFE_Msk)) {
		while(!(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_BUSY_Msk)) {
	    __NOP( ); 
	    __NOP( ); 
		}
		// Transfer Completed Disable SPI
		// SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
  } else {   // Interrupt Mode

    VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);
		
		SPI->reg->IRQ_CLR = SPI_PERIPHERAL_IRQ_CLR_TXIM_Msk;
		memcpy(SPI->info.txdata,data,num);
			
 	  SPI->info.txidx = 0;
		SPI->info.rxidx = 0;
		SPI->info.txcnt = num;
		SPI->info.rxcnt = 0;
			// Enable SPI
    SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;

		SPI->reg->IRQ_ENB = (SPI_PERIPHERAL_IRQ_ENB_TXIM_Msk);
		VOR_Enable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	}
  return VOR_DRIVER_OK;
}

/*
function:    int32_t VOR_SPI_Receive (uint8_t *data, uint32_t num,
																			SPI_RESOURCES *SPI);
  brief:       Receive data from other port
  Input:       data    data receive in bytes
  Input:       num     number of data bytes to receive 
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/	
int32_t VOR_SPI_Receive(uint8_t *data, uint32_t num, VOR_SPI_RESOURCES *SPI)
{
	// uint32_t poll_time, start_time;
	int32_t i;
	uint8_t size_bytes; 
	uint16_t dataIn; 
	if( SPI->info.sizeofwords > 8 )
	 size_bytes = 2; 
	else
	 size_bytes = 1; 
	
	if (!data || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }
	
	if (!(SPI->info.flags & SPI_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }
		
	SPI->reg->IRQ_ENB = 0;
	// Clear FIFO
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk;
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk;
  SPI->info.transfer = 0;
  if(SPI->info.poll_mode) {
		// Get Current Time
		/*start_time = VOR_Get_SysTime();
		poll_time  = start_time - SPI->info.poll_tout;*/ 
		
		// Enable SPI
    // SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		
		// Generate Clock for First Data in Master Mode
		if(SPI->info.msmode == VOR_SPI_MASTER) 
		  SPI->reg->DATA = 0;
		
		SPI->info.rxidx = 0;
		
		while((num !=0) /*&& (poll_time <= VOR_Get_SysTime())*/ ) {
	    __NOP( ); 
	    __NOP( );  
					
			// Check RX fifo status 
			if(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk) {
				// data[SPI->info.rxidx++] = SPI->reg->DATA;
				dataIn = SPI->reg->DATA; 
				memcpy(&data[SPI->info.rxidx++], (const void *)&dataIn, size_bytes);
								
				num--;
				
				// Generte next clock in Master Mode;
				if( num && (SPI->info.msmode == VOR_SPI_MASTER) ) 
					SPI->reg->DATA = 0;
			}		
		} 
		// Disable SPI
		// SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		if(num!=0) {
			return VOR_SPI_ERROR_POLL_TOUT;
		}
  } else {   // Interrupt Mode
  	VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);

   	SPI->reg->IRQ_CLR = (SPI_PERIPHERAL_IRQ_CLR_RORIM_Msk|SPI_PERIPHERAL_IRQ_CLR_RXIM_Msk);
		SPI->info.txidx = 0;
		SPI->info.rxidx = 0;
		SPI->info.txcnt = 0;
		SPI->info.rxcnt = num;
		SPI->info.rxdata = data;
			
		SPI->reg->IRQ_ENB = (SPI_PERIPHERAL_IRQ_ENB_RORIM_Msk | 
													 SPI_PERIPHERAL_IRQ_ENB_RXIM_Msk );
		
		VOR_Enable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	  // Generate Clock for First Data in Master Mode
	  if(SPI->info.msmode == VOR_SPI_MASTER) {
			for(i=0;i<SPI->reg->RXFIFOIRQTRG;i++)
	      SPI->reg->DATA = 0;
		}
		// Enable SPI
    SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
	}	
	return VOR_DRIVER_OK; 
}




/*
function:    int32_t VOR_SPI_Transfer (uint8_t *dataIn, uint8_t *dataOut,uint32_t num,
																			SPI_RESOURCES *SPI);
  brief:       Receive data from other port
  Input:       dataIn  data receive buffer
  Input:       dataOut data send buffer
  Input:       num     number of data bytes  
  Input:       SPI     Pointer to SPI resources
  return:      execution_status
*/	
int32_t VOR_SPI_Transfer(uint8_t *dataIn, uint8_t *dataOut, uint32_t num, VOR_SPI_RESOURCES *SPI)
{
	uint32_t poll_time, start_time;
	
	if (!dataIn || !dataOut || !num ) {
    /* Invalid parameters */
    return VOR_DRIVER_ERROR_PARAMETER;
  }
	
	if (!(SPI->info.flags & SPI_FLAG_INIT)) {
    /* Driver not yet configured */
    return VOR_DRIVER_ERROR;
  }
		
	SPI->reg->IRQ_ENB = 0;
	// Clear FIFO
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk;
	SPI->reg->FIFO_CLR = SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk;
  SPI->info.transfer = 1;
  if(SPI->info.poll_mode) {
		num++;
		// Get Current Time
		start_time = VOR_Get_SysTime();
		poll_time  = start_time - SPI->info.poll_tout;
		// Enable SPI
    SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		SPI->info.rxidx = 0;
		SPI->info.txidx = 0;
		// Generate Clock for First Data in Master Mode
		if(SPI->info.msmode == VOR_SPI_MASTER) 
		  SPI->reg->DATA = (uint32_t)dataOut[SPI->info.txidx++];
		while((num !=0) && (poll_time <= VOR_Get_SysTime())) {
	    __NOP( ); 
	    __NOP( );  
		
			// Check RX fifo status 
			if(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk) {
				dataIn[SPI->info.rxidx++] = SPI->reg->DATA;
			}
			if(SPI->reg->STATUS & SPI_PERIPHERAL_STATUS_TNF_Msk) {
				SPI->reg->DATA = dataOut[SPI->info.txidx++];
			}
			num--;
		} 
		// Disable SPI
		SPI->reg->CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		if(num!=0) {
			return VOR_SPI_ERROR_POLL_TOUT;
		}
  } else {   // Interrupt Mode
  	VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);

   	SPI->reg->IRQ_CLR = (SPI_PERIPHERAL_IRQ_CLR_RORIM_Msk | 
													 SPI_PERIPHERAL_IRQ_CLR_RXIM_Msk );
		SPI->info.txidx = 0;
		SPI->info.rxidx = 0;
		SPI->info.txcnt = num;
		SPI->info.rxcnt = num;
		SPI->info.rxdata = dataIn;
		memcpy(SPI->info.txdata,dataOut,num);
			
		SPI->reg->IRQ_ENB = (SPI_PERIPHERAL_IRQ_ENB_RORIM_Msk | 
													 SPI_PERIPHERAL_IRQ_ENB_RXIM_Msk );
		
		VOR_Enable_Irq(VOR_IRQ_SPI, SPI->SPIId);
		// Enable SPI
    SPI->reg->CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
	  // Generate Clock for First Data in Master Mode
	  if(SPI->info.msmode == VOR_SPI_MASTER) 
	    SPI->reg->DATA = dataOut[SPI->info.txidx++];
	}	
	return VOR_DRIVER_OK; 
}


/**
  function:    void VOR_SPI_IrqHandler (void)
  brief:       SPI Interrupt Handler.
  return:      none
*/
void VOR_SPI_IrqHandler (VOR_SPI_RESOURCES *SPI) {

  uint32_t status, irq_end;
	int32_t i;

  if (!(SPI->info.flags & SPI_FLAG_INIT)) {
    /* Driver not yet configured */
    return;
  }

  irq_end = SPI->reg->IRQ_END;
	
	if(irq_end & SPI_PERIPHERAL_IRQ_END_RORIM_Msk) {
	  if(SPI->info.cb_event)
			SPI->info.cb_event(VOR_SPI_RECEIVE_OVERRUN_RECEIVE_FIFO);
	}
	
  status  = SPI->reg->STATUS;
  
  //Handle all RxFifo IRQ 
  if((irq_end & (SPI_PERIPHERAL_IRQ_END_RORIM_Msk | SPI_PERIPHERAL_IRQ_END_RXIM_Msk)) && SPI->info.rxdata) {
		if(SPI->info.msmode == VOR_SPI_MASTER) {
		  // Check RX fifo status 
		  if((status & SPI_PERIPHERAL_STATUS_RNE_Msk) && (SPI->info.rxidx < SPI->info.rxcnt) ) {
        SPI->info.rxdata[SPI->info.rxidx++] = SPI->reg->DATA;
		    SPI->reg->IRQ_CLR = irq_end;
		    // If Rececive competed, Stop the Recevie
		    if(SPI->info.rxidx >= SPI->info.rxcnt) {
    			SPI->reg->IRQ_ENB &= ~(SPI_PERIPHERAL_IRQ_ENB_RORIM_Msk | 
													   SPI_PERIPHERAL_IRQ_ENB_RXIM_Msk );		
          VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	        if(SPI->info.cb_event)
  			    SPI->info.cb_event(VOR_SPI_TRANSFER_DONE);
			    return;
		    }
				for(i=0;i<SPI->reg->RXFIFOIRQTRG;i++) {
	        // Generate Clock for Next Data in Master Mode
	        if((status & SPI_PERIPHERAL_STATUS_TNF_Msk)) {
  				  if((SPI->info.transfer) && (SPI->info.txidx < SPI->info.txcnt) ) {
    	        SPI->reg->DATA = SPI->info.txdata[SPI->info.txidx++];;
				    }
				    else
      	      SPI->reg->DATA = 0;
				  }
				}
			}
	  } else {   // Slave
		  while((status & SPI_PERIPHERAL_STATUS_RNE_Msk) && (SPI->info.rxidx < SPI->info.rxcnt) ) {
				SPI->info.rxdata[SPI->info.rxidx++] = SPI->reg->DATA;
				status  = SPI->reg->STATUS;
		    // If Rececive competed, Stop the Recevie
		    if(SPI->info.rxidx >= SPI->info.rxcnt) {
    			SPI->reg->IRQ_ENB &= ~(SPI_PERIPHERAL_IRQ_ENB_RORIM_Msk | 
													   SPI_PERIPHERAL_IRQ_ENB_RXIM_Msk );		
          VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	        if(SPI->info.cb_event)
  			    SPI->info.cb_event(VOR_SPI_TRANSFER_DONE);
			    return;
		    }			
			}
			SPI->reg->IRQ_CLR = irq_end;
		}
  		
		return;
  }

  //Handle all TXFifo IRQ 
  if(irq_end & (SPI_PERIPHERAL_IRQ_ENB_TXIM_Msk)) {
    if((status & SPI_PERIPHERAL_STATUS_TXTRIGGER_Msk) && (SPI->info.txidx < SPI->info.txcnt)) {
			SPI->reg->DATA = SPI->info.txdata[SPI->info.txidx++];
		  // If Send competed, Stop the Transmit
		  if(SPI->info.txidx >= SPI->info.txcnt) {
  			SPI->reg->IRQ_ENB &= ~(SPI_PERIPHERAL_IRQ_ENB_TXIM_Msk);	
        VOR_Disable_Irq(VOR_IRQ_SPI, SPI->SPIId);
	      if(SPI->info.cb_event)
			    SPI->info.cb_event(VOR_SPI_TRANSFER_DONE);
		  }
	  }
  }

  SPI->reg->IRQ_CLR = irq_end;
  return;
}

/**
  function:    uint32_t VOR_SPI_GetTxCount (VOR_SPI_RESOURCES *SPI)
  brief:       Get Tx data count.
  return:      number of data bytes transferred; 
*/
uint32_t VOR_SPI_GetTxCount (VOR_SPI_RESOURCES *SPI) {
	return SPI->info.txidx;
}

/**
  function:    int32_t VOR_SPI_GetRxCount (VOR_SPI_RESOURCES *SPI)
  brief:       Get Rx data count.
  return:      number of data bytes transferred; 
*/
uint32_t VOR_SPI_GetRxCount (VOR_SPI_RESOURCES *SPI) {
	return SPI->info.rxidx;
}

