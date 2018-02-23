//----------------------------------------------------------------------------
// Silicon Space Technology Corporation Confidential and Proprietary
//
//----------------------------------------------------------------------------
// Copyright (c) 2013-2014 Silicon Space Technology Corporation
//
// This file contains confidential and proprietary information of
// Silicon Space Technology Corporation.
//
// The contents of this file may not be released to any third
// party without the prior written consent of Silicon Space Technology Corporation.
//
//----------------------------------------------------------------------------

#include "va108xx.h"
#include <stdio.h>
#include "redirect.h" 

#ifndef SKIP_UTILS
//------------------------------------------------------------------------------
// Implement the minimum number of functions required to support
// standard C input/output operations without a debugger attached.
//------------------------------------------------------------------------------
int redirect_stdio_mode = STDIO_MODE_INTERNAL;
//------------------------------------------------------------------------------
void configure_stdio(int mode)
{
 
  redirect_stdio_mode = mode;
  
  if (redirect_stdio_mode == STDIO_MODE_PORTA) { 
    // Configure BANK0 as outputs
    VOR_GPIO->BANK[0].DIR = 0x000000FFU;
   // Configure BANK0 bit 7 as pulse mode
    VOR_GPIO->BANK[0].PULSE = 0x00000080U;
  }
  
  if (redirect_stdio_mode == STDIO_MODE_PORTB) { 
    // Configure BANK1 as outputs
    VOR_GPIO->BANK[1].DIR = 0x000000FFU;
   // Configure BANK1 bit 7 as pulse mode
    VOR_GPIO->BANK[1].PULSE = 0x00000080U;
  }
  
  if (redirect_stdio_mode == STDIO_MODE_UART_A ||
     redirect_stdio_mode == STDIO_MODE_UART_B) { 
    if (redirect_stdio_mode == STDIO_MODE_UART_A) {
       // Use UART A on PORTA[9]
 //     VOR_IOCONFIG->PORTA[9] = 0x4000;      // Standard input, FuncSel=2 => UARTA_TX
    }
    if (redirect_stdio_mode == STDIO_MODE_UART_B) {
      // Use UART B on PORTB[9]
//      VOR_IOCONFIG->PORTB[9] = 0x2000;      // Standard input, FuncSel=1 => UARTA_TX
    }
    // Configure UART0 as output at 2M 
    //VOR_UART->BANK[0].CLKSCALE = 0x80000064U;  // 2M Baud => 1  36/64, Reset Baud
    VOR_UART->BANK[0].CLKSCALE = 0x80005161U;  // Baud => 1, Reset Baud
    VOR_UART->BANK[0].CTRL = 0x00000230U;      // Defaults
    VOR_UART->BANK[0].ENABLE = 0x02U;          // Enable, Tx
  }
  
  if (redirect_stdio_mode == STDIO_MODE_SPI) {
    // Use SPI B on PORTB[5:2]
    VOR_IOCONFIG->PORTB[5] = 0x2000;      // Standard input, FuncSel=1 => SPI_SCKB
    VOR_IOCONFIG->PORTB[4] = 0x2001;      // Standard input no-sync, FuncSel=1 => SPI_MOSIB
    VOR_IOCONFIG->PORTB[3] = 0x2000;      // Standard input, FuncSel=1 => SPI_MISOB
    VOR_IOCONFIG->PORTB[2] = 0x2000;      // Standard input, FuncSel=1 => SPI_SSELB[0]
    // Configure SPI0 as output at 2x clock
    VOR_SPI->BANK[1].CLKPRESCALE = 2;
    VOR_SPI->BANK[1].CTRL0 = 0x00000107U;     // Div 2, 8 bits
    VOR_SPI->BANK[1].CTRL1 = 0x00000002U;     // Enable, SSEL0
  }

  if (redirect_stdio_mode == STDIO_MODE_SPI2) {
		// Use SPI B on PORTB[15:12]
		VOR_IOCONFIG->PORTB[15] = 0x4000;      // Standard input, FuncSel=2 => SPI_SCKB
		VOR_IOCONFIG->PORTB[14] = 0x4001;      // Standard input no-sync, FuncSel=2 => SPI_MOSIB
		VOR_IOCONFIG->PORTB[13] = 0x4000;      // Standard input, FuncSel=2 => SPI_MISOB
		VOR_IOCONFIG->PORTB[12] = 0x4000;      // Standard input, FuncSel=2 => SPI_SSELB[0]
    // Configure SPI0 as output at 2x clock
    VOR_SPI->BANK[1].CLKPRESCALE = 2;
    VOR_SPI->BANK[1].CTRL0 = 0x00000107U;     // Div 2, 8 bits
    VOR_SPI->BANK[1].CTRL1 = 0x00000002U;     // Enable, SSEL0
  }
  
  if (redirect_stdio_mode == STDIO_MODE_SPI3) {
    // Configure SPI0 as output at 2x clock
    VOR_SPI->BANK[2].CLKPRESCALE = 2;
    VOR_SPI->BANK[2].CTRL0 = 0x00000107U;     // Div 2, 8 bits
    VOR_SPI->BANK[2].CTRL1 = 0x00000002U;     // Enable, SSEL0
  }

}

//------------------------------------------------------------------------------
// Define the location of the output console register in the memory model.
volatile unsigned char *console = (volatile unsigned char *) 0x40007000U;

// Implement a simple structure for C's FILE handle.
struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

//------------------------------------------------------------------------------
// Implement file IO handling, only console output is supported.
int fputc(int ch, FILE *f) 
{ 
  int st;
  uint32_t dm;
  if (redirect_stdio_mode == STDIO_MODE_INTERNAL) { 
    *console = ch; 
  }

  if (redirect_stdio_mode == STDIO_MODE_PORTA) { 
    dm = VOR_GPIO->BANK[0].DATAMASK;
    VOR_GPIO->BANK[0].DATAMASK  = 0xff;       // Write DataMask
    VOR_GPIO->BANK[0].DATAOUT   = 0x80 | ch;  // Pulse bit 7
    VOR_GPIO->BANK[0].DATAMASK = dm;
  }
  
  if (redirect_stdio_mode == STDIO_MODE_PORTB) { 
    dm = VOR_GPIO->BANK[1].DATAMASK;
    VOR_GPIO->BANK[1].DATAMASK = 0xff;      // Write DataMask
    VOR_GPIO->BANK[1].DATAOUT = 0x80 | ch;  // Pulse bit 7
    VOR_GPIO->BANK[1].DATAMASK = dm;
  }
  
  if (redirect_stdio_mode == STDIO_MODE_UART_A ||
      redirect_stdio_mode == STDIO_MODE_UART_B) {
    while (1) {
      st = VOR_UART->BANK[0].TXSTATUS;
      if ((st & 0x01) != 0) break;  // Not FifoFull
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
    VOR_UART->BANK[0].DATA = ch;
  }
  
  if (redirect_stdio_mode == STDIO_MODE_SPI ||
		  redirect_stdio_mode == STDIO_MODE_SPI2 ) {
    while (1) {
      st = VOR_SPI->BANK[1].STATUS;
      if ((st & 0x02) != 0) break; // Not FifoFull
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
    VOR_SPI->BANK[1].DATA = ch;
  } 

  if (redirect_stdio_mode == STDIO_MODE_SPI3 ) {
    while (1) {
      st = VOR_SPI->BANK[2].STATUS;
      if ((st & 0x02) != 0) break; // Not FifoFull
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
    VOR_SPI->BANK[2].DATA = ch;
  } 
  
#ifndef VOR_PRINT_BYTE
#define VOR_PRINT_BYTE 0
#endif  
  if (redirect_stdio_mode == STDIO_MODE_TB_IKA) {  
    dm = VOR_GPIO->BANK[1].DATAMASK;
    VOR_GPIO->BANK[1].DATAMASK = 0xFFU;      // Write DataMask
    VOR_GPIO->BANK[1].DATAOUT  = 0x80U | (ch << (VOR_PRINT_BYTE*8)) ;   
    VOR_GPIO->BANK[1].DATAMASK = dm;
  }
  
  return ch; 
}
//------------------------------------------------------------------------------
void stdout_flush(void)
{
  int st;
  if (redirect_stdio_mode == STDIO_MODE_UART_A ||
       redirect_stdio_mode == STDIO_MODE_UART_B) {
    while (1) {
      st = VOR_UART->BANK[0].TXSTATUS;
      if ((st & 0x06) == 0) break;  // NotBusy and FifoEmpy
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
  }
  if (redirect_stdio_mode == STDIO_MODE_SPI ||
		  redirect_stdio_mode == STDIO_MODE_SPI2 ) {
    while (1) {
      st = VOR_SPI->BANK[1].STATUS;
      if ((st & 0x10)==0 && (st & 0x01)==1) break; // NotBusy and FifoEmpty
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
  }
  if (redirect_stdio_mode == STDIO_MODE_SPI3 ) {
    while (1) {
      st = VOR_SPI->BANK[2].STATUS;
      if ((st & 0x10)==0 && (st & 0x01)==1) break; // NotBusy and FifoEmpty
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
  }
}
//------------------------------------------------------------------------------
int fflush(FILE *f) { 
  stdout_flush();
  return 0; 
}
//------------------------------------------------------------------------------
int ferror(FILE *f) { 
  return 0; 
}
//------------------------------------------------------------------------------
int fgetc(FILE *f) { 
  return -1; 
}
//------------------------------------------------------------------------------
int __backspace(FILE *stream) {
  return 0; 
}

//------------------------------------------------------------------------------
void _ttywrch(int ch) {
  fputc(ch,&__stdout); 
}

//------------------------------------------------------------------------------
void simulation_exit(void)
{
  // Writing 0xD to the console causes the DesignStart testbench to finish.
  fputc(0x0D,&__stdout); 
  //stdout_flush();
  while(1);
}

//------------------------------------------------------------------------------
#endif

