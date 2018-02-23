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

#ifndef REDIRECT_H
#define REDIRECT_H

#ifdef __cplusplus
extern "C" {
#endif

//#define GPIOB_WIDTH 10
#define GPIOB_WIDTH 24
	
//------------------------------------------------------------------------------
#ifndef SKIP_UTILS
  
void simulation_exit(void);  // Infinity loop

void configure_stdio(int mode);  // Configure STDIO destination
// Define STDIO destination:  
#define STDIO_MODE_INTERNAL 	0
#define STDIO_MODE_PORTA 			1
#define STDIO_MODE_PORTB 			2
#define STDIO_MODE_UART_A 		3
#define STDIO_MODE_UART_B 		4
#define STDIO_MODE_SPI 				5
#define STDIO_MODE_SPI2 			6
#define STDIO_MODE_SPI3 			7
#define STDIO_MODE_TB_IKA 		8
  
void stdout_flush(void);

#endif
//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif  /* REDIRECT_H */

