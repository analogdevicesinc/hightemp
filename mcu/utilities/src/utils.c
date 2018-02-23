/***************************************************************************************
 * @file     utils.c
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

#include "VA108xx.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "utils.h"

#ifndef SKIP_UTILS
//------------------------------------------------------------------------------
// Sample math
//------------------------------------------------------------------------------
int sum(int n,int report)
{
	int result;
	result = 1;
	while (n>1) {
		result += n;
		n--;
	}
	if (report) {
    printf("Sum Result: %d\n",result);
	}
	return result;
}
//------------------------------------------------------------------------------
// Convert to Hex Display
//------------------------------------------------------------------------------
char* ui8tohex(uint8_t val,char* ptr) // ptr needs to be size>=3
{
	// Use chars vs string, to remove the trailing null byte
	// Making arry of chars vs char* keeps this all in the code space without a char* pointer in the data area
	// Using a table avoids branches in the code, for faster/smaller
  static const char ui8tohex_table[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f'};
	uint8_t v;
	v = (val >> 4);
	//if (v > 9) v += 7;
	//ptr[0] = (char) (v+0x30);
	ptr[0] = ui8tohex_table[v];
	v = val & 0x0f;
	//if (v > 9) v += 7;
	//ptr[1] = (char) (v+0x30);
	ptr[1] = ui8tohex_table[v];
	ptr[2] = (char) 0;
	return ptr;
}
//------------------------------------------------------------------------------
char* ui16tohex(uint16_t val,char* ptr) // ptr needs to be size>=5
{
	ui8tohex(val >> 8,ptr);
	ui8tohex(val & 0xff,ptr+2);
	return ptr;
}
//------------------------------------------------------------------------------
char* ui32tohex(uint32_t val,char* ptr) // ptr needs to be size>=9
{
	ui16tohex(val >> 16,ptr);
	ui16tohex(val & 0xffff,ptr+4);
	return ptr;
}
//------------------------------------------------------------------------------
// Convert to Decimal display
//------------------------------------------------------------------------------
char* ui32todec(uint32_t v,char* ptr) // ptr needs to be size>=11
{
	uint8_t r;
	uint8_t start=9;
	uint8_t i=9;
	ptr[10] = (char) 0;
	ptr[9] = '0';
	while (v!=0) {
		ui32div10(v,&v,&r);
		ptr[i] = (char) (r+0x30);
		if (r!=0) start=i;
		if (i == 0) break;
		i -= 1;
}
	return &(ptr[start]);
}
//------------------------------------------------------------------------------
// Div10 - Without Multiply or Divide
// Reference: http://www.hackersdelight.org/divcMore.pdf
//------------------------------------------------------------------------------
void ui32div10(uint32_t n,uint32_t* qot, uint8_t* rem)
{
	uint32_t q;
	uint8_t r;
  q = (n >> 1) + (n >> 2);
  q = q + (q >> 4);
  q = q + (q >> 8);
  q = q + (q >> 16);
  q = (q >> 3);
  r = n - (((q << 2) + q) << 1);
  if (r>9) {
    q = q + 1;
    r = r - 10;
	}
	*qot = q;
	*rem = r;
}
//------------------------------------------------------------------------------
// Force SYSRESETREQ
//------------------------------------------------------------------------------
void generate_sysresetreq(void)
{
  // Force SYSRESETREQ
  uint32_t *AIRCR = (uint32_t*)0xe000ed0cUL;
  *AIRCR = 0x05fa0004;  
}

#endif

//------------------------------------------------------------------------------
