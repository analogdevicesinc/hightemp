/***************************************************************************************
 * @file     utils.h
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

#ifndef UTILS_H
#define UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------------------------------------------------------
#ifndef SKIP_UTILS
int sum(int n,int report);
char* ui8tohex(uint8_t val,char* ptr);
char* ui16tohex(uint16_t val,char* ptr);
char* ui32tohex(uint32_t val,char* ptr);
char* ui32todec(uint32_t v,char* ptr);
void ui32div10(uint32_t n,uint32_t* qot, uint8_t* rem);
void generate_sysresetreq(void);

#endif

//------------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif  /* UTILS_H */

