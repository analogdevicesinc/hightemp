/***************************************************************************************
 * @file     circular_buffer.c
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
#include <string.h>
#include "circular_buffer.h"
#include "reb_log.h"

void VOR_CircularBuffer_Initialize(VOR_CIRCULAR_BUFFER *pBuffer, void *pData, uint16_t item_size, uint16_t max_items) {
	pBuffer->max_items = max_items; 
	pBuffer->valid_items = 0; 
	pBuffer->data = pData; 
	pBuffer->item_size = item_size; 
	pBuffer->read_index = 0; 
	pBuffer->write_index = 0; 
}

bool VOR_CircularBuffer_IsEmpty(VOR_CIRCULAR_BUFFER *pBuffer) {
	if( pBuffer->valid_items == 0 ) {
		VOR_Log(LOG_DBG, "The Buffer is Empty! \n");
		return true; 
	} else {
		return false; 
	}
}

bool VOR_CircularBuffer_IsFull(VOR_CIRCULAR_BUFFER *pBuffer) {
	if( pBuffer->valid_items == pBuffer->max_items ) {
		VOR_Log(LOG_DBG, "The Buffer is Full! \n");
		return true; 
	} else {
		return false; 
	}
}

int32_t VOR_CircularBuffer_Write(VOR_CIRCULAR_BUFFER *pBuffer, const void *pItem) {
	if( VOR_CircularBuffer_IsFull(pBuffer) ) {
		VOR_Log(LOG_DBG, "The Buffer is Full! \n");
		return -1; 
	} else {
		uint16_t *wPtr = (uint16_t *)pBuffer->data + pBuffer->write_index++; 
		pBuffer->valid_items++; 
		memcpy(wPtr, pItem, pBuffer->item_size); 
		if( pBuffer->write_index == pBuffer->max_items )
			pBuffer->write_index = 0; 
	}
	return 0; 
}

int32_t VOR_CircularBuffer_Read(VOR_CIRCULAR_BUFFER *pBuffer, void *pItem) {
	if( VOR_CircularBuffer_IsEmpty(pBuffer) ) {
		VOR_Log(LOG_DBG, "The Buffer is Empty! \n");
		return -1; 
	} else {
		uint16_t *rPtr = (uint16_t *)pBuffer->data + pBuffer->read_index++; 
		pBuffer->valid_items--; 
		memcpy(pItem, rPtr, pBuffer->item_size); 
		if( pBuffer->read_index == pBuffer->max_items )
			pBuffer->read_index = 0; 
	}
	return 0; 
}
