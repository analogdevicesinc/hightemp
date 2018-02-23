/***************************************************************************************
 * @file     circular_buffer.h
 * @version  V1.0
 * @date     22. April 2016
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
#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
\brief Access structure for circular buffer utility.
*/
typedef struct _VOR_CIRCULAR_BUFFER {
	uint16_t write_index;		///< Write index, incremented when a new item added to buffer
	uint16_t read_index; 		///< Read index, incremented when a new item is consumed from buffer
	uint16_t valid_items; 	///< Specifies number of valid items in buffer
	void *data; 						///< Pointer to buffer where all items are located
	uint16_t max_items; 		///< Specifies maximum number of items that this buffer will hold
	uint16_t item_size; 		///< Specifies size of each item in bytes 
} VOR_CIRCULAR_BUFFER; 

/**
 \fn          void VOR_CircularBuffer_Initialize(VOR_CIRCULAR_BUFFER *pBuffer, void *pData, uint16_t item_size, uint16_t max_items)
 \brief       Initilize circular buffer.
 \param[in]   pBuffer			Pointer to circular buffer access structure \ref VOR_CIRCULAR_BUFFER
 \param[in]		pData				Pointer to buffer where all items are stored
 \param[in]		item_size		Specifies the size of each item in bytes 
 \param[in]		max_items 	Specifies the maximum number of items that this buffer will hold
 \return      none
*/ 
void VOR_CircularBuffer_Initialize(VOR_CIRCULAR_BUFFER *pBuffer, void *pData, uint16_t item_size, uint16_t max_items); 

/**
 \fn				 bool VOR_CircularBuffer_IsEmpty(VOR_CIRCULAR_BUFFER *pBuffer)
 \brief 		 Check whether buffer is empty. 
 \param[in]	 pBuffer Pointer to circular buffer access structure \ref VOR_CIRCULAR_BUFFER
 \return		 True if buffer is empty otherwise return false
*/ 
bool VOR_CircularBuffer_IsEmpty(VOR_CIRCULAR_BUFFER *pBuffer); 

/**
 \fn				 bool VOR_CircularBuffer_IsFull(VOR_CIRCULAR_BUFFER *pBuffer)
 \brief 		 Check whether buffer is full. 
 \param[in]	 pBuffer Pointer to circular buffer access structure \ref VOR_CIRCULAR_BUFFER
 \return		 True if buffer is full otherwise return false
*/ 
bool VOR_CircularBuffer_IsFull(VOR_CIRCULAR_BUFFER *pBuffer); 

/**
 \fn				 int32_t VOR_CircularBuffer_Write(VOR_CIRCULAR_BUFFER *pBuffer, const void *pItem)
 \brief 		 Write specified item to buffer at location pointed by write index. 
 \param[in]	 pBuffer Pointer to circular buffer access structure \ref VOR_CIRCULAR_BUFFER
 \param[in]  pItem 	 Pointer to new item that will be added to circualr buffer
 \return		 -1 if write fails because circular buffer is full
*/ 
int32_t VOR_CircularBuffer_Write(VOR_CIRCULAR_BUFFER *pBuffer, const void *pItem); 

/**
 \fn				 int32_t VOR_CircularBuffer_Read(VOR_CIRCULAR_BUFFER *pBuffer, void *pItem)
 \brief 		 Read an item at current read index. 
 \param[in]	 pBuffer Pointer to circular buffer access structure \ref VOR_CIRCULAR_BUFFER
 \param[out] pItem 	 new item retrieved from circualr buffer
 \return		 -1 if read fails because circular buffer is empty 
*/ 
int32_t VOR_CircularBuffer_Read(VOR_CIRCULAR_BUFFER *pBuffer, void *pItem); 

#endif  /* CIRCULAR_BUFFER_H */
