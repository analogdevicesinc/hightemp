#ifndef __PARSER_H
#define __PARSER_H

#include <stdint.h>

#include "uart.h"
#include "sampling_engine.h"

enum UARTCmdEnum
{
	UARTCmdEnum_None,
	UARTCmdEnum_SetTime,
	UARTCmdEnum_SetExp,
    UARTCmdEnum_SetAcq,
    UARTCmdEnum_SetMux,
	UARTCmdEnum_Single,
    UARTCmdEnum_Start,
    UARTCmdEnum_Stop,
    UARTCmdEnum_Continuous,
	UARTCmdEnum_GetTemp,
    UARTCmdEnum_Version,
    UARTCmdEnum_Reboot,
	UARTCmdEnum_Debug
};

void ParserInit(void);

bool BuildCommand(void);

void ParseCommand(void);

void PrintVersion(void);

void PrintData(void);

#endif
