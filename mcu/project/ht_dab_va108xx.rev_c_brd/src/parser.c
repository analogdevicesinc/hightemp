/***************************************************************************************
 * @file     parser.c
 * @version  V1.0
 * @date     18. Jan 2018
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
#include "parser.h"

#include "output.h" //DWM only hacked in temp for debugging RTD output

/* Global definitions - variables accessed from different subroutines */
/* Max size of RX command */
static const uint32_t ui32RxCmdBufferSize = 64;

static uint8_t pui8RxCmdBuffer[ui32RxCmdBufferSize];
static uint32_t ui32RxCmdBufferIndex;

static char pcStrBuffer[64];

extern uint8_t *pui8RxHead;  
 


/*******************************************************************************
 **
 ** @brief Parser Init -  Original plan was to init buffer but not needed
 **      -  Kept as placeholder for future use if needed. 
 **
 ******************************************************************************/
 void
 ParserInit(void)
 {
    // No init required  
 }

/*******************************************************************************
  * @brief attempts to parse command line string into command enum+param list
  * Call upon reception of \r delimiter
	* looks at rxCommandASCII string which has $ and \r delimiters snipped off
  * assigns UARTCmdEnum + up to 5x decimal integer entries in pui32ParamList[]
  * sets uartRxStatus=UARTRxStatusEnum_Error if command string is unrecognized, or number of valid numeric params does not agree with spec for command string
  * @param  none
  * @retval none
  */

/**
  * @brief Builds a line buffer char by char from the UART Rx ISR's circular buffer
  * Always resets on '$', ends on '\r', discards '\n', backs up on BACKSPACE
  * all other chars add to line buffer
  * No protection against underrun here, do not call unless buffer is not empty
  * @param  none
  * @retval none
  */

/*******************************************************************************
 **
 ** @brief Build Command - Takes characters from UART buffer and creates command string
 **
 ******************************************************************************/
bool
BuildCommand(void)
{
    uint8_t ui8RxChar;
    
    /* Read character received from terminal, exiting if buffer was empty. */
    if(UART0Read(&ui8RxChar) == false)
    {
        return(false);
    }
 

            VOR_UART0.DATA = ui8RxChar;
            
            if(ui8RxChar=='\r')
            {
                VOR_UART0.DATA = '\n'; //add a LF because PuTTY doesn't make them from Enter key
            }
    /* Build command, handling special console characters as needed. */
	switch(ui8RxChar)
    {
        /* Start of command. */
		case '$':
        {
            ui32RxCmdBufferIndex = 0;
            pui8RxCmdBuffer[ui32RxCmdBufferIndex++] = ui8RxChar;
            break;
		}	
        
        /* Cariage return. */
		case '\r':
        {
            /* Terminate new string. */
            pui8RxCmdBuffer[ui32RxCmdBufferIndex] = 0;
            
            /* Check for zero-length string. */
			if(ui32RxCmdBufferIndex != 0)
            {   
                /* Reset index. */
                ui32RxCmdBufferIndex = 0;
                
                return(true);
            }
			break;
        } 
        
        /* New line. Do nothing. */
		case '\n':
        {
			break;
        }
        
        /* Backspace key. Decrement buffer index. */
		case 0x7F:
        {
            /* Decrement index, if able. */
            if(ui32RxCmdBufferIndex > 0)
            {
                ui32RxCmdBufferIndex--;    
            }
            
            break;
        }
        
        /* All other characters. Add them to string if valid alpha-numerics. */
		default:
        {
            /* Only add if character is an alpha-numerics, and a valid command
             * has been started. */
            if(((('a' <= ui8RxChar) && (ui8RxChar <= 'z')) || 
                (('A' <= ui8RxChar) && (ui8RxChar <= 'Z')) || 
                (('0' <= ui8RxChar) && (ui8RxChar <= '9')) ||
                (ui8RxChar == ' ')) && (ui32RxCmdBufferIndex != 0))
            {
                /* Convert all uppercase characters to lowercase if needed. */
                if((('A' <= ui8RxChar) && (ui8RxChar <= 'Z')))
                {
                    ui8RxChar -= ('A' - 'a');
                }
                
                /* Add character to buffer. */
                pui8RxCmdBuffer[ui32RxCmdBufferIndex++] = ui8RxChar;
            }
			break;
		}
	}
    
    return(false);
}
 
/*******************************************************************************
 **
 ** @brief ParseCommand -  Set flags for future action if valid command available
 **
 ******************************************************************************/
void
ParseCommand(void)
{
    char *pcInputStr;
    char pcCmdStr[12];
    uint32_t pui32ParamList[5];
    uint32_t ui32ParamCount;
    uint32_t ui32ExpectedParamCount;
    enum UARTCmdEnum iCmdEnum;
    
    /* Only do things if a new command has been received. */
    if(BuildCommand() == false)
    {
        return;
    }
    
    /* Set pointer to string to ignore '$' command delimeter. */
    pcInputStr = (char *)(pui8RxCmdBuffer + 1);
    
    /* Determine how many parameters command was passed. */
    memset(pui32ParamList, 0, sizeof(pui32ParamList));
	ui32ParamCount = sscanf(pcInputStr, "%12s %d %d %d %d %d", pcCmdStr, 
                            &pui32ParamList[0], &pui32ParamList[1], 
                            &pui32ParamList[2], &pui32ParamList[3], 
                            &pui32ParamList[4]);
	
    /* Parse command string to find a valid command. */
    ui32ExpectedParamCount = 0;
	if(strcmp(pcCmdStr, "settime") == 0)
    {
		iCmdEnum = UARTCmdEnum_SetTime;
        
		ui32ExpectedParamCount = 1;
	}
	else if(strcmp(pcCmdStr, "setexp") == 0)
    {
		iCmdEnum = UARTCmdEnum_SetExp;
        
        ui32ExpectedParamCount = 1;
	}
	else if(strcmp(pcCmdStr, "setacq") == 0)
    {
		iCmdEnum = UARTCmdEnum_SetAcq;
        
        ui32ExpectedParamCount = 5;
	}
	else if(strcmp(pcCmdStr, "continuous") == 0)
    {
		iCmdEnum = UARTCmdEnum_Continuous;
        
        ui32ExpectedParamCount = 1;
	}
	else if(strcmp(pcCmdStr, "debug") == 0)
    {
		iCmdEnum = UARTCmdEnum_Debug;
	}
	else if(strcmp(pcCmdStr, "single") == 0)
    {
		iCmdEnum = UARTCmdEnum_Single;
	}
	else if(strcmp(pcCmdStr, "start") == 0)
    {
		iCmdEnum = UARTCmdEnum_Start;
	}
	else if(strcmp(pcCmdStr, "stop") == 0)
    {
		iCmdEnum = UARTCmdEnum_Stop;
	}	
	else if(strcmp(pcCmdStr, "reboot") == 0)
    {
		iCmdEnum = UARTCmdEnum_Reboot;
	}	
	else if(strcmp(pcCmdStr, "gettemp") == 0)
    {
		iCmdEnum = UARTCmdEnum_GetTemp;
	}
	else if(strcmp(pcCmdStr, "version") == 0)
    {
		iCmdEnum = UARTCmdEnum_Version;
	}
    else
    {
        iCmdEnum = UARTCmdEnum_None;
        
        /* Indicate parsing error and exit. */
        
        sprintf(pcStrBuffer, 
                "$ERROR: unsupported %s Cmd:%0d p0:%0d p1:%0d p2:%0d\r\n",
                pcInputStr, 0, pui32ParamList[0], pui32ParamList[1],
                pui32ParamList[2]);
                
        UART0WriteStr(pcStrBuffer);
        
        return;
    }
  
    /* Validate number of parameters before executing command. */
    if(ui32ParamCount != (ui32ExpectedParamCount + 1))
    {
        /* Null-terminate command in case parsing was unsuccessful. */
        pcCmdStr[sizeof(pcCmdStr) - 1] = 0;
        
        /* Display error to user. */
        sprintf(pcStrBuffer, "\r\n$ERROR: %s\r\n", pcCmdStr);
        UART0WriteStr(pcStrBuffer);
        
        return;
	}

    /* Execute command, parsing parameters if needed. */
    switch(iCmdEnum)
    {
        /* Prints the number of seconds since POR.  Sets the time if a value is\
         * provided. */
        case UARTCmdEnum_SetTime:
        {
            do
            {
                /* Set the time if a value was provided. */
                if(ui32ParamCount == 2)
                {
                    if(SetTime(pui32ParamList[0]) == false)
                    {
                        break;
                    }
                }
                
                /* Display updated time. */
                sprintf(pcStrBuffer, "$TIME:%010d\r\n", GetTime());
                UART0WriteStr(pcStrBuffer);
                
                return;
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;
        }
        
        /* Configures delay between consecutive samples whilst in burst mode. */
        /* Command:
            setexp <Seconds Between Samples> */
        case UARTCmdEnum_SetExp:
        {
            do
            {
                /* Set delay between burst mode samples. */
                if(SetBurstModeSamplePeriod(pui32ParamList[0]) == false)  // 
                {
                   break;
								}
								if(GetMode() == RunModeEnum_ExtTrig) 
								{
								
								       
                  /* Start sampling. */
                  if(Start() == false)
                  {
                     break;
                  }
									UART0WriteStr("$OK: ExternalTrig Mode ready \r\n");
                }
            
 
								

                /* Indicate success. */
                UART0WriteStr("$OK: setexp\r\n"); 
                
                return;
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;
        }
      
        /* Configures ADC01 and ADC Mux data aquisition parameters. */
        /* Command:
            setacq <ADC01 Period (0.1us)> <ADC01 Samples>
             <ADCMux Period (0.1us)> <ADCMux Samples> <Channel BitMask> */
        case UARTCmdEnum_SetAcq:
        {
            do
            {
                /* Set acquisition parameters. */
                if(SetAcquisitionParams(pui32ParamList[4], pui32ParamList[0],
                                        pui32ParamList[1], pui32ParamList[2],
                                        pui32ParamList[3]) == false)
                {
                    break;
                }
            
                /* Indicate success. */
                UART0WriteStr("$OK: setacq\r\n");
                
                return;
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;
        }
        
        /* Starts sampling in continuous mode. */
        case UARTCmdEnum_Continuous:
        {
            do
            {
                /* Start sampling all channels at the user-provided rate. */
                if(SetContinuousModeSamplePeriod(pui32ParamList[0]) == false)
                {
                    break;
                }
                
                /* Set run mode. */
                if(SetMode(RunModeEnum_Continuous) == false)
                {
                    break;
                }
                
                /* Start sampling. */
                if(Start() == false)
                {
                    break;
                }
                
                /* Indicate success. */
                sprintf(pcStrBuffer, "$OK: continuous %0d us\r\n",
                        pui32ParamList[0]);
                UART0WriteStr(pcStrBuffer);
                
                return;
                
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;
        }
        
        /* Start sampling in burst mode. */
        case UARTCmdEnum_Start:
        {
            do
            {
                /* Set run mode. */
                if(SetMode(RunModeEnum_Burst) == false)
                {
                    break;
                }
                
                /* Start sampling. */
                if(Start() == false)
                {
                    break;
                }
                
                /* Indicate success. */
                UART0WriteStr("$OK: start\r\n");
                
                return;
                
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;            
        }
        
        /* Stops all active sampling. */
        case UARTCmdEnum_Stop:    
        {
            Stop();
            
            UART0WriteStr("$OK: stop\r\n");	
            
            break;
        }
        
        /* Performs a single burst sampling cycle. */
        case UARTCmdEnum_Single:
        {
            do
            {
                /* Set run mode. */
                if(SetMode(RunModeEnum_SingleShot) == false)
                {
                    break;
                }
                
                /* Start sampling. */
                if(Start() == false)
                {
                    break;
                }
                
                /* Indicate success. */
                UART0WriteStr("$OK: single\r\n");
                
                return;
                
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;								
        }
        
        /* Configures and starts a single sampling cycle, only sampling the RTD
           and VCC. */
        case UARTCmdEnum_GetTemp:
        {
            do
            {   
                /* Set run mode. */
                if(SetMode(RunModeEnum_SingleShot) == false)
                {
                    break;
                }
                
                /* Only oversample the RTD and VCC channels.  Only compatible
                   with Single Shot mode. */
                if(OnlyOversample() == false)
                {
                    break;
                }
                
                /* Start sampling. */
                if(Start() == false)
                {
                    break;
                }
                
                /* Indicate success. */
                UART0WriteStr("$OK: gettemp\r\n");
                
                return;
                
            }
            while(false);
            
            /* Indicate an error occurred. */
            UART0WriteStr("$ERROR\r\n");
            
            break;
        }
        
        /* Prints application version information. */
        case UARTCmdEnum_Version:
        {
            PrintVersion();
            
            break;
        }
        
        /* Immediately resets the CPU. */
        case UARTCmdEnum_Reboot:
        {
            NVIC_SystemReset();
            
            break;
        }
        
        /* Initial board bring-up helper - Outputs various data to terminal */
        case UARTCmdEnum_Debug:
        {
			      uint32_t adcVal;
					  int32_t retVal;
					
            /*sprintf(pcStrBuffer, "$time=%010x,",secondCount);
            UART0WriteStr(pcStrBuffer);
            
            sprintf(pcStrBuffer, "burstMode_expPeriodSeconds=%010x,",
                    burstMode_expPeriodSeconds);
            UART0WriteStr(pcStrBuffer);
            
            sprintf(pcStrBuffer, "adc01Mode_sampPeriod_100ns=%010x,",
                    adc01Mode_sampPeriod_100ns);
            UART0WriteStr(pcStrBuffer);
            
            sprintf(pcStrBuffer, "adc01Mode_sampSize=%010x,\r\n",
                    adc01Mode_sampSize);
            UART0WriteStr(pcStrBuffer);*/
					
					/*
					    sprintf(pcBurstStatsStrBuffer,
            "$TEMP:%03d.%01dC,$VOLT=%01d.%03dV,$TIME=%010d\r\n",
            ((int32_t)ui32RTD - 2732) / 10, abs(((int32_t)ui32RTD - 2732)) % 10, ui32VCC / 1000,
            ui32VCC % 1000, ui32Time);*/
					
					  for(adcVal=0xa500;adcVal<=0xbd00;adcVal+=0x05){
							  retVal=getTempK10_from_adcCodeSum(adcVal*configRTD_SAMPLES);
					      sprintf(pcStrBuffer, "adcCode=0x%04x, $TEMP=%03d.%01dC\r\n",adcVal,((int32_t)retVal - 2732) / 10, abs(((int32_t)retVal - 2732)) % 10);
                UART0WriteStr(pcStrBuffer);
					  }
            
            /* Manually toggle Ext Trig. */
            UART0WriteStr("Pulsing B15.\r\n");
            VOR_PORTB.DIR |= (1UL << 15);
            VOR_PORTB.SETOUT = 1UL << 15;
					  __NOP( );
            __NOP( );
            __NOP( );
            VOR_PORTB.CLROUT = 1UL << 15;
            
            break;
        }
        
        /* Do nothing. */
        default:
        {
        }
    }
}
