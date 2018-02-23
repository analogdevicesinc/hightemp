/***************************************************************************************
 * @file     output.c
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

#include "output.h"

/** Macro Generic float precalc for parallel resistances **/
#define PARALLEL(X,Y)	((double)1/(((double)1/(double)(X))+(double)1/((double)(Y))))

/** Macro Generic float precalc for series voltage divider (high,low), return val always <1.0 **/
#define SERIES_DIV(X,Y) ((double)(Y)/((double)(Y)+(double)(X)))

/** Fullscale value for ADC **/
#define ADC_FULLSCALE 65535

#define RTD_OHMS_TO_ADC_CODE_SUM(X)                                         \
						((configRTD_SAMPLES*(double)ADC_FULLSCALE)                      \
							*(SERIES_DIV( PARALLEL(ADC_R_BIAS_UPPER,RTD_R_PULLUP),           \
						           PARALLEL(ADC_R_BIAS_LOWER,                         \
						                       (double)(X)+RTD_R_PULLDOWN))))
						
/** RTD resistances at known, 40C increments. */
#define RTD_OHMS_N40C           843
#define RTD_OHMS_0C             1000
#define RTD_OHMS_40C            1155
#define RTD_OHMS_80C            1309
#define RTD_OHMS_120C           1461
#define RTD_OHMS_160C           1610
#define RTD_OHMS_200C           1759
#define RTD_OHMS_240C           1905

/** Precalulated ADC samples for known RTD values. */
#define RTD_ADC_CODE_SUM_N40C     RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_N40C)
#define RTD_ADC_CODE_SUM_0C       RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_0C)
#define RTD_ADC_CODE_SUM_40C      RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_40C)    
#define RTD_ADC_CODE_SUM_80C      RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_80C) 
#define RTD_ADC_CODE_SUM_120C     RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_120C)
#define RTD_ADC_CODE_SUM_160C     RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_160C)
#define RTD_ADC_CODE_SUM_200C     RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_200C)
#define RTD_ADC_CODE_SUM_240C     RTD_OHMS_TO_ADC_CODE_SUM(RTD_OHMS_240C)

/** Precalculated slopes for interpolation calculations. */

#define RTD_SLOPE_N40_TO_0C     ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_0C - RTD_ADC_CODE_SUM_N40C))
#define RTD_SLOPE_0_TO_40C      ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_40C - RTD_ADC_CODE_SUM_0C))
#define RTD_SLOPE_40_TO_80C     ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_80C - RTD_ADC_CODE_SUM_40C))
#define RTD_SLOPE_80_TO_120C    ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_120C - RTD_ADC_CODE_SUM_80C))
#define RTD_SLOPE_120_TO_160C   ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_160C - RTD_ADC_CODE_SUM_120C))
#define RTD_SLOPE_160_TO_200C   ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_200C - RTD_ADC_CODE_SUM_160C))
#define RTD_SLOPE_200_TO_240C   ((65536*400) /                                 \
                                (RTD_ADC_CODE_SUM_240C - RTD_ADC_CODE_SUM_200C))


/* Thevenin voltag as a fraction of fullscale, in FP */
#define VCC_VTH_FRAC    (SERIES_DIV(ADC_R_BIAS_UPPER,PARALLEL(ADC_R_BIAS_LOWER,VCC_R_DIV_LOWER)))
/* This is FP calc for the net divider ratio of Vout/Vin, not counting the Thevenin voltage offset */
#define VCC_TH_FRAC   ((SERIES_DIV(VCC_R_DIV_UPPER,PARALLEL(ADC_R_BIAS_UPPER,PARALLEL(ADC_R_BIAS_LOWER,VCC_R_DIV_LOWER)))))
/* Net uint32_t multiplier for oversampled VCC codesum to make voltage*1000 */
#define VCC_TH_MULT_LSHIFT16 ((uint32_t)(((double)1000.0*VCC_REF*65536)/(configRTD_SAMPLES*(double)ADC_FULLSCALE*VCC_TH_FRAC)))
/*  Net uint32_t voltage*1000 offset for Thevenin voltage */
#define VCC_TH_COMP_LSHIFT16 ((uint32_t)(((double)1000.0*VCC_REF*65536*VCC_VTH_FRAC*(((double)1.0/VCC_TH_FRAC)-(double)1.0))))


static const char m_pcNibbleToAscii[16] =
{
    '0', '1', '2', '3', '4', '5', '6', '7',
    '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};
    
    
static char m_pcStrBuffer[64];

static char m_pcContinuousStrBuffer[64] = "$CONT:";

/*******************************************************************************
 **
 ** @brief Prints welcome message, firmware version, and cause of last reset.
 **
 ******************************************************************************/
void
PrintVersion(void)
{
    /* Print header. */
    sprintf(m_pcStrBuffer, "\r\n--\r\n");
    UART0WriteStr(m_pcStrBuffer);	
    
    sprintf(m_pcStrBuffer, "-- $MSG: High Temperature Demonstration board\r\n");
    UART0WriteStr(m_pcStrBuffer);	
    
    sprintf(m_pcStrBuffer, "--\r\n");
    UART0WriteStr(m_pcStrBuffer);
    
    /* Print cause of last reset. */
    if(VOR_SYSCONFIG->RST_STAT != 0)
    {
        sprintf(m_pcStrBuffer, "-- $MSG: Reset source = ");
        UART0WriteStr(m_pcStrBuffer);
        
        if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_POR_Msk)
        {
            sprintf(m_pcStrBuffer, "POR");
        }
        else if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_EXTRST_Msk)
        {
            sprintf(m_pcStrBuffer, "EXTRST");
        }
        else if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_SYSRSTREQ_Msk)
        {
            sprintf(m_pcStrBuffer, "SYSRSTREQ");
        }
        else if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_LOOKUP_Msk)
        {
            sprintf(m_pcStrBuffer, "LOCKUP");
        }
        else if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_WATCHDOG_Msk)
        {
            sprintf(m_pcStrBuffer, "WATCHDOG");
        }
        else if(VOR_SYSCONFIG->RST_STAT & SYSCONFIG_RST_STAT_MEMERR_Msk)
        {
            sprintf(m_pcStrBuffer, "MEMERR");
        }
        UART0WriteStr(m_pcStrBuffer);

        sprintf(m_pcStrBuffer, "\r\n");
        UART0WriteStr(m_pcStrBuffer);
    
        /* Clear reset flag. */
        VOR_SYSCONFIG->RST_STAT = 0;
    }
    
    /* Print firmware version. */
    sprintf(m_pcStrBuffer, "-- $MSG: Firmware Version = %s\r\n", SOFTWARE_VERSION);
    UART0WriteStr(m_pcStrBuffer);
    
    /* Print CPU ID. */
    sprintf(m_pcStrBuffer, "-- $MSG: CPUID = 0x%08x\r\n" , VOR_SYSCONFIG->EF_ID);
    UART0WriteStr(m_pcStrBuffer);
}

/**
  * @brief  Prints all available data to UART
  * Blocking
  * Only called if data is available
  * @retval None
  */
void
PrintData(void)
{
    /* Only print data if it's ready. */
    if(IsDataAvailable() == false)
    {
        return;
    }
    
    /* Determine which mode is running. */
    switch(GetMode())
    {
        case RunModeEnum_Continuous:
        {
            PrintContinuousModeData();
            
            break;
        }
        
        case RunModeEnum_Burst:
        {
            PrintBurstModeData();
            
            break;
        }
        
        case RunModeEnum_SingleShot:
        {
            PrintBurstModeData();
            
            break;
        }
				
			  case RunModeEnum_ExtTrig:
        {
					  UART0WriteStr("External Trigger Mode\r\n") ;  
            PrintBurstModeData();
            
            break;
        }
        
        default:
        {
            UART0WriteStr("Mode print statement not yet implemented. ");
					            PrintBurstModeData();  
        }
    }
    
    /* Indicate that data has been printed, and sampling can resume. */
    MarkDataAsConsumed();
}

/*******************************************************************************
 **
 ** @brief 
 **
 ******************************************************************************/
void
PrintContinuousModeData(void)
{
    uint16_t *pui16Data;
    uint16_t ui16Datum;
    uint8_t ui8SequenceIdx, ui8ChannelIdx;
    char *pcContnuousStr;
    
    /* Get pointer to next free spot in buffer. */
    pcContnuousStr = m_pcContinuousStrBuffer + 6;

    /* Aquire data to print out. */
    ReadContinuousResults(&pui16Data, &ui8SequenceIdx);

    /* Append sequence index. */
    *pcContnuousStr++ = m_pcNibbleToAscii[ui8SequenceIdx >> 4];
    *pcContnuousStr++ = m_pcNibbleToAscii[ui8SequenceIdx & 0xF];
    *pcContnuousStr++ = ',';

    /* Convert all channel samples to ascii. */
    for(ui8ChannelIdx = 0; ui8ChannelIdx < 10; ui8ChannelIdx++)
    {
        /* Read datum from buffer. */
        ui16Datum = *pui16Data++;
        
        /* Convert datum to ascii. */
        *pcContnuousStr++ = m_pcNibbleToAscii[ui16Datum >> 12];
        *pcContnuousStr++ = m_pcNibbleToAscii[(ui16Datum >> 8) & 0xF];
        *pcContnuousStr++ = m_pcNibbleToAscii[(ui16Datum >> 4) & 0xF];
        *pcContnuousStr++ = m_pcNibbleToAscii[ui16Datum & 0xF];
        *pcContnuousStr++ = ',';
    }
    
    /* Remove last comma and append a new line. */
    pcContnuousStr--;
    *pcContnuousStr++ = '\r';
    *pcContnuousStr++ = '\n';
    *pcContnuousStr = NULL;

    /* Print data. */
    UART0WriteStr(m_pcContinuousStrBuffer);
}

/*******************************************************************************
 **
 ** @brief 
 **
 ******************************************************************************/
void
PrintBurstModeData(void)
{
    ADCParams_t *psADC0, *psADC1, *psADCMux;
    uint32_t ui32RawRTDSum, ui32RawVCCSum, ui32RTD, ui32VCC, ui32Time;
    char pcBurstStatsStrBuffer[64];
    
    /* Aquire data to print out. */
    ui32Time = GetTime();
    ReadBurstResults(&psADC0, &psADC1, &psADCMux, &ui32RawRTDSum,
                     &ui32RawVCCSum);
    
    /* Convert RTD oversampling to 0.1K. */
    ui32RTD = getTempK10_from_adcCodeSum(ui32RawRTDSum);
    
    /* Convert VCC oversampling to 0.001V. */
    ui32VCC = getVolt1000fromSum(ui32RawVCCSum);
    
    /* Print temperature, voltage, and time. */
    sprintf(pcBurstStatsStrBuffer,
            "$TEMP:%03d.%01dC,$VOLT=%01d.%03dV,$TIME=%010d\r\n",
            ((int32_t)ui32RTD - 2732) / 10, abs(((int32_t)ui32RTD - 2732)) % 10, ui32VCC / 1000,
            ui32VCC % 1000, ui32Time);
    UART0WriteStr(pcBurstStatsStrBuffer);
    
    /* Print samples for ADC0 and ADC1. */
    PrintADC01Samples(psADC0);
    PrintADC01Samples(psADC1);
    
    /* Print samples for ADC Mux. */
    PrintADCMuxSamples(psADCMux);
}


/**
  * @brief  composes a $BURST: message for either channel 0/1 and UARTs out
  *         custom, not sprintf, for speed
  * @param  channel: channel to be reported.  Only used to generate the label in
  *         the report, not index the data.
  * @param  uint16_t *mybufPtr: pointer to the start of the channel buffer
  *         (buf0/1 are not interleaved).  NOT the entire buffer space.
  * @retval None
  */
/*******************************************************************************
 **
 ** @brief  Prints samples collected from either ADC channel 0 or 1.
 **
 ******************************************************************************/
void
PrintADC01Samples(ADCParams_t *psADC01)
{
    char *pcBuffer;
    uint16_t *pui16Data;
    uint16_t ui16Datum;
    uint32_t ui32Size;
    
    /* Only print if samples were taken. */
    if(psADC01->ui32Size == 0)
    {
        return;
    }
    
    /* Get pointer to data. */
    pui16Data = psADC01->pui16BufferHead;
    
    /* Print header. */
    UART0WriteStr("$BURST:");
    
    /* Buffer <Channel Num>, */
    pcBuffer = m_pcStrBuffer;
    *pcBuffer++ = m_pcNibbleToAscii[psADC01->ui32ChannelMask - 1];
    *pcBuffer++ = ',';

    /* Buffer <ADC01 Sample Period>, */
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Period << 16) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Period << 20) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Period << 24) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[ psADC01->ui32Period & 0xF];
    *pcBuffer++ = ',';
    
    /* Buffer <ADC01 Sample Size>,*/
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Size << 16) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Size << 20) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[(psADC01->ui32Size << 24) >> 28];
    *pcBuffer++ = m_pcNibbleToAscii[ psADC01->ui32Size & 0xF];
    *pcBuffer++ = ',';

    /* Complete and print buffered string. */ 
    *pcBuffer++ = 0;
    UART0WriteStr(m_pcStrBuffer);
    
    /* Converts and buffers aquired sample data to ascii hex. */
    pcBuffer = m_pcStrBuffer;
    ui32Size = psADC01->ui32Size;
    while(ui32Size--)
    {
        /* Complete and print buffered string. */ 
        *pcBuffer++ = 0;
        UART0WriteStr(m_pcStrBuffer);
        
        /* Pull sample from buffer. */
        ui16Datum = *pui16Data++;
        
        /* Converts sample to ascii hex. */
        pcBuffer = m_pcStrBuffer;
        *pcBuffer++ = m_pcNibbleToAscii[ ui16Datum >> 12];
        *pcBuffer++ = m_pcNibbleToAscii[(ui16Datum >> 8) & 0xF];
        *pcBuffer++ = m_pcNibbleToAscii[(ui16Datum >> 4) & 0xF];
        *pcBuffer++ = m_pcNibbleToAscii[ ui16Datum & 0xF];
        *pcBuffer++ = ',';
    }
    
    /* Replace last comma with a new line. */
    pcBuffer--;
    *pcBuffer++ = '\r';
    *pcBuffer++ = '\n';

    /* Complete and print buffered string. */ 
    *pcBuffer++ = 0;
    UART0WriteStr(m_pcStrBuffer);
}

/**
  * @brief  composes a $BURST: message for all active mux channels and UARTs out
  * custom, not sprintf, for speed
  * blocking
  * @retval None
  */
/*******************************************************************************
 **
 ** @brief  Print ADC Mux Samples - Ch 2-9 have unique output format. 
 **        data is interleaved.  ie. ch2, ch3, ch9, ch2, ch3, ch9.....
 **
 ******************************************************************************/
void
PrintADCMuxSamples(ADCParams_t *psADCMux)
{
    char *pcBuffer;
    uint16_t *pui16Data;
    uint16_t ui16Datum;
    uint32_t ui32Size, ui32ChannelMask, ui32ChannelNum, ui32ChannelCtr;
    uint32_t ui32ChannelIdx, ui32DatumIdx;
    
    /* Only print if samples were taken. */
    if(psADCMux->ui32Size == 0)
    {
        return;
    }
    
    /* Pre-shifted channel mask. */
    ui32ChannelMask = psADCMux->ui32ChannelMask;
    
    /* Get pointer to data. */
    pui16Data = psADCMux->pui16BufferHead;
    
    /* Get size of each buffer chunk. */
    ui32Size = psADCMux->ui32Size;
    
    /* Determine how many channels were sampled. */
    ui32ChannelNum = 0;
    for(ui32ChannelIdx = 0; ui32ChannelIdx < 8; ui32ChannelIdx++)
    {
        if(((1 << ui32ChannelIdx) & ui32ChannelMask) != 0)
        {
            ui32ChannelNum++;
        }
    }
    
    /* Iterates through each channel sampled, converting and printing sampled
       data.*/
    ui32ChannelCtr = 0;
    for(ui32ChannelIdx = 0; ui32ChannelIdx < 8; ui32ChannelIdx++)
    {
        /* Determine if channel was sampled. */
        if(((1 << ui32ChannelIdx) & ui32ChannelMask) != 0)
        {
            /* Print header. */
            UART0WriteStr("$BURST:");
            
            /* Print <Channel Num>, */
            pcBuffer = m_pcStrBuffer;
            *pcBuffer++ = m_pcNibbleToAscii[ui32ChannelIdx + 2];
            *pcBuffer++ = ',';
            
            /* Prints <ADCMux Sample Period>, */
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Period << 16) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Period << 20) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Period << 24) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[ psADCMux->ui32Period & 0xF];
            *pcBuffer++ = ',';
            
            /* Prints <ADCMux Sample Size>, */
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Size << 16) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Size << 20) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[(psADCMux->ui32Size << 24) >> 28];
            *pcBuffer++ = m_pcNibbleToAscii[ psADCMux->ui32Size & 0xF];
            *pcBuffer++ = ',';
            
            /* Complete and print buffered string. */ 
            *pcBuffer++ = 0;
            UART0WriteStr(m_pcStrBuffer);
            
            /* Print each piece of data. */
            pcBuffer = m_pcStrBuffer;
            for(ui32DatumIdx = 0; ui32DatumIdx < ui32Size; ui32DatumIdx++)
            {
                /* Complete and print buffered string. */ 
                *pcBuffer++ = 0;
                UART0WriteStr(m_pcStrBuffer);
                
                /* Pull sample from buffer. */
                ui16Datum = pui16Data[ui32ChannelCtr + (ui32DatumIdx *
                                                        ui32ChannelNum)];
                
                /* Converts sample to ascii hex. */
                pcBuffer = m_pcStrBuffer;
                *pcBuffer++ = m_pcNibbleToAscii[ ui16Datum >> 12];
                *pcBuffer++ = m_pcNibbleToAscii[(ui16Datum >> 8) & 0xF];
                *pcBuffer++ = m_pcNibbleToAscii[(ui16Datum >> 4) & 0xF];
                *pcBuffer++ = m_pcNibbleToAscii[ ui16Datum & 0xF];
                *pcBuffer++ = ',';
            } 
            
            /* Replace last comma with a new line. */
            pcBuffer--;
            *pcBuffer++ = '\r';
            *pcBuffer++ = '\n';

            /* Complete and print buffered string. */ 
            *pcBuffer++ = 0;
            UART0WriteStr(m_pcStrBuffer);
            
            /* Increment channel counter. */
            ui32ChannelCtr++;
        }
    }
}

/*******************************************************************************
 **
 ** @brief Converts sum of RTC ADC samples to Kelvin.
 **
 ** Converts the temperature from a sum of raw RTC ADC values to a Kelvin
 ** temperature with 0.1K precision, with a range of -40C to 240C.  The
 ** calculation uses linear interpolation in 40C increments.
 **
 ** @param ui32Sample is the sum of RTC ADC samples, where the number of samples
 **                   is specified with configRTD_SAMPLES.
 **
 ** @return Returns the converted Kelvin temperature, with a decimal precision
 **         of 0.1K. Returns -1 if the sample sum is greater than the max
 **         temperature, or -2 if the sample sum is less than the min
 **         temperature.
 **
 ******************************************************************************/
int32_t
getTempK10_from_adcCodeSum(uint32_t ui32Sample)
{
  uint32_t ui32Temp;
  uint32_t ui32Delta;
  uint32_t ui32SLOPE;
	
    /* No need to average ui32Sample.  Number of samples taken care of with 
       preprocessor macros. */
    
    /* Verify samples are less than the max temperature. */
	if(ui32Sample > RTD_ADC_CODE_SUM_240C)
  {
    return(RTD_ERRVAL_LOW);
  }
    /* Verify samples are greater than the min temperature. */
	if(ui32Sample < RTD_ADC_CODE_SUM_N40C)
  {
    return(RTD_ERRVAL_HIGH);
  }
    
    /* Assign proper offsets and slopes for interpolating the temperature. */
	if(ui32Sample < RTD_ADC_CODE_SUM_0C)
  {
		ui32Temp = -400 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_N40C;
		ui32SLOPE = RTD_SLOPE_N40_TO_0C;
	}
	else if(ui32Sample < RTD_ADC_CODE_SUM_40C)
  {
		ui32Temp = 0 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_0C;
		ui32SLOPE = RTD_SLOPE_0_TO_40C;
	}
	else if(ui32Sample < RTD_ADC_CODE_SUM_80C)
  {
		ui32Temp = 400 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_40C;
		ui32SLOPE = RTD_SLOPE_40_TO_80C;
	}
	else if(ui32Sample < RTD_ADC_CODE_SUM_120C)
  {
		ui32Temp = 800 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_80C;
		ui32SLOPE = RTD_SLOPE_80_TO_120C;
	}
	else if(ui32Sample < RTD_ADC_CODE_SUM_160C)
  {	  
		ui32Temp = 1200 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_120C;
		ui32SLOPE = RTD_SLOPE_120_TO_160C;
	}
	else if(ui32Sample < RTD_ADC_CODE_SUM_200C)
  {
		ui32Temp = 1600 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_160C;
		ui32SLOPE = RTD_SLOPE_160_TO_200C;
	}
	else
  {
		ui32Temp = 2000 + 2732;
		ui32Delta = ui32Sample - RTD_ADC_CODE_SUM_200C;
		ui32SLOPE = RTD_SLOPE_200_TO_240C;
	}
	
    /* Perform linear interpolation. */
	ui32Temp += (ui32Delta * ui32SLOPE) >> 16;
    
	return ui32Temp;
}

/*******************************************************************************
 **
 ** @brief  Covnerts sum of VCC ADC samples to Volts.
 **
 ** Converts the voltage from a sum of raw VCC ADC samples to a voltage with
 ** 0.001v precision.
 **
 ** @param ui32Sample is the sum of VCC ADC samples, where the number of samples
 **                   is specified with configVCC_SAMPLES
 **
 ** @return Returns the converted Kelvin temperature, with a decimal precision
 **         of 0.001v.
 **
 ******************************************************************************/
uint32_t
getVolt1000fromSum(uint32_t ui32Sample)
{
  return ((ui32Sample*VCC_TH_MULT_LSHIFT16-VCC_TH_COMP_LSHIFT16)>>16);
}
