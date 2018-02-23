/***************************************************************************************
 * @file     sampling_engine.c
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
#include "sampling_engine.h"

//Conservative limits from measured performance, may change upon recompiling
/* Must keep large enough to let the mux settle */
#define CONTINUOUS_ADC01_PERIOD_100NS   (100) 
#define CONTINUOUS_ANAMUX_PERIOD_100NS  (150) 

/* Minimum time required for ADC CONV signal to be asserted. */         
#define MIN_CONV_TIME_NS            (1200)

/* Time required to switch from a mux channel to the RTD-only sampling. */
#define RTD_MUXSWITCH_PERIOD_100NS  (80)

/* Time required to sample RTD channel. */
#define RTD_SAMP_PERIOD_100NS       (40)


/*  */
#define	AD7981_MIN_CONV_CLKS    ((((unsigned long long int)configSYSCLK) *    \
                                  ((unsigned long long int)                   \
                                   MIN_CONV_TIME_NS)) / 1000000000UL)
                                         
#define RTD_MUXSWITCH_PERIOD_CLOCKS ((((unsigned long long int)configSYSCLK) *\
                                      ((unsigned long long int)               \
                                       RTD_MUXSWITCH_PERIOD_100NS)) /         \
                                     10000000UL)
	
#define RTD_SAMP_PERIOD_CLOCKS ((((unsigned long long int)configSYSCLK) *     \
                                  ((unsigned long long int)                    \
                                   RTD_SAMP_PERIOD_100NS)) / 10000000UL)
                                         

/* Width of analog mux attached to ADC2. */
#define ADC2_MUX_WIDTH      (3)
#define ADC2_MUX_Msk        (7)

/* Macro to preshift mux channel selection. */
#define PRESHIFT_MUX_CHANNEL(X)     ((X & ADC2_MUX_Msk) << ADC2_MUX0_PIN)

/* ADC Mux special channels. */
#define ANAMUX_CH_VREF_DIV3         PRESHIFT_MUX_CHANNEL(6)
#define ANAMUX_CH_RTD               PRESHIFT_MUX_CHANNEL(7)

																
volatile const uint32_t ui32Dbg1 = ANAMUX_CH_VREF_DIV3;
volatile const uint32_t ui32Dbg2 = ANAMUX_CH_RTD;



#ifdef USE_RTOS
static bool bRTOSSuspended = false;
static bool bRTOSDisableNeeded = false ;  
#endif



/* Statically allocated buffer for all sampling modes. */
static uint16_t m_pui16Buffer[ADC_BUFFER_SIZE];

/* Structures for accessing ADC parameters. */
static ADCParams_t m_sADC0Params;
static ADCParams_t m_sADC1Params;
static ADCParams_t m_sADCMuxParams;

/* Function pointer for sampling ADC0 and ADC1. */
static void (*pfISR_SampleADC01)(void);

/* Function pointer for sampling ADC2 Mux. */
static void (*pfISR_SampleADC2Mux)(void);
static void ConfigureTrigger(void)  ;

/* */
static bool m_bDataAvailable;

/* The ADC channels being actively sampled. */
static uint32_t m_ui32ChannelMask;
    
/* */
static uint32_t m_ui32ADC01SamplePeriodClks;

/* */
static uint32_t m_ui32ADCMuxSamplePeriodClks;

/* */
static uint32_t m_ui32ADC01NumSamples;

/* */
static uint32_t m_ui32ADCMuxChannelNumSamples;

/* */
static uint32_t m_ui32ContinuousSamplePeriod;

/* */
static bool m_bOnlyOversample;

                 
static volatile uint32_t burstMode_timeStamp = 0xffffffff; //!< burstMode_timeStamp for $continuous mode
static volatile uint32_t burstMode_expPeriodSeconds = 2UL;   //!< $burst period, defaults to 2 sec
static volatile uint32_t burstMode_expPeriodSecondsDownCounter = 0; //!< Downcounter decremented by seconds ISR, triggers experiment cascade when 0

//contMode_sampPeriod_clks

/* The status of the sampling engine. */
static volatile enum StatusEnum m_iStatus;

/* The mode of operation that the sampling engine is set to. */
static volatile enum RunModeEnum m_iRunMode;

static uint32_t adc01Mode_sampDownCounter = 0;     //!< decrementing sample counter 

static struct muxModeStatStruct muxModeStat;
static struct burstModeStruct burstModeConfig;

         
static volatile uint32_t m_ui32Seconds = 0;

static volatile uint32_t lastRTDCode = 0xffffffff;
         
static uint32_t nextMuxChPins;             //precalc of next port val, pre-shifted.  Must be masked

static uint32_t spiCount = 0;

/* */
static uint8_t m_ui8SequenceIdx = 0;

/*******************************************************************************
 **
 ** @brief  Setup timers to generate pulse according to AD7981 datasheet 
 **
 ******************************************************************************/
void
SamplingEngineInit(void)
{
    /* Configure all ADC CONV signals to have a minimum on time. */
    VOR_TIM_ADC0_CONV.PWMA_VALUE = AD7981_MIN_CONV_CLKS; 
    VOR_TIM_ADC0_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    
    VOR_TIM_ADC1_CONV.PWMA_VALUE = AD7981_MIN_CONV_CLKS;
    VOR_TIM_ADC1_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    
    VOR_TIM_ADC2_CONV.PWMA_VALUE = AD7981_MIN_CONV_CLKS; 
    VOR_TIM_ADC2_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    
    /* Timers start disabled. */
    VOR_TIM_ADC0_CONV.ENABLE = 0;
    VOR_TIM_ADC1_CONV.ENABLE = 0;
    VOR_TIM_ADC2_CONV.ENABLE = 0;
    
    /* Initialize function pointers. */
    pfISR_SampleADC01 = &ISR_SampleADC01;
    pfISR_SampleADC2Mux = &ISR_SampleADCMux_SampleRequestedChannels;
    
    /* Initialize internal mode and status. */
    m_iRunMode = RunModeEnum_None;
    m_iStatus = StatusEnum_Idle;
 
    /* Initialize internal ADC parameters. */
    m_sADC0Params.ui32ChannelMask = (1 << 0);
    m_sADC1Params.ui32ChannelMask = (1 << 1);
    
    /* Initlaize flags. */
    m_bDataAvailable = false;
    m_bOnlyOversample = false;
}

/*******************************************************************************
 **
** @brief  Separate memory pointers are setup for: ADC0, ADC1 and ADC2 
 **
 ******************************************************************************/
bool
ConfigureMemory(void)
{
    uint32_t ui32ChannelMask, ui32MemReq;
    uint16_t *pui16Buffer;
    uint8_t ui8ChannelIdx;
    
    /* Reset memory counter. */
    ui32MemReq = 0;
    
    /* Acquire pointer to global buffer. */
    pui16Buffer = m_pui16Buffer;
    
    /* make local copy of channel mask. */
    ui32ChannelMask = m_ui32ChannelMask;
    
    /* Reset all buffer pointers. */
    m_sADC0Params.pui16Buffer = 0;
    m_sADC0Params.pui16BufferHead = 0;
    
    m_sADC1Params.pui16Buffer = 0;
    m_sADC1Params.pui16BufferHead = 0;
    
    m_sADCMuxParams.pui16Buffer = 0;
    m_sADCMuxParams.pui16BufferHead = 0;
    
    /* Clear all sizes. */
    m_sADC0Params.ui32Size = 0;
    m_sADC1Params.ui32Size = 0;
    m_sADCMuxParams.ui32Size = 0;
    
    if(m_bOnlyOversample == false)
    {
        /* Allocate buffer space for ADC0 samples. */
        if((ui32ChannelMask & (1 << 0)))
        {
            m_sADC0Params.pui16Buffer = pui16Buffer;
            m_sADC0Params.pui16BufferHead = pui16Buffer;
            m_sADC0Params.ui32Size = m_ui32ADC01NumSamples;
            
            pui16Buffer += m_ui32ADC01NumSamples;
            ui32MemReq += m_ui32ADC01NumSamples;
        }
        
        /* Allocate buffer space for ADC1 samples. */
        if((ui32ChannelMask & (1 << 1)))
        {
            m_sADC1Params.pui16Buffer = pui16Buffer;
            m_sADC1Params.pui16BufferHead = pui16Buffer;
            m_sADC1Params.ui32Size = m_ui32ADC01NumSamples;
            
            pui16Buffer += m_ui32ADC01NumSamples;   
            ui32MemReq += m_ui32ADC01NumSamples;     
        }
        /* Allocate buffer space for ADC Mux channels. */
        m_sADCMuxParams.pui16Buffer = pui16Buffer;
        m_sADCMuxParams.pui16BufferHead = pui16Buffer;
        
        m_sADCMuxParams.ui32Size = m_ui32ADCMuxChannelNumSamples;
        
        /* Configure ADC Mux sampling settings. */
        muxModeStat.chCount = 0;
        
        /* Only configure Mux if oversampling is disabled. */
        ui32ChannelMask = m_sADCMuxParams.ui32ChannelMask;
        
        /* Convert bitmask of ADC Mux channels into array of channels to sample. */
        for(ui8ChannelIdx = 0; ui8ChannelIdx < 8; ui8ChannelIdx++)
        {
            if((ui32ChannelMask & (1 << ui8ChannelIdx)) != 0)
            {   
                /* Store channel to sample, preshifting pins for ISR performance. */
                muxModeStat.muxChList[muxModeStat.chCount++] =
                    PRESHIFT_MUX_CHANNEL(ui8ChannelIdx);
                    
                ui32MemReq += m_ui32ADCMuxChannelNumSamples;
            }
        }        
        
        /* Wrap entries to prevent needing to branch in ISRs. */
        muxModeStat.muxChList[muxModeStat.chCount] = muxModeStat.muxChList[0];
        muxModeStat.muxChList[muxModeStat.chCount + 1] = muxModeStat.muxChList[1];
    }
    else
    {
        muxModeStat.muxChList[0] = ANAMUX_CH_RTD;
        muxModeStat.muxChList[1] = ANAMUX_CH_RTD;
    }
    
    /* Reset ADC Mux counters. */
    muxModeStat.chDoneCount = 0;
    muxModeStat.sampCounter = 0;
    muxModeStat.sampDownCounter = muxModeStat.chCount * m_sADCMuxParams.ui32Size;
    
    /* Make sure all samples will fit inside of global buffer. */
    return(ui32MemReq <= sizeof(m_pui16Buffer));
}

/*******************************************************************************
 **
 ** @brief  ADC are triggered by timer output. SPI is used to retrieve data
 **
 ******************************************************************************/
void
ConfigurePeripherals(void)
{   
    adc01Mode_sampDownCounter = m_ui32ADC01NumSamples - 1;
    spiCount = 0;
    
    /* Configure all timers to generate CONV signals when VOR_TIM_TRIG 
       completes, */
    VOR_TIM_ADC0_CONV.CASCADE0 = (TIM_CAS_SRC_TIM_0);  
    VOR_TIM_ADC1_CONV.CASCADE0 = (TIM_CAS_SRC_TIM_0);
    VOR_TIM_ADC2_CONV.CASCADE0 = (TIM_CAS_SRC_TIM_0);
    
    /* Reset cascade control to timers. */
    VOR_TIM_ADC0_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
                                  
    VOR_TIM_ADC1_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
                                  
    VOR_TIM_ADC2_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
    
    /* ...but timers default to being disabled. */
    VOR_TIM_ADC0_CONV.ENABLE = 0;
    VOR_TIM_ADC1_CONV.ENABLE = 0;
    VOR_TIM_ADC2_CONV.ENABLE = 0;
    
    /* Disable ADC01 SPI clocking. */
    VOR_SPI_ADC0.CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
    VOR_SPI_ADC1.CTRL1 &= ~SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
    
    /* Disable interrupt generation for ADC0. */

    VOR_TIM_ADC0_CONV.CTRL &= ~(TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
    
    /* Set ADC0 sample period. */
    VOR_TIM_ADC0_CONV.RST_VALUE = m_ui32ADC01SamplePeriodClks;  
    VOR_TIM_ADC0_CONV.CNT_VALUE = m_ui32ADC01SamplePeriodClks + 1;
    
    /* Set ADC1 sample period. */
    VOR_TIM_ADC1_CONV.RST_VALUE = m_ui32ADC01SamplePeriodClks;
    VOR_TIM_ADC1_CONV.CNT_VALUE = m_ui32ADC01SamplePeriodClks + 1;
        
    /* Set ADC2 CONV signal low. */
    VOR_TIM_ADC2_CONV.RST_VALUE = m_ui32ADCMuxSamplePeriodClks;  
    VOR_TIM_ADC2_CONV.CNT_VALUE = m_ui32ADCMuxSamplePeriodClks + 1;
    
    /* Enable ADC0 sampling. */
    if((m_ui32ChannelMask & (1 << 0)) && (m_bOnlyOversample == false))
    {
        VOR_SPI_ADC0.CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
        VOR_TIM_ADC0_CONV.ENABLE = 1;
        
        /* Re-enable ADC0 interrupt. */
        VOR_TIM_ADC0_CONV.CTRL |= (TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
    }
        
    /* Enable ADC1 sampling. */
    if((m_ui32ChannelMask & (1 << 1)) && (m_bOnlyOversample == false))
    {
        VOR_SPI_ADC1.CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
        VOR_TIM_ADC1_CONV.ENABLE = 1;
    }

    /* Configure peripherals for ADC2. */
    if(((m_ui32ChannelMask & 0x003) != 0) && (m_bOnlyOversample == false))
    {
        /* ADC Mux will not be sampled until ADC01 samples are acquired. */
        
        /* Switch shared SPI pins to ADC0. */
        _SetSPI1ToADC0();
        
        /* Disable cascade for VOR_TIM_ADC2, since ADC Mux will not be sampled
           until all samples from ADC0 and ADC1 are acquired. */
        VOR_TIM_ADC2_CONV.CSD_CTRL &= ~(TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                        TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
                                        
        /* Enable interrupt for VOR_TIM_ADC2, and make sure timer operates
           continuously. */
        VOR_TIM_ADC2_CONV.CTRL |= TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk;
        VOR_TIM_ADC2_CONV.CTRL &= ~TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk;
    }
    else
    {
        /* Only ADC Mux channels will be sampled. */
        
        /* Switch shared SPI pins to ADC0. */
        _SetSPI1ToADC2();
        
        /* Set current and next mux channels to sample. */
        _SetADCMuxChannel(muxModeStat.muxChList[0]);
        nextMuxChPins = muxModeStat.muxChList[1];
        
        /* Disable cascading for ADC0 and ADC1 timers. */
        VOR_TIM_ADC0_CONV.CSD_CTRL &= ~(TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                        TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
        VOR_TIM_ADC1_CONV.CSD_CTRL &= ~(TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                        TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
                                    
        /* Enable cascade for ADC2. */
        VOR_TIM_ADC2_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk | 
                                      TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
        
        /* Enable interrupt for ADC2 Timer. */
        VOR_TIM_ADC2_CONV.CTRL |= (TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
        
        /* Enable ADC2 sampling. */
        VOR_SPI_ADC2.CTRL1 |= SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
        VOR_TIM_ADC2_CONV.ENABLE = 1;
    }
    
    /* Enable ADC0 Timer interrupt to be services. */
    NVIC_EnableIRQ(VOR_IRQ_CNV_ADC01);
}

/*******************************************************************************
 **
 ** @brief  Reset valiables needed for sampling ADC2 Mux channels.
 **
 ******************************************************************************/
void
ConfigureMuxSampling(void)
{
    /* Reset MUX sampling counters. */
	muxModeStat.chDoneCount = 0;
	muxModeStat.sampCounter = 0;
	muxModeStat.sampDownCounter = muxModeStat.chCount * m_sADCMuxParams.ui32Size;
    
    /* Pre-select the first mux channel to sample. */
	_SetADCMuxChannel(muxModeStat.muxChList[0]);
	
    /* Initialize the ADC Mux sampling state machine. */
    if(muxModeStat.sampDownCounter >= 2)
    {
        /* State machine will start normally. */
        
        /* State machine starts by sampling all user requested channels, */
		pfISR_SampleADC2Mux = &ISR_SampleADCMux_SampleRequestedChannels;
        
        /* ...with user-requested sampling period. */
		VOR_TIM_ADC2_CONV.RST_VALUE = m_ui32ADCMuxSamplePeriodClks;
    }
    else if(muxModeStat.sampDownCounter == 1)
    {
        /**/
		VOR_TIM_ADC2_CONV.RST_VALUE = RTD_MUXSWITCH_PERIOD_CLOCKS;
        
        /* Set mux sampling state machine to sample requested channels. */
		pfISR_SampleADC2Mux = &ISR_SampleADCMux_TransitionToRTD;	
    }
    else
    {   
        /* Select RTD channel. */
        _SetADCMuxChannel(ANAMUX_CH_RTD);
        
        /* Set sample time for RTD. */
		VOR_TIM_ADC2_CONV.RST_VALUE = RTD_SAMP_PERIOD_CLOCKS;
        
        /* Set the number of sampled needed. */
		muxModeStat.sampDownCounter = configRTD_SAMPLES;	
        
        /* Jump immediately to RTD sampling section of Mux state machine, once
           active. */
		pfISR_SampleADC2Mux = &ISR_SampleADCMux_RTDOnly;
    }
    
    /* Set ADC2 CONV pin low. */
	VOR_TIM_ADC2_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    
    /* Set the next mux channel that will be sampled. */
	nextMuxChPins = muxModeStat.muxChList[1];
}
 
/*******************************************************************************
 **
 ** @brief
 **
 ******************************************************************************/
void
ConfigureTrigger(void)
{
    uint32_t ui32SamplePeriodClks;
    
    /* Disable timer. */
    VOR_TIM_TRIG.ENABLE = 0;
    
    /* Configure trigger timer based on set run mode. */
    if((m_iRunMode == RunModeEnum_SingleShot) ||
       (m_iRunMode == RunModeEnum_Burst))
    {
        /* Timer is manually started with the seconds ISR. */
        VOR_TIM_TRIG.CTRL = (TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk |
                             TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
        VOR_TIM_TRIG.RST_VALUE = 0;  
        VOR_TIM_TRIG.CNT_VALUE = 0;  		
    }
    else if(m_iRunMode == RunModeEnum_Continuous)
    {
        /* Calculate number of clock cycles per continuous sample period. */
        ui32SamplePeriodClks =  ((unsigned long long int)configSYSCLK) * 
                                 ((unsigned long long int)
                                  (m_ui32ContinuousSamplePeriod * 10)) /
                                 10000000UL;
                                
        /* Configure timer as periodic counter. */
        VOR_TIM_TRIG.CTRL &= ~(TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk);
        VOR_TIM_TRIG.CTRL = (TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
        
        /* First sample is taken immediately. */
        VOR_TIM_TRIG.RST_VALUE = ui32SamplePeriodClks;
        VOR_TIM_TRIG.CNT_VALUE = 1;
    }
    else if(m_iRunMode == RunModeEnum_ExtTrig)
    {
						    /* Configure External Trigger pin. */
      VOR_PORTB.DIR &= ~(1UL << 14);
			VOR_PORTB.DIR |= ((7UL << 20) | (1UL << 23));
			VOR_PORTB.IRQ_SEN &= ~(1UL << 14);  /*   select edge sensitivity  */
			VOR_PORTB.IRQ_EDGE &= ~(1UL << 14); /*   use IRQ_EVT  */
			VOR_PORTB.IRQ_EVT |= (1UL << 14);   /*   event on L to H  */
			VOR_PORTB.IRQ_ENB |= (1UL << 14);   /*   enable int */
			/*    setup TIM_TRIG (TIM0) to start with Pin Interrupt on PB[14]  */
			 /* Timer is started when rising edge detected on PORTB[14] . */
        VOR_TIM_TRIG.CTRL = (TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk |
                             TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
        VOR_TIM_TRIG.RST_VALUE = 0;  
        VOR_TIM_TRIG.CNT_VALUE = 1;
       /* Configure Timer 0 to cascade on Ext Trig pin rising edge. */
        VOR_TIM_TRIG.CASCADE0 = TIM_CAS_SRC_TRIG_PIN;
        VOR_TIM_TRIG.CSD_CTRL = TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                            TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk;
                        
       /* Enable external trigger control. */
        VOR_TIM_TRIG.ENABLE = 1;    /*  re-enable Timer for triggering timers for ADC0/1   */			
    }
    
    /* Enable ADC01 interrupt. */
    NVIC_EnableIRQ(VOR_IRQ_CNV_ADC01);
}

/*******************************************************************************
 **
 ** @brief   Sets shared variable m_iRunMode to mode provided with call
 **
 ******************************************************************************/
bool
SetMode(enum RunModeEnum iRunMode)
{
    /* Verify that no sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }

    /* Set new mode. */
    m_iRunMode = iRunMode;
    
    return(true);
}

/*******************************************************************************
 **
 ** @brief - returns shared variable m_iRunMode
 **
 ******************************************************************************/
enum RunModeEnum
GetMode(void)
{
    return(m_iRunMode);
}

/*******************************************************************************
 **
 ** @brief  Sets the aquisition parameters for all sampling modes.
 **
 ** @param  ui32Channelmask is the bit mask describing which channels to sample.
 **
 ** @param  ui32ADC01SamplePeriod is the multiple of 100ns in which to wait in 
 **                               between consecutive samples for ADC0 and ADC1.
 **
 ** @param  ui32ADC01NumSamples is the number of samples to take for ADC0 and
 **                             ADC1.
 **
 ** @param  ui32ADCMuxSamplePeriod is the multiple of 100ns in which to wait in
 **                                between consecutive samples for each ADC Mux
 **                                channel.
 **
 ** @param  ui32ADCMuxNumSamples is the number of samples to take for each
 **                              channel on the ADC Mux.
 **
 ******************************************************************************/
bool
SetAcquisitionParams(uint32_t ui32ChannelMask, uint32_t ui32ADC01SamplePeriod,
                     uint32_t ui32ADC01NumSamples,
                     uint32_t ui32ADCMuxSamplePeriod,
                     uint32_t ui32ADCMuxNumSamples)
{
    /* Verify that no sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    
    /* Calculate number of clock cycles needed to achieve requested period. */
    m_ui32ADC01SamplePeriodClks =
                            (((unsigned long long int)configSYSCLK) *
                             ((unsigned long long int)ui32ADC01SamplePeriod)) / 
                            10000000UL;
    
    m_ui32ADCMuxSamplePeriodClks =
                            (((unsigned long long int)configSYSCLK) *
                             ((unsigned long long int)ui32ADCMuxSamplePeriod)) /
                            10000000UL;
    
    /* Store parameters for later use. */
    m_ui32ChannelMask = ui32ChannelMask;
    
    m_ui32ADC01NumSamples = ui32ADC01NumSamples;
    m_ui32ADCMuxChannelNumSamples = ui32ADCMuxNumSamples;
    m_sADCMuxParams.ui32Size = ui32ADCMuxNumSamples;
    
    m_sADC0Params.ui32Period = ui32ADC01SamplePeriod;
    m_sADC1Params.ui32Period = ui32ADC01SamplePeriod;
    
    m_sADCMuxParams.ui32Period = ui32ADCMuxSamplePeriod;
    m_sADCMuxParams.ui32ChannelMask = (ui32ChannelMask >> 2);
    
    return(true);
}

 
/*******************************************************************************
 **
 ** @brief  Set Continuous Mode Sample Period
 **
 ** @param  ui32SamplePeriod is the time in between consecutive burtsts of 
 **                           samples from all channels, in 100ns units.
 **
 ******************************************************************************/
bool
SetContinuousModeSamplePeriod(uint32_t ui32SamplePeriod)
{
    /* Verify that no sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    
    m_ui32ContinuousSamplePeriod = ui32SamplePeriod;
    
    return(true);
}

/*******************************************************************************
 **
 ** @brief  Configures sampling of all channels to start once the external
 **         trigger is asserted.
 **
 ******************************************************************************/
  #if 0  
void
StartExtTrigMode(void)
{
    /* Clear all SPI FIFOs. */
    VOR_SPI_ADC0.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk | 
                             SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
    VOR_SPI_ADC1.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                             SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
    
    /* Disable timer and reset cascade config. */
    VOR_TIM_TRIG.ENABLE = 0;
    VOR_TIM_TRIG.CSD_CTRL = 0;
    
    /* Configure ADC0 and ADC1 convert signals to start as soon as Timer 0 is 
     * done. */
    VOR_TIM_ADC0_CONV.CASCADE0 = (TIM_CAS_SRC_TIM_0);  
    VOR_TIM_ADC1_CONV.CASCADE0 = (TIM_CAS_SRC_TIM_0);
    
    /* Set internal run mode to External Trigger. */
    m_iRunMode = RunModeEnum_ExtTrig;
    
    /* Reset all ADC convert signal timers. */
    VOR_TIM_ADC0_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    VOR_TIM_ADC1_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    VOR_TIM_ADC2_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
    
    /* Configure Timer 0 to cascade on Ext Trig pin rising edge. */
    VOR_TIM_TRIG.CASCADE0 = TIM_CAS_SRC_TRIG_PIN;
    VOR_TIM_TRIG.CSD_CTRL = TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                            TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk;
                        
    /* Enable external trigger control. */
   VOR_TIM_TRIG.ENABLE = 1;    
     
}
  #endif   

/*******************************************************************************
 **
 ** @brief  Sets the number of seconds in between consecutive samples while in
 **         burst mode.
 **
 ** @param  ui32Seconds is the number of seconds to wait in between consecutive
 **                     samples while in burst mode.
 **
 ******************************************************************************/
bool
SetBurstModeSamplePeriod(uint32_t ui32Seconds)
{ 
    /* Verify that no sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    if(ui32Seconds == 0)   /*  Switch to external trigger mode if 0   */
		{
 			SetMode(RunModeEnum_ExtTrig)  ;  
		}
		else
		{
      burstMode_expPeriodSeconds = ui32Seconds;
    }
    return(true);
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
bool
OnlyOversample(void)
{
    /* Indicate an error if sampling is in progress, */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    
    /* ...or if the single shot mode isn't already selected. */
    if(m_iRunMode != RunModeEnum_SingleShot)
    {
        return(false);
    }
    
    /* Indicate that the only measurements that should be made are of the RTD
       and VCC averages. */
    m_bOnlyOversample = true;
    
    return(true);
}


/*******************************************************************************
 **
 ** @brief  -  Starts the sampling engine.  
 **      Exceptions are made for each mode to properly setup channels and rates
 **
 ******************************************************************************/
bool
Start(void)
{
    /* Indicate an error if sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    
    /* Clear all SPI FIFOs. */
    VOR_SPI_ADC0.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk | 
                             SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
    VOR_SPI_ADC1.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                             SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
    
    /* Disable trigger timer and reset cascade config. */
    VOR_TIM_TRIG.ENABLE = 0;
    VOR_TIM_TRIG.CSD_CTRL = 0;
    
    /* Perform additional configuration based on the mode.*/
    switch(m_iRunMode)
    {   
        case RunModeEnum_Continuous:
        {  
            /* All channels are sampled, with one sample per channel. */
            SetAcquisitionParams(0x3FF, CONTINUOUS_ADC01_PERIOD_100NS, 1,
                                 CONTINUOUS_ANAMUX_PERIOD_100NS, 1);

                                
            /* Reset sequence ID. */
            m_ui8SequenceIdx = 0;
            
            break;
        }
        
        case RunModeEnum_Burst:
        {   
            /* Reset burst counter, allowing Seconds ISR to start sampling. */
            burstMode_expPeriodSecondsDownCounter = 0;
            
            break;
        }
        
        case RunModeEnum_SingleShot:
        {
            /* Reset burst counter, allowing Seconds ISR to start sampling. */
            burstMode_expPeriodSecondsDownCounter = 0;
            
            break;
        }
        
        case RunModeEnum_ExtTrig:
        {
            /* Reset burst counter, allowing Seconds ISR to start sampling. */
            burstMode_expPeriodSecondsDownCounter = 0;
            
            break;
        }
        
        case RunModeEnum_None:
        {
            /* This should never be set by the user. */
            return(false);
        }
    }
    
    /* Setup sampling. */
    if(ConfigureMemory() == false)
    {
        return(false);
    }
    
    ConfigurePeripherals();
    ConfigureMuxSampling();
    ConfigureTrigger();
    
    /* Indicate that mode is active. */
    m_iStatus = StatusEnum_Active;
    
    /* Immediately start Continuous Mode. */
    if(m_iRunMode == RunModeEnum_Continuous)
    {
        VOR_TIM_TRIG.ENABLE = 1;
    }
    
    
    return(true);
}


/*******************************************************************************
 **
 ** @brief  -  Executes activity necessary for the STOP command.  Each mode has
 **            separate case.  
 **
 ******************************************************************************/
void
Stop(void)
{
    switch(m_iRunMode)
    {   
        case RunModeEnum_Continuous:
        {
            /* Stop timer. */
            VOR_TIM_TRIG.ENABLE = 0;
            
            /* Immediately set status to idle. */
            m_iStatus = StatusEnum_Idle;
            
            break;
        }
        
        case RunModeEnum_Burst:
        {
            /* Mode will be stopped in seconds' ISR. */
            m_iStatus = StatusEnum_Stopping;
            
            break;
        }
        
        case RunModeEnum_SingleShot:
        {
            /* Mode is automatically stopped.  Nothing to do. */
            
            break;
        }
				        case RunModeEnum_ExtTrig:
        {
            /* Mode will be stopped and changed to status=idle in seconds ISR . */
            m_iStatus = StatusEnum_Stopping;
            break;
        }
        
        default:
        {
            
        }
    }
    
    VOR_PORTB.CLROUT = (1UL << 9);
}


/*******************************************************************************
 **
 ** @brief  Sets the internal time.
 **
 ** @param  ui32Seconds is the time in seconds.
 **
 ******************************************************************************/
bool
SetTime(uint32_t ui32Seconds)
{
    /* Indicate an error if sampling is in progress. */
    if(m_iStatus != StatusEnum_Idle)
    {
        return(false);
    }
    
    NVIC_DisableIRQ(VOR_IRQ_SEC_INDEX);
    
    m_ui32Seconds = ui32Seconds;
    
    NVIC_EnableIRQ(VOR_IRQ_SEC_INDEX);
    
    return(true);
}

/*******************************************************************************
 **
 ** @brief  Sets the internal time.
 **
 ** @param  ui32Seconds is the time in seconds.
 **
 ******************************************************************************/
uint32_t 
GetTime(void)
{
    return(m_ui32Seconds);
}

/*******************************************************************************
 **
 ** @brief  - Updates pointers to data collected by continuous mode sequence
 **
 ******************************************************************************/
void
ReadContinuousResults(uint16_t **ppui16Data, uint8_t *pui8SequenceIdx)
{
    *ppui16Data = m_pui16Buffer;
    
    *pui8SequenceIdx = m_ui8SequenceIdx;
}
                 
/*******************************************************************************
 **
 ** @brief  Updates pointers to data collected by Burst mode sequence
 **
 ******************************************************************************/
void 
ReadBurstResults(ADCParams_t **psADC0, ADCParams_t **psADC1, 
                 ADCParams_t **psADCMux, uint32_t *pui32RTDSum,
                 uint32_t *pui32VCCSum)
{
    *psADC0 = &m_sADC0Params;
    
    *psADC1 = &m_sADC1Params;
    
    *psADCMux = &m_sADCMuxParams;
    
    *pui32RTDSum = burstModeConfig.rtdSum;
    
    *pui32VCCSum = burstModeConfig.vddSum;
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
bool
IsDataAvailable(void)
{
    return(m_bDataAvailable == true);
}

/*******************************************************************************
 **
 ** @brief  
 **
 ******************************************************************************/
void
MarkDataAsConsumed(void)
{
    m_bDataAvailable = false;
}

/*                          */
/* --- HELPER FUNCTIONS --- */
/*                          */

/*******************************************************************************
 **
 ** @brief  Switches ADC mux channel select pins.
 **
 ** @param  ui32PreShiftedMuxChannel is the mux channel to sample, preshifted 
 **                                  such that the value does not need to be 
 **                                  shifted at runtime.
 **
 ******************************************************************************/
__STATIC_INLINE void
_SetADCMuxChannel(uint32_t ui32PreShiftedMuxChannel)
{
    VOR_PORTB.DATAOUT = (VOR_PORTB.DATAOUT & ~(ADC2_MUX_Msk << ADC2_MUX0_PIN)) |
                        ui32PreShiftedMuxChannel;
}

/*******************************************************************************
 **
 ** @brief  Connects SPI1 peripheral pins to ADC0.
 **
 ******************************************************************************/
__STATIC_INLINE void
_SetSPI1ToADC0(void)
{
    /* Revert pins to GPIO. */
    VOR_GPIO_PinMux(GPORTB, 17, FUNSEL0);
    VOR_GPIO_PinMux(GPORTB, 19, FUNSEL0);

    /* Connect pins to SPI1 peripheral. */
    VOR_GPIO_PinMux(GPORTB, 3, FUNSEL1);
    VOR_GPIO_PinMux(GPORTB, 5, FUNSEL1);
}

/*******************************************************************************
 **
 ** @brief  Connects SPI1 peripheral pins to ADC2.
 **
 ******************************************************************************/
__STATIC_INLINE void
_SetSPI1ToADC2(void)
{
    /* Revert pins to GPIO. */
    VOR_GPIO_PinMux(GPORTB, 3, FUNSEL0);
    VOR_GPIO_PinMux(GPORTB, 5, FUNSEL0);
    
    /* Connect pins to SPI1 peripheral. */
    VOR_GPIO_PinMux(GPORTB, 17, FUNSEL1);
    VOR_GPIO_PinMux(GPORTB, 19, FUNSEL1);
}

/*                                    */
/* --- INTERRUPT SERVICE ROUTINES --- */
/*                                    */

/*******************************************************************************
 **
 ** @brief  The ISR for tracking time.
 **
 ** Counts time in number of seconds.
 **
 ******************************************************************************/
void
OC27_IRQHandler(void)
{
    /* Increment second counter. */
    m_ui32Seconds++;
	
#ifdef USE_RTOS	
	if (m_ui32ADCMuxSamplePeriodClks < 200)  { bRTOSDisableNeeded = 1 ; }   
	if (m_ui32ADC01SamplePeriodClks < 200)  { bRTOSDisableNeeded = 1 ; } 
#endif
	
    /* Handle burst and single shot modes. */
    if(m_iStatus == StatusEnum_Active)
    {
        if(m_iRunMode == RunModeEnum_Burst)
        {
            if(burstMode_expPeriodSecondsDownCounter == 0)
            {
                /* Reload burst mode counter. */
                burstMode_expPeriodSecondsDownCounter =
                    burstMode_expPeriodSeconds - 1;
        
                // Will trigger Cascade of other 2 timers within one cycle then
                // disable, the 2 other timers will then free-run.
                VOR_TIM_TRIG.ENABLE = 1;
                    
#ifdef USE_RTOS
                /* Disable FreeRTOS if sample period is less than 4uS */
                if((bRTOSDisableNeeded == true) && (bRTOSSuspended == false))
                {
                    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
                    vTaskSuspendAll();
                    bRTOSSuspended = true;
                }
#endif
            }
            else
            {
                --burstMode_expPeriodSecondsDownCounter;
            }
        }
        else if(m_iRunMode == RunModeEnum_SingleShot)
        {
            m_iStatus = StatusEnum_Stopping;
            
            // Will trigger Cascade of other 2 timers within one cycle then
            // disable, the 2 other timers will then free-run.
            VOR_TIM_TRIG.ENABLE = 1;
                
#ifdef USE_RTOS
            /* Disable FreeRTOS if sample period is less than 4uS */
            if((bRTOSDisableNeeded == true) && (bRTOSSuspended == false))
            {
                SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
                vTaskSuspendAll();
                bRTOSSuspended = true;
            }
#endif   
        }
        
    }
    else
    {
        /*  */
        burstMode_expPeriodSecondsDownCounter = 0;
        
        m_iStatus = StatusEnum_Idle;
        
        m_bOnlyOversample = false;
        
        VOR_TIM_TRIG.ENABLE = 0;
    }
}


/*******************************************************************************
 **
 ** @brief  The ISR for Timer that generates ADC1 CONV signal.
 **
 ** @note   The actual ISR is located in a different function due to extreme
 **         timing requirements.  Calling the seperate function prevents
 **         excessive PUSH cycles from being placed onto the stack when the ISR
 **         is called. 
 **
 ******************************************************************************/
void
OC30_IRQHandler(void)
{
    /* Transmit empty data to clock SPI. */
    VOR_SPI_ADC0.DATA = 0x00;
	VOR_SPI_ADC1.DATA = 0x00;
	
    /* Call ADC01 sampling function. */
    (*pfISR_SampleADC01)();
}

/*******************************************************************************
 **
 ** @brief  The ISR for Timer that generates ADC2 CONV signal.
 **
 ** @note   The actual ISR is located in a different function due to extreme
 **         timing requirements.  Calling the seperate function prevents
 **         excessive PUSH cycles from being placed onto the stack when the ISR
 **         is called. 
 **
 ******************************************************************************/
void
OC31_IRQHandler(void)
{
    /* Transmit empty data to clock SPI. */
	VOR_SPI_ADC2.DATA = 0x00;

    /* Call ADC Mux sampling state machine. */
    (*pfISR_SampleADC2Mux)();
}

/*******************************************************************************
 **
 ** @brief  Samples ADC0 and ADC1 after falling edge of ADC1 CONV signal.
 **
 ** @note   Must check for first entry which will not have valid results checks performed
 **         Switch to sample ADC2 after all samples on ADC0 & 1 completed.
 **         ADC0 & 1, if both enabled, are sampled at exact same time.
 **
 ******************************************************************************/
void
ISR_SampleADC01(void)
{
  if(spiCount == 0)
    {
        /* This is the first time entering the ISR.  The SPI has just started
           being clocked, so no data will be ready yet.*/
           
        /* Disable external trigger until all samples are acquired. */
		if(m_iRunMode == RunModeEnum_ExtTrig)
        {
			     VOR_TIM_TRIG.ENABLE = 0;
        }
//						if(m_iRunMode != RunModeEnum_Continuous)
//						{
//              adc01Mode_sampDownCounter--  ;  /* decrement counter on first entry to account for 1 too many samples */
//			      }
							/* Verify that last set of data has been completely sent to the user. */
		if(m_bDataAvailable == true)
      {
            /* Old sample data is still in the buffer. */
            
            /* Safely disable ADC0 if still active. */
			  if(VOR_TIM_ADC0_CONV.ENABLE != 0)
            {
                /* Wait for SPI transaction to start, */
				    while((VOR_SPI_ADC0.STATUS &
                       SPI_PERIPHERAL_STATUS_BUSY_Msk))
                {
                }
                
                /* ...and stop. */
				   while(!(VOR_SPI_ADC0.STATUS &
                        SPI_PERIPHERAL_STATUS_RNE_Msk))
                {
                }
                
                /* Disable generation of ADC0 CONV signal, */
				   VOR_TIM_ADC0_CONV.ENABLE = 0;
                
                /* ... and clear ADC0 SPI FIFO. */
                VOR_SPI_ADC0.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                                         SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
			      }				

            /* Safely disable ADC1 if still active. */
			if(VOR_TIM_ADC1_CONV.ENABLE != 0)
            {
                /* Wait for SPI transaction to start, */
				    while((VOR_SPI_ADC1.STATUS &
                       SPI_PERIPHERAL_STATUS_BUSY_Msk))
                {
                }
                
                /* ...and stop. */
				    while(!(VOR_SPI_ADC1.STATUS &
                        SPI_PERIPHERAL_STATUS_RNE_Msk))
                {
                }
                
                /* Disable generation of ADC1 CONV signal, */
				    VOR_TIM_ADC1_CONV.ENABLE = 0;
                
                /* ... and clear ADC1 SPI FIFO. */
                VOR_SPI_ADC1.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                                         SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
			     }
            
            /* Set CONV pins for ADC0 and ADC1 low. */
			VOR_TIM_ADC0_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
			VOR_TIM_ADC1_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
            
            /* Re-enable CONV pin generation for next conversion. */
			if(m_sADC0Params.pui16Buffer != 0)
        {
				VOR_TIM_ADC0_CONV.ENABLE = 1;
			  }
            
			if(m_sADC1Params.pui16Buffer != 0)
        {
				VOR_TIM_ADC1_CONV.ENABLE = 1;
			  } 			
            
            /* Re-enable external trigger. */
			if(m_iRunMode == RunModeEnum_ExtTrig)
        {
				VOR_TIM_TRIG.ENABLE = 1;
        }
            
            /* Clear pending IRQ. */
			NVIC_ClearPendingIRQ(VOR_IRQ_CNV_ADC01);
            return;
		}
        
        /* Increment the number of SPI transactions. */
		spiCount++;
        
        /* Disable CONV generation for ADC2, and set pin low. */

		VOR_TIM_ADC2_CONV.ENABLE = 0;
		VOR_TIM_ADC2_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
		
        /* Immediately exit if this is isn't the last sample. */

		if(adc01Mode_sampDownCounter > 0)
        { 
			NVIC_ClearPendingIRQ(VOR_IRQ_CNV_ADC01);
			return;
		}
  }
  else
  {
        /* Read ADC0 data. */
		if(m_sADC0Params.pui16Buffer != 0)
    {
			*m_sADC0Params.pui16Buffer++ = (uint16_t)VOR_SPI_ADC0.DATA;
		}

        /* Read ADC1 data. */
		if(m_sADC1Params.pui16Buffer != 0)
    {
			*m_sADC1Params.pui16Buffer++ = (uint16_t)VOR_SPI_ADC1.DATA;
		}

        /* Increment number of SPI clocks. */
		spiCount++;
        
        /* Decrement number of samples read. */
		adc01Mode_sampDownCounter--;
	}
	
    /* Determine if all samples for ADC0 and ADC1 have been retrieved. */
 	if(adc01Mode_sampDownCounter == 0)
	{
        /* Disable ADC CONV timers, and set CONV pins low. */
		VOR_TIM_ADC0_CONV.ENABLE = 0;
		VOR_TIM_ADC1_CONV.ENABLE = 0;
		
		VOR_TIM_ADC0_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
		VOR_TIM_ADC1_CONV.CNT_VALUE = AD7981_MIN_CONV_CLKS + 1;
		
        /* Reset the down counter. */
		adc01Mode_sampDownCounter = m_ui32ADC01NumSamples - 1;
        
        /* Reset SPI counter. */
		spiCount = 0;
		
        /* Read last sample from 0. */
		if(m_sADC0Params.pui16Buffer != 0)
    {
            /* Wait for SPI transaction to complete, */
			while(!(VOR_SPI_ADC0.STATUS &
                    SPI_PERIPHERAL_STATUS_RNE_Msk))
      {
      }
            
            /* ...and read data. */
			*m_sADC0Params.pui16Buffer = (uint16_t)VOR_SPI_ADC0.DATA;
            
            /* Re-enable timer to start with the next cascade. */
			VOR_TIM_ADC0_CONV.ENABLE = 1;
		}
        
        /* Read last sample from ADC1. */
		if(m_sADC1Params.pui16Buffer != 0)
    {       
            /* Wait for SPI transaction to complete, */
			while(!(VOR_SPI_ADC1.STATUS &
                    SPI_PERIPHERAL_STATUS_RNE_Msk))
      {
      }
            
            /* ...and read data. */
			*m_sADC1Params.pui16Buffer = (uint16_t)VOR_SPI_ADC1.DATA;
            
            /* Re-enable timer to start with the next cascade. */
			VOR_TIM_ADC1_CONV.ENABLE = 1;
		}
		
        /* Reset buffer pointers. */
		m_sADC0Params.pui16Buffer = m_sADC0Params.pui16BufferHead;
		m_sADC1Params.pui16Buffer = m_sADC1Params.pui16BufferHead;


        
    //decode channel # to mux select pins B[20]-B[21]
		//if there's less than 2 conversions the muxModeStat.muxChList[] will have RTD channel in the array
		//so no special case needed
		if(m_ui32ChannelMask <=3)     //  must setup RTD channel in mux if no-mux channels selected. 
		{
			_SetADCMuxChannel(ANAMUX_CH_RTD)  ;  
		}
		else
		{
		  _SetADCMuxChannel(muxModeStat.muxChList[0]);   
		  nextMuxChPins = muxModeStat.muxChList[1];
		}

		//SETUP ANALOG MUX
		//Move SPI1 to ADC2
		_SetSPI1ToADC2();
		VOR_SPI_ADC2.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                                 SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
		VOR_SPI_ADC2.CTRL1 = SPI_PERIPHERAL_CTRL1_ENABLE_Msk;
		
		VOR_TIM_ADC2_CONV.CSD_CTRL &= ~(TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk | 
                                        TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);  //No cascading
		VOR_TIM_ADC2_CONV.CTRL &= ~TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk; //turn off auto-disable
		VOR_TIM_ADC2_CONV.CTRL |= (TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
		VOR_TIM_ADC2_CONV.ENABLE = 1;
	}
        
  NVIC_ClearPendingIRQ(VOR_IRQ_CNV_ADC01);
  return;
}

/**
  * @brief  Main ADC2 Mux cycle through sample list
  * @retval None
  */
/*******************************************************************************
 **
 ** @brief  Sampling activity for Multiplexed channels 
 **       Channels are interleave (i.e. 2, 4, 5, 2, 4, 5....) 
 **       Mux channel select change done early to allow for output to settle 
 **
 ******************************************************************************/
void ISR_SampleADCMux_SampleRequestedChannels(void)
{
    /* Select next ADC Mux channel. */
	_SetADCMuxChannel(nextMuxChPins);  //needs to be inline

    /* Track sample. */
    muxModeStat.sampDownCounter--;
	  muxModeStat.chDoneCount++;
    
	if (muxModeStat.sampDownCounter != 0)
    {   
        if(muxModeStat.chDoneCount == muxModeStat.chCount)
        {
            muxModeStat.chDoneCount = 0;
            muxModeStat.sampCounter++;
            
            nextMuxChPins = muxModeStat.muxChList[1];
        }
        else
        {
            nextMuxChPins =  muxModeStat.muxChList[muxModeStat.chDoneCount + 1];
        }
        
        /* No change to state machine. */
	}
	else
    {
        VOR_TIM_ADC2_CONV.RST_VALUE = RTD_MUXSWITCH_PERIOD_CLOCKS;
		
        /* Switch to oversampling the RTD. */
		    nextMuxChPins = ANAMUX_CH_RTD;
        
		    pfISR_SampleADC2Mux = &ISR_SampleADCMux_TransitionToRTD;
    }

    /* Wait for data to be read. */
    while(!(VOR_SPI_ADC2.STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk))
    {
    }
    
    /* Read data. */
    *m_sADCMuxParams.pui16Buffer++ = VOR_SPI1.DATA;
    
	return;
}

/**
  * @brief  ADC2 Mux first-cycle-only of RTD sample sequence
  * @retval None
  */
/*******************************************************************************
 **
 ** @brief  Separate routine for reading RTD on channel 9.  This is done for each
 **         burst experiment sequence 
 **
 ******************************************************************************/
void ISR_SampleADCMux_TransitionToRTD(void)
{
    VOR_TIM_ADC2_CONV.RST_VALUE = RTD_SAMP_PERIOD_CLOCKS;
    
    /* Next sample will also be RTD. */
	_SetADCMuxChannel(ANAMUX_CH_RTD);
    
    /* Configure ADC Mux parameters for only measuring RTD. */
	muxModeStat.sampDownCounter = configRTD_SAMPLES;
	burstModeConfig.rtdSum = 0;
    
    /* Set next state of state machine. */
	pfISR_SampleADC2Mux = &ISR_SampleADCMux_RTDOnly;
    
    /* Wait for data to be clocked into SPI. */
    while(!(VOR_SPI_ADC2.STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk))
    {
    }
    
    /* Read data. */
    burstModeConfig.rtdSum += VOR_SPI_ADC2.DATA;
    
	return;
}


volatile uint32_t debug_RTD[4], RTD_Counter = 0 ; 
/**
  * @brief  ADC2 Mux cycle through RTD sample sequence, reset if done
  * @retval None
  */
/*******************************************************************************
 **
 ** @brief  ISR to Sample ADC Mux_RTD Only  (samples it 4 times) 
 **
 ******************************************************************************/
void ISR_SampleADCMux_RTDOnly(void)
{	        
	muxModeStat.sampDownCounter--;
    
    /* Reset RTD sum if this state was entered for the first time. */
	if(muxModeStat.sampDownCounter == configRTD_SAMPLES - 1)
    {
		burstModeConfig.rtdSum = 0;
	}
    
    /* Setup next state if all RTD samples have been read. */
    if(muxModeStat.sampDownCounter == 0)
    {
        /* Reset VCC Sum. */
        burstModeConfig.vddSum = 0;
        
        /* Sample period set to unique value to allow adequate settling time */
		VOR_TIM_ADC2_CONV.RST_VALUE = RTD_SAMP_PERIOD_CLOCKS;
        
        /* Select VCC mux channel. */
		_SetADCMuxChannel(ANAMUX_CH_VREF_DIV3);
        
        /* Reset sample counter. */
		muxModeStat.sampDownCounter = configVCC_SAMPLES;
        
        /* Set ADC Mux ISR pointer to last state. */
     pfISR_SampleADC2Mux = &ISR_SampleADCMux_VCCAndFinish;
	} 
	
    /* Wait for data to be read. */
    while(!(VOR_SPI_ADC2.STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk))
    {
    }
    
    /* Read data. */
    burstModeConfig.rtdSum += VOR_SPI_ADC2.DATA;

}

/*******************************************************************************
 **
 ** @brief  ISR for sampling ch8 & 9  (VDD and Temp sensor)
 **
 ** Sample VCC, and terminate sampling sequence/setup next sequence. 
 **
 ******************************************************************************/
void ISR_SampleADCMux_VCCAndFinish(void)
{
    /* Decrement sample counter. */
    muxModeStat.sampDownCounter--;
        
    /* Determine if VCC samples are needed, */
 	if(muxModeStat.sampDownCounter != 0)
    {
        /* Wait for VCC sample to be received, */
        while(!(VOR_SPI_ADC2.STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk))
        {
        }
        
        /* ...and add VCC sample to sum. */
        burstModeConfig.vddSum += VOR_SPI_ADC2.DATA;
    }
    
    /* ...otherwise complete the sampling process. */
    else
    {
        /* Disable ADC2 convert signal generation, and set pin low. */
      VOR_TIM_ADC2_CONV.ENABLE = 0; 
		  VOR_TIM_ADC2_CONV.CNT_VALUE = m_ui32ADCMuxSamplePeriodClks + 1;
		
        /* Reset mux variables for next cycle. */
        ConfigureMuxSampling();
		
        /* Increment continuous mode sequence index. */
		 m_ui8SequenceIdx++;
        
        /* Timestamp experiment. */
		 burstMode_timeStamp = m_ui32Seconds;
        
        /* Read last sample. */
		while(!(VOR_SPI_ADC2.STATUS & SPI_PERIPHERAL_STATUS_RNE_Msk))
        {
        }
        burstModeConfig.vddSum += VOR_SPI_ADC2.DATA;	
        
        /* Reset ADC01 Buffer. */
		m_sADCMuxParams.pui16Buffer = m_sADCMuxParams.pui16BufferHead;
		
        /* Re-enable external triggering. */
		if(m_iRunMode == RunModeEnum_ExtTrig)
        {
			VOR_TIM_TRIG.ENABLE = 1;
        }
        
        /* Determine if sampling settings need to be moved back to ADC01. */
        if((m_ui32ChannelMask & 0x3) != 0)
        {
            /* Switch SPI1 back to ADC0. */
            _SetSPI1ToADC0();
            
            if((m_ui32ChannelMask & 0x3) == (1 << 1))
            {
                /* Only re-enable ADC1. */
                VOR_TIM_ADC0_CONV.ENABLE = 0;
                VOR_TIM_ADC0_CONV.CTRL &= ~(TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk);
                
                VOR_SPI_ADC0.CTRL1 &= ~(SPI_PERIPHERAL_CTRL1_ENABLE_Msk);
            }
        }
        else
        {
            /* Re-enable ADC2 CONV signal generation. */
            VOR_TIM_ADC2_CONV.ENABLE = 1;
        }
		
        /* Clear SPI1 RX FIFOs. */
		VOR_SPI_ADC2.FIFO_CLR = (SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk |
                                 SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk);
        
        /* Indicate that ADC sampling has been completed. */
		m_bDataAvailable = true;
        
        /* Resume FreeRTOS tasks if needed. */
#ifdef USE_RTOS
        if(bRTOSSuspended == true)
        {
            bRTOSSuspended = false;
            SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
            xTaskResumeAll();
        }
#endif
	}	
}
