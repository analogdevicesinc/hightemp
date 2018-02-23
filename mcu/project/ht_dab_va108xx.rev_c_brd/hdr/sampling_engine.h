#ifndef __SAMPLING_ENGINE_H
#define __SAMPLING_ENGINE_H
 
#include <stdint.h>
#include <stdbool.h>
#include "va108xx.h"
#include "ht_dab.h"
#include "ht_dab_user.h"
#include "driver_common.h"


enum StatusEnum
{
    StatusEnum_Idle,
    StatusEnum_Active,
    StatusEnum_Stopping
};

enum RunModeEnum
{
    RunModeEnum_None,
    RunModeEnum_Continuous,
    RunModeEnum_Burst,
    RunModeEnum_SingleShot,
    RunModeEnum_ExtTrig
};

typedef struct ADCParams
{
    uint16_t *pui16Buffer;
    
    
    uint16_t *pui16BufferHead;
    
    /* Size of each channel's buffer. */
    uint32_t ui32Size;
    
    /* Sample period in number of 100ns. */
    uint32_t ui32Period;
    
    /* */
    uint32_t ui32ChannelMask;
} ADCParams_t;


extern struct muxModeStatStruct
{
  uint32_t chDoneCount; //!<upcounter through channels in $setmux <ch_mask>
  uint32_t sampCounter;         //!< upcounter to $setmux <samp_size>
  uint32_t muxChList[8 + 2];    //!<(can be 8 bit) list of ch from $setmux <ch_mask>, 0="ch2"=Analog Mux sel 0b000, oversized to avoid wrapping calc during pipelining
  uint32_t chCount;  //!< total count of ch from $setmux <ch_mask>
  uint32_t sampDownCounter;
}
muxModeStat;


extern struct burstModeStruct
{
    volatile uint32_t timeStamp; //!< burstMode_timeStamp for $continuous mode //=0xffffffff
	         uint32_t vddSum;
	         uint32_t rtdSum;
    volatile uint32_t expPeriodSeconds;   //!< $burst period, defaults to 2 sec //=2UL
    volatile uint32_t expPeriodSecondsDownCounter; //!< Downcounter decremented by seconds ISR, triggers experiment cascade when 0 //=0
}
burstModeConfig;

extern bool flag_dataAvail;

void SamplingEngineInit(void);

long xTaskResumeAll( void ) ;
void vTaskSuspendAll( void ) ;

bool ConfigureMemory(void);
              
void ConfigurePeripherals(void);

void ConfigureMuxSampling(void);

void ConfigureTrigger(void);


bool SetMode(enum RunModeEnum iRunMode);

enum RunModeEnum GetMode(void);

bool SetAcquisitionParams(uint32_t ui32ChannelMask,
                          uint32_t ui32ADC01SamplePeriod,
                          uint32_t ui32ADC01NumSamples,
                          uint32_t ui32ADCMuxSamplePeriod,
                          uint32_t ui32ADCMuxNumSamples); 

bool SetContinuousModeSamplePeriod(uint32_t ui32SamplePeriod);

bool SetBurstModeSamplePeriod(uint32_t ui32Seconds);

bool OnlyOversample(void);

bool SetTime(uint32_t ui32Seconds);

uint32_t GetTime(void);

bool Start(void);
void Stop(void);


void ReadContinuousResults(uint16_t **ppui16Data, uint8_t *pui8SequenceIdx);
                 
void ReadBurstResults(ADCParams_t **psADC0, ADCParams_t **psADC1, 
                      ADCParams_t **psADCMux, uint32_t *pui32RTDSum,
                      uint32_t *pui32VCCSum);

bool IsDataAvailable(void);

void MarkDataAsConsumed(void);


__STATIC_INLINE void _SetADCMuxChannel(uint32_t ui32PreShiftedMuxChannel);

__STATIC_INLINE void _SetSPI1ToADC0(void);

__STATIC_INLINE void _SetSPI1ToADC2(void);


void ISR_SampleADC01(void);

void ISR_SampleADCMux_SampleRequestedChannels(void);

void ISR_SampleADCMux_TransitionToRTD(void);

void ISR_SampleADCMux_RTDOnly(void);

void ISR_SampleADCMux_VCCAndFinish(void);

#endif // __SAMPLING_ENGINE_H
