#ifndef __OUTPUT_H
#define __OUTPUT_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "uart.h"

#include "ht_dab_user.h"
#include "ht_dab.h"

#include "sampling_engine.h"

void PrintContinuousModeData(void);

void PrintBurstModeData(void);

//void PrintSingleShotModeData(void);

//void PrintExternalTriggerModeData(void);

void PrintADC01Samples(ADCParams_t *psADC01);

void PrintADCMuxSamples(ADCParams_t *psADCMux);

int32_t getTempK10_from_adcCodeSum(uint32_t ui32Sample);

uint32_t getVolt1000fromSum(uint32_t ui32Sample);

#endif
