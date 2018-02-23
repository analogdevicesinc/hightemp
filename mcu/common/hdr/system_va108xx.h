/**************************************************************************//**
 * @file     system_ARMCM0.h
 * @brief    CMSIS Device System Header File for
 *           ARMCM0 Device Series
 * @version  V1.07
 * @date     30. January 2012
 *
 * @note
 * Copyright (C) 2011 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/
 /*
 From: C:\Keil\ARM\Device\ARM\ARMCM0\Include
 */


#ifndef SYSTEM_ARMCM0_H
#define SYSTEM_ARMCM0_H

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------  SST IRQ Index  ------------------------------ */
#define IRQ_DST_IRQ_0 0x00
#define IRQ_DST_IRQ_1 0x01
#define IRQ_DST_IRQ_2 0x02
#define IRQ_DST_IRQ_3 0x03
#define IRQ_DST_IRQ_4 0x04
#define IRQ_DST_IRQ_5 0x05
#define IRQ_DST_IRQ_6 0x06
#define IRQ_DST_IRQ_7 0x07
#define IRQ_DST_IRQ_8 0x08
#define IRQ_DST_IRQ_9 0x09
#define IRQ_DST_IRQ_10 0x0a
#define IRQ_DST_IRQ_11 0x0b
#define IRQ_DST_IRQ_12 0x0c
#define IRQ_DST_IRQ_13 0x0d
#define IRQ_DST_IRQ_14 0x0e
#define IRQ_DST_IRQ_15 0x0f
#define IRQ_DST_IRQ_16 0x10
#define IRQ_DST_IRQ_17 0x11
#define IRQ_DST_IRQ_18 0x12
#define IRQ_DST_IRQ_19 0x13
#define IRQ_DST_IRQ_20 0x14
#define IRQ_DST_IRQ_21 0x15
#define IRQ_DST_IRQ_22 0x16
#define IRQ_DST_IRQ_23 0x17
#define IRQ_DST_IRQ_24 0x18
#define IRQ_DST_IRQ_25 0x19
#define IRQ_DST_IRQ_26 0x1a
#define IRQ_DST_IRQ_27 0x1b
#define IRQ_DST_IRQ_28 0x1c
#define IRQ_DST_IRQ_29 0x1d
#define IRQ_DST_IRQ_30 0x1e
#define IRQ_DST_IRQ_31 0x1f

#define IRQ_DST_EDBGRQ 0xfffffffa
#define IRQ_DST_MERESET 0xfffffffb
#define IRQ_DST_WATCHDOG 0xfffffffc
#define IRQ_DST_RXEV 0xfffffffd
#define IRQ_DST_NMI 0xfffffffe
#define IRQ_DST_NONE 0xffffffff

/* --------------------------  RTC Cascade Source Selects ------------------ */
#define RTC_CAS_SRC_PORTA_0 0
#define RTC_CAS_SRC_PORTA_1 1
#define RTC_CAS_SRC_PORTA_2 2
#define RTC_CAS_SRC_PORTA_3 3
#define RTC_CAS_SRC_PORTA_4 4
#define RTC_CAS_SRC_PORTA_5 5
#define RTC_CAS_SRC_PORTA_6 6
#define RTC_CAS_SRC_PORTA_7 7
#define RTC_CAS_SRC_PORTA_8 8
#define RTC_CAS_SRC_PORTA_9 9
#define RTC_CAS_SRC_PORTA_10 10
#define RTC_CAS_SRC_PORTA_11 11
#define RTC_CAS_SRC_PORTA_12 12
#define RTC_CAS_SRC_PORTA_13 13
#define RTC_CAS_SRC_PORTA_14 14
#define RTC_CAS_SRC_PORTA_15 15
#define RTC_CAS_SRC_PORTA_16 16
#define RTC_CAS_SRC_PORTA_17 17
#define RTC_CAS_SRC_PORTA_18 18
#define RTC_CAS_SRC_PORTA_19 19
#define RTC_CAS_SRC_PORTA_20 20
#define RTC_CAS_SRC_PORTA_21 21
#define RTC_CAS_SRC_PORTA_22 22
#define RTC_CAS_SRC_PORTA_23 23
#define RTC_CAS_SRC_PORTA_24 24
#define RTC_CAS_SRC_PORTA_25 25
#define RTC_CAS_SRC_PORTA_26 26
#define RTC_CAS_SRC_PORTA_27 27
#define RTC_CAS_SRC_PORTA_28 28
#define RTC_CAS_SRC_PORTA_29 29
#define RTC_CAS_SRC_PORTA_30 30
#define RTC_CAS_SRC_PORTA_31 31

#define RTC_CAS_SRC_PORTB_0 32
#define RTC_CAS_SRC_PORTB_1 33
#define RTC_CAS_SRC_PORTB_2 34
#define RTC_CAS_SRC_PORTB_3 35
#define RTC_CAS_SRC_PORTB_4 36
#define RTC_CAS_SRC_PORTB_5 37
#define RTC_CAS_SRC_PORTB_6 38
#define RTC_CAS_SRC_PORTB_7 39
#define RTC_CAS_SRC_PORTB_8 40
#define RTC_CAS_SRC_PORTB_9 41
#define RTC_CAS_SRC_PORTB_10 42
#define RTC_CAS_SRC_PORTB_11 43
#define RTC_CAS_SRC_PORTB_12 44
#define RTC_CAS_SRC_PORTB_13 45
#define RTC_CAS_SRC_PORTB_14 46
#define RTC_CAS_SRC_PORTB_15 47
#define RTC_CAS_SRC_PORTB_16 48
#define RTC_CAS_SRC_PORTB_17 49
#define RTC_CAS_SRC_PORTB_18 50
#define RTC_CAS_SRC_PORTB_19 51
#define RTC_CAS_SRC_PORTB_20 52
#define RTC_CAS_SRC_PORTB_21 53
#define RTC_CAS_SRC_PORTB_22 54
#define RTC_CAS_SRC_PORTB_23 55
#define RTC_CAS_SRC_PORTB_24 56

#define RTC_CAS_SRC_RTC_0 64
#define RTC_CAS_SRC_RTC_1 65
#define RTC_CAS_SRC_RTC_2 66
#define RTC_CAS_SRC_RTC_3 67
#define RTC_CAS_SRC_RTC_4 68
#define RTC_CAS_SRC_RTC_5 69
#define RTC_CAS_SRC_RTC_6 70
#define RTC_CAS_SRC_RTC_7 71
#define RTC_CAS_SRC_RTC_8 72
#define RTC_CAS_SRC_RTC_9 73
#define RTC_CAS_SRC_RTC_10 74
#define RTC_CAS_SRC_RTC_11 75
#define RTC_CAS_SRC_RTC_12 76
#define RTC_CAS_SRC_RTC_13 77
#define RTC_CAS_SRC_RTC_14 78
#define RTC_CAS_SRC_RTC_15 79
#define RTC_CAS_SRC_RTC_16 80
#define RTC_CAS_SRC_RTC_17 81
#define RTC_CAS_SRC_RTC_18 82
#define RTC_CAS_SRC_RTC_19 83
#define RTC_CAS_SRC_RTC_20 84
#define RTC_CAS_SRC_RTC_21 85
#define RTC_CAS_SRC_RTC_22 86
#define RTC_CAS_SRC_RTC_23 87

#define RTC_CAS_SRC_RAM_SBE 96
#define RTC_CAS_SRC_RAM_MBE 97
#define RTC_CAS_SRC_ROM_SBE 98
#define RTC_CAS_SRC_ROM_MBE 99
#define RTC_CAS_SRC_TXEV 100
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_0 120
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_1 121
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_2 122
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_3 123
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_4 124
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_5 125
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_6 126
#define RTC_CAS_SRC_IOCONFIG_CLKDIV_7 127

/* ------------------------------------------------------------------------- */

extern uint32_t SystemCoreClock;     /*!< System Clock Frequency (Core Clock)  */


/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
extern void SystemInit (void);

/**
 * Update SystemCoreClock variable
 *
 * @param  none
 * @return none
 *
 * @brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
extern void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_ARMCM0_H */
