/***************************************************************************************
 * @file     va108xx.h
 * @brief    CMSIS Cortex-M0 Peripheral Access Layer Header File for
 *           VA108XX from Vorago Technologies.
 * @version  V1.3
 * @date     V1.1 update Nov 2016,  Original release 31. March 2016
 * @version   V1.20
 * @date   V1.2 update Feb 2017,   Added missing CSDTRG2 in CSD_CTRL
  * @version   V1.30
 * @date   V1.3 update Jul 2017,   Corrected SPIx_CTRL1_SS_Msk.   Was 0x3, Now 0x7  (should have been 3 bits wide)
 *                                  Added TIM_PERIPHERAL CTRL: STATUS SELECT options 
 *                                  Corrected FIFO_STATE_Msk equation in UART and SPI 
 *
 * @note     
 * Generated with SVDConv V2.83a and from CMSIS SVD File 'VA108XX.svd' Version 1.0. 
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
 * v1.1 changes - I2C block inconsistency in naming convention corrected  
 *
 ****************************************************************************************/

/** @addtogroup Vorago Technologies
  * @{
  */

/** @addtogroup VA108XX
  * @{
  */

#ifndef VA108XX_H
#define VA108XX_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M0 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  DebugMonitor_IRQn             =  -4,              /*!<  12  Debug Monitor                                                    */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ---------------------  PA03CSA Specific Interrupt Numbers  --------------------- */
  OC0_IRQn                      =   0,              /*!<   0  OC0                                                              */
  OC1_IRQn                      =   1,              /*!<   1  OC1                                                              */
  OC2_IRQn                      =   2,              /*!<   2  OC2                                                              */
  OC3_IRQn                      =   3,              /*!<   3  OC3                                                              */
  OC4_IRQn                      =   4,              /*!<   4  OC4                                                              */
  OC5_IRQn                      =   5,              /*!<   5  OC5                                                              */
  OC6_IRQn                      =   6,              /*!<   6  OC6                                                              */
  OC7_IRQn                      =   7,              /*!<   7  OC7                                                              */
  OC8_IRQn                      =   8,              /*!<   8  OC8                                                              */
  OC9_IRQn                      =   9,              /*!<   9  OC9                                                              */
  OC10_IRQn                     =  10,              /*!<  10  OC10                                                             */
  OC11_IRQn                     =  11,              /*!<  11  OC11                                                             */
  OC12_IRQn                     =  12,              /*!<  12  OC12                                                             */
  OC13_IRQn                     =  13,              /*!<  13  OC13                                                             */
  OC14_IRQn                     =  14,              /*!<  14  OC14                                                             */
  OC15_IRQn                     =  15,              /*!<  15  OC15                                                             */
  OC16_IRQn                     =  16,              /*!<  16  OC16                                                             */
  OC17_IRQn                     =  17,              /*!<  17  OC17                                                             */
  OC18_IRQn                     =  18,              /*!<  18  OC18                                                             */
  OC19_IRQn                     =  19,              /*!<  19  OC19                                                             */
  OC20_IRQn                     =  20,              /*!<  20  OC20                                                             */
  OC21_IRQn                     =  21,              /*!<  21  OC21                                                             */
  OC22_IRQn                     =  22,              /*!<  22  OC22                                                             */
  OC23_IRQn                     =  23,              /*!<  23  OC23                                                             */
  OC24_IRQn                     =  24,              /*!<  24  OC24                                                             */
  OC25_IRQn                     =  25,              /*!<  25  OC25                                                             */
  OC26_IRQn                     =  26,              /*!<  26  OC26                                                             */
  OC27_IRQn                     =  27,              /*!<  27  OC27                                                             */
  OC28_IRQn                     =  28,              /*!<  28  OC28                                                             */
  OC29_IRQn                     =  29,              /*!<  29  OC29                                                             */
  OC30_IRQn                     =  30,              /*!<  30  OC30                                                             */
  OC31_IRQn                     =  31               /*!<  31  OC31                                                             */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M0 Processor and Core Peripherals---------------- */
#define __CM0_REV                 0x0000            /*!< Cortex-M0 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               2            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm0.h"                               /*!< Cortex-M0 processor and core peripherals                              */
#include "system_va108xx.h"                         /*!< PA03CSA System                                                        */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                    SYSCONFIG                   ================ */
/* ================================================================================ */


/**
  * @brief System Configuration Peripheral (SYSCONFIG)
  */

typedef struct {                                    /*!< SYSCONFIG Structure                                                   */
  __IO uint32_t  RST_STAT;                          /*!< System Reset Status                                                   */
  __IO uint32_t  RST_CNTL_ROM;                      /*!< ROM Reset Control                                                     */
  __IO uint32_t  RST_CNTL_RAM;                      /*!< RAM Reset Control                                                     */
  __IO uint32_t  ROM_PROT;                          /*!< ROM Protection Configuration                                          */
  __IO uint32_t  ROM_SCRUB;                         /*!< ROM Scrub Period Configuration                                        */
  __IO uint32_t  RAM_SCRUB;                         /*!< RAM Scrub Period Configuration                                        */
  __IO uint32_t  ROM_TRAP_ADDR;                     /*!< ROM Trap Address                                                      */
  __IO uint32_t  ROM_TRAP_SYND;                     /*!< ROM Trap Syndrome                                                     */
  __IO uint32_t  RAM_TRAP_ADDR;                     /*!< RAM Trap Address                                                      */
  __IO uint32_t  RAM_TRAP_SYND;                     /*!< RAM Trap Syndrome                                                     */
  __IO uint32_t  IRQ_ENB;                           /*!< Enable EDAC Error Interrupt Register                                  */
  __I  uint32_t  IRQ_RAW;                           /*!< Raw EDAC Error Interrupt Status                                       */
  __I  uint32_t  IRQ_END;                           /*!< Enabled EDAC Error Interrupt Status                                   */
  __O  uint32_t  IRQ_CLR;                           /*!< Clear EDAC Error Interrupt Status                                     */
  __IO uint32_t  RAM_SBE;                           /*!< Count of RAM EDAC Single Bit Errors                                   */
  __IO uint32_t  RAM_MBE;                           /*!< Count of RAM EDAC Multi Bit Errors                                    */
  __IO uint32_t  ROM_SBE;                           /*!< Count of ROM EDAC Single Bit Errors                                   */
  __IO uint32_t  ROM_MBE;                           /*!< Count of ROM EDAC Multi Bit Errors                                    */
  __I  uint32_t  IOCONFIG_CLKDIV0;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV1;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV2;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV3;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV4;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV5;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV6;                  /*!< IO Configuration Clock Divider Register                               */
  __IO uint32_t  IOCONFIG_CLKDIV7;                  /*!< IO Configuration Clock Divider Register                               */
  __I  uint32_t  ROM_RETRIES;                       /*!< ROM BOOT Retry count                                                  */
  __IO uint32_t  REFRESH_CONFIG;                    /*!< Register Refresh Control                                              */
  __IO uint32_t  TIM_RESET;                         /*!< TIM Reset Control                                                     */
  __IO uint32_t  TIM_CLK_ENABLE;                    /*!< TIM Enable Control                                                    */
  __IO uint32_t  PERIPHERAL_RESET;                  /*!< Peripheral Reset Control                                              */
  __IO uint32_t  PERIPHERAL_CLK_ENABLE;             /*!< Peripheral Enable Control                                             */
  __IO uint32_t  LOCKUP_RESET;                      /*!< Lockup Reset Configuration                                            */
  __I  uint32_t  RESERVED0[987];
  __I  uint32_t  EF_CONFIG;                         /*!< EFuse Config Register                                                 */
  __I  uint32_t  EF_ID;                             /*!< EFuse ID Register                                                     */
  __I  uint32_t  PROCID;                            /*!< Processor ID Register                                                 */
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_SYSCONFIG_Type;


/* ================================================================================ */
/* ================                     IRQSEL                     ================ */
/* ================================================================================ */


/**
  * @brief Interrupt Selector Peripheral (IRQSEL)
  */

typedef struct {                                    /*!< IRQSEL Structure                                                      */
  __IO uint32_t  PORTA[32];                         /*!< PORTA Interrupt Redirect Selection                                    */
  __IO uint32_t  PORTB[32];                         /*!< PORTB Interrupt Redirect Selection                                    */
  __IO uint32_t  TIM[32];                           /*!< TIM Interrupt Redirect Selection                                      */
  __IO uint32_t  UART[4];                           /*!< UART Interrupt Redirect Selection                                     */
  __IO uint32_t  SPI[4];                            /*!< SPI Interrupt Redirect Selection                                      */
  __IO uint32_t  I2C_MS[4];                         /*!< Master I2C Interrupt Redirect Selection                               */
  __IO uint32_t  I2C_SL[4];                         /*!< Slave I2C Interrupt Redirect Selection                                */
  __IO uint32_t  INT_RAM_SBE;                       /*!< Internal Memory RAM SBE Interrupt Redirect Selection                  */
  __IO uint32_t  INT_RAM_MBE;                       /*!< Internal Memory RAM MBE Interrupt Redirect Selection                  */
  __IO uint32_t  INT_ROM_SBE;                       /*!< Internal Memory ROM SBE Interrupt Redirect Selection                  */
  __IO uint32_t  INT_ROM_MBE;                       /*!< Internal Memory ROM MBE Interrupt Redirect Selection                  */
  __IO uint32_t  TXEV;                              /*!< Processor TXEV Interrupt Redirect Selection                           */
  __I  uint32_t  RESERVED0[395];
  __I  uint32_t  IRQS[32];                          /*!< Interrupt Status Register                                             */
  __I  uint32_t  RESERVED1[26];
  __I  uint32_t  EDBGRQ;                            /*!< EDBGRQ Status Register                                                */
  __I  uint32_t  MERESET;                           /*!< MERESET Status Register                                               */
  __I  uint32_t  WATCHDOG;                          /*!< WATCHDOG Status Register                                              */
  __I  uint32_t  RXEV;                              /*!< RXEV Status Register                                                  */
  __I  uint32_t  NMI;                               /*!< NMI Status Register                                                   */
  __I  uint32_t  RESERVED2[448];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_IRQSEL_Type;


/* ================================================================================ */
/* ================                    IOCONFIG                    ================ */
/* ================================================================================ */


/**
  * @brief IO Pin Configuration Peripheral (IOCONFIG)
  */

typedef struct {                                    /*!< IOCONFIG Structure                                                    */
  __IO uint32_t  PORTA[32];                         /*!< PORTA Pin Configuration Register                                      */
  __IO uint32_t  PORTB[32];                         /*!< PORTB Pin Configuration Register                                      */
  __I  uint32_t  RESERVED0[959];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_IOCONFIG_Type;


/* ================================================================================ */
/* ================                     UTILITY                    ================ */
/* ================================================================================ */


/**
  * @brief Utility Peripheral (UTILITY)
  */

typedef struct {                                    /*!< UTILITY Structure                                                     */
  __IO uint32_t  SYND_DATA0;                        /*!< Synd Data 0 Register                                                  */
  __IO uint32_t  SYND_DATA1;                        /*!< Synd Data 1 Register                                                  */
  __IO uint32_t  SYND_SYND;                         /*!< Synd Parity Register                                                  */
  __I  uint32_t  SYND_ENC_32;                       /*!< Synd 32 bit Encoded Syndrome                                          */
  __I  uint32_t  SYND_CHECK_32_DATA;                /*!< Synd 32 bit Corrected Data                                            */
  __I  uint32_t  SYND_CHECK_32_SYND;                /*!< Synd 32 bit Corrected Syndrome and Status                             */
  __I  uint32_t  SYND_ENC_64;                       /*!< Synd 64 bit Encoded Syndrome                                          */
  __I  uint32_t  SYND_CHECK_64_DATA0;               /*!< Synd 64 bit Corrected Data 0                                          */
  __I  uint32_t  SYND_CHECK_64_DATA1;               /*!< Synd 64 bit Corrected Data 1                                          */
  __I  uint32_t  SYND_CHECK_64_SYND;                /*!< Synd 64 bit Corrected Parity and Status                               */
  __I  uint32_t  SYND_ENC_32_52;                    /*!< Synd 32/52 bit Encoded Syndrome                                       */
  __I  uint32_t  SYND_CHECK_32_52_DATA;             /*!< Synd 32/52 bit Corrected Data                                         */
  __I  uint32_t  SYND_CHECK_32_52_SYND;             /*!< Synd 32/52 bit Corrected Syndrome and Status                          */
  __I  uint32_t  RESERVED0[1010];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_UTILITY_Type;


/* ================================================================================ */
/* ================                 GPIO_PERIPHERAL                ================ */
/* ================================================================================ */


/**
  * @brief GPIO Peripheral (GPIO_PERIPHERAL)
  */

typedef struct {                                    /*!< GPIO_PERIPHERAL Structure                                             */
  
  union {
    __I  uint32_t  DATAIN;                          /*!< Data In Register                                                      */
    __I  uint8_t   DATAINBYTE[4];                   /*!< Data In Register by Byte                                              */
  };
  
  union {
    __I  uint32_t  DATAINRAW;                       /*!< Data In Raw Register                                                  */
    __I  uint8_t   DATAINRAWBYTE[4];                /*!< Data In Raw Register by Byte                                          */
  };
  
  union {
    __O  uint32_t  DATAOUT;                         /*!< Data Out Register                                                     */
    __O  uint8_t   DATAOUTBYTE[4];                  /*!< Data Out Register by Byte                                             */
  };
  
  union {
    __O  uint32_t  DATAOUTRAW;                      /*!< Data Out Register                                                     */
    __O  uint8_t   DATAOUTRAWBYTE[4];               /*!< Data Out Register by Byte                                             */
  };
  
  union {
    __O  uint32_t  SETOUT;                          /*!< Set Out Register                                                      */
    __O  uint8_t   SETOUTBYTE[4];                   /*!< Set Out Register by Byte                                              */
  };
  
  union {
    __O  uint32_t  CLROUT;                          /*!< Clear Out Register                                                    */
    __O  uint8_t   CLROUTBYTE[4];                   /*!< Clear Out Register by Byte                                            */
  };
  
  union {
    __O  uint32_t  TOGOUT;                          /*!< Toggle Out Register                                                   */
    __O  uint8_t   TOGOUTBYTE[4];                   /*!< Toggle Out Register by Byte                                           */
  };
  
  union {
    __IO uint32_t  DATAMASK;                        /*!< Data mask Register                                                    */
    __IO uint8_t   DATAMASKBYTE[4];                 /*!< Data Out Register by Byte                                             */
  };
  
  union {
    __IO uint32_t  DIR;                             /*!< Direction Register (1:Output, 0:Input)                                */
    __IO uint8_t   DIRBYTE[4];                      /*!< Direction Register by Byte                                            */
  };
  
  union {
    __IO uint32_t  PULSE;                           /*!< Pulse Mode Register                                                   */
    __IO uint8_t   PULSEBYTE[4];                    /*!< Pulse Mode Register by Byte                                           */
  };
  
  union {
    __IO uint32_t  PULSEBASE;                       /*!< Pulse Base Value Register                                             */
    __IO uint8_t   PULSEBASEBYTE[4];                /*!< Pulse Base Mode Register by Byte                                      */
  };
  
  union {
    __IO uint32_t  DELAY1;                          /*!< Delay1 Register                                                       */
    __IO uint8_t   DELAY1BYTE[4];                   /*!< Delay1 Register by Byte                                               */
  };
  
  union {
    __IO uint32_t  DELAY2;                          /*!< Delay2 Register                                                       */
    __IO uint8_t   DELAY2BYTE[4];                   /*!< Delay2 Register by Byte                                               */
  };
  __IO uint32_t  IRQ_SEN;                           /*!< Interrupt Sense Register (1:Level Sensitive, 0:Edge Sensitive)        */
  __IO uint32_t  IRQ_EDGE;                          /*!< Interrupt Both Edge Register (1:Both Edges, 0:Single Edge)            */
  __IO uint32_t  IRQ_EVT;                           /*!< Interrupt Event Register (1:HighLevel/L->H Edge, 0:LowLevel/H->L
                                                         Edge)                                                                 */
  __IO uint32_t  IRQ_ENB;                           /*!< Interrupt Enable Register                                             */
  __I  uint32_t  IRQ_RAW;                           /*!< Raw Interrupt Status                                                  */
  __I  uint32_t  IRQ_END;                           /*!< Masked Interrupt Status                                               */
  __IO uint32_t  EDGE_STATUS;                       /*!< Edge Status Register                                                  */
  __I  uint32_t  RESERVED0[1003];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_GPIO_PERIPHERAL_Type;


/* ================================================================================ */
/* ================                 TIM_PERIPHERAL                 ================ */
/* ================================================================================ */


/**
  * @brief Timer/Counter Peripheral (TIM_PERIPHERAL)
  */

typedef struct {                                    /*!< TIM_PERIPHERAL Structure                                              */
  __IO uint32_t  CTRL;                              /*!< Control Register                                                      */
  __IO uint32_t  RST_VALUE;                         /*!< The value that counter start from after reaching 0.                   */
  __IO uint32_t  CNT_VALUE;                         /*!< The current value of the counter                                      */
  __IO uint32_t  ENABLE;                            /*!< Alternate access to the Counter ENABLE bit in the CTRL Register       */
  __IO uint32_t  CSD_CTRL;                          /*!< The Cascade Control Register. Controls the counter external
                                                         enable signals                                                        */
  __IO uint32_t  CASCADE0;                          /*!< Cascade Enable Selection                                              */
  __IO uint32_t  CASCADE1;                          /*!< Cascade Enable Selection                                              */
  __IO uint32_t  CASCADE2;                          /*!< Cascade Enable Selection                                              */
  
  union {
    __IO uint32_t  PWMA_VALUE;                      /*!< The Pulse Width Modulation ValueA                                     */
    __IO uint32_t  PWM_VALUE;                       /*!< The Pulse Width Modulation Value                                      */
  };
  __IO uint32_t  PWMB_VALUE;                        /*!< The Pulse Width Modulation ValueB                                     */
  __I  uint32_t  RESERVED0[1013];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_TIM_PERIPHERAL_Type;


/* ================================================================================ */
/* ================                 UART_PERIPHERAL                ================ */
/* ================================================================================ */


/**
  * @brief UART Peripheral (UART_PERIPHERAL)
  */

typedef struct {                                    /*!< UART_PERIPHERAL Structure                                             */
  __IO uint32_t  DATA;                              /*!< Data In/Out Register                                                  */
  __IO uint32_t  ENABLE;                            /*!< Enable Register                                                       */
  __IO uint32_t  CTRL;                              /*!< Control Register                                                      */
  __O  uint32_t  CLKSCALE;                          /*!< Clock Scale Register                                                  */
  __I  uint32_t  RXSTATUS;                          /*!< Status Register                                                       */
  __I  uint32_t  TXSTATUS;                          /*!< Status Register                                                       */
  __O  uint32_t  FIFO_CLR;                          /*!< Clear FIFO Register                                                   */
  __O  uint32_t  TXBREAK;                           /*!< Break Transmit Register                                               */
  __IO uint32_t  ADDR9;                             /*!< Address9 Register                                                     */
  __IO uint32_t  ADDR9MASK;                         /*!< Address9 Mask Register                                                */
  __IO uint32_t  IRQ_ENB;                           /*!< IRQ Enable Register                                                   */
  __I  uint32_t  IRQ_RAW;                           /*!< IRQ Raw Status Register                                               */
  __I  uint32_t  IRQ_END;                           /*!< IRQ Enabled Status Register                                           */
  __O  uint32_t  IRQ_CLR;                           /*!< IRQ Clear Status Register                                             */
  __IO uint32_t  RXFIFOIRQTRG;                      /*!< Rx FIFO IRQ Trigger Level                                             */
  __IO uint32_t  TXFIFOIRQTRG;                      /*!< Tx FIFO IRQ Trigger Level                                             */
  __IO uint32_t  RXFIFORTSTRG;                      /*!< Rx FIFO RTS Trigger Level                                             */
  __I  uint32_t  STATE;                             /*!< Internal STATE of UART Controller                                     */
  __I  uint32_t  RESERVED0[1005];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_UART_PERIPHERAL_Type;


/* ================================================================================ */
/* ================                 SPI_PERIPHERAL                 ================ */
/* ================================================================================ */


/**
  * @brief SPI Peripheral (SPI_PERIPHERAL)
  */

typedef struct {                                    /*!< SPI_PERIPHERAL Structure                                              */
  __IO uint32_t  CTRL0;                             /*!< Control Register 0                                                    */
  __IO uint32_t  CTRL1;                             /*!< Control Register 1                                                    */
  __IO uint32_t  DATA;                              /*!< Data Input/Output                                                     */
  __I  uint32_t  STATUS;                            /*!< Status Register                                                       */
  __IO uint32_t  CLKPRESCALE;                       /*!< Clock Pre Scale divide value                                          */
  __IO uint32_t  IRQ_ENB;                           /*!< Interrupt Enable Register                                             */
  __I  uint32_t  IRQ_RAW;                           /*!< Raw Interrupt Status Register                                         */
  __I  uint32_t  IRQ_END;                           /*!< Enabled Interrupt Status Register                                     */
  __O  uint32_t  IRQ_CLR;                           /*!< Clear Interrupt Status Register                                       */
  __IO uint32_t  RXFIFOIRQTRG;                      /*!< Rx FIFO IRQ Trigger Level                                             */
  __IO uint32_t  TXFIFOIRQTRG;                      /*!< Tx FIFO IRQ Trigger Level                                             */
  __O  uint32_t  FIFO_CLR;                          /*!< Clear FIFO Register                                                   */
  __I  uint32_t  STATE;                             /*!< Internal STATE of SPI Controller                                      */
  __I  uint32_t  RESERVED0[1010];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_SPI_PERIPHERAL_Type;


/* ================================================================================ */
/* ================                 I2C_PERIPHERAL                 ================ */
/* ================================================================================ */


/**
  * @brief I2C Peripheral (I2C_PERIPHERAL)
  */

typedef struct {                                    /*!< I2C_PERIPHERAL Structure                                              */
  __IO uint32_t  CTRL;                              /*!< Control Register                                                      */
  __IO uint32_t  CLKSCALE;                          /*!< Clock Scale divide value                                              */
  __IO uint32_t  WORDS;                             /*!< Word Count value                                                      */
  __IO uint32_t  ADDRESS;                           /*!< I2C Address value                                                     */
  __IO uint32_t  DATA;                              /*!< Data Input/Output                                                     */
  __IO uint32_t  CMD;                               /*!< Command Register                                                      */
  __I  uint32_t  STATUS;                            /*!< I2C Controller Status Register                                        */
  __I  uint32_t  STATE;                             /*!< Internal STATE of I2C Master Controller                               */
  __I  uint32_t  TXCOUNT;                           /*!< TX Count Register                                                     */
  __I  uint32_t  RXCOUNT;                           /*!< RX Count Register                                                     */
  __IO uint32_t  IRQ_ENB;                           /*!< Interrupt Enable Register                                             */
  __I  uint32_t  IRQ_RAW;                           /*!< Raw Interrupt Status Register                                         */
  __I  uint32_t  IRQ_END;                           /*!< Enabled Interrupt Status Register                                     */
  __O  uint32_t  IRQ_CLR;                           /*!< Clear Interrupt Status Register                                       */
  __IO uint32_t  RXFIFOIRQTRG;                      /*!< Rx FIFO IRQ Trigger Level                                             */
  __IO uint32_t  TXFIFOIRQTRG;                      /*!< Tx FIFO IRQ Trigger Level                                             */
  __O  uint32_t  FIFO_CLR;                          /*!< Clear FIFO Register                                                   */
  __IO uint32_t  TMCONFIG;                          /*!< Timing Config Register                                                */
  __IO uint32_t  CLKTOLIMIT;                        /*!< Clock Low Timeout Limit Register                                      */
  __I  uint32_t  RESERVED0[45];
  __IO uint32_t  S0_CTRL;                           /*!< Slave Control Register                                                */
  __IO uint32_t  S0_MAXWORDS;                       /*!< Slave MaxWords Register                                               */
  __IO uint32_t  S0_ADDRESS;                        /*!< Slave I2C Address Value                                               */
  __IO uint32_t  S0_ADDRESSMASK;                    /*!< Slave I2C Address Mask value                                          */
  __IO uint32_t  S0_DATA;                           /*!< Slave Data Input/Output                                               */
  __I  uint32_t  S0_LASTADDRESS;                    /*!< Slave I2C Last Address value                                          */
  __I  uint32_t  S0_STATUS;                         /*!< Slave I2C Controller Status Register                                  */
  __I  uint32_t  S0_STATE;                          /*!< Internal STATE of I2C Slave Controller                                */
  __I  uint32_t  S0_TXCOUNT;                        /*!< Slave TX Count Register                                               */
  __I  uint32_t  S0_RXCOUNT;                        /*!< Slave RX Count Register                                               */
  __IO uint32_t  S0_IRQ_ENB;                        /*!< Slave Interrupt Enable Register                                       */
  __I  uint32_t  S0_IRQ_RAW;                        /*!< Slave Raw Interrupt Status Register                                   */
  __I  uint32_t  S0_IRQ_END;                        /*!< Slave Enabled Interrupt Status Register                               */
  __O  uint32_t  S0_IRQ_CLR;                        /*!< Slave Clear Interrupt Status Register                                 */
  __IO uint32_t  S0_RXFIFOIRQTRG;                   /*!< Slave Rx FIFO IRQ Trigger Level                                       */
  __IO uint32_t  S0_TXFIFOIRQTRG;                   /*!< Slave Tx FIFO IRQ Trigger Level                                       */
  __O  uint32_t  S0_FIFO_CLR;                       /*!< Slave Clear FIFO Register                                             */
  __IO uint32_t  S0_ADDRESSB;                       /*!< Slave I2C Address B Value                                             */
  __IO uint32_t  S0_ADDRESSMASKB;                   /*!< Slave I2C Address B Mask value                                        */
  __I  uint32_t  RESERVED1[940];
  __I  uint32_t  PERID;                             /*!< Peripheral ID Register                                                */
} VOR_I2C_PERIPHERAL_Type;


/**
  * @brief Aternate Peripheral Access in BANK mode
  */
typedef struct { 
  VOR_GPIO_PERIPHERAL_Type BANK[2]; 
} VOR_GPIO_PERIPHERAL_BANK_Type;

typedef struct { 
  VOR_TIM_PERIPHERAL_Type BANK[24]; 
} VOR_TIM_PERIPHERAL_BANK_Type;

typedef struct { 
  VOR_UART_PERIPHERAL_Type BANK[2]; 
} VOR_UART_PERIPHERAL_BANK_Type;

typedef struct { 
  VOR_SPI_PERIPHERAL_Type BANK[3]; 
} VOR_SPI_PERIPHERAL_BANK_Type;

typedef struct { 
  VOR_I2C_PERIPHERAL_Type BANK[2]; 
} VOR_I2C_PERIPHERAL_BANK_Type;

#define VOR_GPIO    ((VOR_GPIO_PERIPHERAL_BANK_Type  *) VOR_PORTA_BASE)
#define VOR_TIM     ((VOR_TIM_PERIPHERAL_BANK_Type   *) VOR_TIM0_BASE)
#define VOR_UART    ((VOR_UART_PERIPHERAL_BANK_Type  *) VOR_UARTA_BASE)
#define VOR_SPI     ((VOR_SPI_PERIPHERAL_BANK_Type   *) VOR_SPIA_BASE)
#define VOR_I2C     ((VOR_I2C_PERIPHERAL_BANK_Type   *) VOR_I2CA_BASE)

#define CLK_ENABLE_PORTA   1<<0 		/*!< Clock Enable for PORTA */		
#define CLK_ENABLE_PORTB   1<<1		    /*!< Clock Enable for PORTB */
#define CLK_ENABLE_SPIA    1<<4		    /*!< Clock Enable for SPIA */
#define CLK_ENABLE_SPIB    1<<5		    /*!< Clock Enable for SPIB */
#define CLK_ENABLE_SPIC    1<<6		    /*!< Clock Enable for SPIC */	
#define CLK_ENABLE_UARTA   1<<8		    /*!< Clock Enable for UARTA */	
#define CLK_ENABLE_UARTB   1<<9		    /*!< Clock Enable for UARTB */	
#define CLK_ENABLE_I2CA    1<<16	    /*!< Clock Enable for I2CA */	
#define CLK_ENABLE_I2CB    1<<17	    /*!< Clock Enable for I2CB */	
#define CLK_ENABLE_SYSTEM  1<<20        /*!< Clock Enable for System */ 
#define CLK_ENABLE_IRQSEL  1<<21		/*!< Clock Enable for IRQSEL */	
#define CLK_ENABLE_IOMGR   1<<22		/*!< Clock Enable for IOMGR */	
// newer version of Programmers Guide calls this out at IOCONFIG instead of IOMGR (duplicating defintion)
#define CLK_ENABLE_IOCONFIG 1<<22		/*!< Clock Enable for IOCONFIG */	
#define CLK_ENABLE_UTILITY 1<<23		/*!< Clock Enable for Utility */	
// newer version of Programmers Guide calls this out at GPIO instead of PORTIO (duplicating defintion)
#define CLK_ENABLE_PORTIO  1<<24		/*!< Clock Enable for PORT IO */	
#define CLK_ENABLE_GPIO  1<<24		/*!< Clock Enable for PORT IO */	
//----------------------------------------------------------------------------
/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================       struct 'SYSCONFIG' Position & Mask       ================ */
/* ================================================================================ */


/* -----------------------------  SYSCONFIG_RST_STAT  ----------------------------- */
#define SYSCONFIG_RST_STAT_POR_Pos            0                                                       /*!< SYSCONFIG RST_STAT: POR Position        */
#define SYSCONFIG_RST_STAT_POR_Msk            (0x01UL << SYSCONFIG_RST_STAT_POR_Pos)                  /*!< SYSCONFIG RST_STAT: POR Mask            */
#define SYSCONFIG_RST_STAT_EXTRST_Pos         1                                                       /*!< SYSCONFIG RST_STAT: EXTRST Position     */
#define SYSCONFIG_RST_STAT_EXTRST_Msk         (0x01UL << SYSCONFIG_RST_STAT_EXTRST_Pos)               /*!< SYSCONFIG RST_STAT: EXTRST Mask         */
#define SYSCONFIG_RST_STAT_SYSRSTREQ_Pos      2                                                       /*!< SYSCONFIG RST_STAT: SYSRSTREQ Position  */
#define SYSCONFIG_RST_STAT_SYSRSTREQ_Msk      (0x01UL << SYSCONFIG_RST_STAT_SYSRSTREQ_Pos)            /*!< SYSCONFIG RST_STAT: SYSRSTREQ Mask      */
#define SYSCONFIG_RST_STAT_LOOKUP_Pos         3                                                       /*!< SYSCONFIG RST_STAT: LOOKUP Position     */
#define SYSCONFIG_RST_STAT_LOOKUP_Msk         (0x01UL << SYSCONFIG_RST_STAT_LOOKUP_Pos)               /*!< SYSCONFIG RST_STAT: LOOKUP Mask         */
#define SYSCONFIG_RST_STAT_WATCHDOG_Pos       4                                                       /*!< SYSCONFIG RST_STAT: WATCHDOG Position   */
#define SYSCONFIG_RST_STAT_WATCHDOG_Msk       (0x01UL << SYSCONFIG_RST_STAT_WATCHDOG_Pos)             /*!< SYSCONFIG RST_STAT: WATCHDOG Mask       */
#define SYSCONFIG_RST_STAT_MEMERR_Pos         5                                                       /*!< SYSCONFIG RST_STAT: MEMERR Position     */
#define SYSCONFIG_RST_STAT_MEMERR_Msk         (0x01UL << SYSCONFIG_RST_STAT_MEMERR_Pos)               /*!< SYSCONFIG RST_STAT: MEMERR Mask         */

/* ---------------------------  SYSCONFIG_RST_CNTL_ROM  --------------------------- */
#define SYSCONFIG_RST_CNTL_ROM_POR_Pos        0                                                       /*!< SYSCONFIG RST_CNTL_ROM: POR Position    */
#define SYSCONFIG_RST_CNTL_ROM_POR_Msk        (0x01UL << SYSCONFIG_RST_CNTL_ROM_POR_Pos)              /*!< SYSCONFIG RST_CNTL_ROM: POR Mask        */
#define SYSCONFIG_RST_CNTL_ROM_EXTRST_Pos     1                                                       /*!< SYSCONFIG RST_CNTL_ROM: EXTRST Position */
#define SYSCONFIG_RST_CNTL_ROM_EXTRST_Msk     (0x01UL << SYSCONFIG_RST_CNTL_ROM_EXTRST_Pos)           /*!< SYSCONFIG RST_CNTL_ROM: EXTRST Mask     */
#define SYSCONFIG_RST_CNTL_ROM_SYSRSTREQ_Pos  2                                                       /*!< SYSCONFIG RST_CNTL_ROM: SYSRSTREQ Position */
#define SYSCONFIG_RST_CNTL_ROM_SYSRSTREQ_Msk  (0x01UL << SYSCONFIG_RST_CNTL_ROM_SYSRSTREQ_Pos)        /*!< SYSCONFIG RST_CNTL_ROM: SYSRSTREQ Mask  */
#define SYSCONFIG_RST_CNTL_ROM_LOOKUP_Pos     3                                                       /*!< SYSCONFIG RST_CNTL_ROM: LOOKUP Position */
#define SYSCONFIG_RST_CNTL_ROM_LOOKUP_Msk     (0x01UL << SYSCONFIG_RST_CNTL_ROM_LOOKUP_Pos)           /*!< SYSCONFIG RST_CNTL_ROM: LOOKUP Mask     */
#define SYSCONFIG_RST_CNTL_ROM_WATCHDOG_Pos   4                                                       /*!< SYSCONFIG RST_CNTL_ROM: WATCHDOG Position */
#define SYSCONFIG_RST_CNTL_ROM_WATCHDOG_Msk   (0x01UL << SYSCONFIG_RST_CNTL_ROM_WATCHDOG_Pos)         /*!< SYSCONFIG RST_CNTL_ROM: WATCHDOG Mask   */
#define SYSCONFIG_RST_CNTL_ROM_MEMERR_Pos     5                                                       /*!< SYSCONFIG RST_CNTL_ROM: MEMERR Position */
#define SYSCONFIG_RST_CNTL_ROM_MEMERR_Msk     (0x01UL << SYSCONFIG_RST_CNTL_ROM_MEMERR_Pos)           /*!< SYSCONFIG RST_CNTL_ROM: MEMERR Mask     */

/* ---------------------------  SYSCONFIG_RST_CNTL_RAM  --------------------------- */
#define SYSCONFIG_RST_CNTL_RAM_POR_Pos        0                                                       /*!< SYSCONFIG RST_CNTL_RAM: POR Position    */
#define SYSCONFIG_RST_CNTL_RAM_POR_Msk        (0x01UL << SYSCONFIG_RST_CNTL_RAM_POR_Pos)              /*!< SYSCONFIG RST_CNTL_RAM: POR Mask        */
#define SYSCONFIG_RST_CNTL_RAM_EXTRST_Pos     1                                                       /*!< SYSCONFIG RST_CNTL_RAM: EXTRST Position */
#define SYSCONFIG_RST_CNTL_RAM_EXTRST_Msk     (0x01UL << SYSCONFIG_RST_CNTL_RAM_EXTRST_Pos)           /*!< SYSCONFIG RST_CNTL_RAM: EXTRST Mask     */
#define SYSCONFIG_RST_CNTL_RAM_SYSRSTREQ_Pos  2                                                       /*!< SYSCONFIG RST_CNTL_RAM: SYSRSTREQ Position */
#define SYSCONFIG_RST_CNTL_RAM_SYSRSTREQ_Msk  (0x01UL << SYSCONFIG_RST_CNTL_RAM_SYSRSTREQ_Pos)        /*!< SYSCONFIG RST_CNTL_RAM: SYSRSTREQ Mask  */
#define SYSCONFIG_RST_CNTL_RAM_LOOKUP_Pos     3                                                       /*!< SYSCONFIG RST_CNTL_RAM: LOOKUP Position */
#define SYSCONFIG_RST_CNTL_RAM_LOOKUP_Msk     (0x01UL << SYSCONFIG_RST_CNTL_RAM_LOOKUP_Pos)           /*!< SYSCONFIG RST_CNTL_RAM: LOOKUP Mask     */
#define SYSCONFIG_RST_CNTL_RAM_WATCHDOG_Pos   4                                                       /*!< SYSCONFIG RST_CNTL_RAM: WATCHDOG Position */
#define SYSCONFIG_RST_CNTL_RAM_WATCHDOG_Msk   (0x01UL << SYSCONFIG_RST_CNTL_RAM_WATCHDOG_Pos)         /*!< SYSCONFIG RST_CNTL_RAM: WATCHDOG Mask   */
#define SYSCONFIG_RST_CNTL_RAM_MEMERR_Pos     5                                                       /*!< SYSCONFIG RST_CNTL_RAM: MEMERR Position */
#define SYSCONFIG_RST_CNTL_RAM_MEMERR_Msk     (0x01UL << SYSCONFIG_RST_CNTL_RAM_MEMERR_Pos)           /*!< SYSCONFIG RST_CNTL_RAM: MEMERR Mask     */

/* -----------------------------  SYSCONFIG_ROM_PROT  ----------------------------- */
#define SYSCONFIG_ROM_PROT_WREN_Pos           0                                                       /*!< SYSCONFIG ROM_PROT: WREN Position       */
#define SYSCONFIG_ROM_PROT_WREN_Msk           (0x01UL << SYSCONFIG_ROM_PROT_WREN_Pos)                 /*!< SYSCONFIG ROM_PROT: WREN Mask           */

/* -----------------------------  SYSCONFIG_ROM_SCRUB  ---------------------------- */
#define SYSCONFIG_ROM_SCRUB_VALUE_Pos         0                                                       /*!< SYSCONFIG ROM_SCRUB: VALUE Position     */
#define SYSCONFIG_ROM_SCRUB_VALUE_Msk         (0x00ffffffUL << SYSCONFIG_ROM_SCRUB_VALUE_Pos)         /*!< SYSCONFIG ROM_SCRUB: VALUE Mask         */
#define SYSCONFIG_ROM_SCRUB_RESET_Pos         31                                                      /*!< SYSCONFIG ROM_SCRUB: RESET Position     */
#define SYSCONFIG_ROM_SCRUB_RESET_Msk         (0x01UL << SYSCONFIG_ROM_SCRUB_RESET_Pos)               /*!< SYSCONFIG ROM_SCRUB: RESET Mask         */

/* -----------------------------  SYSCONFIG_RAM_SCRUB  ---------------------------- */
#define SYSCONFIG_RAM_SCRUB_VALUE_Pos         0                                                       /*!< SYSCONFIG RAM_SCRUB: VALUE Position     */
#define SYSCONFIG_RAM_SCRUB_VALUE_Msk         (0x00ffffffUL << SYSCONFIG_RAM_SCRUB_VALUE_Pos)         /*!< SYSCONFIG RAM_SCRUB: VALUE Mask         */
#define SYSCONFIG_RAM_SCRUB_RESET_Pos         31                                                      /*!< SYSCONFIG RAM_SCRUB: RESET Position     */
#define SYSCONFIG_RAM_SCRUB_RESET_Msk         (0x01UL << SYSCONFIG_RAM_SCRUB_RESET_Pos)               /*!< SYSCONFIG RAM_SCRUB: RESET Mask         */

/* ---------------------------  SYSCONFIG_ROM_TRAP_ADDR  -------------------------- */
#define SYSCONFIG_ROM_TRAP_ADDR_ADDR_Pos      2                                                       /*!< SYSCONFIG ROM_TRAP_ADDR: ADDR Position  */
#define SYSCONFIG_ROM_TRAP_ADDR_ADDR_Msk      (0x00003fffUL << SYSCONFIG_ROM_TRAP_ADDR_ADDR_Pos)      /*!< SYSCONFIG ROM_TRAP_ADDR: ADDR Mask      */
#define SYSCONFIG_ROM_TRAP_ADDR_ENABLE_Pos    31                                                      /*!< SYSCONFIG ROM_TRAP_ADDR: ENABLE Position */
#define SYSCONFIG_ROM_TRAP_ADDR_ENABLE_Msk    (0x01UL << SYSCONFIG_ROM_TRAP_ADDR_ENABLE_Pos)          /*!< SYSCONFIG ROM_TRAP_ADDR: ENABLE Mask    */

/* ---------------------------  SYSCONFIG_ROM_TRAP_SYND  -------------------------- */
#define SYSCONFIG_ROM_TRAP_SYND_SYND_Pos      0                                                       /*!< SYSCONFIG ROM_TRAP_SYND: SYND Position  */
#define SYSCONFIG_ROM_TRAP_SYND_SYND_Msk      (0x000fffffUL << SYSCONFIG_ROM_TRAP_SYND_SYND_Pos)      /*!< SYSCONFIG ROM_TRAP_SYND: SYND Mask      */

/* ---------------------------  SYSCONFIG_RAM_TRAP_ADDR  -------------------------- */
#define SYSCONFIG_RAM_TRAP_ADDR_ADDR_Pos      2                                                       /*!< SYSCONFIG RAM_TRAP_ADDR: ADDR Position  */
#define SYSCONFIG_RAM_TRAP_ADDR_ADDR_Msk      (0x00003fffUL << SYSCONFIG_RAM_TRAP_ADDR_ADDR_Pos)      /*!< SYSCONFIG RAM_TRAP_ADDR: ADDR Mask      */
#define SYSCONFIG_RAM_TRAP_ADDR_ENABLE_Pos    31                                                      /*!< SYSCONFIG RAM_TRAP_ADDR: ENABLE Position */
#define SYSCONFIG_RAM_TRAP_ADDR_ENABLE_Msk    (0x01UL << SYSCONFIG_RAM_TRAP_ADDR_ENABLE_Pos)          /*!< SYSCONFIG RAM_TRAP_ADDR: ENABLE Mask    */

/* ---------------------------  SYSCONFIG_RAM_TRAP_SYND  -------------------------- */
#define SYSCONFIG_RAM_TRAP_SYND_SYND_Pos      0                                                       /*!< SYSCONFIG RAM_TRAP_SYND: SYND Position  */
#define SYSCONFIG_RAM_TRAP_SYND_SYND_Msk      (0x000fffffUL << SYSCONFIG_RAM_TRAP_SYND_SYND_Pos)      /*!< SYSCONFIG RAM_TRAP_SYND: SYND Mask      */

/* ------------------------------  SYSCONFIG_IRQ_ENB  ----------------------------- */
#define SYSCONFIG_IRQ_ENB_RAMSBE_Pos          0                                                       /*!< SYSCONFIG IRQ_ENB: RAMSBE Position      */
#define SYSCONFIG_IRQ_ENB_RAMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_ENB_RAMSBE_Pos)                /*!< SYSCONFIG IRQ_ENB: RAMSBE Mask          */
#define SYSCONFIG_IRQ_ENB_RAMMBE_Pos          1                                                       /*!< SYSCONFIG IRQ_ENB: RAMMBE Position      */
#define SYSCONFIG_IRQ_ENB_RAMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_ENB_RAMMBE_Pos)                /*!< SYSCONFIG IRQ_ENB: RAMMBE Mask          */
#define SYSCONFIG_IRQ_ENB_ROMSBE_Pos          2                                                       /*!< SYSCONFIG IRQ_ENB: ROMSBE Position      */
#define SYSCONFIG_IRQ_ENB_ROMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_ENB_ROMSBE_Pos)                /*!< SYSCONFIG IRQ_ENB: ROMSBE Mask          */
#define SYSCONFIG_IRQ_ENB_ROMMBE_Pos          3                                                       /*!< SYSCONFIG IRQ_ENB: ROMMBE Position      */
#define SYSCONFIG_IRQ_ENB_ROMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_ENB_ROMMBE_Pos)                /*!< SYSCONFIG IRQ_ENB: ROMMBE Mask          */

/* ------------------------------  SYSCONFIG_IRQ_RAW  ----------------------------- */
#define SYSCONFIG_IRQ_RAW_RAMSBE_Pos          0                                                       /*!< SYSCONFIG IRQ_RAW: RAMSBE Position      */
#define SYSCONFIG_IRQ_RAW_RAMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_RAW_RAMSBE_Pos)                /*!< SYSCONFIG IRQ_RAW: RAMSBE Mask          */
#define SYSCONFIG_IRQ_RAW_RAMMBE_Pos          1                                                       /*!< SYSCONFIG IRQ_RAW: RAMMBE Position      */
#define SYSCONFIG_IRQ_RAW_RAMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_RAW_RAMMBE_Pos)                /*!< SYSCONFIG IRQ_RAW: RAMMBE Mask          */
#define SYSCONFIG_IRQ_RAW_ROMSBE_Pos          2                                                       /*!< SYSCONFIG IRQ_RAW: ROMSBE Position      */
#define SYSCONFIG_IRQ_RAW_ROMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_RAW_ROMSBE_Pos)                /*!< SYSCONFIG IRQ_RAW: ROMSBE Mask          */
#define SYSCONFIG_IRQ_RAW_ROMMBE_Pos          3                                                       /*!< SYSCONFIG IRQ_RAW: ROMMBE Position      */
#define SYSCONFIG_IRQ_RAW_ROMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_RAW_ROMMBE_Pos)                /*!< SYSCONFIG IRQ_RAW: ROMMBE Mask          */

/* ------------------------------  SYSCONFIG_IRQ_END  ----------------------------- */
#define SYSCONFIG_IRQ_END_RAMSBE_Pos          0                                                       /*!< SYSCONFIG IRQ_END: RAMSBE Position      */
#define SYSCONFIG_IRQ_END_RAMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_END_RAMSBE_Pos)                /*!< SYSCONFIG IRQ_END: RAMSBE Mask          */
#define SYSCONFIG_IRQ_END_RAMMBE_Pos          1                                                       /*!< SYSCONFIG IRQ_END: RAMMBE Position      */
#define SYSCONFIG_IRQ_END_RAMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_END_RAMMBE_Pos)                /*!< SYSCONFIG IRQ_END: RAMMBE Mask          */
#define SYSCONFIG_IRQ_END_ROMSBE_Pos          2                                                       /*!< SYSCONFIG IRQ_END: ROMSBE Position      */
#define SYSCONFIG_IRQ_END_ROMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_END_ROMSBE_Pos)                /*!< SYSCONFIG IRQ_END: ROMSBE Mask          */
#define SYSCONFIG_IRQ_END_ROMMBE_Pos          3                                                       /*!< SYSCONFIG IRQ_END: ROMMBE Position      */
#define SYSCONFIG_IRQ_END_ROMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_END_ROMMBE_Pos)                /*!< SYSCONFIG IRQ_END: ROMMBE Mask          */

/* ------------------------------  SYSCONFIG_IRQ_CLR  ----------------------------- */
#define SYSCONFIG_IRQ_CLR_RAMSBE_Pos          0                                                       /*!< SYSCONFIG IRQ_CLR: RAMSBE Position      */
#define SYSCONFIG_IRQ_CLR_RAMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_CLR_RAMSBE_Pos)                /*!< SYSCONFIG IRQ_CLR: RAMSBE Mask          */
#define SYSCONFIG_IRQ_CLR_RAMMBE_Pos          1                                                       /*!< SYSCONFIG IRQ_CLR: RAMMBE Position      */
#define SYSCONFIG_IRQ_CLR_RAMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_CLR_RAMMBE_Pos)                /*!< SYSCONFIG IRQ_CLR: RAMMBE Mask          */
#define SYSCONFIG_IRQ_CLR_ROMSBE_Pos          2                                                       /*!< SYSCONFIG IRQ_CLR: ROMSBE Position      */
#define SYSCONFIG_IRQ_CLR_ROMSBE_Msk          (0x01UL << SYSCONFIG_IRQ_CLR_ROMSBE_Pos)                /*!< SYSCONFIG IRQ_CLR: ROMSBE Mask          */
#define SYSCONFIG_IRQ_CLR_ROMMBE_Pos          3                                                       /*!< SYSCONFIG IRQ_CLR: ROMMBE Position      */
#define SYSCONFIG_IRQ_CLR_ROMMBE_Msk          (0x01UL << SYSCONFIG_IRQ_CLR_ROMMBE_Pos)                /*!< SYSCONFIG IRQ_CLR: ROMMBE Mask          */

/* ---------------------------  SYSCONFIG_LOCKUP_RESET  --------------------------- */
#define SYSCONFIG_LOCKUP_RESET_LREN_Pos       0                                                       /*!< SYSCONFIG LOCKUP_RESET: LREN Position   */
#define SYSCONFIG_LOCKUP_RESET_LREN_Msk       (0x01UL << SYSCONFIG_LOCKUP_RESET_LREN_Pos)             /*!< SYSCONFIG LOCKUP_RESET: LREN Mask       */


/* ================================================================================ */
/* ================         struct 'IRQSEL' Position & Mask        ================ */
/* ================================================================================ */


/* ---------------------------------  IRQSEL_IRQS  -------------------------------- */
#define IRQSEL_IRQS_ACTVIE_Pos                0                                                       /*!< IRQSEL IRQS: ACTVIE Position            */
#define IRQSEL_IRQS_ACTVIE_Msk                (0x01UL << IRQSEL_IRQS_ACTVIE_Pos)                      /*!< IRQSEL IRQS: ACTVIE Mask                */

/* --------------------------------  IRQSEL_EDBGRQ  ------------------------------- */
#define IRQSEL_EDBGRQ_ACTVIE_Pos              0                                                       /*!< IRQSEL EDBGRQ: ACTVIE Position          */
#define IRQSEL_EDBGRQ_ACTVIE_Msk              (0x01UL << IRQSEL_EDBGRQ_ACTVIE_Pos)                    /*!< IRQSEL EDBGRQ: ACTVIE Mask              */

/* -------------------------------  IRQSEL_MERESET  ------------------------------- */
#define IRQSEL_MERESET_ACTVIE_Pos             0                                                       /*!< IRQSEL MERESET: ACTVIE Position         */
#define IRQSEL_MERESET_ACTVIE_Msk             (0x01UL << IRQSEL_MERESET_ACTVIE_Pos)                   /*!< IRQSEL MERESET: ACTVIE Mask             */

/* -------------------------------  IRQSEL_WATCHDOG  ------------------------------ */
#define IRQSEL_WATCHDOG_ACTVIE_Pos            0                                                       /*!< IRQSEL WATCHDOG: ACTVIE Position        */
#define IRQSEL_WATCHDOG_ACTVIE_Msk            (0x01UL << IRQSEL_WATCHDOG_ACTVIE_Pos)                  /*!< IRQSEL WATCHDOG: ACTVIE Mask            */

/* ---------------------------------  IRQSEL_RXEV  -------------------------------- */
#define IRQSEL_RXEV_ACTVIE_Pos                0                                                       /*!< IRQSEL RXEV: ACTVIE Position            */
#define IRQSEL_RXEV_ACTVIE_Msk                (0x01UL << IRQSEL_RXEV_ACTVIE_Pos)                      /*!< IRQSEL RXEV: ACTVIE Mask                */

/* ---------------------------------  IRQSEL_NMI  --------------------------------- */
#define IRQSEL_NMI_ACTVIE_Pos                 0                                                       /*!< IRQSEL NMI: ACTVIE Position             */
#define IRQSEL_NMI_ACTVIE_Msk                 (0x01UL << IRQSEL_NMI_ACTVIE_Pos)                       /*!< IRQSEL NMI: ACTVIE Mask                 */


/* ================================================================================ */
/* ================        struct 'IOCONFIG' Position & Mask       ================ */
/* ================================================================================ */


/* -------------------------------  IOCONFIG_PORTA  ------------------------------- */
#define IOCONFIG_PORTA_FLTTYPE_Pos            0                                                       /*!< IOCONFIG PORTA: FLTTYPE Position        */
#define IOCONFIG_PORTA_FLTTYPE_Msk            (0x07UL << IOCONFIG_PORTA_FLTTYPE_Pos)                  /*!< IOCONFIG PORTA: FLTTYPE Mask            */
#define IOCONFIG_PORTA_FLTCLK_Pos             3                                                       /*!< IOCONFIG PORTA: FLTCLK Position         */
#define IOCONFIG_PORTA_FLTCLK_Msk             (0x07UL << IOCONFIG_PORTA_FLTCLK_Pos)                   /*!< IOCONFIG PORTA: FLTCLK Mask             */
#define IOCONFIG_PORTA_INVINP_Pos             6                                                       /*!< IOCONFIG PORTA: INVINP Position         */
#define IOCONFIG_PORTA_INVINP_Msk             (0x01UL << IOCONFIG_PORTA_INVINP_Pos)                   /*!< IOCONFIG PORTA: INVINP Mask             */
#define IOCONFIG_PORTA_IEWO_Pos               7                                                       /*!< IOCONFIG PORTA: IEWO Position           */
#define IOCONFIG_PORTA_IEWO_Msk               (0x01UL << IOCONFIG_PORTA_IEWO_Pos)                     /*!< IOCONFIG PORTA: IEWO Mask               */
#define IOCONFIG_PORTA_OPENDRN_Pos            8                                                       /*!< IOCONFIG PORTA: OPENDRN Position        */
#define IOCONFIG_PORTA_OPENDRN_Msk            (0x01UL << IOCONFIG_PORTA_OPENDRN_Pos)                  /*!< IOCONFIG PORTA: OPENDRN Mask            */
#define IOCONFIG_PORTA_INVOUT_Pos             9                                                       /*!< IOCONFIG PORTA: INVOUT Position         */
#define IOCONFIG_PORTA_INVOUT_Msk             (0x01UL << IOCONFIG_PORTA_INVOUT_Pos)                   /*!< IOCONFIG PORTA: INVOUT Mask             */
#define IOCONFIG_PORTA_PLEVEL_Pos             10                                                      /*!< IOCONFIG PORTA: PLEVEL Position         */
#define IOCONFIG_PORTA_PLEVEL_Msk             (0x01UL << IOCONFIG_PORTA_PLEVEL_Pos)                   /*!< IOCONFIG PORTA: PLEVEL Mask             */
#define IOCONFIG_PORTA_PEN_Pos                11                                                      /*!< IOCONFIG PORTA: PEN Position            */
#define IOCONFIG_PORTA_PEN_Msk                (0x01UL << IOCONFIG_PORTA_PEN_Pos)                      /*!< IOCONFIG PORTA: PEN Mask                */
#define IOCONFIG_PORTA_PWOA_Pos               12                                                      /*!< IOCONFIG PORTA: PWOA Position           */
#define IOCONFIG_PORTA_PWOA_Msk               (0x01UL << IOCONFIG_PORTA_PWOA_Pos)                     /*!< IOCONFIG PORTA: PWOA Mask               */
#define IOCONFIG_PORTA_FUNSEL_Pos             13                                                      /*!< IOCONFIG PORTA: FUNSEL Position         */
#define IOCONFIG_PORTA_FUNSEL_Msk             (0x07UL << IOCONFIG_PORTA_FUNSEL_Pos)                   /*!< IOCONFIG PORTA: FUNSEL Mask             */
#define IOCONFIG_PORTA_IODIS_Pos              16                                                      /*!< IOCONFIG PORTA: IODIS Position          */
#define IOCONFIG_PORTA_IODIS_Msk              (0x01UL << IOCONFIG_PORTA_IODIS_Pos)                    /*!< IOCONFIG PORTA: IODIS Mask              */

/* -------------------------------  IOCONFIG_PORTB  ------------------------------- */
#define IOCONFIG_PORTB_FLTTYPE_Pos            0                                                       /*!< IOCONFIG PORTB: FLTTYPE Position        */
#define IOCONFIG_PORTB_FLTTYPE_Msk            (0x07UL << IOCONFIG_PORTB_FLTTYPE_Pos)                  /*!< IOCONFIG PORTB: FLTTYPE Mask            */
#define IOCONFIG_PORTB_FLTCLK_Pos             3                                                       /*!< IOCONFIG PORTB: FLTCLK Position         */
#define IOCONFIG_PORTB_FLTCLK_Msk             (0x07UL << IOCONFIG_PORTB_FLTCLK_Pos)                   /*!< IOCONFIG PORTB: FLTCLK Mask             */
#define IOCONFIG_PORTB_INVINP_Pos             6                                                       /*!< IOCONFIG PORTB: INVINP Position         */
#define IOCONFIG_PORTB_INVINP_Msk             (0x01UL << IOCONFIG_PORTB_INVINP_Pos)                   /*!< IOCONFIG PORTB: INVINP Mask             */
#define IOCONFIG_PORTB_IEWO_Pos               7                                                       /*!< IOCONFIG PORTB: IEWO Position           */
#define IOCONFIG_PORTB_IEWO_Msk               (0x01UL << IOCONFIG_PORTB_IEWO_Pos)                     /*!< IOCONFIG PORTB: IEWO Mask               */
#define IOCONFIG_PORTB_OPENDRN_Pos            8                                                       /*!< IOCONFIG PORTB: OPENDRN Position        */
#define IOCONFIG_PORTB_OPENDRN_Msk            (0x01UL << IOCONFIG_PORTB_OPENDRN_Pos)                  /*!< IOCONFIG PORTB: OPENDRN Mask            */
#define IOCONFIG_PORTB_INVOUT_Pos             9                                                       /*!< IOCONFIG PORTB: INVOUT Position         */
#define IOCONFIG_PORTB_INVOUT_Msk             (0x01UL << IOCONFIG_PORTB_INVOUT_Pos)                   /*!< IOCONFIG PORTB: INVOUT Mask             */
#define IOCONFIG_PORTB_PLEVEL_Pos             10                                                      /*!< IOCONFIG PORTB: PLEVEL Position         */
#define IOCONFIG_PORTB_PLEVEL_Msk             (0x01UL << IOCONFIG_PORTB_PLEVEL_Pos)                   /*!< IOCONFIG PORTB: PLEVEL Mask             */
#define IOCONFIG_PORTB_PEN_Pos                11                                                      /*!< IOCONFIG PORTB: PEN Position            */
#define IOCONFIG_PORTB_PEN_Msk                (0x01UL << IOCONFIG_PORTB_PEN_Pos)                      /*!< IOCONFIG PORTB: PEN Mask                */
#define IOCONFIG_PORTB_PWOA_Pos               12                                                      /*!< IOCONFIG PORTB: PWOA Position           */
#define IOCONFIG_PORTB_PWOA_Msk               (0x01UL << IOCONFIG_PORTB_PWOA_Pos)                     /*!< IOCONFIG PORTB: PWOA Mask               */
#define IOCONFIG_PORTB_FUNSEL_Pos             13                                                      /*!< IOCONFIG PORTB: FUNSEL Position         */
#define IOCONFIG_PORTB_FUNSEL_Msk             (0x07UL << IOCONFIG_PORTB_FUNSEL_Pos)                   /*!< IOCONFIG PORTB: FUNSEL Mask             */
#define IOCONFIG_PORTB_IODIS_Pos              16                                                      /*!< IOCONFIG PORTB: IODIS Position          */
#define IOCONFIG_PORTB_IODIS_Msk              (0x01UL << IOCONFIG_PORTB_IODIS_Pos)                    /*!< IOCONFIG PORTB: IODIS Mask              */


/* ================================================================================ */
/* ================     Group 'TIM_PERIPHERAL' Position & Mask     ================ */
/* ================================================================================ */


/* -----------------------------  TIM_PERIPHERAL_CTRL  ---------------------------- */
#define TIM_PERIPHERAL_CTRL_ENABLE_Pos        0                                                       /*!< TIM_PERIPHERAL CTRL: ENABLE Position    */
#define TIM_PERIPHERAL_CTRL_ENABLE_Msk        (0x01UL << TIM_PERIPHERAL_CTRL_ENABLE_Pos)              /*!< TIM_PERIPHERAL CTRL: ENABLE Mask        */
#define TIM_PERIPHERAL_CTRL_ACTIVE_Pos        1                                                       /*!< TIM_PERIPHERAL CTRL: ACTIVE Position    */
#define TIM_PERIPHERAL_CTRL_ACTIVE_Msk        (0x01UL << TIM_PERIPHERAL_CTRL_ACTIVE_Pos)              /*!< TIM_PERIPHERAL CTRL: ACTIVE Mask        */
#define TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Pos  2                                                       /*!< TIM_PERIPHERAL CTRL: AUTO_DISABLE Position */
#define TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Msk  (0x01UL << TIM_PERIPHERAL_CTRL_AUTO_DISABLE_Pos)        /*!< TIM_PERIPHERAL CTRL: AUTO_DISABLE Mask  */
#define TIM_PERIPHERAL_CTRL_AUTO_DEACTIVATE_Pos 3                                                     /*!< TIM_PERIPHERAL CTRL: AUTO_DEACTIVATE Position */
#define TIM_PERIPHERAL_CTRL_AUTO_DEACTIVATE_Msk (0x01UL << TIM_PERIPHERAL_CTRL_AUTO_DEACTIVATE_Pos)   /*!< TIM_PERIPHERAL CTRL: AUTO_DEACTIVATE Mask */
#define TIM_PERIPHERAL_CTRL_IRQ_ENB_Pos       4                                                       /*!< TIM_PERIPHERAL CTRL: IRQ_ENB Position   */
#define TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk       (0x01UL << TIM_PERIPHERAL_CTRL_IRQ_ENB_Pos)             /*!< TIM_PERIPHERAL CTRL: IRQ_ENB Mask       */
#define TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos    5                                                       /*!< TIM_PERIPHERAL CTRL: STATUS_SEL Position */
#define TIM_PERIPHERAL_CTRL_STATUS_SEL_Msk    (0x07UL << TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos)          /*!< TIM_PERIPHERAL CTRL: STATUS_SEL Mask    */
#define TIM_PERIPHERAL_CTRL_STATUS_INV_Pos    8                                                       /*!< TIM_PERIPHERAL CTRL: STATUS_INV Position */
#define TIM_PERIPHERAL_CTRL_STATUS_INV_Msk    (0x01UL << TIM_PERIPHERAL_CTRL_STATUS_INV_Pos)          /*!< TIM_PERIPHERAL CTRL: STATUS_INV Mask    */
#define TIM_PERIPHERAL_CTRL_REQ_STOP_Pos      9                                                       /*!< TIM_PERIPHERAL CTRL: REQ_STOP Position  */
#define TIM_PERIPHERAL_CTRL_REQ_STOP_Msk      (0x01UL << TIM_PERIPHERAL_CTRL_REQ_STOP_Pos)            /*!< TIM_PERIPHERAL CTRL: REQ_STOP Mask      */

#define STATUS_SEL_ONECYCLE   0         /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_OUTPUT_ACTIVE 1      /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_TOGGLE 2             /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_PWMA 3               /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_PWMB 4               /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_OUTPUT_ENABLE 5      /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */
#define STATUS_SEL_PWMA_ACTIVE 6        /*!< TIM_PERIPHERAL CTRL: STATUS SELECT options    */

/* ----------------------------  TIM_PERIPHERAL_ENABLE  --------------------------- */
#define TIM_PERIPHERAL_ENABLE_ENABLE_Pos      0                                                       /*!< TIM_PERIPHERAL ENABLE: ENABLE Position  */
#define TIM_PERIPHERAL_ENABLE_ENABLE_Msk      (0x01UL << TIM_PERIPHERAL_ENABLE_ENABLE_Pos)            /*!< TIM_PERIPHERAL ENABLE: ENABLE Mask      */

/* ---------------------------  TIM_PERIPHERAL_CSD_CTRL  -------------------------- */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Pos    0                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN0 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk    (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Pos)          /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN0 Mask    */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV0_Pos   1                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV0 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV0_Msk   (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDINV0_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV0 Mask   */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN1_Pos    2                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN1 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN1_Msk    (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDEN1_Pos)          /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN1 Mask    */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV1_Pos   3                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV1 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV1_Msk   (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDINV1_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV1 Mask   */
#define TIM_PERIPHERAL_CSD_CTRL_DCASOP_Pos    4                                                       /*!< TIM_PERIPHERAL CSD_CTRL: DCASOP Position */
#define TIM_PERIPHERAL_CSD_CTRL_DCASOP_Msk    (0x01UL << TIM_PERIPHERAL_CSD_CTRL_DCASOP_Pos)          /*!< TIM_PERIPHERAL CSD_CTRL: DCASOP Mask    */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Pos   6                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG0 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk   (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG0 Mask   */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG1_Pos   7                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG1 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG1_Msk   (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDTRG1_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG1 Mask   */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN2_Pos    8                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN2 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDEN2_Msk    (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDEN2_Pos)          /*!< TIM_PERIPHERAL CSD_CTRL: CSDEN2 Mask    */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV2_Pos   9                                                       /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV2 Position */
#define TIM_PERIPHERAL_CSD_CTRL_CSDINV2_Msk   (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDINV2_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDINV2 Mask   */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG2_Pos            10                                             /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG2 Position        */
#define TIM_PERIPHERAL_CSD_CTRL_CSDTRG2_Msk  (0x01UL << TIM_PERIPHERAL_CSD_CTRL_CSDTRG2_Pos)         /*!< TIM_PERIPHERAL CSD_CTRL: CSDTRG2 Mask            */


/* ---------------------------  TIM_PERIPHERAL_CASCADE0  -------------------------- */
#define TIM_PERIPHERAL_CASCADE0_CASSEL_Pos    0                                                       /*!< TIM_PERIPHERAL CASCADE0: CASSEL Position */
#define TIM_PERIPHERAL_CASCADE0_CASSEL_Msk    (0x000000ffUL << TIM_PERIPHERAL_CASCADE0_CASSEL_Pos)    /*!< TIM_PERIPHERAL CASCADE0: CASSEL Mask    */

/* ---------------------------  TIM_PERIPHERAL_CASCADE1  -------------------------- */
#define TIM_PERIPHERAL_CASCADE1_CASSEL_Pos    0                                                       /*!< TIM_PERIPHERAL CASCADE1: CASSEL Position */
#define TIM_PERIPHERAL_CASCADE1_CASSEL_Msk    (0x000000ffUL << TIM_PERIPHERAL_CASCADE1_CASSEL_Pos)    /*!< TIM_PERIPHERAL CASCADE1: CASSEL Mask    */

/* ---------------------------  TIM_PERIPHERAL_CASCADE2  -------------------------- */
#define TIM_PERIPHERAL_CASCADE2_CASSEL_Pos    0                                                       /*!< TIM_PERIPHERAL CASCADE2: CASSEL Position */
#define TIM_PERIPHERAL_CASCADE2_CASSEL_Msk    (0x000000ffUL << TIM_PERIPHERAL_CASCADE2_CASSEL_Pos)    /*!< TIM_PERIPHERAL CASCADE2: CASSEL Mask    */


/* ================================================================================ */
/* ================          struct 'TIM0' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM0_CTRL  --------------------------------- */
#define TIM0_CTRL_ENABLE_Pos                  0                                                       /*!< TIM0 CTRL: ENABLE Position              */
#define TIM0_CTRL_ENABLE_Msk                  (0x01UL << TIM0_CTRL_ENABLE_Pos)                        /*!< TIM0 CTRL: ENABLE Mask                  */
#define TIM0_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM0 CTRL: ACTIVE Position              */
#define TIM0_CTRL_ACTIVE_Msk                  (0x01UL << TIM0_CTRL_ACTIVE_Pos)                        /*!< TIM0 CTRL: ACTIVE Mask                  */
#define TIM0_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM0 CTRL: AUTO_DISABLE Position        */
#define TIM0_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM0_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM0 CTRL: AUTO_DISABLE Mask            */
#define TIM0_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM0 CTRL: AUTO_DEACTIVATE Position     */
#define TIM0_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM0_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM0 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM0_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM0 CTRL: IRQ_ENB Position             */
#define TIM0_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM0_CTRL_IRQ_ENB_Pos)                       /*!< TIM0 CTRL: IRQ_ENB Mask                 */
#define TIM0_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM0 CTRL: STATUS_SEL Position          */
#define TIM0_CTRL_STATUS_SEL_Msk              (0x07UL << TIM0_CTRL_STATUS_SEL_Pos)                    /*!< TIM0 CTRL: STATUS_SEL Mask              */
#define TIM0_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM0 CTRL: STATUS_INV Position          */
#define TIM0_CTRL_STATUS_INV_Msk              (0x01UL << TIM0_CTRL_STATUS_INV_Pos)                    /*!< TIM0 CTRL: STATUS_INV Mask              */
#define TIM0_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM0 CTRL: REQ_STOP Position            */
#define TIM0_CTRL_REQ_STOP_Msk                (0x01UL << TIM0_CTRL_REQ_STOP_Pos)                      /*!< TIM0 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM0_ENABLE  -------------------------------- */
#define TIM0_ENABLE_ENABLE_Pos                0                                                       /*!< TIM0 ENABLE: ENABLE Position            */
#define TIM0_ENABLE_ENABLE_Msk                (0x01UL << TIM0_ENABLE_ENABLE_Pos)                      /*!< TIM0 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM0_CSD_CTRL  ------------------------------- */
#define TIM0_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM0 CSD_CTRL: CSDEN0 Position          */
#define TIM0_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM0_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM0 CSD_CTRL: CSDEN0 Mask              */
#define TIM0_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM0 CSD_CTRL: CSDINV0 Position         */
#define TIM0_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM0_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM0 CSD_CTRL: CSDINV0 Mask             */
#define TIM0_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM0 CSD_CTRL: CSDEN1 Position          */
#define TIM0_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM0_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM0 CSD_CTRL: CSDEN1 Mask              */
#define TIM0_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM0 CSD_CTRL: CSDINV1 Position         */
#define TIM0_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM0_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM0 CSD_CTRL: CSDINV1 Mask             */
#define TIM0_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM0 CSD_CTRL: DCASOP Position          */
#define TIM0_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM0_CSD_CTRL_DCASOP_Pos)                    /*!< TIM0 CSD_CTRL: DCASOP Mask              */
#define TIM0_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM0 CSD_CTRL: CSDTRG0 Position         */
#define TIM0_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM0_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM0 CSD_CTRL: CSDTRG0 Mask             */
#define TIM0_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM0 CSD_CTRL: CSDTRG1 Position         */
#define TIM0_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM0_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM0 CSD_CTRL: CSDTRG1 Mask             */
#define TIM0_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM0 CSD_CTRL: CSDEN2 Position          */
#define TIM0_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM0_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM0 CSD_CTRL: CSDEN2 Mask              */
#define TIM0_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM0 CSD_CTRL: CSDINV2 Position         */
#define TIM0_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM0_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM0 CSD_CTRL: CSDINV2 Mask             */
#define TIM0_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM0 CSD_CTRL: CSDTRG2 Position        */
#define TIM0_CSD_CTRL_CSDTRG2_Msk             (0x01UL << TIM0_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM0 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM0_CASCADE0  ------------------------------- */
#define TIM0_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM0 CASCADE0: CASSEL Position          */
#define TIM0_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM0_CASCADE0_CASSEL_Pos)              /*!< TIM0 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM0_CASCADE1  ------------------------------- */
#define TIM0_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM0 CASCADE1: CASSEL Position          */
#define TIM0_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM0_CASCADE1_CASSEL_Pos)              /*!< TIM0 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM0_CASCADE2  ------------------------------- */
#define TIM0_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM0 CASCADE2: CASSEL Position          */
#define TIM0_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM0_CASCADE2_CASSEL_Pos)              /*!< TIM0 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM1' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM1_CTRL  --------------------------------- */
#define TIM1_CTRL_ENABLE_Pos                  0                                                       /*!< TIM1 CTRL: ENABLE Position              */
#define TIM1_CTRL_ENABLE_Msk                  (0x01UL << TIM1_CTRL_ENABLE_Pos)                        /*!< TIM1 CTRL: ENABLE Mask                  */
#define TIM1_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM1 CTRL: ACTIVE Position              */
#define TIM1_CTRL_ACTIVE_Msk                  (0x01UL << TIM1_CTRL_ACTIVE_Pos)                        /*!< TIM1 CTRL: ACTIVE Mask                  */
#define TIM1_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM1 CTRL: AUTO_DISABLE Position        */
#define TIM1_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM1_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM1 CTRL: AUTO_DISABLE Mask            */
#define TIM1_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM1 CTRL: AUTO_DEACTIVATE Position     */
#define TIM1_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM1_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM1 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM1_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM1 CTRL: IRQ_ENB Position             */
#define TIM1_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM1_CTRL_IRQ_ENB_Pos)                       /*!< TIM1 CTRL: IRQ_ENB Mask                 */
#define TIM1_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM1 CTRL: STATUS_SEL Position          */
#define TIM1_CTRL_STATUS_SEL_Msk              (0x07UL << TIM1_CTRL_STATUS_SEL_Pos)                    /*!< TIM1 CTRL: STATUS_SEL Mask              */
#define TIM1_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM1 CTRL: STATUS_INV Position          */
#define TIM1_CTRL_STATUS_INV_Msk              (0x01UL << TIM1_CTRL_STATUS_INV_Pos)                    /*!< TIM1 CTRL: STATUS_INV Mask              */
#define TIM1_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM1 CTRL: REQ_STOP Position            */
#define TIM1_CTRL_REQ_STOP_Msk                (0x01UL << TIM1_CTRL_REQ_STOP_Pos)                      /*!< TIM1 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM1_ENABLE  -------------------------------- */
#define TIM1_ENABLE_ENABLE_Pos                0                                                       /*!< TIM1 ENABLE: ENABLE Position            */
#define TIM1_ENABLE_ENABLE_Msk                (0x01UL << TIM1_ENABLE_ENABLE_Pos)                      /*!< TIM1 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM1_CSD_CTRL  ------------------------------- */
#define TIM1_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM1 CSD_CTRL: CSDEN0 Position          */
#define TIM1_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM1_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM1 CSD_CTRL: CSDEN0 Mask              */
#define TIM1_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM1 CSD_CTRL: CSDINV0 Position         */
#define TIM1_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM1_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM1 CSD_CTRL: CSDINV0 Mask             */
#define TIM1_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM1 CSD_CTRL: CSDEN1 Position          */
#define TIM1_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM1_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM1 CSD_CTRL: CSDEN1 Mask              */
#define TIM1_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM1 CSD_CTRL: CSDINV1 Position         */
#define TIM1_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM1_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM1 CSD_CTRL: CSDINV1 Mask             */
#define TIM1_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM1 CSD_CTRL: DCASOP Position          */
#define TIM1_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM1_CSD_CTRL_DCASOP_Pos)                    /*!< TIM1 CSD_CTRL: DCASOP Mask              */
#define TIM1_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM1 CSD_CTRL: CSDTRG0 Position         */
#define TIM1_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM1_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM1 CSD_CTRL: CSDTRG0 Mask             */
#define TIM1_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM1 CSD_CTRL: CSDTRG1 Position         */
#define TIM1_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM1_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM1 CSD_CTRL: CSDTRG1 Mask             */
#define TIM1_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM1 CSD_CTRL: CSDEN2 Position          */
#define TIM1_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM1_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM1 CSD_CTRL: CSDEN2 Mask              */
#define TIM1_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM1 CSD_CTRL: CSDINV2 Position         */
#define TIM1_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM1_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM1 CSD_CTRL: CSDINV2 Mask             */
#define TIM1_CSD_CTRL_CSDTRG2_Pos             10                                                      /*!< TIM1 CSD_CTRL: CSDTRG2 Position        */
#define TIM1_CSD_CTRL_CSDTRG2_Msk             (0x01UL << TIM1_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM1 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM1_CASCADE0  ------------------------------- */
#define TIM1_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM1 CASCADE0: CASSEL Position          */
#define TIM1_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM1_CASCADE0_CASSEL_Pos)              /*!< TIM1 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM1_CASCADE1  ------------------------------- */
#define TIM1_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM1 CASCADE1: CASSEL Position          */
#define TIM1_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM1_CASCADE1_CASSEL_Pos)              /*!< TIM1 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM1_CASCADE2  ------------------------------- */
#define TIM1_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM1 CASCADE2: CASSEL Position          */
#define TIM1_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM1_CASCADE2_CASSEL_Pos)              /*!< TIM1 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM2' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM2_CTRL  --------------------------------- */
#define TIM2_CTRL_ENABLE_Pos                  0                                                       /*!< TIM2 CTRL: ENABLE Position              */
#define TIM2_CTRL_ENABLE_Msk                  (0x01UL << TIM2_CTRL_ENABLE_Pos)                        /*!< TIM2 CTRL: ENABLE Mask                  */
#define TIM2_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM2 CTRL: ACTIVE Position              */
#define TIM2_CTRL_ACTIVE_Msk                  (0x01UL << TIM2_CTRL_ACTIVE_Pos)                        /*!< TIM2 CTRL: ACTIVE Mask                  */
#define TIM2_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM2 CTRL: AUTO_DISABLE Position        */
#define TIM2_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM2_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM2 CTRL: AUTO_DISABLE Mask            */
#define TIM2_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM2 CTRL: AUTO_DEACTIVATE Position     */
#define TIM2_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM2_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM2 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM2_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM2 CTRL: IRQ_ENB Position             */
#define TIM2_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM2_CTRL_IRQ_ENB_Pos)                       /*!< TIM2 CTRL: IRQ_ENB Mask                 */
#define TIM2_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM2 CTRL: STATUS_SEL Position          */
#define TIM2_CTRL_STATUS_SEL_Msk              (0x07UL << TIM2_CTRL_STATUS_SEL_Pos)                    /*!< TIM2 CTRL: STATUS_SEL Mask              */
#define TIM2_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM2 CTRL: STATUS_INV Position          */
#define TIM2_CTRL_STATUS_INV_Msk              (0x01UL << TIM2_CTRL_STATUS_INV_Pos)                    /*!< TIM2 CTRL: STATUS_INV Mask              */
#define TIM2_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM2 CTRL: REQ_STOP Position            */
#define TIM2_CTRL_REQ_STOP_Msk                (0x01UL << TIM2_CTRL_REQ_STOP_Pos)                      /*!< TIM2 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM2_ENABLE  -------------------------------- */
#define TIM2_ENABLE_ENABLE_Pos                0                                                       /*!< TIM2 ENABLE: ENABLE Position            */
#define TIM2_ENABLE_ENABLE_Msk                (0x01UL << TIM2_ENABLE_ENABLE_Pos)                      /*!< TIM2 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM2_CSD_CTRL  ------------------------------- */
#define TIM2_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM2 CSD_CTRL: CSDEN0 Position          */
#define TIM2_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM2_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM2 CSD_CTRL: CSDEN0 Mask              */
#define TIM2_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM2 CSD_CTRL: CSDINV0 Position         */
#define TIM2_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM2_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM2 CSD_CTRL: CSDINV0 Mask             */
#define TIM2_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM2 CSD_CTRL: CSDEN1 Position          */
#define TIM2_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM2_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM2 CSD_CTRL: CSDEN1 Mask              */
#define TIM2_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM2 CSD_CTRL: CSDINV1 Position         */
#define TIM2_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM2_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM2 CSD_CTRL: CSDINV1 Mask             */
#define TIM2_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM2 CSD_CTRL: DCASOP Position          */
#define TIM2_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM2_CSD_CTRL_DCASOP_Pos)                    /*!< TIM2 CSD_CTRL: DCASOP Mask              */
#define TIM2_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM2 CSD_CTRL: CSDTRG0 Position         */
#define TIM2_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM2_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM2 CSD_CTRL: CSDTRG0 Mask             */
#define TIM2_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM2 CSD_CTRL: CSDTRG1 Position         */
#define TIM2_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM2_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM2 CSD_CTRL: CSDTRG1 Mask             */
#define TIM2_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM2 CSD_CTRL: CSDEN2 Position          */
#define TIM2_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM2_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM2 CSD_CTRL: CSDEN2 Mask              */
#define TIM2_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM2 CSD_CTRL: CSDINV2 Position         */
#define TIM2_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM2_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM2 CSD_CTRL: CSDINV2 Mask             */
#define TIM2_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM2 CSD_CTRL: CSDTRG2 Position        */
#define TIM2_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM2_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM2 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM2_CASCADE0  ------------------------------- */
#define TIM2_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM2 CASCADE0: CASSEL Position          */
#define TIM2_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM2_CASCADE0_CASSEL_Pos)              /*!< TIM2 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM2_CASCADE1  ------------------------------- */
#define TIM2_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM2 CASCADE1: CASSEL Position          */
#define TIM2_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM2_CASCADE1_CASSEL_Pos)              /*!< TIM2 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM2_CASCADE2  ------------------------------- */
#define TIM2_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM2 CASCADE2: CASSEL Position          */
#define TIM2_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM2_CASCADE2_CASSEL_Pos)              /*!< TIM2 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM3' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM3_CTRL  --------------------------------- */
#define TIM3_CTRL_ENABLE_Pos                  0                                                       /*!< TIM3 CTRL: ENABLE Position              */
#define TIM3_CTRL_ENABLE_Msk                  (0x01UL << TIM3_CTRL_ENABLE_Pos)                        /*!< TIM3 CTRL: ENABLE Mask                  */
#define TIM3_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM3 CTRL: ACTIVE Position              */
#define TIM3_CTRL_ACTIVE_Msk                  (0x01UL << TIM3_CTRL_ACTIVE_Pos)                        /*!< TIM3 CTRL: ACTIVE Mask                  */
#define TIM3_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM3 CTRL: AUTO_DISABLE Position        */
#define TIM3_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM3_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM3 CTRL: AUTO_DISABLE Mask            */
#define TIM3_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM3 CTRL: AUTO_DEACTIVATE Position     */
#define TIM3_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM3_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM3 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM3_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM3 CTRL: IRQ_ENB Position             */
#define TIM3_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM3_CTRL_IRQ_ENB_Pos)                       /*!< TIM3 CTRL: IRQ_ENB Mask                 */
#define TIM3_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM3 CTRL: STATUS_SEL Position          */
#define TIM3_CTRL_STATUS_SEL_Msk              (0x07UL << TIM3_CTRL_STATUS_SEL_Pos)                    /*!< TIM3 CTRL: STATUS_SEL Mask              */
#define TIM3_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM3 CTRL: STATUS_INV Position          */
#define TIM3_CTRL_STATUS_INV_Msk              (0x01UL << TIM3_CTRL_STATUS_INV_Pos)                    /*!< TIM3 CTRL: STATUS_INV Mask              */
#define TIM3_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM3 CTRL: REQ_STOP Position            */
#define TIM3_CTRL_REQ_STOP_Msk                (0x01UL << TIM3_CTRL_REQ_STOP_Pos)                      /*!< TIM3 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM3_ENABLE  -------------------------------- */
#define TIM3_ENABLE_ENABLE_Pos                0                                                       /*!< TIM3 ENABLE: ENABLE Position            */
#define TIM3_ENABLE_ENABLE_Msk                (0x01UL << TIM3_ENABLE_ENABLE_Pos)                      /*!< TIM3 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM3_CSD_CTRL  ------------------------------- */
#define TIM3_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM3 CSD_CTRL: CSDEN0 Position          */
#define TIM3_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM3_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM3 CSD_CTRL: CSDEN0 Mask              */
#define TIM3_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM3 CSD_CTRL: CSDINV0 Position         */
#define TIM3_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM3_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM3 CSD_CTRL: CSDINV0 Mask             */
#define TIM3_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM3 CSD_CTRL: CSDEN1 Position          */
#define TIM3_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM3_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM3 CSD_CTRL: CSDEN1 Mask              */
#define TIM3_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM3 CSD_CTRL: CSDINV1 Position         */
#define TIM3_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM3_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM3 CSD_CTRL: CSDINV1 Mask             */
#define TIM3_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM3 CSD_CTRL: DCASOP Position          */
#define TIM3_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM3_CSD_CTRL_DCASOP_Pos)                    /*!< TIM3 CSD_CTRL: DCASOP Mask              */
#define TIM3_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM3 CSD_CTRL: CSDTRG0 Position         */
#define TIM3_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM3_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM3 CSD_CTRL: CSDTRG0 Mask             */
#define TIM3_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM3 CSD_CTRL: CSDTRG1 Position         */
#define TIM3_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM3_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM3 CSD_CTRL: CSDTRG1 Mask             */
#define TIM3_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM3 CSD_CTRL: CSDEN2 Position          */
#define TIM3_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM3_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM3 CSD_CTRL: CSDEN2 Mask              */
#define TIM3_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM3 CSD_CTRL: CSDINV2 Position         */
#define TIM3_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM3_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM3 CSD_CTRL: CSDINV2 Mask             */
#define TIM3_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM3 CSD_CTRL: CSDTRG2 Position        */
#define TIM3_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM3_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM3 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM3_CASCADE0  ------------------------------- */
#define TIM3_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM3 CASCADE0: CASSEL Position          */
#define TIM3_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM3_CASCADE0_CASSEL_Pos)              /*!< TIM3 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM3_CASCADE1  ------------------------------- */
#define TIM3_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM3 CASCADE1: CASSEL Position          */
#define TIM3_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM3_CASCADE1_CASSEL_Pos)              /*!< TIM3 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM3_CASCADE2  ------------------------------- */
#define TIM3_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM3 CASCADE2: CASSEL Position          */
#define TIM3_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM3_CASCADE2_CASSEL_Pos)              /*!< TIM3 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM4' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM4_CTRL  --------------------------------- */
#define TIM4_CTRL_ENABLE_Pos                  0                                                       /*!< TIM4 CTRL: ENABLE Position              */
#define TIM4_CTRL_ENABLE_Msk                  (0x01UL << TIM4_CTRL_ENABLE_Pos)                        /*!< TIM4 CTRL: ENABLE Mask                  */
#define TIM4_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM4 CTRL: ACTIVE Position              */
#define TIM4_CTRL_ACTIVE_Msk                  (0x01UL << TIM4_CTRL_ACTIVE_Pos)                        /*!< TIM4 CTRL: ACTIVE Mask                  */
#define TIM4_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM4 CTRL: AUTO_DISABLE Position        */
#define TIM4_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM4_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM4 CTRL: AUTO_DISABLE Mask            */
#define TIM4_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM4 CTRL: AUTO_DEACTIVATE Position     */
#define TIM4_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM4_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM4 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM4_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM4 CTRL: IRQ_ENB Position             */
#define TIM4_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM4_CTRL_IRQ_ENB_Pos)                       /*!< TIM4 CTRL: IRQ_ENB Mask                 */
#define TIM4_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM4 CTRL: STATUS_SEL Position          */
#define TIM4_CTRL_STATUS_SEL_Msk              (0x07UL << TIM4_CTRL_STATUS_SEL_Pos)                    /*!< TIM4 CTRL: STATUS_SEL Mask              */
#define TIM4_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM4 CTRL: STATUS_INV Position          */
#define TIM4_CTRL_STATUS_INV_Msk              (0x01UL << TIM4_CTRL_STATUS_INV_Pos)                    /*!< TIM4 CTRL: STATUS_INV Mask              */
#define TIM4_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM4 CTRL: REQ_STOP Position            */
#define TIM4_CTRL_REQ_STOP_Msk                (0x01UL << TIM4_CTRL_REQ_STOP_Pos)                      /*!< TIM4 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM4_ENABLE  -------------------------------- */
#define TIM4_ENABLE_ENABLE_Pos                0                                                       /*!< TIM4 ENABLE: ENABLE Position            */
#define TIM4_ENABLE_ENABLE_Msk                (0x01UL << TIM4_ENABLE_ENABLE_Pos)                      /*!< TIM4 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM4_CSD_CTRL  ------------------------------- */
#define TIM4_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM4 CSD_CTRL: CSDEN0 Position          */
#define TIM4_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM4_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM4 CSD_CTRL: CSDEN0 Mask              */
#define TIM4_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM4 CSD_CTRL: CSDINV0 Position         */
#define TIM4_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM4_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM4 CSD_CTRL: CSDINV0 Mask             */
#define TIM4_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM4 CSD_CTRL: CSDEN1 Position          */
#define TIM4_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM4_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM4 CSD_CTRL: CSDEN1 Mask              */
#define TIM4_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM4 CSD_CTRL: CSDINV1 Position         */
#define TIM4_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM4_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM4 CSD_CTRL: CSDINV1 Mask             */
#define TIM4_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM4 CSD_CTRL: DCASOP Position          */
#define TIM4_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM4_CSD_CTRL_DCASOP_Pos)                    /*!< TIM4 CSD_CTRL: DCASOP Mask              */
#define TIM4_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM4 CSD_CTRL: CSDTRG0 Position         */
#define TIM4_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM4_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM4 CSD_CTRL: CSDTRG0 Mask             */
#define TIM4_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM4 CSD_CTRL: CSDTRG1 Position         */
#define TIM4_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM4_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM4 CSD_CTRL: CSDTRG1 Mask             */
#define TIM4_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM4 CSD_CTRL: CSDEN2 Position          */
#define TIM4_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM4_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM4 CSD_CTRL: CSDEN2 Mask              */
#define TIM4_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM4 CSD_CTRL: CSDINV2 Position         */
#define TIM4_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM4_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM4 CSD_CTRL: CSDINV2 Mask             */
#define TIM4_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM4 CSD_CTRL: CSDTRG2 Position        */
#define TIM4_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM4_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM4 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM4_CASCADE0  ------------------------------- */
#define TIM4_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM4 CASCADE0: CASSEL Position          */
#define TIM4_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM4_CASCADE0_CASSEL_Pos)              /*!< TIM4 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM4_CASCADE1  ------------------------------- */
#define TIM4_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM4 CASCADE1: CASSEL Position          */
#define TIM4_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM4_CASCADE1_CASSEL_Pos)              /*!< TIM4 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM4_CASCADE2  ------------------------------- */
#define TIM4_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM4 CASCADE2: CASSEL Position          */
#define TIM4_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM4_CASCADE2_CASSEL_Pos)              /*!< TIM4 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM5' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM5_CTRL  --------------------------------- */
#define TIM5_CTRL_ENABLE_Pos                  0                                                       /*!< TIM5 CTRL: ENABLE Position              */
#define TIM5_CTRL_ENABLE_Msk                  (0x01UL << TIM5_CTRL_ENABLE_Pos)                        /*!< TIM5 CTRL: ENABLE Mask                  */
#define TIM5_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM5 CTRL: ACTIVE Position              */
#define TIM5_CTRL_ACTIVE_Msk                  (0x01UL << TIM5_CTRL_ACTIVE_Pos)                        /*!< TIM5 CTRL: ACTIVE Mask                  */
#define TIM5_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM5 CTRL: AUTO_DISABLE Position        */
#define TIM5_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM5_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM5 CTRL: AUTO_DISABLE Mask            */
#define TIM5_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM5 CTRL: AUTO_DEACTIVATE Position     */
#define TIM5_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM5_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM5 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM5_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM5 CTRL: IRQ_ENB Position             */
#define TIM5_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM5_CTRL_IRQ_ENB_Pos)                       /*!< TIM5 CTRL: IRQ_ENB Mask                 */
#define TIM5_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM5 CTRL: STATUS_SEL Position          */
#define TIM5_CTRL_STATUS_SEL_Msk              (0x07UL << TIM5_CTRL_STATUS_SEL_Pos)                    /*!< TIM5 CTRL: STATUS_SEL Mask              */
#define TIM5_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM5 CTRL: STATUS_INV Position          */
#define TIM5_CTRL_STATUS_INV_Msk              (0x01UL << TIM5_CTRL_STATUS_INV_Pos)                    /*!< TIM5 CTRL: STATUS_INV Mask              */
#define TIM5_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM5 CTRL: REQ_STOP Position            */
#define TIM5_CTRL_REQ_STOP_Msk                (0x01UL << TIM5_CTRL_REQ_STOP_Pos)                      /*!< TIM5 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM5_ENABLE  -------------------------------- */
#define TIM5_ENABLE_ENABLE_Pos                0                                                       /*!< TIM5 ENABLE: ENABLE Position            */
#define TIM5_ENABLE_ENABLE_Msk                (0x01UL << TIM5_ENABLE_ENABLE_Pos)                      /*!< TIM5 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM5_CSD_CTRL  ------------------------------- */
#define TIM5_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM5 CSD_CTRL: CSDEN0 Position          */
#define TIM5_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM5_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM5 CSD_CTRL: CSDEN0 Mask              */
#define TIM5_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM5 CSD_CTRL: CSDINV0 Position         */
#define TIM5_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM5_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM5 CSD_CTRL: CSDINV0 Mask             */
#define TIM5_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM5 CSD_CTRL: CSDEN1 Position          */
#define TIM5_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM5_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM5 CSD_CTRL: CSDEN1 Mask              */
#define TIM5_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM5 CSD_CTRL: CSDINV1 Position         */
#define TIM5_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM5_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM5 CSD_CTRL: CSDINV1 Mask             */
#define TIM5_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM5 CSD_CTRL: DCASOP Position          */
#define TIM5_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM5_CSD_CTRL_DCASOP_Pos)                    /*!< TIM5 CSD_CTRL: DCASOP Mask              */
#define TIM5_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM5 CSD_CTRL: CSDTRG0 Position         */
#define TIM5_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM5_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM5 CSD_CTRL: CSDTRG0 Mask             */
#define TIM5_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM5 CSD_CTRL: CSDTRG1 Position         */
#define TIM5_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM5_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM5 CSD_CTRL: CSDTRG1 Mask             */
#define TIM5_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM5 CSD_CTRL: CSDEN2 Position          */
#define TIM5_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM5_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM5 CSD_CTRL: CSDEN2 Mask              */
#define TIM5_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM5 CSD_CTRL: CSDINV2 Position         */
#define TIM5_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM5_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM5 CSD_CTRL: CSDINV2 Mask             */
#define TIM5_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM5 CSD_CTRL: CSDTRG2 Position        */
#define TIM5_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM5_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM5 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM5_CASCADE0  ------------------------------- */
#define TIM5_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM5 CASCADE0: CASSEL Position          */
#define TIM5_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM5_CASCADE0_CASSEL_Pos)              /*!< TIM5 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM5_CASCADE1  ------------------------------- */
#define TIM5_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM5 CASCADE1: CASSEL Position          */
#define TIM5_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM5_CASCADE1_CASSEL_Pos)              /*!< TIM5 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM5_CASCADE2  ------------------------------- */
#define TIM5_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM5 CASCADE2: CASSEL Position          */
#define TIM5_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM5_CASCADE2_CASSEL_Pos)              /*!< TIM5 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM6' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM6_CTRL  --------------------------------- */
#define TIM6_CTRL_ENABLE_Pos                  0                                                       /*!< TIM6 CTRL: ENABLE Position              */
#define TIM6_CTRL_ENABLE_Msk                  (0x01UL << TIM6_CTRL_ENABLE_Pos)                        /*!< TIM6 CTRL: ENABLE Mask                  */
#define TIM6_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM6 CTRL: ACTIVE Position              */
#define TIM6_CTRL_ACTIVE_Msk                  (0x01UL << TIM6_CTRL_ACTIVE_Pos)                        /*!< TIM6 CTRL: ACTIVE Mask                  */
#define TIM6_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM6 CTRL: AUTO_DISABLE Position        */
#define TIM6_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM6_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM6 CTRL: AUTO_DISABLE Mask            */
#define TIM6_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM6 CTRL: AUTO_DEACTIVATE Position     */
#define TIM6_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM6_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM6 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM6_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM6 CTRL: IRQ_ENB Position             */
#define TIM6_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM6_CTRL_IRQ_ENB_Pos)                       /*!< TIM6 CTRL: IRQ_ENB Mask                 */
#define TIM6_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM6 CTRL: STATUS_SEL Position          */
#define TIM6_CTRL_STATUS_SEL_Msk              (0x07UL << TIM6_CTRL_STATUS_SEL_Pos)                    /*!< TIM6 CTRL: STATUS_SEL Mask              */
#define TIM6_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM6 CTRL: STATUS_INV Position          */
#define TIM6_CTRL_STATUS_INV_Msk              (0x01UL << TIM6_CTRL_STATUS_INV_Pos)                    /*!< TIM6 CTRL: STATUS_INV Mask              */
#define TIM6_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM6 CTRL: REQ_STOP Position            */
#define TIM6_CTRL_REQ_STOP_Msk                (0x01UL << TIM6_CTRL_REQ_STOP_Pos)                      /*!< TIM6 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM6_ENABLE  -------------------------------- */
#define TIM6_ENABLE_ENABLE_Pos                0                                                       /*!< TIM6 ENABLE: ENABLE Position            */
#define TIM6_ENABLE_ENABLE_Msk                (0x01UL << TIM6_ENABLE_ENABLE_Pos)                      /*!< TIM6 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM6_CSD_CTRL  ------------------------------- */
#define TIM6_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM6 CSD_CTRL: CSDEN0 Position          */
#define TIM6_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM6_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM6 CSD_CTRL: CSDEN0 Mask              */
#define TIM6_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM6 CSD_CTRL: CSDINV0 Position         */
#define TIM6_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM6_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM6 CSD_CTRL: CSDINV0 Mask             */
#define TIM6_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM6 CSD_CTRL: CSDEN1 Position          */
#define TIM6_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM6_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM6 CSD_CTRL: CSDEN1 Mask              */
#define TIM6_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM6 CSD_CTRL: CSDINV1 Position         */
#define TIM6_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM6_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM6 CSD_CTRL: CSDINV1 Mask             */
#define TIM6_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM6 CSD_CTRL: DCASOP Position          */
#define TIM6_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM6_CSD_CTRL_DCASOP_Pos)                    /*!< TIM6 CSD_CTRL: DCASOP Mask              */
#define TIM6_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM6 CSD_CTRL: CSDTRG0 Position         */
#define TIM6_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM6_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM6 CSD_CTRL: CSDTRG0 Mask             */
#define TIM6_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM6 CSD_CTRL: CSDTRG1 Position         */
#define TIM6_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM6_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM6 CSD_CTRL: CSDTRG1 Mask             */
#define TIM6_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM6 CSD_CTRL: CSDEN2 Position          */
#define TIM6_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM6_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM6 CSD_CTRL: CSDEN2 Mask              */
#define TIM6_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM6 CSD_CTRL: CSDINV2 Position         */
#define TIM6_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM6_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM6 CSD_CTRL: CSDINV2 Mask             */
#define TIM6_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM6 CSD_CTRL: CSDTRG2 Position        */
#define TIM6_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM6_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM6 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM6_CASCADE0  ------------------------------- */
#define TIM6_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM6 CASCADE0: CASSEL Position          */
#define TIM6_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM6_CASCADE0_CASSEL_Pos)              /*!< TIM6 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM6_CASCADE1  ------------------------------- */
#define TIM6_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM6 CASCADE1: CASSEL Position          */
#define TIM6_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM6_CASCADE1_CASSEL_Pos)              /*!< TIM6 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM6_CASCADE2  ------------------------------- */
#define TIM6_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM6 CASCADE2: CASSEL Position          */
#define TIM6_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM6_CASCADE2_CASSEL_Pos)              /*!< TIM6 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM7' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM7_CTRL  --------------------------------- */
#define TIM7_CTRL_ENABLE_Pos                  0                                                       /*!< TIM7 CTRL: ENABLE Position              */
#define TIM7_CTRL_ENABLE_Msk                  (0x01UL << TIM7_CTRL_ENABLE_Pos)                        /*!< TIM7 CTRL: ENABLE Mask                  */
#define TIM7_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM7 CTRL: ACTIVE Position              */
#define TIM7_CTRL_ACTIVE_Msk                  (0x01UL << TIM7_CTRL_ACTIVE_Pos)                        /*!< TIM7 CTRL: ACTIVE Mask                  */
#define TIM7_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM7 CTRL: AUTO_DISABLE Position        */
#define TIM7_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM7_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM7 CTRL: AUTO_DISABLE Mask            */
#define TIM7_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM7 CTRL: AUTO_DEACTIVATE Position     */
#define TIM7_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM7_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM7 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM7_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM7 CTRL: IRQ_ENB Position             */
#define TIM7_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM7_CTRL_IRQ_ENB_Pos)                       /*!< TIM7 CTRL: IRQ_ENB Mask                 */
#define TIM7_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM7 CTRL: STATUS_SEL Position          */
#define TIM7_CTRL_STATUS_SEL_Msk              (0x07UL << TIM7_CTRL_STATUS_SEL_Pos)                    /*!< TIM7 CTRL: STATUS_SEL Mask              */
#define TIM7_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM7 CTRL: STATUS_INV Position          */
#define TIM7_CTRL_STATUS_INV_Msk              (0x01UL << TIM7_CTRL_STATUS_INV_Pos)                    /*!< TIM7 CTRL: STATUS_INV Mask              */
#define TIM7_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM7 CTRL: REQ_STOP Position            */
#define TIM7_CTRL_REQ_STOP_Msk                (0x01UL << TIM7_CTRL_REQ_STOP_Pos)                      /*!< TIM7 CTRL: REQ_STOP Mask                */



/* ---------------------------------  TIM7_ENABLE  -------------------------------- */
#define TIM7_ENABLE_ENABLE_Pos                0                                                       /*!< TIM7 ENABLE: ENABLE Position            */
#define TIM7_ENABLE_ENABLE_Msk                (0x01UL << TIM7_ENABLE_ENABLE_Pos)                      /*!< TIM7 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM7_CSD_CTRL  ------------------------------- */
#define TIM7_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM7 CSD_CTRL: CSDEN0 Position          */
#define TIM7_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM7_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM7 CSD_CTRL: CSDEN0 Mask              */
#define TIM7_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM7 CSD_CTRL: CSDINV0 Position         */
#define TIM7_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM7_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM7 CSD_CTRL: CSDINV0 Mask             */
#define TIM7_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM7 CSD_CTRL: CSDEN1 Position          */
#define TIM7_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM7_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM7 CSD_CTRL: CSDEN1 Mask              */
#define TIM7_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM7 CSD_CTRL: CSDINV1 Position         */
#define TIM7_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM7_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM7 CSD_CTRL: CSDINV1 Mask             */
#define TIM7_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM7 CSD_CTRL: DCASOP Position          */
#define TIM7_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM7_CSD_CTRL_DCASOP_Pos)                    /*!< TIM7 CSD_CTRL: DCASOP Mask              */
#define TIM7_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM7 CSD_CTRL: CSDTRG0 Position         */
#define TIM7_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM7_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM7 CSD_CTRL: CSDTRG0 Mask             */
#define TIM7_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM7 CSD_CTRL: CSDTRG1 Position         */
#define TIM7_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM7_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM7 CSD_CTRL: CSDTRG1 Mask             */
#define TIM7_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM7 CSD_CTRL: CSDEN2 Position          */
#define TIM7_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM7_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM7 CSD_CTRL: CSDEN2 Mask              */
#define TIM7_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM7 CSD_CTRL: CSDINV2 Position         */
#define TIM7_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM7_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM7 CSD_CTRL: CSDINV2 Mask             */
#define TIM7_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM7 CSD_CTRL: CSDTRG2 Position        */
#define TIM7_CSD_CTRL_CSDTRG2_Msk             (0x01UL << TIM7_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM7 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM7_CASCADE0  ------------------------------- */
#define TIM7_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM7 CASCADE0: CASSEL Position          */
#define TIM7_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM7_CASCADE0_CASSEL_Pos)              /*!< TIM7 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM7_CASCADE1  ------------------------------- */
#define TIM7_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM7 CASCADE1: CASSEL Position          */
#define TIM7_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM7_CASCADE1_CASSEL_Pos)              /*!< TIM7 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM7_CASCADE2  ------------------------------- */
#define TIM7_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM7 CASCADE2: CASSEL Position          */
#define TIM7_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM7_CASCADE2_CASSEL_Pos)              /*!< TIM7 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM8' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM8_CTRL  --------------------------------- */
#define TIM8_CTRL_ENABLE_Pos                  0                                                       /*!< TIM8 CTRL: ENABLE Position              */
#define TIM8_CTRL_ENABLE_Msk                  (0x01UL << TIM8_CTRL_ENABLE_Pos)                        /*!< TIM8 CTRL: ENABLE Mask                  */
#define TIM8_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM8 CTRL: ACTIVE Position              */
#define TIM8_CTRL_ACTIVE_Msk                  (0x01UL << TIM8_CTRL_ACTIVE_Pos)                        /*!< TIM8 CTRL: ACTIVE Mask                  */
#define TIM8_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM8 CTRL: AUTO_DISABLE Position        */
#define TIM8_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM8_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM8 CTRL: AUTO_DISABLE Mask            */
#define TIM8_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM8 CTRL: AUTO_DEACTIVATE Position     */
#define TIM8_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM8_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM8 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM8_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM8 CTRL: IRQ_ENB Position             */
#define TIM8_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM8_CTRL_IRQ_ENB_Pos)                       /*!< TIM8 CTRL: IRQ_ENB Mask                 */
#define TIM8_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM8 CTRL: STATUS_SEL Position          */
#define TIM8_CTRL_STATUS_SEL_Msk              (0x07UL << TIM8_CTRL_STATUS_SEL_Pos)                    /*!< TIM8 CTRL: STATUS_SEL Mask              */
#define TIM8_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM8 CTRL: STATUS_INV Position          */
#define TIM8_CTRL_STATUS_INV_Msk              (0x01UL << TIM8_CTRL_STATUS_INV_Pos)                    /*!< TIM8 CTRL: STATUS_INV Mask              */
#define TIM8_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM8 CTRL: REQ_STOP Position            */
#define TIM8_CTRL_REQ_STOP_Msk                (0x01UL << TIM8_CTRL_REQ_STOP_Pos)                      /*!< TIM8 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM8_ENABLE  -------------------------------- */
#define TIM8_ENABLE_ENABLE_Pos                0                                                       /*!< TIM8 ENABLE: ENABLE Position            */
#define TIM8_ENABLE_ENABLE_Msk                (0x01UL << TIM8_ENABLE_ENABLE_Pos)                      /*!< TIM8 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM8_CSD_CTRL  ------------------------------- */
#define TIM8_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM8 CSD_CTRL: CSDEN0 Position          */
#define TIM8_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM8_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM8 CSD_CTRL: CSDEN0 Mask              */
#define TIM8_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM8 CSD_CTRL: CSDINV0 Position         */
#define TIM8_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM8_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM8 CSD_CTRL: CSDINV0 Mask             */
#define TIM8_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM8 CSD_CTRL: CSDEN1 Position          */
#define TIM8_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM8_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM8 CSD_CTRL: CSDEN1 Mask              */
#define TIM8_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM8 CSD_CTRL: CSDINV1 Position         */
#define TIM8_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM8_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM8 CSD_CTRL: CSDINV1 Mask             */
#define TIM8_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM8 CSD_CTRL: DCASOP Position          */
#define TIM8_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM8_CSD_CTRL_DCASOP_Pos)                    /*!< TIM8 CSD_CTRL: DCASOP Mask              */
#define TIM8_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM8 CSD_CTRL: CSDTRG0 Position         */
#define TIM8_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM8_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM8 CSD_CTRL: CSDTRG0 Mask             */
#define TIM8_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM8 CSD_CTRL: CSDTRG1 Position         */
#define TIM8_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM8_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM8 CSD_CTRL: CSDTRG1 Mask             */
#define TIM8_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM8 CSD_CTRL: CSDEN2 Position          */
#define TIM8_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM8_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM8 CSD_CTRL: CSDEN2 Mask              */
#define TIM8_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM8 CSD_CTRL: CSDINV2 Position         */
#define TIM8_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM8_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM8 CSD_CTRL: CSDINV2 Mask             */
#define TIM8_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM8 CSD_CTRL: CSDTRG2 Position        */
#define TIM8_CSD_CTRL_CSDTRG2_Msk             (0x01UL << TIM8_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM8 CSD_CTRL: CSDTRG2 Mask            */


/* --------------------------------  TIM8_CASCADE0  ------------------------------- */
#define TIM8_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM8 CASCADE0: CASSEL Position          */
#define TIM8_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM8_CASCADE0_CASSEL_Pos)              /*!< TIM8 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM8_CASCADE1  ------------------------------- */
#define TIM8_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM8 CASCADE1: CASSEL Position          */
#define TIM8_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM8_CASCADE1_CASSEL_Pos)              /*!< TIM8 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM8_CASCADE2  ------------------------------- */
#define TIM8_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM8 CASCADE2: CASSEL Position          */
#define TIM8_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM8_CASCADE2_CASSEL_Pos)              /*!< TIM8 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================          struct 'TIM9' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  TIM9_CTRL  --------------------------------- */
#define TIM9_CTRL_ENABLE_Pos                  0                                                       /*!< TIM9 CTRL: ENABLE Position              */
#define TIM9_CTRL_ENABLE_Msk                  (0x01UL << TIM9_CTRL_ENABLE_Pos)                        /*!< TIM9 CTRL: ENABLE Mask                  */
#define TIM9_CTRL_ACTIVE_Pos                  1                                                       /*!< TIM9 CTRL: ACTIVE Position              */
#define TIM9_CTRL_ACTIVE_Msk                  (0x01UL << TIM9_CTRL_ACTIVE_Pos)                        /*!< TIM9 CTRL: ACTIVE Mask                  */
#define TIM9_CTRL_AUTO_DISABLE_Pos            2                                                       /*!< TIM9 CTRL: AUTO_DISABLE Position        */
#define TIM9_CTRL_AUTO_DISABLE_Msk            (0x01UL << TIM9_CTRL_AUTO_DISABLE_Pos)                  /*!< TIM9 CTRL: AUTO_DISABLE Mask            */
#define TIM9_CTRL_AUTO_DEACTIVATE_Pos         3                                                       /*!< TIM9 CTRL: AUTO_DEACTIVATE Position     */
#define TIM9_CTRL_AUTO_DEACTIVATE_Msk         (0x01UL << TIM9_CTRL_AUTO_DEACTIVATE_Pos)               /*!< TIM9 CTRL: AUTO_DEACTIVATE Mask         */
#define TIM9_CTRL_IRQ_ENB_Pos                 4                                                       /*!< TIM9 CTRL: IRQ_ENB Position             */
#define TIM9_CTRL_IRQ_ENB_Msk                 (0x01UL << TIM9_CTRL_IRQ_ENB_Pos)                       /*!< TIM9 CTRL: IRQ_ENB Mask                 */
#define TIM9_CTRL_STATUS_SEL_Pos              5                                                       /*!< TIM9 CTRL: STATUS_SEL Position          */
#define TIM9_CTRL_STATUS_SEL_Msk              (0x07UL << TIM9_CTRL_STATUS_SEL_Pos)                    /*!< TIM9 CTRL: STATUS_SEL Mask              */
#define TIM9_CTRL_STATUS_INV_Pos              8                                                       /*!< TIM9 CTRL: STATUS_INV Position          */
#define TIM9_CTRL_STATUS_INV_Msk              (0x01UL << TIM9_CTRL_STATUS_INV_Pos)                    /*!< TIM9 CTRL: STATUS_INV Mask              */
#define TIM9_CTRL_REQ_STOP_Pos                9                                                       /*!< TIM9 CTRL: REQ_STOP Position            */
#define TIM9_CTRL_REQ_STOP_Msk                (0x01UL << TIM9_CTRL_REQ_STOP_Pos)                      /*!< TIM9 CTRL: REQ_STOP Mask                */

/* ---------------------------------  TIM9_ENABLE  -------------------------------- */
#define TIM9_ENABLE_ENABLE_Pos                0                                                       /*!< TIM9 ENABLE: ENABLE Position            */
#define TIM9_ENABLE_ENABLE_Msk                (0x01UL << TIM9_ENABLE_ENABLE_Pos)                      /*!< TIM9 ENABLE: ENABLE Mask                */

/* --------------------------------  TIM9_CSD_CTRL  ------------------------------- */
#define TIM9_CSD_CTRL_CSDEN0_Pos              0                                                       /*!< TIM9 CSD_CTRL: CSDEN0 Position          */
#define TIM9_CSD_CTRL_CSDEN0_Msk              (0x01UL << TIM9_CSD_CTRL_CSDEN0_Pos)                    /*!< TIM9 CSD_CTRL: CSDEN0 Mask              */
#define TIM9_CSD_CTRL_CSDINV0_Pos             1                                                       /*!< TIM9 CSD_CTRL: CSDINV0 Position         */
#define TIM9_CSD_CTRL_CSDINV0_Msk             (0x01UL << TIM9_CSD_CTRL_CSDINV0_Pos)                   /*!< TIM9 CSD_CTRL: CSDINV0 Mask             */
#define TIM9_CSD_CTRL_CSDEN1_Pos              2                                                       /*!< TIM9 CSD_CTRL: CSDEN1 Position          */
#define TIM9_CSD_CTRL_CSDEN1_Msk              (0x01UL << TIM9_CSD_CTRL_CSDEN1_Pos)                    /*!< TIM9 CSD_CTRL: CSDEN1 Mask              */
#define TIM9_CSD_CTRL_CSDINV1_Pos             3                                                       /*!< TIM9 CSD_CTRL: CSDINV1 Position         */
#define TIM9_CSD_CTRL_CSDINV1_Msk             (0x01UL << TIM9_CSD_CTRL_CSDINV1_Pos)                   /*!< TIM9 CSD_CTRL: CSDINV1 Mask             */
#define TIM9_CSD_CTRL_DCASOP_Pos              4                                                       /*!< TIM9 CSD_CTRL: DCASOP Position          */
#define TIM9_CSD_CTRL_DCASOP_Msk              (0x01UL << TIM9_CSD_CTRL_DCASOP_Pos)                    /*!< TIM9 CSD_CTRL: DCASOP Mask              */
#define TIM9_CSD_CTRL_CSDTRG0_Pos             6                                                       /*!< TIM9 CSD_CTRL: CSDTRG0 Position         */
#define TIM9_CSD_CTRL_CSDTRG0_Msk             (0x01UL << TIM9_CSD_CTRL_CSDTRG0_Pos)                   /*!< TIM9 CSD_CTRL: CSDTRG0 Mask             */
#define TIM9_CSD_CTRL_CSDTRG1_Pos             7                                                       /*!< TIM9 CSD_CTRL: CSDTRG1 Position         */
#define TIM9_CSD_CTRL_CSDTRG1_Msk             (0x01UL << TIM9_CSD_CTRL_CSDTRG1_Pos)                   /*!< TIM9 CSD_CTRL: CSDTRG1 Mask             */
#define TIM9_CSD_CTRL_CSDEN2_Pos              8                                                       /*!< TIM9 CSD_CTRL: CSDEN2 Position          */
#define TIM9_CSD_CTRL_CSDEN2_Msk              (0x01UL << TIM9_CSD_CTRL_CSDEN2_Pos)                    /*!< TIM9 CSD_CTRL: CSDEN2 Mask              */
#define TIM9_CSD_CTRL_CSDINV2_Pos             9                                                       /*!< TIM9 CSD_CTRL: CSDINV2 Position         */
#define TIM9_CSD_CTRL_CSDINV2_Msk             (0x01UL << TIM9_CSD_CTRL_CSDINV2_Pos)                   /*!< TIM9 CSD_CTRL: CSDINV2 Mask             */
#define TIM9_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM9 CSD_CTRL: CSDTRG2 Position        */
#define TIM9_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM9_CSD_CTRL_CSDTRG2_Pos)                   /*!< TIM9 CSD_CTRL: CSDTRG2 Mask            */

/* --------------------------------  TIM9_CASCADE0  ------------------------------- */
#define TIM9_CASCADE0_CASSEL_Pos              0                                                       /*!< TIM9 CASCADE0: CASSEL Position          */
#define TIM9_CASCADE0_CASSEL_Msk              (0x000000ffUL << TIM9_CASCADE0_CASSEL_Pos)              /*!< TIM9 CASCADE0: CASSEL Mask              */

/* --------------------------------  TIM9_CASCADE1  ------------------------------- */
#define TIM9_CASCADE1_CASSEL_Pos              0                                                       /*!< TIM9 CASCADE1: CASSEL Position          */
#define TIM9_CASCADE1_CASSEL_Msk              (0x000000ffUL << TIM9_CASCADE1_CASSEL_Pos)              /*!< TIM9 CASCADE1: CASSEL Mask              */

/* --------------------------------  TIM9_CASCADE2  ------------------------------- */
#define TIM9_CASCADE2_CASSEL_Pos              0                                                       /*!< TIM9 CASCADE2: CASSEL Position          */
#define TIM9_CASCADE2_CASSEL_Msk              (0x000000ffUL << TIM9_CASCADE2_CASSEL_Pos)              /*!< TIM9 CASCADE2: CASSEL Mask              */


/* ================================================================================ */
/* ================         struct 'TIM10' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM10_CTRL  --------------------------------- */
#define TIM10_CTRL_ENABLE_Pos                 0                                                       /*!< TIM10 CTRL: ENABLE Position             */
#define TIM10_CTRL_ENABLE_Msk                 (0x01UL << TIM10_CTRL_ENABLE_Pos)                       /*!< TIM10 CTRL: ENABLE Mask                 */
#define TIM10_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM10 CTRL: ACTIVE Position             */
#define TIM10_CTRL_ACTIVE_Msk                 (0x01UL << TIM10_CTRL_ACTIVE_Pos)                       /*!< TIM10 CTRL: ACTIVE Mask                 */
#define TIM10_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM10 CTRL: AUTO_DISABLE Position       */
#define TIM10_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM10_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM10 CTRL: AUTO_DISABLE Mask           */
#define TIM10_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM10 CTRL: AUTO_DEACTIVATE Position    */
#define TIM10_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM10_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM10 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM10_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM10 CTRL: IRQ_ENB Position            */
#define TIM10_CTRL_IRQ_ENB_Msk                (0x01UL << TIM10_CTRL_IRQ_ENB_Pos)                      /*!< TIM10 CTRL: IRQ_ENB Mask                */
#define TIM10_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM10 CTRL: STATUS_SEL Position         */
#define TIM10_CTRL_STATUS_SEL_Msk             (0x07UL << TIM10_CTRL_STATUS_SEL_Pos)                   /*!< TIM10 CTRL: STATUS_SEL Mask             */
#define TIM10_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM10 CTRL: STATUS_INV Position         */
#define TIM10_CTRL_STATUS_INV_Msk             (0x01UL << TIM10_CTRL_STATUS_INV_Pos)                   /*!< TIM10 CTRL: STATUS_INV Mask             */
#define TIM10_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM10 CTRL: REQ_STOP Position           */
#define TIM10_CTRL_REQ_STOP_Msk               (0x01UL << TIM10_CTRL_REQ_STOP_Pos)                     /*!< TIM10 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM10_ENABLE  -------------------------------- */
#define TIM10_ENABLE_ENABLE_Pos               0                                                       /*!< TIM10 ENABLE: ENABLE Position           */
#define TIM10_ENABLE_ENABLE_Msk               (0x01UL << TIM10_ENABLE_ENABLE_Pos)                     /*!< TIM10 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM10_CSD_CTRL  ------------------------------- */
#define TIM10_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM10 CSD_CTRL: CSDEN0 Position         */
#define TIM10_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM10_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM10 CSD_CTRL: CSDEN0 Mask             */
#define TIM10_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM10 CSD_CTRL: CSDINV0 Position        */
#define TIM10_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM10_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM10 CSD_CTRL: CSDINV0 Mask            */
#define TIM10_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM10 CSD_CTRL: CSDEN1 Position         */
#define TIM10_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM10_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM10 CSD_CTRL: CSDEN1 Mask             */
#define TIM10_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM10 CSD_CTRL: CSDINV1 Position        */
#define TIM10_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM10_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM10 CSD_CTRL: CSDINV1 Mask            */
#define TIM10_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM10 CSD_CTRL: DCASOP Position         */
#define TIM10_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM10_CSD_CTRL_DCASOP_Pos)                   /*!< TIM10 CSD_CTRL: DCASOP Mask             */
#define TIM10_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM10 CSD_CTRL: CSDTRG0 Position        */
#define TIM10_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM10_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM10 CSD_CTRL: CSDTRG0 Mask            */
#define TIM10_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM10 CSD_CTRL: CSDTRG1 Position        */
#define TIM10_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM10_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM10 CSD_CTRL: CSDTRG1 Mask            */
#define TIM10_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM10 CSD_CTRL: CSDEN2 Position         */
#define TIM10_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM10_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM10 CSD_CTRL: CSDEN2 Mask             */
#define TIM10_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM10 CSD_CTRL: CSDINV2 Position        */
#define TIM10_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM10_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM10 CSD_CTRL: CSDINV2 Mask            */
#define TIM10_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM10 CSD_CTRL: CSDTRG2 Position        */
#define TIM10_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM10_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM10 CSD_CTRL: CSDTRG2 Mask            */

/* -------------------------------  TIM10_CASCADE0  ------------------------------- */
#define TIM10_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM10 CASCADE0: CASSEL Position         */
#define TIM10_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM10_CASCADE0_CASSEL_Pos)             /*!< TIM10 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM10_CASCADE1  ------------------------------- */
#define TIM10_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM10 CASCADE1: CASSEL Position         */
#define TIM10_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM10_CASCADE1_CASSEL_Pos)             /*!< TIM10 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM10_CASCADE2  ------------------------------- */
#define TIM10_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM10 CASCADE2: CASSEL Position         */
#define TIM10_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM10_CASCADE2_CASSEL_Pos)             /*!< TIM10 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM11' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM11_CTRL  --------------------------------- */
#define TIM11_CTRL_ENABLE_Pos                 0                                                       /*!< TIM11 CTRL: ENABLE Position             */
#define TIM11_CTRL_ENABLE_Msk                 (0x01UL << TIM11_CTRL_ENABLE_Pos)                       /*!< TIM11 CTRL: ENABLE Mask                 */
#define TIM11_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM11 CTRL: ACTIVE Position             */
#define TIM11_CTRL_ACTIVE_Msk                 (0x01UL << TIM11_CTRL_ACTIVE_Pos)                       /*!< TIM11 CTRL: ACTIVE Mask                 */
#define TIM11_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM11 CTRL: AUTO_DISABLE Position       */
#define TIM11_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM11_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM11 CTRL: AUTO_DISABLE Mask           */
#define TIM11_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM11 CTRL: AUTO_DEACTIVATE Position    */
#define TIM11_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM11_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM11 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM11_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM11 CTRL: IRQ_ENB Position            */
#define TIM11_CTRL_IRQ_ENB_Msk                (0x01UL << TIM11_CTRL_IRQ_ENB_Pos)                      /*!< TIM11 CTRL: IRQ_ENB Mask                */
#define TIM11_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM11 CTRL: STATUS_SEL Position         */
#define TIM11_CTRL_STATUS_SEL_Msk             (0x07UL << TIM11_CTRL_STATUS_SEL_Pos)                   /*!< TIM11 CTRL: STATUS_SEL Mask             */
#define TIM11_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM11 CTRL: STATUS_INV Position         */
#define TIM11_CTRL_STATUS_INV_Msk             (0x01UL << TIM11_CTRL_STATUS_INV_Pos)                   /*!< TIM11 CTRL: STATUS_INV Mask             */
#define TIM11_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM11 CTRL: REQ_STOP Position           */
#define TIM11_CTRL_REQ_STOP_Msk               (0x01UL << TIM11_CTRL_REQ_STOP_Pos)                     /*!< TIM11 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM11_ENABLE  -------------------------------- */
#define TIM11_ENABLE_ENABLE_Pos               0                                                       /*!< TIM11 ENABLE: ENABLE Position           */
#define TIM11_ENABLE_ENABLE_Msk               (0x01UL << TIM11_ENABLE_ENABLE_Pos)                     /*!< TIM11 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM11_CSD_CTRL  ------------------------------- */
#define TIM11_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM11 CSD_CTRL: CSDEN0 Position         */
#define TIM11_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM11_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM11 CSD_CTRL: CSDEN0 Mask             */
#define TIM11_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM11 CSD_CTRL: CSDINV0 Position        */
#define TIM11_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM11_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM11 CSD_CTRL: CSDINV0 Mask            */
#define TIM11_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM11 CSD_CTRL: CSDEN1 Position         */
#define TIM11_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM11_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM11 CSD_CTRL: CSDEN1 Mask             */
#define TIM11_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM11 CSD_CTRL: CSDINV1 Position        */
#define TIM11_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM11_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM11 CSD_CTRL: CSDINV1 Mask            */
#define TIM11_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM11 CSD_CTRL: DCASOP Position         */
#define TIM11_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM11_CSD_CTRL_DCASOP_Pos)                   /*!< TIM11 CSD_CTRL: DCASOP Mask             */
#define TIM11_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM11 CSD_CTRL: CSDTRG0 Position        */
#define TIM11_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM11_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM11 CSD_CTRL: CSDTRG0 Mask            */
#define TIM11_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM11 CSD_CTRL: CSDTRG1 Position        */
#define TIM11_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM11_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM11 CSD_CTRL: CSDTRG1 Mask            */
#define TIM11_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM11 CSD_CTRL: CSDEN2 Position         */
#define TIM11_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM11_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM11 CSD_CTRL: CSDEN2 Mask             */
#define TIM11_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM11 CSD_CTRL: CSDINV2 Position        */
#define TIM11_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM11_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM11 CSD_CTRL: CSDINV2 Mask            */
#define TIM11_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM11 CSD_CTRL: CSDTRG2 Position        */
#define TIM11_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM11_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM11 CSD_CTRL: CSDTRG2 Mask            */

/* -------------------------------  TIM11_CASCADE0  ------------------------------- */
#define TIM11_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM11 CASCADE0: CASSEL Position         */
#define TIM11_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM11_CASCADE0_CASSEL_Pos)             /*!< TIM11 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM11_CASCADE1  ------------------------------- */
#define TIM11_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM11 CASCADE1: CASSEL Position         */
#define TIM11_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM11_CASCADE1_CASSEL_Pos)             /*!< TIM11 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM11_CASCADE2  ------------------------------- */
#define TIM11_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM11 CASCADE2: CASSEL Position         */
#define TIM11_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM11_CASCADE2_CASSEL_Pos)             /*!< TIM11 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM12' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM12_CTRL  --------------------------------- */
#define TIM12_CTRL_ENABLE_Pos                 0                                                       /*!< TIM12 CTRL: ENABLE Position             */
#define TIM12_CTRL_ENABLE_Msk                 (0x01UL << TIM12_CTRL_ENABLE_Pos)                       /*!< TIM12 CTRL: ENABLE Mask                 */
#define TIM12_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM12 CTRL: ACTIVE Position             */
#define TIM12_CTRL_ACTIVE_Msk                 (0x01UL << TIM12_CTRL_ACTIVE_Pos)                       /*!< TIM12 CTRL: ACTIVE Mask                 */
#define TIM12_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM12 CTRL: AUTO_DISABLE Position       */
#define TIM12_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM12_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM12 CTRL: AUTO_DISABLE Mask           */
#define TIM12_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM12 CTRL: AUTO_DEACTIVATE Position    */
#define TIM12_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM12_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM12 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM12_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM12 CTRL: IRQ_ENB Position            */
#define TIM12_CTRL_IRQ_ENB_Msk                (0x01UL << TIM12_CTRL_IRQ_ENB_Pos)                      /*!< TIM12 CTRL: IRQ_ENB Mask                */
#define TIM12_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM12 CTRL: STATUS_SEL Position         */
#define TIM12_CTRL_STATUS_SEL_Msk             (0x07UL << TIM12_CTRL_STATUS_SEL_Pos)                   /*!< TIM12 CTRL: STATUS_SEL Mask             */
#define TIM12_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM12 CTRL: STATUS_INV Position         */
#define TIM12_CTRL_STATUS_INV_Msk             (0x01UL << TIM12_CTRL_STATUS_INV_Pos)                   /*!< TIM12 CTRL: STATUS_INV Mask             */
#define TIM12_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM12 CTRL: REQ_STOP Position           */
#define TIM12_CTRL_REQ_STOP_Msk               (0x01UL << TIM12_CTRL_REQ_STOP_Pos)                     /*!< TIM12 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM12_ENABLE  -------------------------------- */
#define TIM12_ENABLE_ENABLE_Pos               0                                                       /*!< TIM12 ENABLE: ENABLE Position           */
#define TIM12_ENABLE_ENABLE_Msk               (0x01UL << TIM12_ENABLE_ENABLE_Pos)                     /*!< TIM12 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM12_CSD_CTRL  ------------------------------- */
#define TIM12_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM12 CSD_CTRL: CSDEN0 Position         */
#define TIM12_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM12_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM12 CSD_CTRL: CSDEN0 Mask             */
#define TIM12_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM12 CSD_CTRL: CSDINV0 Position        */
#define TIM12_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM12_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM12 CSD_CTRL: CSDINV0 Mask            */
#define TIM12_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM12 CSD_CTRL: CSDEN1 Position         */
#define TIM12_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM12_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM12 CSD_CTRL: CSDEN1 Mask             */
#define TIM12_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM12 CSD_CTRL: CSDINV1 Position        */
#define TIM12_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM12_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM12 CSD_CTRL: CSDINV1 Mask            */
#define TIM12_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM12 CSD_CTRL: DCASOP Position         */
#define TIM12_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM12_CSD_CTRL_DCASOP_Pos)                   /*!< TIM12 CSD_CTRL: DCASOP Mask             */
#define TIM12_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM12 CSD_CTRL: CSDTRG0 Position        */
#define TIM12_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM12_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM12 CSD_CTRL: CSDTRG0 Mask            */
#define TIM12_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM12 CSD_CTRL: CSDTRG1 Position        */
#define TIM12_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM12_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM12 CSD_CTRL: CSDTRG1 Mask            */
#define TIM12_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM12 CSD_CTRL: CSDEN2 Position         */
#define TIM12_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM12_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM12 CSD_CTRL: CSDEN2 Mask             */
#define TIM12_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM12 CSD_CTRL: CSDINV2 Position        */
#define TIM12_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM12_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM12 CSD_CTRL: CSDINV2 Mask            */
#define TIM12_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM12 CSD_CTRL: CSDTRG2 Position        */
#define TIM12_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM12_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM12 CSD_CTRL: CSDTRG2 Mask            */

/* -------------------------------  TIM12_CASCADE0  ------------------------------- */
#define TIM12_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM12 CASCADE0: CASSEL Position         */
#define TIM12_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM12_CASCADE0_CASSEL_Pos)             /*!< TIM12 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM12_CASCADE1  ------------------------------- */
#define TIM12_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM12 CASCADE1: CASSEL Position         */
#define TIM12_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM12_CASCADE1_CASSEL_Pos)             /*!< TIM12 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM12_CASCADE2  ------------------------------- */
#define TIM12_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM12 CASCADE2: CASSEL Position         */
#define TIM12_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM12_CASCADE2_CASSEL_Pos)             /*!< TIM12 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM13' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM13_CTRL  --------------------------------- */
#define TIM13_CTRL_ENABLE_Pos                 0                                                       /*!< TIM13 CTRL: ENABLE Position             */
#define TIM13_CTRL_ENABLE_Msk                 (0x01UL << TIM13_CTRL_ENABLE_Pos)                       /*!< TIM13 CTRL: ENABLE Mask                 */
#define TIM13_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM13 CTRL: ACTIVE Position             */
#define TIM13_CTRL_ACTIVE_Msk                 (0x01UL << TIM13_CTRL_ACTIVE_Pos)                       /*!< TIM13 CTRL: ACTIVE Mask                 */
#define TIM13_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM13 CTRL: AUTO_DISABLE Position       */
#define TIM13_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM13_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM13 CTRL: AUTO_DISABLE Mask           */
#define TIM13_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM13 CTRL: AUTO_DEACTIVATE Position    */
#define TIM13_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM13_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM13 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM13_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM13 CTRL: IRQ_ENB Position            */
#define TIM13_CTRL_IRQ_ENB_Msk                (0x01UL << TIM13_CTRL_IRQ_ENB_Pos)                      /*!< TIM13 CTRL: IRQ_ENB Mask                */
#define TIM13_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM13 CTRL: STATUS_SEL Position         */
#define TIM13_CTRL_STATUS_SEL_Msk             (0x07UL << TIM13_CTRL_STATUS_SEL_Pos)                   /*!< TIM13 CTRL: STATUS_SEL Mask             */
#define TIM13_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM13 CTRL: STATUS_INV Position         */
#define TIM13_CTRL_STATUS_INV_Msk             (0x01UL << TIM13_CTRL_STATUS_INV_Pos)                   /*!< TIM13 CTRL: STATUS_INV Mask             */
#define TIM13_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM13 CTRL: REQ_STOP Position           */
#define TIM13_CTRL_REQ_STOP_Msk               (0x01UL << TIM13_CTRL_REQ_STOP_Pos)                     /*!< TIM13 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM13_ENABLE  -------------------------------- */
#define TIM13_ENABLE_ENABLE_Pos               0                                                       /*!< TIM13 ENABLE: ENABLE Position           */
#define TIM13_ENABLE_ENABLE_Msk               (0x01UL << TIM13_ENABLE_ENABLE_Pos)                     /*!< TIM13 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM13_CSD_CTRL  ------------------------------- */
#define TIM13_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM13 CSD_CTRL: CSDEN0 Position         */
#define TIM13_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM13_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM13 CSD_CTRL: CSDEN0 Mask             */
#define TIM13_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM13 CSD_CTRL: CSDINV0 Position        */
#define TIM13_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM13_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM13 CSD_CTRL: CSDINV0 Mask            */
#define TIM13_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM13 CSD_CTRL: CSDEN1 Position         */
#define TIM13_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM13_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM13 CSD_CTRL: CSDEN1 Mask             */
#define TIM13_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM13 CSD_CTRL: CSDINV1 Position        */
#define TIM13_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM13_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM13 CSD_CTRL: CSDINV1 Mask            */
#define TIM13_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM13 CSD_CTRL: DCASOP Position         */
#define TIM13_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM13_CSD_CTRL_DCASOP_Pos)                   /*!< TIM13 CSD_CTRL: DCASOP Mask             */
#define TIM13_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM13 CSD_CTRL: CSDTRG0 Position        */
#define TIM13_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM13_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM13 CSD_CTRL: CSDTRG0 Mask            */
#define TIM13_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM13 CSD_CTRL: CSDTRG1 Position        */
#define TIM13_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM13_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM13 CSD_CTRL: CSDTRG1 Mask            */
#define TIM13_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM13 CSD_CTRL: CSDEN2 Position         */
#define TIM13_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM13_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM13 CSD_CTRL: CSDEN2 Mask             */
#define TIM13_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM13 CSD_CTRL: CSDINV2 Position        */
#define TIM13_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM13_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM13 CSD_CTRL: CSDINV2 Mask            */
#define TIM13_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM13 CSD_CTRL: CSDTRG2 Position        */
#define TIM13_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM13_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM13 CSD_CTRL: CSDTRG2 Mask            */

/* -------------------------------  TIM13_CASCADE0  ------------------------------- */
#define TIM13_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM13 CASCADE0: CASSEL Position         */
#define TIM13_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM13_CASCADE0_CASSEL_Pos)             /*!< TIM13 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM13_CASCADE1  ------------------------------- */
#define TIM13_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM13 CASCADE1: CASSEL Position         */
#define TIM13_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM13_CASCADE1_CASSEL_Pos)             /*!< TIM13 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM13_CASCADE2  ------------------------------- */
#define TIM13_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM13 CASCADE2: CASSEL Position         */
#define TIM13_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM13_CASCADE2_CASSEL_Pos)             /*!< TIM13 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM14' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM14_CTRL  --------------------------------- */
#define TIM14_CTRL_ENABLE_Pos                 0                                                       /*!< TIM14 CTRL: ENABLE Position             */
#define TIM14_CTRL_ENABLE_Msk                 (0x01UL << TIM14_CTRL_ENABLE_Pos)                       /*!< TIM14 CTRL: ENABLE Mask                 */
#define TIM14_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM14 CTRL: ACTIVE Position             */
#define TIM14_CTRL_ACTIVE_Msk                 (0x01UL << TIM14_CTRL_ACTIVE_Pos)                       /*!< TIM14 CTRL: ACTIVE Mask                 */
#define TIM14_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM14 CTRL: AUTO_DISABLE Position       */
#define TIM14_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM14_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM14 CTRL: AUTO_DISABLE Mask           */
#define TIM14_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM14 CTRL: AUTO_DEACTIVATE Position    */
#define TIM14_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM14_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM14 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM14_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM14 CTRL: IRQ_ENB Position            */
#define TIM14_CTRL_IRQ_ENB_Msk                (0x01UL << TIM14_CTRL_IRQ_ENB_Pos)                      /*!< TIM14 CTRL: IRQ_ENB Mask                */
#define TIM14_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM14 CTRL: STATUS_SEL Position         */
#define TIM14_CTRL_STATUS_SEL_Msk             (0x07UL << TIM14_CTRL_STATUS_SEL_Pos)                   /*!< TIM14 CTRL: STATUS_SEL Mask             */
#define TIM14_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM14 CTRL: STATUS_INV Position         */
#define TIM14_CTRL_STATUS_INV_Msk             (0x01UL << TIM14_CTRL_STATUS_INV_Pos)                   /*!< TIM14 CTRL: STATUS_INV Mask             */
#define TIM14_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM14 CTRL: REQ_STOP Position           */
#define TIM14_CTRL_REQ_STOP_Msk               (0x01UL << TIM14_CTRL_REQ_STOP_Pos)                     /*!< TIM14 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM14_ENABLE  -------------------------------- */
#define TIM14_ENABLE_ENABLE_Pos               0                                                       /*!< TIM14 ENABLE: ENABLE Position           */
#define TIM14_ENABLE_ENABLE_Msk               (0x01UL << TIM14_ENABLE_ENABLE_Pos)                     /*!< TIM14 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM14_CSD_CTRL  ------------------------------- */
#define TIM14_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM14 CSD_CTRL: CSDEN0 Position         */
#define TIM14_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM14_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM14 CSD_CTRL: CSDEN0 Mask             */
#define TIM14_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM14 CSD_CTRL: CSDINV0 Position        */
#define TIM14_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM14_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM14 CSD_CTRL: CSDINV0 Mask            */
#define TIM14_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM14 CSD_CTRL: CSDEN1 Position         */
#define TIM14_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM14_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM14 CSD_CTRL: CSDEN1 Mask             */
#define TIM14_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM14 CSD_CTRL: CSDINV1 Position        */
#define TIM14_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM14_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM14 CSD_CTRL: CSDINV1 Mask            */
#define TIM14_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM14 CSD_CTRL: DCASOP Position         */
#define TIM14_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM14_CSD_CTRL_DCASOP_Pos)                   /*!< TIM14 CSD_CTRL: DCASOP Mask             */
#define TIM14_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM14 CSD_CTRL: CSDTRG0 Position        */
#define TIM14_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM14_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM14 CSD_CTRL: CSDTRG0 Mask            */
#define TIM14_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM14 CSD_CTRL: CSDTRG1 Position        */
#define TIM14_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM14_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM14 CSD_CTRL: CSDTRG1 Mask            */
#define TIM14_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM14 CSD_CTRL: CSDEN2 Position         */
#define TIM14_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM14_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM14 CSD_CTRL: CSDEN2 Mask             */
#define TIM14_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM14 CSD_CTRL: CSDINV2 Position        */
#define TIM14_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM14_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM14 CSD_CTRL: CSDINV2 Mask            */
#define TIM14_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM14 CSD_CTRL: CSDTRG2 Position        */
#define TIM14_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM14_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM14 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM14_CASCADE0  ------------------------------- */
#define TIM14_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM14 CASCADE0: CASSEL Position         */
#define TIM14_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM14_CASCADE0_CASSEL_Pos)             /*!< TIM14 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM14_CASCADE1  ------------------------------- */
#define TIM14_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM14 CASCADE1: CASSEL Position         */
#define TIM14_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM14_CASCADE1_CASSEL_Pos)             /*!< TIM14 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM14_CASCADE2  ------------------------------- */
#define TIM14_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM14 CASCADE2: CASSEL Position         */
#define TIM14_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM14_CASCADE2_CASSEL_Pos)             /*!< TIM14 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM15' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM15_CTRL  --------------------------------- */
#define TIM15_CTRL_ENABLE_Pos                 0                                                       /*!< TIM15 CTRL: ENABLE Position             */
#define TIM15_CTRL_ENABLE_Msk                 (0x01UL << TIM15_CTRL_ENABLE_Pos)                       /*!< TIM15 CTRL: ENABLE Mask                 */
#define TIM15_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM15 CTRL: ACTIVE Position             */
#define TIM15_CTRL_ACTIVE_Msk                 (0x01UL << TIM15_CTRL_ACTIVE_Pos)                       /*!< TIM15 CTRL: ACTIVE Mask                 */
#define TIM15_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM15 CTRL: AUTO_DISABLE Position       */
#define TIM15_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM15_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM15 CTRL: AUTO_DISABLE Mask           */
#define TIM15_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM15 CTRL: AUTO_DEACTIVATE Position    */
#define TIM15_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM15_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM15 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM15_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM15 CTRL: IRQ_ENB Position            */
#define TIM15_CTRL_IRQ_ENB_Msk                (0x01UL << TIM15_CTRL_IRQ_ENB_Pos)                      /*!< TIM15 CTRL: IRQ_ENB Mask                */
#define TIM15_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM15 CTRL: STATUS_SEL Position         */
#define TIM15_CTRL_STATUS_SEL_Msk             (0x07UL << TIM15_CTRL_STATUS_SEL_Pos)                   /*!< TIM15 CTRL: STATUS_SEL Mask             */
#define TIM15_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM15 CTRL: STATUS_INV Position         */
#define TIM15_CTRL_STATUS_INV_Msk             (0x01UL << TIM15_CTRL_STATUS_INV_Pos)                   /*!< TIM15 CTRL: STATUS_INV Mask             */
#define TIM15_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM15 CTRL: REQ_STOP Position           */
#define TIM15_CTRL_REQ_STOP_Msk               (0x01UL << TIM15_CTRL_REQ_STOP_Pos)                     /*!< TIM15 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM15_ENABLE  -------------------------------- */
#define TIM15_ENABLE_ENABLE_Pos               0                                                       /*!< TIM15 ENABLE: ENABLE Position           */
#define TIM15_ENABLE_ENABLE_Msk               (0x01UL << TIM15_ENABLE_ENABLE_Pos)                     /*!< TIM15 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM15_CSD_CTRL  ------------------------------- */
#define TIM15_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM15 CSD_CTRL: CSDEN0 Position         */
#define TIM15_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM15_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM15 CSD_CTRL: CSDEN0 Mask             */
#define TIM15_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM15 CSD_CTRL: CSDINV0 Position        */
#define TIM15_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM15_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM15 CSD_CTRL: CSDINV0 Mask            */
#define TIM15_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM15 CSD_CTRL: CSDEN1 Position         */
#define TIM15_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM15_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM15 CSD_CTRL: CSDEN1 Mask             */
#define TIM15_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM15 CSD_CTRL: CSDINV1 Position        */
#define TIM15_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM15_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM15 CSD_CTRL: CSDINV1 Mask            */
#define TIM15_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM15 CSD_CTRL: DCASOP Position         */
#define TIM15_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM15_CSD_CTRL_DCASOP_Pos)                   /*!< TIM15 CSD_CTRL: DCASOP Mask             */
#define TIM15_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM15 CSD_CTRL: CSDTRG0 Position        */
#define TIM15_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM15_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM15 CSD_CTRL: CSDTRG0 Mask            */
#define TIM15_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM15 CSD_CTRL: CSDTRG1 Position        */
#define TIM15_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM15_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM15 CSD_CTRL: CSDTRG1 Mask            */
#define TIM15_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM15 CSD_CTRL: CSDEN2 Position         */
#define TIM15_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM15_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM15 CSD_CTRL: CSDEN2 Mask             */
#define TIM15_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM15 CSD_CTRL: CSDINV2 Position        */
#define TIM15_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM15_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM15 CSD_CTRL: CSDINV2 Mask            */
#define TIM15_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM15 CSD_CTRL: CSDTRG2 Position        */
#define TIM15_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM15_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM15 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM15_CASCADE0  ------------------------------- */
#define TIM15_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM15 CASCADE0: CASSEL Position         */
#define TIM15_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM15_CASCADE0_CASSEL_Pos)             /*!< TIM15 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM15_CASCADE1  ------------------------------- */
#define TIM15_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM15 CASCADE1: CASSEL Position         */
#define TIM15_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM15_CASCADE1_CASSEL_Pos)             /*!< TIM15 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM15_CASCADE2  ------------------------------- */
#define TIM15_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM15 CASCADE2: CASSEL Position         */
#define TIM15_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM15_CASCADE2_CASSEL_Pos)             /*!< TIM15 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM16' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM16_CTRL  --------------------------------- */
#define TIM16_CTRL_ENABLE_Pos                 0                                                       /*!< TIM16 CTRL: ENABLE Position             */
#define TIM16_CTRL_ENABLE_Msk                 (0x01UL << TIM16_CTRL_ENABLE_Pos)                       /*!< TIM16 CTRL: ENABLE Mask                 */
#define TIM16_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM16 CTRL: ACTIVE Position             */
#define TIM16_CTRL_ACTIVE_Msk                 (0x01UL << TIM16_CTRL_ACTIVE_Pos)                       /*!< TIM16 CTRL: ACTIVE Mask                 */
#define TIM16_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM16 CTRL: AUTO_DISABLE Position       */
#define TIM16_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM16_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM16 CTRL: AUTO_DISABLE Mask           */
#define TIM16_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM16 CTRL: AUTO_DEACTIVATE Position    */
#define TIM16_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM16_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM16 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM16_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM16 CTRL: IRQ_ENB Position            */
#define TIM16_CTRL_IRQ_ENB_Msk                (0x01UL << TIM16_CTRL_IRQ_ENB_Pos)                      /*!< TIM16 CTRL: IRQ_ENB Mask                */
#define TIM16_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM16 CTRL: STATUS_SEL Position         */
#define TIM16_CTRL_STATUS_SEL_Msk             (0x07UL << TIM16_CTRL_STATUS_SEL_Pos)                   /*!< TIM16 CTRL: STATUS_SEL Mask             */
#define TIM16_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM16 CTRL: STATUS_INV Position         */
#define TIM16_CTRL_STATUS_INV_Msk             (0x01UL << TIM16_CTRL_STATUS_INV_Pos)                   /*!< TIM16 CTRL: STATUS_INV Mask             */
#define TIM16_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM16 CTRL: REQ_STOP Position           */
#define TIM16_CTRL_REQ_STOP_Msk               (0x01UL << TIM16_CTRL_REQ_STOP_Pos)                     /*!< TIM16 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM16_ENABLE  -------------------------------- */
#define TIM16_ENABLE_ENABLE_Pos               0                                                       /*!< TIM16 ENABLE: ENABLE Position           */
#define TIM16_ENABLE_ENABLE_Msk               (0x01UL << TIM16_ENABLE_ENABLE_Pos)                     /*!< TIM16 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM16_CSD_CTRL  ------------------------------- */
#define TIM16_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM16 CSD_CTRL: CSDEN0 Position         */
#define TIM16_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM16_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM16 CSD_CTRL: CSDEN0 Mask             */
#define TIM16_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM16 CSD_CTRL: CSDINV0 Position        */
#define TIM16_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM16_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM16 CSD_CTRL: CSDINV0 Mask            */
#define TIM16_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM16 CSD_CTRL: CSDEN1 Position         */
#define TIM16_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM16_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM16 CSD_CTRL: CSDEN1 Mask             */
#define TIM16_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM16 CSD_CTRL: CSDINV1 Position        */
#define TIM16_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM16_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM16 CSD_CTRL: CSDINV1 Mask            */
#define TIM16_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM16 CSD_CTRL: DCASOP Position         */
#define TIM16_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM16_CSD_CTRL_DCASOP_Pos)                   /*!< TIM16 CSD_CTRL: DCASOP Mask             */
#define TIM16_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM16 CSD_CTRL: CSDTRG0 Position        */
#define TIM16_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM16_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM16 CSD_CTRL: CSDTRG0 Mask            */
#define TIM16_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM16 CSD_CTRL: CSDTRG1 Position        */
#define TIM16_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM16_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM16 CSD_CTRL: CSDTRG1 Mask            */
#define TIM16_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM16 CSD_CTRL: CSDEN2 Position         */
#define TIM16_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM16_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM16 CSD_CTRL: CSDEN2 Mask             */
#define TIM16_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM16 CSD_CTRL: CSDINV2 Position        */
#define TIM16_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM16_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM16 CSD_CTRL: CSDINV2 Mask            */
#define TIM16_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM16 CSD_CTRL: CSDTRG2 Position        */
#define TIM16_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM16_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM16 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM16_CASCADE0  ------------------------------- */
#define TIM16_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM16 CASCADE0: CASSEL Position         */
#define TIM16_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM16_CASCADE0_CASSEL_Pos)             /*!< TIM16 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM16_CASCADE1  ------------------------------- */
#define TIM16_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM16 CASCADE1: CASSEL Position         */
#define TIM16_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM16_CASCADE1_CASSEL_Pos)             /*!< TIM16 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM16_CASCADE2  ------------------------------- */
#define TIM16_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM16 CASCADE2: CASSEL Position         */
#define TIM16_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM16_CASCADE2_CASSEL_Pos)             /*!< TIM16 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM17' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM17_CTRL  --------------------------------- */
#define TIM17_CTRL_ENABLE_Pos                 0                                                       /*!< TIM17 CTRL: ENABLE Position             */
#define TIM17_CTRL_ENABLE_Msk                 (0x01UL << TIM17_CTRL_ENABLE_Pos)                       /*!< TIM17 CTRL: ENABLE Mask                 */
#define TIM17_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM17 CTRL: ACTIVE Position             */
#define TIM17_CTRL_ACTIVE_Msk                 (0x01UL << TIM17_CTRL_ACTIVE_Pos)                       /*!< TIM17 CTRL: ACTIVE Mask                 */
#define TIM17_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM17 CTRL: AUTO_DISABLE Position       */
#define TIM17_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM17_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM17 CTRL: AUTO_DISABLE Mask           */
#define TIM17_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM17 CTRL: AUTO_DEACTIVATE Position    */
#define TIM17_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM17_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM17 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM17_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM17 CTRL: IRQ_ENB Position            */
#define TIM17_CTRL_IRQ_ENB_Msk                (0x01UL << TIM17_CTRL_IRQ_ENB_Pos)                      /*!< TIM17 CTRL: IRQ_ENB Mask                */
#define TIM17_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM17 CTRL: STATUS_SEL Position         */
#define TIM17_CTRL_STATUS_SEL_Msk             (0x07UL << TIM17_CTRL_STATUS_SEL_Pos)                   /*!< TIM17 CTRL: STATUS_SEL Mask             */
#define TIM17_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM17 CTRL: STATUS_INV Position         */
#define TIM17_CTRL_STATUS_INV_Msk             (0x01UL << TIM17_CTRL_STATUS_INV_Pos)                   /*!< TIM17 CTRL: STATUS_INV Mask             */
#define TIM17_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM17 CTRL: REQ_STOP Position           */
#define TIM17_CTRL_REQ_STOP_Msk               (0x01UL << TIM17_CTRL_REQ_STOP_Pos)                     /*!< TIM17 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM17_ENABLE  -------------------------------- */
#define TIM17_ENABLE_ENABLE_Pos               0                                                       /*!< TIM17 ENABLE: ENABLE Position           */
#define TIM17_ENABLE_ENABLE_Msk               (0x01UL << TIM17_ENABLE_ENABLE_Pos)                     /*!< TIM17 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM17_CSD_CTRL  ------------------------------- */
#define TIM17_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM17 CSD_CTRL: CSDEN0 Position         */
#define TIM17_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM17_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM17 CSD_CTRL: CSDEN0 Mask             */
#define TIM17_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM17 CSD_CTRL: CSDINV0 Position        */
#define TIM17_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM17_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM17 CSD_CTRL: CSDINV0 Mask            */
#define TIM17_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM17 CSD_CTRL: CSDEN1 Position         */
#define TIM17_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM17_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM17 CSD_CTRL: CSDEN1 Mask             */
#define TIM17_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM17 CSD_CTRL: CSDINV1 Position        */
#define TIM17_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM17_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM17 CSD_CTRL: CSDINV1 Mask            */
#define TIM17_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM17 CSD_CTRL: DCASOP Position         */
#define TIM17_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM17_CSD_CTRL_DCASOP_Pos)                   /*!< TIM17 CSD_CTRL: DCASOP Mask             */
#define TIM17_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM17 CSD_CTRL: CSDTRG0 Position        */
#define TIM17_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM17_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM17 CSD_CTRL: CSDTRG0 Mask            */
#define TIM17_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM17 CSD_CTRL: CSDTRG1 Position        */
#define TIM17_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM17_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM17 CSD_CTRL: CSDTRG1 Mask            */
#define TIM17_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM17 CSD_CTRL: CSDEN2 Position         */
#define TIM17_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM17_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM17 CSD_CTRL: CSDEN2 Mask             */
#define TIM17_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM17 CSD_CTRL: CSDINV2 Position        */
#define TIM17_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM17_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM17 CSD_CTRL: CSDINV2 Mask            */
#define TIM17_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM17 CSD_CTRL: CSDTRG2 Position        */
#define TIM17_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM17_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM17 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM17_CASCADE0  ------------------------------- */
#define TIM17_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM17 CASCADE0: CASSEL Position         */
#define TIM17_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM17_CASCADE0_CASSEL_Pos)             /*!< TIM17 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM17_CASCADE1  ------------------------------- */
#define TIM17_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM17 CASCADE1: CASSEL Position         */
#define TIM17_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM17_CASCADE1_CASSEL_Pos)             /*!< TIM17 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM17_CASCADE2  ------------------------------- */
#define TIM17_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM17 CASCADE2: CASSEL Position         */
#define TIM17_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM17_CASCADE2_CASSEL_Pos)             /*!< TIM17 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM18' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM18_CTRL  --------------------------------- */
#define TIM18_CTRL_ENABLE_Pos                 0                                                       /*!< TIM18 CTRL: ENABLE Position             */
#define TIM18_CTRL_ENABLE_Msk                 (0x01UL << TIM18_CTRL_ENABLE_Pos)                       /*!< TIM18 CTRL: ENABLE Mask                 */
#define TIM18_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM18 CTRL: ACTIVE Position             */
#define TIM18_CTRL_ACTIVE_Msk                 (0x01UL << TIM18_CTRL_ACTIVE_Pos)                       /*!< TIM18 CTRL: ACTIVE Mask                 */
#define TIM18_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM18 CTRL: AUTO_DISABLE Position       */
#define TIM18_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM18_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM18 CTRL: AUTO_DISABLE Mask           */
#define TIM18_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM18 CTRL: AUTO_DEACTIVATE Position    */
#define TIM18_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM18_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM18 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM18_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM18 CTRL: IRQ_ENB Position            */
#define TIM18_CTRL_IRQ_ENB_Msk                (0x01UL << TIM18_CTRL_IRQ_ENB_Pos)                      /*!< TIM18 CTRL: IRQ_ENB Mask                */
#define TIM18_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM18 CTRL: STATUS_SEL Position         */
#define TIM18_CTRL_STATUS_SEL_Msk             (0x07UL << TIM18_CTRL_STATUS_SEL_Pos)                   /*!< TIM18 CTRL: STATUS_SEL Mask             */
#define TIM18_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM18 CTRL: STATUS_INV Position         */
#define TIM18_CTRL_STATUS_INV_Msk             (0x01UL << TIM18_CTRL_STATUS_INV_Pos)                   /*!< TIM18 CTRL: STATUS_INV Mask             */
#define TIM18_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM18 CTRL: REQ_STOP Position           */
#define TIM18_CTRL_REQ_STOP_Msk               (0x01UL << TIM18_CTRL_REQ_STOP_Pos)                     /*!< TIM18 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM18_ENABLE  -------------------------------- */
#define TIM18_ENABLE_ENABLE_Pos               0                                                       /*!< TIM18 ENABLE: ENABLE Position           */
#define TIM18_ENABLE_ENABLE_Msk               (0x01UL << TIM18_ENABLE_ENABLE_Pos)                     /*!< TIM18 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM18_CSD_CTRL  ------------------------------- */
#define TIM18_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM18 CSD_CTRL: CSDEN0 Position         */
#define TIM18_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM18_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM18 CSD_CTRL: CSDEN0 Mask             */
#define TIM18_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM18 CSD_CTRL: CSDINV0 Position        */
#define TIM18_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM18_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM18 CSD_CTRL: CSDINV0 Mask            */
#define TIM18_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM18 CSD_CTRL: CSDEN1 Position         */
#define TIM18_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM18_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM18 CSD_CTRL: CSDEN1 Mask             */
#define TIM18_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM18 CSD_CTRL: CSDINV1 Position        */
#define TIM18_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM18_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM18 CSD_CTRL: CSDINV1 Mask            */
#define TIM18_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM18 CSD_CTRL: DCASOP Position         */
#define TIM18_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM18_CSD_CTRL_DCASOP_Pos)                   /*!< TIM18 CSD_CTRL: DCASOP Mask             */
#define TIM18_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM18 CSD_CTRL: CSDTRG0 Position        */
#define TIM18_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM18_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM18 CSD_CTRL: CSDTRG0 Mask            */
#define TIM18_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM18 CSD_CTRL: CSDTRG1 Position        */
#define TIM18_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM18_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM18 CSD_CTRL: CSDTRG1 Mask            */
#define TIM18_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM18 CSD_CTRL: CSDEN2 Position         */
#define TIM18_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM18_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM18 CSD_CTRL: CSDEN2 Mask             */
#define TIM18_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM18 CSD_CTRL: CSDINV2 Position        */
#define TIM18_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM18_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM18 CSD_CTRL: CSDINV2 Mask            */
#define TIM18_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM18 CSD_CTRL: CSDTRG2 Position        */
#define TIM18_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM18_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM18 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM18_CASCADE0  ------------------------------- */
#define TIM18_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM18 CASCADE0: CASSEL Position         */
#define TIM18_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM18_CASCADE0_CASSEL_Pos)             /*!< TIM18 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM18_CASCADE1  ------------------------------- */
#define TIM18_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM18 CASCADE1: CASSEL Position         */
#define TIM18_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM18_CASCADE1_CASSEL_Pos)             /*!< TIM18 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM18_CASCADE2  ------------------------------- */
#define TIM18_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM18 CASCADE2: CASSEL Position         */
#define TIM18_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM18_CASCADE2_CASSEL_Pos)             /*!< TIM18 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM19' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM19_CTRL  --------------------------------- */
#define TIM19_CTRL_ENABLE_Pos                 0                                                       /*!< TIM19 CTRL: ENABLE Position             */
#define TIM19_CTRL_ENABLE_Msk                 (0x01UL << TIM19_CTRL_ENABLE_Pos)                       /*!< TIM19 CTRL: ENABLE Mask                 */
#define TIM19_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM19 CTRL: ACTIVE Position             */
#define TIM19_CTRL_ACTIVE_Msk                 (0x01UL << TIM19_CTRL_ACTIVE_Pos)                       /*!< TIM19 CTRL: ACTIVE Mask                 */
#define TIM19_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM19 CTRL: AUTO_DISABLE Position       */
#define TIM19_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM19_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM19 CTRL: AUTO_DISABLE Mask           */
#define TIM19_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM19 CTRL: AUTO_DEACTIVATE Position    */
#define TIM19_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM19_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM19 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM19_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM19 CTRL: IRQ_ENB Position            */
#define TIM19_CTRL_IRQ_ENB_Msk                (0x01UL << TIM19_CTRL_IRQ_ENB_Pos)                      /*!< TIM19 CTRL: IRQ_ENB Mask                */
#define TIM19_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM19 CTRL: STATUS_SEL Position         */
#define TIM19_CTRL_STATUS_SEL_Msk             (0x07UL << TIM19_CTRL_STATUS_SEL_Pos)                   /*!< TIM19 CTRL: STATUS_SEL Mask             */
#define TIM19_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM19 CTRL: STATUS_INV Position         */
#define TIM19_CTRL_STATUS_INV_Msk             (0x01UL << TIM19_CTRL_STATUS_INV_Pos)                   /*!< TIM19 CTRL: STATUS_INV Mask             */
#define TIM19_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM19 CTRL: REQ_STOP Position           */
#define TIM19_CTRL_REQ_STOP_Msk               (0x01UL << TIM19_CTRL_REQ_STOP_Pos)                     /*!< TIM19 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM19_ENABLE  -------------------------------- */
#define TIM19_ENABLE_ENABLE_Pos               0                                                       /*!< TIM19 ENABLE: ENABLE Position           */
#define TIM19_ENABLE_ENABLE_Msk               (0x01UL << TIM19_ENABLE_ENABLE_Pos)                     /*!< TIM19 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM19_CSD_CTRL  ------------------------------- */
#define TIM19_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM19 CSD_CTRL: CSDEN0 Position         */
#define TIM19_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM19_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM19 CSD_CTRL: CSDEN0 Mask             */
#define TIM19_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM19 CSD_CTRL: CSDINV0 Position        */
#define TIM19_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM19_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM19 CSD_CTRL: CSDINV0 Mask            */
#define TIM19_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM19 CSD_CTRL: CSDEN1 Position         */
#define TIM19_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM19_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM19 CSD_CTRL: CSDEN1 Mask             */
#define TIM19_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM19 CSD_CTRL: CSDINV1 Position        */
#define TIM19_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM19_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM19 CSD_CTRL: CSDINV1 Mask            */
#define TIM19_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM19 CSD_CTRL: DCASOP Position         */
#define TIM19_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM19_CSD_CTRL_DCASOP_Pos)                   /*!< TIM19 CSD_CTRL: DCASOP Mask             */
#define TIM19_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM19 CSD_CTRL: CSDTRG0 Position        */
#define TIM19_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM19_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM19 CSD_CTRL: CSDTRG0 Mask            */
#define TIM19_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM19 CSD_CTRL: CSDTRG1 Position        */
#define TIM19_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM19_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM19 CSD_CTRL: CSDTRG1 Mask            */
#define TIM19_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM19 CSD_CTRL: CSDEN2 Position         */
#define TIM19_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM19_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM19 CSD_CTRL: CSDEN2 Mask             */
#define TIM19_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM19 CSD_CTRL: CSDINV2 Position        */
#define TIM19_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM19_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM19 CSD_CTRL: CSDINV2 Mask            */
#define TIM19_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM19 CSD_CTRL: CSDTRG2 Position        */
#define TIM19_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM19_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM19 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM19_CASCADE0  ------------------------------- */
#define TIM19_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM19 CASCADE0: CASSEL Position         */
#define TIM19_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM19_CASCADE0_CASSEL_Pos)             /*!< TIM19 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM19_CASCADE1  ------------------------------- */
#define TIM19_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM19 CASCADE1: CASSEL Position         */
#define TIM19_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM19_CASCADE1_CASSEL_Pos)             /*!< TIM19 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM19_CASCADE2  ------------------------------- */
#define TIM19_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM19 CASCADE2: CASSEL Position         */
#define TIM19_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM19_CASCADE2_CASSEL_Pos)             /*!< TIM19 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM20' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM20_CTRL  --------------------------------- */
#define TIM20_CTRL_ENABLE_Pos                 0                                                       /*!< TIM20 CTRL: ENABLE Position             */
#define TIM20_CTRL_ENABLE_Msk                 (0x01UL << TIM20_CTRL_ENABLE_Pos)                       /*!< TIM20 CTRL: ENABLE Mask                 */
#define TIM20_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM20 CTRL: ACTIVE Position             */
#define TIM20_CTRL_ACTIVE_Msk                 (0x01UL << TIM20_CTRL_ACTIVE_Pos)                       /*!< TIM20 CTRL: ACTIVE Mask                 */
#define TIM20_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM20 CTRL: AUTO_DISABLE Position       */
#define TIM20_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM20_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM20 CTRL: AUTO_DISABLE Mask           */
#define TIM20_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM20 CTRL: AUTO_DEACTIVATE Position    */
#define TIM20_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM20_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM20 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM20_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM20 CTRL: IRQ_ENB Position            */
#define TIM20_CTRL_IRQ_ENB_Msk                (0x01UL << TIM20_CTRL_IRQ_ENB_Pos)                      /*!< TIM20 CTRL: IRQ_ENB Mask                */
#define TIM20_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM20 CTRL: STATUS_SEL Position         */
#define TIM20_CTRL_STATUS_SEL_Msk             (0x07UL << TIM20_CTRL_STATUS_SEL_Pos)                   /*!< TIM20 CTRL: STATUS_SEL Mask             */
#define TIM20_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM20 CTRL: STATUS_INV Position         */
#define TIM20_CTRL_STATUS_INV_Msk             (0x01UL << TIM20_CTRL_STATUS_INV_Pos)                   /*!< TIM20 CTRL: STATUS_INV Mask             */
#define TIM20_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM20 CTRL: REQ_STOP Position           */
#define TIM20_CTRL_REQ_STOP_Msk               (0x01UL << TIM20_CTRL_REQ_STOP_Pos)                     /*!< TIM20 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM20_ENABLE  -------------------------------- */
#define TIM20_ENABLE_ENABLE_Pos               0                                                       /*!< TIM20 ENABLE: ENABLE Position           */
#define TIM20_ENABLE_ENABLE_Msk               (0x01UL << TIM20_ENABLE_ENABLE_Pos)                     /*!< TIM20 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM20_CSD_CTRL  ------------------------------- */
#define TIM20_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM20 CSD_CTRL: CSDEN0 Position         */
#define TIM20_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM20_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM20 CSD_CTRL: CSDEN0 Mask             */
#define TIM20_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM20 CSD_CTRL: CSDINV0 Position        */
#define TIM20_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM20_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM20 CSD_CTRL: CSDINV0 Mask            */
#define TIM20_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM20 CSD_CTRL: CSDEN1 Position         */
#define TIM20_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM20_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM20 CSD_CTRL: CSDEN1 Mask             */
#define TIM20_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM20 CSD_CTRL: CSDINV1 Position        */
#define TIM20_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM20_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM20 CSD_CTRL: CSDINV1 Mask            */
#define TIM20_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM20 CSD_CTRL: DCASOP Position         */
#define TIM20_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM20_CSD_CTRL_DCASOP_Pos)                   /*!< TIM20 CSD_CTRL: DCASOP Mask             */
#define TIM20_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM20 CSD_CTRL: CSDTRG0 Position        */
#define TIM20_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM20_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM20 CSD_CTRL: CSDTRG0 Mask            */
#define TIM20_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM20 CSD_CTRL: CSDTRG1 Position        */
#define TIM20_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM20_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM20 CSD_CTRL: CSDTRG1 Mask            */
#define TIM20_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM20 CSD_CTRL: CSDEN2 Position         */
#define TIM20_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM20_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM20 CSD_CTRL: CSDEN2 Mask             */
#define TIM20_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM20 CSD_CTRL: CSDINV2 Position        */
#define TIM20_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM20_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM20 CSD_CTRL: CSDINV2 Mask            */
#define TIM20_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM20 CSD_CTRL: CSDTRG2 Position        */
#define TIM20_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM20_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM20 CSD_CTRL: CSDTRG2 Mask            */

/* -------------------------------  TIM20_CASCADE0  ------------------------------- */
#define TIM20_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM20 CASCADE0: CASSEL Position         */
#define TIM20_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM20_CASCADE0_CASSEL_Pos)             /*!< TIM20 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM20_CASCADE1  ------------------------------- */
#define TIM20_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM20 CASCADE1: CASSEL Position         */
#define TIM20_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM20_CASCADE1_CASSEL_Pos)             /*!< TIM20 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM20_CASCADE2  ------------------------------- */
#define TIM20_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM20 CASCADE2: CASSEL Position         */
#define TIM20_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM20_CASCADE2_CASSEL_Pos)             /*!< TIM20 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM21' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM21_CTRL  --------------------------------- */
#define TIM21_CTRL_ENABLE_Pos                 0                                                       /*!< TIM21 CTRL: ENABLE Position             */
#define TIM21_CTRL_ENABLE_Msk                 (0x01UL << TIM21_CTRL_ENABLE_Pos)                       /*!< TIM21 CTRL: ENABLE Mask                 */
#define TIM21_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM21 CTRL: ACTIVE Position             */
#define TIM21_CTRL_ACTIVE_Msk                 (0x01UL << TIM21_CTRL_ACTIVE_Pos)                       /*!< TIM21 CTRL: ACTIVE Mask                 */
#define TIM21_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM21 CTRL: AUTO_DISABLE Position       */
#define TIM21_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM21_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM21 CTRL: AUTO_DISABLE Mask           */
#define TIM21_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM21 CTRL: AUTO_DEACTIVATE Position    */
#define TIM21_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM21_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM21 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM21_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM21 CTRL: IRQ_ENB Position            */
#define TIM21_CTRL_IRQ_ENB_Msk                (0x01UL << TIM21_CTRL_IRQ_ENB_Pos)                      /*!< TIM21 CTRL: IRQ_ENB Mask                */
#define TIM21_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM21 CTRL: STATUS_SEL Position         */
#define TIM21_CTRL_STATUS_SEL_Msk             (0x07UL << TIM21_CTRL_STATUS_SEL_Pos)                   /*!< TIM21 CTRL: STATUS_SEL Mask             */
#define TIM21_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM21 CTRL: STATUS_INV Position         */
#define TIM21_CTRL_STATUS_INV_Msk             (0x01UL << TIM21_CTRL_STATUS_INV_Pos)                   /*!< TIM21 CTRL: STATUS_INV Mask             */
#define TIM21_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM21 CTRL: REQ_STOP Position           */
#define TIM21_CTRL_REQ_STOP_Msk               (0x01UL << TIM21_CTRL_REQ_STOP_Pos)                     /*!< TIM21 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM21_ENABLE  -------------------------------- */
#define TIM21_ENABLE_ENABLE_Pos               0                                                       /*!< TIM21 ENABLE: ENABLE Position           */
#define TIM21_ENABLE_ENABLE_Msk               (0x01UL << TIM21_ENABLE_ENABLE_Pos)                     /*!< TIM21 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM21_CSD_CTRL  ------------------------------- */
#define TIM21_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM21 CSD_CTRL: CSDEN0 Position         */
#define TIM21_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM21_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM21 CSD_CTRL: CSDEN0 Mask             */
#define TIM21_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM21 CSD_CTRL: CSDINV0 Position        */
#define TIM21_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM21_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM21 CSD_CTRL: CSDINV0 Mask            */
#define TIM21_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM21 CSD_CTRL: CSDEN1 Position         */
#define TIM21_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM21_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM21 CSD_CTRL: CSDEN1 Mask             */
#define TIM21_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM21 CSD_CTRL: CSDINV1 Position        */
#define TIM21_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM21_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM21 CSD_CTRL: CSDINV1 Mask            */
#define TIM21_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM21 CSD_CTRL: DCASOP Position         */
#define TIM21_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM21_CSD_CTRL_DCASOP_Pos)                   /*!< TIM21 CSD_CTRL: DCASOP Mask             */
#define TIM21_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM21 CSD_CTRL: CSDTRG0 Position        */
#define TIM21_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM21_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM21 CSD_CTRL: CSDTRG0 Mask            */
#define TIM21_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM21 CSD_CTRL: CSDTRG1 Position        */
#define TIM21_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM21_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM21 CSD_CTRL: CSDTRG1 Mask            */
#define TIM21_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM21 CSD_CTRL: CSDEN2 Position         */
#define TIM21_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM21_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM21 CSD_CTRL: CSDEN2 Mask             */
#define TIM21_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM21 CSD_CTRL: CSDINV2 Position        */
#define TIM21_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM21_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM21 CSD_CTRL: CSDINV2 Mask            */
#define TIM21_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM21 CSD_CTRL: CSDTRG2 Position        */
#define TIM21_CSD_CTRL_CSDTRG2_Msk           (0x01UL << TIM21_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM21 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM21_CASCADE0  ------------------------------- */
#define TIM21_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM21 CASCADE0: CASSEL Position         */
#define TIM21_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM21_CASCADE0_CASSEL_Pos)             /*!< TIM21 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM21_CASCADE1  ------------------------------- */
#define TIM21_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM21 CASCADE1: CASSEL Position         */
#define TIM21_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM21_CASCADE1_CASSEL_Pos)             /*!< TIM21 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM21_CASCADE2  ------------------------------- */
#define TIM21_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM21 CASCADE2: CASSEL Position         */
#define TIM21_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM21_CASCADE2_CASSEL_Pos)             /*!< TIM21 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM22' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM22_CTRL  --------------------------------- */
#define TIM22_CTRL_ENABLE_Pos                 0                                                       /*!< TIM22 CTRL: ENABLE Position             */
#define TIM22_CTRL_ENABLE_Msk                 (0x01UL << TIM22_CTRL_ENABLE_Pos)                       /*!< TIM22 CTRL: ENABLE Mask                 */
#define TIM22_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM22 CTRL: ACTIVE Position             */
#define TIM22_CTRL_ACTIVE_Msk                 (0x01UL << TIM22_CTRL_ACTIVE_Pos)                       /*!< TIM22 CTRL: ACTIVE Mask                 */
#define TIM22_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM22 CTRL: AUTO_DISABLE Position       */
#define TIM22_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM22_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM22 CTRL: AUTO_DISABLE Mask           */
#define TIM22_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM22 CTRL: AUTO_DEACTIVATE Position    */
#define TIM22_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM22_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM22 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM22_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM22 CTRL: IRQ_ENB Position            */
#define TIM22_CTRL_IRQ_ENB_Msk                (0x01UL << TIM22_CTRL_IRQ_ENB_Pos)                      /*!< TIM22 CTRL: IRQ_ENB Mask                */
#define TIM22_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM22 CTRL: STATUS_SEL Position         */
#define TIM22_CTRL_STATUS_SEL_Msk             (0x07UL << TIM22_CTRL_STATUS_SEL_Pos)                   /*!< TIM22 CTRL: STATUS_SEL Mask             */
#define TIM22_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM22 CTRL: STATUS_INV Position         */
#define TIM22_CTRL_STATUS_INV_Msk             (0x01UL << TIM22_CTRL_STATUS_INV_Pos)                   /*!< TIM22 CTRL: STATUS_INV Mask             */
#define TIM22_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM22 CTRL: REQ_STOP Position           */
#define TIM22_CTRL_REQ_STOP_Msk               (0x01UL << TIM22_CTRL_REQ_STOP_Pos)                     /*!< TIM22 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM22_ENABLE  -------------------------------- */
#define TIM22_ENABLE_ENABLE_Pos               0                                                       /*!< TIM22 ENABLE: ENABLE Position           */
#define TIM22_ENABLE_ENABLE_Msk               (0x01UL << TIM22_ENABLE_ENABLE_Pos)                     /*!< TIM22 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM22_CSD_CTRL  ------------------------------- */
#define TIM22_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM22 CSD_CTRL: CSDEN0 Position         */
#define TIM22_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM22_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM22 CSD_CTRL: CSDEN0 Mask             */
#define TIM22_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM22 CSD_CTRL: CSDINV0 Position        */
#define TIM22_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM22_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM22 CSD_CTRL: CSDINV0 Mask            */
#define TIM22_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM22 CSD_CTRL: CSDEN1 Position         */
#define TIM22_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM22_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM22 CSD_CTRL: CSDEN1 Mask             */
#define TIM22_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM22 CSD_CTRL: CSDINV1 Position        */
#define TIM22_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM22_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM22 CSD_CTRL: CSDINV1 Mask            */
#define TIM22_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM22 CSD_CTRL: DCASOP Position         */
#define TIM22_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM22_CSD_CTRL_DCASOP_Pos)                   /*!< TIM22 CSD_CTRL: DCASOP Mask             */
#define TIM22_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM22 CSD_CTRL: CSDTRG0 Position        */
#define TIM22_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM22_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM22 CSD_CTRL: CSDTRG0 Mask            */
#define TIM22_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM22 CSD_CTRL: CSDTRG1 Position        */
#define TIM22_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM22_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM22 CSD_CTRL: CSDTRG1 Mask            */
#define TIM22_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM22 CSD_CTRL: CSDEN2 Position         */
#define TIM22_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM22_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM22 CSD_CTRL: CSDEN2 Mask             */
#define TIM22_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM22 CSD_CTRL: CSDINV2 Position        */
#define TIM22_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM22_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM22 CSD_CTRL: CSDINV2 Mask            */
#define TIM22_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM22 CSD_CTRL: CSDTRG2 Position        */
#define TIM22_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM22_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM22 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM22_CASCADE0  ------------------------------- */
#define TIM22_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM22 CASCADE0: CASSEL Position         */
#define TIM22_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM22_CASCADE0_CASSEL_Pos)             /*!< TIM22 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM22_CASCADE1  ------------------------------- */
#define TIM22_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM22 CASCADE1: CASSEL Position         */
#define TIM22_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM22_CASCADE1_CASSEL_Pos)             /*!< TIM22 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM22_CASCADE2  ------------------------------- */
#define TIM22_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM22 CASCADE2: CASSEL Position         */
#define TIM22_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM22_CASCADE2_CASSEL_Pos)             /*!< TIM22 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================         struct 'TIM23' Position & Mask         ================ */
/* ================================================================================ */


/* ---------------------------------  TIM23_CTRL  --------------------------------- */
#define TIM23_CTRL_ENABLE_Pos                 0                                                       /*!< TIM23 CTRL: ENABLE Position             */
#define TIM23_CTRL_ENABLE_Msk                 (0x01UL << TIM23_CTRL_ENABLE_Pos)                       /*!< TIM23 CTRL: ENABLE Mask                 */
#define TIM23_CTRL_ACTIVE_Pos                 1                                                       /*!< TIM23 CTRL: ACTIVE Position             */
#define TIM23_CTRL_ACTIVE_Msk                 (0x01UL << TIM23_CTRL_ACTIVE_Pos)                       /*!< TIM23 CTRL: ACTIVE Mask                 */
#define TIM23_CTRL_AUTO_DISABLE_Pos           2                                                       /*!< TIM23 CTRL: AUTO_DISABLE Position       */
#define TIM23_CTRL_AUTO_DISABLE_Msk           (0x01UL << TIM23_CTRL_AUTO_DISABLE_Pos)                 /*!< TIM23 CTRL: AUTO_DISABLE Mask           */
#define TIM23_CTRL_AUTO_DEACTIVATE_Pos        3                                                       /*!< TIM23 CTRL: AUTO_DEACTIVATE Position    */
#define TIM23_CTRL_AUTO_DEACTIVATE_Msk        (0x01UL << TIM23_CTRL_AUTO_DEACTIVATE_Pos)              /*!< TIM23 CTRL: AUTO_DEACTIVATE Mask        */
#define TIM23_CTRL_IRQ_ENB_Pos                4                                                       /*!< TIM23 CTRL: IRQ_ENB Position            */
#define TIM23_CTRL_IRQ_ENB_Msk                (0x01UL << TIM23_CTRL_IRQ_ENB_Pos)                      /*!< TIM23 CTRL: IRQ_ENB Mask                */
#define TIM23_CTRL_STATUS_SEL_Pos             5                                                       /*!< TIM23 CTRL: STATUS_SEL Position         */
#define TIM23_CTRL_STATUS_SEL_Msk             (0x07UL << TIM23_CTRL_STATUS_SEL_Pos)                   /*!< TIM23 CTRL: STATUS_SEL Mask             */
#define TIM23_CTRL_STATUS_INV_Pos             8                                                       /*!< TIM23 CTRL: STATUS_INV Position         */
#define TIM23_CTRL_STATUS_INV_Msk             (0x01UL << TIM23_CTRL_STATUS_INV_Pos)                   /*!< TIM23 CTRL: STATUS_INV Mask             */
#define TIM23_CTRL_REQ_STOP_Pos               9                                                       /*!< TIM23 CTRL: REQ_STOP Position           */
#define TIM23_CTRL_REQ_STOP_Msk               (0x01UL << TIM23_CTRL_REQ_STOP_Pos)                     /*!< TIM23 CTRL: REQ_STOP Mask               */

/* --------------------------------  TIM23_ENABLE  -------------------------------- */
#define TIM23_ENABLE_ENABLE_Pos               0                                                       /*!< TIM23 ENABLE: ENABLE Position           */
#define TIM23_ENABLE_ENABLE_Msk               (0x01UL << TIM23_ENABLE_ENABLE_Pos)                     /*!< TIM23 ENABLE: ENABLE Mask               */

/* -------------------------------  TIM23_CSD_CTRL  ------------------------------- */
#define TIM23_CSD_CTRL_CSDEN0_Pos             0                                                       /*!< TIM23 CSD_CTRL: CSDEN0 Position         */
#define TIM23_CSD_CTRL_CSDEN0_Msk             (0x01UL << TIM23_CSD_CTRL_CSDEN0_Pos)                   /*!< TIM23 CSD_CTRL: CSDEN0 Mask             */
#define TIM23_CSD_CTRL_CSDINV0_Pos            1                                                       /*!< TIM23 CSD_CTRL: CSDINV0 Position        */
#define TIM23_CSD_CTRL_CSDINV0_Msk            (0x01UL << TIM23_CSD_CTRL_CSDINV0_Pos)                  /*!< TIM23 CSD_CTRL: CSDINV0 Mask            */
#define TIM23_CSD_CTRL_CSDEN1_Pos             2                                                       /*!< TIM23 CSD_CTRL: CSDEN1 Position         */
#define TIM23_CSD_CTRL_CSDEN1_Msk             (0x01UL << TIM23_CSD_CTRL_CSDEN1_Pos)                   /*!< TIM23 CSD_CTRL: CSDEN1 Mask             */
#define TIM23_CSD_CTRL_CSDINV1_Pos            3                                                       /*!< TIM23 CSD_CTRL: CSDINV1 Position        */
#define TIM23_CSD_CTRL_CSDINV1_Msk            (0x01UL << TIM23_CSD_CTRL_CSDINV1_Pos)                  /*!< TIM23 CSD_CTRL: CSDINV1 Mask            */
#define TIM23_CSD_CTRL_DCASOP_Pos             4                                                       /*!< TIM23 CSD_CTRL: DCASOP Position         */
#define TIM23_CSD_CTRL_DCASOP_Msk             (0x01UL << TIM23_CSD_CTRL_DCASOP_Pos)                   /*!< TIM23 CSD_CTRL: DCASOP Mask             */
#define TIM23_CSD_CTRL_CSDTRG0_Pos            6                                                       /*!< TIM23 CSD_CTRL: CSDTRG0 Position        */
#define TIM23_CSD_CTRL_CSDTRG0_Msk            (0x01UL << TIM23_CSD_CTRL_CSDTRG0_Pos)                  /*!< TIM23 CSD_CTRL: CSDTRG0 Mask            */
#define TIM23_CSD_CTRL_CSDTRG1_Pos            7                                                       /*!< TIM23 CSD_CTRL: CSDTRG1 Position        */
#define TIM23_CSD_CTRL_CSDTRG1_Msk            (0x01UL << TIM23_CSD_CTRL_CSDTRG1_Pos)                  /*!< TIM23 CSD_CTRL: CSDTRG1 Mask            */
#define TIM23_CSD_CTRL_CSDEN2_Pos             8                                                       /*!< TIM23 CSD_CTRL: CSDEN2 Position         */
#define TIM23_CSD_CTRL_CSDEN2_Msk             (0x01UL << TIM23_CSD_CTRL_CSDEN2_Pos)                   /*!< TIM23 CSD_CTRL: CSDEN2 Mask             */
#define TIM23_CSD_CTRL_CSDINV2_Pos            9                                                       /*!< TIM23 CSD_CTRL: CSDINV2 Position        */
#define TIM23_CSD_CTRL_CSDINV2_Msk            (0x01UL << TIM23_CSD_CTRL_CSDINV2_Pos)                  /*!< TIM23 CSD_CTRL: CSDINV2 Mask            */
#define TIM23_CSD_CTRL_CSDTRG2_Pos            10                                                       /*!< TIM23 CSD_CTRL: CSDTRG2 Position        */
#define TIM23_CSD_CTRL_CSDTRG2_Msk            (0x01UL << TIM23_CSD_CTRL_CSDTRG2_Pos)                  /*!< TIM23 CSD_CTRL: CSDTRG2 Mask            */


/* -------------------------------  TIM23_CASCADE0  ------------------------------- */
#define TIM23_CASCADE0_CASSEL_Pos             0                                                       /*!< TIM23 CASCADE0: CASSEL Position         */
#define TIM23_CASCADE0_CASSEL_Msk             (0x000000ffUL << TIM23_CASCADE0_CASSEL_Pos)             /*!< TIM23 CASCADE0: CASSEL Mask             */

/* -------------------------------  TIM23_CASCADE1  ------------------------------- */
#define TIM23_CASCADE1_CASSEL_Pos             0                                                       /*!< TIM23 CASCADE1: CASSEL Position         */
#define TIM23_CASCADE1_CASSEL_Msk             (0x000000ffUL << TIM23_CASCADE1_CASSEL_Pos)             /*!< TIM23 CASCADE1: CASSEL Mask             */

/* -------------------------------  TIM23_CASCADE2  ------------------------------- */
#define TIM23_CASCADE2_CASSEL_Pos             0                                                       /*!< TIM23 CASCADE2: CASSEL Position         */
#define TIM23_CASCADE2_CASSEL_Msk             (0x000000ffUL << TIM23_CASCADE2_CASSEL_Pos)             /*!< TIM23 CASCADE2: CASSEL Mask             */


/* ================================================================================ */
/* ================     Group 'UART_PERIPHERAL' Position & Mask    ================ */
/* ================================================================================ */

/* ---------------------------  UART_PERIPHERAL_DATA  --------------------------- */
#define UART_PERIPHERAL_DATA_VALUE_Pos   			0                                                       /*!< UART_PERIPHERAL DATA: VALUE Position */
#define UART_PERIPHERAL_DATA_VALUE_Msk   			(0x000000ffUL << UART_PERIPHERAL_DATA_VALUE_Pos)   			/*!< UART_PERIPHERAL DATA: VALUE Mask   */
#define UART_PERIPHERAL_DATA_DPARITY_Pos   		15                                                      /*!< UART_PERIPHERAL DATA: DPARITY Position */
#define UART_PERIPHERAL_DATA_DPARITY_Msk   		(0x01UL << UART_PERIPHERAL_DATA_DPARITY_Pos)         		/*!< UART_PERIPHERAL DATA: DPARITY Mask   */

/* ---------------------------  UART_PERIPHERAL_ENABLE  --------------------------- */
#define UART_PERIPHERAL_ENABLE_RXENABLE_Pos   0                                                       /*!< UART_PERIPHERAL ENABLE: RXENABLE Position */
#define UART_PERIPHERAL_ENABLE_RXENABLE_Msk   (0x01UL << UART_PERIPHERAL_ENABLE_RXENABLE_Pos)         /*!< UART_PERIPHERAL ENABLE: RXENABLE Mask   */
#define UART_PERIPHERAL_ENABLE_TXENABLE_Pos   1                                                       /*!< UART_PERIPHERAL ENABLE: TXENABLE Position */
#define UART_PERIPHERAL_ENABLE_TXENABLE_Msk   (0x01UL << UART_PERIPHERAL_ENABLE_TXENABLE_Pos)         /*!< UART_PERIPHERAL ENABLE: TXENABLE Mask   */

/* ----------------------------  UART_PERIPHERAL_CTRL  ---------------------------- */
#define UART_PERIPHERAL_CTRL_PAREN_Pos        0                                                       /*!< UART_PERIPHERAL CTRL: PAREN Position    */
#define UART_PERIPHERAL_CTRL_PAREN_Msk        (0x01UL << UART_PERIPHERAL_CTRL_PAREN_Pos)              /*!< UART_PERIPHERAL CTRL: PAREN Mask        */
#define UART_PERIPHERAL_CTRL_PAREVEN_Pos      1                                                       /*!< UART_PERIPHERAL CTRL: PAREVEN Position  */
#define UART_PERIPHERAL_CTRL_PAREVEN_Msk      (0x01UL << UART_PERIPHERAL_CTRL_PAREVEN_Pos)            /*!< UART_PERIPHERAL CTRL: PAREVEN Mask      */
#define UART_PERIPHERAL_CTRL_PARSTK_Pos       2                                                       /*!< UART_PERIPHERAL CTRL: PARSTK Position   */
#define UART_PERIPHERAL_CTRL_PARSTK_Msk       (0x01UL << UART_PERIPHERAL_CTRL_PARSTK_Pos)             /*!< UART_PERIPHERAL CTRL: PARSTK Mask       */
#define UART_PERIPHERAL_CTRL_STOPBITS_Pos     3                                                       /*!< UART_PERIPHERAL CTRL: STOPBITS Position */
#define UART_PERIPHERAL_CTRL_STOPBITS_Msk     (0x01UL << UART_PERIPHERAL_CTRL_STOPBITS_Pos)           /*!< UART_PERIPHERAL CTRL: STOPBITS Mask     */
#define UART_PERIPHERAL_CTRL_WORDSIZE_Pos     4                                                       /*!< UART_PERIPHERAL CTRL: WORDSIZE Position */
#define UART_PERIPHERAL_CTRL_WORDSIZE_Msk     (0x03UL << UART_PERIPHERAL_CTRL_WORDSIZE_Pos)           /*!< UART_PERIPHERAL CTRL: WORDSIZE Mask     */
#define UART_PERIPHERAL_CTRL_LOOPBACK_Pos     6                                                       /*!< UART_PERIPHERAL CTRL: LOOPBACK Position */
#define UART_PERIPHERAL_CTRL_LOOPBACK_Msk     (0x01UL << UART_PERIPHERAL_CTRL_LOOPBACK_Pos)           /*!< UART_PERIPHERAL CTRL: LOOPBACK Mask     */
#define UART_PERIPHERAL_CTRL_LOOPBACKBLK_Pos  7                                                       /*!< UART_PERIPHERAL CTRL: LOOPBACKBLK Position */
#define UART_PERIPHERAL_CTRL_LOOPBACKBLK_Msk  (0x01UL << UART_PERIPHERAL_CTRL_LOOPBACKBLK_Pos)        /*!< UART_PERIPHERAL CTRL: LOOPBACKBLK Mask  */
#define UART_PERIPHERAL_CTRL_AUTOCTS_Pos      8                                                       /*!< UART_PERIPHERAL CTRL: AUTOCTS Position  */
#define UART_PERIPHERAL_CTRL_AUTOCTS_Msk      (0x01UL << UART_PERIPHERAL_CTRL_AUTOCTS_Pos)            /*!< UART_PERIPHERAL CTRL: AUTOCTS Mask      */
#define UART_PERIPHERAL_CTRL_DEFRTS_Pos       9                                                       /*!< UART_PERIPHERAL CTRL: DEFRTS Position   */
#define UART_PERIPHERAL_CTRL_DEFRTS_Msk       (0x01UL << UART_PERIPHERAL_CTRL_DEFRTS_Pos)             /*!< UART_PERIPHERAL CTRL: DEFRTS Mask       */
#define UART_PERIPHERAL_CTRL_AUTORTS_Pos      10                                                      /*!< UART_PERIPHERAL CTRL: AUTORTS Position  */
#define UART_PERIPHERAL_CTRL_AUTORTS_Msk      (0x01UL << UART_PERIPHERAL_CTRL_AUTORTS_Pos)            /*!< UART_PERIPHERAL CTRL: AUTORTS Mask      */
#define UART_PERIPHERAL_CTRL_BAUD8_Pos        11                                                      /*!< UART_PERIPHERAL CTRL: BAUD8 Position    */
#define UART_PERIPHERAL_CTRL_BAUD8_Msk        (0x01UL << UART_PERIPHERAL_CTRL_BAUD8_Pos)              /*!< UART_PERIPHERAL CTRL: BAUD8 Mask        */

/* --------------------------  UART_PERIPHERAL_CLKSCALE  -------------------------- */
#define UART_PERIPHERAL_CLKSCALE_FRAC_Pos     0                                                       /*!< UART_PERIPHERAL CLKSCALE: FRAC Position */
#define UART_PERIPHERAL_CLKSCALE_FRAC_Msk     (0x3fUL << UART_PERIPHERAL_CLKSCALE_FRAC_Pos)           /*!< UART_PERIPHERAL CLKSCALE: FRAC Mask     */
#define UART_PERIPHERAL_CLKSCALE_INT_Pos      6                                                       /*!< UART_PERIPHERAL CLKSCALE: INT Position  */
#define UART_PERIPHERAL_CLKSCALE_INT_Msk      (0x0003ffffUL << UART_PERIPHERAL_CLKSCALE_INT_Pos)      /*!< UART_PERIPHERAL CLKSCALE: INT Mask      */
#define UART_PERIPHERAL_CLKSCALE_RESET_Pos    31                                                      /*!< UART_PERIPHERAL CLKSCALE: RESET Position */
#define UART_PERIPHERAL_CLKSCALE_RESET_Msk    (0x01UL << UART_PERIPHERAL_CLKSCALE_RESET_Pos)          /*!< UART_PERIPHERAL CLKSCALE: RESET Mask    */

/* --------------------------  UART_PERIPHERAL_RXSTATUS  -------------------------- */
#define UART_PERIPHERAL_RXSTATUS_RDAVL_Pos    0                                                       /*!< UART_PERIPHERAL RXSTATUS: RDAVL Position */
#define UART_PERIPHERAL_RXSTATUS_RDAVL_Msk    (0x01UL << UART_PERIPHERAL_RXSTATUS_RDAVL_Pos)          /*!< UART_PERIPHERAL RXSTATUS: RDAVL Mask    */
#define UART_PERIPHERAL_RXSTATUS_RDNFULL_Pos  1                                                       /*!< UART_PERIPHERAL RXSTATUS: RDNFULL Position */
#define UART_PERIPHERAL_RXSTATUS_RDNFULL_Msk  (0x01UL << UART_PERIPHERAL_RXSTATUS_RDNFULL_Pos)        /*!< UART_PERIPHERAL RXSTATUS: RDNFULL Mask  */
#define UART_PERIPHERAL_RXSTATUS_RXBUSY_Pos   2                                                       /*!< UART_PERIPHERAL RXSTATUS: RXBUSY Position */
#define UART_PERIPHERAL_RXSTATUS_RXBUSY_Msk   (0x01UL << UART_PERIPHERAL_RXSTATUS_RXBUSY_Pos)         /*!< UART_PERIPHERAL RXSTATUS: RXBUSY Mask   */
#define UART_PERIPHERAL_RXSTATUS_RXTO_Pos     3                                                       /*!< UART_PERIPHERAL RXSTATUS: RXTO Position */
#define UART_PERIPHERAL_RXSTATUS_RXTO_Msk     (0x01UL << UART_PERIPHERAL_RXSTATUS_RXTO_Pos)           /*!< UART_PERIPHERAL RXSTATUS: RXTO Mask     */
#define UART_PERIPHERAL_RXSTATUS_RXOVR_Pos    4                                                       /*!< UART_PERIPHERAL RXSTATUS: RXOVR Position */
#define UART_PERIPHERAL_RXSTATUS_RXOVR_Msk    (0x01UL << UART_PERIPHERAL_RXSTATUS_RXOVR_Pos)          /*!< UART_PERIPHERAL RXSTATUS: RXOVR Mask    */
#define UART_PERIPHERAL_RXSTATUS_RXFRM_Pos    5                                                       /*!< UART_PERIPHERAL RXSTATUS: RXFRM Position */
#define UART_PERIPHERAL_RXSTATUS_RXFRM_Msk    (0x01UL << UART_PERIPHERAL_RXSTATUS_RXFRM_Pos)          /*!< UART_PERIPHERAL RXSTATUS: RXFRM Mask    */
#define UART_PERIPHERAL_RXSTATUS_RXPAR_Pos    6                                                       /*!< UART_PERIPHERAL RXSTATUS: RXPAR Position */
#define UART_PERIPHERAL_RXSTATUS_RXPAR_Msk    (0x01UL << UART_PERIPHERAL_RXSTATUS_RXPAR_Pos)          /*!< UART_PERIPHERAL RXSTATUS: RXPAR Mask    */
#define UART_PERIPHERAL_RXSTATUS_RXBRK_Pos    7                                                       /*!< UART_PERIPHERAL RXSTATUS: RXBRK Position */
#define UART_PERIPHERAL_RXSTATUS_RXBRK_Msk    (0x01UL << UART_PERIPHERAL_RXSTATUS_RXBRK_Pos)          /*!< UART_PERIPHERAL RXSTATUS: RXBRK Mask    */
#define UART_PERIPHERAL_RXSTATUS_RXBUSYBRK_Pos 8                                                      /*!< UART_PERIPHERAL RXSTATUS: RXBUSYBRK Position */
#define UART_PERIPHERAL_RXSTATUS_RXBUSYBRK_Msk (0x01UL << UART_PERIPHERAL_RXSTATUS_RXBUSYBRK_Pos)     /*!< UART_PERIPHERAL RXSTATUS: RXBUSYBRK Mask */
#define UART_PERIPHERAL_RXSTATUS_RXADDR9_Pos  9                                                       /*!< UART_PERIPHERAL RXSTATUS: RXADDR9 Position */
#define UART_PERIPHERAL_RXSTATUS_RXADDR9_Msk  (0x01UL << UART_PERIPHERAL_RXSTATUS_RXADDR9_Pos)        /*!< UART_PERIPHERAL RXSTATUS: RXADDR9 Mask  */
#define UART_PERIPHERAL_RXSTATUS_RXRTSN_Pos   15                                                      /*!< UART_PERIPHERAL RXSTATUS: RXRTSN Position */
#define UART_PERIPHERAL_RXSTATUS_RXRTSN_Msk   (0x01UL << UART_PERIPHERAL_RXSTATUS_RXRTSN_Pos)         /*!< UART_PERIPHERAL RXSTATUS: RXRTSN Mask   */

/* --------------------------  UART_PERIPHERAL_TXSTATUS  -------------------------- */
#define UART_PERIPHERAL_TXSTATUS_WRRDY_Pos    0                                                       /*!< UART_PERIPHERAL TXSTATUS: WRRDY Position */
#define UART_PERIPHERAL_TXSTATUS_WRRDY_Msk    (0x01UL << UART_PERIPHERAL_TXSTATUS_WRRDY_Pos)          /*!< UART_PERIPHERAL TXSTATUS: WRRDY Mask    */
#define UART_PERIPHERAL_TXSTATUS_WRBUSY_Pos   1                                                       /*!< UART_PERIPHERAL TXSTATUS: WRBUSY Position */
#define UART_PERIPHERAL_TXSTATUS_WRBUSY_Msk   (0x01UL << UART_PERIPHERAL_TXSTATUS_WRBUSY_Pos)         /*!< UART_PERIPHERAL TXSTATUS: WRBUSY Mask   */
#define UART_PERIPHERAL_TXSTATUS_TXBUSY_Pos   2                                                       /*!< UART_PERIPHERAL TXSTATUS: TXBUSY Position */
#define UART_PERIPHERAL_TXSTATUS_TXBUSY_Msk   (0x01UL << UART_PERIPHERAL_TXSTATUS_TXBUSY_Pos)         /*!< UART_PERIPHERAL TXSTATUS: TXBUSY Mask   */
#define UART_PERIPHERAL_TXSTATUS_WRLOST_Pos   3                                                       /*!< UART_PERIPHERAL TXSTATUS: WRLOST Position */
#define UART_PERIPHERAL_TXSTATUS_WRLOST_Msk   (0x01UL << UART_PERIPHERAL_TXSTATUS_WRLOST_Pos)         /*!< UART_PERIPHERAL TXSTATUS: WRLOST Mask   */
#define UART_PERIPHERAL_TXSTATUS_TXCTSN_Pos   15                                                      /*!< UART_PERIPHERAL TXSTATUS: TXCTSN Position */
#define UART_PERIPHERAL_TXSTATUS_TXCTSN_Msk   (0x01UL << UART_PERIPHERAL_TXSTATUS_TXCTSN_Pos)         /*!< UART_PERIPHERAL TXSTATUS: TXCTSN Mask   */

/* --------------------------  UART_PERIPHERAL_FIFO_CLR  -------------------------- */
#define UART_PERIPHERAL_FIFO_CLR_RXSTS_Pos    0                                                       /*!< UART_PERIPHERAL FIFO_CLR: RXSTS Position */
#define UART_PERIPHERAL_FIFO_CLR_RXSTS_Msk    (0x01UL << UART_PERIPHERAL_FIFO_CLR_RXSTS_Pos)          /*!< UART_PERIPHERAL FIFO_CLR: RXSTS Mask    */
#define UART_PERIPHERAL_FIFO_CLR_TXSTS_Pos    1                                                       /*!< UART_PERIPHERAL FIFO_CLR: TXSTS Position */
#define UART_PERIPHERAL_FIFO_CLR_TXSTS_Msk    (0x01UL << UART_PERIPHERAL_FIFO_CLR_TXSTS_Pos)          /*!< UART_PERIPHERAL FIFO_CLR: TXSTS Mask    */
#define UART_PERIPHERAL_FIFO_CLR_RXFIFO_Pos   2                                                       /*!< UART_PERIPHERAL FIFO_CLR: RXFIFO Position */
#define UART_PERIPHERAL_FIFO_CLR_RXFIFO_Msk   (0x01UL << UART_PERIPHERAL_FIFO_CLR_RXFIFO_Pos)         /*!< UART_PERIPHERAL FIFO_CLR: RXFIFO Mask   */
#define UART_PERIPHERAL_FIFO_CLR_TXFIFO_Pos   3                                                       /*!< UART_PERIPHERAL FIFO_CLR: TXFIFO Position */
#define UART_PERIPHERAL_FIFO_CLR_TXFIFO_Msk   (0x01UL << UART_PERIPHERAL_FIFO_CLR_TXFIFO_Pos)         /*!< UART_PERIPHERAL FIFO_CLR: TXFIFO Mask   */

/* --------------------------  UART_PERIPHERAL_ADDR9  ------------------------------ */
#define UART_PERIPHERAL_ADDR9_DATA_Pos    		0                                                       /*!< UART_PERIPHERAL ADDR9: DATA Position */
#define UART_PERIPHERAL_ADDR9_DATA_Msk    		(0x01UL << UART_PERIPHERAL_ADDR9_DATA_Pos)          		/*!< UART_PERIPHERAL ADDR9: DATA Mask    */
#define UART_PERIPHERAL_ADDR9_ENB9BIT_Pos    	1                                                       /*!< UART_PERIPHERAL ADDR9:	ENB9BIT Position */
#define UART_PERIPHERAL_ADDR9_ENB9BIT_Msk     (0x01UL << UART_PERIPHERAL_ADDR9_ENB9BIT_Pos)           /*!< UART_PERIPHERAL ADDR9: ENB9BIT Mask    */

/* ---------------------------  UART_PERIPHERAL_IRQ_ENB  -------------------------- */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Pos    0                                                       /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_RX_Pos)          /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX Mask    */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Pos 1                                                   /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_STATUS Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_RX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Pos 2                                                       /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_TO Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_RX_TO_Pos)       /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_TO Mask */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_ADDR9_Pos	3                                                   /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_ADDR9 Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_RX_ADDR9_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_RX_ADDR9_Pos) /*!< UART_PERIPHERAL IRQ_ENB: IRQ_RX_ADDR9 Mask */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Pos    4                                                       /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_TX_Pos)          /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX Mask    */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Pos 5                                                   /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_STATUS Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_TX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Pos 6                                                    /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_EMPTY Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_TX_EMPTY_Pos) /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_EMPTY Mask */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Pos 7                                                      /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_CTS Position */
#define UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Msk (0x01UL << UART_PERIPHERAL_IRQ_ENB_IRQ_TX_CTS_Pos)     /*!< UART_PERIPHERAL IRQ_ENB: IRQ_TX_CTS Mask */

/* ---------------------------  UART_PERIPHERAL_IRQ_RAW  -------------------------- */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_Pos    0                                                       /*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_RX_Pos)          /*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX Mask    */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_STATUS_Pos 1                                                   /*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX_STATUS Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_RX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_TO_Pos 2                                                       /*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX_TO Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_RX_TO_Msk (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_RX_TO_Pos)       /*!< UART_PERIPHERAL IRQ_RAW: IRQ_RX_TO Mask */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_Pos    4                                                       /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_TX_Pos)          /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX Mask    */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_STATUS_Pos 5                                                   /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_STATUS Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_TX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_EMPTY_Pos 6                                                    /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_EMPTY Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_EMPTY_Msk (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_TX_EMPTY_Pos) /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_EMPTY Mask */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_CTS_Pos 7                                                      /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_CTS Position */
#define UART_PERIPHERAL_IRQ_RAW_IRQ_TX_CTS_Msk (0x01UL << UART_PERIPHERAL_IRQ_RAW_IRQ_TX_CTS_Pos)     /*!< UART_PERIPHERAL IRQ_RAW: IRQ_TX_CTS Mask */

/* ---------------------------  UART_PERIPHERAL_IRQ_END  -------------------------- */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_Pos    0                                                       /*!< UART_PERIPHERAL IRQ_END: IRQ_RX Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_RX_Pos)          /*!< UART_PERIPHERAL IRQ_END: IRQ_RX Mask    */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_STATUS_Pos 1                                                   /*!< UART_PERIPHERAL IRQ_END: IRQ_RX_STATUS Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_RX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_END: IRQ_RX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_TO_Pos 2                                                       /*!< UART_PERIPHERAL IRQ_END: IRQ_RX_TO Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_RX_TO_Msk (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_RX_TO_Pos)       /*!< UART_PERIPHERAL IRQ_END: IRQ_RX_TO Mask */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_Pos    4                                                       /*!< UART_PERIPHERAL IRQ_END: IRQ_TX Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_TX_Pos)          /*!< UART_PERIPHERAL IRQ_END: IRQ_TX Mask    */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_STATUS_Pos 5                                                   /*!< UART_PERIPHERAL IRQ_END: IRQ_TX_STATUS Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_TX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_END: IRQ_TX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_EMPTY_Pos 6                                                    /*!< UART_PERIPHERAL IRQ_END: IRQ_TX_EMPTY Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_EMPTY_Msk (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_TX_EMPTY_Pos) /*!< UART_PERIPHERAL IRQ_END: IRQ_TX_EMPTY Mask */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_CTS_Pos 7                                                      /*!< UART_PERIPHERAL IRQ_END: IRQ_TX_CTS Position */
#define UART_PERIPHERAL_IRQ_END_IRQ_TX_CTS_Msk (0x01UL << UART_PERIPHERAL_IRQ_END_IRQ_TX_CTS_Pos)     /*!< UART_PERIPHERAL IRQ_END: IRQ_TX_CTS Mask */

/* ---------------------------  UART_PERIPHERAL_IRQ_CLR  -------------------------- */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_Pos    0                                                       /*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_RX_Pos)          /*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX Mask    */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_STATUS_Pos 1                                                   /*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX_STATUS Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_RX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_TO_Pos 2                                                       /*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX_TO Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_RX_TO_Msk (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_RX_TO_Pos)       /*!< UART_PERIPHERAL IRQ_CLR: IRQ_RX_TO Mask */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_Pos    4                                                       /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_Msk    (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_TX_Pos)          /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX Mask    */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_STATUS_Pos 5                                                   /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_STATUS Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_STATUS_Msk (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_TX_STATUS_Pos)/*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_STATUS Mask */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_EMPTY_Pos 6                                                    /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_EMPTY Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_EMPTY_Msk (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_TX_EMPTY_Pos) /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_EMPTY Mask */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_CTS_Pos 7                                                      /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_CTS Position */
#define UART_PERIPHERAL_IRQ_CLR_IRQ_TX_CTS_Msk (0x01UL << UART_PERIPHERAL_IRQ_CLR_IRQ_TX_CTS_Pos)     /*!< UART_PERIPHERAL IRQ_CLR: IRQ_TX_CTS Mask */

/* ---------------------------  UART_PERIPHERAL_STATE  -------------------------- */
#define UART_PERIPHERAL_STATE_RX_STATE_Pos    0                                                       /*!< UART_PERIPHERAL STATE: RX_STATE Position */
#define UART_PERIPHERAL_STATE_RX_STATE_Msk    (0xffUL << UART_PERIPHERAL_STATE_RX_STATE_Pos)          /*!< UART_PERIPHERAL STATE: RX_STATE Mask    */
#define UART_PERIPHERAL_STATE_RX_FIFO_Pos 		8                                                  	  /*!< UART_PERIPHERAL STATE: RX_FIFO Position */
#define UART_PERIPHERAL_STATE_RX_FIFO_Msk 	(0xffUL << UART_PERIPHERAL_STATE_RX_FIFO_Pos)			  /*!< UART_PERIPHERAL STATE: RX_FIFO Mask */
#define UART_PERIPHERAL_STATE_TX_STATE_Pos 		16                                                    /*!< UART_PERIPHERAL STATE: TX_STATE Position */
#define UART_PERIPHERAL_STATE_TX_STATE_Msk 		(0xffUL << UART_PERIPHERAL_STATE_TX_STATE_Pos)        /*!< UART_PERIPHERAL STATE: TX_STATE Mask */
#define UART_PERIPHERAL_STATE_TX_FIFO_Pos     24                                                      /*!< UART_PERIPHERAL STATE: TX_FIFO Position */
#define UART_PERIPHERAL_STATE_TX_FIFO_Msk     (0xffUL << UART_PERIPHERAL_STATE_TX_FIFO_Pos)           /*!< UART_PERIPHERAL STATE: TX_FIFO Mask    */

/* ================================================================================ */
/* ================         struct 'UARTA' Position & Mask         ================ */
/* ================================================================================ */

/* --------------------------------  UARTA_DATA  ----------------------------------- */
#define UARTA_DATA_VALUE_Pos   								0                                                       /*!< UARTA DATA: VALUE Position 						*/
#define UARTA_DATA_VALUE_Msk   								(0x000000ffUL << UARTA_DATA_VALUE_Pos)   								/*!< UARTA DATA: VALUE Mask   							*/
#define UARTA_DATA_DPARITY_Pos   							15                                                      /*!< UARTA DATA: DPARITY Position 					*/
#define UARTA_DATA_DPARITY_Msk   							(0x01UL << UARTA_DATA_DPARITY_Pos)         							/*!< UARTA DATA: DPARITY Mask   						*/

/* --------------------------------  UARTA_ENABLE  -------------------------------- */
#define UARTA_ENABLE_RXENABLE_Pos             0                                                       /*!< UARTA ENABLE: RXENABLE Position         */
#define UARTA_ENABLE_RXENABLE_Msk             (0x01UL << UARTA_ENABLE_RXENABLE_Pos)                   /*!< UARTA ENABLE: RXENABLE Mask             */
#define UARTA_ENABLE_TXENABLE_Pos             1                                                       /*!< UARTA ENABLE: TXENABLE Position         */
#define UARTA_ENABLE_TXENABLE_Msk             (0x01UL << UARTA_ENABLE_TXENABLE_Pos)                   /*!< UARTA ENABLE: TXENABLE Mask             */

/* ---------------------------------  UARTA_CTRL  --------------------------------- */
#define UARTA_CTRL_PAREN_Pos                  0                                                       /*!< UARTA CTRL: PAREN Position              */
#define UARTA_CTRL_PAREN_Msk                  (0x01UL << UARTA_CTRL_PAREN_Pos)                        /*!< UARTA CTRL: PAREN Mask                  */
#define UARTA_CTRL_PAREVEN_Pos                1                                                       /*!< UARTA CTRL: PAREVEN Position            */
#define UARTA_CTRL_PAREVEN_Msk                (0x01UL << UARTA_CTRL_PAREVEN_Pos)                      /*!< UARTA CTRL: PAREVEN Mask                */
#define UARTA_CTRL_PARSTK_Pos                 2                                                       /*!< UARTA CTRL: PARSTK Position             */
#define UARTA_CTRL_PARSTK_Msk                 (0x01UL << UARTA_CTRL_PARSTK_Pos)                       /*!< UARTA CTRL: PARSTK Mask                 */
#define UARTA_CTRL_STOPBITS_Pos               3                                                       /*!< UARTA CTRL: STOPBITS Position           */
#define UARTA_CTRL_STOPBITS_Msk               (0x01UL << UARTA_CTRL_STOPBITS_Pos)                     /*!< UARTA CTRL: STOPBITS Mask               */
#define UARTA_CTRL_WORDSIZE_Pos               4                                                       /*!< UARTA CTRL: WORDSIZE Position           */
#define UARTA_CTRL_WORDSIZE_Msk               (0x03UL << UARTA_CTRL_WORDSIZE_Pos)                     /*!< UARTA CTRL: WORDSIZE Mask               */
#define UARTA_CTRL_LOOPBACK_Pos               6                                                       /*!< UARTA CTRL: LOOPBACK Position           */
#define UARTA_CTRL_LOOPBACK_Msk               (0x01UL << UARTA_CTRL_LOOPBACK_Pos)                     /*!< UARTA CTRL: LOOPBACK Mask               */
#define UARTA_CTRL_LOOPBACKBLK_Pos            7                                                       /*!< UARTA CTRL: LOOPBACKBLK Position        */
#define UARTA_CTRL_LOOPBACKBLK_Msk            (0x01UL << UARTA_CTRL_LOOPBACKBLK_Pos)                  /*!< UARTA CTRL: LOOPBACKBLK Mask            */
#define UARTA_CTRL_AUTOCTS_Pos                8                                                       /*!< UARTA CTRL: AUTOCTS Position            */
#define UARTA_CTRL_AUTOCTS_Msk                (0x01UL << UARTA_CTRL_AUTOCTS_Pos)                      /*!< UARTA CTRL: AUTOCTS Mask                */
#define UARTA_CTRL_DEFRTS_Pos                 9                                                       /*!< UARTA CTRL: DEFRTS Position             */
#define UARTA_CTRL_DEFRTS_Msk                 (0x01UL << UARTA_CTRL_DEFRTS_Pos)                       /*!< UARTA CTRL: DEFRTS Mask                 */
#define UARTA_CTRL_AUTORTS_Pos                10                                                      /*!< UARTA CTRL: AUTORTS Position            */
#define UARTA_CTRL_AUTORTS_Msk                (0x01UL << UARTA_CTRL_AUTORTS_Pos)                      /*!< UARTA CTRL: AUTORTS Mask                */
#define UARTA_CTRL_BAUD8_Pos                  11                                                      /*!< UARTA CTRL: BAUD8 Position              */
#define UARTA_CTRL_BAUD8_Msk                  (0x01UL << UARTA_CTRL_BAUD8_Pos)                        /*!< UARTA CTRL: BAUD8 Mask                  */

/* -------------------------------  UARTA_CLKSCALE  ------------------------------- */
#define UARTA_CLKSCALE_FRAC_Pos               0                                                       /*!< UARTA CLKSCALE: FRAC Position           */
#define UARTA_CLKSCALE_FRAC_Msk               (0x3fUL << UARTA_CLKSCALE_FRAC_Pos)                     /*!< UARTA CLKSCALE: FRAC Mask               */
#define UARTA_CLKSCALE_INT_Pos                6                                                       /*!< UARTA CLKSCALE: INT Position            */
#define UARTA_CLKSCALE_INT_Msk                (0x0003ffffUL << UARTA_CLKSCALE_INT_Pos)                /*!< UARTA CLKSCALE: INT Mask                */
#define UARTA_CLKSCALE_RESET_Pos              31                                                      /*!< UARTA CLKSCALE: RESET Position          */
#define UARTA_CLKSCALE_RESET_Msk              (0x01UL << UARTA_CLKSCALE_RESET_Pos)                    /*!< UARTA CLKSCALE: RESET Mask              */

/* -------------------------------  UARTA_RXSTATUS  ------------------------------- */
#define UARTA_RXSTATUS_RDAVL_Pos              0                                                       /*!< UARTA RXSTATUS: RDAVL Position          */
#define UARTA_RXSTATUS_RDAVL_Msk              (0x01UL << UARTA_RXSTATUS_RDAVL_Pos)                    /*!< UARTA RXSTATUS: RDAVL Mask              */
#define UARTA_RXSTATUS_RDNFULL_Pos            1                                                       /*!< UARTA RXSTATUS: RDNFULL Position        */
#define UARTA_RXSTATUS_RDNFULL_Msk            (0x01UL << UARTA_RXSTATUS_RDNFULL_Pos)                  /*!< UARTA RXSTATUS: RDNFULL Mask            */
#define UARTA_RXSTATUS_RXBUSY_Pos             2                                                       /*!< UARTA RXSTATUS: RXBUSY Position         */
#define UARTA_RXSTATUS_RXBUSY_Msk             (0x01UL << UARTA_RXSTATUS_RXBUSY_Pos)                   /*!< UARTA RXSTATUS: RXBUSY Mask             */
#define UARTA_RXSTATUS_RXTO_Pos               3                                                       /*!< UARTA RXSTATUS: RXTO Position           */
#define UARTA_RXSTATUS_RXTO_Msk               (0x01UL << UARTA_RXSTATUS_RXTO_Pos)                     /*!< UARTA RXSTATUS: RXTO Mask               */
#define UARTA_RXSTATUS_RXOVR_Pos              4                                                       /*!< UARTA RXSTATUS: RXOVR Position          */
#define UARTA_RXSTATUS_RXOVR_Msk              (0x01UL << UARTA_RXSTATUS_RXOVR_Pos)                    /*!< UARTA RXSTATUS: RXOVR Mask              */
#define UARTA_RXSTATUS_RXFRM_Pos              5                                                       /*!< UARTA RXSTATUS: RXFRM Position          */
#define UARTA_RXSTATUS_RXFRM_Msk              (0x01UL << UARTA_RXSTATUS_RXFRM_Pos)                    /*!< UARTA RXSTATUS: RXFRM Mask              */
#define UARTA_RXSTATUS_RXPAR_Pos              6                                                       /*!< UARTA RXSTATUS: RXPAR Position          */
#define UARTA_RXSTATUS_RXPAR_Msk              (0x01UL << UARTA_RXSTATUS_RXPAR_Pos)                    /*!< UARTA RXSTATUS: RXPAR Mask              */
#define UARTA_RXSTATUS_RXBRK_Pos              7                                                       /*!< UARTA RXSTATUS: RXBRK Position          */
#define UARTA_RXSTATUS_RXBRK_Msk              (0x01UL << UARTA_RXSTATUS_RXBRK_Pos)                    /*!< UARTA RXSTATUS: RXBRK Mask              */
#define UARTA_RXSTATUS_RXBUSYBRK_Pos          8                                                       /*!< UARTA RXSTATUS: RXBUSYBRK Position      */
#define UARTA_RXSTATUS_RXBUSYBRK_Msk          (0x01UL << UARTA_RXSTATUS_RXBUSYBRK_Pos)                /*!< UARTA RXSTATUS: RXBUSYBRK Mask          */
#define UARTA_RXSTATUS_RXADDR9_Pos            9                                                       /*!< UARTA RXSTATUS: RXADDR9 Position        */
#define UARTA_RXSTATUS_RXADDR9_Msk            (0x01UL << UARTA_RXSTATUS_RXADDR9_Pos)                  /*!< UARTA RXSTATUS: RXADDR9 Mask            */
#define UARTA_RXSTATUS_RXRTSN_Pos             15                                                      /*!< UARTA RXSTATUS: RXRTSN Position         */
#define UARTA_RXSTATUS_RXRTSN_Msk             (0x01UL << UARTA_RXSTATUS_RXRTSN_Pos)                   /*!< UARTA RXSTATUS: RXRTSN Mask             */

/* -------------------------------  UARTA_TXSTATUS  ------------------------------- */
#define UARTA_TXSTATUS_WRRDY_Pos              0                                                       /*!< UARTA TXSTATUS: WRRDY Position          */
#define UARTA_TXSTATUS_WRRDY_Msk              (0x01UL << UARTA_TXSTATUS_WRRDY_Pos)                    /*!< UARTA TXSTATUS: WRRDY Mask              */
#define UARTA_TXSTATUS_WRBUSY_Pos             1                                                       /*!< UARTA TXSTATUS: WRBUSY Position         */
#define UARTA_TXSTATUS_WRBUSY_Msk             (0x01UL << UARTA_TXSTATUS_WRBUSY_Pos)                   /*!< UARTA TXSTATUS: WRBUSY Mask             */
#define UARTA_TXSTATUS_TXBUSY_Pos             2                                                       /*!< UARTA TXSTATUS: TXBUSY Position         */
#define UARTA_TXSTATUS_TXBUSY_Msk             (0x01UL << UARTA_TXSTATUS_TXBUSY_Pos)                   /*!< UARTA TXSTATUS: TXBUSY Mask             */
#define UARTA_TXSTATUS_WRLOST_Pos             3                                                       /*!< UARTA TXSTATUS: WRLOST Position         */
#define UARTA_TXSTATUS_WRLOST_Msk             (0x01UL << UARTA_TXSTATUS_WRLOST_Pos)                   /*!< UARTA TXSTATUS: WRLOST Mask             */
#define UARTA_TXSTATUS_TXCTSN_Pos             15                                                      /*!< UARTA TXSTATUS: TXCTSN Position         */
#define UARTA_TXSTATUS_TXCTSN_Msk             (0x01UL << UARTA_TXSTATUS_TXCTSN_Pos)                   /*!< UARTA TXSTATUS: TXCTSN Mask             */

/* -------------------------------  UARTA_FIFO_CLR  ------------------------------- */
#define UARTA_FIFO_CLR_RXSTS_Pos              0                                                       /*!< UARTA FIFO_CLR: RXSTS Position          */
#define UARTA_FIFO_CLR_RXSTS_Msk              (0x01UL << UARTA_FIFO_CLR_RXSTS_Pos)                    /*!< UARTA FIFO_CLR: RXSTS Mask              */
#define UARTA_FIFO_CLR_TXSTS_Pos              1                                                       /*!< UARTA FIFO_CLR: TXSTS Position          */
#define UARTA_FIFO_CLR_TXSTS_Msk              (0x01UL << UARTA_FIFO_CLR_TXSTS_Pos)                    /*!< UARTA FIFO_CLR: TXSTS Mask              */
#define UARTA_FIFO_CLR_RXFIFO_Pos             2                                                       /*!< UARTA FIFO_CLR: RXFIFO Position         */
#define UARTA_FIFO_CLR_RXFIFO_Msk             (0x01UL << UARTA_FIFO_CLR_RXFIFO_Pos)                   /*!< UARTA FIFO_CLR: RXFIFO Mask             */
#define UARTA_FIFO_CLR_TXFIFO_Pos             3                                                       /*!< UARTA FIFO_CLR: TXFIFO Position         */
#define UARTA_FIFO_CLR_TXFIFO_Msk             (0x01UL << UARTA_FIFO_CLR_TXFIFO_Pos)                   /*!< UARTA FIFO_CLR: TXFIFO Mask             */

/* ------------------------------  UARTA_ADDR9  ----------------------------------- */
#define UARTA_ADDR9_DATA_Pos    							0                                                       /*!< UARTA ADDR9: DATA Position */
#define UARTA_ADDR9_DATA_Msk    							(0x01UL << UARTA_ADDR9_DATA_Pos)          							/*!< UARTA ADDR9: DATA Mask    */
#define UARTA_ADDR9_ENB9BIT_Pos    						1                                                       /*!< UARTA ADDR9:	ENB9BIT Position */
#define UARTA_ADDR9_ENB9BIT_Msk     					(0x01UL << UARTA_ADDR9_ENB9BIT_Pos)           					/*!< UARTA ADDR9: ENB9BIT Mask    */

/* --------------------------------  UARTA_IRQ_ENB  ------------------------------- */
#define UARTA_IRQ_ENB_IRQ_RX_Pos              0                                                       /*!< UARTA IRQ_ENB: IRQ_RX Position          */
#define UARTA_IRQ_ENB_IRQ_RX_Msk              (0x01UL << UARTA_IRQ_ENB_IRQ_RX_Pos)                    /*!< UARTA IRQ_ENB: IRQ_RX Mask              */
#define UARTA_IRQ_ENB_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTA IRQ_ENB: IRQ_RX_STATUS Position   */
#define UARTA_IRQ_ENB_IRQ_RX_STATUS_Msk       (0x01UL << UARTA_IRQ_ENB_IRQ_RX_STATUS_Pos)             /*!< UARTA IRQ_ENB: IRQ_RX_STATUS Mask       */
#define UARTA_IRQ_ENB_IRQ_RX_TO_Pos           2                                                       /*!< UARTA IRQ_ENB: IRQ_RX_TO Position       */
#define UARTA_IRQ_ENB_IRQ_RX_TO_Msk           (0x01UL << UARTA_IRQ_ENB_IRQ_RX_TO_Pos)                 /*!< UARTA IRQ_ENB: IRQ_RX_TO Mask           */
#define UARTA_IRQ_ENB_IRQ_RX_ADDR9_Pos				3                                                   		/*!< UARTA IRQ_ENB: IRQ_RX_ADDR9 Position */
#define UARTA_IRQ_ENB_IRQ_RX_ADDR9_Msk 				(0x01UL << UARTA_IRQ_ENB_IRQ_RX_ADDR9_Pos) 							/*!< UARTA IRQ_ENB: IRQ_RX_ADDR9 Mask */
#define UARTA_IRQ_ENB_IRQ_TX_Pos              4                                                       /*!< UARTA IRQ_ENB: IRQ_TX Position          */
#define UARTA_IRQ_ENB_IRQ_TX_Msk              (0x01UL << UARTA_IRQ_ENB_IRQ_TX_Pos)                    /*!< UARTA IRQ_ENB: IRQ_TX Mask              */
#define UARTA_IRQ_ENB_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTA IRQ_ENB: IRQ_TX_STATUS Position   */
#define UARTA_IRQ_ENB_IRQ_TX_STATUS_Msk       (0x01UL << UARTA_IRQ_ENB_IRQ_TX_STATUS_Pos)             /*!< UARTA IRQ_ENB: IRQ_TX_STATUS Mask       */
#define UARTA_IRQ_ENB_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTA IRQ_ENB: IRQ_TX_EMPTY Position    */
#define UARTA_IRQ_ENB_IRQ_TX_EMPTY_Msk        (0x01UL << UARTA_IRQ_ENB_IRQ_TX_EMPTY_Pos)              /*!< UARTA IRQ_ENB: IRQ_TX_EMPTY Mask        */
#define UARTA_IRQ_ENB_IRQ_TX_CTS_Pos          7                                                       /*!< UARTA IRQ_ENB: IRQ_TX_CTS Position      */
#define UARTA_IRQ_ENB_IRQ_TX_CTS_Msk          (0x01UL << UARTA_IRQ_ENB_IRQ_TX_CTS_Pos)                /*!< UARTA IRQ_ENB: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTA_IRQ_RAW  ------------------------------- */
#define UARTA_IRQ_RAW_IRQ_RX_Pos              0                                                       /*!< UARTA IRQ_RAW: IRQ_RX Position          */
#define UARTA_IRQ_RAW_IRQ_RX_Msk              (0x01UL << UARTA_IRQ_RAW_IRQ_RX_Pos)                    /*!< UARTA IRQ_RAW: IRQ_RX Mask              */
#define UARTA_IRQ_RAW_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTA IRQ_RAW: IRQ_RX_STATUS Position   */
#define UARTA_IRQ_RAW_IRQ_RX_STATUS_Msk       (0x01UL << UARTA_IRQ_RAW_IRQ_RX_STATUS_Pos)             /*!< UARTA IRQ_RAW: IRQ_RX_STATUS Mask       */
#define UARTA_IRQ_RAW_IRQ_RX_TO_Pos           2                                                       /*!< UARTA IRQ_RAW: IRQ_RX_TO Position       */
#define UARTA_IRQ_RAW_IRQ_RX_TO_Msk           (0x01UL << UARTA_IRQ_RAW_IRQ_RX_TO_Pos)                 /*!< UARTA IRQ_RAW: IRQ_RX_TO Mask           */
#define UARTA_IRQ_RAW_IRQ_TX_Pos              4                                                       /*!< UARTA IRQ_RAW: IRQ_TX Position          */
#define UARTA_IRQ_RAW_IRQ_TX_Msk              (0x01UL << UARTA_IRQ_RAW_IRQ_TX_Pos)                    /*!< UARTA IRQ_RAW: IRQ_TX Mask              */
#define UARTA_IRQ_RAW_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTA IRQ_RAW: IRQ_TX_STATUS Position   */
#define UARTA_IRQ_RAW_IRQ_TX_STATUS_Msk       (0x01UL << UARTA_IRQ_RAW_IRQ_TX_STATUS_Pos)             /*!< UARTA IRQ_RAW: IRQ_TX_STATUS Mask       */
#define UARTA_IRQ_RAW_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTA IRQ_RAW: IRQ_TX_EMPTY Position    */
#define UARTA_IRQ_RAW_IRQ_TX_EMPTY_Msk        (0x01UL << UARTA_IRQ_RAW_IRQ_TX_EMPTY_Pos)              /*!< UARTA IRQ_RAW: IRQ_TX_EMPTY Mask        */
#define UARTA_IRQ_RAW_IRQ_TX_CTS_Pos          7                                                       /*!< UARTA IRQ_RAW: IRQ_TX_CTS Position      */
#define UARTA_IRQ_RAW_IRQ_TX_CTS_Msk          (0x01UL << UARTA_IRQ_RAW_IRQ_TX_CTS_Pos)                /*!< UARTA IRQ_RAW: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTA_IRQ_END  ------------------------------- */
#define UARTA_IRQ_END_IRQ_RX_Pos              0                                                       /*!< UARTA IRQ_END: IRQ_RX Position          */
#define UARTA_IRQ_END_IRQ_RX_Msk              (0x01UL << UARTA_IRQ_END_IRQ_RX_Pos)                    /*!< UARTA IRQ_END: IRQ_RX Mask              */
#define UARTA_IRQ_END_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTA IRQ_END: IRQ_RX_STATUS Position   */
#define UARTA_IRQ_END_IRQ_RX_STATUS_Msk       (0x01UL << UARTA_IRQ_END_IRQ_RX_STATUS_Pos)             /*!< UARTA IRQ_END: IRQ_RX_STATUS Mask       */
#define UARTA_IRQ_END_IRQ_RX_TO_Pos           2                                                       /*!< UARTA IRQ_END: IRQ_RX_TO Position       */
#define UARTA_IRQ_END_IRQ_RX_TO_Msk           (0x01UL << UARTA_IRQ_END_IRQ_RX_TO_Pos)                 /*!< UARTA IRQ_END: IRQ_RX_TO Mask           */
#define UARTA_IRQ_END_IRQ_TX_Pos              4                                                       /*!< UARTA IRQ_END: IRQ_TX Position          */
#define UARTA_IRQ_END_IRQ_TX_Msk              (0x01UL << UARTA_IRQ_END_IRQ_TX_Pos)                    /*!< UARTA IRQ_END: IRQ_TX Mask              */
#define UARTA_IRQ_END_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTA IRQ_END: IRQ_TX_STATUS Position   */
#define UARTA_IRQ_END_IRQ_TX_STATUS_Msk       (0x01UL << UARTA_IRQ_END_IRQ_TX_STATUS_Pos)             /*!< UARTA IRQ_END: IRQ_TX_STATUS Mask       */
#define UARTA_IRQ_END_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTA IRQ_END: IRQ_TX_EMPTY Position    */
#define UARTA_IRQ_END_IRQ_TX_EMPTY_Msk        (0x01UL << UARTA_IRQ_END_IRQ_TX_EMPTY_Pos)              /*!< UARTA IRQ_END: IRQ_TX_EMPTY Mask        */
#define UARTA_IRQ_END_IRQ_TX_CTS_Pos          7                                                       /*!< UARTA IRQ_END: IRQ_TX_CTS Position      */
#define UARTA_IRQ_END_IRQ_TX_CTS_Msk          (0x01UL << UARTA_IRQ_END_IRQ_TX_CTS_Pos)                /*!< UARTA IRQ_END: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTA_IRQ_CLR  ------------------------------- */
#define UARTA_IRQ_CLR_IRQ_RX_Pos              0                                                       /*!< UARTA IRQ_CLR: IRQ_RX Position          */
#define UARTA_IRQ_CLR_IRQ_RX_Msk              (0x01UL << UARTA_IRQ_CLR_IRQ_RX_Pos)                    /*!< UARTA IRQ_CLR: IRQ_RX Mask              */
#define UARTA_IRQ_CLR_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTA IRQ_CLR: IRQ_RX_STATUS Position   */
#define UARTA_IRQ_CLR_IRQ_RX_STATUS_Msk       (0x01UL << UARTA_IRQ_CLR_IRQ_RX_STATUS_Pos)             /*!< UARTA IRQ_CLR: IRQ_RX_STATUS Mask       */
#define UARTA_IRQ_CLR_IRQ_RX_TO_Pos           2                                                       /*!< UARTA IRQ_CLR: IRQ_RX_TO Position       */
#define UARTA_IRQ_CLR_IRQ_RX_TO_Msk           (0x01UL << UARTA_IRQ_CLR_IRQ_RX_TO_Pos)                 /*!< UARTA IRQ_CLR: IRQ_RX_TO Mask           */
#define UARTA_IRQ_CLR_IRQ_TX_Pos              4                                                       /*!< UARTA IRQ_CLR: IRQ_TX Position          */
#define UARTA_IRQ_CLR_IRQ_TX_Msk              (0x01UL << UARTA_IRQ_CLR_IRQ_TX_Pos)                    /*!< UARTA IRQ_CLR: IRQ_TX Mask              */
#define UARTA_IRQ_CLR_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTA IRQ_CLR: IRQ_TX_STATUS Position   */
#define UARTA_IRQ_CLR_IRQ_TX_STATUS_Msk       (0x01UL << UARTA_IRQ_CLR_IRQ_TX_STATUS_Pos)             /*!< UARTA IRQ_CLR: IRQ_TX_STATUS Mask       */
#define UARTA_IRQ_CLR_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTA IRQ_CLR: IRQ_TX_EMPTY Position    */
#define UARTA_IRQ_CLR_IRQ_TX_EMPTY_Msk        (0x01UL << UARTA_IRQ_CLR_IRQ_TX_EMPTY_Pos)              /*!< UARTA IRQ_CLR: IRQ_TX_EMPTY Mask        */
#define UARTA_IRQ_CLR_IRQ_TX_CTS_Pos          7                                                       /*!< UARTA IRQ_CLR: IRQ_TX_CTS Position      */
#define UARTA_IRQ_CLR_IRQ_TX_CTS_Msk          (0x01UL << UARTA_IRQ_CLR_IRQ_TX_CTS_Pos)                /*!< UARTA IRQ_CLR: IRQ_TX_CTS Mask          */

/* ------------------------------   UARTA_STATE  ----------------------------------- */
#define UARTA_STATE_RX_STATE_Pos    					0                                              					/*!< UARTA STATE: RX_STATE Position 				 */
#define UARTA_STATE_RX_STATE_Msk    					(0xffUL << UARTA_STATE_RX_STATE_Pos)      						/*!< UARTA STATE: RX_STATE Mask    					 */
#define UARTA_STATE_RX_FIFO_Pos 						8	                                        					/*!< UARTA STATE: RX_FIFO Position 					 */
#define UARTA_STATE_RX_FIFO_Msk 						(0xffUL << UARTA_STATE_RX_FIFO_Pos)								/*!< UARTA STATE: RX_FIFO Mask 							 */
#define UARTA_STATE_TX_STATE_Pos 						16                                             					/*!< UARTA STATE: TX_STATE Position 				 */
#define UARTA_STATE_TX_STATE_Msk 						(0xffUL << UARTA_STATE_TX_STATE_Pos)  							/*!< UARTA STATE: TX_STATE Mask 						 */
#define UARTA_STATE_TX_FIFO_Pos     					24                                             					/*!< UARTA STATE: TX_FIFO Position 					 */
#define UARTA_STATE_TX_FIFO_Msk     					(0xffUL << UARTA_STATE_TX_FIFO_Pos) 							/*!< UARTA STATE: TX_FIFO Mask    					 */

/* ================================================================================ */
/* ================         struct 'UARTB' Position & Mask         ================ */
/* ================================================================================ */

/* --------------------------------  UARTB_DATA  ----------------------------------- */
#define UARTB_DATA_VALUE_Pos   								0                                                       /*!< UARTB DATA: VALUE Position 						 */
#define UARTB_DATA_VALUE_Msk   								(0x000000ffUL << UARTB_DATA_VALUE_Pos)   								/*!< UARTB DATA: VALUE Mask   							 */
#define UARTB_DATA_DPARITY_Pos   							15                                                      /*!< UARTB DATA: DPARITY Position 					 */
#define UARTB_DATA_DPARITY_Msk   							(0x01UL << UARTB_DATA_DPARITY_Pos)         							/*!< UARTB DATA: DPARITY Mask   						 */

/* --------------------------------  UARTB_ENABLE  -------------------------------- */
#define UARTB_ENABLE_RXENABLE_Pos             0                                                       /*!< UARTB ENABLE: RXENABLE Position         */
#define UARTB_ENABLE_RXENABLE_Msk             (0x01UL << UARTB_ENABLE_RXENABLE_Pos)                   /*!< UARTB ENABLE: RXENABLE Mask             */
#define UARTB_ENABLE_TXENABLE_Pos             1                                                       /*!< UARTB ENABLE: TXENABLE Position         */
#define UARTB_ENABLE_TXENABLE_Msk             (0x01UL << UARTB_ENABLE_TXENABLE_Pos)                   /*!< UARTB ENABLE: TXENABLE Mask             */

/* ---------------------------------  UARTB_CTRL  --------------------------------- */
#define UARTB_CTRL_PAREN_Pos                  0                                                       /*!< UARTB CTRL: PAREN Position              */
#define UARTB_CTRL_PAREN_Msk                  (0x01UL << UARTB_CTRL_PAREN_Pos)                        /*!< UARTB CTRL: PAREN Mask                  */
#define UARTB_CTRL_PAREVEN_Pos                1                                                       /*!< UARTB CTRL: PAREVEN Position            */
#define UARTB_CTRL_PAREVEN_Msk                (0x01UL << UARTB_CTRL_PAREVEN_Pos)                      /*!< UARTB CTRL: PAREVEN Mask                */
#define UARTB_CTRL_PARSTK_Pos                 2                                                       /*!< UARTB CTRL: PARSTK Position             */
#define UARTB_CTRL_PARSTK_Msk                 (0x01UL << UARTB_CTRL_PARSTK_Pos)                       /*!< UARTB CTRL: PARSTK Mask                 */
#define UARTB_CTRL_STOPBITS_Pos               3                                                       /*!< UARTB CTRL: STOPBITS Position           */
#define UARTB_CTRL_STOPBITS_Msk               (0x01UL << UARTB_CTRL_STOPBITS_Pos)                     /*!< UARTB CTRL: STOPBITS Mask               */
#define UARTB_CTRL_WORDSIZE_Pos               4                                                       /*!< UARTB CTRL: WORDSIZE Position           */
#define UARTB_CTRL_WORDSIZE_Msk               (0x03UL << UARTB_CTRL_WORDSIZE_Pos)                     /*!< UARTB CTRL: WORDSIZE Mask               */
#define UARTB_CTRL_LOOPBACK_Pos               6                                                       /*!< UARTB CTRL: LOOPBACK Position           */
#define UARTB_CTRL_LOOPBACK_Msk               (0x01UL << UARTB_CTRL_LOOPBACK_Pos)                     /*!< UARTB CTRL: LOOPBACK Mask               */
#define UARTB_CTRL_LOOPBACKBLK_Pos            7                                                       /*!< UARTB CTRL: LOOPBACKBLK Position        */
#define UARTB_CTRL_LOOPBACKBLK_Msk            (0x01UL << UARTB_CTRL_LOOPBACKBLK_Pos)                  /*!< UARTB CTRL: LOOPBACKBLK Mask            */
#define UARTB_CTRL_AUTOCTS_Pos                8                                                       /*!< UARTB CTRL: AUTOCTS Position            */
#define UARTB_CTRL_AUTOCTS_Msk                (0x01UL << UARTB_CTRL_AUTOCTS_Pos)                      /*!< UARTB CTRL: AUTOCTS Mask                */
#define UARTB_CTRL_DEFRTS_Pos                 9                                                       /*!< UARTB CTRL: DEFRTS Position             */
#define UARTB_CTRL_DEFRTS_Msk                 (0x01UL << UARTB_CTRL_DEFRTS_Pos)                       /*!< UARTB CTRL: DEFRTS Mask                 */
#define UARTB_CTRL_AUTORTS_Pos                10                                                      /*!< UARTB CTRL: AUTORTS Position            */
#define UARTB_CTRL_AUTORTS_Msk                (0x01UL << UARTB_CTRL_AUTORTS_Pos)                      /*!< UARTB CTRL: AUTORTS Mask                */
#define UARTB_CTRL_BAUD8_Pos                  11                                                      /*!< UARTB CTRL: BAUD8 Position              */
#define UARTB_CTRL_BAUD8_Msk                  (0x01UL << UARTB_CTRL_BAUD8_Pos)                        /*!< UARTB CTRL: BAUD8 Mask                  */

/* -------------------------------  UARTB_CLKSCALE  ------------------------------- */
#define UARTB_CLKSCALE_FRAC_Pos               0                                                       /*!< UARTB CLKSCALE: FRAC Position           */
#define UARTB_CLKSCALE_FRAC_Msk               (0x3fUL << UARTB_CLKSCALE_FRAC_Pos)                     /*!< UARTB CLKSCALE: FRAC Mask               */
#define UARTB_CLKSCALE_INT_Pos                6                                                       /*!< UARTB CLKSCALE: INT Position            */
#define UARTB_CLKSCALE_INT_Msk                (0x0003ffffUL << UARTB_CLKSCALE_INT_Pos)                /*!< UARTB CLKSCALE: INT Mask                */
#define UARTB_CLKSCALE_RESET_Pos              31                                                      /*!< UARTB CLKSCALE: RESET Position          */
#define UARTB_CLKSCALE_RESET_Msk              (0x01UL << UARTB_CLKSCALE_RESET_Pos)                    /*!< UARTB CLKSCALE: RESET Mask              */

/* -------------------------------  UARTB_RXSTATUS  ------------------------------- */
#define UARTB_RXSTATUS_RDAVL_Pos              0                                                       /*!< UARTB RXSTATUS: RDAVL Position          */
#define UARTB_RXSTATUS_RDAVL_Msk              (0x01UL << UARTB_RXSTATUS_RDAVL_Pos)                    /*!< UARTB RXSTATUS: RDAVL Mask              */
#define UARTB_RXSTATUS_RDNFULL_Pos            1                                                       /*!< UARTB RXSTATUS: RDNFULL Position        */
#define UARTB_RXSTATUS_RDNFULL_Msk            (0x01UL << UARTB_RXSTATUS_RDNFULL_Pos)                  /*!< UARTB RXSTATUS: RDNFULL Mask            */
#define UARTB_RXSTATUS_RXBUSY_Pos             2                                                       /*!< UARTB RXSTATUS: RXBUSY Position         */
#define UARTB_RXSTATUS_RXBUSY_Msk             (0x01UL << UARTB_RXSTATUS_RXBUSY_Pos)                   /*!< UARTB RXSTATUS: RXBUSY Mask             */
#define UARTB_RXSTATUS_RXTO_Pos               3                                                       /*!< UARTB RXSTATUS: RXTO Position           */
#define UARTB_RXSTATUS_RXTO_Msk               (0x01UL << UARTB_RXSTATUS_RXTO_Pos)                     /*!< UARTB RXSTATUS: RXTO Mask               */
#define UARTB_RXSTATUS_RXOVR_Pos              4                                                       /*!< UARTB RXSTATUS: RXOVR Position          */
#define UARTB_RXSTATUS_RXOVR_Msk              (0x01UL << UARTB_RXSTATUS_RXOVR_Pos)                    /*!< UARTB RXSTATUS: RXOVR Mask              */
#define UARTB_RXSTATUS_RXFRM_Pos              5                                                       /*!< UARTB RXSTATUS: RXFRM Position          */
#define UARTB_RXSTATUS_RXFRM_Msk              (0x01UL << UARTB_RXSTATUS_RXFRM_Pos)                    /*!< UARTB RXSTATUS: RXFRM Mask              */
#define UARTB_RXSTATUS_RXPAR_Pos              6                                                       /*!< UARTB RXSTATUS: RXPAR Position          */
#define UARTB_RXSTATUS_RXPAR_Msk              (0x01UL << UARTB_RXSTATUS_RXPAR_Pos)                    /*!< UARTB RXSTATUS: RXPAR Mask              */
#define UARTB_RXSTATUS_RXBRK_Pos              7                                                       /*!< UARTB RXSTATUS: RXBRK Position          */
#define UARTB_RXSTATUS_RXBRK_Msk              (0x01UL << UARTB_RXSTATUS_RXBRK_Pos)                    /*!< UARTB RXSTATUS: RXBRK Mask              */
#define UARTB_RXSTATUS_RXBUSYBRK_Pos          8                                                       /*!< UARTB RXSTATUS: RXBUSYBRK Position      */
#define UARTB_RXSTATUS_RXBUSYBRK_Msk          (0x01UL << UARTB_RXSTATUS_RXBUSYBRK_Pos)                /*!< UARTB RXSTATUS: RXBUSYBRK Mask          */
#define UARTB_RXSTATUS_RXADDR9_Pos            9                                                       /*!< UARTB RXSTATUS: RXADDR9 Position        */
#define UARTB_RXSTATUS_RXADDR9_Msk            (0x01UL << UARTB_RXSTATUS_RXADDR9_Pos)                  /*!< UARTB RXSTATUS: RXADDR9 Mask            */
#define UARTB_RXSTATUS_RXRTSN_Pos             15                                                      /*!< UARTB RXSTATUS: RXRTSN Position         */
#define UARTB_RXSTATUS_RXRTSN_Msk             (0x01UL << UARTB_RXSTATUS_RXRTSN_Pos)                   /*!< UARTB RXSTATUS: RXRTSN Mask             */

/* -------------------------------  UARTB_TXSTATUS  ------------------------------- */
#define UARTB_TXSTATUS_WRRDY_Pos              0                                                       /*!< UARTB TXSTATUS: WRRDY Position          */
#define UARTB_TXSTATUS_WRRDY_Msk              (0x01UL << UARTB_TXSTATUS_WRRDY_Pos)                    /*!< UARTB TXSTATUS: WRRDY Mask              */
#define UARTB_TXSTATUS_WRBUSY_Pos             1                                                       /*!< UARTB TXSTATUS: WRBUSY Position         */
#define UARTB_TXSTATUS_WRBUSY_Msk             (0x01UL << UARTB_TXSTATUS_WRBUSY_Pos)                   /*!< UARTB TXSTATUS: WRBUSY Mask             */
#define UARTB_TXSTATUS_TXBUSY_Pos             2                                                       /*!< UARTB TXSTATUS: TXBUSY Position         */
#define UARTB_TXSTATUS_TXBUSY_Msk             (0x01UL << UARTB_TXSTATUS_TXBUSY_Pos)                   /*!< UARTB TXSTATUS: TXBUSY Mask             */
#define UARTB_TXSTATUS_WRLOST_Pos             3                                                       /*!< UARTB TXSTATUS: WRLOST Position         */
#define UARTB_TXSTATUS_WRLOST_Msk             (0x01UL << UARTB_TXSTATUS_WRLOST_Pos)                   /*!< UARTB TXSTATUS: WRLOST Mask             */
#define UARTB_TXSTATUS_TXCTSN_Pos             15                                                      /*!< UARTB TXSTATUS: TXCTSN Position         */
#define UARTB_TXSTATUS_TXCTSN_Msk             (0x01UL << UARTB_TXSTATUS_TXCTSN_Pos)                   /*!< UARTB TXSTATUS: TXCTSN Mask             */

/* -------------------------------  UARTB_FIFO_CLR  ------------------------------- */
#define UARTB_FIFO_CLR_RXSTS_Pos              0                                                       /*!< UARTB FIFO_CLR: RXSTS Position          */
#define UARTB_FIFO_CLR_RXSTS_Msk              (0x01UL << UARTB_FIFO_CLR_RXSTS_Pos)                    /*!< UARTB FIFO_CLR: RXSTS Mask              */
#define UARTB_FIFO_CLR_TXSTS_Pos              1                                                       /*!< UARTB FIFO_CLR: TXSTS Position          */
#define UARTB_FIFO_CLR_TXSTS_Msk              (0x01UL << UARTB_FIFO_CLR_TXSTS_Pos)                    /*!< UARTB FIFO_CLR: TXSTS Mask              */
#define UARTB_FIFO_CLR_RXFIFO_Pos             2                                                       /*!< UARTB FIFO_CLR: RXFIFO Position         */
#define UARTB_FIFO_CLR_RXFIFO_Msk             (0x01UL << UARTB_FIFO_CLR_RXFIFO_Pos)                   /*!< UARTB FIFO_CLR: RXFIFO Mask             */
#define UARTB_FIFO_CLR_TXFIFO_Pos             3                                                       /*!< UARTB FIFO_CLR: TXFIFO Position         */
#define UARTB_FIFO_CLR_TXFIFO_Msk             (0x01UL << UARTB_FIFO_CLR_TXFIFO_Pos)                   /*!< UARTB FIFO_CLR: TXFIFO Mask             */

/* ------------------------------  UARTB_ADDR9  ----------------------------------- */
#define UARTB_ADDR9_DATA_Pos    							0                                                       /*!< UARTB ADDR9: DATA Position 						 */
#define UARTB_ADDR9_DATA_Msk    							(0x01UL << UARTB_ADDR9_DATA_Pos)          							/*!< UARTB ADDR9: DATA Mask    							 */
#define UARTB_ADDR9_ENB9BIT_Pos    						1                                                       /*!< UARTB ADDR9:	ENB9BIT Position 					 */
#define UARTB_ADDR9_ENB9BIT_Msk     					(0x01UL << UARTB_ADDR9_ENB9BIT_Pos)           					/*!< UARTB ADDR9: ENB9BIT Mask    					 */

/* --------------------------------  UARTB_IRQ_ENB  ------------------------------- */
#define UARTB_IRQ_ENB_IRQ_RX_Pos              0                                                       /*!< UARTB IRQ_ENB: IRQ_RX Position          */
#define UARTB_IRQ_ENB_IRQ_RX_Msk              (0x01UL << UARTB_IRQ_ENB_IRQ_RX_Pos)                    /*!< UARTB IRQ_ENB: IRQ_RX Mask              */
#define UARTB_IRQ_ENB_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTB IRQ_ENB: IRQ_RX_STATUS Position   */
#define UARTB_IRQ_ENB_IRQ_RX_STATUS_Msk       (0x01UL << UARTB_IRQ_ENB_IRQ_RX_STATUS_Pos)             /*!< UARTB IRQ_ENB: IRQ_RX_STATUS Mask       */
#define UARTB_IRQ_ENB_IRQ_RX_TO_Pos           2                                                       /*!< UARTB IRQ_ENB: IRQ_RX_TO Position       */
#define UARTB_IRQ_ENB_IRQ_RX_TO_Msk           (0x01UL << UARTB_IRQ_ENB_IRQ_RX_TO_Pos)                 /*!< UARTB IRQ_ENB: IRQ_RX_TO Mask           */
#define UARTB_IRQ_ENB_IRQ_RX_ADDR9_Pos				3                                                   		/*!< UARTB IRQ_ENB: IRQ_RX_ADDR9 Position */
#define UARTB_IRQ_ENB_IRQ_RX_ADDR9_Msk 				(0x01UL << UARTB_IRQ_ENB_IRQ_RX_ADDR9_Pos) 							/*!< UARTB IRQ_ENB: IRQ_RX_ADDR9 Mask */
#define UARTB_IRQ_ENB_IRQ_TX_Pos              4                                                       /*!< UARTB IRQ_ENB: IRQ_TX Position          */
#define UARTB_IRQ_ENB_IRQ_TX_Msk              (0x01UL << UARTB_IRQ_ENB_IRQ_TX_Pos)                    /*!< UARTB IRQ_ENB: IRQ_TX Mask              */
#define UARTB_IRQ_ENB_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTB IRQ_ENB: IRQ_TX_STATUS Position   */
#define UARTB_IRQ_ENB_IRQ_TX_STATUS_Msk       (0x01UL << UARTB_IRQ_ENB_IRQ_TX_STATUS_Pos)             /*!< UARTB IRQ_ENB: IRQ_TX_STATUS Mask       */
#define UARTB_IRQ_ENB_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTB IRQ_ENB: IRQ_TX_EMPTY Position    */
#define UARTB_IRQ_ENB_IRQ_TX_EMPTY_Msk        (0x01UL << UARTB_IRQ_ENB_IRQ_TX_EMPTY_Pos)              /*!< UARTB IRQ_ENB: IRQ_TX_EMPTY Mask        */
#define UARTB_IRQ_ENB_IRQ_TX_CTS_Pos          7                                                       /*!< UARTB IRQ_ENB: IRQ_TX_CTS Position      */
#define UARTB_IRQ_ENB_IRQ_TX_CTS_Msk          (0x01UL << UARTB_IRQ_ENB_IRQ_TX_CTS_Pos)                /*!< UARTB IRQ_ENB: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTB_IRQ_RAW  ------------------------------- */
#define UARTB_IRQ_RAW_IRQ_RX_Pos              0                                                       /*!< UARTB IRQ_RAW: IRQ_RX Position          */
#define UARTB_IRQ_RAW_IRQ_RX_Msk              (0x01UL << UARTB_IRQ_RAW_IRQ_RX_Pos)                    /*!< UARTB IRQ_RAW: IRQ_RX Mask              */
#define UARTB_IRQ_RAW_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTB IRQ_RAW: IRQ_RX_STATUS Position   */
#define UARTB_IRQ_RAW_IRQ_RX_STATUS_Msk       (0x01UL << UARTB_IRQ_RAW_IRQ_RX_STATUS_Pos)             /*!< UARTB IRQ_RAW: IRQ_RX_STATUS Mask       */
#define UARTB_IRQ_RAW_IRQ_RX_TO_Pos           2                                                       /*!< UARTB IRQ_RAW: IRQ_RX_TO Position       */
#define UARTB_IRQ_RAW_IRQ_RX_TO_Msk           (0x01UL << UARTB_IRQ_RAW_IRQ_RX_TO_Pos)                 /*!< UARTB IRQ_RAW: IRQ_RX_TO Mask           */
#define UARTB_IRQ_RAW_IRQ_TX_Pos              4                                                       /*!< UARTB IRQ_RAW: IRQ_TX Position          */
#define UARTB_IRQ_RAW_IRQ_TX_Msk              (0x01UL << UARTB_IRQ_RAW_IRQ_TX_Pos)                    /*!< UARTB IRQ_RAW: IRQ_TX Mask              */
#define UARTB_IRQ_RAW_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTB IRQ_RAW: IRQ_TX_STATUS Position   */
#define UARTB_IRQ_RAW_IRQ_TX_STATUS_Msk       (0x01UL << UARTB_IRQ_RAW_IRQ_TX_STATUS_Pos)             /*!< UARTB IRQ_RAW: IRQ_TX_STATUS Mask       */
#define UARTB_IRQ_RAW_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTB IRQ_RAW: IRQ_TX_EMPTY Position    */
#define UARTB_IRQ_RAW_IRQ_TX_EMPTY_Msk        (0x01UL << UARTB_IRQ_RAW_IRQ_TX_EMPTY_Pos)              /*!< UARTB IRQ_RAW: IRQ_TX_EMPTY Mask        */
#define UARTB_IRQ_RAW_IRQ_TX_CTS_Pos          7                                                       /*!< UARTB IRQ_RAW: IRQ_TX_CTS Position      */
#define UARTB_IRQ_RAW_IRQ_TX_CTS_Msk          (0x01UL << UARTB_IRQ_RAW_IRQ_TX_CTS_Pos)                /*!< UARTB IRQ_RAW: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTB_IRQ_END  ------------------------------- */
#define UARTB_IRQ_END_IRQ_RX_Pos              0                                                       /*!< UARTB IRQ_END: IRQ_RX Position          */
#define UARTB_IRQ_END_IRQ_RX_Msk              (0x01UL << UARTB_IRQ_END_IRQ_RX_Pos)                    /*!< UARTB IRQ_END: IRQ_RX Mask              */
#define UARTB_IRQ_END_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTB IRQ_END: IRQ_RX_STATUS Position   */
#define UARTB_IRQ_END_IRQ_RX_STATUS_Msk       (0x01UL << UARTB_IRQ_END_IRQ_RX_STATUS_Pos)             /*!< UARTB IRQ_END: IRQ_RX_STATUS Mask       */
#define UARTB_IRQ_END_IRQ_RX_TO_Pos           2                                                       /*!< UARTB IRQ_END: IRQ_RX_TO Position       */
#define UARTB_IRQ_END_IRQ_RX_TO_Msk           (0x01UL << UARTB_IRQ_END_IRQ_RX_TO_Pos)                 /*!< UARTB IRQ_END: IRQ_RX_TO Mask           */
#define UARTB_IRQ_END_IRQ_TX_Pos              4                                                       /*!< UARTB IRQ_END: IRQ_TX Position          */
#define UARTB_IRQ_END_IRQ_TX_Msk              (0x01UL << UARTB_IRQ_END_IRQ_TX_Pos)                    /*!< UARTB IRQ_END: IRQ_TX Mask              */
#define UARTB_IRQ_END_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTB IRQ_END: IRQ_TX_STATUS Position   */
#define UARTB_IRQ_END_IRQ_TX_STATUS_Msk       (0x01UL << UARTB_IRQ_END_IRQ_TX_STATUS_Pos)             /*!< UARTB IRQ_END: IRQ_TX_STATUS Mask       */
#define UARTB_IRQ_END_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTB IRQ_END: IRQ_TX_EMPTY Position    */
#define UARTB_IRQ_END_IRQ_TX_EMPTY_Msk        (0x01UL << UARTB_IRQ_END_IRQ_TX_EMPTY_Pos)              /*!< UARTB IRQ_END: IRQ_TX_EMPTY Mask        */
#define UARTB_IRQ_END_IRQ_TX_CTS_Pos          7                                                       /*!< UARTB IRQ_END: IRQ_TX_CTS Position      */
#define UARTB_IRQ_END_IRQ_TX_CTS_Msk          (0x01UL << UARTB_IRQ_END_IRQ_TX_CTS_Pos)                /*!< UARTB IRQ_END: IRQ_TX_CTS Mask          */

/* --------------------------------  UARTB_IRQ_CLR  ------------------------------- */
#define UARTB_IRQ_CLR_IRQ_RX_Pos              0                                                       /*!< UARTB IRQ_CLR: IRQ_RX Position          */
#define UARTB_IRQ_CLR_IRQ_RX_Msk              (0x01UL << UARTB_IRQ_CLR_IRQ_RX_Pos)                    /*!< UARTB IRQ_CLR: IRQ_RX Mask              */
#define UARTB_IRQ_CLR_IRQ_RX_STATUS_Pos       1                                                       /*!< UARTB IRQ_CLR: IRQ_RX_STATUS Position   */
#define UARTB_IRQ_CLR_IRQ_RX_STATUS_Msk       (0x01UL << UARTB_IRQ_CLR_IRQ_RX_STATUS_Pos)             /*!< UARTB IRQ_CLR: IRQ_RX_STATUS Mask       */
#define UARTB_IRQ_CLR_IRQ_RX_TO_Pos           2                                                       /*!< UARTB IRQ_CLR: IRQ_RX_TO Position       */
#define UARTB_IRQ_CLR_IRQ_RX_TO_Msk           (0x01UL << UARTB_IRQ_CLR_IRQ_RX_TO_Pos)                 /*!< UARTB IRQ_CLR: IRQ_RX_TO Mask           */
#define UARTB_IRQ_CLR_IRQ_TX_Pos              4                                                       /*!< UARTB IRQ_CLR: IRQ_TX Position          */
#define UARTB_IRQ_CLR_IRQ_TX_Msk              (0x01UL << UARTB_IRQ_CLR_IRQ_TX_Pos)                    /*!< UARTB IRQ_CLR: IRQ_TX Mask              */
#define UARTB_IRQ_CLR_IRQ_TX_STATUS_Pos       5                                                       /*!< UARTB IRQ_CLR: IRQ_TX_STATUS Position   */
#define UARTB_IRQ_CLR_IRQ_TX_STATUS_Msk       (0x01UL << UARTB_IRQ_CLR_IRQ_TX_STATUS_Pos)             /*!< UARTB IRQ_CLR: IRQ_TX_STATUS Mask       */
#define UARTB_IRQ_CLR_IRQ_TX_EMPTY_Pos        6                                                       /*!< UARTB IRQ_CLR: IRQ_TX_EMPTY Position    */
#define UARTB_IRQ_CLR_IRQ_TX_EMPTY_Msk        (0x01UL << UARTB_IRQ_CLR_IRQ_TX_EMPTY_Pos)              /*!< UARTB IRQ_CLR: IRQ_TX_EMPTY Mask        */
#define UARTB_IRQ_CLR_IRQ_TX_CTS_Pos          7                                                       /*!< UARTB IRQ_CLR: IRQ_TX_CTS Position      */
#define UARTB_IRQ_CLR_IRQ_TX_CTS_Msk          (0x01UL << UARTB_IRQ_CLR_IRQ_TX_CTS_Pos)                /*!< UARTB IRQ_CLR: IRQ_TX_CTS Mask          */

/* ------------------------------   UARTB_STATE  ----------------------------------- */
#define UARTB_STATE_RX_STATE_Pos    					0                                              					/*!< UARTB STATE: RX_STATE Position 				 */
#define UARTB_STATE_RX_STATE_Msk    					(0xffUL << UARTB_STATE_RX_STATE_Pos)      						/*!< UARTB STATE: RX_STATE Mask    					 */
#define UARTB_STATE_RX_FIFO_Pos 						8                                         						/*!< UARTB STATE: RX_FIFO Position 					 */
#define UARTB_STATE_RX_FIFO_Msk 						(0xffUL << UARTB_STATE_RX_FIFO_Pos)								/*!< UARTB STATE: RX_FIFO Mask 							 */
#define UARTB_STATE_TX_STATE_Pos 						16                                             					/*!< UARTB STATE: TX_STATE Position 				 */
#define UARTB_STATE_TX_STATE_Msk 						(0xffUL << UARTB_STATE_TX_STATE_Pos)  							/*!< UARTB STATE: TX_STATE Mask 						 */
#define UARTB_STATE_TX_FIFO_Pos     					24                                             					/*!< UARTB STATE: TX_FIFO Position 					 */
#define UARTB_STATE_TX_FIFO_Msk     					(0xffUL << UARTB_STATE_TX_FIFO_Pos) 							/*!< UARTB STATE: TX_FIFO Mask    					 */

/* ================================================================================ */
/* ================     Group 'SPI_PERIPHERAL' Position & Mask     ================ */
/* ================================================================================ */

/* ---------------------------  SPI_PERIPHERAL_DATA  --------------------------- */
#define SPI_PERIPHERAL_DATA_VALUE_Pos   			 0                                                      /*!< SPI_PERIPHERAL DATA: VALUE Position 					*/
#define SPI_PERIPHERAL_DATA_VALUE_Msk   			 (0x000000ffffUL << SPI_PERIPHERAL_DATA_VALUE_Pos)   		/*!< SPI_PERIPHERAL DATA: VALUE Mask   						*/
#define SPI_PERIPHERAL_DATA_BMSKIPDATA_Pos   	 30                                                     /*!< SPI_PERIPHERAL DATA: BMSKIPDATA Position 		*/
#define SPI_PERIPHERAL_DATA_BMSKIPDATA_Msk   	 (0x01UL << SPI_PERIPHERAL_DATA_BMSKIPDATA_Pos)         /*!< SPI_PERIPHERAL DATA: BMSKIPDATA Mask   			*/
#define SPI_PERIPHERAL_DATA_BMSTART_BMSTOP_Pos 31                                                     /*!< SPI_PERIPHERAL DATA: BMSTART/BMSTOP Position */
#define SPI_PERIPHERAL_DATA_BMSTART_BMSTOP_Msk (0x01UL << SPI_PERIPHERAL_DATA_BMSTART_BMSTOP_Pos)     /*!< SPI_PERIPHERAL DATA: BMSTART/BMSTOP Mask   	*/


/* ----------------------------  SPI_PERIPHERAL_CTRL0  ---------------------------- */
#define SPI_PERIPHERAL_CTRL0_SIZE_Pos         0                                                       /*!< SPI_PERIPHERAL CTRL0: SIZE Position     */
#define SPI_PERIPHERAL_CTRL0_SIZE_Msk         (0x0fUL << SPI_PERIPHERAL_CTRL0_SIZE_Pos)               /*!< SPI_PERIPHERAL CTRL0: SIZE Mask         */
#define SPI_PERIPHERAL_CTRL0_SPO_Pos          6                                                       /*!< SPI_PERIPHERAL CTRL0: SPO Position      */
#define SPI_PERIPHERAL_CTRL0_SPO_Msk          (0x01UL << SPI_PERIPHERAL_CTRL0_SPO_Pos)                /*!< SPI_PERIPHERAL CTRL0: SPO Mask          */
#define SPI_PERIPHERAL_CTRL0_SPH_Pos          7                                                       /*!< SPI_PERIPHERAL CTRL0: SPH Position      */
#define SPI_PERIPHERAL_CTRL0_SPH_Msk          (0x01UL << SPI_PERIPHERAL_CTRL0_SPH_Pos)                /*!< SPI_PERIPHERAL CTRL0: SPH Mask          */
#define SPI_PERIPHERAL_CTRL0_SCRDV_Pos        8                                                       /*!< SPI_PERIPHERAL CTRL0: SCRDV Position    */
#define SPI_PERIPHERAL_CTRL0_SCRDV_Msk        (0x000000ffUL << SPI_PERIPHERAL_CTRL0_SCRDV_Pos)        /*!< SPI_PERIPHERAL CTRL0: SCRDV Mask        */

/* ----------------------------  SPI_PERIPHERAL_CTRL1  ---------------------------- */
#define SPI_PERIPHERAL_CTRL1_LBM_Pos          0                                                       /*!< SPI_PERIPHERAL CTRL1: LBM Position      */
#define SPI_PERIPHERAL_CTRL1_LBM_Msk          (0x01UL << SPI_PERIPHERAL_CTRL1_LBM_Pos)                /*!< SPI_PERIPHERAL CTRL1: LBM Mask          */
#define SPI_PERIPHERAL_CTRL1_ENABLE_Pos       1                                                       /*!< SPI_PERIPHERAL CTRL1: ENABLE Position   */
#define SPI_PERIPHERAL_CTRL1_ENABLE_Msk       (0x01UL << SPI_PERIPHERAL_CTRL1_ENABLE_Pos)             /*!< SPI_PERIPHERAL CTRL1: ENABLE Mask       */
#define SPI_PERIPHERAL_CTRL1_MS_Pos           2                                                       /*!< SPI_PERIPHERAL CTRL1: MS Position       */
#define SPI_PERIPHERAL_CTRL1_MS_Msk           (0x01UL << SPI_PERIPHERAL_CTRL1_MS_Pos)                 /*!< SPI_PERIPHERAL CTRL1: MS Mask           */
#define SPI_PERIPHERAL_CTRL1_SOD_Pos          3                                                       /*!< SPI_PERIPHERAL CTRL1: SOD Position      */
#define SPI_PERIPHERAL_CTRL1_SOD_Msk          (0x01UL << SPI_PERIPHERAL_CTRL1_SOD_Pos)                /*!< SPI_PERIPHERAL CTRL1: SOD Mask          */
#define SPI_PERIPHERAL_CTRL1_SS_Pos           4                                                       /*!< SPI_PERIPHERAL CTRL1: SS Position       */
#define SPI_PERIPHERAL_CTRL1_SS_Msk           (0x07UL << SPI_PERIPHERAL_CTRL1_SS_Pos)                 /*!< SPI_PERIPHERAL CTRL1: SS Mask           */
#define SPI_PERIPHERAL_CTRL1_BLOCKMODE_Pos    7                                                       /*!< SPI_PERIPHERAL CTRL1: BLOCKMODE Position*/
#define SPI_PERIPHERAL_CTRL1_BLOCKMODE_Msk    (0x01UL << SPI_PERIPHERAL_CTRL1_BLOCKMODE_Pos)          /*!< SPI_PERIPHERAL CTRL1: BLOCKMODE Mask    */
//SPI_PERIPHERAL_CTRL1_BYTEMODE is a typo of "BLOCKMODE" and retained only for backwards compatibility reasons
#define SPI_PERIPHERAL_CTRL1_BYTEMODE_Pos     SPI_PERIPHERAL_CTRL1_BLOCKMODE_Pos                      /*!< SPI_PERIPHERAL CTRL1: BYTEMODE Position */
#define SPI_PERIPHERAL_CTRL1_BYTEMODE_Msk     SPI_PERIPHERAL_CTRL1_BLOCKMODE_Msk                      /*!< SPI_PERIPHERAL CTRL1: BYTEMODE Mask     */
#define SPI_PERIPHERAL_CTRL1_BMSTART_Pos      8                                                       /*!< SPI_PERIPHERAL CTRL1: BMSTART Position  */
#define SPI_PERIPHERAL_CTRL1_BMSTART_Msk      (0x01UL << SPI_PERIPHERAL_CTRL1_BMSTART_Pos)            /*!< SPI_PERIPHERAL CTRL1: BMSTART Mask      */
#define SPI_PERIPHERAL_CTRL1_BMSTALL_Pos      9                                                       /*!< SPI_PERIPHERAL CTRL1: BMSTALL Position  */
#define SPI_PERIPHERAL_CTRL1_BMSTALL_Msk      (0x01UL << SPI_PERIPHERAL_CTRL1_BMSTALL_Pos)            /*!< SPI_PERIPHERAL CTRL1: BMSTALL Mask      */
#define SPI_PERIPHERAL_CTRL1_MDLYCAP_Pos      10                                                      /*!< SPI_PERIPHERAL CTRL1: MDLYCAP Position  */
#define SPI_PERIPHERAL_CTRL1_MDLYCAP_Msk      (0x01UL << SPI_PERIPHERAL_CTRL1_MDLYCAP_Pos)            /*!< SPI_PERIPHERAL CTRL1: MDLYCAP Mask      */
#define SPI_PERIPHERAL_CTRL1_MTXPAUSE_Pos     11                                                      /*!< SPI_PERIPHERAL CTRL1: MTXPAUSE Position */
#define SPI_PERIPHERAL_CTRL1_MTXPAUSE_Msk     (0x01UL << SPI_PERIPHERAL_CTRL1_MTXPAUSE_Pos)           /*!< SPI_PERIPHERAL CTRL1: MTXPAUSE Mask     */

/* ----------------------------  SPI_PERIPHERAL_STATUS  --------------------------- */
#define SPI_PERIPHERAL_STATUS_TFE_Pos         0                                                       /*!< SPI_PERIPHERAL STATUS: TFE Position     */
#define SPI_PERIPHERAL_STATUS_TFE_Msk         (0x01UL << SPI_PERIPHERAL_STATUS_TFE_Pos)               /*!< SPI_PERIPHERAL STATUS: TFE Mask         */
#define SPI_PERIPHERAL_STATUS_TNF_Pos         1                                                       /*!< SPI_PERIPHERAL STATUS: TNF Position     */
#define SPI_PERIPHERAL_STATUS_TNF_Msk         (0x01UL << SPI_PERIPHERAL_STATUS_TNF_Pos)               /*!< SPI_PERIPHERAL STATUS: TNF Mask         */
#define SPI_PERIPHERAL_STATUS_RNE_Pos         2                                                       /*!< SPI_PERIPHERAL STATUS: RNE Position     */
#define SPI_PERIPHERAL_STATUS_RNE_Msk         (0x01UL << SPI_PERIPHERAL_STATUS_RNE_Pos)               /*!< SPI_PERIPHERAL STATUS: RNE Mask         */
#define SPI_PERIPHERAL_STATUS_RFF_Pos         3                                                       /*!< SPI_PERIPHERAL STATUS: RFF Position     */
#define SPI_PERIPHERAL_STATUS_RFF_Msk         (0x01UL << SPI_PERIPHERAL_STATUS_RFF_Pos)               /*!< SPI_PERIPHERAL STATUS: RFF Mask         */
#define SPI_PERIPHERAL_STATUS_BUSY_Pos        4                                                       /*!< SPI_PERIPHERAL STATUS: BUSY Position    */
#define SPI_PERIPHERAL_STATUS_BUSY_Msk        (0x01UL << SPI_PERIPHERAL_STATUS_BUSY_Pos)              /*!< SPI_PERIPHERAL STATUS: BUSY Mask        */
#define SPI_PERIPHERAL_STATUS_RXDATAFIRST_Pos 5                                                       /*!< SPI_PERIPHERAL STATUS: RXDATAFIRST Position */
#define SPI_PERIPHERAL_STATUS_RXDATAFIRST_Msk (0x01UL << SPI_PERIPHERAL_STATUS_RXDATAFIRST_Pos)       /*!< SPI_PERIPHERAL STATUS: RXDATAFIRST Mask */
#define SPI_PERIPHERAL_STATUS_RXTRIGGER_Pos    6                                                       /*!< SPI_PERIPHERAL STATUS: RXTRIGGER Position */
#define SPI_PERIPHERAL_STATUS_RXTRIGGER_Msk    (0x01UL << SPI_PERIPHERAL_STATUS_RXTRIGGER_Pos)          /*!< SPI_PERIPHERAL STATUS: RXTRIGGER Mask    */
#define SPI_PERIPHERAL_STATUS_TXTRIGGER_Pos    7                                                       /*!< SPI_PERIPHERAL STATUS: TXTRIGGER Position */
#define SPI_PERIPHERAL_STATUS_TXTRIGGER_Msk    (0x01UL << SPI_PERIPHERAL_STATUS_TXTRIGGER_Pos)          /*!< SPI_PERIPHERAL STATUS: TXTRIGGER Mask    */

/* ---------------------------  SPI_PERIPHERAL_IRQ_ENB  --------------------------- */
#define SPI_PERIPHERAL_IRQ_ENB_RORIM_Pos      0                                                       /*!< SPI_PERIPHERAL IRQ_ENB: RORIM Position  */
#define SPI_PERIPHERAL_IRQ_ENB_RORIM_Msk      (0x01UL << SPI_PERIPHERAL_IRQ_ENB_RORIM_Pos)            /*!< SPI_PERIPHERAL IRQ_ENB: RORIM Mask      */
#define SPI_PERIPHERAL_IRQ_ENB_RTIM_Pos       1                                                       /*!< SPI_PERIPHERAL IRQ_ENB: RTIM Position   */
#define SPI_PERIPHERAL_IRQ_ENB_RTIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_ENB_RTIM_Pos)             /*!< SPI_PERIPHERAL IRQ_ENB: RTIM Mask       */
#define SPI_PERIPHERAL_IRQ_ENB_RXIM_Pos       2                                                       /*!< SPI_PERIPHERAL IRQ_ENB: RXIM Position   */
#define SPI_PERIPHERAL_IRQ_ENB_RXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_ENB_RXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_ENB: RXIM Mask       */
#define SPI_PERIPHERAL_IRQ_ENB_TXIM_Pos       3                                                       /*!< SPI_PERIPHERAL IRQ_ENB: TXIM Position   */
#define SPI_PERIPHERAL_IRQ_ENB_TXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_ENB_TXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_ENB: TXIM Mask       */

/* ---------------------------  SPI_PERIPHERAL_IRQ_RAW  --------------------------- */
#define SPI_PERIPHERAL_IRQ_RAW_RORIM_Pos      0                                                       /*!< SPI_PERIPHERAL IRQ_RAW: RORIM Position  */
#define SPI_PERIPHERAL_IRQ_RAW_RORIM_Msk      (0x01UL << SPI_PERIPHERAL_IRQ_RAW_RORIM_Pos)            /*!< SPI_PERIPHERAL IRQ_RAW: RORIM Mask      */
#define SPI_PERIPHERAL_IRQ_RAW_RTIM_Pos       1                                                       /*!< SPI_PERIPHERAL IRQ_RAW: RTIM Position   */
#define SPI_PERIPHERAL_IRQ_RAW_RTIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_RAW_RTIM_Pos)             /*!< SPI_PERIPHERAL IRQ_RAW: RTIM Mask       */
#define SPI_PERIPHERAL_IRQ_RAW_RXIM_Pos       2                                                       /*!< SPI_PERIPHERAL IRQ_RAW: RXIM Position   */
#define SPI_PERIPHERAL_IRQ_RAW_RXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_RAW_RXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_RAW: RXIM Mask       */
#define SPI_PERIPHERAL_IRQ_RAW_TXIM_Pos       3                                                       /*!< SPI_PERIPHERAL IRQ_RAW: TXIM Position   */
#define SPI_PERIPHERAL_IRQ_RAW_TXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_RAW_TXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_RAW: TXIM Mask       */

/* ---------------------------  SPI_PERIPHERAL_IRQ_END  --------------------------- */
#define SPI_PERIPHERAL_IRQ_END_RORIM_Pos      0                                                       /*!< SPI_PERIPHERAL IRQ_END: RORIM Position  */
#define SPI_PERIPHERAL_IRQ_END_RORIM_Msk      (0x01UL << SPI_PERIPHERAL_IRQ_END_RORIM_Pos)            /*!< SPI_PERIPHERAL IRQ_END: RORIM Mask      */
#define SPI_PERIPHERAL_IRQ_END_RTIM_Pos       1                                                       /*!< SPI_PERIPHERAL IRQ_END: RTIM Position   */
#define SPI_PERIPHERAL_IRQ_END_RTIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_END_RTIM_Pos)             /*!< SPI_PERIPHERAL IRQ_END: RTIM Mask       */
#define SPI_PERIPHERAL_IRQ_END_RXIM_Pos       2                                                       /*!< SPI_PERIPHERAL IRQ_END: RXIM Position   */
#define SPI_PERIPHERAL_IRQ_END_RXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_END_RXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_END: RXIM Mask       */
#define SPI_PERIPHERAL_IRQ_END_TXIM_Pos       3                                                       /*!< SPI_PERIPHERAL IRQ_END: TXIM Position   */
#define SPI_PERIPHERAL_IRQ_END_TXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_END_TXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_END: TXIM Mask       */

/* ---------------------------  SPI_PERIPHERAL_IRQ_CLR  --------------------------- */
#define SPI_PERIPHERAL_IRQ_CLR_RORIM_Pos      0                                                       /*!< SPI_PERIPHERAL IRQ_CLR: RORIM Position  */
#define SPI_PERIPHERAL_IRQ_CLR_RORIM_Msk      (0x01UL << SPI_PERIPHERAL_IRQ_CLR_RORIM_Pos)            /*!< SPI_PERIPHERAL IRQ_CLR: RORIM Mask      */
#define SPI_PERIPHERAL_IRQ_CLR_RTIM_Pos       1                                                       /*!< SPI_PERIPHERAL IRQ_CLR: RTIM Position   */
#define SPI_PERIPHERAL_IRQ_CLR_RTIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_CLR_RTIM_Pos)             /*!< SPI_PERIPHERAL IRQ_CLR: RTIM Mask       */
#define SPI_PERIPHERAL_IRQ_CLR_RXIM_Pos       2                                                       /*!< SPI_PERIPHERAL IRQ_CLR: RXIM Position   */
#define SPI_PERIPHERAL_IRQ_CLR_RXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_CLR_RXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_CLR: RXIM Mask       */
#define SPI_PERIPHERAL_IRQ_CLR_TXIM_Pos       3                                                       /*!< SPI_PERIPHERAL IRQ_CLR: TXIM Position   */
#define SPI_PERIPHERAL_IRQ_CLR_TXIM_Msk       (0x01UL << SPI_PERIPHERAL_IRQ_CLR_TXIM_Pos)             /*!< SPI_PERIPHERAL IRQ_CLR: TXIM Mask       */

/* ---------------------------  SPI_PERIPHERAL_FIFO_CLR  -------------------------- */
#define SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Pos    0                                                       /*!< SPI_PERIPHERAL FIFO_CLR: RXFIFO Position */
#define SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Msk    (0x01UL << SPI_PERIPHERAL_FIFO_CLR_RXFIFO_Pos)          /*!< SPI_PERIPHERAL FIFO_CLR: RXFIFO Mask    */
#define SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Pos    1                                                       /*!< SPI_PERIPHERAL FIFO_CLR: TXFIFO Position */
#define SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Msk    (0x01UL << SPI_PERIPHERAL_FIFO_CLR_TXFIFO_Pos)          /*!< SPI_PERIPHERAL FIFO_CLR: TXFIFO Mask    */

/* ---------------------------  SPI_PERIPHERAL_STATE  -------------------------- */
#define SPI_PERIPHERAL_STATE_RX_STATE_Pos    0                                                        /*!< SPI_PERIPHERAL STATE: RX_STATE Position */
#define SPI_PERIPHERAL_STATE_RX_STATE_Msk    (0xffUL << SPI_PERIPHERAL_STATE_RX_STATE_Pos)            /*!< SPI_PERIPHERAL STATE: RX_STATE Mask    */
#define SPI_PERIPHERAL_STATE_RX_FIFO_Pos 	8                                                  		  /*!< SPI_PERIPHERAL STATE: RX_FIFO Position */
#define SPI_PERIPHERAL_STATE_RX_FIFO_Msk    (0xffUL << SPI_PERIPHERAL_STATE_RX_FIFO_Pos)			  /*!< SPI_PERIPHERAL STATE: RX_FIFO Mask */
#define SPI_PERIPHERAL_STATE_ZERO_Pos 	 	16                                                       /*!< SPI_PERIPHERAL STATE: ZERO Position */
#define SPI_PERIPHERAL_STATE_ZERO_Msk 	    (0xffUL << SPI_PERIPHERAL_STATE_ZERO_Pos)            /*!< SPI_PERIPHERAL STATE: ZERO Mask */
#define SPI_PERIPHERAL_STATE_TX_FIFO_Pos     24                                                       /*!< SPI_PERIPHERAL STATE: TX_FIFO Position */
#define SPI_PERIPHERAL_STATE_TX_FIFO_Msk     (0xffUL << SPI_PERIPHERAL_STATE_TX_FIFO_Pos)       /*!< SPI_PERIPHERAL STATE: TX_FIFO Mask    */


/* ================================================================================ */
/* ================          struct 'SPIA' Position & Mask         ================ */
/* ================================================================================ */

/* ----------------------------------  SPIA_DATA  --------------------------- */
#define SPIA_DATA_VALUE_Pos   			 					0                                                      /*!< SPIA DATA: VALUE Position 					*/
#define SPIA_DATA_VALUE_Msk   			 					(0x000000ffffUL << SPIA_DATA_VALUE_Pos)   						 /*!< SPIA DATA: VALUE Mask   						*/
#define SPIA_DATA_BMSKIPDATA_Pos   	 					30                                                     /*!< SPIA DATA: BMSKIPDATA Position 		*/
#define SPIA_DATA_BMSKIPDATA_Msk   	 					(0x01UL << SPIA_DATA_BMSKIPDATA_Pos)         					 /*!< SPIA DATA: BMSKIPDATA Mask   			*/
#define SPIA_DATA_BMSTART_BMSTOP_Pos 					31                                                     /*!< SPIA DATA: BMSTART/BMSTOP Position */
#define SPIA_DATA_BMSTART_BMSTOP_Msk 					(0x01UL << SPIA_DATA_BMSTART_BMSTOP_Pos)     					 /*!< SPIA DATA: BMSTART/BMSTOP Mask   	*/

/* ---------------------------------  SPIA_CTRL0  --------------------------------- */
#define SPIA_CTRL0_SIZE_Pos                   0                                                       /*!< SPIA CTRL0: SIZE Position               */
#define SPIA_CTRL0_SIZE_Msk                   (0x0fUL << SPIA_CTRL0_SIZE_Pos)                         /*!< SPIA CTRL0: SIZE Mask                   */
#define SPIA_CTRL0_SPO_Pos                    6                                                       /*!< SPIA CTRL0: SPO Position                */
#define SPIA_CTRL0_SPO_Msk                    (0x01UL << SPIA_CTRL0_SPO_Pos)                          /*!< SPIA CTRL0: SPO Mask                    */
#define SPIA_CTRL0_SPH_Pos                    7                                                       /*!< SPIA CTRL0: SPH Position                */
#define SPIA_CTRL0_SPH_Msk                    (0x01UL << SPIA_CTRL0_SPH_Pos)                          /*!< SPIA CTRL0: SPH Mask                    */
#define SPIA_CTRL0_SCRDV_Pos                  8                                                       /*!< SPIA CTRL0: SCRDV Position              */
#define SPIA_CTRL0_SCRDV_Msk                  (0x000000ffUL << SPIA_CTRL0_SCRDV_Pos)                  /*!< SPIA CTRL0: SCRDV Mask                  */

/* ---------------------------------  SPIA_CTRL1  --------------------------------- */
#define SPIA_CTRL1_LBM_Pos                    0                                                       /*!< SPIA CTRL1: LBM Position                */
#define SPIA_CTRL1_LBM_Msk                    (0x01UL << SPIA_CTRL1_LBM_Pos)                          /*!< SPIA CTRL1: LBM Mask                    */
#define SPIA_CTRL1_ENABLE_Pos                 1                                                       /*!< SPIA CTRL1: ENABLE Position             */
#define SPIA_CTRL1_ENABLE_Msk                 (0x01UL << SPIA_CTRL1_ENABLE_Pos)                       /*!< SPIA CTRL1: ENABLE Mask                 */
#define SPIA_CTRL1_MS_Pos                     2                                                       /*!< SPIA CTRL1: MS Position                 */
#define SPIA_CTRL1_MS_Msk                     (0x01UL << SPIA_CTRL1_MS_Pos)                           /*!< SPIA CTRL1: MS Mask                     */
#define SPIA_CTRL1_SOD_Pos                    3                                                       /*!< SPIA CTRL1: SOD Position                */
#define SPIA_CTRL1_SOD_Msk                    (0x01UL << SPIA_CTRL1_SOD_Pos)                          /*!< SPIA CTRL1: SOD Mask                    */
#define SPIA_CTRL1_SS_Pos                     4                                                       /*!< SPIA CTRL1: SS Position                 */
#define SPIA_CTRL1_SS_Msk                     (0x07UL << SPIA_CTRL1_SS_Pos)                           /*!< SPIA CTRL1: SS Mask                     */
#define SPIA_CTRL1_BLOCKMODE_Pos              7                                                       /*!< SPIA CTRL1: BLOCKMODE Position          */
#define SPIA_CTRL1_BLOCKMODE_Msk              (0x01UL << SPIA_CTRL1_BLOCKMODE_Pos)                    /*!< SPIA CTRL1: BLOCKMODE Mask              */
//SPIA_CTRL1_BYTEMODE is a typo of "BLOCKMODE" and retained only for backwards compatibility reasons
#define SPIA_CTRL1_BYTEMODE_Pos               SPIA_CTRL1_BLOCKMODE_Pos                                /*!< SPIA CTRL1: BYTEMODE Position           */
#define SPIA_CTRL1_BYTEMODE_Msk               SPIA_CTRL1_BLOCKMODE_Msk                                /*!< SPIA CTRL1: BYTEMODE Mask               */
#define SPIA_CTRL1_BMSTART_Pos                8                                                       /*!< SPIA CTRL1: BMSTART Position            */
#define SPIA_CTRL1_BMSTART_Msk                (0x01UL << SPIA_CTRL1_BMSTART_Pos)                      /*!< SPIA CTRL1: BMSTART Mask                */
#define SPIA_CTRL1_BMSTALL_Pos                9                                                       /*!< SPIA CTRL1: BMSTALL Position            */
#define SPIA_CTRL1_BMSTALL_Msk                (0x01UL << SPIA_CTRL1_BMSTALL_Pos)                      /*!< SPIA CTRL1: BMSTALL Mask                */
#define SPIA_CTRL1_MDLYCAP_Pos                10                                                      /*!< SPIA CTRL1: MDLYCAP Position            */
#define SPIA_CTRL1_MDLYCAP_Msk                (0x01UL << SPIA_CTRL1_MDLYCAP_Pos)                      /*!< SPIA CTRL1: MDLYCAP Mask                */
#define SPIA_CTRL1_MTXPAUSE_Pos               11                                                      /*!< SPIA CTRL1: MTXPAUSE Position           */
#define SPIA_CTRL1_MTXPAUSE_Msk               (0x01UL << SPIA_CTRL1_MTXPAUSE_Pos)                     /*!< SPIA CTRL1: MTXPAUSE Mask               */

/* ---------------------------------  SPIA_STATUS  -------------------------------- */
#define SPIA_STATUS_TFE_Pos                   0                                                       /*!< SPIA STATUS: TFE Position               */
#define SPIA_STATUS_TFE_Msk                   (0x01UL << SPIA_STATUS_TFE_Pos)                         /*!< SPIA STATUS: TFE Mask                   */
#define SPIA_STATUS_TNF_Pos                   1                                                       /*!< SPIA STATUS: TNF Position               */
#define SPIA_STATUS_TNF_Msk                   (0x01UL << SPIA_STATUS_TNF_Pos)                         /*!< SPIA STATUS: TNF Mask                   */
#define SPIA_STATUS_RNE_Pos                   2                                                       /*!< SPIA STATUS: RNE Position               */
#define SPIA_STATUS_RNE_Msk                   (0x01UL << SPIA_STATUS_RNE_Pos)                         /*!< SPIA STATUS: RNE Mask                   */
#define SPIA_STATUS_RFF_Pos                   3                                                       /*!< SPIA STATUS: RFF Position               */
#define SPIA_STATUS_RFF_Msk                   (0x01UL << SPIA_STATUS_RFF_Pos)                         /*!< SPIA STATUS: RFF Mask                   */
#define SPIA_STATUS_BUSY_Pos                  4                                                       /*!< SPIA STATUS: BUSY Position              */
#define SPIA_STATUS_BUSY_Msk                  (0x01UL << SPIA_STATUS_BUSY_Pos)                        /*!< SPIA STATUS: BUSY Mask                  */
#define SPIA_STATUS_RXDATAFIRST_Pos           5                                                       /*!< SPIA STATUS: RXDATAFIRST Position       */
#define SPIA_STATUS_RXDATAFIRST_Msk           (0x01UL << SPIA_STATUS_RXDATAFIRST_Pos)                 /*!< SPIA STATUS: RXDATAFIRST Mask           */
#define SPIA_STATUS_RXTRIGGER_Pos              6                                                       /*!< SPIA STATUS: RXTRIGGER Position          */
#define SPIA_STATUS_RXTRIGGER_Msk              (0x01UL << SPIA_STATUS_RXTRIGGER_Pos)                    /*!< SPIA STATUS: RXTRIGGER Mask              */
#define SPIA_STATUS_TXTRIGGER_Pos              7                                                       /*!< SPIA STATUS: TXTRIGGER Position          */
#define SPIA_STATUS_TXTRIGGER_Msk              (0x01UL << SPIA_STATUS_TXTRIGGER_Pos)                    /*!< SPIA STATUS: TXTRIGGER Mask              */

/* --------------------------------  SPIA_IRQ_ENB  -------------------------------- */
#define SPIA_IRQ_ENB_RORIM_Pos                0                                                       /*!< SPIA IRQ_ENB: RORIM Position            */
#define SPIA_IRQ_ENB_RORIM_Msk                (0x01UL << SPIA_IRQ_ENB_RORIM_Pos)                      /*!< SPIA IRQ_ENB: RORIM Mask                */
#define SPIA_IRQ_ENB_RTIM_Pos                 1                                                       /*!< SPIA IRQ_ENB: RTIM Position             */
#define SPIA_IRQ_ENB_RTIM_Msk                 (0x01UL << SPIA_IRQ_ENB_RTIM_Pos)                       /*!< SPIA IRQ_ENB: RTIM Mask                 */
#define SPIA_IRQ_ENB_RXIM_Pos                 2                                                       /*!< SPIA IRQ_ENB: RXIM Position             */
#define SPIA_IRQ_ENB_RXIM_Msk                 (0x01UL << SPIA_IRQ_ENB_RXIM_Pos)                       /*!< SPIA IRQ_ENB: RXIM Mask                 */
#define SPIA_IRQ_ENB_TXIM_Pos                 3                                                       /*!< SPIA IRQ_ENB: TXIM Position             */
#define SPIA_IRQ_ENB_TXIM_Msk                 (0x01UL << SPIA_IRQ_ENB_TXIM_Pos)                       /*!< SPIA IRQ_ENB: TXIM Mask                 */

/* --------------------------------  SPIA_IRQ_RAW  -------------------------------- */
#define SPIA_IRQ_RAW_RORIM_Pos                0                                                       /*!< SPIA IRQ_RAW: RORIM Position            */
#define SPIA_IRQ_RAW_RORIM_Msk                (0x01UL << SPIA_IRQ_RAW_RORIM_Pos)                      /*!< SPIA IRQ_RAW: RORIM Mask                */
#define SPIA_IRQ_RAW_RTIM_Pos                 1                                                       /*!< SPIA IRQ_RAW: RTIM Position             */
#define SPIA_IRQ_RAW_RTIM_Msk                 (0x01UL << SPIA_IRQ_RAW_RTIM_Pos)                       /*!< SPIA IRQ_RAW: RTIM Mask                 */
#define SPIA_IRQ_RAW_RXIM_Pos                 2                                                       /*!< SPIA IRQ_RAW: RXIM Position             */
#define SPIA_IRQ_RAW_RXIM_Msk                 (0x01UL << SPIA_IRQ_RAW_RXIM_Pos)                       /*!< SPIA IRQ_RAW: RXIM Mask                 */
#define SPIA_IRQ_RAW_TXIM_Pos                 3                                                       /*!< SPIA IRQ_RAW: TXIM Position             */
#define SPIA_IRQ_RAW_TXIM_Msk                 (0x01UL << SPIA_IRQ_RAW_TXIM_Pos)                       /*!< SPIA IRQ_RAW: TXIM Mask                 */

/* --------------------------------  SPIA_IRQ_END  -------------------------------- */
#define SPIA_IRQ_END_RORIM_Pos                0                                                       /*!< SPIA IRQ_END: RORIM Position            */
#define SPIA_IRQ_END_RORIM_Msk                (0x01UL << SPIA_IRQ_END_RORIM_Pos)                      /*!< SPIA IRQ_END: RORIM Mask                */
#define SPIA_IRQ_END_RTIM_Pos                 1                                                       /*!< SPIA IRQ_END: RTIM Position             */
#define SPIA_IRQ_END_RTIM_Msk                 (0x01UL << SPIA_IRQ_END_RTIM_Pos)                       /*!< SPIA IRQ_END: RTIM Mask                 */
#define SPIA_IRQ_END_RXIM_Pos                 2                                                       /*!< SPIA IRQ_END: RXIM Position             */
#define SPIA_IRQ_END_RXIM_Msk                 (0x01UL << SPIA_IRQ_END_RXIM_Pos)                       /*!< SPIA IRQ_END: RXIM Mask                 */
#define SPIA_IRQ_END_TXIM_Pos                 3                                                       /*!< SPIA IRQ_END: TXIM Position             */
#define SPIA_IRQ_END_TXIM_Msk                 (0x01UL << SPIA_IRQ_END_TXIM_Pos)                       /*!< SPIA IRQ_END: TXIM Mask                 */

/* --------------------------------  SPIA_IRQ_CLR  -------------------------------- */
#define SPIA_IRQ_CLR_RORIM_Pos                0                                                       /*!< SPIA IRQ_CLR: RORIM Position            */
#define SPIA_IRQ_CLR_RORIM_Msk                (0x01UL << SPIA_IRQ_CLR_RORIM_Pos)                      /*!< SPIA IRQ_CLR: RORIM Mask                */
#define SPIA_IRQ_CLR_RTIM_Pos                 1                                                       /*!< SPIA IRQ_CLR: RTIM Position             */
#define SPIA_IRQ_CLR_RTIM_Msk                 (0x01UL << SPIA_IRQ_CLR_RTIM_Pos)                       /*!< SPIA IRQ_CLR: RTIM Mask                 */
#define SPIA_IRQ_CLR_RXIM_Pos                 2                                                       /*!< SPIA IRQ_CLR: RXIM Position             */
#define SPIA_IRQ_CLR_RXIM_Msk                 (0x01UL << SPIA_IRQ_CLR_RXIM_Pos)                       /*!< SPIA IRQ_CLR: RXIM Mask                 */
#define SPIA_IRQ_CLR_TXIM_Pos                 3                                                       /*!< SPIA IRQ_CLR: TXIM Position             */
#define SPIA_IRQ_CLR_TXIM_Msk                 (0x01UL << SPIA_IRQ_CLR_TXIM_Pos)                       /*!< SPIA IRQ_CLR: TXIM Mask                 */

/* --------------------------------  SPIA_FIFO_CLR  ------------------------------- */
#define SPIA_FIFO_CLR_RXFIFO_Pos              0                                                       /*!< SPIA FIFO_CLR: RXFIFO Position          */
#define SPIA_FIFO_CLR_RXFIFO_Msk              (0x01UL << SPIA_FIFO_CLR_RXFIFO_Pos)                    /*!< SPIA FIFO_CLR: RXFIFO Mask              */
#define SPIA_FIFO_CLR_TXFIFO_Pos              1                                                       /*!< SPIA FIFO_CLR: TXFIFO Position          */
#define SPIA_FIFO_CLR_TXFIFO_Msk              (0x01UL << SPIA_FIFO_CLR_TXFIFO_Pos)                    /*!< SPIA FIFO_CLR: TXFIFO Mask              */

/* --------------------------------  SPIA_STATE  ---------------------------------- */
#define SPIA_STATE_RX_STATE_Pos    						0                                                       /*!< SPIA STATE: RX_STATE Position 					 */
#define SPIA_STATE_RX_STATE_Msk    						(0xffUL << SPIA_STATE_RX_STATE_Pos)            			/*!< SPIA STATE: RX_STATE Mask    					 */
#define SPIA_STATE_RX_FIFO_Pos 		 					8                                                  		  /*!< SPIA STATE: RX_FIFO Position 					 */
#define SPIA_STATE_RX_FIFO_Msk 		 					(0xffUL << SPIA_STATE_RX_FIFO_Pos)					  	/*!< SPIA STATE: RX_FIFO Mask 							 */
#define SPIA_STATE_ZERO_Pos 	 		 				16                                                      /*!< SPIA STATE: ZERO Position 							 */
#define SPIA_STATE_ZERO_Msk 	 		 				(0xffUL << SPIA_STATE_ZERO_Pos)            				/*!< SPIA STATE: ZERO Mask 									 */
#define SPIA_STATE_TX_FIFO_Pos     						24                                                      /*!< SPIA STATE: TX_FIFO Position 					 */
#define SPIA_STATE_TX_FIFO_Msk     						(0xffUL << SPIA_STATE_TX_FIFO_Pos)       				/*!< SPIA STATE: TX_FIFO Mask    						 */

/* ================================================================================ */
/* ================          struct 'SPIB' Position & Mask         ================ */
/* ================================================================================ */

/* ----------------------------------  SPIB_DATA  --------------------------- */
#define SPIB_DATA_VALUE_Pos   			 					0                                                      /*!< SPIB DATA: VALUE Position 							*/
#define SPIB_DATA_VALUE_Msk   			 					(0x000000ffffUL << SPIB_DATA_VALUE_Pos)   						 /*!< SPIB DATA: VALUE Mask   								*/
#define SPIB_DATA_BMSKIPDATA_Pos   	 					30                                                     /*!< SPIB DATA: BMSKIPDATA Position 					*/
#define SPIB_DATA_BMSKIPDATA_Msk   	 					(0x01UL << SPIB_DATA_BMSKIPDATA_Pos)         					 /*!< SPIB DATA: BMSKIPDATA Mask   						*/
#define SPIB_DATA_BMSTART_BMSTOP_Pos 					31                                                     /*!< SPIB DATA: BMSTART/BMSTOP Position 			*/
#define SPIB_DATA_BMSTART_BMSTOP_Msk 					(0x01UL << SPIB_DATA_BMSTART_BMSTOP_Pos)     					 /*!< SPIB DATA: BMSTART/BMSTOP Mask   				*/

/* ---------------------------------  SPIB_CTRL0  --------------------------------- */
#define SPIB_CTRL0_SIZE_Pos                   0                                                       /*!< SPIB CTRL0: SIZE Position               */
#define SPIB_CTRL0_SIZE_Msk                   (0x0fUL << SPIB_CTRL0_SIZE_Pos)                         /*!< SPIB CTRL0: SIZE Mask                   */
#define SPIB_CTRL0_SPO_Pos                    6                                                       /*!< SPIB CTRL0: SPO Position                */
#define SPIB_CTRL0_SPO_Msk                    (0x01UL << SPIB_CTRL0_SPO_Pos)                          /*!< SPIB CTRL0: SPO Mask                    */
#define SPIB_CTRL0_SPH_Pos                    7                                                       /*!< SPIB CTRL0: SPH Position                */
#define SPIB_CTRL0_SPH_Msk                    (0x01UL << SPIB_CTRL0_SPH_Pos)                          /*!< SPIB CTRL0: SPH Mask                    */
#define SPIB_CTRL0_SCRDV_Pos                  8                                                       /*!< SPIB CTRL0: SCRDV Position              */
#define SPIB_CTRL0_SCRDV_Msk                  (0x000000ffUL << SPIB_CTRL0_SCRDV_Pos)                  /*!< SPIB CTRL0: SCRDV Mask                  */

/* ---------------------------------  SPIB_CTRL1  --------------------------------- */
#define SPIB_CTRL1_LBM_Pos                    0                                                       /*!< SPIB CTRL1: LBM Position                */
#define SPIB_CTRL1_LBM_Msk                    (0x01UL << SPIB_CTRL1_LBM_Pos)                          /*!< SPIB CTRL1: LBM Mask                    */
#define SPIB_CTRL1_ENABLE_Pos                 1                                                       /*!< SPIB CTRL1: ENABLE Position             */
#define SPIB_CTRL1_ENABLE_Msk                 (0x01UL << SPIB_CTRL1_ENABLE_Pos)                       /*!< SPIB CTRL1: ENABLE Mask                 */
#define SPIB_CTRL1_MS_Pos                     2                                                       /*!< SPIB CTRL1: MS Position                 */
#define SPIB_CTRL1_MS_Msk                     (0x01UL << SPIB_CTRL1_MS_Pos)                           /*!< SPIB CTRL1: MS Mask                     */
#define SPIB_CTRL1_SOD_Pos                    3                                                       /*!< SPIB CTRL1: SOD Position                */
#define SPIB_CTRL1_SOD_Msk                    (0x01UL << SPIB_CTRL1_SOD_Pos)                          /*!< SPIB CTRL1: SOD Mask                    */
#define SPIB_CTRL1_SS_Pos                     4                                                       /*!< SPIB CTRL1: SS Position                 */
#define SPIB_CTRL1_SS_Msk                     (0x07UL << SPIB_CTRL1_SS_Pos)                           /*!< SPIB CTRL1: SS Mask                     */
#define SPIB_CTRL1_BLOCKMODE_Pos              7                                                       /*!< SPIB CTRL1: BLOCKMODE Position           */
#define SPIB_CTRL1_BLOCKMODE_Msk              (0x01UL << SPIB_CTRL1_BLOCKMODE_Pos)                    /*!< SPIB CTRL1: BLOCKMODE Mask               */
//SPIB_CTRL1_BYTEMODE is a typo of "BLOCKMODE" and retained only for backwards compatibility reasons
#define SPIB_CTRL1_BYTEMODE_Pos               SPIB_CTRL1_BLOCKMODE_Pos                                /*!< SPIB CTRL1: BYTEMODE Position           */
#define SPIB_CTRL1_BYTEMODE_Msk               SPIB_CTRL1_BLOCKMODE_Msk                                /*!< SPIB CTRL1: BYTEMODE Mask               */
#define SPIB_CTRL1_BMSTART_Pos                8                                                       /*!< SPIB CTRL1: BMSTART Position            */
#define SPIB_CTRL1_BMSTART_Msk                (0x01UL << SPIB_CTRL1_BMSTART_Pos)                      /*!< SPIB CTRL1: BMSTART Mask                */
#define SPIB_CTRL1_BMSTALL_Pos                9                                                       /*!< SPIB CTRL1: BMSTALL Position            */
#define SPIB_CTRL1_BMSTALL_Msk                (0x01UL << SPIB_CTRL1_BMSTALL_Pos)                      /*!< SPIB CTRL1: BMSTALL Mask                */
#define SPIB_CTRL1_MDLYCAP_Pos                10                                                      /*!< SPIB CTRL1: MDLYCAP Position            */
#define SPIB_CTRL1_MDLYCAP_Msk                (0x01UL << SPIB_CTRL1_MDLYCAP_Pos)                      /*!< SPIB CTRL1: MDLYCAP Mask                */
#define SPIB_CTRL1_MTXPAUSE_Pos               11                                                      /*!< SPIB CTRL1: MTXPAUSE Position           */
#define SPIB_CTRL1_MTXPAUSE_Msk               (0x01UL << SPIB_CTRL1_MTXPAUSE_Pos)                     /*!< SPIB CTRL1: MTXPAUSE Mask               */

/* ---------------------------------  SPIB_STATUS  -------------------------------- */
#define SPIB_STATUS_TFE_Pos                   0                                                       /*!< SPIB STATUS: TFE Position               */
#define SPIB_STATUS_TFE_Msk                   (0x01UL << SPIB_STATUS_TFE_Pos)                         /*!< SPIB STATUS: TFE Mask                   */
#define SPIB_STATUS_TNF_Pos                   1                                                       /*!< SPIB STATUS: TNF Position               */
#define SPIB_STATUS_TNF_Msk                   (0x01UL << SPIB_STATUS_TNF_Pos)                         /*!< SPIB STATUS: TNF Mask                   */
#define SPIB_STATUS_RNE_Pos                   2                                                       /*!< SPIB STATUS: RNE Position               */
#define SPIB_STATUS_RNE_Msk                   (0x01UL << SPIB_STATUS_RNE_Pos)                         /*!< SPIB STATUS: RNE Mask                   */
#define SPIB_STATUS_RFF_Pos                   3                                                       /*!< SPIB STATUS: RFF Position               */
#define SPIB_STATUS_RFF_Msk                   (0x01UL << SPIB_STATUS_RFF_Pos)                         /*!< SPIB STATUS: RFF Mask                   */
#define SPIB_STATUS_BUSY_Pos                  4                                                       /*!< SPIB STATUS: BUSY Position              */
#define SPIB_STATUS_BUSY_Msk                  (0x01UL << SPIB_STATUS_BUSY_Pos)                        /*!< SPIB STATUS: BUSY Mask                  */
#define SPIB_STATUS_RXDATAFIRST_Pos           5                                                       /*!< SPIB STATUS: RXDATAFIRST Position       */
#define SPIB_STATUS_RXDATAFIRST_Msk           (0x01UL << SPIB_STATUS_RXDATAFIRST_Pos)                 /*!< SPIB STATUS: RXDATAFIRST Mask           */
#define SPIB_STATUS_RXTRIGGER_Pos              6                                                       /*!< SPIB STATUS: RXTRIGGER Position          */
#define SPIB_STATUS_RXTRIGGER_Msk              (0x01UL << SPIB_STATUS_RXTRIGGER_Pos)                    /*!< SPIB STATUS: RXTRIGGER Mask              */
#define SPIB_STATUS_TXTRIGGER_Pos              7                                                       /*!< SPIB STATUS: TXTRIGGER Position          */
#define SPIB_STATUS_TXTRIGGER_Msk              (0x01UL << SPIB_STATUS_TXTRIGGER_Pos)                    /*!< SPIB STATUS: TXTRIGGER Mask              */

/* --------------------------------  SPIB_IRQ_ENB  -------------------------------- */
#define SPIB_IRQ_ENB_RORIM_Pos                0                                                       /*!< SPIB IRQ_ENB: RORIM Position            */
#define SPIB_IRQ_ENB_RORIM_Msk                (0x01UL << SPIB_IRQ_ENB_RORIM_Pos)                      /*!< SPIB IRQ_ENB: RORIM Mask                */
#define SPIB_IRQ_ENB_RTIM_Pos                 1                                                       /*!< SPIB IRQ_ENB: RTIM Position             */
#define SPIB_IRQ_ENB_RTIM_Msk                 (0x01UL << SPIB_IRQ_ENB_RTIM_Pos)                       /*!< SPIB IRQ_ENB: RTIM Mask                 */
#define SPIB_IRQ_ENB_RXIM_Pos                 2                                                       /*!< SPIB IRQ_ENB: RXIM Position             */
#define SPIB_IRQ_ENB_RXIM_Msk                 (0x01UL << SPIB_IRQ_ENB_RXIM_Pos)                       /*!< SPIB IRQ_ENB: RXIM Mask                 */
#define SPIB_IRQ_ENB_TXIM_Pos                 3                                                       /*!< SPIB IRQ_ENB: TXIM Position             */
#define SPIB_IRQ_ENB_TXIM_Msk                 (0x01UL << SPIB_IRQ_ENB_TXIM_Pos)                       /*!< SPIB IRQ_ENB: TXIM Mask                 */

/* --------------------------------  SPIB_IRQ_RAW  -------------------------------- */
#define SPIB_IRQ_RAW_RORIM_Pos                0                                                       /*!< SPIB IRQ_RAW: RORIM Position            */
#define SPIB_IRQ_RAW_RORIM_Msk                (0x01UL << SPIB_IRQ_RAW_RORIM_Pos)                      /*!< SPIB IRQ_RAW: RORIM Mask                */
#define SPIB_IRQ_RAW_RTIM_Pos                 1                                                       /*!< SPIB IRQ_RAW: RTIM Position             */
#define SPIB_IRQ_RAW_RTIM_Msk                 (0x01UL << SPIB_IRQ_RAW_RTIM_Pos)                       /*!< SPIB IRQ_RAW: RTIM Mask                 */
#define SPIB_IRQ_RAW_RXIM_Pos                 2                                                       /*!< SPIB IRQ_RAW: RXIM Position             */
#define SPIB_IRQ_RAW_RXIM_Msk                 (0x01UL << SPIB_IRQ_RAW_RXIM_Pos)                       /*!< SPIB IRQ_RAW: RXIM Mask                 */
#define SPIB_IRQ_RAW_TXIM_Pos                 3                                                       /*!< SPIB IRQ_RAW: TXIM Position             */
#define SPIB_IRQ_RAW_TXIM_Msk                 (0x01UL << SPIB_IRQ_RAW_TXIM_Pos)                       /*!< SPIB IRQ_RAW: TXIM Mask                 */

/* --------------------------------  SPIB_IRQ_END  -------------------------------- */
#define SPIB_IRQ_END_RORIM_Pos                0                                                       /*!< SPIB IRQ_END: RORIM Position            */
#define SPIB_IRQ_END_RORIM_Msk                (0x01UL << SPIB_IRQ_END_RORIM_Pos)                      /*!< SPIB IRQ_END: RORIM Mask                */
#define SPIB_IRQ_END_RTIM_Pos                 1                                                       /*!< SPIB IRQ_END: RTIM Position             */
#define SPIB_IRQ_END_RTIM_Msk                 (0x01UL << SPIB_IRQ_END_RTIM_Pos)                       /*!< SPIB IRQ_END: RTIM Mask                 */
#define SPIB_IRQ_END_RXIM_Pos                 2                                                       /*!< SPIB IRQ_END: RXIM Position             */
#define SPIB_IRQ_END_RXIM_Msk                 (0x01UL << SPIB_IRQ_END_RXIM_Pos)                       /*!< SPIB IRQ_END: RXIM Mask                 */
#define SPIB_IRQ_END_TXIM_Pos                 3                                                       /*!< SPIB IRQ_END: TXIM Position             */
#define SPIB_IRQ_END_TXIM_Msk                 (0x01UL << SPIB_IRQ_END_TXIM_Pos)                       /*!< SPIB IRQ_END: TXIM Mask                 */

/* --------------------------------  SPIB_IRQ_CLR  -------------------------------- */
#define SPIB_IRQ_CLR_RORIM_Pos                0                                                       /*!< SPIB IRQ_CLR: RORIM Position            */
#define SPIB_IRQ_CLR_RORIM_Msk                (0x01UL << SPIB_IRQ_CLR_RORIM_Pos)                      /*!< SPIB IRQ_CLR: RORIM Mask                */
#define SPIB_IRQ_CLR_RTIM_Pos                 1                                                       /*!< SPIB IRQ_CLR: RTIM Position             */
#define SPIB_IRQ_CLR_RTIM_Msk                 (0x01UL << SPIB_IRQ_CLR_RTIM_Pos)                       /*!< SPIB IRQ_CLR: RTIM Mask                 */
#define SPIB_IRQ_CLR_RXIM_Pos                 2                                                       /*!< SPIB IRQ_CLR: RXIM Position             */
#define SPIB_IRQ_CLR_RXIM_Msk                 (0x01UL << SPIB_IRQ_CLR_RXIM_Pos)                       /*!< SPIB IRQ_CLR: RXIM Mask                 */
#define SPIB_IRQ_CLR_TXIM_Pos                 3                                                       /*!< SPIB IRQ_CLR: TXIM Position             */
#define SPIB_IRQ_CLR_TXIM_Msk                 (0x01UL << SPIB_IRQ_CLR_TXIM_Pos)                       /*!< SPIB IRQ_CLR: TXIM Mask                 */

/* --------------------------------  SPIB_FIFO_CLR  ------------------------------- */
#define SPIB_FIFO_CLR_RXFIFO_Pos              0                                                       /*!< SPIB FIFO_CLR: RXFIFO Position          */
#define SPIB_FIFO_CLR_RXFIFO_Msk              (0x01UL << SPIB_FIFO_CLR_RXFIFO_Pos)                    /*!< SPIB FIFO_CLR: RXFIFO Mask              */
#define SPIB_FIFO_CLR_TXFIFO_Pos              1                                                       /*!< SPIB FIFO_CLR: TXFIFO Position          */
#define SPIB_FIFO_CLR_TXFIFO_Msk              (0x01UL << SPIB_FIFO_CLR_TXFIFO_Pos)                    /*!< SPIB FIFO_CLR: TXFIFO Mask              */

/* --------------------------------  SPIB_STATE  ---------------------------------- */
#define SPIB_STATE_RX_STATE_Pos    						0                                                       /*!< SPIB STATE: RX_STATE Position 					 */
#define SPIB_STATE_RX_STATE_Msk    						(0xffUL << SPIB_STATE_RX_STATE_Pos)            			/*!< SPIB STATE: RX_STATE Mask    					 */
#define SPIB_STATE_RX_FIFO_Pos 		 					8                                                  		/*!< SPIB STATE: RX_FIFO Position 					 */
#define SPIB_STATE_RX_FIFO_Msk 		 					(0xffUL << SPIB_STATE_RX_FIFO_Pos)					  	/*!< SPIB STATE: RX_FIFO Mask 							 */
#define SPIB_STATE_ZERO_Pos 	 		 				16                                                      /*!< SPIB STATE: ZERO Position 							 */
#define SPIB_STATE_ZERO_Msk 	 		 				(0xffUL << SPIB_STATE_ZERO_Pos)            				/*!< SPIB STATE: ZERO Mask 									 */
#define SPIB_STATE_TX_FIFO_Pos     						24                                                      /*!< SPIB STATE: TX_FIFO Position 					 */
#define SPIB_STATE_TX_FIFO_Msk     						(0xffUL << SPIB_STATE_TX_FIFO_Pos)       				/*!< SPIB STATE: TX_FIFO Mask    						 */


/* ================================================================================ */
/* ================          struct 'SPIC' Position & Mask         ================ */
/* ================================================================================ */

/* ----------------------------------  SPIC_DATA  --------------------------- */
#define SPIC_DATA_VALUE_Pos   			 				0                                                      /*!< SPIC DATA: VALUE Position 					*/
#define SPIC_DATA_VALUE_Msk   			 				(0x000000ffffUL << SPIC_DATA_VALUE_Pos)   						 /*!< SPIC DATA: VALUE Mask   						*/
#define SPIC_DATA_BMSKIPDATA_Pos   	 					30                                                     /*!< SPIC DATA: BMSKIPDATA Position 		*/
#define SPIC_DATA_BMSKIPDATA_Msk   	 					(0x01UL << SPIC_DATA_BMSKIPDATA_Pos)         					 /*!< SPIC DATA: BMSKIPDATA Mask   			*/
#define SPIC_DATA_BMSTART_BMSTOP_Pos 					31                                                     /*!< SPIC DATA: BMSTART/BMSTOP Position */
#define SPIC_DATA_BMSTART_BMSTOP_Msk 					(0x01UL << SPIC_DATA_BMSTART_BMSTOP_Pos)     					 /*!< SPIC DATA: BMSTART/BMSTOP Mask   	*/


/* ---------------------------------  SPIC_CTRL0  --------------------------------- */
#define SPIC_CTRL0_SIZE_Pos                   0                                                       /*!< SPIC CTRL0: SIZE Position               */
#define SPIC_CTRL0_SIZE_Msk                   (0x0fUL << SPIC_CTRL0_SIZE_Pos)                         /*!< SPIC CTRL0: SIZE Mask                   */
#define SPIC_CTRL0_SPO_Pos                    6                                                       /*!< SPIC CTRL0: SPO Position                */
#define SPIC_CTRL0_SPO_Msk                    (0x01UL << SPIC_CTRL0_SPO_Pos)                          /*!< SPIC CTRL0: SPO Mask                    */
#define SPIC_CTRL0_SPH_Pos                    7                                                       /*!< SPIC CTRL0: SPH Position                */
#define SPIC_CTRL0_SPH_Msk                    (0x01UL << SPIC_CTRL0_SPH_Pos)                          /*!< SPIC CTRL0: SPH Mask                    */
#define SPIC_CTRL0_SCRDV_Pos                  8                                                       /*!< SPIC CTRL0: SCRDV Position              */
#define SPIC_CTRL0_SCRDV_Msk                  (0x000000ffUL << SPIC_CTRL0_SCRDV_Pos)                  /*!< SPIC CTRL0: SCRDV Mask                  */

/* ---------------------------------  SPIC_CTRL1  --------------------------------- */
#define SPIC_CTRL1_LBM_Pos                    0                                                       /*!< SPIC CTRL1: LBM Position                */
#define SPIC_CTRL1_LBM_Msk                    (0x01UL << SPIC_CTRL1_LBM_Pos)                          /*!< SPIC CTRL1: LBM Mask                    */
#define SPIC_CTRL1_ENABLE_Pos                 1                                                       /*!< SPIC CTRL1: ENABLE Position             */
#define SPIC_CTRL1_ENABLE_Msk                 (0x01UL << SPIC_CTRL1_ENABLE_Pos)                       /*!< SPIC CTRL1: ENABLE Mask                 */
#define SPIC_CTRL1_MS_Pos                     2                                                       /*!< SPIC CTRL1: MS Position                 */
#define SPIC_CTRL1_MS_Msk                     (0x01UL << SPIC_CTRL1_MS_Pos)                           /*!< SPIC CTRL1: MS Mask                     */
#define SPIC_CTRL1_SOD_Pos                    3                                                       /*!< SPIC CTRL1: SOD Position                */
#define SPIC_CTRL1_SOD_Msk                    (0x01UL << SPIC_CTRL1_SOD_Pos)                          /*!< SPIC CTRL1: SOD Mask                    */
#define SPIC_CTRL1_SS_Pos                     4                                                       /*!< SPIC CTRL1: SS Position                 */
#define SPIC_CTRL1_SS_Msk                     (0x07UL << SPIC_CTRL1_SS_Pos)                           /*!< SPIC CTRL1: SS Mask                     */
#define SPIC_CTRL1_BLOCKMODE_Pos              7                                                       /*!< SPIC CTRL1: BLOCKMODE Position          */
#define SPIC_CTRL1_BLOCKMODE_Msk              (0x01UL << SPIC_CTRL1_BLOCKMODE_Pos)                    /*!< SPIC CTRL1: BLOCKMODE Mask              */
//SPIC_CTRL1_BYTEMODE is a typo of "BLOCKMODE" and retained only for backwards compatibility reasons
#define SPIC_CTRL1_BYTEMODE_Pos               SPIC_CTRL1_BLOCKMODE_Pos                                /*!< SPIC CTRL1: BYTEMODE Position           */
#define SPIC_CTRL1_BYTEMODE_Msk               SPIC_CTRL1_BLOCKMODE_Msk                                /*!< SPIC CTRL1: BYTEMODE Mask               */
#define SPIC_CTRL1_BMSTART_Pos                8                                                       /*!< SPIC CTRL1: BMSTART Position            */
#define SPIC_CTRL1_BMSTART_Msk                (0x01UL << SPIC_CTRL1_BMSTART_Pos)                      /*!< SPIC CTRL1: BMSTART Mask                */
#define SPIC_CTRL1_BMSTALL_Pos                9                                                       /*!< SPIC CTRL1: BMSTALL Position            */
#define SPIC_CTRL1_BMSTALL_Msk                (0x01UL << SPIC_CTRL1_BMSTALL_Pos)                      /*!< SPIC CTRL1: BMSTALL Mask                */
#define SPIC_CTRL1_MDLYCAP_Pos                10                                                      /*!< SPIC CTRL1: MDLYCAP Position            */
#define SPIC_CTRL1_MDLYCAP_Msk                (0x01UL << SPIC_CTRL1_MDLYCAP_Pos)                      /*!< SPIC CTRL1: MDLYCAP Mask                */
#define SPIC_CTRL1_MTXPAUSE_Pos               11                                                      /*!< SPIC CTRL1: MTXPAUSE Position           */
#define SPIC_CTRL1_MTXPAUSE_Msk               (0x01UL << SPIC_CTRL1_MTXPAUSE_Pos)                     /*!< SPIC CTRL1: MTXPAUSE Mask               */

/* ---------------------------------  SPIC_STATUS  -------------------------------- */
#define SPIC_STATUS_TFE_Pos                   0                                                       /*!< SPIC STATUS: TFE Position               */
#define SPIC_STATUS_TFE_Msk                   (0x01UL << SPIC_STATUS_TFE_Pos)                         /*!< SPIC STATUS: TFE Mask                   */
#define SPIC_STATUS_TNF_Pos                   1                                                       /*!< SPIC STATUS: TNF Position               */
#define SPIC_STATUS_TNF_Msk                   (0x01UL << SPIC_STATUS_TNF_Pos)                         /*!< SPIC STATUS: TNF Mask                   */
#define SPIC_STATUS_RNE_Pos                   2                                                       /*!< SPIC STATUS: RNE Position               */
#define SPIC_STATUS_RNE_Msk                   (0x01UL << SPIC_STATUS_RNE_Pos)                         /*!< SPIC STATUS: RNE Mask                   */
#define SPIC_STATUS_RFF_Pos                   3                                                       /*!< SPIC STATUS: RFF Position               */
#define SPIC_STATUS_RFF_Msk                   (0x01UL << SPIC_STATUS_RFF_Pos)                         /*!< SPIC STATUS: RFF Mask                   */
#define SPIC_STATUS_BUSY_Pos                  4                                                       /*!< SPIC STATUS: BUSY Position              */
#define SPIC_STATUS_BUSY_Msk                  (0x01UL << SPIC_STATUS_BUSY_Pos)                        /*!< SPIC STATUS: BUSY Mask                  */
#define SPIC_STATUS_RXDATAFIRST_Pos           5                                                       /*!< SPIC STATUS: RXDATAFIRST Position       */
#define SPIC_STATUS_RXDATAFIRST_Msk           (0x01UL << SPIC_STATUS_RXDATAFIRST_Pos)                 /*!< SPIC STATUS: RXDATAFIRST Mask           */
#define SPIC_STATUS_RXTRIGGER_Pos              6                                                       /*!< SPIC STATUS: RXTRIGGER Position          */
#define SPIC_STATUS_RXTRIGGER_Msk              (0x01UL << SPIC_STATUS_RXTRIGGER_Pos)                    /*!< SPIC STATUS: RXTRIGGER Mask              */
#define SPIC_STATUS_TXTRIGGER_Pos              7                                                       /*!< SPIC STATUS: TXTRIGGER Position          */
#define SPIC_STATUS_TXTRIGGER_Msk              (0x01UL << SPIC_STATUS_TXTRIGGER_Pos)                    /*!< SPIC STATUS: TXTRIGGER Mask              */

/* --------------------------------  SPIC_IRQ_ENB  -------------------------------- */
#define SPIC_IRQ_ENB_RORIM_Pos                0                                                       /*!< SPIC IRQ_ENB: RORIM Position            */
#define SPIC_IRQ_ENB_RORIM_Msk                (0x01UL << SPIC_IRQ_ENB_RORIM_Pos)                      /*!< SPIC IRQ_ENB: RORIM Mask                */
#define SPIC_IRQ_ENB_RTIM_Pos                 1                                                       /*!< SPIC IRQ_ENB: RTIM Position             */
#define SPIC_IRQ_ENB_RTIM_Msk                 (0x01UL << SPIC_IRQ_ENB_RTIM_Pos)                       /*!< SPIC IRQ_ENB: RTIM Mask                 */
#define SPIC_IRQ_ENB_RXIM_Pos                 2                                                       /*!< SPIC IRQ_ENB: RXIM Position             */
#define SPIC_IRQ_ENB_RXIM_Msk                 (0x01UL << SPIC_IRQ_ENB_RXIM_Pos)                       /*!< SPIC IRQ_ENB: RXIM Mask                 */
#define SPIC_IRQ_ENB_TXIM_Pos                 3                                                       /*!< SPIC IRQ_ENB: TXIM Position             */
#define SPIC_IRQ_ENB_TXIM_Msk                 (0x01UL << SPIC_IRQ_ENB_TXIM_Pos)                       /*!< SPIC IRQ_ENB: TXIM Mask                 */

/* --------------------------------  SPIC_IRQ_RAW  -------------------------------- */
#define SPIC_IRQ_RAW_RORIM_Pos                0                                                       /*!< SPIC IRQ_RAW: RORIM Position            */
#define SPIC_IRQ_RAW_RORIM_Msk                (0x01UL << SPIC_IRQ_RAW_RORIM_Pos)                      /*!< SPIC IRQ_RAW: RORIM Mask                */
#define SPIC_IRQ_RAW_RTIM_Pos                 1                                                       /*!< SPIC IRQ_RAW: RTIM Position             */
#define SPIC_IRQ_RAW_RTIM_Msk                 (0x01UL << SPIC_IRQ_RAW_RTIM_Pos)                       /*!< SPIC IRQ_RAW: RTIM Mask                 */
#define SPIC_IRQ_RAW_RXIM_Pos                 2                                                       /*!< SPIC IRQ_RAW: RXIM Position             */
#define SPIC_IRQ_RAW_RXIM_Msk                 (0x01UL << SPIC_IRQ_RAW_RXIM_Pos)                       /*!< SPIC IRQ_RAW: RXIM Mask                 */
#define SPIC_IRQ_RAW_TXIM_Pos                 3                                                       /*!< SPIC IRQ_RAW: TXIM Position             */
#define SPIC_IRQ_RAW_TXIM_Msk                 (0x01UL << SPIC_IRQ_RAW_TXIM_Pos)                       /*!< SPIC IRQ_RAW: TXIM Mask                 */

/* --------------------------------  SPIC_IRQ_END  -------------------------------- */
#define SPIC_IRQ_END_RORIM_Pos                0                                                       /*!< SPIC IRQ_END: RORIM Position            */
#define SPIC_IRQ_END_RORIM_Msk                (0x01UL << SPIC_IRQ_END_RORIM_Pos)                      /*!< SPIC IRQ_END: RORIM Mask                */
#define SPIC_IRQ_END_RTIM_Pos                 1                                                       /*!< SPIC IRQ_END: RTIM Position             */
#define SPIC_IRQ_END_RTIM_Msk                 (0x01UL << SPIC_IRQ_END_RTIM_Pos)                       /*!< SPIC IRQ_END: RTIM Mask                 */
#define SPIC_IRQ_END_RXIM_Pos                 2                                                       /*!< SPIC IRQ_END: RXIM Position             */
#define SPIC_IRQ_END_RXIM_Msk                 (0x01UL << SPIC_IRQ_END_RXIM_Pos)                       /*!< SPIC IRQ_END: RXIM Mask                 */
#define SPIC_IRQ_END_TXIM_Pos                 3                                                       /*!< SPIC IRQ_END: TXIM Position             */
#define SPIC_IRQ_END_TXIM_Msk                 (0x01UL << SPIC_IRQ_END_TXIM_Pos)                       /*!< SPIC IRQ_END: TXIM Mask                 */

/* --------------------------------  SPIC_IRQ_CLR  -------------------------------- */
#define SPIC_IRQ_CLR_RORIM_Pos                0                                                       /*!< SPIC IRQ_CLR: RORIM Position            */
#define SPIC_IRQ_CLR_RORIM_Msk                (0x01UL << SPIC_IRQ_CLR_RORIM_Pos)                      /*!< SPIC IRQ_CLR: RORIM Mask                */
#define SPIC_IRQ_CLR_RTIM_Pos                 1                                                       /*!< SPIC IRQ_CLR: RTIM Position             */
#define SPIC_IRQ_CLR_RTIM_Msk                 (0x01UL << SPIC_IRQ_CLR_RTIM_Pos)                       /*!< SPIC IRQ_CLR: RTIM Mask                 */
#define SPIC_IRQ_CLR_RXIM_Pos                 2                                                       /*!< SPIC IRQ_CLR: RXIM Position             */
#define SPIC_IRQ_CLR_RXIM_Msk                 (0x01UL << SPIC_IRQ_CLR_RXIM_Pos)                       /*!< SPIC IRQ_CLR: RXIM Mask                 */
#define SPIC_IRQ_CLR_TXIM_Pos                 3                                                       /*!< SPIC IRQ_CLR: TXIM Position             */
#define SPIC_IRQ_CLR_TXIM_Msk                 (0x01UL << SPIC_IRQ_CLR_TXIM_Pos)                       /*!< SPIC IRQ_CLR: TXIM Mask                 */

/* --------------------------------  SPIC_FIFO_CLR  ------------------------------- */
#define SPIC_FIFO_CLR_RXFIFO_Pos              0                                                       /*!< SPIC FIFO_CLR: RXFIFO Position          */
#define SPIC_FIFO_CLR_RXFIFO_Msk              (0x01UL << SPIC_FIFO_CLR_RXFIFO_Pos)                    /*!< SPIC FIFO_CLR: RXFIFO Mask              */
#define SPIC_FIFO_CLR_TXFIFO_Pos              1                                                       /*!< SPIC FIFO_CLR: TXFIFO Position          */
#define SPIC_FIFO_CLR_TXFIFO_Msk              (0x01UL << SPIC_FIFO_CLR_TXFIFO_Pos)                    /*!< SPIC FIFO_CLR: TXFIFO Mask              */

/* --------------------------------  SPIC_STATE  ---------------------------------- */
#define SPIC_STATE_RX_STATE_Pos    						0                                                       /*!< SPIC STATE: RX_STATE Position 					 */
#define SPIC_STATE_RX_STATE_Msk    						(0xffUL << SPIC_STATE_RX_STATE_Pos)            			/*!< SPIC STATE: RX_STATE Mask    					 */
#define SPIC_STATE_RX_FIFO_Pos 		 					8                                                  		  /*!< SPIC STATE: RX_FIFO Position 					 */
#define SPIC_STATE_RX_FIFO_Msk 		 					(0xffUL << SPIC_STATE_RX_FIFO_Pos)					  	/*!< SPIC STATE: RX_FIFO Mask 							 */
#define SPIC_STATE_ZERO_Pos 	 		 				16                                                      /*!< SPIC STATE: ZERO Position 							 */
#define SPIC_STATE_ZERO_Msk 	 		 				(0xffUL << SPIC_STATE_ZERO_Pos)            				/*!< SPIC STATE: ZERO Mask 									 */
#define SPIC_STATE_TX_FIFO_Pos     						24                                                      /*!< SPIC STATE: TX_FIFO Position 					 */
#define SPIC_STATE_TX_FIFO_Msk     						(0xffUL << SPIC_STATE_TX_FIFO_Pos)       				/*!< SPIC STATE: TX_FIFO Mask    						 */


/* ================================================================================ */
/* ================     Group 'I2C_PERIPHERAL' Position & Mask     ================ */
/* ================================================================================ */


/* -----------------------------  I2C_PERIPHERAL_CTRL  ---------------------------- */
#define I2C_PERIPHERAL_CTRL_CLKENABLED_Pos    0                                                       /*!< I2C_PERIPHERAL CTRL: CLKENABLED Position   */
#define I2C_PERIPHERAL_CTRL_CLKENABLED_Msk    (0x01UL << I2C_PERIPHERAL_CTRL_CLKENABLED_Pos)          /*!< I2C_PERIPHERAL CTRL: CLKENABLED Mask       */
#define I2C_PERIPHERAL_CTRL_ENABLED_Pos     	1                                                       /*!< I2C_PERIPHERAL CTRL: ENABLED Position */
#define I2C_PERIPHERAL_CTRL_ENABLED_Msk     	(0x01UL << I2C_PERIPHERAL_CTRL_ENABLED_Pos)           	/*!< I2C_PERIPHERAL CTRL: ENABLED Mask     */
#define I2C_PERIPHERAL_CTRL_ENABLE_Pos        2                                                       /*!< I2C_PERIPHERAL CTRL: ENABLE Position    */
#define I2C_PERIPHERAL_CTRL_ENABLE_Msk        (0x01UL << I2C_PERIPHERAL_CTRL_ENABLE_Pos)              /*!< I2C_PERIPHERAL CTRL: ENABLE Mask        */
#define I2C_PERIPHERAL_CTRL_TXFEMD_Pos        3                                                       /*!< I2C_PERIPHERAL CTRL: TXFEMD Position    */
#define I2C_PERIPHERAL_CTRL_TXFEMD_Msk        (0x01UL << I2C_PERIPHERAL_CTRL_TXFEMD_Pos)              /*!< I2C_PERIPHERAL CTRL: TXFEMD Mask        */
#define I2C_PERIPHERAL_CTRL_RXFFMD_Pos        4                                                       /*!< I2C_PERIPHERAL CTRL: RXFFMD Position    */
#define I2C_PERIPHERAL_CTRL_RXFFMD_Msk        (0x01UL << I2C_PERIPHERAL_CTRL_RXFFMD_Pos)              /*!< I2C_PERIPHERAL CTRL: RXFFMD Mask        */
#define I2C_PERIPHERAL_CTRL_ALGFILTER_Pos     5                                                       /*!< I2C_PERIPHERAL CTRL: ALGFILTER Position */
#define I2C_PERIPHERAL_CTRL_ALGFILTER_Msk     (0x01UL << I2C_PERIPHERAL_CTRL_ALGFILTER_Pos)           /*!< I2C_PERIPHERAL CTRL: ALGFILTER Mask     */
#define I2C_PERIPHERAL_CTRL_DLGFILTER_Pos     6                                                       /*!< I2C_PERIPHERAL CTRL: DLGFILTER Position */
#define I2C_PERIPHERAL_CTRL_DLGFILTER_Msk     (0x01UL << I2C_PERIPHERAL_CTRL_DLGFILTER_Pos)           /*!< I2C_PERIPHERAL CTRL: DLGFILTER Mask     */
#define I2C_PERIPHERAL_CTRL_LOOPBACK_Pos      8                                                       /*!< I2C_PERIPHERAL CTRL: LOOPBACK Position  */
#define I2C_PERIPHERAL_CTRL_LOOPBACK_Msk      (0x01UL << I2C_PERIPHERAL_CTRL_LOOPBACK_Pos)            /*!< I2C_PERIPHERAL CTRL: LOOPBACK Mask      */
#define I2C_PERIPHERAL_CTRL_TMCONFIGENB_Pos   9                                                       /*!< I2C_PERIPHERAL CTRL: TMCONFIGENB Position */
#define I2C_PERIPHERAL_CTRL_TMCONFIGENB_Msk   (0x01UL << I2C_PERIPHERAL_CTRL_TMCONFIGENB_Pos)         /*!< I2C_PERIPHERAL CTRL: TMCONFIGENB Mask   */

/* ---------------------------  I2C_PERIPHERAL_CLKSCALE  -------------------------- */
#define I2C_PERIPHERAL_CLKSCALE_VALUE_Pos     0                                                       /*!< I2C_PERIPHERAL CLKSCALE: VALUE Position */
#define I2C_PERIPHERAL_CLKSCALE_VALUE_Msk     (0x7fffffffUL << I2C_PERIPHERAL_CLKSCALE_VALUE_Pos)     /*!< I2C_PERIPHERAL CLKSCALE: VALUE Mask     */
#define I2C_PERIPHERAL_CLKSCALE_FASTMODE_Pos  31                                                      /*!< I2C_PERIPHERAL CLKSCALE: FASTMODE Position */
#define I2C_PERIPHERAL_CLKSCALE_FASTMODE_Msk  (0x01UL << I2C_PERIPHERAL_CLKSCALE_FASTMODE_Pos)        /*!< I2C_PERIPHERAL CLKSCALE: FASTMODE Mask  */

/* ---------------------------  I2C_PERIPHERAL_ADDRESS  -------------------------- */
#define I2C_PERIPHERAL_ADDRESS_DIRECTION_Pos  0                                                       /*!< I2C_PERIPHERAL ADDRESS: DIRECTION Position */
#define I2C_PERIPHERAL_ADDRESS_DIRECTION_Msk  (0x01UL << I2C_PERIPHERAL_ADDRESS_DIRECTION_Pos)     		/*!< I2C_PERIPHERAL ADDRESS: DIRECTION Mask     */
#define I2C_PERIPHERAL_ADDRESS_ADDRESS_Pos  	1	                                                      /*!< I2C_PERIPHERAL ADDRESS: ADDRESS Position */
#define I2C_PERIPHERAL_ADDRESS_ADDRESS_Msk  	(0x3FFUL << I2C_PERIPHERAL_ADDRESS_ADDRESS_Pos)        	/*!< I2C_PERIPHERAL ADDRESS: ADDRESS Mask  */
#define I2C_PERIPHERAL_ADDRESS_A10MODE_Pos  	15                                                      /*!< I2C_PERIPHERAL ADDRESS: A10MODE Position */
#define I2C_PERIPHERAL_ADDRESS_A10MODE_Msk  	(0x01UL << I2C_PERIPHERAL_ADDRESS_A10MODE_Pos)        	/*!< I2C_PERIPHERAL ADDRESS: A10MODE Mask  */

/* ---------------------------  I2C_PERIPHERAL_CMD  ------------------------------ */
#define I2C_PERIPHERAL_CMD_START_Pos  				0                                                       /*!< I2C_PERIPHERAL CMD: START Position */
#define I2C_PERIPHERAL_CMD_START_Msk  				(0x01UL << I2C_PERIPHERAL_CMD_START_Pos)     						/*!< I2C_PERIPHERAL CMD: START Mask     */
#define I2C_PERIPHERAL_CMD_STOP_Pos  				1	                                                      /*!< I2C_PERIPHERAL CMD: STOP Position */
#define I2C_PERIPHERAL_CMD_STOP_Msk  				(0x01UL << I2C_PERIPHERAL_CMD_STOP_Pos)        				/*!< I2C_PERIPHERAL CMD: STOP Mask  */
#define I2C_PERIPHERAL_CMD_CANCEL_Pos  				2                                                      /*!< I2C_PERIPHERAL CMD: CANCEL Position */
#define I2C_PERIPHERAL_CMD_CANCEL_Msk  				(0x01UL << I2C_PERIPHERAL_CMD_CANCEL_Pos)        				/*!< I2C_PERIPHERAL CMD: CANCEL Mask  */

/* ----------------------------  I2C_PERIPHERAL_STATUS  --------------------------- */
#define I2C_PERIPHERAL_STATUS_I2CIDLE_Pos     0                                                       /*!< I2C_PERIPHERAL STATUS: I2CIDLE Position */
#define I2C_PERIPHERAL_STATUS_I2CIDLE_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_I2CIDLE_Pos)           /*!< I2C_PERIPHERAL STATUS: I2CIDLE Mask     */
#define I2C_PERIPHERAL_STATUS_IDLE_Pos        1                                                       /*!< I2C_PERIPHERAL STATUS: IDLE Position    */
#define I2C_PERIPHERAL_STATUS_IDLE_Msk        (0x01UL << I2C_PERIPHERAL_STATUS_IDLE_Pos)              /*!< I2C_PERIPHERAL STATUS: IDLE Mask        */
#define I2C_PERIPHERAL_STATUS_WAITING_Pos     2                                                       /*!< I2C_PERIPHERAL STATUS: WAITING Position */
#define I2C_PERIPHERAL_STATUS_WAITING_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_WAITING_Pos)           /*!< I2C_PERIPHERAL STATUS: WAITING Mask     */
#define I2C_PERIPHERAL_STATUS_STALLED_Pos     3                                                       /*!< I2C_PERIPHERAL STATUS: STALLED Position */
#define I2C_PERIPHERAL_STATUS_STALLED_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_STALLED_Pos)           /*!< I2C_PERIPHERAL STATUS: STALLED Mask     */
#define I2C_PERIPHERAL_STATUS_ARBLOST_Pos     4                                                       /*!< I2C_PERIPHERAL STATUS: ARBLOST Position */
#define I2C_PERIPHERAL_STATUS_ARBLOST_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_ARBLOST_Pos)           /*!< I2C_PERIPHERAL STATUS: ARBLOST Mask     */
#define I2C_PERIPHERAL_STATUS_NACKADDR_Pos    5                                                       /*!< I2C_PERIPHERAL STATUS: NACKADDR Position */
#define I2C_PERIPHERAL_STATUS_NACKADDR_Msk    (0x01UL << I2C_PERIPHERAL_STATUS_NACKADDR_Pos)          /*!< I2C_PERIPHERAL STATUS: NACKADDR Mask    */
#define I2C_PERIPHERAL_STATUS_NACKDATA_Pos    6                                                       /*!< I2C_PERIPHERAL STATUS: NACKDATA Position */
#define I2C_PERIPHERAL_STATUS_NACKDATA_Msk    (0x01UL << I2C_PERIPHERAL_STATUS_NACKDATA_Pos)          /*!< I2C_PERIPHERAL STATUS: NACKDATA Mask    */
#define I2C_PERIPHERAL_STATUS_RXNEMPTY_Pos    8                                                       /*!< I2C_PERIPHERAL STATUS: RXNEMPTY Position */
#define I2C_PERIPHERAL_STATUS_RXNEMPTY_Msk    (0x01UL << I2C_PERIPHERAL_STATUS_RXNEMPTY_Pos)          /*!< I2C_PERIPHERAL STATUS: RXNEMPTY Mask    */
#define I2C_PERIPHERAL_STATUS_RXFULL_Pos      9                                                       /*!< I2C_PERIPHERAL STATUS: RXFULL Position  */
#define I2C_PERIPHERAL_STATUS_RXFULL_Msk      (0x01UL << I2C_PERIPHERAL_STATUS_RXFULL_Pos)            /*!< I2C_PERIPHERAL STATUS: RXFULL Mask      */
#define I2C_PERIPHERAL_STATUS_RXTRIGGER_Pos    11                                                      /*!< I2C_PERIPHERAL STATUS: RXTRIGGER Position */
#define I2C_PERIPHERAL_STATUS_RXTRIGGER_Msk    (0x01UL << I2C_PERIPHERAL_STATUS_RXTRIGGER_Pos)          /*!< I2C_PERIPHERAL STATUS: RXTRIGGER Mask    */
#define I2C_PERIPHERAL_STATUS_TXEMPTY_Pos     12                                                      /*!< I2C_PERIPHERAL STATUS: TXEMPTY Position */
#define I2C_PERIPHERAL_STATUS_TXEMPTY_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_TXEMPTY_Pos)           /*!< I2C_PERIPHERAL STATUS: TXEMPTY Mask     */
#define I2C_PERIPHERAL_STATUS_TXNFULL_Pos     13                                                      /*!< I2C_PERIPHERAL STATUS: TXNFULL Position */
#define I2C_PERIPHERAL_STATUS_TXNFULL_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_TXNFULL_Pos)           /*!< I2C_PERIPHERAL STATUS: TXNFULL Mask     */
#define I2C_PERIPHERAL_STATUS_TXTRIGGER_Pos    15                                                      /*!< I2C_PERIPHERAL STATUS: TXTRIGGER Position */
#define I2C_PERIPHERAL_STATUS_TXTRIGGER_Msk    (0x01UL << I2C_PERIPHERAL_STATUS_TXTRIGGER_Pos)          /*!< I2C_PERIPHERAL STATUS: TXTRIGGER Mask    */
#define I2C_PERIPHERAL_STATUS_RAW_SDA_Pos     30                                                      /*!< I2C_PERIPHERAL STATUS: RAW_SDA Position */
#define I2C_PERIPHERAL_STATUS_RAW_SDA_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_RAW_SDA_Pos)           /*!< I2C_PERIPHERAL STATUS: RAW_SDA Mask     */
#define I2C_PERIPHERAL_STATUS_RAW_SCL_Pos     31                                                      /*!< I2C_PERIPHERAL STATUS: RAW_SCL Position */
#define I2C_PERIPHERAL_STATUS_RAW_SCL_Msk     (0x01UL << I2C_PERIPHERAL_STATUS_RAW_SCL_Pos)           /*!< I2C_PERIPHERAL STATUS: RAW_SCL Mask     */

/* ---------------------------  I2C_PERIPHERAL_STATE  ------------------------------ */
#define I2C_PERIPHERAL_STATE_STATE_Pos  			0                                                       /*!< I2C_PERIPHERAL STATE: STATE Position */
#define I2C_PERIPHERAL_STATE_STATE_Msk  			(0x0FUL << I2C_PERIPHERAL_STATE_STATE_Pos)     				  /*!< I2C_PERIPHERAL STATE: STATE Mask     */
#define I2C_PERIPHERAL_STATE_STEP_Pos  				4	                                                      /*!< I2C_PERIPHERAL STATE: STEP Position */
#define I2C_PERIPHERAL_STATE_STEP_Msk  				(0x0FUL << I2C_PERIPHERAL_STATE_STEP_Pos)        			  /*!< I2C_PERIPHERAL STATE: STEP Mask  */
#define I2C_PERIPHERAL_STATE_RXFIFO_Pos  			8                                                       /*!< I2C_PERIPHERAL STATE: RXFIFO Position */
#define I2C_PERIPHERAL_STATE_RXFIFO_Msk  			(0x01FUL << I2C_PERIPHERAL_STATE_RXFIFO_Pos)        		/*!< I2C_PERIPHERAL STATE: RXFIFO Mask  */
#define I2C_PERIPHERAL_STATE_TXFIFO_Pos  			14                                                      /*!< I2C_PERIPHERAL STATE: TXFIFO Position */
#define I2C_PERIPHERAL_STATE_TXFIFO_Msk  			(0x01FUL << I2C_PERIPHERAL_STATE_TXFIFO_Pos)     			  /*!< I2C_PERIPHERAL STATE: TXFIFO Mask     */
#define I2C_PERIPHERAL_STATE_BITSTATE_Pos  		20                                                      /*!< I2C_PERIPHERAL STATE: BITSTATE Position */
#define I2C_PERIPHERAL_STATE_BITSTATE_Msk  		(0x01FFUL << I2C_PERIPHERAL_STATE_BITSTATE_Pos)     		/*!< I2C_PERIPHERAL STATE: BITSTATE Mask     */

/* ---------------------------  I2C_PERIPHERAL_IRQ_ENB  --------------------------- */
#define I2C_PERIPHERAL_IRQ_ENB_I2CIDLE_Pos    0                                                       /*!< I2C_PERIPHERAL IRQ_ENB: I2CIDLE Position */
#define I2C_PERIPHERAL_IRQ_ENB_I2CIDLE_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_I2CIDLE_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: I2CIDLE Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_IDLE_Pos       1                                                       /*!< I2C_PERIPHERAL IRQ_ENB: IDLE Position   */
#define I2C_PERIPHERAL_IRQ_ENB_IDLE_Msk       (0x01UL << I2C_PERIPHERAL_IRQ_ENB_IDLE_Pos)             /*!< I2C_PERIPHERAL IRQ_ENB: IDLE Mask       */
#define I2C_PERIPHERAL_IRQ_ENB_WAITING_Pos    2                                                       /*!< I2C_PERIPHERAL IRQ_ENB: WAITING Position */
#define I2C_PERIPHERAL_IRQ_ENB_WAITING_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_WAITING_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: WAITING Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_STALLED_Pos    3                                                       /*!< I2C_PERIPHERAL IRQ_ENB: STALLED Position */
#define I2C_PERIPHERAL_IRQ_ENB_STALLED_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_STALLED_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: STALLED Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_ARBLOST_Pos    4                                                       /*!< I2C_PERIPHERAL IRQ_ENB: ARBLOST Position */
#define I2C_PERIPHERAL_IRQ_ENB_ARBLOST_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_ARBLOST_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: ARBLOST Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_NACKADDR_Pos   5                                                       /*!< I2C_PERIPHERAL IRQ_ENB: NACKADDR Position */
#define I2C_PERIPHERAL_IRQ_ENB_NACKADDR_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_ENB_NACKADDR_Pos)         /*!< I2C_PERIPHERAL IRQ_ENB: NACKADDR Mask   */
#define I2C_PERIPHERAL_IRQ_ENB_NACKDATA_Pos   6                                                       /*!< I2C_PERIPHERAL IRQ_ENB: NACKDATA Position */
#define I2C_PERIPHERAL_IRQ_ENB_NACKDATA_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_ENB_NACKDATA_Pos)         /*!< I2C_PERIPHERAL IRQ_ENB: NACKDATA Mask   */
#define I2C_PERIPHERAL_IRQ_ENB_CLKLOTO_Pos    7                                                       /*!< I2C_PERIPHERAL IRQ_ENB: CLKLOTO Position */
#define I2C_PERIPHERAL_IRQ_ENB_CLKLOTO_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_CLKLOTO_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: CLKLOTO Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_TXOVERFLOW_Pos 10                                                      /*!< I2C_PERIPHERAL IRQ_ENB: TXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_ENB_TXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_ENB_TXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_ENB: TXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_ENB_RXOVERFLOW_Pos 11                                                      /*!< I2C_PERIPHERAL IRQ_ENB: RXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_ENB_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_ENB_RXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_ENB: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_ENB_TXREADY_Pos    12                                                      /*!< I2C_PERIPHERAL IRQ_ENB: TXREADY Position */
#define I2C_PERIPHERAL_IRQ_ENB_TXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_TXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: TXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_RXREADY_Pos    13                                                      /*!< I2C_PERIPHERAL IRQ_ENB: RXREADY Position */
#define I2C_PERIPHERAL_IRQ_ENB_RXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_RXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: RXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_TXEMPTY_Pos    14                                                      /*!< I2C_PERIPHERAL IRQ_ENB: TXEMPTY Position */
#define I2C_PERIPHERAL_IRQ_ENB_TXEMPTY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_ENB_TXEMPTY_Pos)          /*!< I2C_PERIPHERAL IRQ_ENB: TXEMPTY Mask    */
#define I2C_PERIPHERAL_IRQ_ENB_RXFULL_Pos     15                                                      /*!< I2C_PERIPHERAL IRQ_ENB: RXFULL Position */
#define I2C_PERIPHERAL_IRQ_ENB_RXFULL_Msk     (0x01UL << I2C_PERIPHERAL_IRQ_ENB_RXFULL_Pos)           /*!< I2C_PERIPHERAL IRQ_ENB: RXFULL Mask     */

/* ---------------------------  I2C_PERIPHERAL_IRQ_RAW  --------------------------- */
#define I2C_PERIPHERAL_IRQ_RAW_I2CIDLE_Pos    0                                                       /*!< I2C_PERIPHERAL IRQ_RAW: I2CIDLE Position */
#define I2C_PERIPHERAL_IRQ_RAW_I2CIDLE_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_I2CIDLE_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: I2CIDLE Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_IDLE_Pos       1                                                       /*!< I2C_PERIPHERAL IRQ_RAW: IDLE Position   */
#define I2C_PERIPHERAL_IRQ_RAW_IDLE_Msk       (0x01UL << I2C_PERIPHERAL_IRQ_RAW_IDLE_Pos)             /*!< I2C_PERIPHERAL IRQ_RAW: IDLE Mask       */
#define I2C_PERIPHERAL_IRQ_RAW_WAITING_Pos    2                                                       /*!< I2C_PERIPHERAL IRQ_RAW: WAITING Position */
#define I2C_PERIPHERAL_IRQ_RAW_WAITING_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_WAITING_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: WAITING Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_STALLED_Pos    3                                                       /*!< I2C_PERIPHERAL IRQ_RAW: STALLED Position */
#define I2C_PERIPHERAL_IRQ_RAW_STALLED_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_STALLED_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: STALLED Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_ARBLOST_Pos    4                                                       /*!< I2C_PERIPHERAL IRQ_RAW: ARBLOST Position */
#define I2C_PERIPHERAL_IRQ_RAW_ARBLOST_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_ARBLOST_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: ARBLOST Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_NACKADDR_Pos   5                                                       /*!< I2C_PERIPHERAL IRQ_RAW: NACKADDR Position */
#define I2C_PERIPHERAL_IRQ_RAW_NACKADDR_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_RAW_NACKADDR_Pos)         /*!< I2C_PERIPHERAL IRQ_RAW: NACKADDR Mask   */
#define I2C_PERIPHERAL_IRQ_RAW_NACKDATA_Pos   6                                                       /*!< I2C_PERIPHERAL IRQ_RAW: NACKDATA Position */
#define I2C_PERIPHERAL_IRQ_RAW_NACKDATA_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_RAW_NACKDATA_Pos)         /*!< I2C_PERIPHERAL IRQ_RAW: NACKDATA Mask   */
#define I2C_PERIPHERAL_IRQ_RAW_CLKLOTO_Pos    7                                                       /*!< I2C_PERIPHERAL IRQ_RAW: CLKLOTO Position */
#define I2C_PERIPHERAL_IRQ_RAW_CLKLOTO_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_CLKLOTO_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: CLKLOTO Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_TXOVERFLOW_Pos 10                                                      /*!< I2C_PERIPHERAL IRQ_RAW: TXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_RAW_TXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_RAW_TXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_RAW: TXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_RAW_RXOVERFLOW_Pos 11                                                      /*!< I2C_PERIPHERAL IRQ_RAW: RXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_RAW_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_RAW_RXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_RAW: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_RAW_TXREADY_Pos    12                                                      /*!< I2C_PERIPHERAL IRQ_RAW: TXREADY Position */
#define I2C_PERIPHERAL_IRQ_RAW_TXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_TXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: TXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_RXREADY_Pos    13                                                      /*!< I2C_PERIPHERAL IRQ_RAW: RXREADY Position */
#define I2C_PERIPHERAL_IRQ_RAW_RXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_RXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: RXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_TXEMPTY_Pos    14                                                      /*!< I2C_PERIPHERAL IRQ_RAW: TXEMPTY Position */
#define I2C_PERIPHERAL_IRQ_RAW_TXEMPTY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_RAW_TXEMPTY_Pos)          /*!< I2C_PERIPHERAL IRQ_RAW: TXEMPTY Mask    */
#define I2C_PERIPHERAL_IRQ_RAW_RXFULL_Pos     15                                                      /*!< I2C_PERIPHERAL IRQ_RAW: RXFULL Position */
#define I2C_PERIPHERAL_IRQ_RAW_RXFULL_Msk     (0x01UL << I2C_PERIPHERAL_IRQ_RAW_RXFULL_Pos)           /*!< I2C_PERIPHERAL IRQ_RAW: RXFULL Mask     */

/* ---------------------------  I2C_PERIPHERAL_IRQ_END  --------------------------- */
#define I2C_PERIPHERAL_IRQ_END_I2CIDLE_Pos    0                                                       /*!< I2C_PERIPHERAL IRQ_END: I2CIDLE Position */
#define I2C_PERIPHERAL_IRQ_END_I2CIDLE_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_I2CIDLE_Pos)          /*!< I2C_PERIPHERAL IRQ_END: I2CIDLE Mask    */
#define I2C_PERIPHERAL_IRQ_END_IDLE_Pos       1                                                       /*!< I2C_PERIPHERAL IRQ_END: IDLE Position   */
#define I2C_PERIPHERAL_IRQ_END_IDLE_Msk       (0x01UL << I2C_PERIPHERAL_IRQ_END_IDLE_Pos)             /*!< I2C_PERIPHERAL IRQ_END: IDLE Mask       */
#define I2C_PERIPHERAL_IRQ_END_WAITING_Pos    2                                                       /*!< I2C_PERIPHERAL IRQ_END: WAITING Position */
#define I2C_PERIPHERAL_IRQ_END_WAITING_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_WAITING_Pos)          /*!< I2C_PERIPHERAL IRQ_END: WAITING Mask    */
#define I2C_PERIPHERAL_IRQ_END_STALLED_Pos    3                                                       /*!< I2C_PERIPHERAL IRQ_END: STALLED Position */
#define I2C_PERIPHERAL_IRQ_END_STALLED_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_STALLED_Pos)          /*!< I2C_PERIPHERAL IRQ_END: STALLED Mask    */
#define I2C_PERIPHERAL_IRQ_END_ARBLOST_Pos    4                                                       /*!< I2C_PERIPHERAL IRQ_END: ARBLOST Position */
#define I2C_PERIPHERAL_IRQ_END_ARBLOST_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_ARBLOST_Pos)          /*!< I2C_PERIPHERAL IRQ_END: ARBLOST Mask    */
#define I2C_PERIPHERAL_IRQ_END_NACKADDR_Pos   5                                                       /*!< I2C_PERIPHERAL IRQ_END: NACKADDR Position */
#define I2C_PERIPHERAL_IRQ_END_NACKADDR_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_END_NACKADDR_Pos)         /*!< I2C_PERIPHERAL IRQ_END: NACKADDR Mask   */
#define I2C_PERIPHERAL_IRQ_END_NACKDATA_Pos   6                                                       /*!< I2C_PERIPHERAL IRQ_END: NACKDATA Position */
#define I2C_PERIPHERAL_IRQ_END_NACKDATA_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_END_NACKDATA_Pos)         /*!< I2C_PERIPHERAL IRQ_END: NACKDATA Mask   */
#define I2C_PERIPHERAL_IRQ_END_CLKLOTO_Pos    7                                                       /*!< I2C_PERIPHERAL IRQ_END: CLKLOTO Position */
#define I2C_PERIPHERAL_IRQ_END_CLKLOTO_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_CLKLOTO_Pos)          /*!< I2C_PERIPHERAL IRQ_END: CLKLOTO Mask    */
#define I2C_PERIPHERAL_IRQ_END_TXOVERFLOW_Pos 10                                                      /*!< I2C_PERIPHERAL IRQ_END: TXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_END_TXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_END_TXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_END: TXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_END_RXOVERFLOW_Pos 11                                                      /*!< I2C_PERIPHERAL IRQ_END: RXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_END_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_END_RXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_END: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_END_TXREADY_Pos    12                                                      /*!< I2C_PERIPHERAL IRQ_END: TXREADY Position */
#define I2C_PERIPHERAL_IRQ_END_TXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_TXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_END: TXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_END_RXREADY_Pos    13                                                      /*!< I2C_PERIPHERAL IRQ_END: RXREADY Position */
#define I2C_PERIPHERAL_IRQ_END_RXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_RXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_END: RXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_END_TXEMPTY_Pos    14                                                      /*!< I2C_PERIPHERAL IRQ_END: TXEMPTY Position */
#define I2C_PERIPHERAL_IRQ_END_TXEMPTY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_END_TXEMPTY_Pos)          /*!< I2C_PERIPHERAL IRQ_END: TXEMPTY Mask    */
#define I2C_PERIPHERAL_IRQ_END_RXFULL_Pos     15                                                      /*!< I2C_PERIPHERAL IRQ_END: RXFULL Position */
#define I2C_PERIPHERAL_IRQ_END_RXFULL_Msk     (0x01UL << I2C_PERIPHERAL_IRQ_END_RXFULL_Pos)           /*!< I2C_PERIPHERAL IRQ_END: RXFULL Mask     */

/* ---------------------------  I2C_PERIPHERAL_IRQ_CLR  --------------------------- */
#define I2C_PERIPHERAL_IRQ_CLR_I2CIDLE_Pos    0                                                       /*!< I2C_PERIPHERAL IRQ_CLR: I2CIDLE Position */
#define I2C_PERIPHERAL_IRQ_CLR_I2CIDLE_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_I2CIDLE_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: I2CIDLE Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_IDLE_Pos       1                                                       /*!< I2C_PERIPHERAL IRQ_CLR: IDLE Position   */
#define I2C_PERIPHERAL_IRQ_CLR_IDLE_Msk       (0x01UL << I2C_PERIPHERAL_IRQ_CLR_IDLE_Pos)             /*!< I2C_PERIPHERAL IRQ_CLR: IDLE Mask       */
#define I2C_PERIPHERAL_IRQ_CLR_WAITING_Pos    2                                                       /*!< I2C_PERIPHERAL IRQ_CLR: WAITING Position */
#define I2C_PERIPHERAL_IRQ_CLR_WAITING_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_WAITING_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: WAITING Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_STALLED_Pos    3                                                       /*!< I2C_PERIPHERAL IRQ_CLR: STALLED Position */
#define I2C_PERIPHERAL_IRQ_CLR_STALLED_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_STALLED_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: STALLED Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_ARBLOST_Pos    4                                                       /*!< I2C_PERIPHERAL IRQ_CLR: ARBLOST Position */
#define I2C_PERIPHERAL_IRQ_CLR_ARBLOST_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_ARBLOST_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: ARBLOST Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_NACKADDR_Pos   5                                                       /*!< I2C_PERIPHERAL IRQ_CLR: NACKADDR Position */
#define I2C_PERIPHERAL_IRQ_CLR_NACKADDR_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_CLR_NACKADDR_Pos)         /*!< I2C_PERIPHERAL IRQ_CLR: NACKADDR Mask   */
#define I2C_PERIPHERAL_IRQ_CLR_NACKDATA_Pos   6                                                       /*!< I2C_PERIPHERAL IRQ_CLR: NACKDATA Position */
#define I2C_PERIPHERAL_IRQ_CLR_NACKDATA_Msk   (0x01UL << I2C_PERIPHERAL_IRQ_CLR_NACKDATA_Pos)         /*!< I2C_PERIPHERAL IRQ_CLR: NACKDATA Mask   */
#define I2C_PERIPHERAL_IRQ_CLR_CLKLOTO_Pos    7                                                       /*!< I2C_PERIPHERAL IRQ_CLR: CLKLOTO Position */
#define I2C_PERIPHERAL_IRQ_CLR_CLKLOTO_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_CLKLOTO_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: CLKLOTO Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_TXOVERFLOW_Pos 10                                                      /*!< I2C_PERIPHERAL IRQ_CLR: TXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_CLR_TXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_CLR_TXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_CLR: TXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_CLR_RXOVERFLOW_Pos 11                                                      /*!< I2C_PERIPHERAL IRQ_CLR: RXOVERFLOW Position */
#define I2C_PERIPHERAL_IRQ_CLR_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_IRQ_CLR_RXOVERFLOW_Pos)       /*!< I2C_PERIPHERAL IRQ_CLR: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_IRQ_CLR_TXREADY_Pos    12                                                      /*!< I2C_PERIPHERAL IRQ_CLR: TXREADY Position */
#define I2C_PERIPHERAL_IRQ_CLR_TXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_TXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: TXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_RXREADY_Pos    13                                                      /*!< I2C_PERIPHERAL IRQ_CLR: RXREADY Position */
#define I2C_PERIPHERAL_IRQ_CLR_RXREADY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_RXREADY_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: RXREADY Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_TXEMPTY_Pos    14                                                      /*!< I2C_PERIPHERAL IRQ_CLR: TXEMPTY Position */
#define I2C_PERIPHERAL_IRQ_CLR_TXEMPTY_Msk    (0x01UL << I2C_PERIPHERAL_IRQ_CLR_TXEMPTY_Pos)          /*!< I2C_PERIPHERAL IRQ_CLR: TXEMPTY Mask    */
#define I2C_PERIPHERAL_IRQ_CLR_RXFULL_Pos     15                                                      /*!< I2C_PERIPHERAL IRQ_CLR: RXFULL Position */
#define I2C_PERIPHERAL_IRQ_CLR_RXFULL_Msk     (0x01UL << I2C_PERIPHERAL_IRQ_CLR_RXFULL_Pos)           /*!< I2C_PERIPHERAL IRQ_CLR: RXFULL Mask     */

/* ---------------------------  I2C_PERIPHERAL_FIFO_CLR  -------------------------- */
#define I2C_PERIPHERAL_FIFO_CLR_RXFIFO_Pos    0                                                       /*!< I2C_PERIPHERAL FIFO_CLR: RXFIFO Position */
#define I2C_PERIPHERAL_FIFO_CLR_RXFIFO_Msk    (0x01UL << I2C_PERIPHERAL_FIFO_CLR_RXFIFO_Pos)          /*!< I2C_PERIPHERAL FIFO_CLR: RXFIFO Mask    */
#define I2C_PERIPHERAL_FIFO_CLR_TXFIFO_Pos    1                                                       /*!< I2C_PERIPHERAL FIFO_CLR: TXFIFO Position */
#define I2C_PERIPHERAL_FIFO_CLR_TXFIFO_Msk    (0x01UL << I2C_PERIPHERAL_FIFO_CLR_TXFIFO_Pos)          /*!< I2C_PERIPHERAL FIFO_CLR: TXFIFO Mask    */

/* ---------------------------  I2C_PERIPHERAL_S0_CTRL  --------------------------- */
#define I2C_PERIPHERAL_S0_CTRL_CLKENABLED_Pos 0                                                       /*!< I2C_PERIPHERAL S0_CTRL: CLKENABLED Position */
#define I2C_PERIPHERAL_S0_CTRL_CLKENABLED_Msk (0x01UL << I2C_PERIPHERAL_S0_CTRL_CLKENABLED_Pos)       /*!< I2C_PERIPHERAL S0_CTRL: CLKENABLED Mask    */
#define I2C_PERIPHERAL_S0_CTRL_ENABLED_Pos    1                                                       /*!< I2C_PERIPHERAL S0_CTRL: ENABLED Position */
#define I2C_PERIPHERAL_S0_CTRL_ENABLED_Msk    (0x01UL << I2C_PERIPHERAL_S0_CTRL_ENABLED_Pos)          /*!< I2C_PERIPHERAL S0_CTRL: ENABLED Mask    */
#define I2C_PERIPHERAL_S0_CTRL_ENABLE_Pos     2                                                       /*!< I2C_PERIPHERAL S0_CTRL: ENABLE Position */
#define I2C_PERIPHERAL_S0_CTRL_ENABLE_Msk     (0x01UL << I2C_PERIPHERAL_S0_CTRL_ENABLE_Pos)           /*!< I2C_PERIPHERAL S0_CTRL: ENABLE Mask     */
#define I2C_PERIPHERAL_S0_CTRL_TXFEMD_Pos     3                                                       /*!< I2C_PERIPHERAL S0_CTRL: TXFEMD Position */
#define I2C_PERIPHERAL_S0_CTRL_TXFEMD_Msk     (0x01UL << I2C_PERIPHERAL_S0_CTRL_TXFEMD_Pos)           /*!< I2C_PERIPHERAL S0_CTRL: TXFEMD Mask     */
#define I2C_PERIPHERAL_S0_CTRL_RXFFMD_Pos     4                                                       /*!< I2C_PERIPHERAL S0_CTRL: RXFFMD Position */
#define I2C_PERIPHERAL_S0_CTRL_RXFFMD_Msk     (0x01UL << I2C_PERIPHERAL_S0_CTRL_RXFFMD_Pos)           /*!< I2C_PERIPHERAL S0_CTRL: RXFFMD Mask     */

/* ---------------------------  I2C_PERIPHERAL_S0_MAXWORDS  --------------------------- */
#define I2C_PERIPHERAL_S0_MAXWORDS_MAXWORDS_Pos  0                                                    /*!< I2C_PERIPHERAL S0_MAXWORDS: MAXWORDS Position */
#define I2C_PERIPHERAL_S0_MAXWORDS_MAXWORDS_Msk  (0x07FFUL << I2C_PERIPHERAL_S0_MAXWORDS_MAXWORDS_Pos)/*!< I2C_PERIPHERAL S0_MAXWORDS: MAXWORDS Mask    */
#define I2C_PERIPHERAL_S0_MAXWORDS_ENABLE_Pos  	 31                                                   /*!< I2C_PERIPHERAL S0_MAXWORDS: ENABLE Position */
#define I2C_PERIPHERAL_S0_MAXWORDS_ENABLE_Msk  	 (0x01UL << I2C_PERIPHERAL_S0_MAXWORDS_ENABLE_Pos) 		/*!< I2C_PERIPHERAL S0_MAXWORDS: ENABLE Mask  */

/* ---------------------------  I2C_PERIPHERAL_S0_ADDRESS  -------------------------- */
#define I2C_PERIPHERAL_S0_ADDRESS_RW_Pos  			0                                                     /*!< I2C_PERIPHERAL S0_ADDRESS: RW Position */
#define I2C_PERIPHERAL_S0_ADDRESS_RW_Msk  			(0x01UL << I2C_PERIPHERAL_S0_ADDRESS_RW_Pos)     			/*!< I2C_PERIPHERAL S0_ADDRESS: RW Mask     */
#define I2C_PERIPHERAL_S0_ADDRESS_ADDRESS_Pos  	1	                                                    /*!< I2C_PERIPHERAL S0_ADDRESS: ADDRESS Position */
#define I2C_PERIPHERAL_S0_ADDRESS_ADDRESS_Msk  	(0x3FFUL << I2C_PERIPHERAL_S0_ADDRESS_ADDRESS_Pos)    /*!< I2C_PERIPHERAL S0_ADDRESS: ADDRESS Mask  */
#define I2C_PERIPHERAL_S0_ADDRESS_A10MODE_Pos  	15                                                    /*!< I2C_PERIPHERAL S0_ADDRESS: A10MODE Position */
#define I2C_PERIPHERAL_S0_ADDRESS_A10MODE_Msk  	(0x01UL << I2C_PERIPHERAL_S0_ADDRESS_A10MODE_Pos)     /*!< I2C_PERIPHERAL S0_ADDRESS: A10MODE Mask  */

/* ---------------------------  I2C_PERIPHERAL_S0_ADDRESSMASK  -------------------------- */
#define I2C_PERIPHERAL_S0_ADDRESSMASK_RWMASK_Pos 0                                                     /*!< I2C_PERIPHERAL S0_ADDRESSMASK: RWMASK Position */
#define I2C_PERIPHERAL_S0_ADDRESSMASK_RWMASK_Msk (0x01UL << I2C_PERIPHERAL_S0_ADDRESSMASK_RWMASK_Pos)  /*!< I2C_PERIPHERAL S0_ADDRESSMASK: RWMASK Mask     */
#define I2C_PERIPHERAL_S0_ADDRESSMASK_MASK_Pos   1	                                                   /*!< I2C_PERIPHERAL S0_ADDRESSMASK: MASK Position */
#define I2C_PERIPHERAL_S0_ADDRESSMASK_MASK_Msk   (0x3FFUL << I2C_PERIPHERAL_S0_ADDRESSMASK_MASK_Pos)   /*!< I2C_PERIPHERAL S0_ADDRESSMASK: MASK Mask  */

/* --------------------------  I2C_PERIPHERAL_S0_STATUS  -------------------------- */
#define I2C_PERIPHERAL_S0_STATUS_COMPLETED_Pos 0                                                      /*!< I2C_PERIPHERAL S0_STATUS: COMPLETED Position */
#define I2C_PERIPHERAL_S0_STATUS_COMPLETED_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_COMPLETED_Pos)     /*!< I2C_PERIPHERAL S0_STATUS: COMPLETED Mask */
#define I2C_PERIPHERAL_S0_STATUS_IDLE_Pos     1                                                       /*!< I2C_PERIPHERAL S0_STATUS: IDLE Position */
#define I2C_PERIPHERAL_S0_STATUS_IDLE_Msk     (0x01UL << I2C_PERIPHERAL_S0_STATUS_IDLE_Pos)           /*!< I2C_PERIPHERAL S0_STATUS: IDLE Mask     */
#define I2C_PERIPHERAL_S0_STATUS_WAITING_Pos  2                                                       /*!< I2C_PERIPHERAL S0_STATUS: WAITING Position */
#define I2C_PERIPHERAL_S0_STATUS_WAITING_Msk  (0x01UL << I2C_PERIPHERAL_S0_STATUS_WAITING_Pos)        /*!< I2C_PERIPHERAL S0_STATUS: WAITING Mask  */
#define I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Pos 3                                                      /*!< I2C_PERIPHERAL S0_STATUS: TXSTALLED Position */
#define I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_TXSTALLED_Pos)     /*!< I2C_PERIPHERAL S0_STATUS: TXSTALLED Mask */
#define I2C_PERIPHERAL_S0_STATUS_RXSTALLED_Pos 4                                                      /*!< I2C_PERIPHERAL S0_STATUS: RXSTALLED Position */
#define I2C_PERIPHERAL_S0_STATUS_RXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_RXSTALLED_Pos)     /*!< I2C_PERIPHERAL S0_STATUS: RXSTALLED Mask */
#define I2C_PERIPHERAL_S0_STATUS_ADDRESSMATCH_Pos 5                                                   /*!< I2C_PERIPHERAL S0_STATUS: ADDRESSMATCH Position */
#define I2C_PERIPHERAL_S0_STATUS_ADDRESSMATCH_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_ADDRESSMATCH_Pos)/*!< I2C_PERIPHERAL S0_STATUS: ADDRESSMATCH Mask */
#define I2C_PERIPHERAL_S0_STATUS_NACKDATA_Pos 6                                                       /*!< I2C_PERIPHERAL S0_STATUS: NACKDATA Position */
#define I2C_PERIPHERAL_S0_STATUS_NACKDATA_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_NACKDATA_Pos)       /*!< I2C_PERIPHERAL S0_STATUS: NACKDATA Mask */
#define I2C_PERIPHERAL_S0_STATUS_RXDATAFIRST_Pos 7                                                    /*!< I2C_PERIPHERAL S0_STATUS: RXDATAFIRST Position */
#define I2C_PERIPHERAL_S0_STATUS_RXDATAFIRST_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_RXDATAFIRST_Pos) /*!< I2C_PERIPHERAL S0_STATUS: RXDATAFIRST Mask */
#define I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Pos 8                                                       /*!< I2C_PERIPHERAL S0_STATUS: RXNEMPTY Position */
#define I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_RXNEMPTY_Pos)       /*!< I2C_PERIPHERAL S0_STATUS: RXNEMPTY Mask */
#define I2C_PERIPHERAL_S0_STATUS_RXFULL_Pos   9                                                       /*!< I2C_PERIPHERAL S0_STATUS: RXFULL Position */
#define I2C_PERIPHERAL_S0_STATUS_RXFULL_Msk   (0x01UL << I2C_PERIPHERAL_S0_STATUS_RXFULL_Pos)         /*!< I2C_PERIPHERAL S0_STATUS: RXFULL Mask   */
#define I2C_PERIPHERAL_S0_STATUS_RXTRIGGER_Pos 11                                                      /*!< I2C_PERIPHERAL S0_STATUS: RXTRIGGER Position */
#define I2C_PERIPHERAL_S0_STATUS_RXTRIGGER_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_RXTRIGGER_Pos)       /*!< I2C_PERIPHERAL S0_STATUS: RXTRIGGER Mask */
#define I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Pos  12                                                      /*!< I2C_PERIPHERAL S0_STATUS: TXEMPTY Position */
#define I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Msk  (0x01UL << I2C_PERIPHERAL_S0_STATUS_TXEMPTY_Pos)        /*!< I2C_PERIPHERAL S0_STATUS: TXEMPTY Mask  */
#define I2C_PERIPHERAL_S0_STATUS_TXNFULL_Pos  13                                                      /*!< I2C_PERIPHERAL S0_STATUS: TXNFULL Position */
#define I2C_PERIPHERAL_S0_STATUS_TXNFULL_Msk  (0x01UL << I2C_PERIPHERAL_S0_STATUS_TXNFULL_Pos)        /*!< I2C_PERIPHERAL S0_STATUS: TXNFULL Mask  */
#define I2C_PERIPHERAL_S0_STATUS_TXTRIGGER_Pos 15                                                      /*!< I2C_PERIPHERAL S0_STATUS: TXTRIGGER Position */
#define I2C_PERIPHERAL_S0_STATUS_TXTRIGGER_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_TXTRIGGER_Pos)       /*!< I2C_PERIPHERAL S0_STATUS: TXTRIGGER Mask */
#define I2C_PERIPHERAL_S0_STATUS_RAW_BUSY_Pos 29                                                      /*!< I2C_PERIPHERAL S0_STATUS: RAW_BUSY Position */
#define I2C_PERIPHERAL_S0_STATUS_RAW_BUSY_Msk (0x01UL << I2C_PERIPHERAL_S0_STATUS_RAW_BUSY_Pos)       /*!< I2C_PERIPHERAL S0_STATUS: RAW_BUSY Mask */
#define I2C_PERIPHERAL_S0_STATUS_RAW_SDA_Pos  30                                                      /*!< I2C_PERIPHERAL S0_STATUS: RAW_SDA Position */
#define I2C_PERIPHERAL_S0_STATUS_RAW_SDA_Msk  (0x01UL << I2C_PERIPHERAL_S0_STATUS_RAW_SDA_Pos)        /*!< I2C_PERIPHERAL S0_STATUS: RAW_SDA Mask  */
#define I2C_PERIPHERAL_S0_STATUS_RAW_SCL_Pos  31                                                      /*!< I2C_PERIPHERAL S0_STATUS: RAW_SCL Position */
#define I2C_PERIPHERAL_S0_STATUS_RAW_SCL_Msk  (0x01UL << I2C_PERIPHERAL_S0_STATUS_RAW_SCL_Pos)        /*!< I2C_PERIPHERAL S0_STATUS: RAW_SCL Mask  */

/* ---------------------------  I2C_PERIPHERAL_S0_STATE  ------------------------------ */
#define I2C_PERIPHERAL_S0_STATE_STATE_Pos  		0                                                       /*!< I2C_PERIPHERAL S0_STATE: STATE Position */
#define I2C_PERIPHERAL_S0_STATE_STATE_Msk  		(0x07UL << I2C_PERIPHERAL_S0_STATE_STATE_Pos)     			/*!< I2C_PERIPHERAL S0_STATE: STATE Mask     */
#define I2C_PERIPHERAL_S0_STATE_STEP_Pos  		4	                                                      /*!< I2C_PERIPHERAL S0_STATE: STEP Position */
#define I2C_PERIPHERAL_S0_STATE_STEP_Msk  		(0x0FUL << I2C_PERIPHERAL_S0_STATE_STEP_Pos)        		/*!< I2C_PERIPHERAL S0_STATE: STEP Mask  */
#define I2C_PERIPHERAL_S0_STATE_RXFIFO_Pos  	8                                                       /*!< I2C_PERIPHERAL S0_STATE: RXFIFO Position */
#define I2C_PERIPHERAL_S0_STATE_RXFIFO_Msk  	(0x01FUL << I2C_PERIPHERAL_S0_STATE_RXFIFO_Pos)        	/*!< I2C_PERIPHERAL S0_STATE: RXFIFO Mask  */
#define I2C_PERIPHERAL_S0_STATE_TXFIFO_Pos  	14                                                      /*!< I2C_PERIPHERAL S0_STATE: TXFIFO Position */
#define I2C_PERIPHERAL_S0_STATE_TXFIFO_Msk  	(0x01FUL << I2C_PERIPHERAL_S0_STATE_TXFIFO_Pos)     		/*!< I2C_PERIPHERAL S0_STATE: TXFIFO Mask     */

/* --------------------------  I2C_PERIPHERAL_S0_IRQ_ENB  ------------------------- */
#define I2C_PERIPHERAL_S0_IRQ_ENB_COMPLETED_Pos 0                                                     /*!< I2C_PERIPHERAL S0_IRQ_ENB: COMPLETED Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_COMPLETED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_COMPLETED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_ENB: COMPLETED Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_IDLE_Pos    1                                                       /*!< I2C_PERIPHERAL S0_IRQ_ENB: IDLE Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_IDLE_Msk    (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_IDLE_Pos)          /*!< I2C_PERIPHERAL S0_IRQ_ENB: IDLE Mask    */
#define I2C_PERIPHERAL_S0_IRQ_ENB_WAITING_Pos 2                                                       /*!< I2C_PERIPHERAL S0_IRQ_ENB: WAITING Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_WAITING_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_WAITING_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_ENB: WAITING Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXSTALLED_Pos 3                                                     /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_TXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXSTALLED_Pos 4                                                     /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_RXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_ADDRESSMATCH_Pos 5                                                  /*!< I2C_PERIPHERAL S0_IRQ_ENB: ADDRESSMATCH Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_ADDRESSMATCH_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_ADDRESSMATCH_Pos)/*!< I2C_PERIPHERAL S0_IRQ_ENB: ADDRESSMATCH Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_NACKDATA_Pos 6                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: NACKDATA Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_NACKDATA_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_NACKDATA_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_ENB: NACKDATA Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXDATAFIRST_Pos 7                                                   /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXDATAFIRST Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXDATAFIRST_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_RXDATAFIRST_Pos)/*!< I2C_PERIPHERAL S0_IRQ_ENB: RXDATAFIRST Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_I2C_START_Pos 8                                                     /*!< I2C_PERIPHERAL S0_IRQ_ENB: I2C_START Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_I2C_START_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_I2C_START_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_ENB: I2C_START Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Pos 9                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: I2C_STOP Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_I2C_STOP_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_ENB: I2C_STOP Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXUNDERFLOW_Pos 10                                                  /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXUNDERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXUNDERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_TXUNDERFLOW_Pos)/*!< I2C_PERIPHERAL S0_IRQ_ENB: TXUNDERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXOVERFLOW_Pos 11                                                   /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXOVERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_RXOVERFLOW_Pos) /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXREADY_Pos 12                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_TXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXREADY_Pos 13                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_RXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXEMPTY_Pos 14                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXEMPTY Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_TXEMPTY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_TXEMPTY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_ENB: TXEMPTY Mask */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXFULL_Pos  15                                                      /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXFULL Position */
#define I2C_PERIPHERAL_S0_IRQ_ENB_RXFULL_Msk  (0x01UL << I2C_PERIPHERAL_S0_IRQ_ENB_RXFULL_Pos)        /*!< I2C_PERIPHERAL S0_IRQ_ENB: RXFULL Mask  */

/* --------------------------  I2C_PERIPHERAL_S0_IRQ_RAW  ------------------------- */
#define I2C_PERIPHERAL_S0_IRQ_RAW_COMPLETED_Pos 0                                                     /*!< I2C_PERIPHERAL S0_IRQ_RAW: COMPLETED Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_COMPLETED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_COMPLETED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_RAW: COMPLETED Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_IDLE_Pos    1                                                       /*!< I2C_PERIPHERAL S0_IRQ_RAW: IDLE Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_IDLE_Msk    (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_IDLE_Pos)          /*!< I2C_PERIPHERAL S0_IRQ_RAW: IDLE Mask    */
#define I2C_PERIPHERAL_S0_IRQ_RAW_WAITING_Pos 2                                                       /*!< I2C_PERIPHERAL S0_IRQ_RAW: WAITING Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_WAITING_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_WAITING_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_RAW: WAITING Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXSTALLED_Pos 3                                                     /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_TXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXSTALLED_Pos 4                                                     /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_RXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_ADDRESSMATCH_Pos 5                                                  /*!< I2C_PERIPHERAL S0_IRQ_RAW: ADDRESSMATCH Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_ADDRESSMATCH_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_ADDRESSMATCH_Pos)/*!< I2C_PERIPHERAL S0_IRQ_RAW: ADDRESSMATCH Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_NACKDATA_Pos 6                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: NACKDATA Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_NACKDATA_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_NACKDATA_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_RAW: NACKDATA Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXDATAFIRST_Pos 7                                                   /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXDATAFIRST Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXDATAFIRST_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_RXDATAFIRST_Pos)/*!< I2C_PERIPHERAL S0_IRQ_RAW: RXDATAFIRST Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_I2C_START_Pos 8                                                     /*!< I2C_PERIPHERAL S0_IRQ_RAW: I2C_START Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_I2C_START_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_I2C_START_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_RAW: I2C_START Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Pos 9                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: I2C_STOP Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_I2C_STOP_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_RAW: I2C_STOP Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXUNDERFLOW_Pos 10                                                  /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXUNDERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXUNDERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_TXUNDERFLOW_Pos)/*!< I2C_PERIPHERAL S0_IRQ_RAW: TXUNDERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXOVERFLOW_Pos 11                                                   /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXOVERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_RXOVERFLOW_Pos) /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXREADY_Pos 12                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_TXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXREADY_Pos 13                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_RXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXEMPTY_Pos 14                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXEMPTY Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_TXEMPTY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_TXEMPTY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_RAW: TXEMPTY Mask */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXFULL_Pos  15                                                      /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXFULL Position */
#define I2C_PERIPHERAL_S0_IRQ_RAW_RXFULL_Msk  (0x01UL << I2C_PERIPHERAL_S0_IRQ_RAW_RXFULL_Pos)        /*!< I2C_PERIPHERAL S0_IRQ_RAW: RXFULL Mask  */

/* --------------------------  I2C_PERIPHERAL_S0_IRQ_END  ------------------------- */
#define I2C_PERIPHERAL_S0_IRQ_END_COMPLETED_Pos 0                                                     /*!< I2C_PERIPHERAL S0_IRQ_END: COMPLETED Position */
#define I2C_PERIPHERAL_S0_IRQ_END_COMPLETED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_COMPLETED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_END: COMPLETED Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_IDLE_Pos    1                                                       /*!< I2C_PERIPHERAL S0_IRQ_END: IDLE Position */
#define I2C_PERIPHERAL_S0_IRQ_END_IDLE_Msk    (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_IDLE_Pos)          /*!< I2C_PERIPHERAL S0_IRQ_END: IDLE Mask    */
#define I2C_PERIPHERAL_S0_IRQ_END_WAITING_Pos 2                                                       /*!< I2C_PERIPHERAL S0_IRQ_END: WAITING Position */
#define I2C_PERIPHERAL_S0_IRQ_END_WAITING_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_WAITING_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_END: WAITING Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_TXSTALLED_Pos 3                                                     /*!< I2C_PERIPHERAL S0_IRQ_END: TXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_END_TXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_TXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_END: TXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_RXSTALLED_Pos 4                                                     /*!< I2C_PERIPHERAL S0_IRQ_END: RXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_END_RXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_RXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_END: RXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_ADDRESSMATCH_Pos 5                                                  /*!< I2C_PERIPHERAL S0_IRQ_END: ADDRESSMATCH Position */
#define I2C_PERIPHERAL_S0_IRQ_END_ADDRESSMATCH_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_ADDRESSMATCH_Pos)/*!< I2C_PERIPHERAL S0_IRQ_END: ADDRESSMATCH Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_NACKDATA_Pos 6                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: NACKDATA Position */
#define I2C_PERIPHERAL_S0_IRQ_END_NACKDATA_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_NACKDATA_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_END: NACKDATA Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_RXDATAFIRST_Pos 7                                                   /*!< I2C_PERIPHERAL S0_IRQ_END: RXDATAFIRST Position */
#define I2C_PERIPHERAL_S0_IRQ_END_RXDATAFIRST_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_RXDATAFIRST_Pos)/*!< I2C_PERIPHERAL S0_IRQ_END: RXDATAFIRST Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_I2C_START_Pos 8                                                     /*!< I2C_PERIPHERAL S0_IRQ_END: I2C_START Position */
#define I2C_PERIPHERAL_S0_IRQ_END_I2C_START_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_I2C_START_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_END: I2C_START Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_I2C_STOP_Pos 9                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: I2C_STOP Position */
#define I2C_PERIPHERAL_S0_IRQ_END_I2C_STOP_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_I2C_STOP_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_END: I2C_STOP Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Pos 10                                                  /*!< I2C_PERIPHERAL S0_IRQ_END: TXUNDERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_TXUNDERFLOW_Pos)/*!< I2C_PERIPHERAL S0_IRQ_END: TXUNDERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_RXOVERFLOW_Pos 11                                                   /*!< I2C_PERIPHERAL S0_IRQ_END: RXOVERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_END_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_RXOVERFLOW_Pos) /*!< I2C_PERIPHERAL S0_IRQ_END: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Pos 12                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: TXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_TXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_END: TXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Pos 13                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: RXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_RXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_END: RXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Pos 14                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: TXEMPTY Position */
#define I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_TXEMPTY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_END: TXEMPTY Mask */
#define I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Pos  15                                                      /*!< I2C_PERIPHERAL S0_IRQ_END: RXFULL Position */
#define I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Msk  (0x01UL << I2C_PERIPHERAL_S0_IRQ_END_RXFULL_Pos)        /*!< I2C_PERIPHERAL S0_IRQ_END: RXFULL Mask  */

/* --------------------------  I2C_PERIPHERAL_S0_IRQ_CLR  ------------------------- */
#define I2C_PERIPHERAL_S0_IRQ_CLR_COMPLETED_Pos 0                                                     /*!< I2C_PERIPHERAL S0_IRQ_CLR: COMPLETED Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_COMPLETED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_COMPLETED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_CLR: COMPLETED Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_IDLE_Pos    1                                                       /*!< I2C_PERIPHERAL S0_IRQ_CLR: IDLE Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_IDLE_Msk    (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_IDLE_Pos)          /*!< I2C_PERIPHERAL S0_IRQ_CLR: IDLE Mask    */
#define I2C_PERIPHERAL_S0_IRQ_CLR_WAITING_Pos 2                                                       /*!< I2C_PERIPHERAL S0_IRQ_CLR: WAITING Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_WAITING_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_WAITING_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_CLR: WAITING Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXSTALLED_Pos 3                                                     /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_TXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXSTALLED_Pos 4                                                     /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXSTALLED Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXSTALLED_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_RXSTALLED_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXSTALLED Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_ADDRESSMATCH_Pos 5                                                  /*!< I2C_PERIPHERAL S0_IRQ_CLR: ADDRESSMATCH Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_ADDRESSMATCH_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_ADDRESSMATCH_Pos)/*!< I2C_PERIPHERAL S0_IRQ_CLR: ADDRESSMATCH Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_NACKDATA_Pos 6                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: NACKDATA Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_NACKDATA_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_NACKDATA_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_CLR: NACKDATA Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXDATAFIRST_Pos 7                                                   /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXDATAFIRST Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXDATAFIRST_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_RXDATAFIRST_Pos)/*!< I2C_PERIPHERAL S0_IRQ_CLR: RXDATAFIRST Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_I2C_START_Pos 8                                                     /*!< I2C_PERIPHERAL S0_IRQ_CLR: I2C_START Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_I2C_START_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_I2C_START_Pos)   /*!< I2C_PERIPHERAL S0_IRQ_CLR: I2C_START Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_I2C_STOP_Pos 9                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: I2C_STOP Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_I2C_STOP_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_I2C_STOP_Pos)     /*!< I2C_PERIPHERAL S0_IRQ_CLR: I2C_STOP Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXUNDERFLOW_Pos 10                                                  /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXUNDERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXUNDERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_TXUNDERFLOW_Pos)/*!< I2C_PERIPHERAL S0_IRQ_CLR: TXUNDERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXOVERFLOW_Pos 11                                                   /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXOVERFLOW Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXOVERFLOW_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_RXOVERFLOW_Pos) /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXOVERFLOW Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXREADY_Pos 12                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_TXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXREADY_Pos 13                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXREADY Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXREADY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_RXREADY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXREADY Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXEMPTY_Pos 14                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXEMPTY Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_TXEMPTY_Msk (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_TXEMPTY_Pos)       /*!< I2C_PERIPHERAL S0_IRQ_CLR: TXEMPTY Mask */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXFULL_Pos  15                                                      /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXFULL Position */
#define I2C_PERIPHERAL_S0_IRQ_CLR_RXFULL_Msk  (0x01UL << I2C_PERIPHERAL_S0_IRQ_CLR_RXFULL_Pos)        /*!< I2C_PERIPHERAL S0_IRQ_CLR: RXFULL Mask  */

/* -------------------------  I2C_PERIPHERAL_S0_FIFO_CLR  ------------------------- */
#define I2C_PERIPHERAL_S0_FIFO_CLR_RXFIFO_Pos 0                                                       /*!< I2C_PERIPHERAL S0_FIFO_CLR: RXFIFO Position */
#define I2C_PERIPHERAL_S0_FIFO_CLR_RXFIFO_Msk (0x01UL << I2C_PERIPHERAL_S0_FIFO_CLR_RXFIFO_Pos)       /*!< I2C_PERIPHERAL S0_FIFO_CLR: RXFIFO Mask */
#define I2C_PERIPHERAL_S0_FIFO_CLR_TXFIFO_Pos 1                                                       /*!< I2C_PERIPHERAL S0_FIFO_CLR: TXFIFO Position */
#define I2C_PERIPHERAL_S0_FIFO_CLR_TXFIFO_Msk (0x01UL << I2C_PERIPHERAL_S0_FIFO_CLR_TXFIFO_Pos)       /*!< I2C_PERIPHERAL S0_FIFO_CLR: TXFIFO Mask */

/* ---------------------------  I2C_PERIPHERAL_S0_ADDRESSB  -------------------------- */
#define I2C_PERIPHERAL_S0_ADDRESSB_RW_Pos  				0                                                   /*!< I2C_PERIPHERAL S0_ADDRESSB: RW Position */
#define I2C_PERIPHERAL_S0_ADDRESSB_RW_Msk  			  (0x01UL << I2C_PERIPHERAL_S0_ADDRESSB_RW_Pos)     	/*!< I2C_PERIPHERAL S0_ADDRESSB: RW Mask     */
#define I2C_PERIPHERAL_S0_ADDRESSB_ADDRESS_Pos  	1	                                                  /*!< I2C_PERIPHERAL S0_ADDRESSB: ADDRESS Position */
#define I2C_PERIPHERAL_S0_ADDRESSB_ADDRESS_Msk  	(0x3FFUL << I2C_PERIPHERAL_S0_ADDRESSB_ADDRESS_Pos) /*!< I2C_PERIPHERAL S0_ADDRESSB: ADDRESS Mask  */
#define I2C_PERIPHERAL_S0_ADDRESSB_A10MODE_Pos  	15                                                  /*!< I2C_PERIPHERAL S0_ADDRESSB: A10MODE Position */
#define I2C_PERIPHERAL_S0_ADDRESSB_A10MODE_Msk  	(0x01UL << I2C_PERIPHERAL_S0_ADDRESSB_A10MODE_Pos)  /*!< I2C_PERIPHERAL S0_ADDRESSB: A10MODE Mask  */

/* ---------------------------  I2C_PERIPHERAL_S0_ADDRESSMASKB  -------------------------- */
#define I2C_PERIPHERAL_S0_ADDRESSMASKB_RWMASK_Pos 0                                                    /*!< I2C_PERIPHERAL S0_ADDRESSMASKB: RWMASK Position */
#define I2C_PERIPHERAL_S0_ADDRESSMASKB_RWMASK_Msk (0x01UL << I2C_PERIPHERAL_S0_ADDRESSMASKB_RWMASK_Pos)/*!< I2C_PERIPHERAL S0_ADDRESSMASKB: RWMASK Mask     */
#define I2C_PERIPHERAL_S0_ADDRESSMASKB_MASK_Pos   1	                                                   /*!< I2C_PERIPHERAL S0_ADDRESSMASKB: MASK Position */
#define I2C_PERIPHERAL_S0_ADDRESSMASKB_MASK_Msk   (0x3FFUL << I2C_PERIPHERAL_S0_ADDRESSMASKB_MASK_Pos) /*!< I2C_PERIPHERAL S0_ADDRESSMASKB: MASK Mask  */

/* ================================================================================ */
/* ================          struct 'I2CA' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  I2CA_CTRL  --------------------------------- */
#define I2CA_CTRL_CLKENABLED_Pos              0                                                       /*!< I2CA CTRL: CLKENABLED Position          */
#define I2CA_CTRL_CLKENABLED_Msk              (0x01UL << I2CA_CTRL_CLKENABLED_Pos)                    /*!< I2CA CTRL: CLKENABLED Mask              */
#define I2CA_CTRL_ENABLED_Pos               	1                                                       /*!< I2CA CTRL: ENABLED Position             */
#define I2CA_CTRL_ENABLED_Msk               	(0x01UL << I2CA_CTRL_ENABLED_Pos)                     	/*!< I2CA CTRL: ENABLED Mask                 */
#define I2CA_CTRL_ENABLE_Pos                  2                                                       /*!< I2CA CTRL: ENABLE Position              */
#define I2CA_CTRL_ENABLE_Msk                  (0x01UL << I2CA_CTRL_ENABLE_Pos)                        /*!< I2CA CTRL: ENABLE Mask                  */
#define I2CA_CTRL_TXFEMD_Pos                  3                                                       /*!< I2CA CTRL: TXFEMD Position              */
#define I2CA_CTRL_TXFEMD_Msk                  (0x01UL << I2CA_CTRL_TXFEMD_Pos)                        /*!< I2CA CTRL: TXFEMD Mask                  */
#define I2CA_CTRL_RXFFMD_Pos                  4                                                       /*!< I2CA CTRL: RXFFMD Position              */
#define I2CA_CTRL_RXFFMD_Msk                  (0x01UL << I2CA_CTRL_RXFFMD_Pos)                        /*!< I2CA CTRL: RXFFMD Mask                  */
#define I2CA_CTRL_ALGFILTER_Pos               5                                                       /*!< I2CA CTRL: ALGFILTER Position           */
#define I2CA_CTRL_ALGFILTER_Msk               (0x01UL << I2CA_CTRL_ALGFILTER_Pos)                     /*!< I2CA CTRL: ALGFILTER Mask               */
#define I2CA_CTRL_DLGFILTER_Pos               6                                                       /*!< I2CA CTRL: DLGFILTER Position           */
#define I2CA_CTRL_DLGFILTER_Msk               (0x01UL << I2CA_CTRL_DLGFILTER_Pos)                     /*!< I2CA CTRL: DLGFILTER Mask               */
#define I2CA_CTRL_LOOPBACK_Pos                8                                                       /*!< I2CA CTRL: LOOPBACK Position            */
#define I2CA_CTRL_LOOPBACK_Msk                (0x01UL << I2CA_CTRL_LOOPBACK_Pos)                      /*!< I2CA CTRL: LOOPBACK Mask                */
#define I2CA_CTRL_TMCONFIGENB_Pos             9                                                       /*!< I2CA CTRL: TMCONFIGENB Position         */
#define I2CA_CTRL_TMCONFIGENB_Msk             (0x01UL << I2CA_CTRL_TMCONFIGENB_Pos)                   /*!< I2CA CTRL: TMCONFIGENB Mask             */

/* --------------------------------  I2CA_CLKSCALE  ------------------------------- */
#define I2CA_CLKSCALE_VALUE_Pos               0                                                       /*!< I2CA CLKSCALE: VALUE Position           */
#define I2CA_CLKSCALE_VALUE_Msk               (0x7fffffffUL << I2CA_CLKSCALE_VALUE_Pos)               /*!< I2CA CLKSCALE: VALUE Mask               */
#define I2CA_CLKSCALE_FASTMODE_Pos            31                                                      /*!< I2CA CLKSCALE: FASTMODE Position        */
#define I2CA_CLKSCALE_FASTMODE_Msk            (0x01UL << I2CA_CLKSCALE_FASTMODE_Pos)                  /*!< I2CA CLKSCALE: FASTMODE Mask            */

/* --------------------------------  I2CA_ADDRESS  -------------------------------- */
#define I2CA_ADDRESS_DIRECTION_Pos  					0                                              					/*!< I2CA ADDRESS: DIRECTION Position 			 */
#define I2CA_ADDRESS_DIRECTION_Msk  					(0x01UL << I2CA_ADDRESS_DIRECTION_Pos)   								/*!< I2CA ADDRESS: DIRECTION Mask     			 */
#define I2CA_ADDRESS_ADDRESS_Pos  						1	                                             					/*!< I2CA ADDRESS: ADDRESS Position 				 */
#define I2CA_ADDRESS_ADDRESS_Msk  						(0x3FFUL << I2CA_ADDRESS_ADDRESS_Pos)										/*!< I2CA ADDRESS: ADDRESS Mask  						 */
#define I2CA_ADDRESS_A10MODE_Pos  						15                                             					/*!< I2CA ADDRESS: A10MODE Position 				 */
#define I2CA_ADDRESS_A10MODE_Msk  						(0x01UL << I2CA_ADDRESS_A10MODE_Pos)										/*!< I2CA ADDRESS: A10MODE Mask  						 */

/* -------------------------------  I2CA_CMD  ------------------------------------ */
#define I2CA_CMD_START_Pos  									0                                                       /*!< I2CA CMD: START Position 							 */
#define I2CA_CMD_START_Msk  									(0x01UL << I2CA_CMD_START_Pos)     											/*!< I2CA CMD: START Mask     							 */
#define I2CA_CMD_STOP_Pos  										1	                                                      /*!< I2CA CMD: STOP Position 								 */
#define I2CA_CMD_STOP_Msk  										(0x01UL << I2CA_CMD_STOP_Pos)        									/*!< I2CA CMD: STOP Mask  									 */
#define I2CA_CMD_CANCEL_Pos  									2                                                      /*!< I2CA CMD: CANCEL Position 							 */
#define I2CA_CMD_CANCEL_Msk  									(0x01UL << I2CA_CMD_CANCEL_Pos)        									/*!< I2CA CMD: CANCEL Mask  								 */

/* ---------------------------------  I2CA_STATUS  -------------------------------- */
#define I2CA_STATUS_I2CIDLE_Pos               0                                                       /*!< I2CA STATUS: I2CIDLE Position           */
#define I2CA_STATUS_I2CIDLE_Msk               (0x01UL << I2CA_STATUS_I2CIDLE_Pos)                     /*!< I2CA STATUS: I2CIDLE Mask               */
#define I2CA_STATUS_IDLE_Pos                  1                                                       /*!< I2CA STATUS: IDLE Position              */
#define I2CA_STATUS_IDLE_Msk                  (0x01UL << I2CA_STATUS_IDLE_Pos)                        /*!< I2CA STATUS: IDLE Mask                  */
#define I2CA_STATUS_WAITING_Pos               2                                                       /*!< I2CA STATUS: WAITING Position           */
#define I2CA_STATUS_WAITING_Msk               (0x01UL << I2CA_STATUS_WAITING_Pos)                     /*!< I2CA STATUS: WAITING Mask               */
#define I2CA_STATUS_STALLED_Pos               3                                                       /*!< I2CA STATUS: STALLED Position           */
#define I2CA_STATUS_STALLED_Msk               (0x01UL << I2CA_STATUS_STALLED_Pos)                     /*!< I2CA STATUS: STALLED Mask               */
#define I2CA_STATUS_ARBLOST_Pos               4                                                       /*!< I2CA STATUS: ARBLOST Position           */
#define I2CA_STATUS_ARBLOST_Msk               (0x01UL << I2CA_STATUS_ARBLOST_Pos)                     /*!< I2CA STATUS: ARBLOST Mask               */
#define I2CA_STATUS_NACKADDR_Pos              5                                                       /*!< I2CA STATUS: NACKADDR Position          */
#define I2CA_STATUS_NACKADDR_Msk              (0x01UL << I2CA_STATUS_NACKADDR_Pos)                    /*!< I2CA STATUS: NACKADDR Mask              */
#define I2CA_STATUS_NACKDATA_Pos              6                                                       /*!< I2CA STATUS: NACKDATA Position          */
#define I2CA_STATUS_NACKDATA_Msk              (0x01UL << I2CA_STATUS_NACKDATA_Pos)                    /*!< I2CA STATUS: NACKDATA Mask              */
#define I2CA_STATUS_RXNEMPTY_Pos              8                                                       /*!< I2CA STATUS: RXNEMPTY Position          */
#define I2CA_STATUS_RXNEMPTY_Msk              (0x01UL << I2CA_STATUS_RXNEMPTY_Pos)                    /*!< I2CA STATUS: RXNEMPTY Mask              */
#define I2CA_STATUS_RXFULL_Pos                9                                                       /*!< I2CA STATUS: RXFULL Position            */
#define I2CA_STATUS_RXFULL_Msk                (0x01UL << I2CA_STATUS_RXFULL_Pos)                      /*!< I2CA STATUS: RXFULL Mask                */
#define I2CA_STATUS_RXTRIGGER_Pos              11                                                      /*!< I2CA STATUS: RXTRIGGER Position          */
#define I2CA_STATUS_RXTRIGGER_Msk              (0x01UL << I2CA_STATUS_RXTRIGGER_Pos)                    /*!< I2CA STATUS: RXTRIGGER Mask              */
#define I2CA_STATUS_TXEMPTY_Pos               12                                                      /*!< I2CA STATUS: TXEMPTY Position           */
#define I2CA_STATUS_TXEMPTY_Msk               (0x01UL << I2CA_STATUS_TXEMPTY_Pos)                     /*!< I2CA STATUS: TXEMPTY Mask               */
#define I2CA_STATUS_TXNFULL_Pos               13                                                      /*!< I2CA STATUS: TXNFULL Position           */
#define I2CA_STATUS_TXNFULL_Msk               (0x01UL << I2CA_STATUS_TXNFULL_Pos)                     /*!< I2CA STATUS: TXNFULL Mask               */
#define I2CA_STATUS_TXTRIGGER_Pos              15                                                      /*!< I2CA STATUS: TXTRIGGER Position          */
#define I2CA_STATUS_TXTRIGGER_Msk              (0x01UL << I2CA_STATUS_TXTRIGGER_Pos)                    /*!< I2CA STATUS: TXTRIGGER Mask              */
#define I2CA_STATUS_RAW_SDA_Pos               30                                                      /*!< I2CA STATUS: RAW_SDA Position           */
#define I2CA_STATUS_RAW_SDA_Msk               (0x01UL << I2CA_STATUS_RAW_SDA_Pos)                     /*!< I2CA STATUS: RAW_SDA Mask               */
#define I2CA_STATUS_RAW_SCL_Pos               31                                                      /*!< I2CA STATUS: RAW_SCL Position           */
#define I2CA_STATUS_RAW_SCL_Msk               (0x01UL << I2CA_STATUS_RAW_SCL_Pos)                     /*!< I2CA STATUS: RAW_SCL Mask               */

/* ---------------------------------  I2CA_STATE  ------------------------------ */
#define I2CA_STATE_STATE_Pos  								0                                                       /*!< I2CA STATE: STATE Position 						 */
#define I2CA_STATE_STATE_Msk  								(0x0FUL << I2CA_STATE_STATE_Pos)     				  					/*!< I2CA STATE: STATE Mask     					   */
#define I2CA_STATE_STEP_Pos  									4	                                                      /*!< I2CA STATE: STEP Position 							 */
#define I2CA_STATE_STEP_Msk  									(0x0FUL << I2CA_STATE_STEP_Pos)        			  					/*!< I2CA STATE: STEP Mask  							   */
#define I2CA_STATE_RXFIFO_Pos  								8                                                       /*!< I2CA STATE: RXFIFO Position 						 */
#define I2CA_STATE_RXFIFO_Msk  								(0x01FUL << I2CA_STATE_RXFIFO_Pos)        							/*!< I2CA STATE: RXFIFO Mask  							 */
#define I2CA_STATE_TXFIFO_Pos  								14                                                      /*!< I2CA STATE: TXFIFO Position 						 */
#define I2CA_STATE_TXFIFO_Msk  								(0x01FUL << I2CA_STATE_TXFIFO_Pos)     			  					/*!< I2CA STATE: TXFIFO Mask     						 */
#define I2CA_STATE_BITSTATE_Pos  							20                                                      /*!< I2CA STATE: BITSTATE Position 					 */
#define I2CA_STATE_BITSTATE_Msk  							(0x01FFUL << I2CA_STATE_BITSTATE_Pos)     							/*!< I2CA STATE: BITSTATE Mask     					 */

/* --------------------------------  I2CA_IRQ_ENB  -------------------------------- */
#define I2CA_IRQ_ENB_I2CIDLE_Pos              0                                                       /*!< I2CA IRQ_ENB: I2CIDLE Position          */
#define I2CA_IRQ_ENB_I2CIDLE_Msk              (0x01UL << I2CA_IRQ_ENB_I2CIDLE_Pos)                    /*!< I2CA IRQ_ENB: I2CIDLE Mask              */
#define I2CA_IRQ_ENB_IDLE_Pos                 1                                                       /*!< I2CA IRQ_ENB: IDLE Position             */
#define I2CA_IRQ_ENB_IDLE_Msk                 (0x01UL << I2CA_IRQ_ENB_IDLE_Pos)                       /*!< I2CA IRQ_ENB: IDLE Mask                 */
#define I2CA_IRQ_ENB_WAITING_Pos              2                                                       /*!< I2CA IRQ_ENB: WAITING Position          */
#define I2CA_IRQ_ENB_WAITING_Msk              (0x01UL << I2CA_IRQ_ENB_WAITING_Pos)                    /*!< I2CA IRQ_ENB: WAITING Mask              */
#define I2CA_IRQ_ENB_STALLED_Pos              3                                                       /*!< I2CA IRQ_ENB: STALLED Position          */
#define I2CA_IRQ_ENB_STALLED_Msk              (0x01UL << I2CA_IRQ_ENB_STALLED_Pos)                    /*!< I2CA IRQ_ENB: STALLED Mask              */
#define I2CA_IRQ_ENB_ARBLOST_Pos              4                                                       /*!< I2CA IRQ_ENB: ARBLOST Position          */
#define I2CA_IRQ_ENB_ARBLOST_Msk              (0x01UL << I2CA_IRQ_ENB_ARBLOST_Pos)                    /*!< I2CA IRQ_ENB: ARBLOST Mask              */
#define I2CA_IRQ_ENB_NACKADDR_Pos             5                                                       /*!< I2CA IRQ_ENB: NACKADDR Position         */
#define I2CA_IRQ_ENB_NACKADDR_Msk             (0x01UL << I2CA_IRQ_ENB_NACKADDR_Pos)                   /*!< I2CA IRQ_ENB: NACKADDR Mask             */
#define I2CA_IRQ_ENB_NACKDATA_Pos             6                                                       /*!< I2CA IRQ_ENB: NACKDATA Position         */
#define I2CA_IRQ_ENB_NACKDATA_Msk             (0x01UL << I2CA_IRQ_ENB_NACKDATA_Pos)                   /*!< I2CA IRQ_ENB: NACKDATA Mask             */
#define I2CA_IRQ_ENB_CLKLOTO_Pos              7                                                       /*!< I2CA IRQ_ENB: CLKLOTO Position          */
#define I2CA_IRQ_ENB_CLKLOTO_Msk              (0x01UL << I2CA_IRQ_ENB_CLKLOTO_Pos)                    /*!< I2CA IRQ_ENB: CLKLOTO Mask              */
#define I2CA_IRQ_ENB_TXOVERFLOW_Pos           10                                                      /*!< I2CA IRQ_ENB: TXOVERFLOW Position       */
#define I2CA_IRQ_ENB_TXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_ENB_TXOVERFLOW_Pos)                 /*!< I2CA IRQ_ENB: TXOVERFLOW Mask           */
#define I2CA_IRQ_ENB_RXOVERFLOW_Pos           11                                                      /*!< I2CA IRQ_ENB: RXOVERFLOW Position       */
#define I2CA_IRQ_ENB_RXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_ENB_RXOVERFLOW_Pos)                 /*!< I2CA IRQ_ENB: RXOVERFLOW Mask           */
#define I2CA_IRQ_ENB_TXREADY_Pos              12                                                      /*!< I2CA IRQ_ENB: TXREADY Position          */
#define I2CA_IRQ_ENB_TXREADY_Msk              (0x01UL << I2CA_IRQ_ENB_TXREADY_Pos)                    /*!< I2CA IRQ_ENB: TXREADY Mask              */
#define I2CA_IRQ_ENB_RXREADY_Pos              13                                                      /*!< I2CA IRQ_ENB: RXREADY Position          */
#define I2CA_IRQ_ENB_RXREADY_Msk              (0x01UL << I2CA_IRQ_ENB_RXREADY_Pos)                    /*!< I2CA IRQ_ENB: RXREADY Mask              */
#define I2CA_IRQ_ENB_TXEMPTY_Pos              14                                                      /*!< I2CA IRQ_ENB: TXEMPTY Position          */
#define I2CA_IRQ_ENB_TXEMPTY_Msk              (0x01UL << I2CA_IRQ_ENB_TXEMPTY_Pos)                    /*!< I2CA IRQ_ENB: TXEMPTY Mask              */
#define I2CA_IRQ_ENB_RXFULL_Pos               15                                                      /*!< I2CA IRQ_ENB: RXFULL Position           */
#define I2CA_IRQ_ENB_RXFULL_Msk               (0x01UL << I2CA_IRQ_ENB_RXFULL_Pos)                     /*!< I2CA IRQ_ENB: RXFULL Mask               */

/* --------------------------------  I2CA_IRQ_RAW  -------------------------------- */
#define I2CA_IRQ_RAW_I2CIDLE_Pos              0                                                       /*!< I2CA IRQ_RAW: I2CIDLE Position          */
#define I2CA_IRQ_RAW_I2CIDLE_Msk              (0x01UL << I2CA_IRQ_RAW_I2CIDLE_Pos)                    /*!< I2CA IRQ_RAW: I2CIDLE Mask              */
#define I2CA_IRQ_RAW_IDLE_Pos                 1                                                       /*!< I2CA IRQ_RAW: IDLE Position             */
#define I2CA_IRQ_RAW_IDLE_Msk                 (0x01UL << I2CA_IRQ_RAW_IDLE_Pos)                       /*!< I2CA IRQ_RAW: IDLE Mask                 */
#define I2CA_IRQ_RAW_WAITING_Pos              2                                                       /*!< I2CA IRQ_RAW: WAITING Position          */
#define I2CA_IRQ_RAW_WAITING_Msk              (0x01UL << I2CA_IRQ_RAW_WAITING_Pos)                    /*!< I2CA IRQ_RAW: WAITING Mask              */
#define I2CA_IRQ_RAW_STALLED_Pos              3                                                       /*!< I2CA IRQ_RAW: STALLED Position          */
#define I2CA_IRQ_RAW_STALLED_Msk              (0x01UL << I2CA_IRQ_RAW_STALLED_Pos)                    /*!< I2CA IRQ_RAW: STALLED Mask              */
#define I2CA_IRQ_RAW_ARBLOST_Pos              4                                                       /*!< I2CA IRQ_RAW: ARBLOST Position          */
#define I2CA_IRQ_RAW_ARBLOST_Msk              (0x01UL << I2CA_IRQ_RAW_ARBLOST_Pos)                    /*!< I2CA IRQ_RAW: ARBLOST Mask              */
#define I2CA_IRQ_RAW_NACKADDR_Pos             5                                                       /*!< I2CA IRQ_RAW: NACKADDR Position         */
#define I2CA_IRQ_RAW_NACKADDR_Msk             (0x01UL << I2CA_IRQ_RAW_NACKADDR_Pos)                   /*!< I2CA IRQ_RAW: NACKADDR Mask             */
#define I2CA_IRQ_RAW_NACKDATA_Pos             6                                                       /*!< I2CA IRQ_RAW: NACKDATA Position         */
#define I2CA_IRQ_RAW_NACKDATA_Msk             (0x01UL << I2CA_IRQ_RAW_NACKDATA_Pos)                   /*!< I2CA IRQ_RAW: NACKDATA Mask             */
#define I2CA_IRQ_RAW_CLKLOTO_Pos              7                                                       /*!< I2CA IRQ_RAW: CLKLOTO Position          */
#define I2CA_IRQ_RAW_CLKLOTO_Msk              (0x01UL << I2CA_IRQ_RAW_CLKLOTO_Pos)                    /*!< I2CA IRQ_RAW: CLKLOTO Mask              */
#define I2CA_IRQ_RAW_TXOVERFLOW_Pos           10                                                      /*!< I2CA IRQ_RAW: TXOVERFLOW Position       */
#define I2CA_IRQ_RAW_TXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_RAW_TXOVERFLOW_Pos)                 /*!< I2CA IRQ_RAW: TXOVERFLOW Mask           */
#define I2CA_IRQ_RAW_RXOVERFLOW_Pos           11                                                      /*!< I2CA IRQ_RAW: RXOVERFLOW Position       */
#define I2CA_IRQ_RAW_RXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_RAW_RXOVERFLOW_Pos)                 /*!< I2CA IRQ_RAW: RXOVERFLOW Mask           */
#define I2CA_IRQ_RAW_TXREADY_Pos              12                                                      /*!< I2CA IRQ_RAW: TXREADY Position          */
#define I2CA_IRQ_RAW_TXREADY_Msk              (0x01UL << I2CA_IRQ_RAW_TXREADY_Pos)                    /*!< I2CA IRQ_RAW: TXREADY Mask              */
#define I2CA_IRQ_RAW_RXREADY_Pos              13                                                      /*!< I2CA IRQ_RAW: RXREADY Position          */
#define I2CA_IRQ_RAW_RXREADY_Msk              (0x01UL << I2CA_IRQ_RAW_RXREADY_Pos)                    /*!< I2CA IRQ_RAW: RXREADY Mask              */
#define I2CA_IRQ_RAW_TXEMPTY_Pos              14                                                      /*!< I2CA IRQ_RAW: TXEMPTY Position          */
#define I2CA_IRQ_RAW_TXEMPTY_Msk              (0x01UL << I2CA_IRQ_RAW_TXEMPTY_Pos)                    /*!< I2CA IRQ_RAW: TXEMPTY Mask              */
#define I2CA_IRQ_RAW_RXFULL_Pos               15                                                      /*!< I2CA IRQ_RAW: RXFULL Position           */
#define I2CA_IRQ_RAW_RXFULL_Msk               (0x01UL << I2CA_IRQ_RAW_RXFULL_Pos)                     /*!< I2CA IRQ_RAW: RXFULL Mask               */

/* --------------------------------  I2CA_IRQ_END  -------------------------------- */
#define I2CA_IRQ_END_I2CIDLE_Pos              0                                                       /*!< I2CA IRQ_END: I2CIDLE Position          */
#define I2CA_IRQ_END_I2CIDLE_Msk              (0x01UL << I2CA_IRQ_END_I2CIDLE_Pos)                    /*!< I2CA IRQ_END: I2CIDLE Mask              */
#define I2CA_IRQ_END_IDLE_Pos                 1                                                       /*!< I2CA IRQ_END: IDLE Position             */
#define I2CA_IRQ_END_IDLE_Msk                 (0x01UL << I2CA_IRQ_END_IDLE_Pos)                       /*!< I2CA IRQ_END: IDLE Mask                 */
#define I2CA_IRQ_END_WAITING_Pos              2                                                       /*!< I2CA IRQ_END: WAITING Position          */
#define I2CA_IRQ_END_WAITING_Msk              (0x01UL << I2CA_IRQ_END_WAITING_Pos)                    /*!< I2CA IRQ_END: WAITING Mask              */
#define I2CA_IRQ_END_STALLED_Pos              3                                                       /*!< I2CA IRQ_END: STALLED Position          */
#define I2CA_IRQ_END_STALLED_Msk              (0x01UL << I2CA_IRQ_END_STALLED_Pos)                    /*!< I2CA IRQ_END: STALLED Mask              */
#define I2CA_IRQ_END_ARBLOST_Pos              4                                                       /*!< I2CA IRQ_END: ARBLOST Position          */
#define I2CA_IRQ_END_ARBLOST_Msk              (0x01UL << I2CA_IRQ_END_ARBLOST_Pos)                    /*!< I2CA IRQ_END: ARBLOST Mask              */
#define I2CA_IRQ_END_NACKADDR_Pos             5                                                       /*!< I2CA IRQ_END: NACKADDR Position         */
#define I2CA_IRQ_END_NACKADDR_Msk             (0x01UL << I2CA_IRQ_END_NACKADDR_Pos)                   /*!< I2CA IRQ_END: NACKADDR Mask             */
#define I2CA_IRQ_END_NACKDATA_Pos             6                                                       /*!< I2CA IRQ_END: NACKDATA Position         */
#define I2CA_IRQ_END_NACKDATA_Msk             (0x01UL << I2CA_IRQ_END_NACKDATA_Pos)                   /*!< I2CA IRQ_END: NACKDATA Mask             */
#define I2CA_IRQ_END_CLKLOTO_Pos              7                                                       /*!< I2CA IRQ_END: CLKLOTO Position          */
#define I2CA_IRQ_END_CLKLOTO_Msk              (0x01UL << I2CA_IRQ_END_CLKLOTO_Pos)                    /*!< I2CA IRQ_END: CLKLOTO Mask              */
#define I2CA_IRQ_END_TXOVERFLOW_Pos           10                                                      /*!< I2CA IRQ_END: TXOVERFLOW Position       */
#define I2CA_IRQ_END_TXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_END_TXOVERFLOW_Pos)                 /*!< I2CA IRQ_END: TXOVERFLOW Mask           */
#define I2CA_IRQ_END_RXOVERFLOW_Pos           11                                                      /*!< I2CA IRQ_END: RXOVERFLOW Position       */
#define I2CA_IRQ_END_RXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_END_RXOVERFLOW_Pos)                 /*!< I2CA IRQ_END: RXOVERFLOW Mask           */
#define I2CA_IRQ_END_TXREADY_Pos              12                                                      /*!< I2CA IRQ_END: TXREADY Position          */
#define I2CA_IRQ_END_TXREADY_Msk              (0x01UL << I2CA_IRQ_END_TXREADY_Pos)                    /*!< I2CA IRQ_END: TXREADY Mask              */
#define I2CA_IRQ_END_RXREADY_Pos              13                                                      /*!< I2CA IRQ_END: RXREADY Position          */
#define I2CA_IRQ_END_RXREADY_Msk              (0x01UL << I2CA_IRQ_END_RXREADY_Pos)                    /*!< I2CA IRQ_END: RXREADY Mask              */
#define I2CA_IRQ_END_TXEMPTY_Pos              14                                                      /*!< I2CA IRQ_END: TXEMPTY Position          */
#define I2CA_IRQ_END_TXEMPTY_Msk              (0x01UL << I2CA_IRQ_END_TXEMPTY_Pos)                    /*!< I2CA IRQ_END: TXEMPTY Mask              */
#define I2CA_IRQ_END_RXFULL_Pos               15                                                      /*!< I2CA IRQ_END: RXFULL Position           */
#define I2CA_IRQ_END_RXFULL_Msk               (0x01UL << I2CA_IRQ_END_RXFULL_Pos)                     /*!< I2CA IRQ_END: RXFULL Mask               */

/* --------------------------------  I2CA_IRQ_CLR  -------------------------------- */
#define I2CA_IRQ_CLR_I2CIDLE_Pos              0                                                       /*!< I2CA IRQ_CLR: I2CIDLE Position          */
#define I2CA_IRQ_CLR_I2CIDLE_Msk              (0x01UL << I2CA_IRQ_CLR_I2CIDLE_Pos)                    /*!< I2CA IRQ_CLR: I2CIDLE Mask              */
#define I2CA_IRQ_CLR_IDLE_Pos                 1                                                       /*!< I2CA IRQ_CLR: IDLE Position             */
#define I2CA_IRQ_CLR_IDLE_Msk                 (0x01UL << I2CA_IRQ_CLR_IDLE_Pos)                       /*!< I2CA IRQ_CLR: IDLE Mask                 */
#define I2CA_IRQ_CLR_WAITING_Pos              2                                                       /*!< I2CA IRQ_CLR: WAITING Position          */
#define I2CA_IRQ_CLR_WAITING_Msk              (0x01UL << I2CA_IRQ_CLR_WAITING_Pos)                    /*!< I2CA IRQ_CLR: WAITING Mask              */
#define I2CA_IRQ_CLR_STALLED_Pos              3                                                       /*!< I2CA IRQ_CLR: STALLED Position          */
#define I2CA_IRQ_CLR_STALLED_Msk              (0x01UL << I2CA_IRQ_CLR_STALLED_Pos)                    /*!< I2CA IRQ_CLR: STALLED Mask              */
#define I2CA_IRQ_CLR_ARBLOST_Pos              4                                                       /*!< I2CA IRQ_CLR: ARBLOST Position          */
#define I2CA_IRQ_CLR_ARBLOST_Msk              (0x01UL << I2CA_IRQ_CLR_ARBLOST_Pos)                    /*!< I2CA IRQ_CLR: ARBLOST Mask              */
#define I2CA_IRQ_CLR_NACKADDR_Pos             5                                                       /*!< I2CA IRQ_CLR: NACKADDR Position         */
#define I2CA_IRQ_CLR_NACKADDR_Msk             (0x01UL << I2CA_IRQ_CLR_NACKADDR_Pos)                   /*!< I2CA IRQ_CLR: NACKADDR Mask             */
#define I2CA_IRQ_CLR_NACKDATA_Pos             6                                                       /*!< I2CA IRQ_CLR: NACKDATA Position         */
#define I2CA_IRQ_CLR_NACKDATA_Msk             (0x01UL << I2CA_IRQ_CLR_NACKDATA_Pos)                   /*!< I2CA IRQ_CLR: NACKDATA Mask             */
#define I2CA_IRQ_CLR_CLKLOTO_Pos              7                                                       /*!< I2CA IRQ_CLR: CLKLOTO Position          */
#define I2CA_IRQ_CLR_CLKLOTO_Msk              (0x01UL << I2CA_IRQ_CLR_CLKLOTO_Pos)                    /*!< I2CA IRQ_CLR: CLKLOTO Mask              */
#define I2CA_IRQ_CLR_TXOVERFLOW_Pos           10                                                      /*!< I2CA IRQ_CLR: TXOVERFLOW Position       */
#define I2CA_IRQ_CLR_TXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_CLR_TXOVERFLOW_Pos)                 /*!< I2CA IRQ_CLR: TXOVERFLOW Mask           */
#define I2CA_IRQ_CLR_RXOVERFLOW_Pos           11                                                      /*!< I2CA IRQ_CLR: RXOVERFLOW Position       */
#define I2CA_IRQ_CLR_RXOVERFLOW_Msk           (0x01UL << I2CA_IRQ_CLR_RXOVERFLOW_Pos)                 /*!< I2CA IRQ_CLR: RXOVERFLOW Mask           */
#define I2CA_IRQ_CLR_TXREADY_Pos              12                                                      /*!< I2CA IRQ_CLR: TXREADY Position          */
#define I2CA_IRQ_CLR_TXREADY_Msk              (0x01UL << I2CA_IRQ_CLR_TXREADY_Pos)                    /*!< I2CA IRQ_CLR: TXREADY Mask              */
#define I2CA_IRQ_CLR_RXREADY_Pos              13                                                      /*!< I2CA IRQ_CLR: RXREADY Position          */
#define I2CA_IRQ_CLR_RXREADY_Msk              (0x01UL << I2CA_IRQ_CLR_RXREADY_Pos)                    /*!< I2CA IRQ_CLR: RXREADY Mask              */
#define I2CA_IRQ_CLR_TXEMPTY_Pos              14                                                      /*!< I2CA IRQ_CLR: TXEMPTY Position          */
#define I2CA_IRQ_CLR_TXEMPTY_Msk              (0x01UL << I2CA_IRQ_CLR_TXEMPTY_Pos)                    /*!< I2CA IRQ_CLR: TXEMPTY Mask              */
#define I2CA_IRQ_CLR_RXFULL_Pos               15                                                      /*!< I2CA IRQ_CLR: RXFULL Position           */
#define I2CA_IRQ_CLR_RXFULL_Msk               (0x01UL << I2CA_IRQ_CLR_RXFULL_Pos)                     /*!< I2CA IRQ_CLR: RXFULL Mask               */

/* --------------------------------  I2CA_FIFO_CLR  ------------------------------- */
#define I2CA_FIFO_CLR_RXFIFO_Pos              0                                                       /*!< I2CA FIFO_CLR: RXFIFO Position          */
#define I2CA_FIFO_CLR_RXFIFO_Msk              (0x01UL << I2CA_FIFO_CLR_RXFIFO_Pos)                    /*!< I2CA FIFO_CLR: RXFIFO Mask              */
#define I2CA_FIFO_CLR_TXFIFO_Pos              1                                                       /*!< I2CA FIFO_CLR: TXFIFO Position          */
#define I2CA_FIFO_CLR_TXFIFO_Msk              (0x01UL << I2CA_FIFO_CLR_TXFIFO_Pos)                    /*!< I2CA FIFO_CLR: TXFIFO Mask              */

/* --------------------------------  I2CA_S0_CTRL  -------------------------------- */
#define I2CB_S0_CTRL_CLKENABLED_Pos            0                                                       /*!< I2CB S0_CTRL: ENABLED Position          */
#define I2CB_S0_CTRL_CLKENABLED_Msk           (0x01UL << I2CB_S0_CTRL_CLKENABLED_Pos)                    /*!< I2CB S0_CTRL: ENABLED Mask              */
#define I2CB_S0_CTRL_ENABLED_Pos            1                                                       /*!< I2CB S0_CTRL: ACTIVATED Position        */
#define I2CB_S0_CTRL_ENABLED_Msk            (0x01UL << I2CB_S0_CTRL_ENABLED_Pos)                  /*!< I2CB S0_CTRL: ACTIVATED Mask            */
#define I2CB_S0_CTRL_ENABLE_Pos               2                                                       /*!< I2CB S0_CTRL: ACTIVE Position           */
#define I2CB_S0_CTRL_ENABLE_Msk               (0x01UL << I2CB_S0_CTRL_ENABLE_Pos)                     /*!< I2CB S0_CTRL: ACTIVE Mask               */
#define I2CA_S0_CTRL_TXFEMD_Pos               3                                                       /*!< I2CA S0_CTRL: TXFEMD Position           */
#define I2CA_S0_CTRL_TXFEMD_Msk               (0x01UL << I2CA_S0_CTRL_TXFEMD_Pos)                     /*!< I2CA S0_CTRL: TXFEMD Mask               */
#define I2CA_S0_CTRL_RXFFMD_Pos               4                                                       /*!< I2CA S0_CTRL: RXFFMD Position           */
#define I2CA_S0_CTRL_RXFFMD_Msk               (0x01UL << I2CA_S0_CTRL_RXFFMD_Pos)                     /*!< I2CA S0_CTRL: RXFFMD Mask               */

/* --------------------------------  I2CA_S0_MAXWORDS  --------------------------- */
#define I2CA_S0_MAXWORDS_MAXWORDS_Pos  				0                                                    		/*!< I2CA S0_MAXWORDS: MAXWORDS Position 		 */
#define I2CA_S0_MAXWORDS_MAXWORDS_Msk  				(0x07FFUL << I2CA_S0_MAXWORDS_MAXWORDS_Pos)							/*!< I2CA S0_MAXWORDS: MAXWORDS Mask    		 */
#define I2CA_S0_MAXWORDS_ENABLE_Pos  	 				31                                                   		/*!< I2CA S0_MAXWORDS: ENABLE Position 			 */
#define I2CA_S0_MAXWORDS_ENABLE_Msk  	 				(0x01UL << I2CA_S0_MAXWORDS_ENABLE_Pos) 								/*!< I2CA S0_MAXWORDS: ENABLE Mask  				 */

/* --------------------------------  I2CA_S0_ADDRESS  -------------------------- */
#define I2CA_S0_ADDRESS_RW_Pos  							0                                                     	/*!< I2CA S0_ADDRESS: RW Position 					 */
#define I2CA_S0_ADDRESS_RW_Msk  							(0x01UL << I2CA_S0_ADDRESS_RW_Pos)     									/*!< I2CA S0_ADDRESS: RW Mask     					 */
#define I2CA_S0_ADDRESS_ADDRESS_Pos  					1	                                                    	/*!< I2CA S0_ADDRESS: ADDRESS Position 			 */
#define I2CA_S0_ADDRESS_ADDRESS_Msk  					(0x3FFUL << I2CA_S0_ADDRESS_ADDRESS_Pos)    						/*!< I2CA S0_ADDRESS: ADDRESS Mask  				 */
#define I2CA_S0_ADDRESS_A10MODE_Pos  					15                                                    	/*!< I2CA S0_ADDRESS: A10MODE Position 			 */
#define I2CA_S0_ADDRESS_A10MODE_Msk  					(0x01UL << I2CA_S0_ADDRESS_A10MODE_Pos)     						/*!< I2CA S0_ADDRESS: A10MODE Mask  				 */

/* -------------------------------  I2CA_S0_ADDRESSMASK  -------------------------- */
#define I2CA_S0_ADDRESSMASK_RWMASK_Pos 				0                                                     	/*!< I2CA S0_ADDRESSMASK: RWMASK Position 	 */
#define I2CA_S0_ADDRESSMASK_RWMASK_Msk 				(0x01UL << I2CA_S0_ADDRESSMASK_RWMASK_Pos)  						/*!< I2CA S0_ADDRESSMASK: RWMASK Mask     	 */
#define I2CA_S0_ADDRESSMASK_MASK_Pos   				1	                                                   		/*!< I2CA S0_ADDRESSMASK: MASK Position 		 */
#define I2CA_S0_ADDRESSMASK_MASK_Msk   				(0x3FFUL << I2CA_S0_ADDRESSMASK_MASK_Pos)   						/*!< I2CA S0_ADDRESSMASK: MASK Mask  				 */

/* -------------------------------  I2CA_S0_STATUS  ------------------------------- */
#define I2CA_S0_STATUS_COMPLETED_Pos          0                                                       /*!< I2CA S0_STATUS: COMPLETED Position      */
#define I2CA_S0_STATUS_COMPLETED_Msk          (0x01UL << I2CA_S0_STATUS_COMPLETED_Pos)                /*!< I2CA S0_STATUS: COMPLETED Mask          */
#define I2CA_S0_STATUS_IDLE_Pos               1                                                       /*!< I2CA S0_STATUS: IDLE Position           */
#define I2CA_S0_STATUS_IDLE_Msk               (0x01UL << I2CA_S0_STATUS_IDLE_Pos)                     /*!< I2CA S0_STATUS: IDLE Mask               */
#define I2CA_S0_STATUS_WAITING_Pos            2                                                       /*!< I2CA S0_STATUS: WAITING Position        */
#define I2CA_S0_STATUS_WAITING_Msk            (0x01UL << I2CA_S0_STATUS_WAITING_Pos)                  /*!< I2CA S0_STATUS: WAITING Mask            */
#define I2CA_S0_STATUS_TXSTALLED_Pos          3                                                       /*!< I2CA S0_STATUS: TXSTALLED Position      */
#define I2CA_S0_STATUS_TXSTALLED_Msk          (0x01UL << I2CA_S0_STATUS_TXSTALLED_Pos)                /*!< I2CA S0_STATUS: TXSTALLED Mask          */
#define I2CA_S0_STATUS_RXSTALLED_Pos          4                                                       /*!< I2CA S0_STATUS: RXSTALLED Position      */
#define I2CA_S0_STATUS_RXSTALLED_Msk          (0x01UL << I2CA_S0_STATUS_RXSTALLED_Pos)                /*!< I2CA S0_STATUS: RXSTALLED Mask          */
#define I2CA_S0_STATUS_ADDRESSMATCH_Pos       5                                                       /*!< I2CA S0_STATUS: ADDRESSMATCH Position   */
#define I2CA_S0_STATUS_ADDRESSMATCH_Msk       (0x01UL << I2CA_S0_STATUS_ADDRESSMATCH_Pos)             /*!< I2CA S0_STATUS: ADDRESSMATCH Mask       */
#define I2CA_S0_STATUS_NACKDATA_Pos           6                                                       /*!< I2CA S0_STATUS: NACKDATA Position       */
#define I2CA_S0_STATUS_NACKDATA_Msk           (0x01UL << I2CA_S0_STATUS_NACKDATA_Pos)                 /*!< I2CA S0_STATUS: NACKDATA Mask           */
#define I2CA_S0_STATUS_RXDATAFIRST_Pos        7                                                       /*!< I2CA S0_STATUS: RXDATAFIRST Position    */
#define I2CA_S0_STATUS_RXDATAFIRST_Msk        (0x01UL << I2CA_S0_STATUS_RXDATAFIRST_Pos)              /*!< I2CA S0_STATUS: RXDATAFIRST Mask        */
#define I2CA_S0_STATUS_RXNEMPTY_Pos           8                                                       /*!< I2CA S0_STATUS: RXNEMPTY Position       */
#define I2CA_S0_STATUS_RXNEMPTY_Msk           (0x01UL << I2CA_S0_STATUS_RXNEMPTY_Pos)                 /*!< I2CA S0_STATUS: RXNEMPTY Mask           */
#define I2CA_S0_STATUS_RXFULL_Pos             9                                                       /*!< I2CA S0_STATUS: RXFULL Position         */
#define I2CA_S0_STATUS_RXFULL_Msk             (0x01UL << I2CA_S0_STATUS_RXFULL_Pos)                   /*!< I2CA S0_STATUS: RXFULL Mask             */
#define I2CA_S0_STATUS_RXTRIGGER_Pos           11                                                      /*!< I2CA S0_STATUS: RXTRIGGER Position       */
#define I2CA_S0_STATUS_RXTRIGGER_Msk           (0x01UL << I2CA_S0_STATUS_RXTRIGGER_Pos)                 /*!< I2CA S0_STATUS: RXTRIGGER Mask           */
#define I2CA_S0_STATUS_TXEMPTY_Pos            12                                                      /*!< I2CA S0_STATUS: TXEMPTY Position        */
#define I2CA_S0_STATUS_TXEMPTY_Msk            (0x01UL << I2CA_S0_STATUS_TXEMPTY_Pos)                  /*!< I2CA S0_STATUS: TXEMPTY Mask            */
#define I2CA_S0_STATUS_TXNFULL_Pos            13                                                      /*!< I2CA S0_STATUS: TXNFULL Position        */
#define I2CA_S0_STATUS_TXNFULL_Msk            (0x01UL << I2CA_S0_STATUS_TXNFULL_Pos)                  /*!< I2CA S0_STATUS: TXNFULL Mask            */
#define I2CA_S0_STATUS_TXTRIGGER_Pos           15                                                      /*!< I2CA S0_STATUS: TXTRIGGER Position       */
#define I2CA_S0_STATUS_TXTRIGGER_Msk           (0x01UL << I2CA_S0_STATUS_TXTRIGGER_Pos)                 /*!< I2CA S0_STATUS: TXTRIGGER Mask           */
#define I2CA_S0_STATUS_RAW_BUSY_Pos           29                                                      /*!< I2CA S0_STATUS: RAW_BUSY Position       */
#define I2CA_S0_STATUS_RAW_BUSY_Msk           (0x01UL << I2CA_S0_STATUS_RAW_BUSY_Pos)                 /*!< I2CA S0_STATUS: RAW_BUSY Mask           */
#define I2CA_S0_STATUS_RAW_SDA_Pos            30                                                      /*!< I2CA S0_STATUS: RAW_SDA Position        */
#define I2CA_S0_STATUS_RAW_SDA_Msk            (0x01UL << I2CA_S0_STATUS_RAW_SDA_Pos)                  /*!< I2CA S0_STATUS: RAW_SDA Mask            */
#define I2CA_S0_STATUS_RAW_SCL_Pos            31                                                      /*!< I2CA S0_STATUS: RAW_SCL Position        */
#define I2CA_S0_STATUS_RAW_SCL_Msk            (0x01UL << I2CA_S0_STATUS_RAW_SCL_Pos)                  /*!< I2CA S0_STATUS: RAW_SCL Mask            */

/* ---------------------------  I2CA_S0_STATE  ------------------------------ */
#define I2CA_S0_STATE_STATE_Pos  							0                                                       /*!< I2CA S0_STATE: STATE Position 					 */
#define I2CA_S0_STATE_STATE_Msk  							(0x07UL << I2CA_S0_STATE_STATE_Pos)     								/*!< I2CA S0_STATE: STATE Mask     					 */
#define I2CA_S0_STATE_STEP_Pos  							4	                                                      /*!< I2CA S0_STATE: STEP Position 					 */
#define I2CA_S0_STATE_STEP_Msk  							(0x0FUL << I2CA_S0_STATE_STEP_Pos)        							/*!< I2CA S0_STATE: STEP Mask  							 */
#define I2CA_S0_STATE_RXFIFO_Pos  						8                                                       /*!< I2CA S0_STATE: RXFIFO Position 				 */
#define I2CA_S0_STATE_RXFIFO_Msk  						(0x01FUL << I2CA_S0_STATE_RXFIFO_Pos)        						/*!< I2CA S0_STATE: RXFIFO Mask  						 */
#define I2CA_S0_STATE_TXFIFO_Pos  						14                                                      /*!< I2CA S0_STATE: TXFIFO Position 				 */
#define I2CA_S0_STATE_TXFIFO_Msk  						(0x01FUL << I2CA_S0_STATE_TXFIFO_Pos)     							/*!< I2CA S0_STATE: TXFIFO Mask     				 */

/* -------------------------------  I2CA_S0_IRQ_ENB  ------------------------------ */
#define I2CA_S0_IRQ_ENB_COMPLETED_Pos         0                                                       /*!< I2CA S0_IRQ_ENB: COMPLETED Position     */
#define I2CA_S0_IRQ_ENB_COMPLETED_Msk         (0x01UL << I2CA_S0_IRQ_ENB_COMPLETED_Pos)               /*!< I2CA S0_IRQ_ENB: COMPLETED Mask         */
#define I2CA_S0_IRQ_ENB_IDLE_Pos              1                                                       /*!< I2CA S0_IRQ_ENB: IDLE Position          */
#define I2CA_S0_IRQ_ENB_IDLE_Msk              (0x01UL << I2CA_S0_IRQ_ENB_IDLE_Pos)                    /*!< I2CA S0_IRQ_ENB: IDLE Mask              */
#define I2CA_S0_IRQ_ENB_WAITING_Pos           2                                                       /*!< I2CA S0_IRQ_ENB: WAITING Position       */
#define I2CA_S0_IRQ_ENB_WAITING_Msk           (0x01UL << I2CA_S0_IRQ_ENB_WAITING_Pos)                 /*!< I2CA S0_IRQ_ENB: WAITING Mask           */
#define I2CA_S0_IRQ_ENB_TXSTALLED_Pos         3                                                       /*!< I2CA S0_IRQ_ENB: TXSTALLED Position     */
#define I2CA_S0_IRQ_ENB_TXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_ENB_TXSTALLED_Pos)               /*!< I2CA S0_IRQ_ENB: TXSTALLED Mask         */
#define I2CA_S0_IRQ_ENB_RXSTALLED_Pos         4                                                       /*!< I2CA S0_IRQ_ENB: RXSTALLED Position     */
#define I2CA_S0_IRQ_ENB_RXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_ENB_RXSTALLED_Pos)               /*!< I2CA S0_IRQ_ENB: RXSTALLED Mask         */
#define I2CA_S0_IRQ_ENB_ADDRESSMATCH_Pos      5                                                       /*!< I2CA S0_IRQ_ENB: ADDRESSMATCH Position  */
#define I2CA_S0_IRQ_ENB_ADDRESSMATCH_Msk      (0x01UL << I2CA_S0_IRQ_ENB_ADDRESSMATCH_Pos)            /*!< I2CA S0_IRQ_ENB: ADDRESSMATCH Mask      */
#define I2CA_S0_IRQ_ENB_NACKDATA_Pos          6                                                       /*!< I2CA S0_IRQ_ENB: NACKDATA Position      */
#define I2CA_S0_IRQ_ENB_NACKDATA_Msk          (0x01UL << I2CA_S0_IRQ_ENB_NACKDATA_Pos)                /*!< I2CA S0_IRQ_ENB: NACKDATA Mask          */
#define I2CA_S0_IRQ_ENB_RXDATAFIRST_Pos       7                                                       /*!< I2CA S0_IRQ_ENB: RXDATAFIRST Position   */
#define I2CA_S0_IRQ_ENB_RXDATAFIRST_Msk       (0x01UL << I2CA_S0_IRQ_ENB_RXDATAFIRST_Pos)             /*!< I2CA S0_IRQ_ENB: RXDATAFIRST Mask       */
#define I2CA_S0_IRQ_ENB_I2C_START_Pos         8                                                       /*!< I2CA S0_IRQ_ENB: I2C_START Position     */
#define I2CA_S0_IRQ_ENB_I2C_START_Msk         (0x01UL << I2CA_S0_IRQ_ENB_I2C_START_Pos)               /*!< I2CA S0_IRQ_ENB: I2C_START Mask         */
#define I2CA_S0_IRQ_ENB_I2C_STOP_Pos          9                                                       /*!< I2CA S0_IRQ_ENB: I2C_STOP Position      */
#define I2CA_S0_IRQ_ENB_I2C_STOP_Msk          (0x01UL << I2CA_S0_IRQ_ENB_I2C_STOP_Pos)                /*!< I2CA S0_IRQ_ENB: I2C_STOP Mask          */
#define I2CA_S0_IRQ_ENB_TXUNDERFLOW_Pos       10                                                      /*!< I2CA S0_IRQ_ENB: TXUNDERFLOW Position   */
#define I2CA_S0_IRQ_ENB_TXUNDERFLOW_Msk       (0x01UL << I2CA_S0_IRQ_ENB_TXUNDERFLOW_Pos)             /*!< I2CA S0_IRQ_ENB: TXUNDERFLOW Mask       */
#define I2CA_S0_IRQ_ENB_RXOVERFLOW_Pos        11                                                      /*!< I2CA S0_IRQ_ENB: RXOVERFLOW Position    */
#define I2CA_S0_IRQ_ENB_RXOVERFLOW_Msk        (0x01UL << I2CA_S0_IRQ_ENB_RXOVERFLOW_Pos)              /*!< I2CA S0_IRQ_ENB: RXOVERFLOW Mask        */
#define I2CA_S0_IRQ_ENB_TXREADY_Pos           12                                                      /*!< I2CA S0_IRQ_ENB: TXREADY Position       */
#define I2CA_S0_IRQ_ENB_TXREADY_Msk           (0x01UL << I2CA_S0_IRQ_ENB_TXREADY_Pos)                 /*!< I2CA S0_IRQ_ENB: TXREADY Mask           */
#define I2CA_S0_IRQ_ENB_RXREADY_Pos           13                                                      /*!< I2CA S0_IRQ_ENB: RXREADY Position       */
#define I2CA_S0_IRQ_ENB_RXREADY_Msk           (0x01UL << I2CA_S0_IRQ_ENB_RXREADY_Pos)                 /*!< I2CA S0_IRQ_ENB: RXREADY Mask           */
#define I2CA_S0_IRQ_ENB_TXEMPTY_Pos           14                                                      /*!< I2CA S0_IRQ_ENB: TXEMPTY Position       */
#define I2CA_S0_IRQ_ENB_TXEMPTY_Msk           (0x01UL << I2CA_S0_IRQ_ENB_TXEMPTY_Pos)                 /*!< I2CA S0_IRQ_ENB: TXEMPTY Mask           */
#define I2CA_S0_IRQ_ENB_RXFULL_Pos            15                                                      /*!< I2CA S0_IRQ_ENB: RXFULL Position        */
#define I2CA_S0_IRQ_ENB_RXFULL_Msk            (0x01UL << I2CA_S0_IRQ_ENB_RXFULL_Pos)                  /*!< I2CA S0_IRQ_ENB: RXFULL Mask            */

/* -------------------------------  I2CA_S0_IRQ_RAW  ------------------------------ */
#define I2CA_S0_IRQ_RAW_COMPLETED_Pos         0                                                       /*!< I2CA S0_IRQ_RAW: COMPLETED Position     */
#define I2CA_S0_IRQ_RAW_COMPLETED_Msk         (0x01UL << I2CA_S0_IRQ_RAW_COMPLETED_Pos)               /*!< I2CA S0_IRQ_RAW: COMPLETED Mask         */
#define I2CA_S0_IRQ_RAW_IDLE_Pos              1                                                       /*!< I2CA S0_IRQ_RAW: IDLE Position          */
#define I2CA_S0_IRQ_RAW_IDLE_Msk              (0x01UL << I2CA_S0_IRQ_RAW_IDLE_Pos)                    /*!< I2CA S0_IRQ_RAW: IDLE Mask              */
#define I2CA_S0_IRQ_RAW_WAITING_Pos           2                                                       /*!< I2CA S0_IRQ_RAW: WAITING Position       */
#define I2CA_S0_IRQ_RAW_WAITING_Msk           (0x01UL << I2CA_S0_IRQ_RAW_WAITING_Pos)                 /*!< I2CA S0_IRQ_RAW: WAITING Mask           */
#define I2CA_S0_IRQ_RAW_TXSTALLED_Pos         3                                                       /*!< I2CA S0_IRQ_RAW: TXSTALLED Position     */
#define I2CA_S0_IRQ_RAW_TXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_RAW_TXSTALLED_Pos)               /*!< I2CA S0_IRQ_RAW: TXSTALLED Mask         */
#define I2CA_S0_IRQ_RAW_RXSTALLED_Pos         4                                                       /*!< I2CA S0_IRQ_RAW: RXSTALLED Position     */
#define I2CA_S0_IRQ_RAW_RXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_RAW_RXSTALLED_Pos)               /*!< I2CA S0_IRQ_RAW: RXSTALLED Mask         */
#define I2CA_S0_IRQ_RAW_ADDRESSMATCH_Pos      5                                                       /*!< I2CA S0_IRQ_RAW: ADDRESSMATCH Position  */
#define I2CA_S0_IRQ_RAW_ADDRESSMATCH_Msk      (0x01UL << I2CA_S0_IRQ_RAW_ADDRESSMATCH_Pos)            /*!< I2CA S0_IRQ_RAW: ADDRESSMATCH Mask      */
#define I2CA_S0_IRQ_RAW_NACKDATA_Pos          6                                                       /*!< I2CA S0_IRQ_RAW: NACKDATA Position      */
#define I2CA_S0_IRQ_RAW_NACKDATA_Msk          (0x01UL << I2CA_S0_IRQ_RAW_NACKDATA_Pos)                /*!< I2CA S0_IRQ_RAW: NACKDATA Mask          */
#define I2CA_S0_IRQ_RAW_RXDATAFIRST_Pos       7                                                       /*!< I2CA S0_IRQ_RAW: RXDATAFIRST Position   */
#define I2CA_S0_IRQ_RAW_RXDATAFIRST_Msk       (0x01UL << I2CA_S0_IRQ_RAW_RXDATAFIRST_Pos)             /*!< I2CA S0_IRQ_RAW: RXDATAFIRST Mask       */
#define I2CA_S0_IRQ_RAW_I2C_START_Pos         8                                                       /*!< I2CA S0_IRQ_RAW: I2C_START Position     */
#define I2CA_S0_IRQ_RAW_I2C_START_Msk         (0x01UL << I2CA_S0_IRQ_RAW_I2C_START_Pos)               /*!< I2CA S0_IRQ_RAW: I2C_START Mask         */
#define I2CA_S0_IRQ_RAW_I2C_STOP_Pos          9                                                       /*!< I2CA S0_IRQ_RAW: I2C_STOP Position      */
#define I2CA_S0_IRQ_RAW_I2C_STOP_Msk          (0x01UL << I2CA_S0_IRQ_RAW_I2C_STOP_Pos)                /*!< I2CA S0_IRQ_RAW: I2C_STOP Mask          */
#define I2CA_S0_IRQ_RAW_TXUNDERFLOW_Pos       10                                                      /*!< I2CA S0_IRQ_RAW: TXUNDERFLOW Position   */
#define I2CA_S0_IRQ_RAW_TXUNDERFLOW_Msk       (0x01UL << I2CA_S0_IRQ_RAW_TXUNDERFLOW_Pos)             /*!< I2CA S0_IRQ_RAW: TXUNDERFLOW Mask       */
#define I2CA_S0_IRQ_RAW_RXOVERFLOW_Pos        11                                                      /*!< I2CA S0_IRQ_RAW: RXOVERFLOW Position    */
#define I2CA_S0_IRQ_RAW_RXOVERFLOW_Msk        (0x01UL << I2CA_S0_IRQ_RAW_RXOVERFLOW_Pos)              /*!< I2CA S0_IRQ_RAW: RXOVERFLOW Mask        */
#define I2CA_S0_IRQ_RAW_TXREADY_Pos           12                                                      /*!< I2CA S0_IRQ_RAW: TXREADY Position       */
#define I2CA_S0_IRQ_RAW_TXREADY_Msk           (0x01UL << I2CA_S0_IRQ_RAW_TXREADY_Pos)                 /*!< I2CA S0_IRQ_RAW: TXREADY Mask           */
#define I2CA_S0_IRQ_RAW_RXREADY_Pos           13                                                      /*!< I2CA S0_IRQ_RAW: RXREADY Position       */
#define I2CA_S0_IRQ_RAW_RXREADY_Msk           (0x01UL << I2CA_S0_IRQ_RAW_RXREADY_Pos)                 /*!< I2CA S0_IRQ_RAW: RXREADY Mask           */
#define I2CA_S0_IRQ_RAW_TXEMPTY_Pos           14                                                      /*!< I2CA S0_IRQ_RAW: TXEMPTY Position       */
#define I2CA_S0_IRQ_RAW_TXEMPTY_Msk           (0x01UL << I2CA_S0_IRQ_RAW_TXEMPTY_Pos)                 /*!< I2CA S0_IRQ_RAW: TXEMPTY Mask           */
#define I2CA_S0_IRQ_RAW_RXFULL_Pos            15                                                      /*!< I2CA S0_IRQ_RAW: RXFULL Position        */
#define I2CA_S0_IRQ_RAW_RXFULL_Msk            (0x01UL << I2CA_S0_IRQ_RAW_RXFULL_Pos)                  /*!< I2CA S0_IRQ_RAW: RXFULL Mask            */

/* -------------------------------  I2CA_S0_IRQ_END  ------------------------------ */
#define I2CA_S0_IRQ_END_COMPLETED_Pos         0                                                       /*!< I2CA S0_IRQ_END: COMPLETED Position     */
#define I2CA_S0_IRQ_END_COMPLETED_Msk         (0x01UL << I2CA_S0_IRQ_END_COMPLETED_Pos)               /*!< I2CA S0_IRQ_END: COMPLETED Mask         */
#define I2CA_S0_IRQ_END_IDLE_Pos              1                                                       /*!< I2CA S0_IRQ_END: IDLE Position          */
#define I2CA_S0_IRQ_END_IDLE_Msk              (0x01UL << I2CA_S0_IRQ_END_IDLE_Pos)                    /*!< I2CA S0_IRQ_END: IDLE Mask              */
#define I2CA_S0_IRQ_END_WAITING_Pos           2                                                       /*!< I2CA S0_IRQ_END: WAITING Position       */
#define I2CA_S0_IRQ_END_WAITING_Msk           (0x01UL << I2CA_S0_IRQ_END_WAITING_Pos)                 /*!< I2CA S0_IRQ_END: WAITING Mask           */
#define I2CA_S0_IRQ_END_TXSTALLED_Pos         3                                                       /*!< I2CA S0_IRQ_END: TXSTALLED Position     */
#define I2CA_S0_IRQ_END_TXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_END_TXSTALLED_Pos)               /*!< I2CA S0_IRQ_END: TXSTALLED Mask         */
#define I2CA_S0_IRQ_END_RXSTALLED_Pos         4                                                       /*!< I2CA S0_IRQ_END: RXSTALLED Position     */
#define I2CA_S0_IRQ_END_RXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_END_RXSTALLED_Pos)               /*!< I2CA S0_IRQ_END: RXSTALLED Mask         */
#define I2CA_S0_IRQ_END_ADDRESSMATCH_Pos      5                                                       /*!< I2CA S0_IRQ_END: ADDRESSMATCH Position  */
#define I2CA_S0_IRQ_END_ADDRESSMATCH_Msk      (0x01UL << I2CA_S0_IRQ_END_ADDRESSMATCH_Pos)            /*!< I2CA S0_IRQ_END: ADDRESSMATCH Mask      */
#define I2CA_S0_IRQ_END_NACKDATA_Pos          6                                                       /*!< I2CA S0_IRQ_END: NACKDATA Position      */
#define I2CA_S0_IRQ_END_NACKDATA_Msk          (0x01UL << I2CA_S0_IRQ_END_NACKDATA_Pos)                /*!< I2CA S0_IRQ_END: NACKDATA Mask          */
#define I2CA_S0_IRQ_END_RXDATAFIRST_Pos       7                                                       /*!< I2CA S0_IRQ_END: RXDATAFIRST Position   */
#define I2CA_S0_IRQ_END_RXDATAFIRST_Msk       (0x01UL << I2CA_S0_IRQ_END_RXDATAFIRST_Pos)             /*!< I2CA S0_IRQ_END: RXDATAFIRST Mask       */
#define I2CA_S0_IRQ_END_I2C_START_Pos         8                                                       /*!< I2CA S0_IRQ_END: I2C_START Position     */
#define I2CA_S0_IRQ_END_I2C_START_Msk         (0x01UL << I2CA_S0_IRQ_END_I2C_START_Pos)               /*!< I2CA S0_IRQ_END: I2C_START Mask         */
#define I2CA_S0_IRQ_END_I2C_STOP_Pos          9                                                       /*!< I2CA S0_IRQ_END: I2C_STOP Position      */
#define I2CA_S0_IRQ_END_I2C_STOP_Msk          (0x01UL << I2CA_S0_IRQ_END_I2C_STOP_Pos)                /*!< I2CA S0_IRQ_END: I2C_STOP Mask          */
#define I2CA_S0_IRQ_END_TXUNDERFLOW_Pos       10                                                      /*!< I2CA S0_IRQ_END: TXUNDERFLOW Position   */
#define I2CA_S0_IRQ_END_TXUNDERFLOW_Msk       (0x01UL << I2CA_S0_IRQ_END_TXUNDERFLOW_Pos)             /*!< I2CA S0_IRQ_END: TXUNDERFLOW Mask       */
#define I2CA_S0_IRQ_END_RXOVERFLOW_Pos        11                                                      /*!< I2CA S0_IRQ_END: RXOVERFLOW Position    */
#define I2CA_S0_IRQ_END_RXOVERFLOW_Msk        (0x01UL << I2CA_S0_IRQ_END_RXOVERFLOW_Pos)              /*!< I2CA S0_IRQ_END: RXOVERFLOW Mask        */
#define I2CA_S0_IRQ_END_TXREADY_Pos           12                                                      /*!< I2CA S0_IRQ_END: TXREADY Position       */
#define I2CA_S0_IRQ_END_TXREADY_Msk           (0x01UL << I2CA_S0_IRQ_END_TXREADY_Pos)                 /*!< I2CA S0_IRQ_END: TXREADY Mask           */
#define I2CA_S0_IRQ_END_RXREADY_Pos           13                                                      /*!< I2CA S0_IRQ_END: RXREADY Position       */
#define I2CA_S0_IRQ_END_RXREADY_Msk           (0x01UL << I2CA_S0_IRQ_END_RXREADY_Pos)                 /*!< I2CA S0_IRQ_END: RXREADY Mask           */
#define I2CA_S0_IRQ_END_TXEMPTY_Pos           14                                                      /*!< I2CA S0_IRQ_END: TXEMPTY Position       */
#define I2CA_S0_IRQ_END_TXEMPTY_Msk           (0x01UL << I2CA_S0_IRQ_END_TXEMPTY_Pos)                 /*!< I2CA S0_IRQ_END: TXEMPTY Mask           */
#define I2CA_S0_IRQ_END_RXFULL_Pos            15                                                      /*!< I2CA S0_IRQ_END: RXFULL Position        */
#define I2CA_S0_IRQ_END_RXFULL_Msk            (0x01UL << I2CA_S0_IRQ_END_RXFULL_Pos)                  /*!< I2CA S0_IRQ_END: RXFULL Mask            */

/* -------------------------------  I2CA_S0_IRQ_CLR  ------------------------------ */
#define I2CA_S0_IRQ_CLR_COMPLETED_Pos         0                                                       /*!< I2CA S0_IRQ_CLR: COMPLETED Position     */
#define I2CA_S0_IRQ_CLR_COMPLETED_Msk         (0x01UL << I2CA_S0_IRQ_CLR_COMPLETED_Pos)               /*!< I2CA S0_IRQ_CLR: COMPLETED Mask         */
#define I2CA_S0_IRQ_CLR_IDLE_Pos              1                                                       /*!< I2CA S0_IRQ_CLR: IDLE Position          */
#define I2CA_S0_IRQ_CLR_IDLE_Msk              (0x01UL << I2CA_S0_IRQ_CLR_IDLE_Pos)                    /*!< I2CA S0_IRQ_CLR: IDLE Mask              */
#define I2CA_S0_IRQ_CLR_WAITING_Pos           2                                                       /*!< I2CA S0_IRQ_CLR: WAITING Position       */
#define I2CA_S0_IRQ_CLR_WAITING_Msk           (0x01UL << I2CA_S0_IRQ_CLR_WAITING_Pos)                 /*!< I2CA S0_IRQ_CLR: WAITING Mask           */
#define I2CA_S0_IRQ_CLR_TXSTALLED_Pos         3                                                       /*!< I2CA S0_IRQ_CLR: TXSTALLED Position     */
#define I2CA_S0_IRQ_CLR_TXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_CLR_TXSTALLED_Pos)               /*!< I2CA S0_IRQ_CLR: TXSTALLED Mask         */
#define I2CA_S0_IRQ_CLR_RXSTALLED_Pos         4                                                       /*!< I2CA S0_IRQ_CLR: RXSTALLED Position     */
#define I2CA_S0_IRQ_CLR_RXSTALLED_Msk         (0x01UL << I2CA_S0_IRQ_CLR_RXSTALLED_Pos)               /*!< I2CA S0_IRQ_CLR: RXSTALLED Mask         */
#define I2CA_S0_IRQ_CLR_ADDRESSMATCH_Pos      5                                                       /*!< I2CA S0_IRQ_CLR: ADDRESSMATCH Position  */
#define I2CA_S0_IRQ_CLR_ADDRESSMATCH_Msk      (0x01UL << I2CA_S0_IRQ_CLR_ADDRESSMATCH_Pos)            /*!< I2CA S0_IRQ_CLR: ADDRESSMATCH Mask      */
#define I2CA_S0_IRQ_CLR_NACKDATA_Pos          6                                                       /*!< I2CA S0_IRQ_CLR: NACKDATA Position      */
#define I2CA_S0_IRQ_CLR_NACKDATA_Msk          (0x01UL << I2CA_S0_IRQ_CLR_NACKDATA_Pos)                /*!< I2CA S0_IRQ_CLR: NACKDATA Mask          */
#define I2CA_S0_IRQ_CLR_RXDATAFIRST_Pos       7                                                       /*!< I2CA S0_IRQ_CLR: RXDATAFIRST Position   */
#define I2CA_S0_IRQ_CLR_RXDATAFIRST_Msk       (0x01UL << I2CA_S0_IRQ_CLR_RXDATAFIRST_Pos)             /*!< I2CA S0_IRQ_CLR: RXDATAFIRST Mask       */
#define I2CA_S0_IRQ_CLR_I2C_START_Pos         8                                                       /*!< I2CA S0_IRQ_CLR: I2C_START Position     */
#define I2CA_S0_IRQ_CLR_I2C_START_Msk         (0x01UL << I2CA_S0_IRQ_CLR_I2C_START_Pos)               /*!< I2CA S0_IRQ_CLR: I2C_START Mask         */
#define I2CA_S0_IRQ_CLR_I2C_STOP_Pos          9                                                       /*!< I2CA S0_IRQ_CLR: I2C_STOP Position      */
#define I2CA_S0_IRQ_CLR_I2C_STOP_Msk          (0x01UL << I2CA_S0_IRQ_CLR_I2C_STOP_Pos)                /*!< I2CA S0_IRQ_CLR: I2C_STOP Mask          */
#define I2CA_S0_IRQ_CLR_TXUNDERFLOW_Pos       10                                                      /*!< I2CA S0_IRQ_CLR: TXUNDERFLOW Position   */
#define I2CA_S0_IRQ_CLR_TXUNDERFLOW_Msk       (0x01UL << I2CA_S0_IRQ_CLR_TXUNDERFLOW_Pos)             /*!< I2CA S0_IRQ_CLR: TXUNDERFLOW Mask       */
#define I2CA_S0_IRQ_CLR_RXOVERFLOW_Pos        11                                                      /*!< I2CA S0_IRQ_CLR: RXOVERFLOW Position    */
#define I2CA_S0_IRQ_CLR_RXOVERFLOW_Msk        (0x01UL << I2CA_S0_IRQ_CLR_RXOVERFLOW_Pos)              /*!< I2CA S0_IRQ_CLR: RXOVERFLOW Mask        */
#define I2CA_S0_IRQ_CLR_TXREADY_Pos           12                                                      /*!< I2CA S0_IRQ_CLR: TXREADY Position       */
#define I2CA_S0_IRQ_CLR_TXREADY_Msk           (0x01UL << I2CA_S0_IRQ_CLR_TXREADY_Pos)                 /*!< I2CA S0_IRQ_CLR: TXREADY Mask           */
#define I2CA_S0_IRQ_CLR_RXREADY_Pos           13                                                      /*!< I2CA S0_IRQ_CLR: RXREADY Position       */
#define I2CA_S0_IRQ_CLR_RXREADY_Msk           (0x01UL << I2CA_S0_IRQ_CLR_RXREADY_Pos)                 /*!< I2CA S0_IRQ_CLR: RXREADY Mask           */
#define I2CA_S0_IRQ_CLR_TXEMPTY_Pos           14                                                      /*!< I2CA S0_IRQ_CLR: TXEMPTY Position       */
#define I2CA_S0_IRQ_CLR_TXEMPTY_Msk           (0x01UL << I2CA_S0_IRQ_CLR_TXEMPTY_Pos)                 /*!< I2CA S0_IRQ_CLR: TXEMPTY Mask           */
#define I2CA_S0_IRQ_CLR_RXFULL_Pos            15                                                      /*!< I2CA S0_IRQ_CLR: RXFULL Position        */
#define I2CA_S0_IRQ_CLR_RXFULL_Msk            (0x01UL << I2CA_S0_IRQ_CLR_RXFULL_Pos)                  /*!< I2CA S0_IRQ_CLR: RXFULL Mask            */

/* ------------------------------  I2CA_S0_FIFO_CLR  ------------------------------ */
#define I2CA_S0_FIFO_CLR_RXFIFO_Pos           0                                                       /*!< I2CA S0_FIFO_CLR: RXFIFO Position       */
#define I2CA_S0_FIFO_CLR_RXFIFO_Msk           (0x01UL << I2CA_S0_FIFO_CLR_RXFIFO_Pos)                 /*!< I2CA S0_FIFO_CLR: RXFIFO Mask           */
#define I2CA_S0_FIFO_CLR_TXFIFO_Pos           1                                                       /*!< I2CA S0_FIFO_CLR: TXFIFO Position       */
#define I2CA_S0_FIFO_CLR_TXFIFO_Msk           (0x01UL << I2CA_S0_FIFO_CLR_TXFIFO_Pos)                 /*!< I2CA S0_FIFO_CLR: TXFIFO Mask           */

/* ---------------------------  I2CA_S0_ADDRESSB  -------------------------- */
#define I2CA_S0_ADDRESSB_RW_Pos  							0                                                   		/*!< I2CA S0_ADDRESSB: RW Position 					 */
#define I2CA_S0_ADDRESSB_RW_Msk  			  			(0x01UL << I2CA_S0_ADDRESSB_RW_Pos)     								/*!< I2CA S0_ADDRESSB: RW Mask     					 */
#define I2CA_S0_ADDRESSB_ADDRESS_Pos  				1	                                                  		/*!< I2CA S0_ADDRESSB: ADDRESS Position 		 */
#define I2CA_S0_ADDRESSB_ADDRESS_Msk  				(0x3FFUL << I2CA_S0_ADDRESSB_ADDRESS_Pos) 							/*!< I2CA S0_ADDRESSB: ADDRESS Mask  				 */
#define I2CA_S0_ADDRESSB_A10MODE_Pos  				15                                                  		/*!< I2CA S0_ADDRESSB: A10MODE Position 		 */
#define I2CA_S0_ADDRESSB_A10MODE_Msk  				(0x01UL << I2CA_S0_ADDRESSB_A10MODE_Pos)  							/*!< I2CA S0_ADDRESSB: A10MODE Mask  				 */

/* ---------------------------  I2CA_S0_ADDRESSMASKB  -------------------------- */
#define I2CA_S0_ADDRESSMASKB_RWMASK_Pos 			0                                                    		/*!< I2CA S0_ADDRESSMASKB: RWMASK Position 		*/
#define I2CA_S0_ADDRESSMASKB_RWMASK_Msk 			(0x01UL << I2CA_S0_ADDRESSMASKB_RWMASK_Pos)							/*!< I2CA S0_ADDRESSMASKB: RWMASK Mask     		*/
#define I2CA_S0_ADDRESSMASKB_MASK_Pos   			1	                                                   		/*!< I2CA S0_ADDRESSMASKB: MASK Position 			*/
#define I2CA_S0_ADDRESSMASKB_MASK_Msk   			(0x3FFUL << I2CA_S0_ADDRESSMASKB_MASK_Pos) 							/*!< I2CA S0_ADDRESSMASKB: MASK Mask  				*/

/* ================================================================================ */
/* ================          struct 'I2CB' Position & Mask         ================ */
/* ================================================================================ */


/* ----------------------------------  I2CB_CTRL  --------------------------------- */
#define I2CB_CTRL_CLKENABLED_Pos              0                                                       /*!< I2CB CTRL: CLKENABLED Position          */
#define I2CB_CTRL_CLKENABLED_Msk              (0x01UL << I2CB_CTRL_CLKENABLED_Pos)                    /*!< I2CB CTRL: CLKENABLED Mask              */
#define I2CB_CTRL_ENABLED_Pos               	1                                                       /*!< I2CB CTRL: ENABLED Position           	 */
#define I2CB_CTRL_ENABLED_Msk               	(0x01UL << I2CB_CTRL_ENABLED_Pos)                     	/*!< I2CB CTRL: ENABLED Mask               	 */
#define I2CB_CTRL_ENABLE_Pos                  2                                                       /*!< I2CB CTRL: ENABLE Position              */
#define I2CB_CTRL_ENABLE_Msk                  (0x01UL << I2CB_CTRL_ENABLE_Pos)                        /*!< I2CB CTRL: ENABLE Mask                  */
#define I2CB_CTRL_TXFEMD_Pos                  3                                                       /*!< I2CB CTRL: TXFEMD Position              */
#define I2CB_CTRL_TXFEMD_Msk                  (0x01UL << I2CB_CTRL_TXFEMD_Pos)                        /*!< I2CB CTRL: TXFEMD Mask                  */
#define I2CB_CTRL_RXFFMD_Pos                  4                                                       /*!< I2CB CTRL: RXFFMD Position              */
#define I2CB_CTRL_RXFFMD_Msk                  (0x01UL << I2CB_CTRL_RXFFMD_Pos)                        /*!< I2CB CTRL: RXFFMD Mask                  */
#define I2CB_CTRL_ALGFILTER_Pos               5                                                       /*!< I2CB CTRL: ALGFILTER Position           */
#define I2CB_CTRL_ALGFILTER_Msk               (0x01UL << I2CB_CTRL_ALGFILTER_Pos)                     /*!< I2CB CTRL: ALGFILTER Mask               */
#define I2CB_CTRL_DLGFILTER_Pos               6                                                       /*!< I2CB CTRL: DLGFILTER Position           */
#define I2CB_CTRL_DLGFILTER_Msk               (0x01UL << I2CB_CTRL_DLGFILTER_Pos)                     /*!< I2CB CTRL: DLGFILTER Mask               */
#define I2CB_CTRL_LOOPBACK_Pos                8                                                       /*!< I2CB CTRL: LOOPBACK Position            */
#define I2CB_CTRL_LOOPBACK_Msk                (0x01UL << I2CB_CTRL_LOOPBACK_Pos)                      /*!< I2CB CTRL: LOOPBACK Mask                */
#define I2CB_CTRL_TMCONFIGENB_Pos             9                                                       /*!< I2CB CTRL: TMCONFIGENB Position         */
#define I2CB_CTRL_TMCONFIGENB_Msk             (0x01UL << I2CB_CTRL_TMCONFIGENB_Pos)                   /*!< I2CB CTRL: TMCONFIGENB Mask             */

/* --------------------------------  I2CB_CLKSCALE  ------------------------------- */
#define I2CB_CLKSCALE_VALUE_Pos               0                                                       /*!< I2CB CLKSCALE: VALUE Position           */
#define I2CB_CLKSCALE_VALUE_Msk               (0x7fffffffUL << I2CB_CLKSCALE_VALUE_Pos)               /*!< I2CB CLKSCALE: VALUE Mask               */
#define I2CB_CLKSCALE_FASTMODE_Pos            31                                                      /*!< I2CB CLKSCALE: FASTMODE Position        */
#define I2CB_CLKSCALE_FASTMODE_Msk            (0x01UL << I2CB_CLKSCALE_FASTMODE_Pos)                  /*!< I2CB CLKSCALE: FASTMODE Mask            */

/* --------------------------------  I2CB_ADDRESS  -------------------------------- */
#define I2CB_ADDRESS_DIRECTION_Pos  					0                                              					/*!< I2CB ADDRESS: DIRECTION Position 			 */
#define I2CB_ADDRESS_DIRECTION_Msk  					(0x01UL << I2CB_ADDRESS_DIRECTION_Pos)   								/*!< I2CB ADDRESS: DIRECTION Mask     			 */
#define I2CB_ADDRESS_ADDRESS_Pos  						1	                                             					/*!< I2CB ADDRESS: ADDRESS Position 				 */
#define I2CB_ADDRESS_ADDRESS_Msk  						(0x3FFUL << I2CB_ADDRESS_ADDRESS_Pos)										/*!< I2CB ADDRESS: ADDRESS Mask  						 */
#define I2CB_ADDRESS_A10MODE_Pos  						15                                             					/*!< I2CB ADDRESS: A10MODE Position 				 */
#define I2CB_ADDRESS_A10MODE_Msk  						(0x01UL << I2CB_ADDRESS_A10MODE_Pos)										/*!< I2CB ADDRESS: A10MODE Mask  						 */

/* -------------------------------  I2CB_CMD  ------------------------------------ */
#define I2CB_CMD_START_Pos  									0                                                       /*!< I2CB CMD: START Position 							 */
#define I2CB_CMD_START_Msk  									(0x01UL << I2CB_CMD_START_Pos)     											/*!< I2CB CMD: START Mask     							 */
#define I2CB_CMD_STOP_Pos  										1	                                                      /*!< I2CB CMD: STOP Position 								 */
#define I2CB_CMD_STOP_Msk  										(0x01UL << I2CB_CMD_STOP_Pos)        									/*!< I2CB CMD: STOP Mask  									 */
#define I2CB_CMD_CANCEL_Pos  									2                                                      /*!< I2CB CMD: CANCEL Position 							 */
#define I2CB_CMD_CANCEL_Msk  									(0x01UL << I2CB_CMD_CANCEL_Pos)        									/*!< I2CB CMD: CANCEL Mask  								 */

/* ---------------------------------  I2CB_STATUS  -------------------------------- */
#define I2CB_STATUS_I2CIDLE_Pos               0                                                       /*!< I2CB STATUS: I2CIDLE Position           */
#define I2CB_STATUS_I2CIDLE_Msk               (0x01UL << I2CB_STATUS_I2CIDLE_Pos)                     /*!< I2CB STATUS: I2CIDLE Mask               */
#define I2CB_STATUS_IDLE_Pos                  1                                                       /*!< I2CB STATUS: IDLE Position              */
#define I2CB_STATUS_IDLE_Msk                  (0x01UL << I2CB_STATUS_IDLE_Pos)                        /*!< I2CB STATUS: IDLE Mask                  */
#define I2CB_STATUS_WAITING_Pos               2                                                       /*!< I2CB STATUS: WAITING Position           */
#define I2CB_STATUS_WAITING_Msk               (0x01UL << I2CB_STATUS_WAITING_Pos)                     /*!< I2CB STATUS: WAITING Mask               */
#define I2CB_STATUS_STALLED_Pos               3                                                       /*!< I2CB STATUS: STALLED Position           */
#define I2CB_STATUS_STALLED_Msk               (0x01UL << I2CB_STATUS_STALLED_Pos)                     /*!< I2CB STATUS: STALLED Mask               */
#define I2CB_STATUS_ARBLOST_Pos               4                                                       /*!< I2CB STATUS: ARBLOST Position           */
#define I2CB_STATUS_ARBLOST_Msk               (0x01UL << I2CB_STATUS_ARBLOST_Pos)                     /*!< I2CB STATUS: ARBLOST Mask               */
#define I2CB_STATUS_NACKADDR_Pos              5                                                       /*!< I2CB STATUS: NACKADDR Position          */
#define I2CB_STATUS_NACKADDR_Msk              (0x01UL << I2CB_STATUS_NACKADDR_Pos)                    /*!< I2CB STATUS: NACKADDR Mask              */
#define I2CB_STATUS_NACKDATA_Pos              6                                                       /*!< I2CB STATUS: NACKDATA Position          */
#define I2CB_STATUS_NACKDATA_Msk              (0x01UL << I2CB_STATUS_NACKDATA_Pos)                    /*!< I2CB STATUS: NACKDATA Mask              */
#define I2CB_STATUS_RXNEMPTY_Pos              8                                                       /*!< I2CB STATUS: RXNEMPTY Position          */
#define I2CB_STATUS_RXNEMPTY_Msk              (0x01UL << I2CB_STATUS_RXNEMPTY_Pos)                    /*!< I2CB STATUS: RXNEMPTY Mask              */
#define I2CB_STATUS_RXFULL_Pos                9                                                       /*!< I2CB STATUS: RXFULL Position            */
#define I2CB_STATUS_RXFULL_Msk                (0x01UL << I2CB_STATUS_RXFULL_Pos)                      /*!< I2CB STATUS: RXFULL Mask                */
#define I2CB_STATUS_RXTRIGGER_Pos              11                                                      /*!< I2CB STATUS: RXTRIGGER Position          */
#define I2CB_STATUS_RXTRIGGER_Msk              (0x01UL << I2CB_STATUS_RXTRIGGER_Pos)                    /*!< I2CB STATUS: RXTRIGGER Mask              */
#define I2CB_STATUS_TXEMPTY_Pos               12                                                      /*!< I2CB STATUS: TXEMPTY Position           */
#define I2CB_STATUS_TXEMPTY_Msk               (0x01UL << I2CB_STATUS_TXEMPTY_Pos)                     /*!< I2CB STATUS: TXEMPTY Mask               */
#define I2CB_STATUS_TXNFULL_Pos               13                                                      /*!< I2CB STATUS: TXNFULL Position           */
#define I2CB_STATUS_TXNFULL_Msk               (0x01UL << I2CB_STATUS_TXNFULL_Pos)                     /*!< I2CB STATUS: TXNFULL Mask               */
#define I2CB_STATUS_TXTRIGGER_Pos              15                                                      /*!< I2CB STATUS: TXTRIGGER Position          */
#define I2CB_STATUS_TXTRIGGER_Msk              (0x01UL << I2CB_STATUS_TXTRIGGER_Pos)                    /*!< I2CB STATUS: TXTRIGGER Mask              */
#define I2CB_STATUS_RAW_SDA_Pos               30                                                      /*!< I2CB STATUS: RAW_SDA Position           */
#define I2CB_STATUS_RAW_SDA_Msk               (0x01UL << I2CB_STATUS_RAW_SDA_Pos)                     /*!< I2CB STATUS: RAW_SDA Mask               */
#define I2CB_STATUS_RAW_SCL_Pos               31                                                      /*!< I2CB STATUS: RAW_SCL Position           */
#define I2CB_STATUS_RAW_SCL_Msk               (0x01UL << I2CB_STATUS_RAW_SCL_Pos)                     /*!< I2CB STATUS: RAW_SCL Mask               */

/* ---------------------------------  I2CB_STATE  ------------------------------ */
#define I2CB_STATE_STATE_Pos  								0                                                       /*!< I2CB STATE: STATE Position 						 */
#define I2CB_STATE_STATE_Msk  								(0x0FUL << I2CB_STATE_STATE_Pos)     				  					/*!< I2CB STATE: STATE Mask     					   */
#define I2CB_STATE_STEP_Pos  									4	                                                      /*!< I2CB STATE: STEP Position 							 */
#define I2CB_STATE_STEP_Msk  									(0x0FUL << I2CB_STATE_STEP_Pos)        			  					/*!< I2CB STATE: STEP Mask  							   */
#define I2CB_STATE_RXFIFO_Pos  								8                                                       /*!< I2CB STATE: RXFIFO Position 						 */
#define I2CB_STATE_RXFIFO_Msk  								(0x01FUL << I2CB_STATE_RXFIFO_Pos)        							/*!< I2CB STATE: RXFIFO Mask  							 */
#define I2CB_STATE_TXFIFO_Pos  								14                                                      /*!< I2CB STATE: TXFIFO Position 						 */
#define I2CB_STATE_TXFIFO_Msk  								(0x01FUL << I2CB_STATE_TXFIFO_Pos)     			  					/*!< I2CB STATE: TXFIFO Mask     						 */
#define I2CB_STATE_BITSTATE_Pos  							20                                                      /*!< I2CB STATE: BITSTATE Position 					 */
#define I2CB_STATE_BITSTATE_Msk  							(0x01FFUL << I2CB_STATE_BITSTATE_Pos)     							/*!< I2CB STATE: BITSTATE Mask     					 */

/* --------------------------------  I2CB_IRQ_ENB  -------------------------------- */
#define I2CB_IRQ_ENB_I2CIDLE_Pos              0                                                       /*!< I2CB IRQ_ENB: I2CIDLE Position          */
#define I2CB_IRQ_ENB_I2CIDLE_Msk              (0x01UL << I2CB_IRQ_ENB_I2CIDLE_Pos)                    /*!< I2CB IRQ_ENB: I2CIDLE Mask              */
#define I2CB_IRQ_ENB_IDLE_Pos                 1                                                       /*!< I2CB IRQ_ENB: IDLE Position             */
#define I2CB_IRQ_ENB_IDLE_Msk                 (0x01UL << I2CB_IRQ_ENB_IDLE_Pos)                       /*!< I2CB IRQ_ENB: IDLE Mask                 */
#define I2CB_IRQ_ENB_WAITING_Pos              2                                                       /*!< I2CB IRQ_ENB: WAITING Position          */
#define I2CB_IRQ_ENB_WAITING_Msk              (0x01UL << I2CB_IRQ_ENB_WAITING_Pos)                    /*!< I2CB IRQ_ENB: WAITING Mask              */
#define I2CB_IRQ_ENB_STALLED_Pos              3                                                       /*!< I2CB IRQ_ENB: STALLED Position          */
#define I2CB_IRQ_ENB_STALLED_Msk              (0x01UL << I2CB_IRQ_ENB_STALLED_Pos)                    /*!< I2CB IRQ_ENB: STALLED Mask              */
#define I2CB_IRQ_ENB_ARBLOST_Pos              4                                                       /*!< I2CB IRQ_ENB: ARBLOST Position          */
#define I2CB_IRQ_ENB_ARBLOST_Msk              (0x01UL << I2CB_IRQ_ENB_ARBLOST_Pos)                    /*!< I2CB IRQ_ENB: ARBLOST Mask              */
#define I2CB_IRQ_ENB_NACKADDR_Pos             5                                                       /*!< I2CB IRQ_ENB: NACKADDR Position         */
#define I2CB_IRQ_ENB_NACKADDR_Msk             (0x01UL << I2CB_IRQ_ENB_NACKADDR_Pos)                   /*!< I2CB IRQ_ENB: NACKADDR Mask             */
#define I2CB_IRQ_ENB_NACKDATA_Pos             6                                                       /*!< I2CB IRQ_ENB: NACKDATA Position         */
#define I2CB_IRQ_ENB_NACKDATA_Msk             (0x01UL << I2CB_IRQ_ENB_NACKDATA_Pos)                   /*!< I2CB IRQ_ENB: NACKDATA Mask             */
#define I2CB_IRQ_ENB_CLKLOTO_Pos              7                                                       /*!< I2CB IRQ_ENB: CLKLOTO Position          */
#define I2CB_IRQ_ENB_CLKLOTO_Msk              (0x01UL << I2CB_IRQ_ENB_CLKLOTO_Pos)                    /*!< I2CB IRQ_ENB: CLKLOTO Mask              */
#define I2CB_IRQ_ENB_TXOVERFLOW_Pos           10                                                      /*!< I2CB IRQ_ENB: TXOVERFLOW Position       */
#define I2CB_IRQ_ENB_TXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_ENB_TXOVERFLOW_Pos)                 /*!< I2CB IRQ_ENB: TXOVERFLOW Mask           */
#define I2CB_IRQ_ENB_RXOVERFLOW_Pos           11                                                      /*!< I2CB IRQ_ENB: RXOVERFLOW Position       */
#define I2CB_IRQ_ENB_RXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_ENB_RXOVERFLOW_Pos)                 /*!< I2CB IRQ_ENB: RXOVERFLOW Mask           */
#define I2CB_IRQ_ENB_TXREADY_Pos              12                                                      /*!< I2CB IRQ_ENB: TXREADY Position          */
#define I2CB_IRQ_ENB_TXREADY_Msk              (0x01UL << I2CB_IRQ_ENB_TXREADY_Pos)                    /*!< I2CB IRQ_ENB: TXREADY Mask              */
#define I2CB_IRQ_ENB_RXREADY_Pos              13                                                      /*!< I2CB IRQ_ENB: RXREADY Position          */
#define I2CB_IRQ_ENB_RXREADY_Msk              (0x01UL << I2CB_IRQ_ENB_RXREADY_Pos)                    /*!< I2CB IRQ_ENB: RXREADY Mask              */
#define I2CB_IRQ_ENB_TXEMPTY_Pos              14                                                      /*!< I2CB IRQ_ENB: TXEMPTY Position          */
#define I2CB_IRQ_ENB_TXEMPTY_Msk              (0x01UL << I2CB_IRQ_ENB_TXEMPTY_Pos)                    /*!< I2CB IRQ_ENB: TXEMPTY Mask              */
#define I2CB_IRQ_ENB_RXFULL_Pos               15                                                      /*!< I2CB IRQ_ENB: RXFULL Position           */
#define I2CB_IRQ_ENB_RXFULL_Msk               (0x01UL << I2CB_IRQ_ENB_RXFULL_Pos)                     /*!< I2CB IRQ_ENB: RXFULL Mask               */

/* --------------------------------  I2CB_IRQ_RAW  -------------------------------- */
#define I2CB_IRQ_RAW_I2CIDLE_Pos              0                                                       /*!< I2CB IRQ_RAW: I2CIDLE Position          */
#define I2CB_IRQ_RAW_I2CIDLE_Msk              (0x01UL << I2CB_IRQ_RAW_I2CIDLE_Pos)                    /*!< I2CB IRQ_RAW: I2CIDLE Mask              */
#define I2CB_IRQ_RAW_IDLE_Pos                 1                                                       /*!< I2CB IRQ_RAW: IDLE Position             */
#define I2CB_IRQ_RAW_IDLE_Msk                 (0x01UL << I2CB_IRQ_RAW_IDLE_Pos)                       /*!< I2CB IRQ_RAW: IDLE Mask                 */
#define I2CB_IRQ_RAW_WAITING_Pos              2                                                       /*!< I2CB IRQ_RAW: WAITING Position          */
#define I2CB_IRQ_RAW_WAITING_Msk              (0x01UL << I2CB_IRQ_RAW_WAITING_Pos)                    /*!< I2CB IRQ_RAW: WAITING Mask              */
#define I2CB_IRQ_RAW_STALLED_Pos              3                                                       /*!< I2CB IRQ_RAW: STALLED Position          */
#define I2CB_IRQ_RAW_STALLED_Msk              (0x01UL << I2CB_IRQ_RAW_STALLED_Pos)                    /*!< I2CB IRQ_RAW: STALLED Mask              */
#define I2CB_IRQ_RAW_ARBLOST_Pos              4                                                       /*!< I2CB IRQ_RAW: ARBLOST Position          */
#define I2CB_IRQ_RAW_ARBLOST_Msk              (0x01UL << I2CB_IRQ_RAW_ARBLOST_Pos)                    /*!< I2CB IRQ_RAW: ARBLOST Mask              */
#define I2CB_IRQ_RAW_NACKADDR_Pos             5                                                       /*!< I2CB IRQ_RAW: NACKADDR Position         */
#define I2CB_IRQ_RAW_NACKADDR_Msk             (0x01UL << I2CB_IRQ_RAW_NACKADDR_Pos)                   /*!< I2CB IRQ_RAW: NACKADDR Mask             */
#define I2CB_IRQ_RAW_NACKDATA_Pos             6                                                       /*!< I2CB IRQ_RAW: NACKDATA Position         */
#define I2CB_IRQ_RAW_NACKDATA_Msk             (0x01UL << I2CB_IRQ_RAW_NACKDATA_Pos)                   /*!< I2CB IRQ_RAW: NACKDATA Mask             */
#define I2CB_IRQ_RAW_CLKLOTO_Pos              7                                                       /*!< I2CB IRQ_RAW: CLKLOTO Position          */
#define I2CB_IRQ_RAW_CLKLOTO_Msk              (0x01UL << I2CB_IRQ_RAW_CLKLOTO_Pos)                    /*!< I2CB IRQ_RAW: CLKLOTO Mask              */
#define I2CB_IRQ_RAW_TXOVERFLOW_Pos           10                                                      /*!< I2CB IRQ_RAW: TXOVERFLOW Position       */
#define I2CB_IRQ_RAW_TXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_RAW_TXOVERFLOW_Pos)                 /*!< I2CB IRQ_RAW: TXOVERFLOW Mask           */
#define I2CB_IRQ_RAW_RXOVERFLOW_Pos           11                                                      /*!< I2CB IRQ_RAW: RXOVERFLOW Position       */
#define I2CB_IRQ_RAW_RXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_RAW_RXOVERFLOW_Pos)                 /*!< I2CB IRQ_RAW: RXOVERFLOW Mask           */
#define I2CB_IRQ_RAW_TXREADY_Pos              12                                                      /*!< I2CB IRQ_RAW: TXREADY Position          */
#define I2CB_IRQ_RAW_TXREADY_Msk              (0x01UL << I2CB_IRQ_RAW_TXREADY_Pos)                    /*!< I2CB IRQ_RAW: TXREADY Mask              */
#define I2CB_IRQ_RAW_RXREADY_Pos              13                                                      /*!< I2CB IRQ_RAW: RXREADY Position          */
#define I2CB_IRQ_RAW_RXREADY_Msk              (0x01UL << I2CB_IRQ_RAW_RXREADY_Pos)                    /*!< I2CB IRQ_RAW: RXREADY Mask              */
#define I2CB_IRQ_RAW_TXEMPTY_Pos              14                                                      /*!< I2CB IRQ_RAW: TXEMPTY Position          */
#define I2CB_IRQ_RAW_TXEMPTY_Msk              (0x01UL << I2CB_IRQ_RAW_TXEMPTY_Pos)                    /*!< I2CB IRQ_RAW: TXEMPTY Mask              */
#define I2CB_IRQ_RAW_RXFULL_Pos               15                                                      /*!< I2CB IRQ_RAW: RXFULL Position           */
#define I2CB_IRQ_RAW_RXFULL_Msk               (0x01UL << I2CB_IRQ_RAW_RXFULL_Pos)                     /*!< I2CB IRQ_RAW: RXFULL Mask               */

/* --------------------------------  I2CB_IRQ_END  -------------------------------- */
#define I2CB_IRQ_END_I2CIDLE_Pos              0                                                       /*!< I2CB IRQ_END: I2CIDLE Position          */
#define I2CB_IRQ_END_I2CIDLE_Msk              (0x01UL << I2CB_IRQ_END_I2CIDLE_Pos)                    /*!< I2CB IRQ_END: I2CIDLE Mask              */
#define I2CB_IRQ_END_IDLE_Pos                 1                                                       /*!< I2CB IRQ_END: IDLE Position             */
#define I2CB_IRQ_END_IDLE_Msk                 (0x01UL << I2CB_IRQ_END_IDLE_Pos)                       /*!< I2CB IRQ_END: IDLE Mask                 */
#define I2CB_IRQ_END_WAITING_Pos              2                                                       /*!< I2CB IRQ_END: WAITING Position          */
#define I2CB_IRQ_END_WAITING_Msk              (0x01UL << I2CB_IRQ_END_WAITING_Pos)                    /*!< I2CB IRQ_END: WAITING Mask              */
#define I2CB_IRQ_END_STALLED_Pos              3                                                       /*!< I2CB IRQ_END: STALLED Position          */
#define I2CB_IRQ_END_STALLED_Msk              (0x01UL << I2CB_IRQ_END_STALLED_Pos)                    /*!< I2CB IRQ_END: STALLED Mask              */
#define I2CB_IRQ_END_ARBLOST_Pos              4                                                       /*!< I2CB IRQ_END: ARBLOST Position          */
#define I2CB_IRQ_END_ARBLOST_Msk              (0x01UL << I2CB_IRQ_END_ARBLOST_Pos)                    /*!< I2CB IRQ_END: ARBLOST Mask              */
#define I2CB_IRQ_END_NACKADDR_Pos             5                                                       /*!< I2CB IRQ_END: NACKADDR Position         */
#define I2CB_IRQ_END_NACKADDR_Msk             (0x01UL << I2CB_IRQ_END_NACKADDR_Pos)                   /*!< I2CB IRQ_END: NACKADDR Mask             */
#define I2CB_IRQ_END_NACKDATA_Pos             6                                                       /*!< I2CB IRQ_END: NACKDATA Position         */
#define I2CB_IRQ_END_NACKDATA_Msk             (0x01UL << I2CB_IRQ_END_NACKDATA_Pos)                   /*!< I2CB IRQ_END: NACKDATA Mask             */
#define I2CB_IRQ_END_CLKLOTO_Pos              7                                                       /*!< I2CB IRQ_END: CLKLOTO Position          */
#define I2CB_IRQ_END_CLKLOTO_Msk              (0x01UL << I2CB_IRQ_END_CLKLOTO_Pos)                    /*!< I2CB IRQ_END: CLKLOTO Mask              */
#define I2CB_IRQ_END_TXOVERFLOW_Pos           10                                                      /*!< I2CB IRQ_END: TXOVERFLOW Position       */
#define I2CB_IRQ_END_TXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_END_TXOVERFLOW_Pos)                 /*!< I2CB IRQ_END: TXOVERFLOW Mask           */
#define I2CB_IRQ_END_RXOVERFLOW_Pos           11                                                      /*!< I2CB IRQ_END: RXOVERFLOW Position       */
#define I2CB_IRQ_END_RXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_END_RXOVERFLOW_Pos)                 /*!< I2CB IRQ_END: RXOVERFLOW Mask           */
#define I2CB_IRQ_END_TXREADY_Pos              12                                                      /*!< I2CB IRQ_END: TXREADY Position          */
#define I2CB_IRQ_END_TXREADY_Msk              (0x01UL << I2CB_IRQ_END_TXREADY_Pos)                    /*!< I2CB IRQ_END: TXREADY Mask              */
#define I2CB_IRQ_END_RXREADY_Pos              13                                                      /*!< I2CB IRQ_END: RXREADY Position          */
#define I2CB_IRQ_END_RXREADY_Msk              (0x01UL << I2CB_IRQ_END_RXREADY_Pos)                    /*!< I2CB IRQ_END: RXREADY Mask              */
#define I2CB_IRQ_END_TXEMPTY_Pos              14                                                      /*!< I2CB IRQ_END: TXEMPTY Position          */
#define I2CB_IRQ_END_TXEMPTY_Msk              (0x01UL << I2CB_IRQ_END_TXEMPTY_Pos)                    /*!< I2CB IRQ_END: TXEMPTY Mask              */
#define I2CB_IRQ_END_RXFULL_Pos               15                                                      /*!< I2CB IRQ_END: RXFULL Position           */
#define I2CB_IRQ_END_RXFULL_Msk               (0x01UL << I2CB_IRQ_END_RXFULL_Pos)                     /*!< I2CB IRQ_END: RXFULL Mask               */

/* --------------------------------  I2CB_IRQ_CLR  -------------------------------- */
#define I2CB_IRQ_CLR_I2CIDLE_Pos              0                                                       /*!< I2CB IRQ_CLR: I2CIDLE Position          */
#define I2CB_IRQ_CLR_I2CIDLE_Msk              (0x01UL << I2CB_IRQ_CLR_I2CIDLE_Pos)                    /*!< I2CB IRQ_CLR: I2CIDLE Mask              */
#define I2CB_IRQ_CLR_IDLE_Pos                 1                                                       /*!< I2CB IRQ_CLR: IDLE Position             */
#define I2CB_IRQ_CLR_IDLE_Msk                 (0x01UL << I2CB_IRQ_CLR_IDLE_Pos)                       /*!< I2CB IRQ_CLR: IDLE Mask                 */
#define I2CB_IRQ_CLR_WAITING_Pos              2                                                       /*!< I2CB IRQ_CLR: WAITING Position          */
#define I2CB_IRQ_CLR_WAITING_Msk              (0x01UL << I2CB_IRQ_CLR_WAITING_Pos)                    /*!< I2CB IRQ_CLR: WAITING Mask              */
#define I2CB_IRQ_CLR_STALLED_Pos              3                                                       /*!< I2CB IRQ_CLR: STALLED Position          */
#define I2CB_IRQ_CLR_STALLED_Msk              (0x01UL << I2CB_IRQ_CLR_STALLED_Pos)                    /*!< I2CB IRQ_CLR: STALLED Mask              */
#define I2CB_IRQ_CLR_ARBLOST_Pos              4                                                       /*!< I2CB IRQ_CLR: ARBLOST Position          */
#define I2CB_IRQ_CLR_ARBLOST_Msk              (0x01UL << I2CB_IRQ_CLR_ARBLOST_Pos)                    /*!< I2CB IRQ_CLR: ARBLOST Mask              */
#define I2CB_IRQ_CLR_NACKADDR_Pos             5                                                       /*!< I2CB IRQ_CLR: NACKADDR Position         */
#define I2CB_IRQ_CLR_NACKADDR_Msk             (0x01UL << I2CB_IRQ_CLR_NACKADDR_Pos)                   /*!< I2CB IRQ_CLR: NACKADDR Mask             */
#define I2CB_IRQ_CLR_NACKDATA_Pos             6                                                       /*!< I2CB IRQ_CLR: NACKDATA Position         */
#define I2CB_IRQ_CLR_NACKDATA_Msk             (0x01UL << I2CB_IRQ_CLR_NACKDATA_Pos)                   /*!< I2CB IRQ_CLR: NACKDATA Mask             */
#define I2CB_IRQ_CLR_CLKLOTO_Pos              7                                                       /*!< I2CB IRQ_CLR: CLKLOTO Position          */
#define I2CB_IRQ_CLR_CLKLOTO_Msk              (0x01UL << I2CB_IRQ_CLR_CLKLOTO_Pos)                    /*!< I2CB IRQ_CLR: CLKLOTO Mask              */
#define I2CB_IRQ_CLR_TXOVERFLOW_Pos           10                                                      /*!< I2CB IRQ_CLR: TXOVERFLOW Position       */
#define I2CB_IRQ_CLR_TXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_CLR_TXOVERFLOW_Pos)                 /*!< I2CB IRQ_CLR: TXOVERFLOW Mask           */
#define I2CB_IRQ_CLR_RXOVERFLOW_Pos           11                                                      /*!< I2CB IRQ_CLR: RXOVERFLOW Position       */
#define I2CB_IRQ_CLR_RXOVERFLOW_Msk           (0x01UL << I2CB_IRQ_CLR_RXOVERFLOW_Pos)                 /*!< I2CB IRQ_CLR: RXOVERFLOW Mask           */
#define I2CB_IRQ_CLR_TXREADY_Pos              12                                                      /*!< I2CB IRQ_CLR: TXREADY Position          */
#define I2CB_IRQ_CLR_TXREADY_Msk              (0x01UL << I2CB_IRQ_CLR_TXREADY_Pos)                    /*!< I2CB IRQ_CLR: TXREADY Mask              */
#define I2CB_IRQ_CLR_RXREADY_Pos              13                                                      /*!< I2CB IRQ_CLR: RXREADY Position          */
#define I2CB_IRQ_CLR_RXREADY_Msk              (0x01UL << I2CB_IRQ_CLR_RXREADY_Pos)                    /*!< I2CB IRQ_CLR: RXREADY Mask              */
#define I2CB_IRQ_CLR_TXEMPTY_Pos              14                                                      /*!< I2CB IRQ_CLR: TXEMPTY Position          */
#define I2CB_IRQ_CLR_TXEMPTY_Msk              (0x01UL << I2CB_IRQ_CLR_TXEMPTY_Pos)                    /*!< I2CB IRQ_CLR: TXEMPTY Mask              */
#define I2CB_IRQ_CLR_RXFULL_Pos               15                                                      /*!< I2CB IRQ_CLR: RXFULL Position           */
#define I2CB_IRQ_CLR_RXFULL_Msk               (0x01UL << I2CB_IRQ_CLR_RXFULL_Pos)                     /*!< I2CB IRQ_CLR: RXFULL Mask               */

/* --------------------------------  I2CB_FIFO_CLR  ------------------------------- */
#define I2CB_FIFO_CLR_RXFIFO_Pos              0                                                       /*!< I2CB FIFO_CLR: RXFIFO Position          */
#define I2CB_FIFO_CLR_RXFIFO_Msk              (0x01UL << I2CB_FIFO_CLR_RXFIFO_Pos)                    /*!< I2CB FIFO_CLR: RXFIFO Mask              */
#define I2CB_FIFO_CLR_TXFIFO_Pos              1                                                       /*!< I2CB FIFO_CLR: TXFIFO Position          */
#define I2CB_FIFO_CLR_TXFIFO_Msk              (0x01UL << I2CB_FIFO_CLR_TXFIFO_Pos)                    /*!< I2CB FIFO_CLR: TXFIFO Mask              */

/* --------------------------------  I2CB_S0_CTRL  -------------------------------- */
#define I2CB_S0_CTRL_CLKENABLED_Pos            0                                                       /*!< I2CB S0_CTRL: ENABLED Position          */
#define I2CB_S0_CTRL_CLKENABLED_Msk           (0x01UL << I2CB_S0_CTRL_CLKENABLED_Pos)                    /*!< I2CB S0_CTRL: ENABLED Mask              */
#define I2CB_S0_CTRL_ENABLED_Pos            1                                                       /*!< I2CB S0_CTRL: ACTIVATED Position        */
#define I2CB_S0_CTRL_ENABLED_Msk            (0x01UL << I2CB_S0_CTRL_ENABLED_Pos)                  /*!< I2CB S0_CTRL: ACTIVATED Mask            */
#define I2CB_S0_CTRL_ENABLE_Pos               2                                                       /*!< I2CB S0_CTRL: ACTIVE Position           */
#define I2CB_S0_CTRL_ENABLE_Msk               (0x01UL << I2CB_S0_CTRL_ENABLE_Pos)                     /*!< I2CB S0_CTRL: ACTIVE Mask               */
#define I2CB_S0_CTRL_TXFEMD_Pos               3                                                       /*!< I2CB S0_CTRL: TXFEMD Position           */
#define I2CB_S0_CTRL_TXFEMD_Msk               (0x01UL << I2CB_S0_CTRL_TXFEMD_Pos)                     /*!< I2CB S0_CTRL: TXFEMD Mask               */
#define I2CB_S0_CTRL_RXFFMD_Pos               4                                                       /*!< I2CB S0_CTRL: RXFFMD Position           */
#define I2CB_S0_CTRL_RXFFMD_Msk               (0x01UL << I2CB_S0_CTRL_RXFFMD_Pos)                     /*!< I2CB S0_CTRL: RXFFMD Mask               */

/* --------------------------------  I2CB_S0_MAXWORDS  --------------------------- */
#define I2CB_S0_MAXWORDS_MAXWORDS_Pos  				0                                                    		/*!< I2CB S0_MAXWORDS: MAXWORDS Position 		 */
#define I2CB_S0_MAXWORDS_MAXWORDS_Msk  				(0x07FFUL << I2CB_S0_MAXWORDS_MAXWORDS_Pos)							/*!< I2CB S0_MAXWORDS: MAXWORDS Mask    		 */
#define I2CB_S0_MAXWORDS_ENABLE_Pos  	 				31                                                   		/*!< I2CB S0_MAXWORDS: ENABLE Position 			 */
#define I2CB_S0_MAXWORDS_ENABLE_Msk  	 				(0x01UL << I2CB_S0_MAXWORDS_ENABLE_Pos) 								/*!< I2CB S0_MAXWORDS: ENABLE Mask  				 */

/* --------------------------------  I2CB_S0_ADDRESS  -------------------------- */
#define I2CB_S0_ADDRESS_RW_Pos  							0                                                     	/*!< I2CB S0_ADDRESS: RW Position 					 */
#define I2CB_S0_ADDRESS_RW_Msk  							(0x01UL << I2CB_S0_ADDRESS_RW_Pos)     									/*!< I2CB S0_ADDRESS: RW Mask     					 */
#define I2CB_S0_ADDRESS_ADDRESS_Pos  					1	                                                    	/*!< I2CB S0_ADDRESS: ADDRESS Position 			 */
#define I2CB_S0_ADDRESS_ADDRESS_Msk  					(0x3FFUL << I2CB_S0_ADDRESS_ADDRESS_Pos)    						/*!< I2CB S0_ADDRESS: ADDRESS Mask  				 */
#define I2CB_S0_ADDRESS_A10MODE_Pos  					15                                                    	/*!< I2CB S0_ADDRESS: A10MODE Position 			 */
#define I2CB_S0_ADDRESS_A10MODE_Msk  					(0x01UL << I2CB_S0_ADDRESS_A10MODE_Pos)     						/*!< I2CB S0_ADDRESS: A10MODE Mask  				 */

/* -------------------------------  I2CB_S0_ADDRESSMASK  -------------------------- */
#define I2CB_S0_ADDRESSMASK_RWMASK_Pos 				0                                                     	/*!< I2CB S0_ADDRESSMASK: RWMASK Position 	 */
#define I2CB_S0_ADDRESSMASK_RWMASK_Msk 				(0x01UL << I2CB_S0_ADDRESSMASK_RWMASK_Pos)  						/*!< I2CB S0_ADDRESSMASK: RWMASK Mask     	 */
#define I2CB_S0_ADDRESSMASK_MASK_Pos   				1	                                                   		/*!< I2CB S0_ADDRESSMASK: MASK Position 		 */
#define I2CB_S0_ADDRESSMASK_MASK_Msk   				(0x3FFUL << I2CB_S0_ADDRESSMASK_MASK_Pos)   						/*!< I2CB S0_ADDRESSMASK: MASK Mask  				 */

/* -------------------------------  I2CA_S0_ADDRESSMASK  -------------------------- */
#define I2CA_S0_ADDRESSMASK_RWMASK_Pos 				0                                                     	/*!< I2CA S0_ADDRESSMASK: RWMASK Position 	 */
#define I2CA_S0_ADDRESSMASK_RWMASK_Msk 				(0x01UL << I2CA_S0_ADDRESSMASK_RWMASK_Pos)  						/*!< I2CA S0_ADDRESSMASK: RWMASK Mask     	 */
#define I2CA_S0_ADDRESSMASK_MASK_Pos   				1	                                                   		/*!< I2CA S0_ADDRESSMASK: MASK Position 		 */
#define I2CA_S0_ADDRESSMASK_MASK_Msk   				(0x3FFUL << I2CA_S0_ADDRESSMASK_MASK_Pos)   						/*!< I2CA S0_ADDRESSMASK: MASK Mask  				 */

/* -------------------------------  I2CB_S0_STATUS  ------------------------------- */
#define I2CB_S0_STATUS_COMPLETED_Pos          0                                                       /*!< I2CB S0_STATUS: COMPLETED Position      */
#define I2CB_S0_STATUS_COMPLETED_Msk          (0x01UL << I2CB_S0_STATUS_COMPLETED_Pos)                /*!< I2CB S0_STATUS: COMPLETED Mask          */
#define I2CB_S0_STATUS_IDLE_Pos               1                                                       /*!< I2CB S0_STATUS: IDLE Position           */
#define I2CB_S0_STATUS_IDLE_Msk               (0x01UL << I2CB_S0_STATUS_IDLE_Pos)                     /*!< I2CB S0_STATUS: IDLE Mask               */
#define I2CB_S0_STATUS_WAITING_Pos            2                                                       /*!< I2CB S0_STATUS: WAITING Position        */
#define I2CB_S0_STATUS_WAITING_Msk            (0x01UL << I2CB_S0_STATUS_WAITING_Pos)                  /*!< I2CB S0_STATUS: WAITING Mask            */
#define I2CB_S0_STATUS_TXSTALLED_Pos          3                                                       /*!< I2CB S0_STATUS: TXSTALLED Position      */
#define I2CB_S0_STATUS_TXSTALLED_Msk          (0x01UL << I2CB_S0_STATUS_TXSTALLED_Pos)                /*!< I2CB S0_STATUS: TXSTALLED Mask          */
#define I2CB_S0_STATUS_RXSTALLED_Pos          4                                                       /*!< I2CB S0_STATUS: RXSTALLED Position      */
#define I2CB_S0_STATUS_RXSTALLED_Msk          (0x01UL << I2CB_S0_STATUS_RXSTALLED_Pos)                /*!< I2CB S0_STATUS: RXSTALLED Mask          */
#define I2CB_S0_STATUS_ADDRESSMATCH_Pos       5                                                       /*!< I2CB S0_STATUS: ADDRESSMATCH Position   */
#define I2CB_S0_STATUS_ADDRESSMATCH_Msk       (0x01UL << I2CB_S0_STATUS_ADDRESSMATCH_Pos)             /*!< I2CB S0_STATUS: ADDRESSMATCH Mask       */
#define I2CB_S0_STATUS_NACKDATA_Pos           6                                                       /*!< I2CB S0_STATUS: NACKDATA Position       */
#define I2CB_S0_STATUS_NACKDATA_Msk           (0x01UL << I2CB_S0_STATUS_NACKDATA_Pos)                 /*!< I2CB S0_STATUS: NACKDATA Mask           */
#define I2CB_S0_STATUS_RXDATAFIRST_Pos        7                                                       /*!< I2CB S0_STATUS: RXDATAFIRST Position    */
#define I2CB_S0_STATUS_RXDATAFIRST_Msk        (0x01UL << I2CB_S0_STATUS_RXDATAFIRST_Pos)              /*!< I2CB S0_STATUS: RXDATAFIRST Mask        */
#define I2CB_S0_STATUS_RXNEMPTY_Pos           8                                                       /*!< I2CB S0_STATUS: RXNEMPTY Position       */
#define I2CB_S0_STATUS_RXNEMPTY_Msk           (0x01UL << I2CB_S0_STATUS_RXNEMPTY_Pos)                 /*!< I2CB S0_STATUS: RXNEMPTY Mask           */
#define I2CB_S0_STATUS_RXFULL_Pos             9                                                       /*!< I2CB S0_STATUS: RXFULL Position         */
#define I2CB_S0_STATUS_RXFULL_Msk             (0x01UL << I2CB_S0_STATUS_RXFULL_Pos)                   /*!< I2CB S0_STATUS: RXFULL Mask             */
#define I2CB_S0_STATUS_RXTRIGGER_Pos           11                                                      /*!< I2CB S0_STATUS: RXTRIGGER Position       */
#define I2CB_S0_STATUS_RXTRIGGER_Msk           (0x01UL << I2CB_S0_STATUS_RXTRIGGER_Pos)                 /*!< I2CB S0_STATUS: RXTRIGGER Mask           */
#define I2CB_S0_STATUS_TXEMPTY_Pos            12                                                      /*!< I2CB S0_STATUS: TXEMPTY Position        */
#define I2CB_S0_STATUS_TXEMPTY_Msk            (0x01UL << I2CB_S0_STATUS_TXEMPTY_Pos)                  /*!< I2CB S0_STATUS: TXEMPTY Mask            */
#define I2CB_S0_STATUS_TXNFULL_Pos            13                                                      /*!< I2CB S0_STATUS: TXNFULL Position        */
#define I2CB_S0_STATUS_TXNFULL_Msk            (0x01UL << I2CB_S0_STATUS_TXNFULL_Pos)                  /*!< I2CB S0_STATUS: TXNFULL Mask            */
#define I2CB_S0_STATUS_TXTRIGGER_Pos           15                                                      /*!< I2CB S0_STATUS: TXTRIGGER Position       */
#define I2CB_S0_STATUS_TXTRIGGER_Msk           (0x01UL << I2CB_S0_STATUS_TXTRIGGER_Pos)                 /*!< I2CB S0_STATUS: TXTRIGGER Mask           */
#define I2CB_S0_STATUS_RAW_BUSY_Pos           29                                                      /*!< I2CB S0_STATUS: RAW_BUSY Position       */
#define I2CB_S0_STATUS_RAW_BUSY_Msk           (0x01UL << I2CB_S0_STATUS_RAW_BUSY_Pos)                 /*!< I2CB S0_STATUS: RAW_BUSY Mask           */
#define I2CB_S0_STATUS_RAW_SDA_Pos            30                                                      /*!< I2CB S0_STATUS: RAW_SDA Position        */
#define I2CB_S0_STATUS_RAW_SDA_Msk            (0x01UL << I2CB_S0_STATUS_RAW_SDA_Pos)                  /*!< I2CB S0_STATUS: RAW_SDA Mask            */
#define I2CB_S0_STATUS_RAW_SCL_Pos            31                                                      /*!< I2CB S0_STATUS: RAW_SCL Position        */
#define I2CB_S0_STATUS_RAW_SCL_Msk            (0x01UL << I2CB_S0_STATUS_RAW_SCL_Pos)                  /*!< I2CB S0_STATUS: RAW_SCL Mask            */

/* ---------------------------  I2CB_S0_STATE  ------------------------------ */
#define I2CB_S0_STATE_STATE_Pos  							0                                                       /*!< I2CB S0_STATE: STATE Position 					 */
#define I2CB_S0_STATE_STATE_Msk  							(0x07UL << I2CB_S0_STATE_STATE_Pos)     								/*!< I2CB S0_STATE: STATE Mask     					 */
#define I2CB_S0_STATE_STEP_Pos  							4	                                                      /*!< I2CB S0_STATE: STEP Position 					 */
#define I2CB_S0_STATE_STEP_Msk  							(0x0FUL << I2CB_S0_STATE_STEP_Pos)        							/*!< I2CB S0_STATE: STEP Mask  							 */
#define I2CB_S0_STATE_RXFIFO_Pos  						8                                                       /*!< I2CB S0_STATE: RXFIFO Position 				 */
#define I2CB_S0_STATE_RXFIFO_Msk  						(0x01FUL << I2CB_S0_STATE_RXFIFO_Pos)        						/*!< I2CB S0_STATE: RXFIFO Mask  						 */
#define I2CB_S0_STATE_TXFIFO_Pos  						14                                                      /*!< I2CB S0_STATE: TXFIFO Position 				 */
#define I2CB_S0_STATE_TXFIFO_Msk  						(0x01FUL << I2CB_S0_STATE_TXFIFO_Pos)     							/*!< I2CB S0_STATE: TXFIFO Mask     				 */

/* -------------------------------  I2CB_S0_IRQ_ENB  ------------------------------ */
#define I2CB_S0_IRQ_ENB_COMPLETED_Pos         0                                                       /*!< I2CB S0_IRQ_ENB: COMPLETED Position     */
#define I2CB_S0_IRQ_ENB_COMPLETED_Msk         (0x01UL << I2CB_S0_IRQ_ENB_COMPLETED_Pos)               /*!< I2CB S0_IRQ_ENB: COMPLETED Mask         */
#define I2CB_S0_IRQ_ENB_IDLE_Pos              1                                                       /*!< I2CB S0_IRQ_ENB: IDLE Position          */
#define I2CB_S0_IRQ_ENB_IDLE_Msk              (0x01UL << I2CB_S0_IRQ_ENB_IDLE_Pos)                    /*!< I2CB S0_IRQ_ENB: IDLE Mask              */
#define I2CB_S0_IRQ_ENB_WAITING_Pos           2                                                       /*!< I2CB S0_IRQ_ENB: WAITING Position       */
#define I2CB_S0_IRQ_ENB_WAITING_Msk           (0x01UL << I2CB_S0_IRQ_ENB_WAITING_Pos)                 /*!< I2CB S0_IRQ_ENB: WAITING Mask           */
#define I2CB_S0_IRQ_ENB_TXSTALLED_Pos         3                                                       /*!< I2CB S0_IRQ_ENB: TXSTALLED Position     */
#define I2CB_S0_IRQ_ENB_TXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_ENB_TXSTALLED_Pos)               /*!< I2CB S0_IRQ_ENB: TXSTALLED Mask         */
#define I2CB_S0_IRQ_ENB_RXSTALLED_Pos         4                                                       /*!< I2CB S0_IRQ_ENB: RXSTALLED Position     */
#define I2CB_S0_IRQ_ENB_RXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_ENB_RXSTALLED_Pos)               /*!< I2CB S0_IRQ_ENB: RXSTALLED Mask         */
#define I2CB_S0_IRQ_ENB_ADDRESSMATCH_Pos      5                                                       /*!< I2CB S0_IRQ_ENB: ADDRESSMATCH Position  */
#define I2CB_S0_IRQ_ENB_ADDRESSMATCH_Msk      (0x01UL << I2CB_S0_IRQ_ENB_ADDRESSMATCH_Pos)            /*!< I2CB S0_IRQ_ENB: ADDRESSMATCH Mask      */
#define I2CB_S0_IRQ_ENB_NACKDATA_Pos          6                                                       /*!< I2CB S0_IRQ_ENB: NACKDATA Position      */
#define I2CB_S0_IRQ_ENB_NACKDATA_Msk          (0x01UL << I2CB_S0_IRQ_ENB_NACKDATA_Pos)                /*!< I2CB S0_IRQ_ENB: NACKDATA Mask          */
#define I2CB_S0_IRQ_ENB_RXDATAFIRST_Pos       7                                                       /*!< I2CB S0_IRQ_ENB: RXDATAFIRST Position   */
#define I2CB_S0_IRQ_ENB_RXDATAFIRST_Msk       (0x01UL << I2CB_S0_IRQ_ENB_RXDATAFIRST_Pos)             /*!< I2CB S0_IRQ_ENB: RXDATAFIRST Mask       */
#define I2CB_S0_IRQ_ENB_I2C_START_Pos         8                                                       /*!< I2CB S0_IRQ_ENB: I2C_START Position     */
#define I2CB_S0_IRQ_ENB_I2C_START_Msk         (0x01UL << I2CB_S0_IRQ_ENB_I2C_START_Pos)               /*!< I2CB S0_IRQ_ENB: I2C_START Mask         */
#define I2CB_S0_IRQ_ENB_I2C_STOP_Pos          9                                                       /*!< I2CB S0_IRQ_ENB: I2C_STOP Position      */
#define I2CB_S0_IRQ_ENB_I2C_STOP_Msk          (0x01UL << I2CB_S0_IRQ_ENB_I2C_STOP_Pos)                /*!< I2CB S0_IRQ_ENB: I2C_STOP Mask          */
#define I2CB_S0_IRQ_ENB_TXUNDERFLOW_Pos       10                                                      /*!< I2CB S0_IRQ_ENB: TXUNDERFLOW Position   */
#define I2CB_S0_IRQ_ENB_TXUNDERFLOW_Msk       (0x01UL << I2CB_S0_IRQ_ENB_TXUNDERFLOW_Pos)             /*!< I2CB S0_IRQ_ENB: TXUNDERFLOW Mask       */
#define I2CB_S0_IRQ_ENB_RXOVERFLOW_Pos        11                                                      /*!< I2CB S0_IRQ_ENB: RXOVERFLOW Position    */
#define I2CB_S0_IRQ_ENB_RXOVERFLOW_Msk        (0x01UL << I2CB_S0_IRQ_ENB_RXOVERFLOW_Pos)              /*!< I2CB S0_IRQ_ENB: RXOVERFLOW Mask        */
#define I2CB_S0_IRQ_ENB_TXREADY_Pos           12                                                      /*!< I2CB S0_IRQ_ENB: TXREADY Position       */
#define I2CB_S0_IRQ_ENB_TXREADY_Msk           (0x01UL << I2CB_S0_IRQ_ENB_TXREADY_Pos)                 /*!< I2CB S0_IRQ_ENB: TXREADY Mask           */
#define I2CB_S0_IRQ_ENB_RXREADY_Pos           13                                                      /*!< I2CB S0_IRQ_ENB: RXREADY Position       */
#define I2CB_S0_IRQ_ENB_RXREADY_Msk           (0x01UL << I2CB_S0_IRQ_ENB_RXREADY_Pos)                 /*!< I2CB S0_IRQ_ENB: RXREADY Mask           */
#define I2CB_S0_IRQ_ENB_TXEMPTY_Pos           14                                                      /*!< I2CB S0_IRQ_ENB: TXEMPTY Position       */
#define I2CB_S0_IRQ_ENB_TXEMPTY_Msk           (0x01UL << I2CB_S0_IRQ_ENB_TXEMPTY_Pos)                 /*!< I2CB S0_IRQ_ENB: TXEMPTY Mask           */
#define I2CB_S0_IRQ_ENB_RXFULL_Pos            15                                                      /*!< I2CB S0_IRQ_ENB: RXFULL Position        */
#define I2CB_S0_IRQ_ENB_RXFULL_Msk            (0x01UL << I2CB_S0_IRQ_ENB_RXFULL_Pos)                  /*!< I2CB S0_IRQ_ENB: RXFULL Mask            */

/* -------------------------------  I2CB_S0_IRQ_RAW  ------------------------------ */
#define I2CB_S0_IRQ_RAW_COMPLETED_Pos         0                                                       /*!< I2CB S0_IRQ_RAW: COMPLETED Position     */
#define I2CB_S0_IRQ_RAW_COMPLETED_Msk         (0x01UL << I2CB_S0_IRQ_RAW_COMPLETED_Pos)               /*!< I2CB S0_IRQ_RAW: COMPLETED Mask         */
#define I2CB_S0_IRQ_RAW_IDLE_Pos              1                                                       /*!< I2CB S0_IRQ_RAW: IDLE Position          */
#define I2CB_S0_IRQ_RAW_IDLE_Msk              (0x01UL << I2CB_S0_IRQ_RAW_IDLE_Pos)                    /*!< I2CB S0_IRQ_RAW: IDLE Mask              */
#define I2CB_S0_IRQ_RAW_WAITING_Pos           2                                                       /*!< I2CB S0_IRQ_RAW: WAITING Position       */
#define I2CB_S0_IRQ_RAW_WAITING_Msk           (0x01UL << I2CB_S0_IRQ_RAW_WAITING_Pos)                 /*!< I2CB S0_IRQ_RAW: WAITING Mask           */
#define I2CB_S0_IRQ_RAW_TXSTALLED_Pos         3                                                       /*!< I2CB S0_IRQ_RAW: TXSTALLED Position     */
#define I2CB_S0_IRQ_RAW_TXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_RAW_TXSTALLED_Pos)               /*!< I2CB S0_IRQ_RAW: TXSTALLED Mask         */
#define I2CB_S0_IRQ_RAW_RXSTALLED_Pos         4                                                       /*!< I2CB S0_IRQ_RAW: RXSTALLED Position     */
#define I2CB_S0_IRQ_RAW_RXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_RAW_RXSTALLED_Pos)               /*!< I2CB S0_IRQ_RAW: RXSTALLED Mask         */
#define I2CB_S0_IRQ_RAW_ADDRESSMATCH_Pos      5                                                       /*!< I2CB S0_IRQ_RAW: ADDRESSMATCH Position  */
#define I2CB_S0_IRQ_RAW_ADDRESSMATCH_Msk      (0x01UL << I2CB_S0_IRQ_RAW_ADDRESSMATCH_Pos)            /*!< I2CB S0_IRQ_RAW: ADDRESSMATCH Mask      */
#define I2CB_S0_IRQ_RAW_NACKDATA_Pos          6                                                       /*!< I2CB S0_IRQ_RAW: NACKDATA Position      */
#define I2CB_S0_IRQ_RAW_NACKDATA_Msk          (0x01UL << I2CB_S0_IRQ_RAW_NACKDATA_Pos)                /*!< I2CB S0_IRQ_RAW: NACKDATA Mask          */
#define I2CB_S0_IRQ_RAW_RXDATAFIRST_Pos       7                                                       /*!< I2CB S0_IRQ_RAW: RXDATAFIRST Position   */
#define I2CB_S0_IRQ_RAW_RXDATAFIRST_Msk       (0x01UL << I2CB_S0_IRQ_RAW_RXDATAFIRST_Pos)             /*!< I2CB S0_IRQ_RAW: RXDATAFIRST Mask       */
#define I2CB_S0_IRQ_RAW_I2C_START_Pos         8                                                       /*!< I2CB S0_IRQ_RAW: I2C_START Position     */
#define I2CB_S0_IRQ_RAW_I2C_START_Msk         (0x01UL << I2CB_S0_IRQ_RAW_I2C_START_Pos)               /*!< I2CB S0_IRQ_RAW: I2C_START Mask         */
#define I2CB_S0_IRQ_RAW_I2C_STOP_Pos          9                                                       /*!< I2CB S0_IRQ_RAW: I2C_STOP Position      */
#define I2CB_S0_IRQ_RAW_I2C_STOP_Msk          (0x01UL << I2CB_S0_IRQ_RAW_I2C_STOP_Pos)                /*!< I2CB S0_IRQ_RAW: I2C_STOP Mask          */
#define I2CB_S0_IRQ_RAW_TXUNDERFLOW_Pos       10                                                      /*!< I2CB S0_IRQ_RAW: TXUNDERFLOW Position   */
#define I2CB_S0_IRQ_RAW_TXUNDERFLOW_Msk       (0x01UL << I2CB_S0_IRQ_RAW_TXUNDERFLOW_Pos)             /*!< I2CB S0_IRQ_RAW: TXUNDERFLOW Mask       */
#define I2CB_S0_IRQ_RAW_RXOVERFLOW_Pos        11                                                      /*!< I2CB S0_IRQ_RAW: RXOVERFLOW Position    */
#define I2CB_S0_IRQ_RAW_RXOVERFLOW_Msk        (0x01UL << I2CB_S0_IRQ_RAW_RXOVERFLOW_Pos)              /*!< I2CB S0_IRQ_RAW: RXOVERFLOW Mask        */
#define I2CB_S0_IRQ_RAW_TXREADY_Pos           12                                                      /*!< I2CB S0_IRQ_RAW: TXREADY Position       */
#define I2CB_S0_IRQ_RAW_TXREADY_Msk           (0x01UL << I2CB_S0_IRQ_RAW_TXREADY_Pos)                 /*!< I2CB S0_IRQ_RAW: TXREADY Mask           */
#define I2CB_S0_IRQ_RAW_RXREADY_Pos           13                                                      /*!< I2CB S0_IRQ_RAW: RXREADY Position       */
#define I2CB_S0_IRQ_RAW_RXREADY_Msk           (0x01UL << I2CB_S0_IRQ_RAW_RXREADY_Pos)                 /*!< I2CB S0_IRQ_RAW: RXREADY Mask           */
#define I2CB_S0_IRQ_RAW_TXEMPTY_Pos           14                                                      /*!< I2CB S0_IRQ_RAW: TXEMPTY Position       */
#define I2CB_S0_IRQ_RAW_TXEMPTY_Msk           (0x01UL << I2CB_S0_IRQ_RAW_TXEMPTY_Pos)                 /*!< I2CB S0_IRQ_RAW: TXEMPTY Mask           */
#define I2CB_S0_IRQ_RAW_RXFULL_Pos            15                                                      /*!< I2CB S0_IRQ_RAW: RXFULL Position        */
#define I2CB_S0_IRQ_RAW_RXFULL_Msk            (0x01UL << I2CB_S0_IRQ_RAW_RXFULL_Pos)                  /*!< I2CB S0_IRQ_RAW: RXFULL Mask            */

/* -------------------------------  I2CB_S0_IRQ_END  ------------------------------ */
#define I2CB_S0_IRQ_END_COMPLETED_Pos         0                                                       /*!< I2CB S0_IRQ_END: COMPLETED Position     */
#define I2CB_S0_IRQ_END_COMPLETED_Msk         (0x01UL << I2CB_S0_IRQ_END_COMPLETED_Pos)               /*!< I2CB S0_IRQ_END: COMPLETED Mask         */
#define I2CB_S0_IRQ_END_IDLE_Pos              1                                                       /*!< I2CB S0_IRQ_END: IDLE Position          */
#define I2CB_S0_IRQ_END_IDLE_Msk              (0x01UL << I2CB_S0_IRQ_END_IDLE_Pos)                    /*!< I2CB S0_IRQ_END: IDLE Mask              */
#define I2CB_S0_IRQ_END_WAITING_Pos           2                                                       /*!< I2CB S0_IRQ_END: WAITING Position       */
#define I2CB_S0_IRQ_END_WAITING_Msk           (0x01UL << I2CB_S0_IRQ_END_WAITING_Pos)                 /*!< I2CB S0_IRQ_END: WAITING Mask           */
#define I2CB_S0_IRQ_END_TXSTALLED_Pos         3                                                       /*!< I2CB S0_IRQ_END: TXSTALLED Position     */
#define I2CB_S0_IRQ_END_TXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_END_TXSTALLED_Pos)               /*!< I2CB S0_IRQ_END: TXSTALLED Mask         */
#define I2CB_S0_IRQ_END_RXSTALLED_Pos         4                                                       /*!< I2CB S0_IRQ_END: RXSTALLED Position     */
#define I2CB_S0_IRQ_END_RXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_END_RXSTALLED_Pos)               /*!< I2CB S0_IRQ_END: RXSTALLED Mask         */
#define I2CB_S0_IRQ_END_ADDRESSMATCH_Pos      5                                                       /*!< I2CB S0_IRQ_END: ADDRESSMATCH Position  */
#define I2CB_S0_IRQ_END_ADDRESSMATCH_Msk      (0x01UL << I2CB_S0_IRQ_END_ADDRESSMATCH_Pos)            /*!< I2CB S0_IRQ_END: ADDRESSMATCH Mask      */
#define I2CB_S0_IRQ_END_NACKDATA_Pos          6                                                       /*!< I2CB S0_IRQ_END: NACKDATA Position      */
#define I2CB_S0_IRQ_END_NACKDATA_Msk          (0x01UL << I2CB_S0_IRQ_END_NACKDATA_Pos)                /*!< I2CB S0_IRQ_END: NACKDATA Mask          */
#define I2CB_S0_IRQ_END_RXDATAFIRST_Pos       7                                                       /*!< I2CB S0_IRQ_END: RXDATAFIRST Position   */
#define I2CB_S0_IRQ_END_RXDATAFIRST_Msk       (0x01UL << I2CB_S0_IRQ_END_RXDATAFIRST_Pos)             /*!< I2CB S0_IRQ_END: RXDATAFIRST Mask       */
#define I2CB_S0_IRQ_END_I2C_START_Pos         8                                                       /*!< I2CB S0_IRQ_END: I2C_START Position     */
#define I2CB_S0_IRQ_END_I2C_START_Msk         (0x01UL << I2CB_S0_IRQ_END_I2C_START_Pos)               /*!< I2CB S0_IRQ_END: I2C_START Mask         */
#define I2CB_S0_IRQ_END_I2C_STOP_Pos          9                                                       /*!< I2CB S0_IRQ_END: I2C_STOP Position      */
#define I2CB_S0_IRQ_END_I2C_STOP_Msk          (0x01UL << I2CB_S0_IRQ_END_I2C_STOP_Pos)                /*!< I2CB S0_IRQ_END: I2C_STOP Mask          */
#define I2CB_S0_IRQ_END_TXUNDERFLOW_Pos       10                                                      /*!< I2CB S0_IRQ_END: TXUNDERFLOW Position   */
#define I2CB_S0_IRQ_END_TXUNDERFLOW_Msk       (0x01UL << I2CB_S0_IRQ_END_TXUNDERFLOW_Pos)             /*!< I2CB S0_IRQ_END: TXUNDERFLOW Mask       */
#define I2CB_S0_IRQ_END_RXOVERFLOW_Pos        11                                                      /*!< I2CB S0_IRQ_END: RXOVERFLOW Position    */
#define I2CB_S0_IRQ_END_RXOVERFLOW_Msk        (0x01UL << I2CB_S0_IRQ_END_RXOVERFLOW_Pos)              /*!< I2CB S0_IRQ_END: RXOVERFLOW Mask        */
#define I2CB_S0_IRQ_END_TXREADY_Pos           12                                                      /*!< I2CB S0_IRQ_END: TXREADY Position       */
#define I2CB_S0_IRQ_END_TXREADY_Msk           (0x01UL << I2CB_S0_IRQ_END_TXREADY_Pos)                 /*!< I2CB S0_IRQ_END: TXREADY Mask           */
#define I2CB_S0_IRQ_END_RXREADY_Pos           13                                                      /*!< I2CB S0_IRQ_END: RXREADY Position       */
#define I2CB_S0_IRQ_END_RXREADY_Msk           (0x01UL << I2CB_S0_IRQ_END_RXREADY_Pos)                 /*!< I2CB S0_IRQ_END: RXREADY Mask           */
#define I2CB_S0_IRQ_END_TXEMPTY_Pos           14                                                      /*!< I2CB S0_IRQ_END: TXEMPTY Position       */
#define I2CB_S0_IRQ_END_TXEMPTY_Msk           (0x01UL << I2CB_S0_IRQ_END_TXEMPTY_Pos)                 /*!< I2CB S0_IRQ_END: TXEMPTY Mask           */
#define I2CB_S0_IRQ_END_RXFULL_Pos            15                                                      /*!< I2CB S0_IRQ_END: RXFULL Position        */
#define I2CB_S0_IRQ_END_RXFULL_Msk            (0x01UL << I2CB_S0_IRQ_END_RXFULL_Pos)                  /*!< I2CB S0_IRQ_END: RXFULL Mask            */

/* -------------------------------  I2CB_S0_IRQ_CLR  ------------------------------ */
#define I2CB_S0_IRQ_CLR_COMPLETED_Pos         0                                                       /*!< I2CB S0_IRQ_CLR: COMPLETED Position     */
#define I2CB_S0_IRQ_CLR_COMPLETED_Msk         (0x01UL << I2CB_S0_IRQ_CLR_COMPLETED_Pos)               /*!< I2CB S0_IRQ_CLR: COMPLETED Mask         */
#define I2CB_S0_IRQ_CLR_IDLE_Pos              1                                                       /*!< I2CB S0_IRQ_CLR: IDLE Position          */
#define I2CB_S0_IRQ_CLR_IDLE_Msk              (0x01UL << I2CB_S0_IRQ_CLR_IDLE_Pos)                    /*!< I2CB S0_IRQ_CLR: IDLE Mask              */
#define I2CB_S0_IRQ_CLR_WAITING_Pos           2                                                       /*!< I2CB S0_IRQ_CLR: WAITING Position       */
#define I2CB_S0_IRQ_CLR_WAITING_Msk           (0x01UL << I2CB_S0_IRQ_CLR_WAITING_Pos)                 /*!< I2CB S0_IRQ_CLR: WAITING Mask           */
#define I2CB_S0_IRQ_CLR_TXSTALLED_Pos         3                                                       /*!< I2CB S0_IRQ_CLR: TXSTALLED Position     */
#define I2CB_S0_IRQ_CLR_TXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_CLR_TXSTALLED_Pos)               /*!< I2CB S0_IRQ_CLR: TXSTALLED Mask         */
#define I2CB_S0_IRQ_CLR_RXSTALLED_Pos         4                                                       /*!< I2CB S0_IRQ_CLR: RXSTALLED Position     */
#define I2CB_S0_IRQ_CLR_RXSTALLED_Msk         (0x01UL << I2CB_S0_IRQ_CLR_RXSTALLED_Pos)               /*!< I2CB S0_IRQ_CLR: RXSTALLED Mask         */
#define I2CB_S0_IRQ_CLR_ADDRESSMATCH_Pos      5                                                       /*!< I2CB S0_IRQ_CLR: ADDRESSMATCH Position  */
#define I2CB_S0_IRQ_CLR_ADDRESSMATCH_Msk      (0x01UL << I2CB_S0_IRQ_CLR_ADDRESSMATCH_Pos)            /*!< I2CB S0_IRQ_CLR: ADDRESSMATCH Mask      */
#define I2CB_S0_IRQ_CLR_NACKDATA_Pos          6                                                       /*!< I2CB S0_IRQ_CLR: NACKDATA Position      */
#define I2CB_S0_IRQ_CLR_NACKDATA_Msk          (0x01UL << I2CB_S0_IRQ_CLR_NACKDATA_Pos)                /*!< I2CB S0_IRQ_CLR: NACKDATA Mask          */
#define I2CB_S0_IRQ_CLR_RXDATAFIRST_Pos       7                                                       /*!< I2CB S0_IRQ_CLR: RXDATAFIRST Position   */
#define I2CB_S0_IRQ_CLR_RXDATAFIRST_Msk       (0x01UL << I2CB_S0_IRQ_CLR_RXDATAFIRST_Pos)             /*!< I2CB S0_IRQ_CLR: RXDATAFIRST Mask       */
#define I2CB_S0_IRQ_CLR_I2C_START_Pos         8                                                       /*!< I2CB S0_IRQ_CLR: I2C_START Position     */
#define I2CB_S0_IRQ_CLR_I2C_START_Msk         (0x01UL << I2CB_S0_IRQ_CLR_I2C_START_Pos)               /*!< I2CB S0_IRQ_CLR: I2C_START Mask         */
#define I2CB_S0_IRQ_CLR_I2C_STOP_Pos          9                                                       /*!< I2CB S0_IRQ_CLR: I2C_STOP Position      */
#define I2CB_S0_IRQ_CLR_I2C_STOP_Msk          (0x01UL << I2CB_S0_IRQ_CLR_I2C_STOP_Pos)                /*!< I2CB S0_IRQ_CLR: I2C_STOP Mask          */
#define I2CB_S0_IRQ_CLR_TXUNDERFLOW_Pos       10                                                      /*!< I2CB S0_IRQ_CLR: TXUNDERFLOW Position   */
#define I2CB_S0_IRQ_CLR_TXUNDERFLOW_Msk       (0x01UL << I2CB_S0_IRQ_CLR_TXUNDERFLOW_Pos)             /*!< I2CB S0_IRQ_CLR: TXUNDERFLOW Mask       */
#define I2CB_S0_IRQ_CLR_RXOVERFLOW_Pos        11                                                      /*!< I2CB S0_IRQ_CLR: RXOVERFLOW Position    */
#define I2CB_S0_IRQ_CLR_RXOVERFLOW_Msk        (0x01UL << I2CB_S0_IRQ_CLR_RXOVERFLOW_Pos)              /*!< I2CB S0_IRQ_CLR: RXOVERFLOW Mask        */
#define I2CB_S0_IRQ_CLR_TXREADY_Pos           12                                                      /*!< I2CB S0_IRQ_CLR: TXREADY Position       */
#define I2CB_S0_IRQ_CLR_TXREADY_Msk           (0x01UL << I2CB_S0_IRQ_CLR_TXREADY_Pos)                 /*!< I2CB S0_IRQ_CLR: TXREADY Mask           */
#define I2CB_S0_IRQ_CLR_RXREADY_Pos           13                                                      /*!< I2CB S0_IRQ_CLR: RXREADY Position       */
#define I2CB_S0_IRQ_CLR_RXREADY_Msk           (0x01UL << I2CB_S0_IRQ_CLR_RXREADY_Pos)                 /*!< I2CB S0_IRQ_CLR: RXREADY Mask           */
#define I2CB_S0_IRQ_CLR_TXEMPTY_Pos           14                                                      /*!< I2CB S0_IRQ_CLR: TXEMPTY Position       */
#define I2CB_S0_IRQ_CLR_TXEMPTY_Msk           (0x01UL << I2CB_S0_IRQ_CLR_TXEMPTY_Pos)                 /*!< I2CB S0_IRQ_CLR: TXEMPTY Mask           */
#define I2CB_S0_IRQ_CLR_RXFULL_Pos            15                                                      /*!< I2CB S0_IRQ_CLR: RXFULL Position        */
#define I2CB_S0_IRQ_CLR_RXFULL_Msk            (0x01UL << I2CB_S0_IRQ_CLR_RXFULL_Pos)                  /*!< I2CB S0_IRQ_CLR: RXFULL Mask            */

/* ------------------------------  I2CB_S0_FIFO_CLR  ------------------------------ */
#define I2CB_S0_FIFO_CLR_RXFIFO_Pos           0                                                       /*!< I2CB S0_FIFO_CLR: RXFIFO Position       */
#define I2CB_S0_FIFO_CLR_RXFIFO_Msk           (0x01UL << I2CB_S0_FIFO_CLR_RXFIFO_Pos)                 /*!< I2CB S0_FIFO_CLR: RXFIFO Mask           */
#define I2CB_S0_FIFO_CLR_TXFIFO_Pos           1                                                       /*!< I2CB S0_FIFO_CLR: TXFIFO Position       */
#define I2CB_S0_FIFO_CLR_TXFIFO_Msk           (0x01UL << I2CB_S0_FIFO_CLR_TXFIFO_Pos)                 /*!< I2CB S0_FIFO_CLR: TXFIFO Mask           */

/* -----------------------------  I2CB_S0_ADDRESSB  -------------------------- */
#define I2CB_S0_ADDRESSB_RW_Pos  							0                                                   		/*!< I2CB S0_ADDRESSB: RW Position 					 */
#define I2CB_S0_ADDRESSB_RW_Msk  			  			(0x01UL << I2CB_S0_ADDRESSB_RW_Pos)     								/*!< I2CB S0_ADDRESSB: RW Mask     					 */
#define I2CB_S0_ADDRESSB_ADDRESS_Pos  				1	                                                  		/*!< I2CB S0_ADDRESSB: ADDRESS Position 		 */
#define I2CB_S0_ADDRESSB_ADDRESS_Msk  				(0x3FFUL << I2CB_S0_ADDRESSB_ADDRESS_Pos) 							/*!< I2CB S0_ADDRESSB: ADDRESS Mask  				 */
#define I2CB_S0_ADDRESSB_ADDRESSBEN_Pos				15                                                  		/*!< I2CB S0_ADDRESSB: ADDRESSBEN Position	 */
#define I2CB_S0_ADDRESSB_ADDRESSBEN_Msk  			(0x01UL << I2CB_S0_ADDRESSB_ADDRESSBEN_Pos)							/*!< I2CB S0_ADDRESSB: ADDRESSBEN Mask			 */

/* ---------------------------  I2CA_S0_ADDRESSMASKB  -------------------------- */
#define I2CB_S0_ADDRESSMASKB_RWMASK_Pos 			0                                                    		/*!< I2CB S0_ADDRESSMASKB: RWMASK Position 		*/
#define I2CB_S0_ADDRESSMASKB_RWMASK_Msk 			(0x01UL << I2CB_S0_ADDRESSMASKB_RWMASK_Pos)							/*!< I2CB S0_ADDRESSMASKB: RWMASK Mask     		*/
#define I2CB_S0_ADDRESSMASKB_MASK_Pos   			1	                                                   		/*!< I2CB S0_ADDRESSMASKB: MASK Position 			*/
#define I2CB_S0_ADDRESSMASKB_MASK_Msk   			(0x3FFUL << I2CB_S0_ADDRESSMASKB_MASK_Pos) 							/*!< I2CB S0_ADDRESSMASKB: MASK Mask  				*/

/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define VOR_SYSCONFIG_BASE              0x40000000UL
#define VOR_IRQSEL_BASE                 0x40001000UL
#define VOR_IOCONFIG_BASE               0x40002000UL
#define VOR_UTILITY_BASE                0x40003000UL
#define VOR_PORTA_BASE                  0x50000000UL
#define VOR_PORTB_BASE                  0x50001000UL
#define VOR_TIM0_BASE                   0x40020000UL
#define VOR_TIM1_BASE                   0x40021000UL
#define VOR_TIM2_BASE                   0x40022000UL
#define VOR_TIM3_BASE                   0x40023000UL
#define VOR_TIM4_BASE                   0x40024000UL
#define VOR_TIM5_BASE                   0x40025000UL
#define VOR_TIM6_BASE                   0x40026000UL
#define VOR_TIM7_BASE                   0x40027000UL
#define VOR_TIM8_BASE                   0x40028000UL
#define VOR_TIM9_BASE                   0x40029000UL
#define VOR_TIM10_BASE                  0x4002A000UL
#define VOR_TIM11_BASE                  0x4002B000UL
#define VOR_TIM12_BASE                  0x4002C000UL
#define VOR_TIM13_BASE                  0x4002D000UL
#define VOR_TIM14_BASE                  0x4002E000UL
#define VOR_TIM15_BASE                  0x4002F000UL
#define VOR_TIM16_BASE                  0x40030000UL
#define VOR_TIM17_BASE                  0x40031000UL
#define VOR_TIM18_BASE                  0x40032000UL
#define VOR_TIM19_BASE                  0x40033000UL
#define VOR_TIM20_BASE                  0x40034000UL
#define VOR_TIM21_BASE                  0x40035000UL
#define VOR_TIM22_BASE                  0x40036000UL
#define VOR_TIM23_BASE                  0x40037000UL
#define VOR_UARTA_BASE                  0x40040000UL
#define VOR_UARTB_BASE                  0x40041000UL
#define VOR_SPIA_BASE                   0x40050000UL
#define VOR_SPIB_BASE                   0x40051000UL
#define VOR_SPIC_BASE                   0x40052000UL
#define VOR_I2CA_BASE                   0x40060000UL
#define VOR_I2CB_BASE                   0x40061000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define VOR_SYSCONFIG                   ((VOR_SYSCONFIG_Type      *) VOR_SYSCONFIG_BASE)
#define VOR_IRQSEL                      ((VOR_IRQSEL_Type         *) VOR_IRQSEL_BASE)
#define VOR_IOCONFIG                    ((VOR_IOCONFIG_Type       *) VOR_IOCONFIG_BASE)
#define VOR_UTILITY                     ((VOR_UTILITY_Type        *) VOR_UTILITY_BASE)
#define VOR_PORTA                       ((VOR_GPIO_PERIPHERAL_Type*) VOR_PORTA_BASE)
#define VOR_PORTB                       ((VOR_GPIO_PERIPHERAL_Type*) VOR_PORTB_BASE)
#define VOR_TIM0                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM0_BASE)
#define VOR_TIM1                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM1_BASE)
#define VOR_TIM2                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM2_BASE)
#define VOR_TIM3                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM3_BASE)
#define VOR_TIM4                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM4_BASE)
#define VOR_TIM5                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM5_BASE)
#define VOR_TIM6                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM6_BASE)
#define VOR_TIM7                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM7_BASE)
#define VOR_TIM8                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM8_BASE)
#define VOR_TIM9                        ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM9_BASE)
#define VOR_TIM10                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM10_BASE)
#define VOR_TIM11                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM11_BASE)
#define VOR_TIM12                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM12_BASE)
#define VOR_TIM13                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM13_BASE)
#define VOR_TIM14                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM14_BASE)
#define VOR_TIM15                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM15_BASE)
#define VOR_TIM16                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM16_BASE)
#define VOR_TIM17                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM17_BASE)
#define VOR_TIM18                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM18_BASE)
#define VOR_TIM19                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM19_BASE)
#define VOR_TIM20                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM20_BASE)
#define VOR_TIM21                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM21_BASE)
#define VOR_TIM22                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM22_BASE)
#define VOR_TIM23                       ((VOR_TIM_PERIPHERAL_Type *) VOR_TIM23_BASE)
#define VOR_UARTA                       ((VOR_UART_PERIPHERAL_Type*) VOR_UARTA_BASE)
#define VOR_UARTB                       ((VOR_UART_PERIPHERAL_Type*) VOR_UARTB_BASE)
#define VOR_SPIA                        ((VOR_SPI_PERIPHERAL_Type *) VOR_SPIA_BASE)
#define VOR_SPIB                        ((VOR_SPI_PERIPHERAL_Type *) VOR_SPIB_BASE)
#define VOR_SPIC                        ((VOR_SPI_PERIPHERAL_Type *) VOR_SPIC_BASE)
#define VOR_I2CA                        ((VOR_I2C_PERIPHERAL_Type *) VOR_I2CA_BASE)
#define VOR_I2CB                        ((VOR_I2C_PERIPHERAL_Type *) VOR_I2CB_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group VA108XX */
/** @} */ /* End of group Vorago Technologies */

#ifdef __cplusplus
}
#endif


#endif  /* VA108XX_H */

