/***************************************************************************************
 * @file     main.c
 * @version  V1.0
 * @date     18 January 2018
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
 /** @file */ 
/** \addtogroup MAIN 
 *  @{
 */
 
 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "driver_common.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ht_dab.h"
#include "ht_dab_user.h"

#include "uart.h"
#include "parser.h"
#include "sampling_engine.h"

//includes for RTOS
#ifdef configENABLE_RTOS
#include "FreeRTOS.h"
#endif

/*******************************************************************************
 **
 ** @brief Configures SPI peripherals.
 **
 ** Configures SPI0 and SPI1 with the following settings:
 **  Clock @ 25MHz
 **  Master mode.
 **  Stall on empty TX FIFO.
 **  Word size = 16
 **  SCLK idles low.
 **  Data clocked on falling edge.
 **  Enable and clear TX & RX FIFOs.
 **
 ******************************************************************************/
void
SPIInit(void)
{	
    /* Configure SPI0. */
    VOR_SPI0.CLKPRESCALE = (1UL << 1);
    VOR_SPI0.CTRL0 = (0 << SPI_PERIPHERAL_CTRL0_SCRDV_Pos) |
                     (1UL << SPI_PERIPHERAL_CTRL0_SPH_Pos) |
                     (0 << SPI_PERIPHERAL_CTRL0_SPO_Pos) |
                     (0x0F << SPI_PERIPHERAL_CTRL0_SIZE_Pos); 
    VOR_SPI0.CTRL1 = (SPI_PERIPHERAL_CTRL1_ENABLE_Msk |
                      SPI_PERIPHERAL_CTRL1_BMSTALL_Msk);
    VOR_SPI0.FIFO_CLR = (SPIC_FIFO_CLR_RXFIFO_Msk | SPIC_FIFO_CLR_TXFIFO_Msk);

    /* Configure SPI1. */
    VOR_SPI1.CLKPRESCALE = (1UL << 1);
    VOR_SPI1.CTRL0 = (0 << SPI_PERIPHERAL_CTRL0_SCRDV_Pos) |
                     (1UL << SPI_PERIPHERAL_CTRL0_SPH_Pos) |
                     (0 << SPI_PERIPHERAL_CTRL0_SPO_Pos) |
                     (0x0F << SPI_PERIPHERAL_CTRL0_SIZE_Pos); 
    VOR_SPI1.CTRL1 = (SPI_PERIPHERAL_CTRL1_ENABLE_Msk |
                      SPI_PERIPHERAL_CTRL1_BMSTALL_Msk);
    VOR_SPI1.FIFO_CLR = (SPIC_FIFO_CLR_RXFIFO_Msk | SPIC_FIFO_CLR_TXFIFO_Msk);
}

/*******************************************************************************
 **
 ** @brief Configures Timer peripherals.
 **
 ** Timers 0, 2, 18, and 20 are used to control the ADC sampling engine.  Timers
 ** 2, 18, and 20 generated inverted PWM waveforms and map directly to pins B2,
 ** B18, A28, respectively, to drive ADC convert signals.  Timers 2 and 18 are
 ** started by Timer 0 by causing a simultaneous cascade event when Timer 0 is
 ** enabled, whereas Timer 20 is manually started.
 **
 ** Timer 27 is used for the system's timekeeping needs, interrupting at 1Hz.
 **
 ** Timer 5 controls a software watchdog, and will reset the MCU if it's
 ** interrupt is ever asserted. 
 **
 ** Timer mappings:
 **  0: Triggers the start of all sampling sequences.
 **  2: Generates CONVERT waveform for ADC0 connected to SPI1
 **  5: Watchdog
 **  18: Generates COVNERT waveform for ADC2 connected to SPI1
 **  20: Generates CONVERT waveform for ADC1 connected to SPI0
 **  27: Seconds tick.
 **
 **
 ******************************************************************************/
void
TimersInit(void)
{   
    /* Enable all Timers, except for Watchdog. */
    VOR_SYSCONFIG->TIM_CLK_ENABLE |= ((1UL << VOR_TIM_SEC_INDEX) | 
                                      (1UL << VOR_TIM_ADC0_INDEX) |
                                      (1UL << VOR_TIM_ADC1_INDEX) |
                                      (1UL << VOR_TIM_ADC2_INDEX) |
                                      (1UL << VOR_TIM_TRIG_INDEX));
    
    /* Immediately disable all Timers. */
    VOR_TIM_SEC.ENABLE = 0;
    VOR_TIM_ADC0_CONV.ENABLE = 0;
    VOR_TIM_ADC1_CONV.ENABLE = 0;
    VOR_TIM_ADC2_CONV.ENABLE = 0;
    VOR_TIM_TRIG.ENABLE = 0;
    
    /* VOR_TIM_TRIG is configured at runtime depending on the command. No
       configuration to be done here. */
    
    /* Configuire generation of ADC0 CONV signal, */
    VOR_TIM_ADC0_CONV.CTRL = (TIM_PERIPHERAL_CTRL_STATUS_INV_Msk |
                              TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk |
                              (STAT_SEL_PWMA <<
                               TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos));

    /* ...automatically starting when VOR_TIM_TRIG has completed. */
    VOR_TIM_ADC0_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);

    /* Configuire generation of ADC1 CONV signal, */
    VOR_TIM_ADC1_CONV.CTRL = (TIM_PERIPHERAL_CTRL_STATUS_INV_Msk |
                              TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk |
                              (STAT_SEL_PWMA <<
                               TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos));
                              
    /* ...automatically starting when VOR_TIM_TRIG has completed. */
    VOR_TIM_ADC1_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
    
    /* Configuire generation of ADC2 CONV signal, */
    VOR_TIM_ADC2_CONV.CTRL = (TIM_PERIPHERAL_CTRL_STATUS_INV_Msk |
                              TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk |
                              (STAT_SEL_PWMA <<
                               TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos));
    
    /* ...automatically starting when VOR_TIM_TRIG has completed. */
    VOR_TIM_ADC2_CONV.CSD_CTRL = (TIM_PERIPHERAL_CSD_CTRL_CSDEN0_Msk |
                                  TIM_PERIPHERAL_CSD_CTRL_CSDTRG0_Msk);
     
    /* Enable interrupt for falling edge of ADC0 CONV pin.  Controls
       simultaneous sampling of ADC0 and ADC1. */
    VOR_IRQSEL->TIM[VOR_TIM_ADC0_INDEX] = VOR_IRQ_CNV_ADC01;
		VOR_IRQSEL->TIM[VOR_TIM_ADC1_INDEX] = VOR_IRQ_CNV_ADC01;  
    NVIC_SetPriority(VOR_IRQ_CNV_ADC01, 0);
    NVIC_EnableIRQ(VOR_IRQ_CNV_ADC01);	
    
    /* Enable interrupt for falling edge of ADC2 CONV pin.  Controls
       state machine for sampling ADC Mux. */
    VOR_IRQSEL->TIM[VOR_TIM_ADC2_INDEX] = VOR_IRQ_CNV_ANAMUX;  
    NVIC_SetPriority(VOR_IRQ_CNV_ANAMUX, 0);
    NVIC_EnableIRQ(VOR_IRQ_CNV_ANAMUX);	
    
    /* Configure Timer to generate interupt every second, for internal
       timekeeping. */
    VOR_TIM_SEC.CTRL = (TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk |
                        (STAT_SEL_PWMA << TIM_PERIPHERAL_CTRL_STATUS_SEL_Pos));
    VOR_TIM_SEC.RST_VALUE = configSYSCLK;  
    VOR_TIM_SEC.CNT_VALUE = 0;
    
    VOR_IRQSEL->TIM[VOR_TIM_SEC_INDEX] = VOR_IRQ_SEC_INDEX;
    NVIC_SetPriority(VOR_IRQ_SEC_INDEX, 1);
    NVIC_EnableIRQ(VOR_IRQ_SEC_INDEX);
    
    VOR_TIM_SEC.ENABLE = 1;
    
    /* Configure Watchdog Timer if enabled in application. */
#ifdef USE_WDT
    VOR_SYSCONFIG->TIM_CLK_ENABLE |= (1UL << VOR_TIM_WDT_INDEX);
    VOR_TIM_WDT.ENABLE = 0;
    
    /* Configure the Timer to interupt if not "fed". */
    VOR_TIM_WDT.CTRL = TIM_PERIPHERAL_CTRL_IRQ_ENB_Msk;
    VOR_TIM_WDT.RST_VALUE = WDT_PERIOD_CLKS;
    VOR_TIM_WDT.CNT_VALUE = WDT_PERIOD_CLKS;
    
    /* Map the Timer interrupt to the watchdog interrupt in the NVIC. */
    VOR_IRQSEL.TIM[VOR_TIM_WDT_INDEX] = IRQ_DST_WATCHDOG;
    NVIC_EnableIRQ(IRQ_DST_WATCHDOG);
    
    /* Enable watchdog timer. */
    VOR_TIM_WDT.ENABLE = 1;
#endif
}

/*******************************************************************************
 **
 ** @brief Configures MCU pins.
 **
 ** Peripheral Pin Assignments with required function select setting:
 ** 
 ** UART0 (Serial Comms):
 **  Function    Port Pin   Func Select#
 **  TX             A17      F3
 **  RX             A16      F3
 **  TX             A8       F2
 **  RX             A9       F2
 **  CTS            A6       F2
 ** 
 ** SPI0 (ADC1 - Single Channel):
 **  Function    Port Pin   Func Select#
 **  SCK            A31      F1
 **  MISO           A29      F1
 **  CNV(TIM20)     A28      F2
 **
 ** SPI1 (ADC0 - Single Channel):
 **  Function    Port Pin   Func Select#
 **  SCK            B5       F1
 **  MISO           B3       F1
 **  CNV(TIM2)      B2       F3
 **
 ** SPI1 (ADC2 - Analog Mux):
 **  Function    Port Pin   Func Select#
 **  SCK            B19      F1
 **  MISO           B17      F1
 **  CNV(TIM18)     B18      F3
 **
 ** GPIO:
 **  Mux Select     B20, B21, B22
 **  Mux Enable     B23
 **  Ext Trig       B14
 **  TrigModeTest   B15
 **
 ** 
 **
 ******************************************************************************/
void
ConfigurePins(void)
{
    uint32_t ui32PortAPins = ((1UL << PA17_UARTA_TX) | (1UL << PA16_UARTA_RX)) |
                             ((1UL << PA8) | (1UL << PA9)) |
                             ((1UL << PA31_SCKA) | (1UL << PA29_MISOA) | (1UL << PA28_TIM20));  // create mask with used pins 
    uint32_t ui32PortBPins = ((1UL << PB5_SCKB) | (1UL << PB3_MISOB) | (1UL << PB2_TIM2)) |
                             ((1UL << PB19_SCKB) | (1UL << PB17_MISOB) | (1UL << PB18_TIM18)) |
                             ((7UL << PB20) | (1UL << PB23)) | 
                             (1UL << PB14_EXTRIG) |
                             (1UL << PB15) |
                             (1UL << PB9);  // create mask with used pins 
    uint32_t ui8Tmp;
    
    /* Enable weak pulldowns on all unused IO. */
    for(ui8Tmp = 0; ui8Tmp < 32; ui8Tmp++)
    {
        if(!(ui32PortAPins & (1UL << ui8Tmp)))
        { 
            VOR_IOCONFIG->PORTA[ui8Tmp] |= (IOCONFIG_PORTA_PEN_Msk);
        }
        
        if(!(ui32PortBPins & (1UL << ui8Tmp)))
        { 
            VOR_IOCONFIG->PORTB[ui8Tmp] |= (IOCONFIG_PORTB_PEN_Msk);
        }
    }
    
    /* Enable DataMask for pins. */
    VOR_PORTA.DATAMASK = ui32PortAPins;
    VOR_PORTB.DATAMASK = ui32PortBPins;

    /* Configure debug pins as outputs. */
    VOR_PORTA.DIR |= (1UL << 3);  /* PORTA[3] is UARTB_RX on breakout board.   */
    VOR_PORTB.DIR |= (1UL << 15);  /* PORTB[15] is on Analog breakout board.  Used for loopback on trigger PB[14]   */
    

    /* Configure mux select pins as outputs, with mux enable always on. */
    VOR_PORTB.DIR |= ((7UL << ADC2_MUX0_PIN ) | (1UL << ADC2_MUX_EN_PIN)); /* set Mux control signals to outputs */ 
    VOR_PORTB.CLROUT = (7UL << ADC2_MUX0_PIN ); /* drive all channels select pins low  */ 
    VOR_PORTB.SETOUT = (1UL << ADC2_MUX_EN_PIN ); /* enable Mux  */ 

    /* Switch pins of SPI0. */
    VOR_PORTA.DIR |= (1UL << 31);
    VOR_PORTA.CLROUT = (1UL << 31);
    VOR_GPIO_PinMux(GPORTA, 29, FUNSEL1);
    VOR_GPIO_PinMux(GPORTA, 31, FUNSEL1);
    
    /* SPI1 pins must be configured before switching their function, since the
     * application will switch between two set of pins. */
         
    /* Configure SPI1 SCLK pins to idle low, when not in use. */
    VOR_PORTB.DIR |=  ((1UL << 5) | (1UL << 19));
    VOR_PORTB.CLROUT =  ((1UL << 5) | (1UL << 19));
	
    /* Configure SPI1 SDO pins as inputs when not in use. */
    VOR_PORTB.DIR &= ~((1UL << 3) | (1UL << 17));
    
    /* The application will control which pins are mapped to SPI1. */
    
    /* Enable weak pullups on all MISO pins, without an input filter. */
    VOR_IOCONFIG->PORTA[29] |= (IOCONFIG_PORTA_PLEVEL_Msk |
                                IOCONFIG_PORTA_PEN_Msk |
                                (1UL << IOCONFIG_PORTA_FLTTYPE_Pos));
    VOR_IOCONFIG->PORTB[03] |= (IOCONFIG_PORTB_PLEVEL_Msk |
                                IOCONFIG_PORTB_PEN_Msk |
                                (1UL << IOCONFIG_PORTA_FLTTYPE_Pos));
    VOR_IOCONFIG->PORTB[17] |= (IOCONFIG_PORTB_PLEVEL_Msk |
                                IOCONFIG_PORTB_PEN_Msk |
                                (1UL << IOCONFIG_PORTA_FLTTYPE_Pos));
                                
    /* Configure Timer-controlled ADC Convert pins. */
    VOR_PORTA.DIR |= (1UL << PA28_TIM20);
    VOR_PORTA.CLROUT = (1UL << PA28_TIM20);
    VOR_PORTB.DIR |= ((1UL << PB2_TIM2) | (1UL << PB18_TIM18));
    VOR_PORTB.CLROUT = ((1UL << PB2_TIM2) | (1UL << PB18_TIM18));
    
    VOR_GPIO_PinMux(GPORTA, PA28_TIM20, FUNSEL2);
    VOR_GPIO_PinMux(GPORTB, PB2_TIM2, FUNSEL3);
    VOR_GPIO_PinMux(GPORTB, PB18_TIM18, FUNSEL3);
    
    /* Configure UART pins. */ 

    VOR_GPIO_PinMux(GPORTA, PA6_UARTA_CTS, FUNSEL2);
    VOR_GPIO_PinMux(GPORTA, PA16_UARTA_RX, FUNSEL3);
    VOR_GPIO_PinMux(GPORTA, PA17_UARTA_TX, FUNSEL3);
    
    VOR_PORTA.DIR &= ~(1UL << PA16_UARTA_RX);	
    VOR_PORTA.DIR |= (1UL << PA17_UARTA_TX);
    
    /* Configure External Trigger pin. */
    VOR_PORTB.DIR &= ~(1UL << PB14_EXTRIG);
    
}

/*******************************************************************************
 **
 ** @brief 
 **
 **
 ******************************************************************************/
void
FeedWatchdog(void)
{
#ifdef USE_WDT
    VOR_TIM_WDT.CNT_VALUE = WDT_PERIOD_CLKS;
#endif
}

/*******************************************************************************
 **
 ** @brief Parses commands and prints resulting data.
 **
 ** Parses commands sent from the user, and prints ADC sample data as it becomes
 ** available.
 ** 
 ** @param pvParameters is a void pointer to a set of variables that are passed
 **                     to the task.
 **
 ******************************************************************************/
void
TaskMain(void *pvParameters)
{
    /* Print application version information. */
    PrintVersion();
    
    while(1)
    {
        /* Parse commands. */
        ParseCommand();
        
        /* Print data as soon as it's ready. */
        PrintData();
                
        /* Feed watchdog. */
        FeedWatchdog();
    }
}


/*******************************************************************************
 **
 ** @brief Read EF_config, compare to known good value, reset MCU if not matching 
 **
 ** 
 ******************************************************************************/
void check_EFCONFIG(void)
{
	if(VOR_SYSCONFIG->EF_CONFIG != VOR_EF_CONFIG )
		
	{
		VOR_SYSCONFIG->RST_STAT  = 0x0000   ;  
		NVIC_SystemReset(); /*  reset processor for clean read of eFuse information   */
		
	}
	
}
/*******************************************************************************
 **
 ** @brief Example FreeRTOS task.
 **
 ** Toggles PA3 as fast as possible to show when user task is executing.  Debug purposes only.
 ** 
 ** @param pvParameters is a void pointer to a set of variables that are passed
 **                     to the task.
 **
 **
 ******************************************************************************/
#define Brkout_UARTB_Rx 3  
void
TaskDemo(void *pvParameters)
{
    while(1)
    {
        VOR_PORTA.TOGOUT = (1UL << Brkout_UARTB_Rx); 	/*  this pin is available on breakout board */	
    }
}

/*******************************************************************************
 **
 ** @brief Application entry point.
 **
 ******************************************************************************/
int
main(void)
{	
	  
#ifdef USE_RTOS
    TaskHandle_t xTaskMainHandle, xTaskDemoHandle;
#endif
    
    /* Enable peripheral clocks */
    VOR_SYSCONFIG->PERIPHERAL_CLK_ENABLE =  
	  (CLK_ENABLE_PORTA | CLK_ENABLE_PORTB | CLK_ENABLE_GPIO |
	   CLK_ENABLE_IOCONFIG | CLK_ENABLE_SPIA | CLK_ENABLE_SPIB |
	   CLK_ENABLE_UARTA | CLK_ENABLE_UARTB | 
	   CLK_ENABLE_IRQSEL |
	   CLK_ENABLE_IOMGR | CLK_ENABLE_UTILITY | CLK_ENABLE_PORTIO |
	   CLK_ENABLE_SYSTEM ); 
    
    /* Configure IO. */
    ConfigurePins();
        
    /* Setup peripherals. */
    TimersInit();
    UART0Init();
    SPIInit();
    
    /* Setup application. */
    ParserInit();
    SamplingEngineInit();
    
#ifdef USE_RTOS
    /* Setup RTOS Threads */
    xTaskMainHandle = NULL;
    xTaskDemoHandle = NULL;
    
    xTaskCreate(TaskMain, "MainLoop", 100, NULL, configMAX_PRIORITIES - 1,
                xTaskMainHandle);
  	xTaskCreate(TaskDemo, "DemoThread", configMINIMAL_STACK_SIZE, NULL,
                configMAX_PRIORITIES - 1, xTaskDemoHandle);

    vTaskStartScheduler();
#else
    /* Manually run the Main Task. */
    TaskMain(NULL);
#endif
}


/*******************************************************************************
 **
 ** @brief 
 **
 ******************************************************************************/
void
vApplicationTickHook(void)
{
}

/*******************************************************************************
 **
 ** @brief 
 **
 ******************************************************************************/
void
vApplicationMallocFailedHook(void)
{
}

/*******************************************************************************
 **
 ** @brief 
 **
 ******************************************************************************/
void
vApplicationStackOverflowHook(void)
{
}

