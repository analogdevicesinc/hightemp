;/**************************************************************************//**
; * @file     startup_<Device>.s
; * @brief    CMSIS Cortex-M# Core Device Startup File for
; *           Device <Device>
; * @version  V3.01
; * @date     06. March 2012
; *
; * @note
; * Copyright (C) 2012 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000C00

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     OC0_IRQHandler   	      ;  0: TIM 0 ISR
                DCD     VOR_TIM1_IRQHandler 	  ;  1: TIM 1 ISR
                DCD     OC2_IRQHandler            ;  2: General
                DCD     VOR_UART0_IRQHandler      ;  3: UART A ISR
                DCD     VOR_UART1_IRQHandler      ;  4: UART B ISR
                DCD     VOR_GPIO_IRQHandler   	  ;  5: GPIO ISR
                DCD     VOR_SPI0_IRQHandler       ;  6: SPI A ISR
                DCD     VOR_SPI1_IRQHandler       ;  7: SPI B ISR
                DCD     VOR_SPI2_IRQHandler       ;  8: SPI C ISR
                DCD     VOR_I2C0_MS_IRQHandler    ;  9: I2C A Master ISR
                DCD     VOR_I2C0_SL_IRQHandler    ; 10: I2C A Slave ISR
                DCD     VOR_I2C1_MS_IRQHandler    ; 11: I2C B Master ISR
                DCD     VOR_I2C1_SL_IRQHandler    ; 12: I2C B Slave ISR
                DCD     OC13_IRQHandler           ; 13: General
                DCD     OC14_IRQHandler           ; 14: General
                DCD     OC15_IRQHandler           ; 15: General
                DCD     OC16_IRQHandler           ; 16: Reserved
                DCD     OC17_IRQHandler           ; 17: Reserved
                DCD     OC18_IRQHandler           ; 18: Reserved
                DCD     OC19_IRQHandler           ; 19: Reserved
                DCD     OC20_IRQHandler           ; 20: Reserved
                DCD     OC21_IRQHandler           ; 21: Reserved
                DCD     OC22_IRQHandler           ; 22: Reserved
                DCD     OC23_IRQHandler           ; 23: Reserved
                DCD     OC24_IRQHandler           ; 24: Reserved
                DCD     OC25_IRQHandler           ; 25: Reserved
                DCD     OC26_IRQHandler           ; 26: Reserved
                DCD     OC27_IRQHandler           ; 27: Reserved
                DCD     OC28_IRQHandler           ; 28: Reserved
                DCD     OC29_IRQHandler           ; 29: Reserved
                DCD     OC30_IRQHandler           ; 30: Reserved
                DCD     OC31_IRQHandler           ; 31: Reserved
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  OC0_IRQHandler	      	  [WEAK]
                EXPORT  VOR_TIM1_IRQHandler	  	  [WEAK]
                EXPORT  OC2_IRQHandler            [WEAK]
                EXPORT  VOR_UART0_IRQHandler      [WEAK]
                EXPORT  VOR_UART1_IRQHandler      [WEAK]
                EXPORT  VOR_GPIO_IRQHandler		  [WEAK]
                EXPORT  VOR_SPI0_IRQHandler       [WEAK]
                EXPORT  VOR_SPI1_IRQHandler       [WEAK]
                EXPORT  VOR_SPI2_IRQHandler       [WEAK]
                EXPORT  VOR_I2C0_MS_IRQHandler    [WEAK]
                EXPORT  VOR_I2C0_SL_IRQHandler    [WEAK]
                EXPORT  VOR_I2C1_MS_IRQHandler    [WEAK]
                EXPORT  VOR_I2C1_SL_IRQHandler    [WEAK]
                EXPORT  OC13_IRQHandler           [WEAK]
                EXPORT  OC14_IRQHandler           [WEAK]
                EXPORT  OC15_IRQHandler           [WEAK]
                EXPORT  OC16_IRQHandler           [WEAK]
                EXPORT  OC17_IRQHandler           [WEAK]
                EXPORT  OC18_IRQHandler           [WEAK]
                EXPORT  OC19_IRQHandler           [WEAK]
                EXPORT  OC20_IRQHandler           [WEAK]
                EXPORT  OC21_IRQHandler           [WEAK]
                EXPORT  OC22_IRQHandler           [WEAK]
                EXPORT  OC23_IRQHandler           [WEAK]
                EXPORT  OC24_IRQHandler           [WEAK]
                EXPORT  OC25_IRQHandler           [WEAK]
                EXPORT  OC26_IRQHandler           [WEAK]
                EXPORT  OC27_IRQHandler           [WEAK]
                EXPORT  OC28_IRQHandler           [WEAK]
                EXPORT  OC29_IRQHandler           [WEAK]
                EXPORT  OC30_IRQHandler           [WEAK]
                EXPORT  OC31_IRQHandler           [WEAK]

OC0_IRQHandler
VOR_TIM1_IRQHandler
OC2_IRQHandler
VOR_UART0_IRQHandler
VOR_UART1_IRQHandler
VOR_GPIO_IRQHandler
VOR_SPI0_IRQHandler
VOR_SPI1_IRQHandler
VOR_SPI2_IRQHandler
VOR_I2C0_MS_IRQHandler
VOR_I2C0_SL_IRQHandler
VOR_I2C1_MS_IRQHandler
VOR_I2C1_SL_IRQHandler
OC13_IRQHandler
OC14_IRQHandler
OC15_IRQHandler
OC16_IRQHandler
OC17_IRQHandler
OC18_IRQHandler
OC19_IRQHandler
OC20_IRQHandler
OC21_IRQHandler
OC22_IRQHandler
OC23_IRQHandler
OC24_IRQHandler
OC25_IRQHandler
OC26_IRQHandler
OC27_IRQHandler
OC28_IRQHandler
OC29_IRQHandler
OC30_IRQHandler
OC31_IRQHandler
                B       .

                ENDP

                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
