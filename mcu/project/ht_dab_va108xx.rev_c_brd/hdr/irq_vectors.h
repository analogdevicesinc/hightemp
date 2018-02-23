#ifndef __IRQ_VECTORS
#define __IRQ_VECTORS

#include "va108xx.h"

/* Configured in startup_va108xx.s */
#define VOR_TIM0_IRQ_INDEX		((IRQn_Type)0)
#define VOR_TIM1_IRQ_INDEX		((IRQn_Type)1)
#define VOR_UART0_IRQ_INDEX		((IRQn_Type)3)
#define VOR_UART1_IRQ_INDEX		((IRQn_Type)4)
#define VOR_GPIO_IRQ_INDEX		((IRQn_Type)5)
#define VOR_SPI0_IRQ_INDEX		((IRQn_Type)6)
#define VOR_SPI1_IRQ_INDEX		((IRQn_Type)7)
#define VOR_SPI2_IRQ_INDEX		((IRQn_Type)8)
#define VOR_I2C0_M_IRQ_INDEX	((IRQn_Type)9)
#define VOR_I2C0_S_IRQ_INDEX	((IRQn_Type)10)
#define VOR_I2C1_M_IRQ_INDEX	((IRQn_Type)11)
#define VOR_I2C1_S_IRQ_INDEX	((IRQn_Type)12)

/* Configured for application. */
#define VOR_IRQ_SEC_INDEX 		((IRQn_Type)27)
#define VOR_IRQ_CNV_ADC01		((IRQn_Type)30)
#define VOR_IRQ_CNV_ANAMUX		((IRQn_Type)31)

#endif // __IRQ_VECTORS
