#ifndef __SPI_APP_C
#define __SPI_APP_C

#include<stdint.h>

/* Defines used for transfer communication */

#define CMD_MASTER_READ           ((uint16_t) 0x1234)
#define CMD_MASTER_WRITE          ((uint16_t) 0x5678)
#define CMD_LENGTH                2
#define DATA_LENGTH               4
#define ACK_LEN                   2
#define SPI_ACK_BYTES             0xD5E5

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                   SPI1_IRQn
#define SPIx_IRQHandler             SPI1_IRQHandler

//!!! check this is the appropriate IRQ line for the on board button
#define EXTIx_IRQn                   EXTI0_IRQn
#define EXTIx_IRQHandler             EXTI0_IRQHandler

/* Macros used for Configuring GPIO pins for SPI functionality */
#define GPIO_PA5    5
#define GPIO_PA6    6
#define GPIO_PA7    7
#define GPIO_PB6    6

#define SPI_CLK_PIN     GPIO_PA5
#define SPI_MISO_PIN    GPIO_PA6
#define SPI_MOSI_PIN    GPIO_PA7
#define SPI_SS_PIN      GPIO_PB6

/* SPI alternate functionality value */
#define GPIO_PIN_AF5_SPI1       0x05

#endif
