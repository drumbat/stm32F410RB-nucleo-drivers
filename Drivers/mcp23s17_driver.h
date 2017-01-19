/* Note MCP23S17 max clock speed is 10MHZ */
#ifndef MCP23S17_driver_h
#define MCP23S17_driver_h

#ifdef TEST
#include "testing/lib/stm32f410rx.h"
#else
#include "stm32f410rx.h"
#endif

/***********************************************************/
/*                                                         */
/*           1. MCP23S17 Bit Definitions                   */
/*                                                         */
/***********************************************************/



/***********************************************************/
/*                                                         */
/*           2. Data Structures used by                    */
/*              MCPS23S17                                  */
/*                                                         */
/***********************************************************/

/**
* @brief    mcp23s17 Configuration Structure definition
*/

typedef struct {

  __IO uint8_t IODIRA;          // I/O direction register
  __IO uint8_t IODIRB;          // 1 = Input (default), 0 = Output

  __IO uint8_t IPOLA;           // MCP23x17 Input Polarity Register
  __IO uint8_t IPOLB;           // 0 = Normal (default)(low reads as 0), 1 = Inverted (low reads as 1)

  __IO uint8_t GPINTENA;        // MCP23x17 Interrupt on Change Pin Assignments
  __IO uint8_t GPINTENB;        // 0 = No Interrupt on Change (default), 1 = Interrupt on Change

  __IO uint8_t DEFVALA;         // MCP23x17 Default Compare Register for Interrupt on Change
  __IO uint8_t DEFVALB;         // Opposite of what is here will trigger an interrupt (default = 0)

  __IO uint8_t INTCONA;         // MCP23x17 Interrupt on Change Control Register
  __IO uint8_t INTCONB;         // 1 = pin is compared to DEFVAL, 0 = pin is compared to previous state (default)

  __IO uint8_t IOCONA;          // MCP23x17 Configuration Register
  __IO uint8_t IOCONB;          //

  __IO uint8_t GPPUA;           // MCP23x17 Weak Pull-Up Resistor Register
  __IO uint8_t GPPUB;           // INPUT ONLY: 0 = No Internal 100k Pull-Up (default) 1 = Internal 100k Pull-Up

  __IO uint8_t INTFA;           // MCP23x17 Interrupt Flag Register
  __IO uint8_t INTFB;           // READ ONLY: 1 = This Pin Triggered the Interrupt

  __IO uint8_t INTCAPA;         // MCP23x17 Interrupt Captured Value for Port Register
  __IO uint8_t INTCAPB;         // READ ONLY: State of the Pin at the Time the Interrupt Occurred

  __IO uint8_t MCP_GPIO_A;      // MCP23x17 GPIO Port Register
  __IO uint8_t MCP_GPIO_B;      // Value on the Port - Writing Sets Bits in the Output Latch

  __IO uint8_t OLATA;           // MCP23x17 Output Latch Register
  __IO uint8_t OLATB;           // 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!

}MCP23S17_TypeDef;

// /**
// * @brief    HAL SPI State structure definition
// */
// typedef enum {
//     HAL_SPI_STATE_RESET         = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_READY         = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_BUSY          = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_BUSY_TX       = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_BUSY_RX       = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_BUSY_TX_RX    = 0x00, /* SPI not yet initialized or disabled */
//     HAL_SPI_STATE_ERROR         = 0x00, /* SPI not yet initialized or disabled */
// } hal_spi_state_t;

// /**
// * @brief    SPI Configuration Structure definition
// */

// typedef struct {
//     uint8_t Mode;              /* Specifies the SPI operating mode. */
//     uint8_t Direction;         /* Specifies the SPI Directional mode state. */
//     uint8_t DataSize;          /* Specifies the SPI data size. */
//     uint8_t CLKPolarity;       /* Specifies serial clock steady state. */
//     uint8_t CLKPhase;          /* Specifies the clock active edge for bit capture. */
//     uint8_t NSS;               /* Specifies whether the NSS signal is managed
//                                    by hardware (NSS pin) or by software using
//                                    the SSI bit. */
//     uint8_t BaudRatePrescaler; /* Specifies the Baud Rate prescaler value which
//                                    will be used to configure the transmit and receive
//                                    SCK clock. */
//     uint8_t FirstBit;          /* Specifies whether data transfers start from MSB or LSB */
// } mcp23s17_init_t;

/****************************************************************************/
/*                                                                          */
/*                        Driver exposed APIs                               */
/*                                                                          */
/****************************************************************************/

/*
Init
- Set slave select pin as output
- set slave select high
- start the SPI bus
- set the SPI bus speed
- set the SPI bit order
- set the SPI timing mode
- set IOCON

BYTE WRITE
- given the register and byte value
- set slave select low
- send the MCP23S17 opcode, chip address and write bit
- send the register we want to write
- send the byte
- take slave select high

WORD WRITE
- accept the start register and the word
- take slave select low
- send the MCP23S17 opcode, chip address and write bit
- send the register we want to write
- send the low byte (register address pointer will auto increment after write)
- shift the high byte down to the low byte location and send

SET MODE BY PIN
- accept the pin and I/O mode
- if the pin value is not valid, do nothing and return
- create a
-
-
-
-
-
-
-
-
*/
/**
* @brief    Configures the mcp23s17
* @param    *mcp23s17 : mcp23s17  Base struct
* @param    *mcp23S17_address : address (0-7) of the mcp23s17
*                               being configured.
*                               (as set by pins A0, A1 A2 on the mcp23s17)
* @retval  None
*/
void hal_mcp23S17_init(MCP23S17_TypeDef *mcp23s17,uint8_t mcp23S17_address);


#endif //MCP23S17

