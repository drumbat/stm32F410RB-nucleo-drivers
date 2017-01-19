#ifndef __LED_H
#define __LED_H

#include "src/hal_gpio_driver.h"
#include "hal_spi_driver.h"
#include "mcp23s17_driver.h"

#define GPIO_LED_PIN            5
#define GPIO_LED_GREEN_PIN      3
#define GPIO_LED_YELLOW_PIN     4
#define GPIO_LED_RED_PIN        5

#define GPIO_LED_PORT           GPIOA
#define GPIO_EXT_LED_PORT       GPIOB

#define LED_NUCLEO_USER         GPIO_LED_PIN
#define LED_GREEN               GPIO_LED_GREEN_PIN
#define LED_YELLOW              GPIO_LED_YELLOW_PIN
#define LED_RED                 GPIO_LED_RED_PIN

#define GPIO_BUTTON_PIN     13
#define GPIO_BUTTON_PORT    GPIOC

#define USER_BUTTON              GPIO_BUTTON_PIN


/**
* @brief  Initialize the LEDs
* @param  None
* @retval None
*/
void led_init(void);


/**
* @brief  Turns ON the led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Turns OFF the led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Toggles the led which is connected on the given pin
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif
