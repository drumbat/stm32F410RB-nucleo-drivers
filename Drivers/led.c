#include "led.h"


/**
 * @brief  Initialize LED_NUCLEO_USER
 * @param  None
 * @retval None
 */

void led_init(void) {
    gpio_pin_conf_t led_pin_conf;
    /* enable the clock for the GPIOA port */
    _HAL_RCC_GPIOA_CLK_ENABLE();
    _HAL_RCC_GPIOB_CLK_ENABLE();

    led_pin_conf.pin = LED_NUCLEO_USER;
    led_pin_conf.mode = GPIO_PIN_OUTPUT_MODE;
    led_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    led_pin_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    led_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
    hal_gpio_init(GPIOA, &led_pin_conf);

    led_pin_conf.pin = LED_GREEN;
    hal_gpio_init(GPIO_EXT_LED_PORT, &led_pin_conf);

    led_pin_conf.pin = LED_YELLOW;
    hal_gpio_init(GPIO_EXT_LED_PORT, &led_pin_conf);

    led_pin_conf.pin = LED_RED;
    hal_gpio_init(GPIO_EXT_LED_PORT, &led_pin_conf);

}

/**
 * @brief  Initialize Built in Button
 * @param  None
 * @retval None
 */


void button_init(void) {

	  gpio_pin_conf_t button_pin_conf;
    /* enable the clock for the GPIOA port */
    _HAL_RCC_GPIOA_CLK_ENABLE();

    button_pin_conf.pin = USER_BUTTON;
    button_pin_conf.mode = GPIO_PIN_INPUT_MODE;
    button_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
    button_pin_conf.speed = GPIO_PIN_SPEED_MEDIUM;
    button_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
    hal_gpio_init(GPIOC, &button_pin_conf);

}

/**
 * @brief  Turn ON the led connected to the given pin
 * @param  *GPIOx : base address of the GPIO Port
 * @param  Pin : pin number of the LED
 * @retval None
 */

void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin) {
    hal_gpio_write_to_pin(GPIOx, pin, 1);
}

/**
 * @brief  Turn OFF the led connected to the given pin
 * @param  *GPIOx : base address of the GPIO Port
 * @param  Pin : pin number of the LED
 * @retval None
 */

void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin) {
    hal_gpio_write_to_pin(GPIOx, pin, 0);
}

/**
 * @brief  Toggle the led connected to the given pin
 * @param  *GPIOx : base address of the GPIO Port
 * @param  Pin : pin number of the LED
 * @retval None
 */

void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin) {
    if(hal_gpio_read_from_pin(GPIOx, pin)) {
        hal_gpio_write_to_pin(GPIOx, pin, 0);

    } else {
        hal_gpio_write_to_pin(GPIOx, pin, 1);

    }
}


int main(void) {
    //uint32_t i;

    /* Initializes the LED*/
    led_init();
	  // enable clock for GPIO Port C
    _HAL_RCC_GPIOC_CLK_ENABLE();

			//set the mode as input
	button_init();
	/*
	GPIOC->MODER &= ~0x3;
	GPIOC->PUPDR  &= ~0x3;
		*/
	//enable clock for RCC
	RCC->APB2ENR |= 0x00004000;

	SYSCFG->EXTICR[3] |= (0x02 << 4);//refer (SYSCFG_EXTICR4) page number 139 in your reference manual


    /* Set the port and pin number for the interrupt */
    hal_gpio_set_pin_as_interrupt(GPIO_BUTTON_PIN, GPIO_EXTIx_SET_PORT_C);
    /* Configure the button interrupt as falling edge */
    hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);

    /* enable the interrupt on EXTIO line */
    hal_gpio_enable_interrupt(GPIO_BUTTON_PIN, EXTI15_10_IRQn);
    led_turn_on(GPIO_LED_PORT, LED_NUCLEO_USER);
    led_turn_on(GPIO_EXT_LED_PORT, LED_GREEN);
    led_turn_on(GPIO_EXT_LED_PORT, LED_YELLOW);
    led_turn_on(GPIO_EXT_LED_PORT, LED_RED);



#if 0

    while(1) {

        led_toggle(GPIO_LED_PORT, LED_NUCLEO_USER);
        for(i=0;i<500000;i++);

    }

#endif
		while(1);
}

/**
 * @brief ISR for the configured EXTI0 interrupt
 * @retval None
 */



void EXTI15_10_IRQHandler (void) {
    hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);

    led_toggle(GPIO_LED_PORT, LED_NUCLEO_USER);
}


