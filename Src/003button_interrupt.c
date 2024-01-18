#include "tm4c123gh6pm.h"
#include <string.h>

#define LOW                                        0
#define BTN_PRESSED                                 LOW

//this will introduce ~200ms delay when system clock is 16MHz
void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000/2; i++);
}

void main(void)
{
    GPIO_Handle_t gpioLed, gpioBtn;
    memset(&gpioLed, 0, sizeof(gpioLed));
    memset(&gpioBtn, 0, sizeof(gpioBtn));

    gpioLed.pGPIOx = GPIOF_AHB;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_2;
    gpioLed.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType           = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_NO_PUPD;

    GPIO_PeriClockControl(gpioLed.pGPIOx, ENABLE);

    GPIO_Init(&gpioLed);

    gpioBtn.pGPIOx = GPIOF_AHB;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_4;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_IT_FT;
    gpioBtn.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_PIN_PU;

    GPIO_PeriClockControl(gpioBtn.pGPIOx, ENABLE);

    GPIO_Init(&gpioBtn);

    //IRQ configurations
    GPIO_IRQPriorityConfig(IRQ_NO_GPIOF, NVIC_IRQ_PRI7);
    GPIO_IRQInterruptConfig(IRQ_NO_GPIOF, ENABLE);

    while(1);
}

void GPIOF_Handler(void)
{
    delay();

    GPIO_IRQHandling(GPIOF_AHB, GPIO_PIN_NO_4);

    GPIO_ToggleOutputPin(GPIOF_AHB, GPIO_PIN_NO_2);
}
