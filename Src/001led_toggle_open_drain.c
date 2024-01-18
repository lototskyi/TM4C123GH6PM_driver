#include "tm4c123gh6pm.h"

void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000; i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed;

    gpioLed.pGPIOx = GPIOF_AHB;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_3;
    gpioLed.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType           = GPIO_OP_TYPE_OD;
    gpioLed.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_PIN_PU;

    GPIO_PeriClockControl(gpioLed.pGPIOx, ENABLE);

    GPIO_Init(&gpioLed);

    //GPIO_WriteToOutputPin(GPIOF_APB, GPIO_PIN_NO_3, SET);

    while(1) {
        GPIO_ToggleOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
        delay();
    }

    return 0;
}

