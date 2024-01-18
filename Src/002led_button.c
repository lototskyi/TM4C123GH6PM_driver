#include "tm4c123gh6pm.h"

#define LOW                                        0
#define BTN_PRESSED                                 LOW

void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000/2; i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed, gpioBtn;

    gpioLed.pGPIOx = GPIOF_AHB;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_2;
    gpioLed.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType           = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_NO_PUPD;

    GPIO_PeriClockControl(gpioLed.pGPIOx, ENABLE);

    GPIO_Init(&gpioLed);

    gpioBtn.pGPIOx = GPIOF_AHB;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_4;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_IN;
    gpioBtn.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_PIN_PU;

    GPIO_PeriClockControl(gpioBtn.pGPIOx, ENABLE);

    GPIO_Init(&gpioBtn);

    //GPIO_WriteToOutputPin(GPIOF_APB, GPIO_PIN_NO_3, SET);

    while(1) {

        if (GPIO_ReadFromInputPin(gpioBtn.pGPIOx, gpioBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED) {
            delay();
            GPIO_ToggleOutputPin(gpioLed.pGPIOx, gpioLed.GPIO_PinConfig.GPIO_PinNumber);
        }
    }

    return 0;
}
