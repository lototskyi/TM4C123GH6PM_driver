#include <string.h>
#include "tm4c123gh6pm.h"

/*
 * PA6 -> SCL
 * PA7 -> SDA
 */

I2C_Handle_t I2C1Handle;

#define MY_ADDR             0x61
#define SLAVE_ADDR          0x68

uint8_t data[] = "We are testing I2C master Tx\n";

void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000; i++);
}

void I2C3_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOD_AHB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 3;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPinPuPdControl = GPIO_PIN_PU;



    //SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&I2CPins);

    //SCL
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Init(&I2CPins);
}

void I2C3_Inits(void)
{
    I2C1Handle.pI2Cx = I2C3;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t gpioBtn;

    gpioBtn.pGPIOx = GPIOF_AHB;
    gpioBtn.GPIO_PinConfig.GPIO_PinNumber           = GPIO_PIN_NO_4;
    gpioBtn.GPIO_PinConfig.GPIO_PinMode             = GPIO_MODE_IN;
    gpioBtn.GPIO_PinConfig.GPIO_PinPinPuPdControl   = GPIO_PIN_PU;

    GPIO_Init(&gpioBtn);
}

int main(void)
{

    I2C3_GPIOInits();
    GPIO_ButtonInit();
    I2C3_Inits();
    I2C_PeripheralMasterControl(I2C1Handle.pI2Cx, ENABLE);

    while(1) {
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDR);
    }

    return 0;
}
