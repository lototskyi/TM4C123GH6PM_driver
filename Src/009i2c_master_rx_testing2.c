#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"

/*
 * PA6 -> SCL
 * PA7 -> SDA
 */

I2C_Handle_t I2C1Handle;

#define MY_ADDR             0x61
#define SLAVE_ADDR          0x68

const uint8_t data[32];

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
    I2C1Handle.I2C_Config.I2C_MasterEnable = I2C_MASTER_ENABLE;
    I2C1Handle.I2C_Config.I2C_SlaveEnable = I2C_SLAVE_DISABLE;
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
    //main clock is 16MHz crystal oscillator
    *SCR_RCC &= ~(0x1f << 6);
    *SCR_RCC &= ~(1 << 0); //clear MOSCDIS
    *SCR_RCC &= ~(1 << 4); //clear OSCSRC
    *SCR_RCC |= (0x17 << 6); //set XTAL to 16MHz
    *SCR_MOSCCTL |= (1 << 0); //enable Main Oscillator Verification Circuit

    I2C3_GPIOInits();
    GPIO_ButtonInit();
    I2C3_Inits();

    uint8_t commandCode, len;

    while(1) {
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        commandCode = 0x51;

        I2C_MasterSendData(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_REPEAT_START);
        I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_REPEAT_START);

        commandCode = 0x52;

        I2C_MasterSendData(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_REPEAT_START);

        I2C_MasterReceiveData(&I2C1Handle, (uint8_t*)data, len, SLAVE_ADDR, I2C_NO_REPEAT_START);

        printf("%s\n", data);
    }

    return 0;
}
