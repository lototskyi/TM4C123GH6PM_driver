#include <string.h>
#include "tm4c123gh6pm.h"

/*
 * PA6 -> SCL
 * PA7 -> SDA
 */

I2C_Handle_t I2C3Handle;

#define MY_ADDR             0x61
#define SLAVE_ADDR          0x68

#define STATUS_SEND_CODE1   0
#define STATUS_READ_LENGTH  1
#define STATUS_SEND_CODE2   2
#define STATUS_READ_DATA    3

uint8_t status;
uint8_t commandCode, len;

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
    I2C3Handle.pI2Cx = I2C3;
    I2C3Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
    I2C3Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C3Handle.I2C_Config.I2C_MasterEnable = I2C_MASTER_ENABLE;
    I2C3Handle.I2C_Config.I2C_SlaveEnable = I2C_SLAVE_DISABLE;

    I2C_Init(&I2C3Handle);

    //enable interrupt
    I2C3Handle.pI2Cx->MIMR |= (1 << I2CMIMR_IM);

    I2C_IRQInterruptConfig(IRQ_NO_I2C3, ENABLE);
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

    while(1) {
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        commandCode = 0x51;
        status = STATUS_SEND_CODE1;
        I2C_MasterSendData(&I2C3Handle, &commandCode, 1, SLAVE_ADDR, I2C_REPEAT_START);
    }

    return 0;
}

void I2C3_Handler(void)
{
    I2C_IRQHandling(&I2C3Handle);
}

void I2C_ApplicationEventMasterCallback(I2C_Handle_t *pI2CHandle)
{

    if (status == STATUS_READ_DATA) {
        printf("%s\n", data);
    }

    if (status == STATUS_SEND_CODE1) {
        status = STATUS_READ_LENGTH;
        I2C_MasterReceiveData(pI2CHandle, &len, 1, SLAVE_ADDR, I2C_REPEAT_START);
    } else if (status == STATUS_READ_LENGTH) {
        status = STATUS_SEND_CODE2;
        commandCode = 0x52;
        I2C_MasterSendData(pI2CHandle, &commandCode, 1, SLAVE_ADDR, I2C_REPEAT_START);
    } else if (status == STATUS_SEND_CODE2) {
        status = STATUS_READ_DATA;
        I2C_MasterReceiveData(pI2CHandle, (uint8_t*)data, len, SLAVE_ADDR, I2C_NO_REPEAT_START);
    }

}
