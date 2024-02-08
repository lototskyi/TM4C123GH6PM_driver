#include <string.h>
#include "tm4c123gh6pm.h"

/*
 * PA6 -> SCL
 * PA7 -> SDA
 */

I2C_Handle_t I2C3Handle;

#define SLAVE_ADDR          0x68
#define MY_ADDR             SLAVE_ADDR

uint32_t dataLen, tempLen = 0;
const uint8_t data[] = "In publishing and graphic design, Lorem ipsum is a placeholder text commonly used to demonstrate the visual form of a document or a typeface without relying on meaningful content. Lorem ipsum may be used as a placeholder before the final copy is available.\n"; //0x101

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
    I2C3Handle.I2C_Config.I2C_SlaveDeviceAddress = MY_ADDR;
    I2C3Handle.I2C_Config.I2C_MasterEnable = I2C_MASTER_DISABLE;
    I2C3Handle.I2C_Config.I2C_SlaveEnable = I2C_SLAVE_ENABLE;

    I2C_Init(&I2C3Handle);

    //enable interrupt
    I2C3Handle.pI2Cx->SIMR |= (1 << I2CSIMR_DATAIM);
//    I2C3Handle.pI2Cx->SIMR |= (1 << I2CSIMR_STARTIM);
//    I2C3Handle.pI2Cx->SIMR |= (1 << I2CSIMR_STOPIM);

    I2C_IRQInterruptConfig(IRQ_NO_I2C3, ENABLE);
}

int main(void)
{
    //main clock is 16MHz crystal oscillator
    *SCR_RCC &= ~(0x1f << 6);
    *SCR_RCC &= ~(1 << 0); //clear MOSCDIS
    *SCR_RCC &= ~(1 << 4); //clear OSCSRC
    *SCR_RCC |= (0x17 << 6); //set XTAL to 16MHz
    *SCR_MOSCCTL |= (1 << 0); //enable Main Oscillator Verification Circuit

    dataLen = strlen((char*)data);
    tempLen = dataLen;

    I2C3_GPIOInits();
    I2C3_Inits();

    while(1) {

    }

    return 0;
}

void I2C3_Handler(void)
{
    I2C_IRQHandling(&I2C3Handle);
}

void I2C_ApplicationEventSlaveCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
    static uint8_t commandCode = 0xff;
    static uint32_t Cnt = 0;

    if (event == I2C_EV_DATA_REQ) {
        //slave has to send data
        if (commandCode == 0x51) {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, tempLen);
            tempLen = (tempLen >> 8) & 0xff;
        } else if (commandCode == 0x52) {
            I2C_SlaveSendData(pI2CHandle->pI2Cx, data[Cnt++]);

            if (Cnt == dataLen) {
                commandCode = 0xff;
                Cnt = 0;
            }
        }
    } else if (event == I2C_EV_DATA_RCV) {
        //slave has to receive data
        commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
    } else if (event == I2C_EV_STOP) {
//        printf("stop\n");
    }

}
