#include <string.h>
#include "tm4c123gh6pm.h"
/*
 * PD0 --> SSI1Clk
 * PD1 --> SSI1Fss
 * PD2 --> SSI1Rx
 * PD3 --> SSI1Tx
 * Alt func 2
 */

uint8_t dummy_write = 0xff;
uint8_t dummy_read;

//command codes
#define COMMAND_LED_CTRL                                    0x50
#define COMMAND_SENSOR_READ                                 0x51
#define COMMAND_LED_READ                                    0x52
#define COMMAND_PRINT                                       0x53
#define COMMAND_ID_READ                                     0x54

#define LED_ON                                              1
#define LED_OFF                                             0

//arduino analog pins
#define ANALOG_PIN0                                         0
#define ANALOG_PIN1                                         1
#define ANALOG_PIN2                                         2
#define ANALOG_PIN3                                         3
#define ANALOG_PIN4                                         4
#define ANALOG_PIN5                                         5

//arduino led
#define LED_PIN                                             9


void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000; i++);
}

void SSI1_GPIOInits(void)
{
    GPIO_Handle_t SSIPins;

    SSIPins.pGPIOx = GPIOD_AHB;
    SSIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SSIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    SSIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SSIPins.GPIO_PinConfig.GPIO_PinPinPuPdControl = GPIO_PIN_PU;

    //SCLK
    SSIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Init(&SSIPins);

    //Fss
    SSIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&SSIPins);

    //Rx
    SSIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&SSIPins);

    //Tx
    SSIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&SSIPins);
}

void SSI1_Inits(void)
{
    SSI_Handle_t SSI1Handle;
    SSI1Handle.pSSIx = SSI1;
    SSI1Handle.SSIConfig.SSI_FRF = FREESCALE_SPI_FRAME_FORMAT;
    SSI1Handle.SSIConfig.SSI_DeviceMode = SSI_DEVICE_MODE_MASTER;
    SSI1Handle.SSIConfig.SSI_ClkSpeed = SSI_SCLK_SPEED_DIV8;
    SSI1Handle.SSIConfig.SSI_DSS = SSI_DSS_8BITS;
    SSI1Handle.SSIConfig.SSI_SPO = SSI_SPO_LOW;
    SSI1Handle.SSIConfig.SSI_SPH = SSI_SPH_LOW;

    SSI_Init(&SSI1Handle);
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

uint8_t SSI_VerifyResponse(uint8_t ackByte)
{
    if (ackByte == 0xF5) {
        //ack
        return 1;
    } else {
        //nack
        return 0;
    }
}

uint8_t SSI_SendCommand(uint8_t commandCode, uint8_t *args, uint32_t argsLen)
{
    uint8_t ackByte;

    SSI_SendData(SSI1, &commandCode, 1);

    //do dummy read
    SSI_ReceiveData(SSI1, &dummy_read, 1);

    //send some dummy bits (1 byte) to fetch the response from the slave
    SSI_SendData(SSI1, &dummy_write, 1);

    //read ACK byte received
    SSI_ReceiveData(SSI1, &ackByte, 1);

    uint8_t response = SSI_VerifyResponse(ackByte);

    if ( response ) {
        //send arguments
        SSI_SendData(SSI1, args, argsLen);

        //clean receive buffer
        uint8_t i;
        for (i = 0; i < argsLen; i++) {
            SSI_ReceiveData(SSI1, &dummy_read, 1);
        }
    }

    return response;
}

int main(void)
{
    //main clock is 16MHz crystal oscillator
    *SCR_RCC &= ~(0x1f << 6);
    *SCR_RCC &= ~(1 << 0); //clear MOSCDIS
    *SCR_RCC &= ~(1 << 4); //clear OSCSRC
    *SCR_RCC |= (0x17 << 6); //set XTAL to 16MHz
    *SCR_MOSCCTL |= (1 << 0); //enable Main Oscillator Verification Circuit

    GPIO_ButtonInit();

    SSI1_GPIOInits();
    SSI1_Inits();

    while(1) {
        /** LED ON **/
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        SSI_PeripheralControl(SSI1, ENABLE);

        //1. COMMAND_LED_CTRL <pin no(1)>   <value(1)> ON
        uint8_t args[2];
        args[0] = LED_PIN;
        args[1] = LED_ON;

        SSI_SendCommand(COMMAND_LED_CTRL, args, 2);


        /** LED OFF **/
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        args[1] = LED_OFF;
        SSI_SendCommand(COMMAND_LED_CTRL, args, 2);
//
//        /** ANALOG READ **/
//        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
//        delay();
//
//        args[0] = ANALOG_PIN0;
//
//        uint8_t analogResponse = SSI_SendCommand(COMMAND_SENSOR_READ, args, 1);
//
//        uint8_t analog_read;
//
//        if (analogResponse) {
//            delay();
//
//            SSI_SendData(SSI1, &dummy_write, 1);
//            SSI_ReceiveData(SSI1, &analog_read, 1);
//        }
//
//        /** LED READ **/
//        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
//        delay();
//
//        args[0] = LED_PIN;
//        uint8_t ledPinResponse = SSI_SendCommand(COMMAND_LED_READ, args, 1);
//
//        uint8_t ledRead;
//
//        if (ledPinResponse) {
//            SSI_SendData(SSI1, &dummy_write, 1);
//            SSI_ReceiveData(SSI1, &ledRead, 1);
//        }
//
//
//        /** PRINT **/
//        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
//        delay();
//
//        uint8_t string[] = "Hello I'm here!";
//
//        args[0] = strlen((char*)string);
//        uint8_t printResponse = SSI_SendCommand(COMMAND_PRINT, args, 1);
//
//        if (printResponse) {
//            int i;
//
//            for (i = 0; i < args[0]; i++) {
//                SSI_SendData(SSI1, &string[i], 1);
//                SSI_ReceiveData(SSI1, &dummy_read, 1);
//            }
//        }

        /** ID READ **/
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();

        uint8_t idResponse = SSI_SendCommand(COMMAND_ID_READ, args, 0);

        uint8_t id[16];

        if (idResponse) {
            int i;
            for (i = 0; i < 15; i++) {
                SSI_SendData(SSI1, &dummy_write, 1);
                SSI_ReceiveData(SSI1, &id[i], 1);
            }
            id[16] = '\0';
        }

        while( SSI_GetFlagStatus(SSI1, SSI_BSY_FLAG) );

        SSI_PeripheralControl(SSI1, DISABLE);
    }


    return 0;
}









