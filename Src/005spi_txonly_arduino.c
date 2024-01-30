#include <string.h>
#include "tm4c123gh6pm.h"
/*
 * PD0 --> SSI1Clk
 * PD1 --> SSI1Fss
 * PD2 --> SSI1Rx
 * PD3 --> SSI1Tx
 * Alt func 2
 */

void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000/2; i++);
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

int main(void)
{
   //main clock is 16MHz crystal oscillator
   *SCR_RCC &= ~(0x1f << 6);
   *SCR_RCC &= ~(1 << 0); //clear MOSCDIS
   *SCR_RCC &= ~(1 << 4); //clear OSCSRC
   *SCR_RCC |= (0x17 << 6); //set XTAL to 16MHz
   *SCR_MOSCCTL |= (1 << 0); //enable Main Oscillator Verification Circuit


    char user_data[] = "Hello world!";

    GPIO_ButtonInit();

    SSI1_GPIOInits();
    SSI1_Inits();

    while(1) {
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );

        delay();

        SSI_PeripheralControl(SSI1, ENABLE);

        //send size of data
        uint8_t datalen = strlen(user_data);

        SSI_SendData(SSI1, &datalen, 1);

        SSI_SendData(SSI1, (uint8_t*)user_data, strlen(user_data));

        while( SSI_GetFlagStatus(SSI1, SSI_BSY_FLAG) );

        SSI_PeripheralControl(SSI1, DISABLE);
    }


    return 0;
}





