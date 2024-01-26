#include <string.h>
#include "tm4c123gh6pm.h"
/*
 * PD0 --> SSI1Clk
 * PD1 --> SSI1Fss
 * PD2 --> SSI1Rx
 * PD3 --> SSI1Tx
 * Alt func 2
 */

void SSI1_GPIOInits(void)
{
    GPIO_Handle_t SSIPins;

    SSIPins.pGPIOx = GPIOD_AHB;
    SSIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SSIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 2;
    SSIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SSIPins.GPIO_PinConfig.GPIO_PinPinPuPdControl = GPIO_NO_PUPD;

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
    SSI1Handle.SSIConfig.SSI_ClkSpeed = SSI_SCLK_SPEED_DIV2;
    SSI1Handle.SSIConfig.SSI_DSS = SSI_DSS_8BITS;
    SSI1Handle.SSIConfig.SSI_SPO = SSI_SPO_LOW;
    SSI1Handle.SSIConfig.SSI_SPH = SSI_SPH_LOW;

    SSI_Init(&SSI1Handle);
}

int main(void)
{
   //main clock is 16MHz crystal oscillator
   *SCR_RCC &= ~(0x3f << 6);
   *SCR_RCC &= ~(1 << 0); //clear MOSCDIS
   *SCR_RCC &= ~(1 << 4); //clear OSCSRC
   *SCR_RCC |= (0x16 << 6); //set XTAL to 16MHz
   *SCR_MOSCCTL |= (1 << 0); //enable Main Oscillator Verification Circuit


    char user_data[] = "Hello world!";

    SSI1_GPIOInits();
    SSI1_Inits();

    SSI_PeripheralControl(SSI1, ENABLE);

    SSI_SendData(SSI1, (uint8_t*)user_data, strlen(user_data));

    SSI_PeripheralControl(SSI1, DISABLE);

    while(1);

    return 0;
}

