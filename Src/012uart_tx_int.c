#include <string.h>
#include "tm4c123gh6pm.h"

char msg[1024] = "UART Tx testing...\n\r";
char rmsg[1024];

UART_Handle_t uart5_handle;

void delay(void)
{
    uint32_t i;
    for(i = 0; i < 500000; i++);
}

void UART5_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOE_AHB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 1;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    I2CPins.GPIO_PinConfig.GPIO_PinPinPuPdControl = GPIO_NO_PUPD;


    //Rx
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
    GPIO_Init(&I2CPins);

    //Tx
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GPIO_Init(&I2CPins);
}


void UART5_Init(void)
{
    uart5_handle.pUARTx = UART5;
    uart5_handle.UART_Config.UART_Baud = UART_STD_BAUD_115200;
    uart5_handle.UART_Config.UART_HWFlowControl = UART_HW_FLOW_CTRL_NONE;
    uart5_handle.UART_Config.UART_Mode = UART_MODE_TXRX;
    uart5_handle.UART_Config.UART_NoOfStopBits = UART_STOPBITS_1;
    uart5_handle.UART_Config.UART_WordLength = UART_WORDLEN_8BITS;
    uart5_handle.UART_Config.UART_ParityControl = UART_PARITY_DISABLE;

    UART_Init(&uart5_handle);

    UART_IRQInterruptConfig(IRQ_NO_UART5, ENABLE);
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
    GPIO_ButtonInit();
    UART5_GPIOInits();
    UART5_Init();
    UART_PeripheralControl(uart5_handle.pUARTx, ENABLE);

    while(1) {
        while( GPIO_ReadFromInputPin(GPIOF_AHB, GPIO_PIN_NO_4) );
        delay();
        UART_ReceiveDataIT(&uart5_handle, (uint8_t*)rmsg, strlen(msg));
        UART_SendDataIT(&uart5_handle, (uint8_t*)msg, strlen(msg));

    }

    return 1;
}

void UART5_Handler(void)
{
    UART_IRQHandling(&uart5_handle);
}

void UART_ApplicationEventCallback(UART_Handle_t *pUARTHandle, uint8_t AppEv)
{
    if (AppEv == UART_EVENT_RX_CMPLT) {
        if (pUARTHandle->RxLen == 0) {
            rmsg[strlen(msg) + 1] = '\0';
            printf("%s\n", rmsg);
        }
    }
}
