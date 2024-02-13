#include "tm4c123gh6pm_uart_driver.h"


static void storeDataIntoBuffer(UART_Handle_t *pUARTHandle);

/*
 * Peripheral Clock setup
 */
void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        if (pUARTx == UART0) {
            UART0_PCLK_RUN_EN();
        } else if (pUARTx == UART1) {
            UART1_PCLK_RUN_EN();
        } else if (pUARTx == UART2) {
            UART2_PCLK_RUN_EN();
        } else if (pUARTx == UART3) {
            UART3_PCLK_RUN_EN();
        } else if (pUARTx == UART4) {
            UART4_PCLK_RUN_EN();
        } else if (pUARTx == UART5) {
            UART5_PCLK_RUN_EN();
        } else if (pUARTx == UART6) {
            UART6_PCLK_RUN_EN();
        } else if (pUARTx == UART7) {
            UART7_PCLK_RUN_EN();
        }

    } else {
        if (pUARTx == UART0) {
            UART0_PCLK_RUN_DI();
        } else if (pUARTx == UART1) {
            UART1_PCLK_RUN_DI();
        } else if (pUARTx == UART2) {
            UART2_PCLK_RUN_DI();
        } else if (pUARTx == UART3) {
            UART3_PCLK_RUN_DI();
        } else if (pUARTx == UART4) {
            UART4_PCLK_RUN_DI();
        } else if (pUARTx == UART5) {
            UART5_PCLK_RUN_DI();
        } else if (pUARTx == UART6) {
            UART6_PCLK_RUN_DI();
        } else if (pUARTx == UART7) {
            UART7_PCLK_RUN_DI();
        }
    }
}

/*
 * Init and De-init
 */
void UART_Init(UART_Handle_t *pUARTHandle)
{
    UART_RegDef_t *pUARTx = pUARTHandle->pUARTx;
    UART_Config_t UART_Config = pUARTHandle->UART_Config;

    UART_PeriClockControl(pUARTx, ENABLE);

    //baud rate calculation
    float BRD = (float)16000000 / ((float)16 * (float)UART_Config.UART_Baud);
    uint16_t intBRD = (uint16_t)BRD;
    float fract = BRD - intBRD;

    uint8_t DIVFRAC = (uint32_t) (fract * 64 + 0.5);

    pUARTx->IBRD |= (intBRD << UARTIBRD_DIVUNT);
    pUARTx->FBRD |= (DIVFRAC << UARTFBRD_DIVFRAC);

    //mode (RX, TX, RXTX)
    if (UART_Config.UART_Mode == UART_MODE_ONLY_RX) {
        pUARTx->CTL &= ~(1 << UARTCTL_TXE);
        pUARTx->CTL |= (1 << UARTCTL_RXE);
    } else if (UART_Config.UART_Mode == UART_MODE_ONLY_TX) {
        pUARTx->CTL &= ~(1 << UARTCTL_RXE);
        pUARTx->CTL |= (1 << UARTCTL_TXE);
    } else if (UART_Config.UART_Mode == UART_MODE_TXRX) {
        pUARTx->CTL |= (1 << UARTCTL_RXE);
        pUARTx->CTL |= (1 << UARTCTL_TXE);
    }

    //word length
    pUARTx->LCRH |= (UART_Config.UART_WordLength << UARTLCRH_WLEN);

    //parity
    if (UART_Config.UART_ParityControl == UART_PARITY_EN_EVEN) {
        pUARTx->LCRH |= (1 << UARTLCRH_PEN);
        pUARTx->LCRH |= (1 << UARTLCRH_EPS);
    } else if (UART_Config.UART_ParityControl == UART_PARITY_EN_ODD) {
        pUARTx->LCRH |= (1 << UARTLCRH_PEN);
        pUARTx->LCRH &= ~(1 << UARTLCRH_EPS);
    } else {
        pUARTx->LCRH &= ~(1 << UARTLCRH_PEN);
    }

    //nr of stop bits
    if (UART_Config.UART_NoOfStopBits == UART_STOPBITS_1) {
        pUARTx->LCRH &= ~(1 << UARTLCRH_STP2);
    } else {
        pUARTx->LCRH |= (1 << UARTLCRH_STP2);
    }

    //HW flow control
    if (UART_Config.UART_HWFlowControl == UART_HW_FLOW_CTRL_CTS) {
        pUARTx->CTL |= (1 << UARTCTL_CTSEN);
        pUARTx->CTL &= ~(1 << UARTCTL_RTSEN);
    } else if (UART_Config.UART_HWFlowControl == UART_HW_FLOW_CTRL_RTS) {
        pUARTx->CTL &= ~(1 << UARTCTL_CTSEN);
        pUARTx->CTL |= (1 << UARTCTL_RTSEN);
    } else if (UART_Config.UART_HWFlowControl == UART_HW_FLOW_CTRL_CTS_RTS) {
        pUARTx->CTL |= (1 << UARTCTL_CTSEN);
        pUARTx->CTL |= (1 << UARTCTL_RTSEN);
    }


}

void UART_DeInit(UART_RegDef_t *pUARTx)
{
    if (pUARTx == UART0) {
        UART0_REG_RESET();
    } else if (pUARTx == UART1) {
        UART1_REG_RESET();
    } else if (pUARTx == UART2) {
        UART2_REG_RESET();
    } else if (pUARTx == UART3) {
        UART3_REG_RESET();
    } else if (pUARTx == UART4) {
        UART4_REG_RESET();
    } else if (pUARTx == UART5) {
        UART5_REG_RESET();
    } else if (pUARTx == UART6) {
        UART6_REG_RESET();
    } else if (pUARTx == UART7) {
        UART7_REG_RESET();
    }
}


/*
 * Data Send and Receive
 */
void UART_SendData(UART_Handle_t *pUARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    UART_RegDef_t *pUARTx = pUARTHandle->pUARTx;
    UART_Config_t UART_Config = pUARTHandle->UART_Config;

    uint32_t i;

    for(i = 0; i < Len; i++) {

        while(! UART_GetFlagStatus(pUARTx, UART_FLAG_TXFE));

        if(UART_Config.UART_WordLength == UART_WORDLEN_8BITS) {
            pUARTx->DR = (*pTxBuffer & (uint8_t)0x0FF);
        } else if (UART_Config.UART_WordLength == UART_WORDLEN_7BITS) {
            pUARTx->DR = (*pTxBuffer & (uint8_t)0x07F);
        } else if (UART_Config.UART_WordLength == UART_WORDLEN_6BITS) {
            pUARTx->DR = (*pTxBuffer & (uint8_t)0x03F);
        } else if (UART_Config.UART_WordLength == UART_WORDLEN_5BITS) {
            pUARTx->DR = (*pTxBuffer & (uint8_t)0x01F);
        }

        pTxBuffer++;
    }

    while(UART_GetFlagStatus(pUARTx, UART_FLAG_BUSY));
}

void UART_ReceiveData(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    UART_RegDef_t *pUARTx = pUARTHandle->pUARTx;

    uint32_t i;

    for(i = 0; i < Len; i++) {

        while(! UART_GetFlagStatus(pUARTx, UART_FLAG_RXFF));

        *pRxBuffer = pUARTx->DR;

        pRxBuffer++;
    }

    while(UART_GetFlagStatus(pUARTx, UART_FLAG_BUSY));
}

uint8_t UART_SendDataIT(UART_Handle_t *pUARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t txState = pUARTHandle->TxBusyState;

    if (txState != UART_BUSY_IN_TX) {

        pUARTHandle->TxLen = Len;
        pUARTHandle->pTxBuffer = pTxBuffer;
        pUARTHandle->TxBusyState = UART_BUSY_IN_TX;

        pUARTHandle->pUARTx->IM |= (1 << UARTIM_TXIM);

        storeDataIntoBuffer(pUARTHandle);
    }

    return txState;
}

uint8_t UART_ReceiveDataIT(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t rxState = pUARTHandle->RxBusyState;

    if (rxState != UART_BUSY_IN_RX) {

        pUARTHandle->RxLen = Len;
        pUARTHandle->pRxBuffer = pRxBuffer;
        pUARTHandle->RxBusyState = UART_BUSY_IN_RX;

        pUARTHandle->pUARTx->IM |= (1 << UARTIM_RXIM);
    }

    return rxState;
}

/*
 * IRQ Configuration and ISR handling
 */
void UART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
    uint8_t IRQBitNumber = (uint8_t)(IRQNumber % 32);

    if (EnOrDi == ENABLE) {
        if (IRQNumber <= 31) {
            //EN0
            *NVIC_ISER0 |= (1 << IRQBitNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            //EN1
            *NVIC_ISER1 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            //EN2
            *NVIC_ISER2 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 96 && IRQNumber < 128) {
            //EN3
            *NVIC_ISER3 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 127 && IRQNumber <= 138) {
            //EN4
            *NVIC_ISER4 |= (1 << IRQBitNumber);
        }
    } else {
        if (IRQNumber <= 31) {
            //EN0
            *NVIC_ICER0 |= (1 << IRQBitNumber);
        } else if (IRQNumber > 31 && IRQNumber < 64) {
            //EN1
            *NVIC_ICER1 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 64 && IRQNumber < 96) {
            //EN2
            *NVIC_ICER2 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 96 && IRQNumber < 128) {
            //EN3
            *NVIC_ICER3 |= (1 << IRQBitNumber);
        } else if (IRQNumber >= 127 && IRQNumber <= 138) {
            //EN4
            *NVIC_ICER4 |= (1 << IRQBitNumber);
        }
    }
}

void UART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/*
 * Other Peripheral Control APIs
 */
void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        pUARTx->CTL |= (1 << UARTCTL_UARTEN);
    } else {
        pUARTx->CTL &= ~(1 << UARTCTL_UARTEN);
    }
}

uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx , uint32_t FlagName)
{
    if (pUARTx->FR & FlagName) {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

void UART_ClearFlag(UART_RegDef_t *pUARTx, uint16_t StatusFlagName)
{

}

static void storeDataIntoBuffer(UART_Handle_t *pUARTHandle)
{
    UART_RegDef_t *pUARTx = pUARTHandle->pUARTx;
    UART_Config_t UART_Config = pUARTHandle->UART_Config;

    if(UART_Config.UART_WordLength == UART_WORDLEN_8BITS) {
        pUARTx->DR = (*pUARTHandle->pTxBuffer & (uint8_t)0x0FF);
    } else if (UART_Config.UART_WordLength == UART_WORDLEN_7BITS) {
        pUARTx->DR = (*pUARTHandle->pTxBuffer & (uint8_t)0x07F);
    } else if (UART_Config.UART_WordLength == UART_WORDLEN_6BITS) {
        pUARTx->DR = (*pUARTHandle->pTxBuffer & (uint8_t)0x03F);
    } else if (UART_Config.UART_WordLength == UART_WORDLEN_5BITS) {
        pUARTx->DR = (*pUARTHandle->pTxBuffer & (uint8_t)0x01F);
    }
}

void UART_IRQHandling(UART_Handle_t *pUARTHandle)
{

    UART_RegDef_t *pUARTx = pUARTHandle->pUARTx;

    while(UART_GetFlagStatus(pUARTx, UART_FLAG_BUSY));

    if (pUARTHandle->RxBusyState == UART_BUSY_IN_RX && pUARTx->RIS & (1 << UARTRIS_RXRIS) && pUARTHandle->RxLen > 0) {
        *pUARTHandle->pRxBuffer = pUARTx->DR;
        pUARTHandle->pRxBuffer++;
        pUARTHandle->RxLen--;
    } else if (pUARTHandle->TxBusyState == UART_BUSY_IN_TX && (pUARTx->RIS & (1 << UARTRIS_TXRIS)) && pUARTHandle->TxLen > 0) {

        pUARTHandle->pTxBuffer++;
        pUARTHandle->TxLen--;

        if (pUARTHandle->TxLen > 0) {
            storeDataIntoBuffer(pUARTHandle);
        }
    }

    if (pUARTHandle->RxLen == 0 && pUARTHandle->RxBusyState == UART_BUSY_IN_RX) {
        pUARTHandle->pUARTx->ICR |= (1 << UARTICR_RXIC);
        //disable interrupt
        pUARTHandle->pUARTx->IM &= ~(1 << UARTIM_RXIM);

        pUARTHandle->pRxBuffer = NULL;
        pUARTHandle->RxLen = 0;
        pUARTHandle->RxBusyState = UART_READY;

        UART_ApplicationEventCallback(pUARTHandle, UART_EVENT_RX_CMPLT);
    }



    if (pUARTHandle->TxLen == 0) {
        pUARTHandle->pUARTx->ICR |= (1 << UARTICR_TXIC);
        //disable interrupt
        pUARTHandle->pUARTx->IM &= ~(1 << UARTIM_TXIM);

        pUARTHandle->pTxBuffer = NULL;
        pUARTHandle->TxLen = 0;
        pUARTHandle->TxBusyState = UART_READY;

        UART_ApplicationEventCallback(pUARTHandle, UART_EVENT_TX_CMPLT);
    }

}

/*
 * Application callback
 */
__attribute__((weak)) void UART_ApplicationEventCallback(UART_Handle_t *pUARTHandle, uint8_t AppEv)
{

}
