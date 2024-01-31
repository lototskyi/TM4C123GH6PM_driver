

#include "tm4c123gh6pm_ssi_driver.h"

static void ssi_txe_interrupt_handle(SSI_Handle_t *pSSIHandle);
static void ssi_rxe_interrupt_handle(SSI_Handle_t *pSSIHandle);
static void ssi_rvr_error_interrupt_handle(SSI_Handle_t *pSSIHandle);

void SSI_PeriClockControl(SSI_RegDef_t *pSSIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        if (pSSIx == SSI0) {
            SSI0_PCLK_RUN_EN();
        } else if (pSSIx == SSI1) {
            SSI1_PCLK_RUN_EN();
        } else if (pSSIx == SSI2) {
            SSI2_PCLK_RUN_EN();
        } else if (pSSIx == SSI3) {
            SSI3_PCLK_RUN_EN();
        }

    } else {
        if (pSSIx == SSI0) {
            SSI0_PCLK_RUN_DI();
        } else if (pSSIx == SSI1) {
            SSI1_PCLK_RUN_DI();
        } else if (pSSIx == SSI2) {
            SSI2_PCLK_RUN_DI();
        } else if (pSSIx == SSI3) {
            SSI3_PCLK_RUN_DI();
        }
    }
}

void SSI_Init(SSI_Handle_t *pSSIHandle)
{
    //enable the peripheral clock
    SSI_PeriClockControl(pSSIHandle->pSSIx, ENABLE);

    //1. configure frame format
    const uint8_t frameFormat = pSSIHandle->SSIConfig.SSI_FRF;

    pSSIHandle->pSSIx->CR0 &= ~(0x3 << SSICR0_FRF); // freescale SPI by default

    if (frameFormat == TI_SS_FRAME_FORMAT) {
        pSSIHandle->pSSIx->CR0 |= (0x1 << SSICR0_FRF);
    } else if (frameFormat == MICROWIRE_FRAME_FORMAT) {
        pSSIHandle->pSSIx->CR0 |= (0x2 << SSICR0_FRF);
    }

    //2. configure device mode (master/slave)
    const uint8_t deviceMode = pSSIHandle->SSIConfig.SSI_DeviceMode;

    if (deviceMode == SSI_DEVICE_MODE_MASTER) {
        pSSIHandle->pSSIx->CR1 &= ~(1 << SSICR1_MS);
    } else {
        pSSIHandle->pSSIx->CR1 |= (1 << SSICR1_MS);
    }

    //3. set data size DSS
    pSSIHandle->pSSIx->CR0 &= ~(0xF << SSICR0_DSS);
    pSSIHandle->pSSIx->CR0 |= (pSSIHandle->SSIConfig.SSI_DSS << SSICR0_DSS);

    //4. set serial clock rate ( BR=SysClk/(CPSDVSR * (1 + SCR)))
    //   where SCR is in SSICR0(8:15) and range is number 0-255 and CPSDVSR is in SSICPSR(0:7) and range is even 2-254
    const uint8_t sClkDiv = pSSIHandle->SSIConfig.SSI_ClkSpeed;

    uint8_t scr = 0;
    uint8_t cpsdvsr = 2;

    if (sClkDiv == SSI_SCLK_SPEED_DIV4) {
        scr = 1;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV8) {
        scr = 3;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV16) {
        scr = 7;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV32) {
        scr = 15;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV64) {
        scr = 31;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV128) {
        scr = 63;
    } else if (sClkDiv == SSI_SCLK_SPEED_DIV256) {
        scr = 127;
    }

    pSSIHandle->pSSIx->CR0 |= (scr << SSICR0_SCR);
    pSSIHandle->pSSIx->CPSR |= (cpsdvsr << SSICPSR_CPSDVSR);

    //5. set clock phase
    if (pSSIHandle->SSIConfig.SSI_SPH == SSI_SPH_HIGH) {
        pSSIHandle->pSSIx->CR0 |= (1 << SSICR0_SPH);
    } else {
        pSSIHandle->pSSIx->CR0 &= ~(1 << SSICR0_SPH);
    }

    //6. set clock polarity
    if (pSSIHandle->SSIConfig.SSI_SPO == SSI_SPO_HIGH) {
        pSSIHandle->pSSIx->CR0 |= (1 << SSICR0_SPO);
    } else {
        pSSIHandle->pSSIx->CR0 &= ~(1 << SSICR0_SPO);
    }

    //7. loop-back mode
    if (pSSIHandle->SSIConfig.SSI_LBM == SSI_LOOPBACK_MODE_ON) {
        pSSIHandle->pSSIx->CR1 |= (1 << SSICR1_LBM);
    } else {
        pSSIHandle->pSSIx->CR1 &= ~(1 << SSICR1_LBM);
    }

//    pSSIHandle->pSSIx->CR1 |= (1 << SSICR1_EOT);

}

void SSI_DeInit(SSI_Handle_t *pSSIHandle)
{
    if (pSSIHandle->pSSIx == SSI0) {
        SSI0_REG_RESET();
    } else if (pSSIHandle->pSSIx == SSI1) {
        SSI1_REG_RESET();
    } else if (pSSIHandle->pSSIx == SSI2) {
        SSI2_REG_RESET();
    } else if (pSSIHandle->pSSIx == SSI3) {
        SSI3_REG_RESET();
    }
}

void SSI_PeripheralControl(SSI_RegDef_t *pSSIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        pSSIx->CR1 |= (1 << SSICR1_SSE);
    } else {
        pSSIx->CR1 &= ~(1 << SSICR1_SSE);
    }
}

uint8_t SSI_GetFlagStatus(SSI_RegDef_t *pSSIx, uint32_t FlagName)
{
    if (pSSIx->SR & FlagName) {
        return FLAG_SET;
    }

    return FLAG_RESET;
}

void SSI_SendData(SSI_RegDef_t *pSSIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while (Len > 0) {

        //1. wait until TFE is set
        while (SSI_GetFlagStatus(pSSIx, SSI_TFE_FLAG) == FLAG_RESET);

        //2. check if data size is more than 8 bits
        if ((pSSIx->CR0 & 0xF) > SSI_DSS_8BITS) {
            pSSIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        } else {
            pSSIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }

    }
}

void SSI_ReceiveData(SSI_RegDef_t *pSSIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while (Len > 0) {

        //1. wait until buffer is not empty
        while (SSI_GetFlagStatus(pSSIx, SSI_RNE_FLAG) == FLAG_RESET);

        //2. check if data size is more than 8 bits
        if ((pSSIx->CR0 & 0xF) > SSI_DSS_8BITS) {
            *((uint16_t*)pRxBuffer) = pSSIx->DR;
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        } else {
            *pRxBuffer = pSSIx->DR;
            Len--;
            pRxBuffer++;
        }

    }
}

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

uint8_t SSI_SendDataIT(SSI_Handle_t *pSSIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSSIHandle->TxState;

    if (state != SSI_BUSY_IN_TX) {
        pSSIHandle->pTxBuffer = pTxBuffer;
        pSSIHandle->TxLen = Len;

        pSSIHandle->TxState = SSI_BUSY_IN_TX;

        pSSIHandle->pSSIx->IM |= (1 << SSIIM_TXIM);
    }

    return state;
}

uint8_t SSI_ReceiveDataIT(SSI_Handle_t *pSSIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSSIHandle->RxState;

    if (state != SSI_BUSY_IN_RX) {
        pSSIHandle->pRxBuffer = pRxBuffer;
        pSSIHandle->RxLen = Len;

        pSSIHandle->RxState = SSI_BUSY_IN_RX;

        pSSIHandle->pSSIx->IM |= (1 << SSIIM_RXIM);
    }

    return state;
}

void SSI_IRQHandling(SSI_Handle_t *pSSIHandle)
{
    uint8_t Ris, Mis;

    Mis = pSSIHandle->pSSIx->MIS & ( 1 << SSIMIS_TXMIS );
    //Ris = pSSIHandle->pSSIx->RIS & ( 1 << SSIRIS_TXRIS );

    if (Mis) {
        //handle TXE
        ssi_txe_interrupt_handle(pSSIHandle);
    }

//    Mis = pSSIHandle->pSSIx->MIS & ( 1 << SSIMIS_RXMIS );
//    //Ris = pSSIHandle->pSSIx->RIS & ( 1 << SSIRIS_RXRIS );
//
//    if (Mis) {
//        //handle RXE
//        ssi_rxe_interrupt_handle(pSSIHandle);
//    }

    //OVERRUN
    Mis = pSSIHandle->pSSIx->MIS & ( 1 << SSIMIS_RORMIS );
    //Ris = pSSIHandle->pSSIx->RIS & ( 1 << SSIRIS_RORRIS );

    if (Mis) {
        //handle overrun
        ssi_rvr_error_interrupt_handle(pSSIHandle);
    }

}

//helper function implementations

static void ssi_txe_interrupt_handle(SSI_Handle_t *pSSIHandle)
{
    if (pSSIHandle->TxLen != 0) {
        if ((pSSIHandle->pSSIx->CR0 & 0xF) > SSI_DSS_8BITS) {
            pSSIHandle->pSSIx->DR = *((uint16_t*)pSSIHandle->pTxBuffer);
            pSSIHandle->TxLen--;
            pSSIHandle->TxLen--;
            (uint16_t*)pSSIHandle->pTxBuffer++;
        } else {
            pSSIHandle->pSSIx->DR = *pSSIHandle->pTxBuffer;
            pSSIHandle->TxLen--;
            pSSIHandle->pTxBuffer++;
        }
    } else {
        //disable interrupt
        SSI_CloseTransmission(pSSIHandle);

        SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_TX_CMPLT);
    }
}

static void ssi_rxe_interrupt_handle(SSI_Handle_t *pSSIHandle)
{
    if ((pSSIHandle->pSSIx->CR0 & 0xF) > SSI_DSS_8BITS) {
        *((uint16_t*)pSSIHandle->pRxBuffer) = pSSIHandle->pSSIx->DR;
        pSSIHandle->RxLen--;
        pSSIHandle->RxLen--;
        (uint16_t*)pSSIHandle->pRxBuffer++;
    } else {
        *pSSIHandle->pRxBuffer = pSSIHandle->pSSIx->DR;
        pSSIHandle->RxLen--;
        pSSIHandle->pRxBuffer++;
    }

    if (pSSIHandle->RxLen == 0) {
        //disable interrupt
        SSI_CloseReception(pSSIHandle);

        SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_RX_CMPLT);
    }
}

static void ssi_rvr_error_interrupt_handle(SSI_Handle_t *pSSIHandle)
{
    if (pSSIHandle->TxState != SSI_BUSY_IN_TX) {
        SSI_ClearOVRFlag(pSSIHandle->pSSIx);
    }

    SSI_ApplicationEventCallback(pSSIHandle, SSI_EVENT_OVR_ERR);
}

void SSI_CloseTransmission(SSI_Handle_t *pSSIHandle)
{
    pSSIHandle->pSSIx->IM &= ~(1 << SSIIM_TXIM);
    pSSIHandle->pTxBuffer = NULL;
    pSSIHandle->TxLen = 0;
    pSSIHandle->TxState = SSI_READY;
}

void SSI_CloseReception(SSI_Handle_t *pSSIHandle)
{
    pSSIHandle->pSSIx->IM &= ~(1 << SSIIM_RXIM);
    pSSIHandle->pRxBuffer = NULL;
    pSSIHandle->RxLen = 0;
    pSSIHandle->RxState = SSI_READY;
}

void SSI_ClearOVRFlag(SSI_RegDef_t *pSSIx)
{
    pSSIx->ICR |= (1 << SSIICR_RORIC);
}

__weak void SSI_ApplicationEventCallback(SSI_Handle_t *pSSIHandle, uint8_t appEv)
{

}


