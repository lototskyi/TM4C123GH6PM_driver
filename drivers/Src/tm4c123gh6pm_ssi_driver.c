

#include "tm4c123gh6pm_ssi_driver.h"

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

void SSI_SendData(SSI_RegDef_t *pSSIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

void SSI_ReceiveData(SSI_RegDef_t *pSSIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}

void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

void SSI_IRQHandling(SSI_RegDef_t *pSSIx)
{

}
