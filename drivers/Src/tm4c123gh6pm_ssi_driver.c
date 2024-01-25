

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

    //8. serial port enable
    pSSIHandle->pSSIx->CR1 |= (1 << SSICR1_SSE);
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
