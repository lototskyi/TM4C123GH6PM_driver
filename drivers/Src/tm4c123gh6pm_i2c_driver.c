#include "tm4c123gh6pm_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStartStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_Run(I2C_RegDef_t *pI2Cx);
static void I2C_Stop(I2C_RegDef_t *pI2Cx);

uint32_t getSystemClockVal(void)
{
    if (*SCR_RCC & (0x3 << 4) == 0x1) { //PIOSC - Precision internal oscillator: 16MHz
        return 16000000;
    }

    if (*SCR_RCC & (0x1f << 6) == 0x15) { //Crystal oscillator 16MHz
        return 16000000;
    }

    //TODO: complete the function with other clock values and sources
    return 16000000;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        if (pI2Cx == I2C0) {
            I2C0_PCLK_RUN_EN();
        } else if (pI2Cx == I2C1) {
            I2C1_PCLK_RUN_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_RUN_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_RUN_EN();
        }

    } else {
        if (pI2Cx == I2C0) {
            I2C0_PCLK_RUN_DI();
        } else if (pI2Cx == I2C1) {
            I2C1_PCLK_RUN_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_RUN_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_RUN_DI();
        }
    }
}

void I2C_PeripheralMasterControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        pI2Cx->MCR |= (1 << I2CMCR_MFE);
    } else {
        pI2Cx->MCR &= ~(1 << I2CMCR_MFE);
    }
}

void I2C_PeripheralSlaveControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        pI2Cx->MCR |= (1 << I2CMCR_SFE);
        pI2Cx->SCSR |= (1 << I2CSCSR_DA);
    } else {
        pI2Cx->MCR &= ~(1 << I2CMCR_SFE);
        pI2Cx->SCSR &= ~(1 << I2CSCSR_DA);
    }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    I2C_PeriClockControl(pI2Cx, ENABLE);

    //configure timer period
    //from datasheet
    //TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
    //TPR = (20MHz/(2*(6+4)*100000))-1;
    //TPR = 9

    uint32_t I2CSCK = pI2CHandle->I2C_Config.I2C_SCLSpeed;
    uint32_t SCLK = getSystemClockVal();

    uint8_t TPR = (SCLK / (2 * (6 + 4) * I2CSCK)) - 1;
    pI2Cx->MTPR = TPR;

    //program the device own address
    pI2Cx->SOAR = pI2CHandle->I2C_Config.I2C_DeviceAddress;

    pI2Cx->MCLKOCNT = 200;
}

void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->pI2Cx == I2C0) {
        I2C0_REG_RESET();
    } else if (pI2CHandle->pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2CHandle->pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2CHandle->pI2Cx == I2C3) {
        I2C3_REG_RESET();
    }
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    //1. write slave addr to I2CMSA
    pI2Cx->MSA = (SlaveAddr << I2CMSA_SA);
    pI2Cx->MSA &= ~(1 << I2CMSA_RS);

    while(pI2Cx->MCS & (1 << I2CMCS_BUSBSY));

    I2C_GenerateStartCondition(pI2Cx);

    while (Len) {

        while(pI2Cx->MCS & (1 << I2CMCS_BUSY));

        pI2Cx->MDR = *pTxBuffer;

        I2C_Run(pI2Cx);

        pTxBuffer++;
        Len--;
    }

    I2C_GenerateStopCondition(pI2Cx);

}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x3;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x5;
}

static void I2C_GenerateStartStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x17;
}

static void I2C_Run(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x1;
}

static void I2C_Stop(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0;
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv)
{

}
