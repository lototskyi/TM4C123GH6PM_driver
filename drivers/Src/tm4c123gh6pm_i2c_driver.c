#include "tm4c123gh6pm_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx, uint8_t Ack);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStartStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_Run(I2C_RegDef_t *pI2Cx);
static void I2C_Stop(I2C_RegDef_t *pI2Cx);

static void I2C_Ack(I2C_RegDef_t *pI2Cx);

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

    if (pI2CHandle->I2C_Config.I2C_MasterEnable == I2C_MASTER_ENABLE) {
        I2C_PeripheralMasterControl(pI2Cx, ENABLE);
    }

    if (pI2CHandle->I2C_Config.I2C_SlaveEnable == I2C_SLAVE_ENABLE) {
        I2C_PeripheralSlaveControl(pI2Cx, ENABLE);
    }

    //configure timer period
    //from datasheet
    //TPR = (System Clock/(2*(SCL_LP + SCL_HP)*SCL_CLK))-1;
    //TPR = (20MHz/(2*(6+4)*100000))-1;
    //TPR = 9

    uint32_t I2CSCK = pI2CHandle->I2C_Config.I2C_SCLSpeed;
    uint32_t SCLK = getSystemClockVal();

    uint8_t TPR = (SCLK / (2 * (6 + 4) * I2CSCK)) - 1;
    pI2Cx->MTPR |= (TPR << I2CMTPR_TPR);

    //program the device own address
    pI2Cx->SOAR = pI2CHandle->I2C_Config.I2C_SlaveDeviceAddress;

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

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    //1. write slave addr to I2CMSA
    pI2Cx->MSA = (SlaveAddr << I2CMSA_SA);
    pI2Cx->MSA &= ~(1 << I2CMSA_RS); //write

    if (Sr == I2C_NO_REPEAT_START) {
        while(pI2Cx->MCS & (1 << I2CMCS_BUSBSY));
    }

    pI2Cx->MDR = *pTxBuffer;

    pTxBuffer++;
    Len--;

    I2C_GenerateStartCondition(pI2Cx, I2C_ACK_DISABLE);

    while (Len) {

        while(pI2Cx->MCS & (1 << I2CMCS_BUSY));

        pI2Cx->MDR = *pTxBuffer;

        I2C_Run(pI2Cx);

        pTxBuffer++;
        Len--;
    }

    if (Sr == I2C_NO_REPEAT_START) {
        I2C_GenerateStopCondition(pI2Cx);
    }

    //while(pI2Cx->MCS & (1 << I2CMCS_BUSY));
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    //1. write slave addr to I2CMSA
    pI2Cx->MSA = (SlaveAddr << I2CMSA_SA);
    pI2Cx->MSA |= (1 << I2CMSA_RS); //receive

    if (Len == 1) {
        if (Sr == I2C_NO_REPEAT_START) {
            while(pI2Cx->MCS & (1 << I2CMCS_BUSBSY));
            I2C_GenerateStartStopCondition(pI2Cx);
        } else {
            I2C_GenerateStartCondition(pI2Cx, I2C_ACK_ENABLE);
        }

        while(pI2Cx->MCS & (1 << I2CMCS_BUSY));
        *pRxBuffer = pI2Cx->MDR;
    } else {
        I2C_GenerateStartCondition(pI2Cx, I2C_ACK_ENABLE);

        while (Len) {
            while(pI2Cx->MCS & (1 << I2CMCS_BUSY));

            *pRxBuffer = pI2Cx->MDR;

            if (Len > 2) {
                I2C_Ack(pI2Cx);
            } else {
                if (Sr == I2C_NO_REPEAT_START) {
                    I2C_GenerateStopCondition(pI2Cx);
                } else {
                    I2C_Run(pI2Cx);
                }
            }

            pRxBuffer++;
            Len--;
        }
    }

    //while(pI2Cx->MCS & (1 << I2CMCS_BUSY));
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->SDR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
    return (uint8_t)pI2Cx->SDR;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx, uint8_t Ack)
{
    if (Ack == I2C_ACK_DISABLE) {
        pI2Cx->MCS = 0x3;
    } else {
        pI2Cx->MCS = 0xb;
    }
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x5;
}

static void I2C_GenerateStartStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x7;
}

static void I2C_Run(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x1;
}

static void I2C_Stop(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0;
}

static void I2C_Ack(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->MCS = 0x9;
}

void I2C_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    I2C_RegDef_t *pI2Cx = pI2CHandle->pI2Cx;

    //master
    if (pI2Cx->MMIS & (1 << I2CMMIS_MIS) && !(pI2Cx->MCS & (1 << I2CMCS_BUSY)) ) {
        pI2Cx->MICR |= (1 << I2CMICR_IC);

        if (pI2Cx->MCS & (1 << I2CMCS_ERROR)) {
            //handle error
            if (pI2Cx->MCS & (1 << I2CMCS_ADRACK)) {
                printf("Address NACK!\n");
            } else if (pI2Cx->MCS & (1 << I2CMCS_DATACK)) {
                printf("Data NACK!\n");
            }

        } else {
            I2C_ApplicationEventMasterCallback(pI2CHandle);
        }
    } else if (pI2Cx->SMIS & (1 << I2CSMIS_DATAMIS)) { //slave data
        pI2Cx->SICR |= (1 << I2CSICR_DATAIC);
        if (pI2Cx->SCSR & (1 << I2CSCSR_TREQ)) {
            I2C_ApplicationEventSlaveCallback(pI2CHandle, I2C_EV_DATA_REQ);
        } else if (pI2Cx->SCSR & (1 << I2CSCSR_RREQ)) {
            I2C_ApplicationEventSlaveCallback(pI2CHandle, I2C_EV_DATA_RCV);
        }

    } else if (pI2Cx->SMIS & (1 << I2CSMIS_STARTMIS)) { //slave start
        pI2Cx->SICR |= (1 << I2CSICR_STARTIC);
    } else if (pI2Cx->SMIS & (1 << I2CSMIS_STOPMIS)) { //slave stop
        pI2Cx->SICR |= (1 << I2CSICR_STOPIC);
        I2C_ApplicationEventSlaveCallback(pI2CHandle, I2C_EV_STOP);
    }

}

__attribute__((weak)) void I2C_ApplicationEventMasterCallback(I2C_Handle_t *pI2CHandle)
{

}

__attribute__((weak)) void I2C_ApplicationEventSlaveCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{

}
