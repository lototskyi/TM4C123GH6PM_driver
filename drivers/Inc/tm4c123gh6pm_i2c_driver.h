#ifndef DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_

#include "tm4c123gh6pm.h"

typedef struct {
    uint32_t I2C_SCLSpeed;
    uint32_t I2C_DeviceAddress;
    uint32_t I2C_ACKControl;
    uint8_t  I2C_MasterEnable;
    uint8_t  I2C_SlaveEnable;
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM4K      400000
#define I2C_SCL_SPEED_FM2K      200000

/*
 * @I2C_AckControl
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

#define I2C_MASTER_ENABLE       1
#define I2C_SLAVE_ENABLE        1

#define I2C_MASTER_DISABLE      0
#define I2C_SLAVE_DISABLE       0

#define I2C_NO_REPEAT_START     RESET
#define I2C_REPEAT_START        SET

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_PeripheralMasterControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_PeripheralSlaveControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t appEv);


#endif /* DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_ */
