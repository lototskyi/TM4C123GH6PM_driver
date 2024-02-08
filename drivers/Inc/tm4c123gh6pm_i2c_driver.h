#ifndef DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_

#include "tm4c123gh6pm.h"

typedef struct {
    uint32_t I2C_SCLSpeed;
    uint32_t I2C_SlaveDeviceAddress;
    uint32_t I2C_ACKControl;
    uint8_t  I2C_MasterEnable;
    uint8_t  I2C_SlaveEnable;
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
    uint8_t      *pTxBuffer;
    uint8_t      *pRxBuffer;
    uint32_t     TxLen;
    uint32_t     RxLen;
    uint8_t      TxRxState;
    uint8_t      DevAddr;
    uint32_t     RxSize;
    uint8_t      Sr;                /* !< To store repeated start value > */

} I2C_Handle_t;

/*
 * I2C application states
 */
#define I2C_READY               0
#define I2C_BUSY_IN_RX          1
#define I2C_BUSY_IN_TX          2

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

#define I2C_EV_DATA_REQ         0
#define I2C_EV_DATA_RCV         1
#define I2C_EV_STOP             2

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_PeripheralMasterControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_PeripheralSlaveControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_Handle_t *pI2CHandle);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2C_ApplicationEventMasterCallback(I2C_Handle_t *pI2CHandle);
void I2C_ApplicationEventSlaveCallback(I2C_Handle_t *pI2CHandle, uint8_t event);


#endif /* DRIVERS_INC_TM4C123GH6PM_I2C_DRIVER_H_ */
