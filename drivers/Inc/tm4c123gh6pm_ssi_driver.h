
#ifndef DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_

#include "tm4c123gh6pm.h"


typedef struct {
    uint8_t SSI_DeviceMode;
    uint8_t SSI_BusConfig;
    uint8_t SSI_ClkSpeed;
    uint8_t SSI_DFF;
    uint8_t SSI_CPOL;
    uint8_t SSI_CPHA;
    uint8_t SSI_SSM;//!
} SSI_Config_t;

typedef struct {
    SSI_RegDef_t *pSSIx;                                    /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
    SSI_Config_t SSIConfig;
    uint8_t      *pTxBuffer;
    uint8_t      *pRxBuffer;
    uint32_t     TxLen;
    uint32_t     RxLen;
    uint8_t      TxState;
    uint8_t      RxState;
} SSI_Handle_t;


/*
 * @SSI_DeviceMode
 */
#define SSI_DEVICE_MODE_MASTER                              1
#define SSI_DEVICE_MODE_SLAVE                               2

void SSI_PeriClockControl(SSI_RegDef_t *pSSIx, uint8_t EnOrDi);

void SSI_Init(SSI_Handle_t *pSSIHandle);
void SSI_DeInit(SSI_Handle_t *pSSIHandle);

void SSI_SendData(SSI_RegDef_t *pSSIx, uint8_t *pTxBuffer, uint32_t Len);
void SSI_ReceiveData(SSI_RegDef_t *pSSIx, uint8_t *pRxBuffer, uint32_t Len);

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SSI_IRQHandling(SSI_RegDef_t *pSSIx);

#endif /* DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_ */
