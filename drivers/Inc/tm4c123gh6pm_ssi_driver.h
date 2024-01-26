
#ifndef DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_

#include "tm4c123gh6pm.h"


typedef struct {
    uint8_t SSI_DeviceMode;
    uint8_t SSI_ClkSpeed;
    uint8_t SSI_FRF;                                        /* @SSI_FRF Frame Format Select */
    uint8_t SSI_DSS;                                        /* @SSI_DSS Data Size Select */
    uint8_t SSI_SPO;                                        /* @SSI_SPO Serial Clock Polarity */
    uint8_t SSI_SPH;                                        /* @SSI_SPH Serial Clock Phase */
    uint8_t SSI_LBM;                                        /* @SSI_LBM Loopback Mode */
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
#define SSI_DEVICE_MODE_MASTER                              0
#define SSI_DEVICE_MODE_SLAVE                               1

/*
 * @SSI_SclkSpeed
 */
#define SSI_SCLK_SPEED_DIV2                                 0
#define SSI_SCLK_SPEED_DIV4                                 1
#define SSI_SCLK_SPEED_DIV8                                 2
#define SSI_SCLK_SPEED_DIV16                                3
#define SSI_SCLK_SPEED_DIV32                                4
#define SSI_SCLK_SPEED_DIV64                                5
#define SSI_SCLK_SPEED_DIV128                               6
#define SSI_SCLK_SPEED_DIV256                               7

/*
 * @SSI_FRF
 */
#define FREESCALE_SPI_FRAME_FORMAT                          0
#define TI_SS_FRAME_FORMAT                                  1
#define MICROWIRE_FRAME_FORMAT                              2

/*
 * @SSI_LBM
 */
#define SSI_LOOPBACK_MODE_OFF                               0
#define SSI_LOOPBACK_MODE_ON                                1

/*
 * @SSI_DSS
 */
#define SSI_DSS_4BITS                                       0x3
#define SSI_DSS_5BITS                                       0x4
#define SSI_DSS_6BITS                                       0x5
#define SSI_DSS_7BITS                                       0x6
#define SSI_DSS_8BITS                                       0x7
#define SSI_DSS_9BITS                                       0x8
#define SSI_DSS_10BITS                                      0x9
#define SSI_DSS_11BITS                                      0xA
#define SSI_DSS_12BITS                                      0xB
#define SSI_DSS_13BITS                                      0xC
#define SSI_DSS_14BITS                                      0xD
#define SSI_DSS_15BITS                                      0xE
#define SSI_DSS_16BITS                                      0xF

/*
 * @SSI_SPO
 */
#define SSI_SPO_LOW                                         0
#define SSI_SPO_HIGH                                        1

/*
 * @SSI_SPH
 */
#define SSI_SPH_LOW                                         0
#define SSI_SPH_HIGH                                        1


/*
 * SSI related status flags definitions
 */
#define SSI_TFE_FLAG                                        ( 1 << SSISR_TFE )
#define SSI_RNE_FLAG                                        ( 1 << SSISR_RNE )
#define SSI_BSY_FLAG                                        ( 1 << SSISR_BSY )

void SSI_PeriClockControl(SSI_RegDef_t *pSSIx, uint8_t EnOrDi);

void SSI_PeripheralControl(SSI_RegDef_t *pSSIx, uint8_t EnOrDi);

void SSI_Init(SSI_Handle_t *pSSIHandle);
void SSI_DeInit(SSI_Handle_t *pSSIHandle);

void SSI_SendData(SSI_RegDef_t *pSSIx, uint8_t *pTxBuffer, uint32_t Len);
void SSI_ReceiveData(SSI_RegDef_t *pSSIx, uint8_t *pRxBuffer, uint32_t Len);

void SSI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SSI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SSI_IRQHandling(SSI_RegDef_t *pSSIx);

#endif /* DRIVERS_INC_TM4C123GH6PM_SSI_DRIVER_H_ */
