#ifndef DRIVERS_INC_TM4C123GH6PM_UART_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_UART_DRIVER_H_

#include "tm4c123gh6pm.h"

typedef struct {
    uint8_t UART_Mode;
    uint32_t UART_Baud;
    uint8_t UART_NoOfStopBits;
    uint8_t UART_WordLength;
    uint8_t UART_ParityControl;
    uint8_t UART_HWFlowControl;
} UART_Config_t;

typedef struct {
    UART_RegDef_t *pUARTx;
    UART_Config_t UART_Config;
} UART_Handle_t;

/*
 *@UART_Mode
 *Possible options for UART_Mode
 */
#define UART_MODE_ONLY_TX 0
#define UART_MODE_ONLY_RX 1
#define UART_MODE_TXRX  2

/*
 *@UART_Baud
 *Possible options for UART_Baud
 */
#define UART_STD_BAUD_1200                 1200
#define UART_STD_BAUD_2400                 2400
#define UART_STD_BAUD_9600                 9600
#define UART_STD_BAUD_19200                19200
#define UART_STD_BAUD_38400                38400
#define UART_STD_BAUD_57600                57600
#define UART_STD_BAUD_115200               115200
#define UART_STD_BAUD_230400               230400
#define UART_STD_BAUD_460800               460800
#define UART_STD_BAUD_921600               921600
#define UART_STD_BAUD_2M                   2000000
#define UART_STD_BAUD_3M                   3000000

/*
 *@UART_ParityControl
 *Possible options for UART_ParityControl
 */
#define UART_PARITY_EN_ODD                 2
#define UART_PARITY_EN_EVEN                1
#define UART_PARITY_DISABLE                0

/*
 *@UART_WordLength
 *Possible options for UART_WordLength
 */
#define UART_WORDLEN_5BITS                 0
#define UART_WORDLEN_6BITS                 1
#define UART_WORDLEN_7BITS                 2
#define UART_WORDLEN_8BITS                 3

#define UART_WORDLEN_9BITS                 4

/*
 *@UART_NoOfStopBits
 *Possible options for UART_NoOfStopBits
 */
#define UART_STOPBITS_1                    1
#define UART_STOPBITS_2                    2


/*
 *@UART_HWFlowControl
 *Possible options for UART_HWFlowControl
 */
#define UART_HW_FLOW_CTRL_NONE             0
#define UART_HW_FLOW_CTRL_CTS              1
#define UART_HW_FLOW_CTRL_RTS              2
#define UART_HW_FLOW_CTRL_CTS_RTS          3


/*
 * USART flags
 */

#define UART_FLAG_TXFE                      ( 1 << UARTFR_TXFE)
#define UART_FLAG_RXFF                      ( 1 << UARTFR_RXFF)
#define UART_FLAG_BUSY                      ( 1 << UARTFR_BUSY)

/*
 * Application states
// */
//#define UART_BUSY_IN_RX                    1
//#define UART_BUSY_IN_TX                    2
//#define UART_READY                         0
//
//
//#define USART_EVENT_TX_CMPLT                0
//#define USART_EVENT_RX_CMPLT                1
//#define USART_EVENT_IDLE                    2
//#define USART_EVENT_CTS                     3
//#define USART_EVENT_PE                      4
//#define USART_ERR_FE                        5
//#define USART_ERR_NE                        6
//#define USART_ERR_ORE                       7
//
//
///*
// * SPI related status flags definitions
// */
//#define USART_TXE_FLAG                                      (1 << USART_SR_TXE)
//#define USART_RXNE_FLAG                                     (1 << USART_SR_RXNE)


/******************************************************************************************
 *                              APIs supported by this driver
 *       For more information about the APIs check the function definitions
 ******************************************************************************************/


/*
 * Peripheral Clock setup
 */
void UART_PeriClockControl(UART_RegDef_t *pUARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void UART_Init(UART_Handle_t *pUARTHandle);
void UART_DeInit(UART_RegDef_t *pUARTx);


/*
 * Data Send and Receive
 */
void UART_SendData(UART_Handle_t *pUARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void UART_ReceiveData(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t UART_SendDataIT(UART_Handle_t *pUARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t UART_ReceiveDataIT(UART_Handle_t *pUARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void UART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void UART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void UART_IRQHandling(UART_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */
void UART_PeripheralControl(UART_RegDef_t *pUARTx, uint8_t EnOrDi);
uint8_t UART_GetFlagStatus(UART_RegDef_t *pUARTx , uint32_t FlagName);
void UART_ClearFlag(UART_RegDef_t *pUARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void UART_ApplicationEventCallback(UART_Handle_t *pUARTHandle,uint8_t AppEv);


#endif /* DRIVERS_INC_TM4C123GH6PM_UART_DRIVER_H_ */
