#ifndef DRIVERS_INC_TM4C123GH6PM_GPIO_DRIVER_H_
#define DRIVERS_INC_TM4C123GH6PM_GPIO_DRIVER_H_

#include "tm4c123gh6pm.h"

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN                                    0
#define GPIO_MODE_OUT                                   1
#define GPIO_MODE_ALTFN                                 2
#define GPIO_MODE_ANALOG                                3
#define GPIO_MODE_IT_FT                                 4
#define GPIO_MODE_IT_RT                                 5
#define GPIO_MODE_IT_RFT                                6

/*
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP                                 0 /* push-pull */
#define GPIO_OP_TYPE_OD                                 1 /* open drain */

/*
 * GPIO pin pull-up and pull-down configuration macros
 */
#define GPIO_NO_PUPD                                    0
#define GPIO_PIN_PU                                     1
#define GPIO_PIN_PD                                     2

/*
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0                                   0
#define GPIO_PIN_NO_1                                   1
#define GPIO_PIN_NO_2                                   2
#define GPIO_PIN_NO_3                                   3
#define GPIO_PIN_NO_4                                   4
#define GPIO_PIN_NO_5                                   5
#define GPIO_PIN_NO_6                                   6
#define GPIO_PIN_NO_7                                   7

typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;   /* possible values @GPIO_PIN_MODES */
    uint8_t GPIO_MaDriveSelect;
    uint8_t GPIO_PinSlew;
    uint8_t GPIO_PinPinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void Gpio_DeInit(GPIO_Handle_t *pGPIOHandle);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t pinNumber);


#endif /* DRIVERS_INC_TM4C123GH6PM_GPIO_DRIVER_H_ */
