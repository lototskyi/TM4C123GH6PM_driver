

#include "tm4c123gh6pm_gpio_driver.h"


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {
        if (pGPIOx == GPIOA_APB || pGPIOx == GPIOA_AHB) {
            GPIOA_PCLK_RUN_EN();
        } else if (pGPIOx == GPIOB_APB || pGPIOx == GPIOB_AHB) {
            GPIOB_PCLK_RUN_EN();
        } else if (pGPIOx == GPIOC_APB || pGPIOx == GPIOC_AHB) {
            GPIOC_PCLK_RUN_EN();
        } else if (pGPIOx == GPIOD_APB || pGPIOx == GPIOD_AHB) {
            GPIOD_PCLK_RUN_EN();
        } else if (pGPIOx == GPIOE_APB || pGPIOx == GPIOE_AHB) {
            GPIOE_PCLK_RUN_EN();
        } else if (pGPIOx == GPIOF_APB || pGPIOx == GPIOF_AHB) {
            GPIOF_PCLK_RUN_EN();
        }
    } else {
        if (pGPIOx == GPIOA_APB || pGPIOx == GPIOA_AHB) {
            GPIOA_PCLK_RUN_DI();
        } else if (pGPIOx == GPIOB_APB || pGPIOx == GPIOB_AHB) {
            GPIOB_PCLK_RUN_DI();
        } else if (pGPIOx == GPIOC_APB || pGPIOx == GPIOC_AHB) {
            GPIOC_PCLK_RUN_DI();
        } else if (pGPIOx == GPIOD_APB || pGPIOx == GPIOD_AHB) {
            GPIOD_PCLK_RUN_DI();
        } else if (pGPIOx == GPIOE_APB || pGPIOx == GPIOE_AHB) {
            GPIOE_PCLK_RUN_DI();
        } else if (pGPIOx == GPIOF_APB || pGPIOx == GPIOF_AHB) {
            GPIOF_PCLK_RUN_DI();
        }
    }
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    const uint8_t pinMode       = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
    const uint8_t pinNumber     = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
    const uint8_t puPdSettings  = pGPIOHandle->GPIO_PinConfig.GPIO_PinPinPuPdControl;
    const uint8_t outputType    = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
    const uint8_t altFunc       = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;



    //1. configure the mode of gpio pin
    if (pinMode == GPIO_MODE_IN) {
        pGPIOHandle->pGPIOx->DIR &= ~(1 << pinNumber);
    } else if (pinMode == GPIO_MODE_OUT) {
        pGPIOHandle->pGPIOx->DIR |= (1 << pinNumber);
    } else if (pinMode == GPIO_MODE_ALTFN) {
        pGPIOHandle->pGPIOx->AFSEL |= (1 << pinNumber);
        pGPIOHandle->pGPIOx->PCTL &= ~(0xF << (4 * pinNumber));
        pGPIOHandle->pGPIOx->PCTL |= (altFunc << (4 * pinNumber));
    } else if (pinMode == GPIO_MODE_ANALOG) {
        pGPIOHandle->pGPIOx->AMSEL |= (1 << pinNumber);
    }

    //2. configure the pupd settings
    if (puPdSettings == GPIO_PIN_PU) {
        pGPIOHandle->pGPIOx->PUR |= (1 << pinNumber);
    } else if (puPdSettings == GPIO_PIN_PD) {
        pGPIOHandle->pGPIOx->PDR |= (1 << pinNumber);
    } else if (puPdSettings == GPIO_NO_PUPD) {
        pGPIOHandle->pGPIOx->PUR &= ~(1 << pinNumber);
        pGPIOHandle->pGPIOx->PDR &= ~(1 << pinNumber);
    }

    //3. configure the output type
    if (outputType == GPIO_OP_TYPE_OD) {
        pGPIOHandle->pGPIOx->ODR |= (1 << pinNumber);
    } else if (outputType == GPIO_OP_TYPE_PP) {
        pGPIOHandle->pGPIOx->ODR &= ~(1 << pinNumber);
    }

    if (pinMode != GPIO_MODE_ANALOG) {
        pGPIOHandle->pGPIOx->DEN |= (1 << pinNumber);
    }
}

void Gpio_DeInit(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t *scrSRGPIO = SCR_SRGPIO;
    *scrSRGPIO |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    *scrSRGPIO &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}

uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{

}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{

}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

void GPIO_IRQHandling(uint8_t pinNumber)
{

}
