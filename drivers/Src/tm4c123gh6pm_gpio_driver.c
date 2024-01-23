

#include "tm4c123gh6pm_gpio_driver.h"

static uint32_t pinBusAddrOffset[8] = {0x004, 0x008, 0x010, 0x020, 0x040, 0x080, 0x100, 0x200};

void GPIO_AHBEnable(GPIO_RegDef_t *pGPIOx)
{
    __vo uint32_t *pGPIOHBCTLr = SCR_GPIOHBCTL;

    if (pGPIOx == GPIOA_AHB) {
        *pGPIOHBCTLr |= (1 << 0);
    } else if(pGPIOx == GPIOB_AHB) {
        *pGPIOHBCTLr |= (1 << 1);
    } else if(pGPIOx == GPIOC_AHB) {
        *pGPIOHBCTLr |= (1 << 2);
    } else if(pGPIOx == GPIOD_AHB) {
        *pGPIOHBCTLr |= (1 << 3);
    } else if(pGPIOx == GPIOE_AHB) {
        *pGPIOHBCTLr |= (1 << 4);
    } else if(pGPIOx == GPIOF_AHB) {
        *pGPIOHBCTLr |= (1 << 5);
    }
}

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE) {

        GPIO_AHBEnable(pGPIOx);

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

    //4. interrupts
    if (pinMode > GPIO_MODE_ANALOG) {
        if (pinMode == GPIO_MODE_IT_FT) {
            pGPIOHandle->pGPIOx->IEV &= ~(1 << pinNumber);
        } else if (pinMode == GPIO_MODE_IT_RT) {
            pGPIOHandle->pGPIOx->IEV |= (1 << pinNumber);
        } else if (pinMode == GPIO_MODE_IT_RFT) {
            pGPIOHandle->pGPIOx->IBE |= (1 << pinNumber);
        }

        //enable interrupt
        pGPIOHandle->pGPIOx->IM |= (1 << pinNumber);
    }

}

void Gpio_DeInit(GPIO_Handle_t *pGPIOHandle)
{
    GPIO_RegDef_t *pGPIOx = pGPIOHandle->pGPIOx;

    if (pGPIOx == GPIOA_APB || pGPIOx == GPIOA_AHB) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB_APB || pGPIOx == GPIOB_AHB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC_APB || pGPIOx == GPIOC_AHB) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD_APB || pGPIOx == GPIOD_AHB) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE_APB || pGPIOx == GPIOE_AHB) {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOF_APB || pGPIOx == GPIOF_AHB) {
        GPIOF_REG_RESET();
    }
}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
    uint32_t *dataAddr;


    dataAddr = (uint32_t*)((uint32_t)pGPIOx + pinBusAddrOffset[pinNumber]);

    return (uint8_t)*dataAddr;
}

uint8_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint32_t *dataAddr;

    dataAddr = (uint32_t*)((uint32_t)pGPIOx + 0x3fc);

    return (uint8_t)*dataAddr;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
    uint32_t *dataAddr;
    dataAddr = (uint32_t*)((uint32_t)pGPIOx + pinBusAddrOffset[pinNumber]);

    *dataAddr = (value << pinNumber);
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{
    uint32_t *dataAddr;
    dataAddr = (uint32_t*)((uint32_t)pGPIOx + 0x3fc);

    *dataAddr = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
    uint32_t *dataAddr;
    dataAddr = (uint32_t*)((uint32_t)pGPIOx + pinBusAddrOffset[pinNumber]);

    *dataAddr ^= (1 << pinNumber);
}

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

void GPIO_IRQHandling(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber)
{
    //clear interrupt
    pGPIOx->ICR |= (1 << pinNumber);
}
