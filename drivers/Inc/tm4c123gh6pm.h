/*
 * tm4c123gh6pm.h
 *
 *  Created on: 16 ñ³÷. 2024 ð.
 *      Author: olexandr
 */

#ifndef DRIVERS_INC_TM4C123GH6PM_H_
#define DRIVERS_INC_TM4C123GH6PM_H_

#include <stdint.h>

#define __vo                        volatile

#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET

/*
 * base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR              0x00000000U
#define SRAM_BASEADDR               0x20000000U
#define SRAM                        SRAM_BASEADDR

#define PERIPH_BASEADDR             0x40000000U

/**
 * Base addresses of peripherals
 */

//GPIO
#define GPIOA_APB_BASEADDR          0x40004000U
#define GPIOB_APB_BASEADDR          0x40005000U
#define GPIOC_APB_BASEADDR          0x40006000U
#define GPIOD_APB_BASEADDR          0x40007000U
#define GPIOE_APB_BASEADDR          0x40024000U
#define GPIOF_APB_BASEADDR          0x40025000U

#define GPIOA_AHB_BASEADDR          0x40058000U
#define GPIOB_AHB_BASEADDR          0x40059000U
#define GPIOC_AHB_BASEADDR          0x4005A000U
#define GPIOD_AHB_BASEADDR          0x4005B000U
#define GPIOE_AHB_BASEADDR          0x4005C000U
#define GPIOF_AHB_BASEADDR          0x4005D000U

//I2C
#define I2C0_BASEADDR               0x40020000U
#define I2C1_BASEADDR               0x40021000U
#define I2C2_BASEADDR               0x40022000U
#define I2C3_BASEADDR               0x40023000U

//SPI
#define SSI0_BASEADDR               0x40008000U
#define SSI1_BASEADDR               0x40009000U
#define SSI2_BASEADDR               0x4000A000U
#define SSI3_BASEADDR               0x4000B000U

//UART
#define UART0_BASEADDR              0x4000C000U
#define UART1_BASEADDR              0x4000D000U
#define UART2_BASEADDR              0x4000E000U
#define UART3_BASEADDR              0x4000F000U
#define UART4_BASEADDR              0x40010000U
#define UART5_BASEADDR              0x40011000U
#define UART6_BASEADDR              0x40012000U
#define UART7_BASEADDR              0x40013000U

#define SCR_BASE_ADDR                0x400FE000UL

/* System Control Register */
#define SCR_RCC                      ( (uint32_t*) (SCR_BASE_ADDR + 0x060) )      /* Run-Mode Clock Configuration :                                   0x060 */
#define SCR_MOSCCTL                  ( (uint32_t*) (SCR_BASE_ADDR + 0x07C) )      /* Main Oscillator Control :                                        0x07c */

//GPIO related system control registers
#define SCR_SRGPIO                   ( (uint32_t*) (SCR_BASE_ADDR + 0x508) )      /* General-Purpose Input/Output Software Reset :                    0x508 */
#define SCR_RCGCGPIO                 ( (uint32_t*) (SCR_BASE_ADDR + 0x608) )      /* General-Purpose Input/Output Run Mode Clock Gating Control :     0x608 */
#define SRC_PRGPIO                   ( (uint32_t*) (SCR_BASE_ADDR + 0xA08) )      /* General-Purpose Input/Output Peripheral Ready :                  0xa08 */

#define SCR_RCGCI2C                  ( (uint32_t*) (SCR_BASE_ADDR + 0x620) )      /* Inter-Integrated Circuit Run Mode Clock Gating Control :         0x620 */
#define SCR_RCGCSSI                  ( (uint32_t*) (SCR_BASE_ADDR + 0x61C) )      /* Synchronous Serial Interface Run Mode Clock Gating Control :     0x61c */
#define SCR_RCGCUART                 ( (uint32_t*) (SCR_BASE_ADDR + 0x618) )      /* Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating Control : 0x618 */

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 0) )
#define GPIOB_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 1) )
#define GPIOC_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 2) )
#define GPIOD_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 3) )
#define GPIOE_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 4) )
#define GPIOF_PCLK_RUN_EN()         ( *SCR_RCGCGPIO |= (1 << 5) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C0_PCLK_RUN_EN()          ( *SCR_RCGCI2C |= (1 << 0) )
#define I2C1_PCLK_RUN_EN()          ( *SCR_RCGCI2C |= (1 << 1) )
#define I2C2_PCLK_RUN_EN()          ( *SCR_RCGCI2C |= (1 << 2) )
#define I2C3_PCLK_RUN_EN()          ( *SCR_RCGCI2C |= (1 << 3) )

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SSI0_PCLK_RUN_EN()          ( *SCR_RCGCSSI |= (1 << 0) )
#define SSI1_PCLK_RUN_EN()          ( *SCR_RCGCSSI |= (1 << 1) )
#define SSI2_PCLK_RUN_EN()          ( *SCR_RCGCSSI |= (1 << 2) )
#define SSI3_PCLK_RUN_EN()          ( *SCR_RCGCSSI |= (1 << 3) )

/*
 * Clock Enable Macros for UARTx peripherals
 */
#define UART0_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 0) )
#define UART1_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 1) )
#define UART2_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 2) )
#define UART3_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 3) )
#define UART4_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 4) )
#define UART5_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 5) )
#define UART6_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 6) )
#define UART7_PCLK_RUN_EN()         ( *SCR_RCGCUART |= (1 << 7) )


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 0) )
#define GPIOB_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 1) )
#define GPIOC_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 2) )
#define GPIOD_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 3) )
#define GPIOE_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 4) )
#define GPIOF_PCLK_RUN_DI()         ( *SCR_RCGCGPIO &= ~(1 << 5) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C0_PCLK_RUN_DI()          ( *SCR_RCGCI2C &= ~(1 << 0) )
#define I2C1_PCLK_RUN_DI()          ( *SCR_RCGCI2C &= ~(1 << 1) )
#define I2C2_PCLK_RUN_DI()          ( *SCR_RCGCI2C &= ~(1 << 2) )
#define I2C3_PCLK_RUN_DI()          ( *SCR_RCGCI2C &= ~(1 << 3) )

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SSI0_PCLK_RUN_DI()          ( *SCR_RCGCSSI &= ~(1 << 0) )
#define SSI1_PCLK_RUN_DI()          ( *SCR_RCGCSSI &= ~(1 << 1) )
#define SSI2_PCLK_RUN_DI()          ( *SCR_RCGCSSI &= ~(1 << 2) )
#define SSI3_PCLK_RUN_DI()          ( *SCR_RCGCSSI &= ~(1 << 3) )

/*
 * Clock Disable Macros for UARTx peripherals
 */
#define UART0_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 0) )
#define UART1_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 1) )
#define UART2_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 2) )
#define UART3_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 3) )
#define UART4_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 4) )
#define UART5_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 5) )
#define UART6_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 6) )
#define UART7_PCLK_RUN_DI()         ( *SCR_RCGCUART &= ~(1 << 7) )

/***********************************peripheral register definition structures******************************************/

typedef struct {
    __vo uint32_t DATA;                              /* GPIO Data :                                  0x000*/
    __vo uint32_t DATA_BIT_MASK[255];                /* Bit mask */
    __vo uint32_t DIR;                               /* GPIO Direction :                             0x400*/
    __vo uint32_t IS;                                /* GPIO Interrupt Sense :                       0x404 */
    __vo uint32_t IBE;                               /* GPIO Interrupt Both Edges :                  0x408 */
    __vo uint32_t IEV;                               /* GPIO Interrupt Event :                       0x40C */
    __vo uint32_t IM;                                /* GPIO Interrupt Mask :                        0x410 */
    __vo uint32_t RIS;                               /* GPIO Raw Interrupt Status :                  0x414 */
    __vo uint32_t MIS;                               /* GPIO Masked Interrupt Status :               0x418 */
    __vo uint32_t ICR;                               /* GPIO Interrupt Clear :                       0x41C */
    __vo uint32_t AFSEL;                             /* GPIO Alternate Function Select :             0x420 */
    uint32_t reserved_0[55];
    __vo uint32_t DR2R;                              /* GPIO 2-mA Drive Select :                     0x500 */
    __vo uint32_t DR4R;                              /* GPIO 4-mA Drive Select :                     0x504 */
    __vo uint32_t DR8R;                              /* GPIO 8-mA Drive Select :                     0x508 */
    __vo uint32_t ODR;                               /* GPIO Open Drain Select :                     0x50C */
    __vo uint32_t PUR;                               /* GPIO Pull-Up Select :                        0x510 */
    __vo uint32_t PDR;                               /* GPIO Pull-Down Select :                      0x514 */
    __vo uint32_t SLR;                               /* GPIO Slew Rate Control Select :              0x518 */
    __vo uint32_t DEN;                               /* GPIO Digital Enable :                        0x51C */
    __vo uint32_t LOCK;                              /* GPIO Lock :                                  0x520 */
    __vo uint32_t CR;                                /* GPIO Commit :                                0x524 */
    __vo uint32_t AMSEL;                             /* GPIO Analog Mode Select :                    0x528 */
    __vo uint32_t PCTL;                              /* GPIO Port Control :                          0x52C */
    __vo uint32_t ADCCTL;                            /* GPIO ADC Control :                           0x530 */
    __vo uint32_t DMACTL;                            /* GPIO DMA Control :                           0x534 */
    uint32_t reserved_1[678];
    __vo uint32_t PeriphID4;                         /* GPIO Peripheral Identification 4 :           0xFD0 */
    __vo uint32_t PeriphID5;                         /* GPIO Peripheral Identification 5 :           0xFD4 */
    __vo uint32_t PeriphID6;                         /* GPIO Peripheral Identification 6 :           0xFD8 */
    __vo uint32_t PeriphID7;                         /* GPIO Peripheral Identification 7 :           0xFDC */
    __vo uint32_t PeriphID0;                         /* GPIO Peripheral Identification 0 :           0xFE0 */
    __vo uint32_t PeriphID1;                         /* GPIO Peripheral Identification 1 :           0xFE4 */
    __vo uint32_t PeriphID2;                         /* GPIO Peripheral Identification 2 :           0xFE8 */
    __vo uint32_t PeriphID3;                         /* GPIO Peripheral Identification 3 :           0xFEC */
    __vo uint32_t PCellID0;                          /* GPIO PrimeCell Identification 0 :            0xFF0 */
    __vo uint32_t PCellID1;                          /* GPIO PrimeCell Identification 1 :            0xFF4 */
    __vo uint32_t PCellID2;                          /* GPIO PrimeCell Identification 2 :            0xFF8 */
    __vo uint32_t PCellID3;                          /* GPIO PrimeCell Identification 3 :            0xFFC */
} GPIO_RegDef_t;


/**
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA_APB                  ( (GPIO_RegDef_t*) GPIOA_APB_BASEADDR )
#define GPIOB_APB                  ( (GPIO_RegDef_t*) GPIOB_APB_BASEADDR )
#define GPIOC_APB                  ( (GPIO_RegDef_t*) GPIOC_APB_BASEADDR )
#define GPIOD_APB                  ( (GPIO_RegDef_t*) GPIOD_APB_BASEADDR )
#define GPIOE_APB                  ( (GPIO_RegDef_t*) GPIOE_APB_BASEADDR )
#define GPIOF_APB                  ( (GPIO_RegDef_t*) GPIOF_APB_BASEADDR )

#define GPIOA_AHB                  ( (GPIO_RegDef_t*) GPIOA_AHB_BASEADDR )
#define GPIOB_AHB                  ( (GPIO_RegDef_t*) GPIOB_AHB_BASEADDR )
#define GPIOC_AHB                  ( (GPIO_RegDef_t*) GPIOC_AHB_BASEADDR )
#define GPIOD_AHB                  ( (GPIO_RegDef_t*) GPIOD_AHB_BASEADDR )
#define GPIOE_AHB                  ( (GPIO_RegDef_t*) GPIOE_AHB_BASEADDR )
#define GPIOF_AHB                  ( (GPIO_RegDef_t*) GPIOF_AHB_BASEADDR )

#endif /* DRIVERS_INC_TM4C123GH6PM_H_ */
