/*
 * tm4c123gh6pm.h
 *
 *  Created on: 16 ñ³÷. 2024 ð.
 *      Author: olexandr
 */

#ifndef DRIVERS_INC_TM4C123GH6PM_H_
#define DRIVERS_INC_TM4C123GH6PM_H_

#include <stddef.h>
#include <stdint.h>

#define __vo                        volatile
#define __weak                      __attribute__((weak))

#define ENABLE                      1
#define DISABLE                     0
#define SET                         ENABLE
#define RESET                       DISABLE
#define GPIO_PIN_SET                SET
#define GPIO_PIN_RESET              RESET
#define FLAG_RESET                  RESET
#define FLAG_SET                    SET

/************************** START: Processor Specific Details **********************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0                  ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1                  ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2                  ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3                  ( (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4                  ( (__vo uint32_t*)0xE000E110 )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0                  ( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1                  ( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2                  ( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3                  ( (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4                  ( (__vo uint32_t*)0xE000E190 )

/*
 * ARM Cortex Mx Processor Priority register Addresses Calculation
 */

#define NVIC_PR_BASE_ADDR           ( (__vo uint32_t*)0xE000E400 )

/************************** STOP: Processor Specific Details **********************************/

#define NO_PR_BITS_IMPLEMENTED      3

#define IRQ_NO_GPIOA                0
#define IRQ_NO_GPIOB                1
#define IRQ_NO_GPIOC                2
#define IRQ_NO_GPIOD                3
#define IRQ_NO_GPIOE                4
#define IRQ_NO_GPIOF                30
#define IRQ_NO_SSI0                 7
#define IRQ_NO_SSI1                 34
#define IRQ_NO_SSI2                 57
#define IRQ_NO_SSI3                 58

#define NVIC_IRQ_PRI0               0
#define NVIC_IRQ_PRI1               1
#define NVIC_IRQ_PRI2               2
#define NVIC_IRQ_PRI3               3
#define NVIC_IRQ_PRI4               4
#define NVIC_IRQ_PRI5               5
#define NVIC_IRQ_PRI6               6
#define NVIC_IRQ_PRI7               7

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
#define SCR_RCC                      ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x060) )      /* Run-Mode Clock Configuration :                                   0x060 */
#define SCR_MOSCCTL                  ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x07C) )      /* Main Oscillator Control :                                        0x07c */

//GPIO related system control registers
#define SCR_GPIOHBCTL                ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x06C) )      /* GPIO High-Performance Bus Control :                              0x06c */
#define SCR_SRGPIO                   ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x508) )      /* General-Purpose Input/Output Software Reset :                    0x508 */
#define SCR_RCGCGPIO                 ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x608) )      /* General-Purpose Input/Output Run Mode Clock Gating Control :     0x608 */
#define SRC_PRGPIO                   ( (__vo uint32_t*) (SCR_BASE_ADDR + 0xA08) )      /* General-Purpose Input/Output Peripheral Ready :                  0xa08 */

#define SCR_RCGCI2C                  ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x620) )      /* Inter-Integrated Circuit Run Mode Clock Gating Control :         0x620 */
#define SCR_SRI2C                    ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x520) )      /* Inter-Integrated Circuit Software Reset :                        0x520 */

#define SCR_RCGCSSI                  ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x61C) )      /* Synchronous Serial Interface Run Mode Clock Gating Control :     0x61c */
#define SCR_SRSSI                    ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x51C) )      /* Synchronous Serial Interface Software Reset :                    0x51c */

#define SCR_RCGCUART                 ( (__vo uint32_t*) (SCR_BASE_ADDR + 0x618) )      /* Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating Control : 0x618 */

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

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 0) );\
                                    ( *SCR_SRGPIO &= ~(1 << 0) )
#define GPIOB_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 1) );\
                                    ( *SCR_SRGPIO &= ~(1 << 1) )
#define GPIOC_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 2) );\
                                    ( *SCR_SRGPIO &= ~(1 << 2) )
#define GPIOD_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 3) );\
                                    ( *SCR_SRGPIO &= ~(1 << 3) )
#define GPIOE_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 4) );\
                                    ( *SCR_SRGPIO &= ~(1 << 4) )
#define GPIOF_REG_RESET()           ( *SCR_SRGPIO |=  (1 << 5) );\
                                    ( *SCR_SRGPIO &= ~(1 << 5) )
/*
 * Macros to reset SSIx peripherals
 */
#define SSI0_REG_RESET()            ( *SCR_SRSSI |=  (1 << 0) );\
                                    ( *SCR_SRSSI &= ~(1 << 0) )
#define SSI1_REG_RESET()            ( *SCR_SRSSI |=  (1 << 1) );\
                                    ( *SCR_SRSSI &= ~(1 << 1) )
#define SSI2_REG_RESET()            ( *SCR_SRSSI |=  (1 << 2) );\
                                    ( *SCR_SRSSI &= ~(1 << 2) )
#define SSI3_REG_RESET()            ( *SCR_SRSSI |=  (1 << 3) );\
                                    ( *SCR_SRSSI &= ~(1 << 3) )


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C0_REG_RESET()            ( *SCR_SRI2C |=  (1 << 0) );\
                                    ( *SCR_SRI2C &= ~(1 << 0) )
#define I2C1_REG_RESET()            ( *SCR_SRI2C |=  (1 << 1) );\
                                    ( *SCR_SRI2C &= ~(1 << 1) )
#define I2C2_REG_RESET()            ( *SCR_SRI2C |=  (1 << 2) );\
                                    ( *SCR_SRI2C &= ~(1 << 2) )
#define I2C3_REG_RESET()            ( *SCR_SRI2C |=  (1 << 3) );\
                                    ( *SCR_SRI2C &= ~(1 << 3) )

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


typedef struct {
    __vo uint32_t CR0;                              /* SSI Control 0 :                              0x000 */
    __vo uint32_t CR1;                              /* SSI Control 1 :                              0x004 */
    __vo uint32_t DR;                               /* SSI Data :                                   0x008 */
    __vo uint32_t SR;                               /* SSI Status :                                 0x00C */
    __vo uint32_t CPSR;                             /* SSI Clock Prescaler :                        0x010 */
    __vo uint32_t IM;                               /* SSI Interrupt Mask :                         0x014 */
    __vo uint32_t RIS;                              /* SSI Interrupt Status :                       0x018 */
    __vo uint32_t MIS;                              /* SSI Masked Interrupt Status :                0x01C */
    __vo uint32_t ICR;                              /* SSI Interrupt Clear :                        0x020 */
    __vo uint32_t DMACTL;                           /* SSI DMA Control :                            0x024 */
    uint32_t reserved_0[1000];
    __vo uint32_t CC;                               /* SSI Clock Configuration :                    0xFC8 */
    uint32_t reserved_1;
    __vo uint32_t PeriphID4;                        /* SSI Peripheral Identification 4 :            0xFD0 */
    __vo uint32_t PeriphID5;                        /* SSI Peripheral Identification 5 :            0xFD4 */
    __vo uint32_t PeriphID6;                        /* SSI Peripheral Identification 6 :            0xFD8 */
    __vo uint32_t PeriphID7;                        /* SSI Peripheral Identification 7 :            0xFDC */
    __vo uint32_t PeriphID0;                        /* SSI Peripheral Identification 0 :            0xFE0 */
    __vo uint32_t PeriphID1;                        /* SSI Peripheral Identification 1 :            0xFE4 */
    __vo uint32_t PeriphID2;                        /* SSI Peripheral Identification 2 :            0xFE8 */
    __vo uint32_t PeriphID3;                        /* SSI Peripheral Identification 3 :            0xFEC */
    __vo uint32_t PCellID0;                         /* SSI PrimeCell Identification 0 :             0xFF0 */
    __vo uint32_t PCellID1;                         /* SSI PrimeCell Identification 1 :             0xFF4 */
    __vo uint32_t PCellID2;                         /* SSI PrimeCell Identification 2 :             0xFF8 */
    __vo uint32_t PCellID3;                         /* SSI PrimeCell Identification 3 :             0xFFC */
} SSI_RegDef_t;

#define SSI0                       ( (SSI_RegDef_t*) SSI0_BASEADDR )
#define SSI1                       ( (SSI_RegDef_t*) SSI1_BASEADDR )
#define SSI2                       ( (SSI_RegDef_t*) SSI2_BASEADDR )
#define SSI3                       ( (SSI_RegDef_t*) SSI3_BASEADDR )


/******************************************************************************************************************
 * Bit position definitions of SSI peripheral
 ******************************************************************************************************************/
#define SSICR0_DSS                 0
#define SSICR0_FRF                 4
#define SSICR0_SPO                 6
#define SSICR0_SPH                 7
#define SSICR0_SCR                 8

#define SSICR1_LBM                 0
#define SSICR1_SSE                 1
#define SSICR1_MS                  2
#define SSICR1_EOT                 4

#define SSISR_TFE                  0
#define SSISR_TNF                  1
#define SSISR_RNE                  2
#define SSISR_RFF                  3
#define SSISR_BSY                  4

#define SSICPSR_CPSDVSR            0

#define SSIIM_RORIM                0
#define SSIIM_RTIM                 1
#define SSIIM_RXIM                 2
#define SSIIM_TXIM                 3

#define SSIRIS_RORRIS              0
#define SSIRIS_RTRIS               1
#define SSIRIS_RXRIS               2
#define SSIRIS_TXRIS               3

#define SSIMIS_RORMIS              0
#define SSIMIS_RTMIS               1
#define SSIMIS_RXMIS               2
#define SSIMIS_TXMIS               3

#define SSIICR_RORIC               0
#define SSIICR_RTIC                1

#define SSIDMACTL_RXDMAE           0
#define SSIDMACTL_TXDMAE           1

#define SSICC_CS                   0

typedef struct {
    __vo uint32_t MSA;                                /* I2C Master Slave Address :                              0x000 */
    __vo uint32_t MCS;                                /* I2C Master Control/Status :                             0x004 */
    __vo uint32_t MDR;                                /* I2C Master Data :                                       0x008 */
    __vo uint32_t MTPR;                               /* I2C Master Timer Period :                               0x00C */
    __vo uint32_t MIMR;                               /* I2C Master Interrupt Mask :                             0x010 */
    __vo uint32_t MRIS;                               /* I2C Master Raw Interrupt Status :                       0x014 */
    __vo uint32_t MMIS;                               /* I2C Master Masked Interrupt Status :                    0x018 */
    __vo uint32_t MICR;                               /* I2C Master Interrupt Clear :                            0x01C */
    __vo uint32_t MCR;                                /* I2C Master Configuration :                              0x020 */
    __vo uint32_t MCLKOCNT;                           /* I2C Master Clock Low Timeout Count :                    0x024 */
    uint32_t reserved_0;
    __vo uint32_t MBMON;                              /* I2C Master Bus Monitor :                                0x02C */
    uint32_t reserved_1[2];
    __vo uint32_t MCR2;                               /* I2C Master Configuration 2 :                            0x038 */
    uint32_t reserved_2[497];
    __vo uint32_t SOAR;                               /* I2C Slave Own Address :                                 0x800 */
    __vo uint32_t SCSR;                               /* I2C Slave Control/Status :                              0x804 */
    __vo uint32_t SDR;                                /* I2C Slave Data :                                        0x808 */
    __vo uint32_t SIMR;                               /* I2C Slave Interrupt Mask :                              0x80C */
    __vo uint32_t SRIS;                               /* I2C Slave Raw Interrupt Status :                        0x810 */
    __vo uint32_t SMIS;                               /* I2C Slave Masked Interrupt Status :                     0x814 */
    __vo uint32_t SICR;                               /* I2C Slave Interrupt Clear :                             0x818 */
    __vo uint32_t SOAR2;                              /* I2C Slave Own Address 2 :                               0x81C */
    __vo uint32_t SACKCTL;                            /* I2C Slave ACK Control :                                 0x820 */
    uint32_t reserved_3[487];
    __vo uint32_t PP;                                 /* I2C Peripheral Properties :                             0xFC0 */
    __vo uint32_t PC;                                 /* I2C Peripheral Configuration :                          0xFC4 */
} I2C_RegDef_t;

#define I2C0                       ( (I2C_RegDef_t*) I2C0_BASEADDR )
#define I2C1                       ( (I2C_RegDef_t*) I2C1_BASEADDR )
#define I2C2                       ( (I2C_RegDef_t*) I2C2_BASEADDR )
#define I2C3                       ( (I2C_RegDef_t*) I2C3_BASEADDR )

/******************************************************************************************************************
 * Bit position definitions of I2C peripheral
 ******************************************************************************************************************/
#define I2CMSA_RS                 0
#define I2CMSA_SA                 1

//read-only
#define I2CMCS_BUSY               0
#define I2CMCS_ERROR              1
#define I2CMCS_ADRACK             2
#define I2CMCS_DATACK             3
#define I2CMCS_ARBLST             4
#define I2CMCS_IDLE               5
#define I2CMCS_BUSBSY             6
#define I2CMCS_CLKTO              7

//write-only
#define I2CMCS_RUN                0
#define I2CMCS_START              1
#define I2CMCS_STOP               2
#define I2CMCS_ACK                3
#define I2CMCS_HS                 4

#define I2CMDR_DATA               0

#define I2CMTPR_TPR               0
#define I2CMTPR_HS                1

#define I2CMIMR_IM                0
#define I2CMIMR_CLKIM             1

#define I2CMRIS_RIS               0
#define I2CMRIS_CLKRIS            1

#define I2CMMIS_MIS               0
#define I2CMMIS_CLKMIS            1

#define I2CMICR_IC                0
#define I2CMICR_CLKIC             1

#define I2CMCR_LPBK               0
#define I2CMCR_MFE                4
#define I2CMCR_SFE                5
#define I2CMCR_GFE                6

#define I2CMCLKOCNT_CNTL          0

#define I2CMBMON_SCL              0
#define I2CMBMON_SDA              1

#define I2CMCR2_GFPW              4

#define I2CSOAR_OAR               0

//read-only
#define I2CSCSR_RREQ              0
#define I2CSCSR_TREQ              1
#define I2CSCSR_FBR               2
#define I2CSCSR_OAR2SEL           3

//write-only
#define I2CSCSR_DA                0

#define I2CSDR_DATA               0

#define I2CSIMR_DATAIM            0
#define I2CSIMR_STARTIM           1
#define I2CSIMR_STOPIM            2

#define I2CSRIS_DATARIS           0
#define I2CSRIS_STARTRIS          1
#define I2CSRIS_STOPRIS           2

#define I2CSMIS_DATAMIS           0
#define I2CSMIS_STARTMIS          1
#define I2CSMIS_STOPMIS           2

#define I2CSICR_DATAIC            0
#define I2CSICR_STARTIC           1
#define I2CSICR_STOPIC            2

#define I2CSOAR2_OAR2             0
#define I2CSOAR2_OAR2EN           1

#define I2CSACKCTL_ACKOEN         0
#define I2CSACKCTL_ACKOVAL        1

#define I2CPP_HS                  0

#define I2CPC_HS                  0

#include "tm4c123gh6pm_gpio_driver.h"
#include "tm4c123gh6pm_ssi_driver.h"
#include "tm4c123gh6pm_i2c_driver.h"

#endif /* DRIVERS_INC_TM4C123GH6PM_H_ */
