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





#endif /* DRIVERS_INC_TM4C123GH6PM_H_ */
