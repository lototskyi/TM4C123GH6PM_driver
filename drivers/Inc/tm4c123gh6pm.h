/*
 * tm4c123gh6pm.h
 *
 *  Created on: 16 ñ³÷. 2024 ð.
 *      Author: olexandr
 */

#ifndef DRIVERS_INC_TM4C123GH6PM_H_
#define DRIVERS_INC_TM4C123GH6PM_H_

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

#endif /* DRIVERS_INC_TM4C123GH6PM_H_ */
