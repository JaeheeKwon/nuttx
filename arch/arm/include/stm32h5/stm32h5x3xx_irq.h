/****************************************************************************
 * arch/arm/include/stm32h5/stm32h5x3xx_irq.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* This file should never be included directly but, rather,
 * only indirectly through arch/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_STM32H5_STM32H5X3XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds vector number and hence map directly to bits
 * in the NVIC.
 * This does, however, waste several words of memory in the IRQ to handle
 * mapping tables.
 *
 * Processor Exceptions (vectors 0-15).  These common definitions can be
 * found in the file nuttx/arch/arm/include/stm32h5/irq.h which includes
 * this file
 *
 * External interrupts (vectors >= 16)
 */
/* Table 135, Reference Manual */
#define STM32_IRQ_WWDG1        (STM32_IRQ_FIRST + 0)    /*  0: Window Watchdog interrupt */
#define STM32_IRQ_PVDPVM       (STM32_IRQ_FIRST + 1)    /*  1: PVD through EXTI line detection interrupt */
#define STM32_IRQ_RTC          (STM32_IRQ_FIRST + 2)    /*  2: RTC golbal non-secure interrupt */
#define STM32_IRQ_RTCS         (STM32_IRQ_FIRST + 3)    /*  3: RTC golbal secure interrupt */
#define STM32_IRQ_TAMP         (STM32_IRQ_FIRST + 4)    /*  4: Tamper global interrupt */
#define STM32_IRQ_RAMCFG       (STM32_IRQ_FIRST + 5)    /*  5: RAM configuration interrupt */
#define STM32_IRQ_FLASH        (STM32_IRQ_FIRST + 6)    /*  6: Flash non-secure global interrupt */
#define STM32_IRQ_FLASHS       (STM32_IRQ_FIRST + 7)    /*  7: Flash secure global interrupt */
#define STM32_IRQ_GTZC         (STM32_IRQ_FIRST + 8)    /*  8: GTZC global interrupt */
#define STM32_IRQ_RCC          (STM32_IRQ_FIRST + 9)    /*  9: RCC non-secure global interrupt */
#define STM32_IRQ_RCCS         (STM32_IRQ_FIRST + 10)   /* 10: RCC secure global interrupt */
#define STM32_IRQ_EXTI0        (STM32_IRQ_FIRST + 11)   /* 11: EXTI Line 0 interrupt */
#define STM32_IRQ_EXTI1        (STM32_IRQ_FIRST + 12)   /* 12: EXTI Line 1 interrupt */
#define STM32_IRQ_EXTI2        (STM32_IRQ_FIRST + 13)   /* 13: EXTI Line 2 interrupt */
#define STM32_IRQ_EXTI3        (STM32_IRQ_FIRST + 14)   /* 14: EXTI Line 3 interrupt */
#define STM32_IRQ_EXTI4        (STM32_IRQ_FIRST + 15)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI5        (STM32_IRQ_FIRST + 16)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI6        (STM32_IRQ_FIRST + 16)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI7        (STM32_IRQ_FIRST + 18)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI8        (STM32_IRQ_FIRST + 19)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI9        (STM32_IRQ_FIRST + 20)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI10       (STM32_IRQ_FIRST + 21)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI11       (STM32_IRQ_FIRST + 22)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI12       (STM32_IRQ_FIRST + 23)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI13       (STM32_IRQ_FIRST + 24)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI14       (STM32_IRQ_FIRST + 25)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_EXTI15       (STM32_IRQ_FIRST + 26)   /* 10: EXTI Line 4 interrupt */
#define STM32_IRQ_DMA1S0       (STM32_IRQ_FIRST + 27)   /* 11: DMA1 Stream0 global interrupt */
#define STM32_IRQ_DMA1S1       (STM32_IRQ_FIRST + 28)   /* 12: DMA1 Stream1 global interrupt */
#define STM32_IRQ_DMA1S2       (STM32_IRQ_FIRST + 29)   /* 13: DMA1 Stream2 global interrupt */
#define STM32_IRQ_DMA1S3       (STM32_IRQ_FIRST + 30)   /* 14: DMA1 Stream3 global interrupt */
#define STM32_IRQ_DMA1S4       (STM32_IRQ_FIRST + 31)   /* 15: DMA1 Stream4 global interrupt */
#define STM32_IRQ_DMA1S5       (STM32_IRQ_FIRST + 32)   /* 16: DMA1 Stream5 global interrupt */
#define STM32_IRQ_DMA1S6       (STM32_IRQ_FIRST + 33)   /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_DMA1S7       (STM32_IRQ_FIRST + 34)   /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_IWDG         (STM32_IRQ_FIRST + 35)   /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_SAES         (STM32_IRQ_FIRST + 36)   /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_ADC1         (STM32_IRQ_FIRST + 37)   /* 18: ADC1 global interrupt */
#define STM32_IRQ_DAC1         (STM32_IRQ_FIRST + 38)   /* 54: DAC1 underrun error interrupt */
#define STM32_IRQ_FDCAN1_0     (STM32_IRQ_FIRST + 39)   /* 19: FDCAN1 Interrupt 0 */
#define STM32_IRQ_FDCAN1_1     (STM32_IRQ_FIRST + 40)   /* 21: FDCAN1 Interrupt 1 */
#define STM32_IRQ_TIM1BRK      (STM32_IRQ_FIRST + 41)   /* 24: TIM1 break interrupt */
#define STM32_IRQ_TIM1UP       (STM32_IRQ_FIRST + 42)   /* 25: TIM1 update interrupt */
#define STM32_IRQ_TIM1TRGCOM   (STM32_IRQ_FIRST + 43)   /* 26: TIM1 trigger and commutation interrupts */
#define STM32_IRQ_TIM1CC       (STM32_IRQ_FIRST + 44)   /* 27: TIM1 capture / compare interrupt */
#define STM32_IRQ_TIM2         (STM32_IRQ_FIRST + 45)   /* 28: TIM2 global interrupt */
#define STM32_IRQ_TIM3         (STM32_IRQ_FIRST + 46)   /* 29: TIM3 global interrupt */
#define STM32_IRQ_TIM4         (STM32_IRQ_FIRST + 47)   /* 30: TIM4 global interrupt */
#define STM32_IRQ_TIM5         (STM32_IRQ_FIRST + 48)   /* 28: TIM2 global interrupt */
#define STM32_IRQ_TIM6         (STM32_IRQ_FIRST + 49)   /* 29: TIM3 global interrupt */
#define STM32_IRQ_TIM7         (STM32_IRQ_FIRST + 50)   /* 30: TIM4 global interrupt */
#define STM32_IRQ_I2C1EV       (STM32_IRQ_FIRST + 51)   /* 31: I2C1 event interrupt */
#define STM32_IRQ_I2C1ER       (STM32_IRQ_FIRST + 52)   /* 32: I2C1 error interrupt */
#define STM32_IRQ_I2C2EV       (STM32_IRQ_FIRST + 53)   /* 33: I2C2 event interrupt */
#define STM32_IRQ_I2C2ER       (STM32_IRQ_FIRST + 54)   /* 34: I2C2 error interrupt */
#define STM32_IRQ_SPI1         (STM32_IRQ_FIRST + 55)   /* 35: SPI1 global interrupt */
#define STM32_IRQ_SPI2         (STM32_IRQ_FIRST + 56)   /* 36: SPI2 global interrupt */
#define STM32_IRQ_SPI3         (STM32_IRQ_FIRST + 57)   /* 36: SPI2 global interrupt */
#define STM32_IRQ_USART1       (STM32_IRQ_FIRST + 58)   /* 37: USART1 global interrupt */
#define STM32_IRQ_USART2       (STM32_IRQ_FIRST + 59)   /* 38: USART2 global interrupt */
#define STM32_IRQ_USART3       (STM32_IRQ_FIRST + 60)   /* 39: USART3 global interrupt */
#define STM32_IRQ_USART4       (STM32_IRQ_FIRST + 61)   /* 39: USART4 global interrupt */
#define STM32_IRQ_USART5       (STM32_IRQ_FIRST + 62)   /* 39: USART5 global interrupt */
#define STM32_IRQ_LPUART1      (STM32_IRQ_FIRST + 63)   /* 142: LPUART global interrupt */
#define STM32_IRQ_LPTIM1       (STM32_IRQ_FIRST + 64)   /* 93: LPTIM1 global interrupt */
#define STM32_IRQ_TIM8BRK      (STM32_IRQ_FIRST + 65)   /* 43: TIM8 break interrupt */
#define STM32_IRQ_TIM8UP       (STM32_IRQ_FIRST + 66)   /* 44: TIM8 update interrupt */
#define STM32_IRQ_TIM8TRGCOM   (STM32_IRQ_FIRST + 67)   /* 45: TIM8 trigger /commutation interrupt */
#define STM32_IRQ_TIM8CC       (STM32_IRQ_FIRST + 68)   /* 46: TIM8 capture / compare interrupts */
#define STM32_IRQ_ADC2         (STM32_IRQ_FIRST + 69)   /* 18: ADC2 global interrupt */
#define STM32_IRQ_LPTIM2       (STM32_IRQ_FIRST + 70)   /* 138: LPTIM2 timer interrupt */
#define STM32_IRQ_TIM15        (STM32_IRQ_FIRST + 71)   /* 116: TIM15 global interrupt */
#define STM32_IRQ_TIM16        (STM32_IRQ_FIRST + 72)   /* 117: TIM16 global interrupt */
#define STM32_IRQ_TIM17        (STM32_IRQ_FIRST + 73)   /* 118: TIM17 global interrupt */
#define STM32_IRQ_USBFS        (STM32_IRQ_FIRST + 74)   /* 74: USB FS global interrupt */
#define STM32_IRQ_CRS          (STM32_IRQ_FIRST + 75)   /* 144: Clock Recovery System global interrupt */
#define STM32_IRQ_UCPD1        (STM32_IRQ_FIRST + 76)   /* 144: Clock Recovery System global interrupt */
#define STM32_IRQ_FMC          (STM32_IRQ_FIRST + 77)   /* 48: FMC global interrupt */
#define STM32_IRQ_OCTOSPI1     (STM32_IRQ_FIRST + 78)   /* 48: FMC global interrupt */
#define STM32_IRQ_SDMMC1       (STM32_IRQ_FIRST + 79)   /* 49: SDMMC1 global interrupt */
#define STM32_IRQ_I2C3EV       (STM32_IRQ_FIRST + 80)   /* 72: I2C3 event interrupt */
#define STM32_IRQ_I2C3ER       (STM32_IRQ_FIRST + 81)   /* 73: I2C3 error interrupt*/
#define STM32_IRQ_SPI4         (STM32_IRQ_FIRST + 82)   /* 84: SPI4 global interrupt */
#define STM32_IRQ_SPI5         (STM32_IRQ_FIRST + 83)   /* 85: SPI5 global interrupt */
#define STM32_IRQ_SPI6         (STM32_IRQ_FIRST + 84)   /* 86: SPI6  global interrupt */
#define STM32_IRQ_USART6       (STM32_IRQ_FIRST + 85)   /* 71: USART6 global interrupt */
#define STM32_IRQ_USART10      (STM32_IRQ_FIRST + 86)   /* 71: USART6 global interrupt */
#define STM32_IRQ_USART11      (STM32_IRQ_FIRST + 87)   /* 71: USART6 global interrupt */
#define STM32_IRQ_SAI1         (STM32_IRQ_FIRST + 88)   /* 87: SAI1  global interrupt */
#define STM32_IRQ_SAI2         (STM32_IRQ_FIRST + 89)   /* 91: SAI2 global interrupt */
#define STM32_IRQ_DMA2S0       (STM32_IRQ_FIRST + 90)   /* 56: DMA2 Stream0 interrupt */
#define STM32_IRQ_DMA2S1       (STM32_IRQ_FIRST + 91)   /* 57: DMA2 Stream1 interrupt */
#define STM32_IRQ_DMA2S2       (STM32_IRQ_FIRST + 92)   /* 58: FMA2 Stream2 interrupt */
#define STM32_IRQ_DMA2S3       (STM32_IRQ_FIRST + 93)   /* 59: DMA2 Stream3 interrupt */
#define STM32_IRQ_DMA2S4       (STM32_IRQ_FIRST + 94)   /* 60: DMA2 Stream4 interrupt */
#define STM32_IRQ_DMA2S5       (STM32_IRQ_FIRST + 95)   /* 68: DMA2 Stream5 interrupt */
#define STM32_IRQ_DMA2S6       (STM32_IRQ_FIRST + 96)   /* 69: DMA2 Stream6 interrupt */
#define STM32_IRQ_DMA2S7       (STM32_IRQ_FIRST + 97)   /* 70: DMA2 Stream7 interrupt */
#define STM32_IRQ_UART7        (STM32_IRQ_FIRST + 98)   /* 82: UART7 global interrupt */
#define STM32_IRQ_UART8        (STM32_IRQ_FIRST + 99)   /* 83: UART8 global interrupt */
#define STM32_IRQ_UART9        (STM32_IRQ_FIRST + 100)  /* 83: UART8 global interrupt */
#define STM32_IRQ_UART12       (STM32_IRQ_FIRST + 101)  /* 83: UART8 global interrupt */
#define STM32_IRQ_SDMMC2       (STM32_IRQ_FIRST + 102)  /* 124: SDMMC2 global interrupt */
#define STM32_IRQ_FPU          (STM32_IRQ_FIRST + 103)  /* 81: CPU FPU */
#define STM32_IRQ_ICACHE       (STM32_IRQ_FIRST + 104)  /* 81: CPU FPU */
#define STM32_IRQ_DCACHE       (STM32_IRQ_FIRST + 105)  /* 81: CPU FPU */
#define STM32_IRQ_ETH          (STM32_IRQ_FIRST + 106)  /* 61: Ethernet global interrupt */
#define STM32_IRQ_ETHWKUP      (STM32_IRQ_FIRST + 107)  /* 62: Ethernet wakeup through EXTI line interrupt */
#define STM32_IRQ_DCMI         (STM32_IRQ_FIRST + 108)  /* 78: DCMI global interrupt */
#define STM32_IRQ_FDCAN2_0     (STM32_IRQ_FIRST + 109)  /* 20: FDCAN2 Interrupt 0 */
#define STM32_IRQ_FDCAN2_1     (STM32_IRQ_FIRST + 110)  /* 22: FDCAN2 Interrupt 1 */
#define STM32_IRQ_CORDIC       (STM32_IRQ_FIRST + 111)  /* 22: FDCAN2 Interrupt 1 */
#define STM32_IRQ_FMAC         (STM32_IRQ_FIRST + 112)  /* 80: RNG global interrupt */
#define STM32_IRQ_DTS          (STM32_IRQ_FIRST + 113)  /* 80: RNG global interrupt */
#define STM32_IRQ_RNG          (STM32_IRQ_FIRST + 114)  /* 80: RNG global interrupt */
#define STM32_IRQ_OTFDEC1      (STM32_IRQ_FIRST + 115)  /* 80: RNG global interrupt */
#define STM32_IRQ_AES          (STM32_IRQ_FIRST + 116)  /* 17: DMA1 Stream6 global interrupt */
#define STM32_IRQ_HASH         (STM32_IRQ_FIRST + 117)  /* 80: HASH global interrupt */
#define STM32_IRQ_PKA          (STM32_IRQ_FIRST + 118)  /* 80: HASH global interrupt */
#define STM32_IRQ_CEC          (STM32_IRQ_FIRST + 119)  /* 94: HDMI-CEC global interrupt */
#define STM32_IRQ_TIM12        (STM32_IRQ_FIRST + 120)  /* 43: TIM12 global interrupt */
#define STM32_IRQ_TIM13        (STM32_IRQ_FIRST + 121)  /* 44: TIM13 global interrupt */
#define STM32_IRQ_TIM14        (STM32_IRQ_FIRST + 122)  /* 45: TIM14 global interrupts */
#define STM32_IRQ_I3C1EV       (STM32_IRQ_FIRST + 123)  /* 95: I2C4 event interrupt */
#define STM32_IRQ_I3C1ER       (STM32_IRQ_FIRST + 124)  /* 96: I2C4 error interrupt */
#define STM32_IRQ_I2C4EV       (STM32_IRQ_FIRST + 125)  /* 95: I2C4 event interrupt */
#define STM32_IRQ_I2C4ER       (STM32_IRQ_FIRST + 126)  /* 96: I2C4 error interrupt */
#define STM32_IRQ_LPTIM3       (STM32_IRQ_FIRST + 127)  /* 139: LPTIM2 timer interrupt */
#define STM32_IRQ_LPTIM4       (STM32_IRQ_FIRST + 128)  /* 140: LPTIM2 timer interrupt */
#define STM32_IRQ_LPTIM5       (STM32_IRQ_FIRST + 129)  /* 141: LPTIM2 timer interrupt */
#define STM32_IRQ_LPTIM6       (STM32_IRQ_FIRST + 130)  /* 141: LPTIM2 timer interrupt */


#define STM32_IRQ_NEXTINTS     130
#define NR_IRQS                (STM32_IRQ_FIRST + STM32_IRQ_NEXTINTS)

#endif /* __ARCH_ARM_INCLUDE_STM32H5_STM32H5X3XX_IRQ_H */
