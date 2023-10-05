/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32h5x3xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_RCC_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* TODO: Complete comments */

#define STM32_RCC_CR_OFFSET             0x0000  /* Clock control register */
#define STM32_RCC_HSICFGR_OFFSET        0x0010  /* Clock control register */
#define STM32_RCC_CRRCR_OFFSET          0x0014
#define STM32_RCC_CSICFGR_OFFSET        0x0018
#define STM32_RCC_CFGR1_OFFSET          0x001c  /* Clock configuration register */
#define STM32_RCC_CFGR2_OFFSET          0x0020
#define STM32_RCC_PLL1CFGR_OFFSET       0x0028
#define STM32_RCC_PLL2CFGR_OFFSET       0x002c
#define STM32_RCC_PLL3CFGR_OFFSET       0x0030
#define STM32_RCC_PLL1DIVR_OFFSET       0x0034
#define STM32_RCC_PLL1FRACR_OFFSET      0x0038
#define STM32_RCC_PLL2DIVR_OFFSET       0x003c
#define STM32_RCC_PLL2FRACR_OFFSET      0x0040
#define STM32_RCC_PLL3DIVR_OFFSET       0x0044
#define STM32_RCC_PLL3FRACR_OFFSET      0x0048
#define STM32_RCC_CIER_OFFSET           0x0050  /* Clock Source Interrupt enable register */
#define STM32_RCC_CIFR_OFFSET           0x0054  /* Clock Source Interrupt Flag register */
#define STM32_RCC_CICR_OFFSET           0x0058  /* Clock Source Interrupt Clear register  */
#define STM32_RCC_AHB1RSTR_OFFSET       0x0060  /* AHB1 peripheral reset register */
#define STM32_RCC_AHB2RSTR_OFFSET       0x0064  /* AHB2 peripheral reset register */
#define STM32_RCC_AHB4RSTR_OFFSET       0x006c  /* AHB4 peripheral reset register */
#define STM32_RCC_APB1LRSTR_OFFSET      0x0074  /* APB1 L Peripheral reset register */
#define STM32_RCC_APB1HRSTR_OFFSET      0x0078  /* APB1 H Peripheral reset register */
#define STM32_RCC_APB2RSTR_OFFSET       0x007c  /* APB2 Peripheral reset register */
#define STM32_RCC_APB3RSTR_OFFSET       0x0080  /* APB3 Peripheral reset register */
#define STM32_RCC_AHB1ENR_OFFSET        0x0088  /* AHB1 Peripheral Clock enable register */
#define STM32_RCC_AHB2ENR_OFFSET        0x008c  /* AHB2 Peripheral Clock enable register */
#define STM32_RCC_AHB4ENR_OFFSET        0x0094  /* AHB4 Peripheral Clock enable register */
#define STM32_RCC_APB1LENR_OFFSET       0x009c  /* APB1 L Peripheral Clock enable register */
#define STM32_RCC_APB1HENR_OFFSET       0x00a0  /* APB1 H Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET        0x00a4  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB3ENR_OFFSET        0x00a8  /* APB3 Peripheral Clock enable register */
#define STM32_RCC_AHB1LPENR_OFFSET      0x00b0  /* RCC AHB1 low power mode peripheral clock enable register */
#define STM32_RCC_AHB2LPENR_OFFSET      0x00b4  /* RCC AHB2 low power mode peripheral clock enable register */
#define STM32_RCC_AHB4LPENR_OFFSET      0x00bc  /* RCC AHB4 low power mode peripheral clock enable register */
#define STM32_RCC_APB1LLPENR_OFFSET     0x00c4  /* RCC APB1 L low power mode peripheral clock enable register */
#define STM32_RCC_APB1HLPENR_OFFSET     0x00c8  /* RCC APB1 H low power mode peripheral clock enable register */
#define STM32_RCC_APB2LPENR_OFFSET      0x00cc  /* RCC APB2 low power mode peripheral clock enable register */
#define STM32_RCC_APB3LPENR_OFFSET      0x00d0  /* RCC APB3 low power mode peripheral clock enable register */
#define STM32_RCC_CCIPR1_OFFSET         0x00d8  
#define STM32_RCC_CCIPR2_OFFSET         0x00dc  
#define STM32_RCC_CCIPR3_OFFSET         0x00e0  
#define STM32_RCC_CCIPR4_OFFSET         0x00e4  
#define STM32_RCC_CCIPR5_OFFSET         0x00e8  
#define STM32_RCC_BDCR_OFFSET           0x00f0  /* Backup Domain Control register */
#define STM32_RCC_RSR_OFFSET            0x00f4  /* RCC Reset Status register */
#define STM32_RCC_SECCFGR_OFFSET        0x0110  
#define STM32_RCC_PRIVCFGR_OFFSET       0x0114  
/* Register Addresses **********
 *
 * *********************************************/

#define STM32_RCC_CR                    (STM32_RCC_BASE + STM32_RCC_CR_OFFSET)
#define STM32_RCC_HSICFGR               (STM32_RCC_BASE + STM32_RCC_HSICFGR_OFFSET)
#define STM32_RCC_CRRCR                 (STM32_RCC_BASE + STM32_RCC_CRRCR_OFFSET)
#define STM32_RCC_CFGR1                 (STM32_RCC_BASE + STM32_RCC_CFGR1_OFFSET)
#define STM32_RCC_CFGR2                 (STM32_RCC_BASE + STM32_RCC_CFGR2_OFFSET)
#define STM32_RCC_PLL1CFGR              (STM32_RCC_BASE + STM32_RCC_PLL1CFGR_OFFSET)
#define STM32_RCC_PLL2CFGR              (STM32_RCC_BASE + STM32_RCC_PLL2CFGR_OFFSET)
#define STM32_RCC_PLL3CFGR              (STM32_RCC_BASE + STM32_RCC_PLL3CFGR_OFFSET)
#define STM32_RCC_PLL1DIVR              (STM32_RCC_BASE + STM32_RCC_PLL1DIVR_OFFSET)
#define STM32_RCC_PLL1FRACR             (STM32_RCC_BASE + STM32_RCC_PLL1FRACR_OFFSET)
#define STM32_RCC_PLL2DIVR              (STM32_RCC_BASE + STM32_RCC_PLL2DIVR_OFFSET)
#define STM32_RCC_PLL2FRACR             (STM32_RCC_BASE + STM32_RCC_PLL2FRACR_OFFSET)
#define STM32_RCC_PLL3DIVR              (STM32_RCC_BASE + STM32_RCC_PLL3DIVR_OFFSET)
#define STM32_RCC_PLL3FRACR             (STM32_RCC_BASE + STM32_RCC_PLL3FRACR_OFFSET)
#define STM32_RCC_CIER                  (STM32_RCC_BASE + STM32_RCC_CIER_OFFSET)
#define STM32_RCC_CIFR                  (STM32_RCC_BASE + STM32_RCC_CIFR_OFFSET)
#define STM32_RCC_CICR                  (STM32_RCC_BASE + STM32_RCC_CICR_OFFSET)
#define STM32_RCC_AHB1RSTR              (STM32_RCC_BASE + STM32_RCC_AHB1RSTR_OFFSET)
#define STM32_RCC_AHB2RSTR              (STM32_RCC_BASE + STM32_RCC_AHB2RSTR_OFFSET)
#define STM32_RCC_AHB4RSTR              (STM32_RCC_BASE + STM32_RCC_AHB4RSTR_OFFSET)
#define STM32_RCC_APB1LRSTR             (STM32_RCC_BASE + STM32_RCC_APB1LRSTR_OFFSET)
#define STM32_RCC_APB1HRSTR             (STM32_RCC_BASE + STM32_RCC_APB1HRSTR_OFFSET)
#define STM32_RCC_APB2RSTR              (STM32_RCC_BASE + STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB3RSTR              (STM32_RCC_BASE + STM32_RCC_APB3RSTR_OFFSET)
#define STM32_RCC_AHB1ENR               (STM32_RCC_BASE + STM32_RCC_AHB1ENR_OFFSET)
#define STM32_RCC_AHB2ENR               (STM32_RCC_BASE + STM32_RCC_AHB2ENR_OFFSET)
#define STM32_RCC_AHB4ENR               (STM32_RCC_BASE + STM32_RCC_AHB4ENR_OFFSET)
#define STM32_RCC_APB1LENR              (STM32_RCC_BASE + STM32_RCC_APB1LENR_OFFSET)
#define STM32_RCC_APB1HENR              (STM32_RCC_BASE + STM32_RCC_APB1HENR_OFFSET)
#define STM32_RCC_APB2ENR               (STM32_RCC_BASE + STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB3ENR               (STM32_RCC_BASE + STM32_RCC_APB3ENR_OFFSET)
#define STM32_RCC_AHB1LPENR             (STM32_RCC_BASE + STM32_RCC_AHB1LPENR_OFFSET)
#define STM32_RCC_AHB2LPENR             (STM32_RCC_BASE + STM32_RCC_AHB2LPENR_OFFSET)
#define STM32_RCC_AHB4LPENR             (STM32_RCC_BASE + STM32_RCC_AHB4LPENR_OFFSET)
#define STM32_RCC_APB1LLPENR            (STM32_RCC_BASE + STM32_RCC_APB1LLPENR_OFFSET)
#define STM32_RCC_APB1HLPENR            (STM32_RCC_BASE + STM32_RCC_APB1HLPENR_OFFSET)
#define STM32_RCC_APB2LPENR             (STM32_RCC_BASE + STM32_RCC_APB2LPENR_OFFSET)
#define STM32_RCC_APB3LPENR             (STM32_RCC_BASE + STM32_RCC_APB3LPENR_OFFSET)
#define STM32_RCC_CCIPR1                (STM32_RCC_BASE + STM32_RCC_CCIPR1_OFFSET)
#define STM32_RCC_CCIPR2                (STM32_RCC_BASE + STM32_RCC_CCIPR2_OFFSET)
#define STM32_RCC_CCIPR3                (STM32_RCC_BASE + STM32_RCC_CCIPR3_OFFSET)
#define STM32_RCC_CCIPR4                (STM32_RCC_BASE + STM32_RCC_CCIPR4_OFFSET)
#define STM32_RCC_CCIPR5                (STM32_RCC_BASE + STM32_RCC_CCIPR5_OFFSET)
#define STM32_RCC_BDCR                  (STM32_RCC_BASE + STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_RSR                   (STM32_RCC_BASE + STM32_RCC_RSR_OFFSET)
#define STM32_RCC_SECCFGR               (STM32_RCC_BASE + STM32_RCC_SECCFGR_OFFSET)
#define STM32_RCC_PRIVCFGR              (STM32_RCC_BASE + STM32_RCC_PRIVCFGR_OFFSET)


/* Register Bitfield Definitions ********************************************/

/* Source Control Register */

#define RCC_CR_HSION                    (1 << 0)   /* Bit 0: Internal High Speed clock enable */
#define RCC_CR_HSIRDY                   (1 << 1)   /* Bit 1: Internal High Speed clock ready flag */
#define RCC_CR_HSIKERON                 (1 << 2)   /* Bit 2: Internal High Speed clock enable for some IPs Kernel ?? */
#define RCC_CR_HSIDIV_SHIFT             (3)        /* Bits 3-4: HSI clock divider */
#define RCC_CR_HSIDIV_MASK              (3 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_1               (0 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_2               (1 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_4               (2 << RCC_CR_HSIDIV_SHIFT)
#  define RCC_CR_HSIDIV_8               (3 << RCC_CR_HSIDIV_SHIFT)
#define RCC_CR_HSIDIVF                  (1 << 5)   /* Bit 5: HSI Divider flag */
                                                   /* Bit 6: Reserved */
#define RCC_CR_CSION                    (1 << 8)   /* Bit 7: The Internal RC 4MHz oscillator clock enable */
#define RCC_CR_CSIRDY                   (1 << 9)   /* Bit 8: The Internal RC 4MHz oscillator clock ready */
#define RCC_CR_CSIKERON                 (1 << 10)   /* Bit 9: Internal RC 4MHz oscillator clock enable for some IPs Kernel */
                                                   /* Bits 10-11: Reserved */
#define RCC_CR_HSI48ON                  (1 << 12)  /* Bit 12: HSI48 clock enable clock enable */
#define RCC_CR_HSI48RDY                 (1 << 13)  /* Bit 13: HSI48 clock ready */

#define RCC_CR_HSEON                    (1 << 16)  /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY                   (1 << 17)  /* Bit 17: External High Speed clock ready */
#define RCC_CR_HSEBYP                   (1 << 18)  /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSHSEON                 (1 << 19)  /* Bit 19: HSE Clock security System enable */
#define RCC_CR_HSEEXT                   (1 << 20)  /* Bit 19: HSE Clock security System enable */
                                                   /* Bits 20-23: Reserved */
#define RCC_CR_PLL1ON                   (1 << 24)  /* Bit 24: System PLL1 clock enable */
#define RCC_CR_PLL1RDY                  (1 << 25)  /* Bit 25: System PLL1 clock ready */
#define RCC_CR_PLL2ON                   (1 << 26)  /* Bit 26: System PLL2 clock enable */
#define RCC_CR_PLL2RDY                  (1 << 27)  /* Bit 27: System PLL2 clock ready */
#define RCC_CR_PLL3ON                   (1 << 28)  /* Bit 28: System PLL3 clock enable */
#define RCC_CR_PLL3RDY                  (1 << 29)  /* Bit 29: System PLL3 clock ready */
                                                   /* Bits 30-31: Reserved */

/* Internal Clock Source Calibration Register */

/* HSICAL configuration */

#define RCC_HSICFGR_HSICAL_SHIFT          (0ul)
#define RCC_HSICFGR_HSICAL_MASK           (0xFFFul << RCC_HSICFGR_HSICAL_SHIFT)
#define RCC_HSICFGR_HSICAL                RCC_HSICFGR_HSICAL_MASK  /* HSICAL[11:0] bits */

/* HSITRIM configuration */

#define RCC_HSICFGR_HSITRIM_SHIFT         (12ul)
#define RCC_HSICFGR_HSITRIM_MASK          (0x3Ful << RCC_HSICFGR_HSITRIM_SHIFT)
#define RCC_HSICFGR_HSITRIM               RCC_HSICFGR_HSITRIM_MASK /* HSITRIM[5:0] bits */

/* CSICAL configuration */

#define RCC_CSICFGR_CSICAL_SHIFT          (18ul)
#define RCC_CSICFGR_CSICAL_MASK           (0xFFul << RCC_CSICFGR_CSICAL_SHIFT)
#define RCC_CSICFGR_CSICAL                RCC_CSICFGR_CSICAL_MASK  /* CSICAL[7:0] bits */

/* CSITRIM configuration */

#define RCC_CSICFGR_CSITRIM_SHIFT         (26ul)
#define RCC_CSICFGR_CSITRIM_MASK          (0x1Ful << RCC_CSICFGR_CSITRIM_SHIFT)
#define RCC_CSICFGR_CSITRIM               RCC_CSICFGR_CSITRIM_MASK /* CSITRIM[4:0] bits */

/* Clock Recovery RC Register */

/* HSI48CAL configuration */

#define RCC_CRRCR_HSI48CAL_SHIFT        (0ul)
#define RCC_CRRCR_HSI48CAL_MASK         (0x3FFul << RCC_CRRCR_HSI48CAL_SHIFT)
#define RCC_CRRCR_HSI48CAL               RCC_CRRCR_HSI48CAL_MASK   /* HSI48CAL[9:0] bits */

/* Clock Configuration Register (CFGR) */

#define RCC_CFGR1_SW_SHIFT               (0)                        /* Bits 0-2: System clock Switch */
#define RCC_CFGR1_SW_MASK                (3 << RCC_CFGR1_SW_SHIFT)
#  define RCC_CFGR1_SW_HSI               (0 << RCC_CFGR1_SW_SHIFT)   /* 000: HSI selection as system clock */
#  define RCC_CFGR1_SW_CSI               (1 << RCC_CFGR1_SW_SHIFT)   /* 001: CSI selection as system clock */
#  define RCC_CFGR1_SW_HSE               (2 << RCC_CFGR1_SW_SHIFT)   /* 010: HSE selection as system clock */
#  define RCC_CFGR1_SW_PLL1              (3 << RCC_CFGR1_SW_SHIFT)   /* 011: PLL1 selection as system clock */
#define RCC_CFGR1_SWS_SHIFT              (3)                        /* Bits 3-5: System Clock Switch Status */
#define RCC_CFGR1_SWS_MASK               (3 << RCC_CFGR1_SWS_SHIFT)
#  define RCC_CFGR1_SWS_HSI              (0 << RCC_CFGR1_SWS_SHIFT)  /* 000: HSI used as system clock */
#  define RCC_CFGR1_SWS_CSI              (1 << RCC_CFGR1_SWS_SHIFT)  /* 001: CSI used as system clock */
#  define RCC_CFGR1_SWS_HSE              (2 << RCC_CFGR1_SWS_SHIFT)  /* 010: HSE used as system clock */
#  define RCC_CFGR1_SWS_PLL1             (3 << RCC_CFGR1_SWS_SHIFT)  /* 011: PLL1 used as system clock */
#define RCC_CFGR1_STOPWUCK               (1 << 6)                   /* Bit 6: Wake Up from stop and CSS backup clock selection */
#define RCC_CFGR1_STOPKERWUCK            (1 << 7)                   /* Bit 7: Kernel Clock Selection after a Wake Up from STOP */
#define RCC_CFGR1_RTCPRE_SHIFT           (8)                        /* Bits 8-13: HSE division factor for RTC clock */
#define RCC_CFGR1_RTCPRE_MASK            (0x3f << RCC_CFGR1_RTCPRE_SHIFT)
#  define RCC_CFGR1_RTCPRE(x)            (((uint32_t)(x)) << RCC_CFGR1_RTCPRE_SHIFT)
#define RCC_CFGR1_TIMPRE                 (1 << 15)                  /* Timers clocks prescaler */
#define RCC_CFGR1_MCO1PRE_SHIFT          (18)                       /* Bits 18-21: MCO1 prescaler */
#define RCC_CFGR1_MCO1PRE_MASK           (0xf << RCC_CFGR1_MCO1PRE_SHIFT)
#  define RCC_CFGR1_MCO1PRE(x)           (((uint32_t)(x)) << 18)
#define RCC_CFGR1_MCO1_SHIFT             (22)                       /* Bits 22-24: Microcontroller Clock Output 1 */
#define RCC_CFGR1_MCO1_MASK              (7 << RCC_CFGR1_MCO1_SHIFT)
#  define RCC_CFGR1_MCO1_HSI             (0 << RCC_CFGR1_MCO1_SHIFT) /* 000: HSI clock selected */
#  define RCC_CFGR1_MCO1_LSE             (1 << RCC_CFGR1_MCO1_SHIFT) /* 001: LSE oscillator selected */
#  define RCC_CFGR1_MCO1_HSE             (2 << RCC_CFGR1_MCO1_SHIFT) /* 010: HSE oscillator clock selected */
#  define RCC_CFGR1_MCO1_PLL1Q           (3 << RCC_CFGR1_MCO1_SHIFT) /* 011: PLL clock selected */
#  define RCC_CFGR1_MCO1_HSI48           (4 << RCC_CFGR1_MCO1_SHIFT) /* 100: HSI48 clock selected */
#define RCC_CFGR1_MCO2PRE_SHIFT          (25)                       /* Bits 25-28: MCO2 prescaler */
#define RCC_CFGR1_MCO2PRE_MASK           (0xf << RCC_CFGR1_MCO2PRE_SHIFT)
#  define RCC_CFGR1_MCO2PRE(x)           (((uint32_t)(x)) << RCC_CFGR1_MCO2PRE_SHIFT)
#define RCC_CFGR1_MCO2_SHIFT             (29)                       /* Bits 29-31: Microcontroller Clock Output 2 */
#define RCC_CFGR1_MCO2_MASK              (7 << RCC_CFGR1_MCO2_SHIFT)
#  define RCC_CFGR1_MCO2_SYS             (0 << RCC_CFGR1_MCO2_SHIFT) /* 000: HSI clock selected */
#  define RCC_CFGR1_MCO2_PLL2P           (1 << RCC_CFGR1_MCO2_SHIFT) /* 001: PLL2 peripheral clock selected */
#  define RCC_CFGR1_MCO2_HSE             (2 << RCC_CFGR1_MCO2_SHIFT) /* 010: HSE oscillator clock selected */
#  define RCC_CFGR1_MCO2_PLL1P           (3 << RCC_CFGR1_MCO2_SHIFT) /* 011: PLL1 peripheral clock selected */
#  define RCC_CFGR1_MCO2_CSI             (4 << RCC_CFGR1_MCO2_SHIFT) /* 100: CSI clock selected */
#  define RCC_CFGR1_MCO2_LSI             (5 << RCC_CFGR1_MCO2_SHIFT) /* 101: LSI clock selected */

/* Bit definitions for RCC_CFGR2 */

#define RCC_CFGR2_HPRE_SHIFT           (0)  /* Bits 0-3: AHB prescaler */
#define RCC_CFGR2_HPRE_MASK            (15 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLK        (0 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd2      (8 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd4      (9 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd8      (10 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd16     (11 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd64     (12 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd128    (13 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd256    (14 << RCC_CFGR2_HPRE_SHIFT)
#  define RCC_CFGR2_HPRE_SYSCLKd512    (15 << RCC_CFGR2_HPRE_SHIFT)

#define RCC_CFGR2_PPRE1_SHIFT         (4)  /* Bits 4-6: APB1 prescaler */
#define RCC_CFGR2_PPRE1_MASK          (7 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLK        (0 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLKd2      (4 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLKd4      (5 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLKd8      (6 << RCC_CFGR2_PPRE1_SHIFT)
#  define RCC_CFGR2_PPRE1_HCLKd16     (7 << RCC_CFGR2_PPRE1_SHIFT)
                                             /* Bit 7: Reserved */

#define RCC_CFGR2_PPRE2_SHIFT         (8)  /* Bits 8-10: APB2 Core prescaler */
#define RCC_CFGR2_PPRE2_MASK          (7 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLK        (0 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLKd2      (4 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLKd4      (5 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLKd8      (6 << RCC_CFGR2_PPRE2_SHIFT)
#  define RCC_CFGR2_PPRE2_HCLKd16     (7 << RCC_CFGR2_PPRE2_SHIFT)

#define RCC_CFGR2_PPRE3_SHIFT         (12)  /* Bits 12-14: APB3 Core prescaler */
#define RCC_CFGR2_PPRE3_MASK          (7 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLK        (0 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLKd2      (4 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLKd4      (5 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLKd8      (6 << RCC_CFGR2_PPRE3_SHIFT)
#  define RCC_CFGR2_PPRE3_HCLKd16     (7 << RCC_CFGR2_PPRE3_SHIFT)

#define RCC_CFGR2_AHB1DIS             (1 << 16)  /* Bits 16: AHB1 Disable */
#define RCC_CFGR2_AHB2DIS             (1 << 17)  /* Bits 17: AHB2 Disable */
#define RCC_CFGR2_AHB4DIS             (1 << 19)  /* Bits 19: AHB4 Disable */

#define RCC_CFGR2_APB1DIS             (1 << 20)  /* Bits 20: APB1 Disable */
#define RCC_CFGR2_APB2DIS             (1 << 21)  /* Bits 21: APB2 Disable */
#define RCC_CFGR2_APB3DIS             (1 << 22)  /* Bits 22: APB3 Disable */


/*  Bit definition for RCCPLL1CFGR register */

#define RCC_PLL1CFGR_RESET               ((uint32_t)0x00070000)

#define RCC_PLL1CFGR_PLL1RC_SHIFT      (0)  /* Bit 0: */
#define RCC_PLL1CFGR_PLL1RC_MASK       (0x3 << RCC_PLL1CFGR_PLL1RC_SHIFT)
#define RCC_PLL1CFGR_PLL1RC            RCC_PLL1CFGR_PLL1RC_MASK
# define RCC_PLL1CFGR_PLL1RC_NONE      ((uint32_t)0x00000000)  /* No source clock selected */
# define RCC_PLL1CFGR_PLL1RC_HSI       ((uint32_t)0x00000001)  /* HSI source clock selected */
# define RCC_PLL1CFGR_PLL1RC_CSI       ((uint32_t)0x00000002)  /* CSI source clock selected */
# define RCC_PLL1CFGR_PLL1RC_HSE       ((uint32_t)0x00000003)  /* HSE source clock selected */

#define RCC_PLL1CFGR_PLL1RGE_SHIFT       (2ul)
#define RCC_PLL1CFGR_PLL1RGE_MASK        (0x3ul << RCC_PLL1CFGR_PLL1RGE_SHIFT)
#define RCC_PLL1CFGR_PLL1RGE             RCC_PLL1CFGR_PLL1RGE_MASK
# define RCC_PLL1CFGR_PLL1RGE_1_2_MHZ    (0x0ul << RCC_PLL1CFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
# define RCC_PLL1CFGR_PLL1RGE_2_4_MHZ    (0x1ul << RCC_PLL1CFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
# define RCC_PLL1CFGR_PLL1RGE_4_8_MHZ    (0x2ul << RCC_PLL1CFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
# define RCC_PLL1CFGR_PLL1RGE_8_16_MHZ   (0x3ul << RCC_PLL1CFGR_PLL1RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLL1CFGR_PLL1FRACEN          (1 << 4) /* Fractional latch enable */
#define RCC_PLL1CFGR_PLL1VCOSEL          (1 << 5)

#define RCC_PLL1CFGR_PLL1M_SHIFT       (8ul)
#define RCC_PLL1CFGR_PLL1M_MASK        (0x3Ful << RCC_PLL1CFGR_PLL1M_SHIFT)
#define RCC_PLL1CFGR_PLL1M             RCC_PLL1CFGR_PLL1M_MASK

#define RCC_PLL1CFGR_PLL1PEN           (1 << 16)
#define RCC_PLL1CFGR_PLL1QEN           (1 << 17)
#define RCC_PLL1CFGR_PLL1REN           (1 << 18)




/*  Bit definition for RCCPLL2CFGR register */

/* #define RCCPLL2CFGR_RESET               ((uint32_t)0x01FF0000) */

#define RCC_PLL2CFGR_PLL2SRC_SHIFT      (0)  /* Bit 0: */
#define RCC_PLL2CFGR_PLL2SRC_MASK       (0x3 << RCC_PLL2CFGR_PLL2SRC_SHIFT)
#define RCC_PLL2CFGR_PLL2SRC            RCC_PLL2CFGR_PLL2SRC_MASK
# define RCC_PLL2CFGR_PLL2SRC_NONE      ((uint32_t)0x00000000)  /* No source clock selected */
# define RCC_PLL2CFGR_PLL2SRC_HSI       ((uint32_t)0x00000001)  /* HSI source clock selected */
# define RCC_PLL2CFGR_PLL2SRC_CSI       ((uint32_t)0x00000002)  /* CSI source clock selected */
# define RCC_PLL2CFGR_PLL2SRC_HSE       ((uint32_t)0x00000003)  /* HSE source clock selected */

#define RCC_PLL2CFGR_PLL2RGE_SHIFT       (2ul)
#define RCC_PLL2CFGR_PLL2RGE_MASK        (0x3ul << RCC_PLL2CFGR_PLL2RGE_SHIFT)
#define RCC_PLL2CFGR_PLL2RGE             RCC_PLL2CFGR_PLL2RGE_MASK
# define RCC_PLL2CFGR_PLL2RGE_1_2_MHZ    (0x0ul << RCC_PLL2CFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
# define RCC_PLL2CFGR_PLL2RGE_2_4_MHZ    (0x1ul << RCC_PLL2CFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
# define RCC_PLL2CFGR_PLL2RGE_4_8_MHZ    (0x2ul << RCC_PLL2CFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
# define RCC_PLL2CFGR_PLL2RGE_8_16_MHZ   (0x3ul << RCC_PLL2CFGR_PLL2RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLL2CFGR_PLL2FRACEN          (1 << 4) /* Fractional latch enable */
#define RCC_PLL2CFGR_PLL2VCOSEL          (1 << 5)

#define RCC_PLL2CFGR_PLL2M_SHIFT       (8ul)
#define RCC_PLL2CFGR_PLL2M_MASK        (0x3Ful << RCC_PLL2CFGR_PLL2M_SHIFT)
#define RCC_PLL2CFGR_PLL2M             RCC_PLL2CFGR_PLL2M_MASK

#define RCC_PLL2CFGR_PLL2PEN           (1 << 16)
#define RCC_PLL2CFGR_PLL2QEN           (1 << 17)
#define RCC_PLL2CFGR_PLL2REN           (1 << 18)



/*  Bit definition for RCCPLL3CFGR register */

/* #define RCCPLL3CFGR_RESET               ((uint32_t)0x01FF0000) */

#define RCC_PLL3CFGR_PLL3SRC_SHIFT      (0)  /* Bit 0: */
#define RCC_PLL3CFGR_PLL3SRC_MASK       (0x3 << RCC_PLL3CFGR_PLL3SRC_SHIFT)
#define RCC_PLL3CFGR_PLL3SRC            RCC_PLL3CFGR_PLL3SRC_MASK
# define RCC_PLL3CFGR_PLL3SRC_NONE      ((uint32_t)0x00000000)  /* No source clock selected */
# define RCC_PLL3CFGR_PLL3SRC_HSI       ((uint32_t)0x00000001)  /* HSI source clock selected */
# define RCC_PLL3CFGR_PLL3SRC_CSI       ((uint32_t)0x00000002)  /* CSI source clock selected */
# define RCC_PLL3CFGR_PLL3SRC_HSE       ((uint32_t)0x00000003)  /* HSE source clock selected */

#define RCC_PLL3CFGR_PLL3RGE_SHIFT       (2ul)
#define RCC_PLL3CFGR_PLL3RGE_MASK        (0x3ul << RCC_PLL3CFGR_PLL3RGE_SHIFT)
#define RCC_PLL3CFGR_PLL3RGE             RCC_PLL3CFGR_PLL3RGE_MASK
# define RCC_PLL3CFGR_PLL3RGE_1_2_MHZ    (0x0ul << RCC_PLL3CFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 1 and 2 MHz */
# define RCC_PLL3CFGR_PLL3RGE_2_4_MHZ    (0x1ul << RCC_PLL3CFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 2 and 4 MHz */
# define RCC_PLL3CFGR_PLL3RGE_4_8_MHZ    (0x2ul << RCC_PLL3CFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 4 and 8 MHz */
# define RCC_PLL3CFGR_PLL3RGE_8_16_MHZ   (0x3ul << RCC_PLL3CFGR_PLL3RGE_SHIFT)  /* The PLL input clock range frequency is between 8 and 16 MHz */

#define RCC_PLL3CFGR_PLL3FRACEN          (1 << 4) /* Fractional latch enable */
#define RCC_PLL3CFGR_PLL3VCOSEL          (1 << 5)

#define RCC_PLL3CFGR_PLL3M_SHIFT       (8ul)
#define RCC_PLL3CFGR_PLL3M_MASK        (0x3Ful << RCC_PLL3CFGR_PLL3M_SHIFT)
#define RCC_PLL3CFGR_PLL3M             RCC_PLL3CFGR_PLL3M_MASK

#define RCC_PLL3CFGR_PLL3PEN           (1 << 16)
#define RCC_PLL3CFGR_PLL3QEN           (1 << 17)
#define RCC_PLL3CFGR_PLL3REN           (1 << 18)



/* Bit definitions for RCC_PLL1DIVR register */

#define RCC_PLL1DIVR_N_SHIFT           (0ul)
#define RCC_PLL1DIVR_N(x)              (((x) - 1) << RCC_PLL1DIVR_N_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL1DIVR_P_SHIFT           (9ul)
#define RCC_PLL1DIVR_P(x)              (((x) - 1) << RCC_PLL1DIVR_P_SHIFT)  /* DIVP division factor: 2 - 128, must be even */
#define RCC_PLL1DIVR_Q_SHIFT           (16ul)
#define RCC_PLL1DIVR_Q(x)              (((x) - 1) << RCC_PLL1DIVR_Q_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL1DIVR_R_SHIFT           (24ul)
#define RCC_PLL1DIVR_R(x)              (((x) - 1) << RCC_PLL1DIVR_R_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL1FRACR register */

#define RCC_PLL1FRACR_FRACN_SHIFT      (3ul)
#define RCC_PLL1FRACR_FRACN_MASK       (0x1FFFul << RCC_PLL1FRACR_FRACN_SHIFT)
#define RCC_PLL1FRACR_FRACN            RCC_PLL1FRACR_FRACN_MASK

/* Bit definitions for RCC_PLL2DIVR register */

#define RCC_PLL2DIVR_N_SHIFT           (0ul)
#define RCC_PLL2DIVR_N(x)              (((x) - 1) << RCC_PLL2DIVR_N_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL2DIVR_P_SHIFT           (9ul)
#define RCC_PLL2DIVR_P(x)              (((x) - 1) << RCC_PLL2DIVR_P_SHIFT)  /* DIVP division factor: 2 - 128 */
#define RCC_PLL2DIVR_Q_SHIFT           (16ul)
#define RCC_PLL2DIVR_Q(x)              (((x) - 1) << RCC_PLL2DIVR_Q_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL2DIVR_R_SHIFT           (24ul)
#define RCC_PLL2DIVR_R(x)              (((x) - 1) << RCC_PLL2DIVR_R_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL2FRACR register */

#define RCC_PLL2FRACR_FRACN_SHIFT      (3ul)
#define RCC_PLL2FRACR_FRACN_MASK       (0x1FFFul << RCC_PLL2FRACR_FRACN_SHIFT)
#define RCC_PLL2FRACR_FRACN             RCC_PLL2FRACR_FRACN_MASK

/* Bit definitions for RCC_PLL3DIVR register */

#define RCC_PLL3DIVR_N_SHIFT           (0ul)
#define RCC_PLL3DIVR_N(x)              (((x) - 1) << RCC_PLL3DIVR_N_SHIFT)  /* Multiplication factor for VCO: 4 - 512 */
#define RCC_PLL3DIVR_P_SHIFT           (9ul)
#define RCC_PLL3DIVR_P(x)              (((x) - 1) << RCC_PLL3DIVR_P_SHIFT)  /* DIVP division factor: 2 - 128 */
#define RCC_PLL3DIVR_Q_SHIFT           (16ul)
#define RCC_PLL3DIVR_Q(x)              (((x) - 1) << RCC_PLL3DIVR_Q_SHIFT)  /* DIVQ division factor: 2 - 128 */
#define RCC_PLL3DIVR_R_SHIFT           (24ul)
#define RCC_PLL3DIVR_R(x)              (((x) - 1) << RCC_PLL3DIVR_R_SHIFT)  /* DIVR division factor: 2 - 128 */

/* Bit definitions for RCC_PLL3FRACR register */

#define RCC_PLL3FRACR_FRACN_SHIFT      (3ul)
#define RCC_PLL3FRACR_FRACN_MASK       (0x1FFFul << RCC_PLL3FRACR_FRACN_SHIFT)
#define RCC_PLL3FRACR_FRACN            RCC_PLL3FRACR_FRACN_MASK


// HERE

/* Bit definitions for RCC_D1CCIPR register */

#define RCC_D1CCIPR_FMCSEL_SHIFT        (0)  /* Bits 0-1: */
#define RCC_D1CCIPR_FMCSEL_MASK         (3 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_HCLK       (0 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PLL1       (1 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PLL2       (2 << RCC_D1CCIPR_FMCSEL_SHIFT)
#  define RCC_D1CCIPR_FMCSEL_PER        (3 << RCC_D1CCIPR_FMCSEL_SHIFT)
                                             /* Bits 2-3: Reserved */
#define RCC_D1CCIPR_QSPISEL_SHIFT       (4)  /* Bits 4-5: */
#define RCC_D1CCIPR_QSPISEL_MASK        (3 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_HCLK      (0 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PLL1      (1 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PLL2      (2 << RCC_D1CCIPR_QSPISEL_SHIFT)
#  define RCC_D1CCIPR_QSPISEL_PER       (3 << RCC_D1CCIPR_QSPISEL_SHIFT)
                                             /* Bits 6-15: Reserved */
#define RCC_D1CCIPR_SDMMC_SHIFT         (16) /* Bit 16: */
#define RCC_D1CCIPR_SDMMC_MASK          (1 << RCC_D1CCIPR_SDMMC_SHIFT)
#  define RCC_D1CCIPR_SDMMC_PLL1        (0 << RCC_D1CCIPR_SDMMC_SHIFT)
#  define RCC_D1CCIPR_SDMMC_PLL2        (1 << RCC_D1CCIPR_SDMMC_SHIFT)
                                             /* Bits 17-27: Reserved */
#define RCC_D1CCIPR_CKPERSEL_SHIFT      (28) /* Bits 28-29: */
#define RCC_D1CCIPR_CKPERSEL_MASK       (3 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_HSI      (0 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_CSI      (1 << RCC_D1CCIPR_CKPERSEL_SHIFT)
#  define RCC_D1CCIPR_CKPERSEL_HSE      (2 << RCC_D1CCIPR_CKPERSEL_SHIFT)
                                             /* Bits 30-31: Reserved */

/* Bit definitions for RCC_D2CCIP1R register */

#define RCC_D2CCIP1R_SAI1SEL_SHIFT      (0)    /* Bits 0-2 */
#define RCC_D2CCIP1R_SAI1SEL_MASK       (7 << RCC_D2CCIP1R_SAI1SEL_MASK)
#  define RCC_D2CCIP1R_SAI1SEL_PLL1     (0 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PLL2     (1 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PLL3     (2 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_I2SCKIN  (3 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI1SEL_PER      (4 << RCC_D2CCIP1R_SAI1SEL_SHIFT)
                                               /* Bits 3-5: Reserved */
#define RCC_D2CCIP1R_SAI23SEL_SHIFT     (6)    /* Bits 6-8 */
#define RCC_D2CCIP1R_SAI23SEL_MASK      (7 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL1    (0 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL2    (1 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PLL3    (2 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_I2SCKIN (3 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
#  define RCC_D2CCIP1R_SAI23SEL_PER     (4 << RCC_D2CCIP1R_SAI23SEL_SHIFT)
                                               /* Bits 9-11: Reserved */
#define RCC_D2CCIP1R_SPI123SEL_SHIFT    (12)   /* Bits 12-14 */
#define RCC_D2CCIP1R_SPI123SEL_MASK     (7 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL1   (0 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL2   (1 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PLL3   (2 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_I2SCKIN (3 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI123SEL_PER    (4 << RCC_D2CCIP1R_SPI123SEL_SHIFT)
                                               /* Bit 15: Reserved */
#define RCC_D2CCIP1R_SPI45SEL_SHIFT     (16)   /* Bits 16-18 */
#define RCC_D2CCIP1R_SPI45SEL_MASK      (7 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_APB     (0 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_PLL2    (1 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_PLL3    (2 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_HSI     (3 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_CSI     (4 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
#  define RCC_D2CCIP1R_SPI45SEL_HSE     (5 << RCC_D2CCIP1R_SPI45SEL_SHIFT)
                                               /* Bit 19: Reserved */
#define RCC_D2CCIP1R_SPDIFSEL_SHIFT     (20)   /* Bits 20-21 */
#define RCC_D2CCIP1R_SPDIFSEL_MASK      (3 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL1    (0 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL2    (1 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_PLL3    (2 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
#  define RCC_D2CCIP1R_SPDIFSEL_HSI     (3 << RCC_D2CCIP1R_SPDIFSEL_SHIFT)
                                               /* Bits 22-23: Reserved */
#define RCC_D2CCIP1R_DFSDM1SEL_SHIFT    (24)   /* Bit 24 */
#define RCC_D2CCIP1R_DFSDM1SEL_MASK     (1 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
#  define RCC_D2CCIP1R_DFSDM1SEL_PCLK2  (0 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
#  define RCC_D2CCIP1R_DFSDM1SEL_SYSCLK (1 << RCC_D2CCIP1R_DFSDM1SEL_SHIFT)
                                               /* Bits 25-27: Reserved */
#define RCC_D2CCIP1R_FDCANSEL_SHIFT     (28)   /* Bits 28-29 */
#define RCC_D2CCIP1R_FDCANSEL_MASK      (3 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_HSE     (0 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_PLL1    (1 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
#  define RCC_D2CCIP1R_FDCANSEL_PLL2    (2 << RCC_D2CCIP1R_FDCANSEL_SHIFT)
                                               /* Bit 30: Reserved */
#define RCC_D2CCIP1R_SWPSEL_SHIFT       (31)   /* Bit 31 */
#define RCC_D2CCIP1R_SWPSEL_MASK        (1 << RCC_D2CCIP1R_SWPSEL_SHIFT)
#  define RCC_D2CCIP1R_SWPSEL_PCLK      (0 << RCC_D2CCIP1R_SWPSEL_SHIFT)
#  define RCC_D2CCIP1R_SWPSEL_HSI       (1 << RCC_D2CCIP1R_SWPSEL_SHIFT)

/* Bit definitions for RCC_D2CCIP2R register */

#define RCC_D2CCIP2R_USART234578SEL_SHIFT  (0)  /* Bits 0-2 */
#  define RCC_D2CCIP2R_USART234578SEL_MASK (7 << RCC_D2CCIP2R_USART234578SEL_SHIFT)
#define RCC_D2CCIP2R_USART16SEL_SHIFT      (3)  /* Bits 3-5 */
#  define RCC_D2CCIP2R_USART16SEL_MASK     (7 << RCC_D2CCIP2R_USART16SEL_SHIFT)
                                                /* Bits 6-7: Reserved */
#define RCC_D2CCIP2R_RNGSEL_SHIFT          (8)  /* Bits 8-9 */
#  define RCC_D2CCIP2R_RNGSEL_MASK         (3 << RCC_D2CCIP2R_RNGSEL_SHIFT)
                                                /* Bits 10-11: Reserved */
#define RCC_D2CCIP2R_I2C123SEL_SHIFT       (12) /* Bits 12-13 */
#define RCC_D2CCIP2R_I2C123SEL_MASK        (3 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_PCLK1    (0 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_PLL3     (1 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_HSI      (2 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
#   define RCC_D2CCIP2R_I2C123SEL_CSI      (3 << RCC_D2CCIP2R_I2C123SEL_SHIFT)
                                                /* Bits 14-19: Reserved */
#define RCC_D2CCIP2R_USBSEL_SHIFT          (20) /* Bits 20-21 */
#  define RCC_D2CCIP2R_USBSEL_MASK         (3 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_DISABLE      (0 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_PLL1         (1 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_PLL3         (2 << RCC_D2CCIP2R_USBSEL_SHIFT)
#  define RCC_D2CCIP2R_USBSEL_HSI48        (3 << RCC_D2CCIP2R_USBSEL_SHIFT)
#define RCC_D2CCIP2R_CECSEL_SHIFT          (22) /* Bits 22-23 */
#  define RCC_D2CCIP2R_CECSEL_MASK         (3 << RCC_D2CCIP2R_CECSEL_SHIFT)
                                                /* Bits 24-27: Reserved */
#define RCC_D2CCIP2R_LPTIM1SEL_SHIFT       (28) /* Bits 28-30 */
#  define RCC_D2CCIP2R_LPTIM1SEL_MASK      (3 << RCC_D2CCIP2R_LPTIM1SEL_SHIFT)
                                                /* Bit 31: Reserved */

/* Bit definitions for RCC_D3CCIPR register */

#define RCC_D3CCIPR_LPUART1SEL_SHIFT     (0)  /* Bits 0-2: LPUART1 kernel clock source selection */
#define RCC_D3CCIPR_LPUART1SEL_MASK      (7 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PCLK    (0 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PLL2    (1 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_PLL3    (2 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_HSI     (3 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_CSI     (4 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
#  define RCC_D3CCIPR_LPUART1SEL_LSE     (5 << RCC_D3CCIPR_LPUART1SEL_SHIFT)
                                              /* Bits 3-7: Reserved */
#define RCC_D3CCIPR_I2C4SEL_SHIFT        (8)  /* Bits 8-9: I2C4 kernel clock source selection */
#define RCC_D3CCIPR_I2C4SEL_MASK         (3 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_PCLK4     (0 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_PLL3      (1 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_HSI       (2 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#   define RCC_D3CCIPR_I2C4SEL_CSI       (3 << RCC_D3CCIPR_I2C4SEL_SHIFT)
#define RCC_D3CCIPR_LPTIM2SEL_SHIFT      (10) /* Bits 10-12: LPTIM2 kernel clock source selection */
#define RCC_D3CCIPR_LPTIM2SEL_MASK       (7 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PCLK4   (0 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PLL2    (1 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PLL3    (2 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_LSE     (3 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_LSI     (4 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM2SEL_PER     (5 << RCC_D3CCIPR_LPTIM2SEL_SHIFT)
#define RCC_D3CCIPR_LPTIM345SEL_SHIFT    (13) /* Bits 13-15: LPTIM3,4,5 kernel clock source selection */
#define RCC_D3CCIPR_LPTIM345SEL_MASK     (7 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PCLK4 (0 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PLL2  (1 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PLL3  (2 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_LSE   (3 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_LSI   (4 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#   define RCC_D3CCIPR_LPTIM345SEL_PER   (5 << RCC_D3CCIPR_LPTIM345SEL_SHIFT)
#define RCC_D3CCIPR_ADCSEL_SHIFT         (16) /* Bits 16-17: SAR ADC kernel clock source selection */
#define RCC_D3CCIPR_ADCSEL_MASK          (3 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PLL2       (0 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PLL3       (1 << RCC_D3CCIPR_ADCSEL_SHIFT)
#   define RCC_D3CCIPR_ADCSEL_PER        (2 << RCC_D3CCIPR_ADCSEL_SHIFT)
                                              /* Bits 18-20: Reserved */
#define RCC_D3CCIPR_SAI4ASEL_SHIFT       (21) /* Bits 21-23: Sub-Block A of SAI4 kernel clock source selection */
#define RCC_D3CCIPR_SAI4ASEL_MASK        (7 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL1     (0 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL2     (1 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PLL3     (2 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_I2CCKIN  (3 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#   define RCC_D3CCIPR_SAI4ASEL_PER      (4 << RCC_D3CCIPR_SAI4ASEL_SHIFT)
#define RCC_D3CCIPR_SAI4BSEL_SHIFT       (24) /* Bits 24-26: Sub-Block B of SAI4 kernel clock source selection */
#define RCC_D3CCIPR_SAI4BSEL_MASK        (7 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL1     (0 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL2     (1 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PLL3     (2 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_I2CCKIN  (3 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#   define RCC_D3CCIPR_SAI4BSEL_PER      (4 << RCC_D3CCIPR_SAI4BSEL_SHIFT)
#define RCC_D3CCIPR_SPI6SEL_SHIFT        (28) /* Bits 28-30: SPI6 kernel clock source selection */
#define RCC_D3CCIPR_SPI6SEL_MASK         (7 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PCLK4     (0 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PLL2      (1 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_PLL3      (2 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_HSI       (3 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_CSI       (4 << RCC_D3CCIPR_SPI6SEL_SHIFT)
#   define RCC_D3CCIPR_SPI6SEL_HSE       (5 << RCC_D3CCIPR_SPI6SEL_SHIFT)
                                              /* Bit 31: Reserved */

/* TODO: CIER */

/* TODO: CIFR */

/* TODO: CICR */

/* TODO: BDCR */

/* Bit definitions for RCC_CSR register */

#define RCC_BDCR_LSION                   (1 << 26)  /* RCC BDCR: LSION */
#define RCC_BDCR_LSIRDY                  (1 << 27)  /* RCC BDCR: LSIRDY */

/* AHB1 peripheral reset register */

#define RCC_AHB1RSTR_GDMA1RST            (1 << 0)  /* RCC AHB1RSTR: DMA1RST */
#define RCC_AHB1RSTR_GDMA2RST            (1 << 1)  /* RCC AHB1RSTR: DMA2RST */
#define RCC_AHB1RSTR_CRCRST              (1 << 12) /* RCC AHB1RSTR: CRCRST */
#define RCC_AHB1RSTR_FMACRST             (1 << 15)
#define RCC_AHB1RSTR_RAMCFGRST           (1 << 17) /* RCC AHB1RSTR: ETH1RST */
#define RCC_AHB1RSTR_ETH1RST             (1 << 19) /* RCC AHB1RSTR: ETH1RST */

/* AHB2 peripheral reset register */
#define RCC_AHB2RSTR_GPIOARST           (1 << 0)  /* RCC AHB2RSTR: GPIOARST */
#define RCC_AHB2RSTR_GPIOBRST           (1 << 1)  /* RCC AHB2RSTR: GPIOBRST */
#define RCC_AHB2RSTR_GPIOCRST           (1 << 2)  /* RCC AHB2RSTR: GPIOCRST */
#define RCC_AHB2RSTR_GPIODRST           (1 << 3)  /* RCC AHB2RSTR: GPIODRST */
#define RCC_AHB2RSTR_GPIOERST           (1 << 4)  /* RCC AHB2RSTR: GPIOERST */
#define RCC_AHB2RSTR_GPIOFRST           (1 << 5)  /* RCC AHB2RSTR: GPIOFRST */
#define RCC_AHB2RSTR_GPIOGRST           (1 << 6)  /* RCC AHB2RSTR: GPIOGRST */
#define RCC_AHB2RSTR_GPIOHRST           (1 << 7)  /* RCC AHB2RSTR: GPIOHRST */
#define RCC_AHB2RSTR_GPIOIRST           (1 << 8)  /* RCC AHB2RSTR: GPIOIRST */
#define RCC_AHB2RSTR_ADCRST             (1 << 10) /* RCC AHB2RSTR: ADCRST */
#define RCC_AHB2RSTR_DAC1RST            (1 << 11) /* RCC AHB2RSTR: DAC1RST */
#define RCC_AHB2RSTR_DCMIPSSIRST        (1 << 12) /* RCC AHB2RSTR: DCMIPSSIRST */
#define RCC_AHB2RSTR_AESRST             (1 << 16) /* RCC AHB2RSTR: ADCRST */
#define RCC_AHB2RSTR_HASHRST            (1 << 17) /* RCC AHB2RSTR: HASHRST */
#define RCC_AHB2RSTR_RNGRST             (1 << 18) /* RCC AHB2RSTR: RNGRST */
#define RCC_AHB2RSTR_PKARST             (1 << 19) /* RCC AHB2RSTR: PKARST */
#define RCC_AHB2RSTR_SAESRST            (1 << 20) /* RCC AHB2RSTR: SAESRST */


/* AHB4 peripheral reset register */

#define RCC_AHB4RSTR_OTFDEC1RST         (1 << 7)  /* RCC AHB4RSTR: OTFEC1RST */
#define RCC_AHB4RSTR_SDMMC1RST          (1 << 11) /* RCC AHB4RSTR: SDMMC1RST */
#define RCC_AHB4RSTR_SDMMC2RST          (1 << 12) /* RCC AHB4RSTR: SDMMC2RST */
#define RCC_AHB4RSTR_FMCRST             (1 << 16) /* RCC AHB4RSTR: FMCRST */
#define RCC_AHB4RSTR_OTOSPI1RST         (1 << 20) /* RCC AHB4RSTR: OTOSPI1RST */


/* APB1 peripheral low reset register */

#define RCC_APB1LRSTR_TIM2RST           (1 << 0)  /* RCC APB1LRSTR: TIM2RST */
#define RCC_APB1LRSTR_TIM3RST           (1 << 1)  /* RCC APB1LRSTR: TIM3RST */
#define RCC_APB1LRSTR_TIM4RST           (1 << 2)  /* RCC APB1LRSTR: TIM4RST */
#define RCC_APB1LRSTR_TIM5RST           (1 << 3)  /* RCC APB1LRSTR: TIM5RST */
#define RCC_APB1LRSTR_TIM6RST           (1 << 4)  /* RCC APB1LRSTR: TIM6RST */
#define RCC_APB1LRSTR_TIM7RST           (1 << 5)  /* RCC APB1LRSTR: TIM7RST */
#define RCC_APB1LRSTR_TIM12RST          (1 << 6)  /* RCC APB1LRSTR: TIM12RST */
#define RCC_APB1LRSTR_TIM13RST          (1 << 7)  /* RCC APB1LRSTR: TIM13RST */
#define RCC_APB1LRSTR_TIM14RST          (1 << 8)  /* RCC APB1LRSTR: TIM14RST */
#define RCC_APB1LRSTR_SPI2RST           (1 << 14) /* RCC APB1LRSTR: SPI2RST */
#define RCC_APB1LRSTR_SPI3RST           (1 << 15) /* RCC APB1LRSTR: SPI3RST */
#define RCC_APB1LRSTR_USART2RST         (1 << 17) /* RCC APB1LRSTR: USART2RST */
#define RCC_APB1LRSTR_USART3RST         (1 << 18) /* RCC APB1LRSTR: USART3RST */
#define RCC_APB1LRSTR_UART4RST          (1 << 19) /* RCC APB1LRSTR: UART4RST */
#define RCC_APB1LRSTR_UART5RST          (1 << 20) /* RCC APB1LRSTR: UART5RST */
#define RCC_APB1LRSTR_I2C1RST           (1 << 21) /* RCC APB1LRSTR: I2C1RST */
#define RCC_APB1LRSTR_I2C2RST           (1 << 22) /* RCC APB1LRSTR: I2C2RST */
#define RCC_APB1LRSTR_I3C1RST           (1 << 23) /* RCC APB1LRSTR: I3C1RST */
#define RCC_APB1LRSTR_CRSRST            (1 << 24) /* RCC APB1LRSTR: CRSRST */
#define RCC_APB1LRSTR_UART6RST          (1 << 25) /* RCC APB1LRSTR: UART6RST */
#define RCC_APB1LRSTR_UART10RST         (1 << 26) /* RCC APB1LRSTR: UART10RST */
#define RCC_APB1LRSTR_UART11RST         (1 << 27) /* RCC APB1LRSTR: UART11RST */
#define RCC_APB1LRSTR_CECRST            (1 << 28) /* RCC APB1LRSTR: CECRST */
#define RCC_APB1LRSTR_USART7RST         (1 << 30) /* RCC APB1LRSTR: USART7RST */
#define RCC_APB1LRSTR_USART8RST         (1 << 31) /* RCC APB1LRSTR: USART8RST */


/* APB1 peripheral high reset register */

#define RCC_APB1HRSTR_UART9RST          (1 << 0)  /* RCC APB1HRSTR: UART9RST */
#define RCC_APB1HRSTR_UART12RST         (1 << 1)  /* RCC APB1HRSTR: UART12RST */
#define RCC_APB1HRSTR_DTSRST            (1 << 3)  /* RCC APB1HRSTR: DTSRST */
#define RCC_APB1HRSTR_LPTIM2RST         (1 << 5)  /* RCC APB1HRSTR: LPTIM2RST */
#define RCC_APB1HRSTR_FDCANRST          (1 << 9)  /* RCC APB1HRSTR: FDCANRST*/
#define RCC_APB1HRSTR_UCPD1RST          (1 << 23) /* RCC APB1HRSTR: UCPD1RST*/


/* APB2 peripheral reset register */

#define RCC_APB2RSTR_TIM1RST            (1 << 11)  /* RCC APB2RSTR: TIM1RST */
#define RCC_APB2RSTR_SPI1RST            (1 << 12)  /* RCC APB2RSTR: SPI1RST */
#define RCC_APB2RSTR_TIM8RST            (1 << 13)  /* RCC APB2RSTR: TIM8RST */
#define RCC_APB2RSTR_USART1RST          (1 << 14)  /* RCC APB2RSTR: USART1RST */
#define RCC_APB2RSTR_TIM15RST           (1 << 16) /* RCC APB2RSTR: TIM15RST */
#define RCC_APB2RSTR_TIM16RST           (1 << 17) /* RCC APB2RSTR: TIM16RST */
#define RCC_APB2RSTR_TIM17RST           (1 << 18) /* RCC APB2RSTR: TIM17RST */
#define RCC_APB2RSTR_SPI4RST            (1 << 19) /* RCC APB2RSTR: SPI4RST */
#define RCC_APB2RSTR_SPI6RST            (1 << 20) /* RCC APB2RSTR: SPI6RST */
#define RCC_APB2RSTR_SAI1RST            (1 << 21) /* RCC APB2RSTR: SAI1RST */
#define RCC_APB2RSTR_SAI2RST            (1 << 22) /* RCC APB2RSTR: SAI2RST */
#define RCC_APB2RSTR_USBRST             (1 << 24) /* RCC APB2RSTR: USBRST */


/* APB3 peripheral reset register */

#define RCC_APB3RSTR_SPI5RST            (1 << 5) /* RCC APB3RSTR: SPI5RST */
#define RCC_APB3RSTR_LPUART1RST         (1 << 6) /* RCC APB3RSTR: LPUART1RST */
#define RCC_APB3RSTR_I2C3RST            (1 << 7) /* RCC APB3RSTR: I2C3RST */
#define RCC_APB3RSTR_I2C4RST            (1 << 8) /* RCC APB3RSTR: I2C4RST */
#define RCC_APB3RSTR_LPTIM1RST          (1 << 11) /* RCC APB3RSTR: LPTIM1RST */
#define RCC_APB3RSTR_LPTIM3RST          (1 << 12) /* RCC APB3RSTR: LPTIM3RST */
#define RCC_APB3RSTR_LPTIM4RST          (1 << 13) /* RCC APB3RSTR: LPTIM4RST */
#define RCC_APB3RSTR_LPTIM5RST          (1 << 14) /* RCC APB3RSTR: LPTIM5RST */
#define RCC_APB3RSTR_LPTIM6RST          (1 << 15) /* RCC APB3RSTR: LPTIM6RST */
#define RCC_APB3RSTR_VREFRST            (1 << 20) /* RCC APB3RSTR: VREFRST */

/* AHB1 Peripheral Clock enable register */

#define RCC_AHB1ENR_GDMA1EN             (1 << 0)  /* RCC AHB1ENR: GDMA1EN */
#define RCC_AHB1ENR_GDMA2EN             (1 << 1)  /* RCC AHB1ENR: GDMA2EN */
#define RCC_AHB1ENR_FLITFEN             (1 << 8)  /* RCC AHB1ENR: FLITFEN */
#define RCC_AHB1ENR_CRCEN               (1 << 12) /* RCC AHB1ENR: CRCEN */
#define RCC_AHB1ENR_CORDICEN            (1 << 14) /* RCC AHB1ENR: CORDICEN */
#define RCC_AHB1ENR_FMACEN              (1 << 15) /* RCC AHB1ENR: FMACEN */
#define RCC_AHB1ENR_RAMCFGEN            (1 << 17) /* RCC AHB1ENR: RAMCFGEN */
#define RCC_AHB1ENR_ETHMACEN            (1 << 19) /* RCC AHB1ENR: ETHMACEN */
#define RCC_AHB1ENR_ETHTXEN             (1 << 20) /* RCC AHB1ENR: ETHTXEN */
#define RCC_AHB1ENR_ETHRXEN             (1 << 21) /* RCC AHB1ENR: ETHRXEN */
#define RCC_AHB1ENR_TZSC1EN             (1 << 24) /* RCC AHB1ENR: TZSC1EN */
#define RCC_AHB1ENR_BKPRAMEN            (1 << 28) /* RCC AHB1ENR: BKPRAMEN */
#define RCC_AHB1ENR_DCACHEEN            (1 << 30) /* RCC AHB1ENR: DCACHEEN */
#define RCC_AHB1ENR_SRAM1EN             (1 << 31) /* RCC AHB1ENR: SRAM1EN */

/* AHB2 Peripheral Clock enable register */

#define RCC_AHB2ENR_GPIOAEN             (1 << 0)  /* RCC AHB2ENR: GPIOAEN */
#define RCC_AHB2ENR_GPIOBEN             (1 << 1)  /* RCC AHB2ENR: GPIOBEN */
#define RCC_AHB2ENR_GPIOCEN             (1 << 2)  /* RCC AHB2ENR: GPIOCEN */
#define RCC_AHB2ENR_GPIODEN             (1 << 3)  /* RCC AHB2ENR: GPIODEN */
#define RCC_AHB2ENR_GPIOEEN             (1 << 4)  /* RCC AHB2ENR: GPIOEEN */
#define RCC_AHB2ENR_GPIOFEN             (1 << 5)  /* RCC AHB2ENR: GPIOFEN */
#define RCC_AHB2ENR_GPIOGEN             (1 << 6)  /* RCC AHB2ENR: GPIOGEN */
#define RCC_AHB2ENR_GPIOHEN             (1 << 7)  /* RCC AHB2ENR: GPIOHEN */
#define RCC_AHB2ENR_GPIOIEN             (1 << 8)  /* RCC AHB2ENR: GPIOIEN */
#define RCC_AHB2ENR_ADCEN               (1 << 10) /* RCC AHB2ENR: ADCEN */
#define RCC_AHB2ENR_DAC1EN              (1 << 11) /* RCC AHB2ENR: DAC1EN */
#define RCC_AHB2ENR_DCMIPSSIEN          (1 << 12) /* RCC AHB2ENR: DCMIPSSIEN */
#define RCC_AHB2ENR_AESMEN              (1 << 16) /* RCC AHB2ENR: AESEN */
#define RCC_AHB2ENR_HASHEN              (1 << 17) /* RCC AHB2ENR: HASHEN */
#define RCC_AHB2ENR_RNGEN               (1 << 18) /* RCC AHB2ENR: RNGEN */
#define RCC_AHB2ENR_PKAEN               (1 << 19) /* RCC AHB2ENR: PKAEN */
#define RCC_AHB2ENR_SAESEN              (1 << 20) /* RCC AHB2ENR: SAESEN */
#define RCC_AHB2ENR_SRAM2EN             (1 << 30) /* RCC AHB2ENR: SRAM2EN */
#define RCC_AHB2ENR_SRAM3EN             (1 << 31) /* RCC AHB2ENR: SRAM3EN */

/* AHB4 Peripheral Clock enable register */

#define RCC_AHB4ENR_OTFDEC1EN           (1 << 0)  /* RCC AHB4ENR: OTFDEC1EN */
#define RCC_AHB4ENR_SDMMC1EN            (1 << 11) /* RCC AHB4ENR: SDMMC1EN */
#define RCC_AHB4ENR_SDMMC2EN            (1 << 12) /* RCC AHB4ENR: SDMMC1EN */
#define RCC_AHB4ENR_FMCEN               (1 << 16) /* RCC AHB4ENR: FMCEN */
#define RCC_AHB4ENR_OCTOSPI1EN          (1 << 20) /* RCC AHB4ENR: OCTOSPI1EN */


/* APB1 L Peripheral Clock enable register */

#define RCC_APB1LENR_TIM2EN             (1 << 0)  /* RCC APB1LENR: TIM2EN */
#define RCC_APB1LENR_TIM3EN             (1 << 1)  /* RCC APB1LENR: TIM3EN */
#define RCC_APB1LENR_TIM4EN             (1 << 2)  /* RCC APB1LENR: TIM4EN */
#define RCC_APB1LENR_TIM5EN             (1 << 3)  /* RCC APB1LENR: TIM5EN */
#define RCC_APB1LENR_TIM6EN             (1 << 4)  /* RCC APB1LENR: TIM6EN */
#define RCC_APB1LENR_TIM7EN             (1 << 5)  /* RCC APB1LENR: TIM7EN */
#define RCC_APB1LENR_TIM12EN            (1 << 6)  /* RCC APB1LENR: TIM12EN */
#define RCC_APB1LENR_TIM13EN            (1 << 7)  /* RCC APB1LENR: TIM13EN */
#define RCC_APB1LENR_TIM14EN            (1 << 8)  /* RCC APB1LENR: TIM14EN */
#define RCC_APB1LENR_WWDGEN             (1 << 11) /* RCC APB1LENR: WWDGEN */
#define RCC_APB1LENR_SPI2EN             (1 << 14) /* RCC APB1LENR: SPI2EN */
#define RCC_APB1LENR_SPI3EN             (1 << 15) /* RCC APB1LENR: SPI3EN */
#define RCC_APB1LENR_USART2EN           (1 << 17) /* RCC APB1LENR: USART2EN */
#define RCC_APB1LENR_USART3EN           (1 << 18) /* RCC APB1LENR: USART3EN */
#define RCC_APB1LENR_UART4EN            (1 << 19) /* RCC APB1LENR: UART4EN */
#define RCC_APB1LENR_UART5EN            (1 << 20) /* RCC APB1LENR: UART5EN */
#define RCC_APB1LENR_I2C1EN             (1 << 21) /* RCC APB1LENR: I2C1EN */
#define RCC_APB1LENR_I2C2EN             (1 << 22) /* RCC APB1LENR: I2C2EN */
#define RCC_APB1LENR_I3C1EN             (1 << 23) /* RCC APB1LENR: I3C1EN */
#define RCC_APB1LENR_CRSEN              (1 << 24) /* RCC APB1LENR: CRSEN */
#define RCC_APB1LENR_USART6EN           (1 << 25) /* RCC APB1LENR: USART6EN */
#define RCC_APB1LENR_USART10EN          (1 << 26) /* RCC APB1LENR: USART10EN */
#define RCC_APB1LENR_USART11EN          (1 << 27) /* RCC APB1LENR: USART11EN */
#define RCC_APB1LENR_CECEN              (1 << 28) /* RCC APB1LENR: CECEN */
#define RCC_APB1LENR_UART7EN            (1 << 30) /* RCC APB1LENR: UART7EN */
#define RCC_APB1LENR_UART8EN            (1 << 31) /* RCC APB1LENR: UART8EN */


/* APB1 H Peripheral Clock enable register */

#define RCC_APB1HENR_UART9EN            (1 << 0) /* RCC APB1HENR: UART9EN */
#define RCC_APB1HENR_UART12EN           (1 << 1) /* RCC APB1HENR: UART12EN */
#define RCC_APB1HENR_DTSEN              (1 << 3) /* RCC APB1HENR: DTSEN */
#define RCC_APB1HENR_LPTIM2EN           (1 << 5) /* RCC APB1HENR: LPTIM2EN */
#define RCC_APB1HENR_FDCANEN            (1 << 9) /* RCC APB1HENR: FDCANEN */
#define RCC_APB1HENR_UCPD1EN            (1 << 23)/* RCC APB1HENR: UCPD1EN */


/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_TIM1EN              (1 << 11) /* Bit 11: RCC APB2ENR: TIM1EN */
#define RCC_APB2ENR_SPI1EN              (1 << 12) /* Bit 12: RCC APB2ENR: SPI1EN */
#define RCC_APB2ENR_TIM8EN              (1 << 13) /* Bit 13: RCC APB2ENR: TIM8EN  */
#define RCC_APB2ENR_USART1EN            (1 << 14) /* Bit 14: RCC APB2ENR: USART1EN */
#define RCC_APB2ENR_TIM15EN             (1 << 16) /* Bit 16: RCC APB2ENR: TIM15EN */
#define RCC_APB2ENR_TIM16EN             (1 << 17) /* Bit 17: RCC APB2ENR: TIM16EN */
#define RCC_APB2ENR_TIM17EN             (1 << 18) /* Bit 18: RCC APB2ENR: TIM17EN  */
#define RCC_APB2ENR_SPI4EN              (1 << 19) /* Bit 19: RCC APB2ENR: SPI4EN */
#define RCC_APB2ENR_SPI6EN              (1 << 20) /* Bit 20: RCC APB2ENR: SPI6EN */
#define RCC_APB2ENR_SAI1EN              (1 << 21) /* Bit 21: RCC APB2ENR: SAI1EN */
#define RCC_APB2ENR_SAI2EN              (1 << 22) /* Bit 22: RCC APB2ENR: SAI2EN */
#define RCC_APB2ENR_USBEN               (1 << 24) /* Bit 24: RCC APB2ENR: USBEN */


/* APB3 Peripheral Clock enable register */

#define RCC_APB3ENR_SBSEN               (1 << 1)  /* RCC APB3ENR: SBSEN */
#define RCC_APB3ENR_SPI5EN              (1 << 5)  /* RCC APB3ENR: SPI5EN */
#define RCC_APB3ENR_LPUART1EN           (1 << 6)  /* RCC APB3ENR: LPUART1EN */
#define RCC_APB3ENR_I2C3EN              (1 << 7)  /* RCC APB3ENR: I2C3EN */
#define RCC_APB3ENR_I2C4EN              (1 << 8)  /* RCC APB3ENR: I2C4EN */
#define RCC_APB3ENR_LPTIM1EN            (1 << 11) /* RCC APB3ENR: LPTIM1EN */
#define RCC_APB3ENR_LPTIM3EN            (1 << 12) /* RCC APB3ENR: LPTIM3EN */
#define RCC_APB3ENR_LPTIM4EN            (1 << 13) /* RCC APB3ENR: LPTIM4EN */
#define RCC_APB3ENR_LPTIM5EN            (1 << 14) /* RCC APB3ENR: LPTIM5EN */
#define RCC_APB3ENR_LPTIM6EN            (1 << 15) /* RCC APB3ENR: LPTIM6EN */
#define RCC_APB3ENR_VREFBUFEN           (1 << 20) /* RCC APB3ENR: VREFBUFEN */
#define RCC_APB3ENR_RTCAPBEN            (1 << 21) /* RCC APB3ENR: RTCAPBEN */


//TODO Low power not supported yet.
/* AHB3 low power mode peripheral clock enable register */

#define RCC_AHB3LPENR_MDMALPEN          (1 << 0)  /* RCC AHB3LPENR: MDMALPEN */
#define RCC_AHB3LPENR_DMA2DLPEN         (1 << 4)  /* RCC AHB3LPENR: DMA2DLPEN */
#define RCC_AHB3LPENR_JPGDECLPEN        (1 << 5)  /* RCC AHB3LPENR: JPGDECLPEN */
#define RCC_AHB3LPENR_FLITFLPEN         (1 << 8)  /* RCC AHB3LPENR: FLITFLPEN */
#define RCC_AHB3LPENR_FMCLPEN           (1 << 12) /* RCC AHB3LPENR: FMCLPEN */
#define RCC_AHB3LPENR_QSPILPEN          (1 << 14) /* RCC AHB3LPENR: QSPILPEN */
#define RCC_AHB3LPENR_SDMMC1LPEN        (1 << 16) /* RCC AHB3LPENR: SDMMC1LPEN */
#define RCC_AHB3LPENR_D1DTCM1LPEN       (1 << 28) /* RCC AHB3LPENR: D1DTCM1LPEN */
#define RCC_AHB3LPENR_DTCM2LPEN         (1 << 29) /* RCC AHB3LPENR: DTCM2LPEN */
#define RCC_AHB3LPENR_ITCMLPEN          (1 << 30) /* RCC AHB3LPENR: ITCMLPEN */
#define RCC_AHB3LPENR_AXISRAMLPEN       (1 << 31) /* RCC AHB3LPENR: AXISRAMLPEN */

/* AHB1 low power mode peripheral clock enable register */

#define RCC_AHB1LPENR_DMA1LPEN          (1 << 0)  /* RCC AHB1LPENR: DMA1LPEN */
#define RCC_AHB1LPENR_DMA2LPEN          (1 << 1)  /* RCC AHB1LPENR: DMA2LPEN */
#define RCC_AHB1LPENR_ADC12LPEN         (1 << 5)  /* RCC AHB1LPENR: ADC12LPEN */
#define RCC_AHB1LPENR_ETH1MACLPEN       (1 << 15) /* RCC AHB1LPENR: ETH1MACLPEN */
#define RCC_AHB1LPENR_ETH1TXLPEN        (1 << 16) /* RCC AHB1LPENR: ETH1TXLPEN */
#define RCC_AHB1LPENR_ETH1RXLPEN        (1 << 17) /* RCC AHB1LPENR: ETH1RXLPEN */
#define RCC_AHB1LPENR_OTGHSLPEN         (1 << 25) /* RCC AHB1LPENR: OTGHSLPEN */
#define RCC_AHB1LPENR_OTGHSULPILPEN     (1 << 26) /* RCC AHB1LPENR: OTGHSULPILPEN */
#define RCC_AHB1LPENR_OTGFSLPEN         (1 << 27) /* RCC AHB1LPENR: OTGFSLPEN */

/* AHB2 low power mode peripheral clock enable register */

#define RCC_AHB2LPENR_CAMITFLPEN        (1 << 0)  /* RCC AHB2LPENR: CAMITFLPEN */
#define RCC_AHB2LPENR_CRYPTLPEN         (1 << 4)  /* RCC AHB2LPENR: CRYPTLPEN */
#define RCC_AHB2LPENR_HASHLPEN          (1 << 5)  /* RCC AHB2LPENR: HASHLPEN */
#define RCC_AHB2LPENR_SDMMC2LPEN        (1 << 6)  /* RCC AHB2LPENR: SDMMC2LPEN */
#define RCC_AHB2LPENR_RNGLPEN           (1 << 9)  /* RCC AHB2LPENR: RNGLPEN */
#define RCC_AHB2LPENR_SRAM1LPEN         (1 << 29) /* RCC AHB2LPENR: SRAM1LPEN */
#define RCC_AHB2LPENR_SRAM2LPEN         (1 << 30) /* RCC AHB2LPENR: SRAM2LPEN */
#define RCC_AHB2LPENR_SRAM3LPEN         (1 << 31) /* RCC AHB2LPENR: SRAM3LPEN */


/* APB3 low power mode peripheral clock enable register */

#define RCC_APB3LPENR_LTDCLPEN          (1 << 3)  /* RCC APB3LPENR: LTDCLPEN */
#define RCC_APB3LPENR_WWDG1LPEN         (1 << 6)  /* RCC APB3LPENR: WWDG1LPEN */

/* APB1 L low power mode peripheral clock enable register */

#define RCC_APB1LLPENR_TIM2LPEN         (1 << 0)  /* RCC APB1LLPENR: TIM2LPEN */
#define RCC_APB1LLPENR_TIM3LPEN         (1 << 1)  /* RCC APB1LLPENR: TIM3LPEN */
#define RCC_APB1LLPENR_TIM4LPEN         (1 << 2)  /* RCC APB1LLPENR: TIM4LPEN */
#define RCC_APB1LLPENR_TIM5LPEN         (1 << 3)  /* RCC APB1LLPENR: TIM5LPEN */
#define RCC_APB1LLPENR_TIM6LPEN         (1 << 4)  /* RCC APB1LLPENR: TIM6LPEN */
#define RCC_APB1LLPENR_TIM7LPEN         (1 << 5)  /* RCC APB1LLPENR: TIM7LPEN */
#define RCC_APB1LLPENR_TIM12LPEN        (1 << 6)  /* RCC APB1LLPENR: TIM12LPEN */
#define RCC_APB1LLPENR_TIM13LPEN        (1 << 7)  /* RCC APB1LLPENR: TIM13LPEN */
#define RCC_APB1LLPENR_TIM14LPEN        (1 << 8)  /* RCC APB1LLPENR: TIM14LPEN */
#define RCC_APB1LLPENR_LPTIM1LPEN       (1 << 9)  /* RCC APB1LLPENR: LPTIM1LPEN */
#define RCC_APB1LLPENR_SPI2LPEN         (1 << 14) /* RCC APB1LLPENR: SPI2LPEN */
#define RCC_APB1LLPENR_SPI3LPEN         (1 << 15) /* RCC APB1LLPENR: SPI3LPEN */
#define RCC_APB1LLPENR_SPDIFRXLPEN      (1 << 16) /* RCC APB1LLPENR: SPDIFRXLPEN */
#define RCC_APB1LLPENR_USART2LPEN       (1 << 17) /* RCC APB1LLPENR: USART2LPEN */
#define RCC_APB1LLPENR_USART3LPEN       (1 << 18) /* RCC APB1LLPENR: USART3LPEN */
#define RCC_APB1LLPENR_UART4LPEN        (1 << 19) /* RCC APB1LLPENR: UART4LPEN */
#define RCC_APB1LLPENR_UART5LPEN        (1 << 20) /* RCC APB1LLPENR: UART5LPEN */
#define RCC_APB1LLPENR_I2C1LPEN         (1 << 21) /* RCC APB1LLPENR: I2C1LPEN */
#define RCC_APB1LLPENR_I2C2LPEN         (1 << 22) /* RCC APB1LLPENR: I2C2LPEN */
#define RCC_APB1LLPENR_I2C3LPEN         (1 << 23) /* RCC APB1LLPENR: I2C3LPEN */
#define RCC_APB1LLPENR_HDMICECLPEN      (1 << 27) /* RCC APB1LLPENR: HDMICECLPEN */
#define RCC_APB1LLPENR_DAC1LPEN         (1 << 29) /* RCC APB1LLPENR: DAC1LPEN */
#define RCC_APB1LLPENR_UART7LPEN        (1 << 30) /* RCC APB1LLPENR: UART7LPEN */
#define RCC_APB1LLPENR_UART8LPEN        (1 << 31) /* RCC APB1LLPENR: UART8LPEN */

/* APB1 H low power mode peripheral clock enable register */

#define RCC_APB1HLPENR_CRSLPEN          (1 << 1)  /* RCC APB1HLPENR: CRSLPEN */
#define RCC_APB1HLPENR_SWPLPEN          (1 << 2)  /* RCC APB1HLPENR: SWPLPEN */
#define RCC_APB1HLPENR_OPAMPLPEN        (1 << 4)  /* RCC APB1HLPENR: OPAMPLPEN */
#define RCC_APB1HLPENR_MDIOSLPEN        (1 << 5)  /* RCC APB1HLPENR: MDIOSLPEN */
#define RCC_APB1HLPENR_FDCANLPEN        (1 << 8)  /* RCC APB1HLPENR: FDCANLPEN */

/* APB2 low power mode peripheral clock enable register */

#define RCC_APB2LPENR_TIM1LPEN          (1 << 0)  /* RCC APB2LPENR: TIM1LPEN */
#define RCC_APB2LPENR_TIM8LPEN          (1 << 1)  /* RCC APB2LPENR: TIM8LPEN */
#define RCC_APB2LPENR_USART1LPEN        (1 << 4)  /* RCC APB2LPENR: USART1LPEN */
#define RCC_APB2LPENR_USART6LPEN        (1 << 5)  /* RCC APB2LPENR: USART6LPEN */
#define RCC_APB2LPENR_SPI1LPEN          (1 << 12) /* RCC APB2LPENR: SPI1LPEN */
#define RCC_APB2LPENR_SPI4LPEN          (1 << 13) /* RCC APB2LPENR: SPI4LPEN */
#define RCC_APB2LPENR_TIM15LPEN         (1 << 16) /* RCC APB2LPENR: TIM15LPEN */
#define RCC_APB2LPENR_TIM16LPEN         (1 << 17) /* RCC APB2LPENR: TIM16LPEN */
#define RCC_APB2LPENR_TIM17LPEN         (1 << 18) /* RCC APB2LPENR: TIM17LPEN */
#define RCC_APB2LPENR_SPI5LPEN          (1 << 20) /* RCC APB2LPENR: SPI5LPEN */
#define RCC_APB2LPENR_SAI1LPEN          (1 << 22) /* RCC APB2LPENR: SAI1LPEN */
#define RCC_APB2LPENR_SAI2LPEN          (1 << 23) /* RCC APB2LPENR: SAI2LPEN */
#define RCC_APB2LPENR_SAI3LPEN          (1 << 24) /* RCC APB2LPENR: SAI3LPEN */
#define RCC_APB2LPENR_DFSDM1LPEN        (1 << 28) /* RCC APB2LPENR: DFSDM1LPEN */
#define RCC_APB2LPENR_HRTIMLPEN         (1 << 29) /* RCC APB2LPENR: HRTIMLPEN */

/* APB4 low power mode peripheral clock enable register */

#define RCC_APB4LPENR_SYSCFGLPEN        (1 << 1)  /* RCC APB4LPENR: SYSCFGLPEN */
#define RCC_APB4LPENR_LPUART1LPEN       (1 << 3)  /* RCC APB4LPENR: LPUART1LPEN */
#define RCC_APB4LPENR_SPI6LPEN          (1 << 5)  /* RCC APB4LPENR: SPI6LPEN */
#define RCC_APB4LPENR_I2C4LPEN          (1 << 7)  /* RCC APB4LPENR: I2C4LPEN */
#define RCC_APB4LPENR_LPTIM2LPEN        (1 << 9)  /* RCC APB4LPENR: LPTIM2LPEN */
#define RCC_APB4LPENR_LPTIM3LPEN        (1 << 10) /* RCC APB4LPENR: LPTIM3LPEN */
#define RCC_APB4LPENR_LPTIM4LPEN        (1 << 11) /* RCC APB4LPENR: LPTIM4LPEN */
#define RCC_APB4LPENR_LPTIM5LPEN        (1 << 12) /* RCC APB4LPENR: LPTIM5LPEN */
#define RCC_APB4LPENR_COMP12LPEN        (1 << 14) /* RCC APB4LPENR: COMP12LPEN */
#define RCC_APB4LPENR_VREFLPEN          (1 << 15) /* RCC APB4LPENR: VREFLPEN */
#define RCC_APB4LPENR_RTCAPBLPEN        (1 << 16) /* RCC APB4LPENR: RTCAPBLPEN */
#define RCC_APB4LPENR_SAI4LPEN          (1 << 21) /* RCC APB4LPENR: SAI4LPEN */


/* RCC Global Control register */

#define RCC_GCR_WW1RSC                  (1 << 0)  /* Bit 0: WWDG1 reset scope control */
#ifdef CONFIG_STM32H5_HAVE_CM4
#  define RCC_GCR_WW2RSC                (1 << 1)  /* Bit 1: WWDG2 reset scope control */
#endif
#ifdef CONFIG_STM32H5_HAVE_CM4
#  define RCC_GCR_BOOT_C1               (1 << 2)  /* Bit 2: Allows CPU1 to boot */
#  define RCC_GCR_BOOT_C2               (1 << 3)  /* Bit 3: Allows CPU2 to boot */
#endif

/* TODO: D3 Autonomous mode register */

/* RCC Reset Status register */

                                                  /* Bits 0-15: Reserved */
#define RCC_RSR_RMVF                    (1 << 16) /* Bit 16: Remove reset flag */
#define RCC_RSR_CPURSTF                 (1 << 17) /* Bit 17: CPU reset flag */
                                                  /* Bit 18: Reserved */
#define RCC_RSR_D1RSTF                  (1 << 19) /* Bit 19: D1 domain power switch reset flag */
#define RCC_RSR_D2RSTF                  (1 << 20) /* Bit 20: D2 domain power switch reset flag */
#define RCC_RSR_BORRSTF                 (1 << 21) /* Bit 21: BOR reset flag */
#define RCC_RSR_PINRSTF                 (1 << 22) /* Bit 22: Pin reset flag */
#define RCC_RSR_PORRSTF                 (1 << 23) /* Bit 23: POR/PDR reset flag */
#define RCC_RSR_SFTRSTF                 (1 << 24) /* Bit 24: System reset from CPU flag */
                                                  /* Bit 25: Reserved */
#define RCC_RSR_IWDG1RSTF               (1 << 26) /* Bit 26: Independent watchdog reset flag */
                                                  /* Bit 27: Reserved */
#define RCC_RSR_WWDG1RSTF               (1 << 28) /* Bit 28: Window watchdog reset flag */
                                                  /* Bit 29: Reserved */
#define RCC_RSR_LPWRRSTF                (1 << 30) /* Bit 30: Reset due to illegal D1 DStandby or CPU Cstop flag */
                                                  /* Bit 31: Reserved */

/* Backup domain control register */

#define RCC_BDCR_LSEON                  (1 << 0)                     /* Bit 0: External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY                 (1 << 1)                     /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP                 (1 << 2)                     /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_LSEDRV_SHIFT           (3)                          /* Bits 4:3: LSE oscillator Drive selection */
#define RCC_BDCR_LSEDRV_MASK            (3 << RCC_BDCR_LSEDRV_SHIFT) /* See errata ES0392 Rev 7. 2.2.14 */
#  define RCC_BDCR_LSEDRV_LOW           (0 << RCC_BDCR_LSEDRV_SHIFT) /* 00: Low driving capability */
#  define RCC_BDCR_LSEDRV_MEDHI_Y       (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium high driving capability rev y */
#  define RCC_BDCR_LSEDRV_MEDHI         (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium high driving capability */
#  define RCC_BDCR_LSEDRV_MEDLO_Y       (2 << RCC_BDCR_LSEDRV_SHIFT) /* 10: Medium low driving capability rev y */
#  define RCC_BDCR_LSEDRV_MEDLO         (1 << RCC_BDCR_LSEDRV_SHIFT) /* 01: Medium low driving capability */
#  define RCC_BDCR_LSEDRV_HIGH          (3 << RCC_BDCR_LSEDRV_SHIFT) /* 11: High driving capability */
#define RCC_BDCR_LSECSSON               (1 << 5)                     /* Bit 5: LSE clock security system enable */
#define RCC_BDCR_LSECSSD                (1 << 6)                     /* Bit 6: LSE clock security system failure detection */
                                                                     /* Bit 7: Reserved */
#define RCC_BDCR_RTCSEL_SHIFT           (8)                          /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK            (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK         (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE           (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI           (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE           (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 128 used as RTC clock */
                                                                     /* Bits 10-15: Reserved */
#define RCC_BDCR_RTCEN                  (1 << 15)                    /* Bit 15: RTC clock enable */
#define RCC_BDCR_BDRST                  (1 << 16)                    /* Bit 16: Backup domain software reset */
                                                                     /* Bits 17-31: Reserved */

#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_RCC_H */
