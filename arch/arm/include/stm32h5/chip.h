/****************************************************************************
 * arch/arm/include/stm32h5/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_STM32H5_CHIP_H
#define __ARCH_ARM_INCLUDE_STM32H5_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* STM32H5x3xx  Differences between family members:
 *
 *   ----------- ---------------- ----- ----
 *                                       SPI
 *   PART        PACKAGE          GPIOs  I2S
 *   ----------- ---------------- ----- ----
 *   STM32H563ZI  
 *   ----------- ---------------- ----- ----
 *
 * STM32H563ZI have 2MB of FLASH
 *
 * The correct FLASH size will be set CONFIG_STM32H5_FLASH_CONFIG_x or
 * overridden with CONFIG_STM32H5_FLASH_OVERRIDE_x
 */

#if defined (CONFIG_ARCH_CHIP_STM32H563ZI) || defined (CONFIG_ARCH_CHIP_STM32H573ZI)
#else
#  error STM32 H5 chip not identified
#endif

/* Size SRAM */

#if defined(CONFIG_STM32H5_STM32H5X3XX)
/* Memory */

#    define STM32H5_SRAM_SIZE             (0)  /* 512Kb SRAM on AXI bus Matrix (D1) */
#    define STM32H5_SRAM1_SIZE            (256*1024)  /* 128Kb SRAM1 on AHB bus Matrix (D2) */
#    define STM32H5_SRAM2_SIZE            (64*1024)  /* 128Kb SRAM2 on AHB bus Matrix (D2) */
#    define STM32H5_SRAM3_SIZE            (320*1024)  /* 128Kb SRAM2 on AHB bus Matrix (D2) */

/* Peripherals */

#  if defined(CONFIG_STM32H5_IO_CONFIG_A)
#      define STM32H5_NGPIO               (10)        /* GPIOA-GPIOJ */
#  elif defined(CONFIG_STM32H5_IO_CONFIG_B)
#      define STM32H5_NGPIO               (11)        /* GPIOA-GPIOK */
#  elif defined(CONFIG_STM32H5_IO_CONFIG_I)
#      define STM32H5_NGPIO               (9)         /* GPIOA-GPIOI */
#  elif defined(CONFIG_STM32H5_IO_CONFIG_V)
#      define STM32H5_NGPIO               (8)         /* GPIOA-GPIOH, missing GPIOF-GPIOG */
#  elif defined(CONFIG_STM32H5_IO_CONFIG_X)
#      define STM32H5_NGPIO               (11)        /* GPIOA-GPIOK */
#  elif defined(CONFIG_STM32H5_IO_CONFIG_Z)
#      define STM32H5_NGPIO               (8)         /* GPIOA-GPIOH */
#  else
#      error CONFIG_STM32H5_IO_CONFIG_x Not Set
#  endif

#  define STM32H5_NDMA                    (2)         /* (4) DMA1, DMA2, BDMA and MDMA */
#  define STM32H5_NADC                    (2)         /* (3) ADC1-3*/
#  define STM32H5_NDAC                    (2)         /* (2) DAC1-2*/
#  define STM32H5_NCMP                    (0U)         /* (2) ultra-low power comparators */
#  define STM32H5_NPGA                    (0U)         /* (2) Operational amplifiers: OPAMP */
#  define STM32H5_NDFSDM                  (1)         /* (1) digital filters for sigma delta modulator */
#  define STM32H5_NUSART                  (6)         /* (4) USART1-3, 6, 10, 11 */
#  define STM32H5_NSPI                    (6)         /* (6) SPI1-6 */
#  define STM32H5_NI2S                    (3)         /* (3) I2S1-3 */
#  define STM32H5_NUART                   (5)         /* (4) UART4-5, 7-9, 12 */
#  define STM32H5_NI2C                    (4)         /* (4) I2C1-4 */
#  define STM32H5_NI3C                    (1)         /* (4) I2C1-4 */
#  define STM32H5_NSAI                    (1)         /* (4) SAI1-4*/
#  define STM32H5_NCAN                    (2)         /* (2) CAN1-2 */
#  define STM32H5_NSDIO                   (0U)         /* (2) SDIO */
#else
#  error STM32 H5 chip Family not identified
#endif

/* TBD FPU Configuration */

#if defined(CONFIG_ARCH_HAVE_FPU)
#else
#endif

/* Diversification based on Family and package */

#if defined(CONFIG_STM32H5_HAVE_ETHERNET)
#  define STM32H5_NETHERNET                1   /* 100/100 Ethernet MAC */
#else
#  define STM32H5_NETHERNET                0   /* No 100/100 Ethernet MAC */
#endif

#if defined(CONFIG_STM32H5_HAVE_FMC)
#  define STM32H5_NFMC                     1   /* Have FMC memory controller */
#else
#  define STM32H5_NFMC                     0   /* No FMC memory controller */
#endif

/* NVIC priority levels *****************************************************/

/* 16 Programmable interrupt levels */

#define NVIC_SYSH_PRIORITY_MIN     0xf0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x10 /* Four bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_STM32H5_CHIP_H */
