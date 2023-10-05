/****************************************************************************
 * arch/arm/src/stm32h5/hardware/stm32h5x3xx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STM32H5X3XX Address Blocks ***********************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb CODE block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb SRAM block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb AHB1-2 peripheral blocks */
#define STM32_FMC_BANK1      0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/PSRMA/SRAM */
#define STM32_FMC_BANK2      0x70000000     /* 0x70000000-0x7fffffff: 256Mb SDRAM */
#define STM32_FMC_BANK3      0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#define STM32_FMC_BANK4      0x90000000     /* 0x90000000-0x9fffffff: 256Mb QUADSPI */
#define STM32_FMC_BANK5      0xc0000000     /* 0xc0000000-0xcfffffff: 256Mb FMC SDRAM Bank 1 */
#define STM32_FMC_BANK6      0xd0000000     /* 0xd0000000-0xdfffffff: 256Mb FMC SDRAM Bank 2 */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M337 block */

#define STM32_REGION_MASK    0xff000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FMC_BANK1)

/* Code Base Addresses ******************************************************/

#define STM32_FLASH_BANK1    0x08000000     /* 0x08000000-0x080fffff: FLASH memory 1 */
#define STM32_FLASH_BANK2    0x08100000     /* 0x08100000-0x081fffff: FLASH memory 2 */
#define STM32_FLASH_BASE     STM32_FLASH_BANK1
/* #define STM32_SYSMEM_MEM     0x1ff00000     /1* 0x1ff00000-0x1ff1ffff: System memory *1/ */

/* SRAM Base Addresses ******************************************************/

#define STM32_SRAM1_BASE   0x20000000     /* 0x20000000-0x2003ffff: System SRAM1 */
#define STM32_SRAM2_BASE   0x20040000     /* 0x20040000-0x2004ffff: System SRAM2 */
#define STM32_SRAM3_BASE   0x20050000     /* 0x20050000-0x20047fff: System SRAM3 */
#define STM32_SRAM123_BASE 0x20000000     /* 0x20000000-0x20047fff: System SRAM123 */
#define STM32_BBSRAM_BASE  0x40036400     /* 0x40036400-0x400373ff: System Backup SRAM */

/* Peripheral Base Addresses ************************************************/
/* Non-Secure boundary */
#define STM32_PREGION_MASK   0xff000000
#define STM32_D2_BASE        0x40000000     /* 0x40000000-0x48022800: D2 domain */
#  define STM32_APB1_BASE    0x40000000     /* 0x40000000-0x40007fff: APB1 */
#  define STM32_APB2_BASE    0x40010000     /* 0x40010000-0x40016bff: APB2 */
#  define STM32_AHB1_BASE    0x40020000     /* 0x40020000-0x4007ffff: APB1 */
#  define STM32_AHB2_BASE    0x42020000     /* 0x50000000-0x48022bff: AHB2 */
#define STM32_D1_BASE        0x44000000     /* 0x50000000-0x50003fff: D1 domain */
#  define STM32_APB3_BASE    0x44000000     /* 0x60000000-0x50003fff: APB3 */
#  define STM32_AHB3_BASE    0x44020000     /* 0x51000000-0x52008fff: AHB3 */
#define STM32_D3_BASE        0x46000000     /* 0x58000000-0x580267ff: D3 domain */
#  define STM32_APB4_BASE    0x46000000     /* 0x60000000-0x58006bff: APB4 */
#  define STM32_AHB4_BASE    0x46020000     /* 0x58020000-0x580267ff: AHB4 */

/* APB1 Base Addresses ******************************************************/

#define STM32_TIM2_BASE        0x40000000     /* 0x40000000-0x400003ff: TIM2 */
#define STM32_TIM3_BASE        0x40000400     /* 0x40000400-0x400007ff: TIM3 */
#define STM32_TIM4_BASE        0x40000800     /* 0x40000800-0x40000bff: TIM4 */
#define STM32_TIM5_BASE        0x40000c00     /* 0x40000c00-0x40000fff: TIM5 */
#define STM32_TIM6_BASE        0x40001000     /* 0x40001000-0x400013ff: TIM6 */
#define STM32_TIM7_BASE        0x40001400     /* 0x40001400-0x400017ff: TIM7 */
#define STM32_TIM12_BASE       0x40001800     /* 0x40001800-0x40001bff: TIM12 */
#define STM32_TIM13_BASE       0x40001c00     /* 0x40001c00-0x40001fff: TIM13 */
#define STM32_TIM14_BASE       0x40002000     /* 0x40002000-0x400023ff: TIM14 */
#define STM32_WWDG_BASE        0x40002C00     /* 0x50003000-0x50003fff WWDG1 */
#define STM32_IWDG_BASE        0x40003000     /* 0x50003000-0x50003fff WWDG1 */
#define STM32_SPI2_BASE        0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_I2S2_BASE        0x40003800     /* 0x40003800-0x40003bff: SPI2 / I2S2 */
#define STM32_SPI3_BASE        0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_I2S3_BASE        0x40003c00     /* 0x40003c00-0x40003fff: SPI3 / I2S3 */
#define STM32_USART2_BASE      0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE      0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE       0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE       0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE        0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE        0x40005800     /* 0x40005800-0x40005bff: I2C2 */
#define STM32_I3C1_BASE        0x40005c00     /* 0x40005800-0x40005bff: I2C2 */
#define STM32_CRS_BASE         0x40006000     /* 0x40008400-0x400087ff: CRS */
#define STM32_USART6_BASE      0x40006400     /* 0x40011400-0x400117ff USART6 */
#define STM32_USART10_BASE     0x40006800     /* 0x40011400-0x400117ff USART6 */
#define STM32_USART11_BASE     0x40006c00     /* 0x40011400-0x400117ff USART6 */
#define STM32_HDMICEC_BASE     0x40007000     /* 0x40006c00-0x40006fff: HDMI-CEC */
#define STM32_UART7_BASE       0x40007800     /* 0x40007800-0x40007bff: UART7 */
#define STM32_UART8_BASE       0x40007c00     /* 0x40007c00-0x40007fff: UART8 */
#define STM32_UART9_BASE       0x40008000     /* 0x40007800-0x40007bff: UART7 */
#define STM32_UART12_BASE      0x40008400     /* 0x40007c00-0x40007fff: UART8 */
#define STM32_DTS_BASE         0x40008c00     /* 0x40007c00-0x40007fff: UART8 */
#define STM32_LPTIM2_BASE      0x40009400     /* 0x40002400-0x400027ff: LPTIM2 */
#define STM32_FDCAN1_BASE      0x4000a400     /* 0x4000a000 - 0x4000a3ff FDCAN1 Section */
#define STM32_FDCAN2_BASE      0x4000a800     /* 0x4000a400 - 0x4000A7ff FDCAN2 Section */
#define STM32_FDCAN_BASE       0x4000ac00     /* 0x4000D400 - 0x4000D7FF FDCAN Section */
#define STM32_UCPD1_BASE       0x4000dc00     /* 0x4000D400 - 0x4000D7FF FDCAN Section */


/* APB2 Base Addresses ******************************************************/

#define STM32_TIM1_BASE        0x40012c00     /* 0x40014000-0x400143ff TIM1 */
#define STM32_SPI1_BASE        0x40013000     /* 0x40013000-0x400133ff SPI1/I2S1 */
#define STM32_I2S1_BASE        0x40013000     /* 0x40013000-0x400133ff SPI1/I2S1 */
#define STM32_TIM8_BASE        0x40013400     /* 0x40010400-0x400107ff TIM8 */
#define STM32_USART1_BASE      0x40013800     /* 0x40011000-0x400113ff USART1 */
#define STM32_TIM15_BASE       0x40014000     /* 0x40014000-0x400143ff TIM15 */
#define STM32_TIM16_BASE       0x40014400     /* 0x40014400-0x400147ff TIM16 */
#define STM32_TIM17_BASE       0x40014800     /* 0x40014800-0x40014bff TIM17 */
#define STM32_SPI4_BASE        0x40014c00     /* 0x40013400-0x400137ff SPI4 */
#define STM32_SPI6_BASE        0x40015000     /* 0x58001400-0x580017ff SPI6 */
#define STM32_SAI1_BASE        0x40015400     /* 0x40015800-0x40015bff SAI1 */
#define STM32_SAI2_BASE        0x40015800     /* 0x40015c00-0x40015ffF SAI2 */
#define STM32_USBFS_BASE       0x40016000     /* 0x40015c00-0x40015ffF SAI2 */
#define STM32_USBFSRAM_BASE    0x40016400     /* 0x40015c00-0x40015ffF SAI2 */

/* AHB1 Base Addresses ******************************************************/

#define STM32_DMA1_BASE        0x40020000     /* 0x40020000-0x400203ff DMA1 */
#define STM32_DMA2_BASE        0x40020400     /* 0x40020400-0x400207ff DMA2 */
#define STM32_FLASHIF_BASE     0x40022000     /* 0x52002000-0x52002fff FLASH interface */
#define STM32_CRC_BASE         0x40023000     /* 0x58024c00-0x58024fff CRC */
#define STM32_CORDIC_BASE      0x40023800     /* 0x58024c00-0x58024fff CRC */
#define STM32_FMAC_BASE        0x40023c00     /* 0x58024c00-0x58024fff CRC */
#define STM32_RAMCFG_BASE      0x40026000     /* 0x58024c00-0x58024fff CRC */
#define STM32_EMAC_BASE        0x40028000     /* 0x40028000-0x400293ff Ethernet MAC */


/* AHB2 Base Addresses ******************************************************/

#define STM32_GPIOA_BASE       0x42020000     /* 0x42020000-0x420203ff GPIOA */
#define STM32_GPIOB_BASE       0x42020400     /* 0x42020400-0x420207ff GPIOB */
#define STM32_GPIOC_BASE       0x42020800     /* 0x42020800-0x42020bff GPIOC */
#define STM32_GPIOD_BASE       0x42020c00     /* 0x42020c00-0x42020fff GPIOD */
#define STM32_GPIOE_BASE       0x42021000     /* 0x42021000-0x420213ff GPIOE */
#define STM32_GPIOF_BASE       0x42021400     /* 0x42021400-0x420217ff GPIOF */
#define STM32_GPIOG_BASE       0x42021800     /* 0x42021800-0x42021bff GPIOG */
#define STM32_GPIOH_BASE       0x42021c00     /* 0x42021c00-0x42021fff GPIOH */
#define STM32_GPIOI_BASE       0x42022000     /* 0x42022000-0x420223ff GPIOI */
#define STM32_ADC12_BASE       0x42028000     /* 0x40022000-0x400223ff ADC1-ADC2 */
#define STM32_DAC1_BASE        0x42028400     /* 0x40007400-0x400077ff: DAC */
#define STM32_DCMI_BASE        0x4202c000     /* 0x48020000-0x480203ff DCMI */
#define STM32_PSSI_BASE        0x4202c400     /* 0x48020000-0x480203ff DCMI */
#define STM32_AES_BASE         0x420c0000     /* 0x48020000-0x480203ff DCMI */
#define STM32_HASH_BASE        0x420c0400     /* 0x48021400-0x480217ff HASH */
#define STM32_RNG_BASE         0x420c0800     /* 0x48021800-0x48021bff RNG */
#define STM32_SAES_BASE        0x420c0c00     /* 0x48020000-0x480203ff DCMI */
#define STM32_PKA_BASE         0x420c2000     /* 0x48020000-0x480203ff DCMI */




/* APB3 Base Addresses ******************************************************/

#define STM32_SBS_BASE         0x44000400     /* 0x40015000-0x400153ff SPI5 */
#define STM32_SPI5_BASE        0x44002000     /* 0x40015000-0x400153ff SPI5 */
#define STM32_LPUART1_BASE     0x44002400     /* 0x58000c00-0x58000fff LPUART1 */
#define STM32_I2C3_BASE        0x44002800     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_I2C4_BASE        0x44002c00     /* 0x58001c00-0x58001fff I2C4 */
#define STM32_LPTIM1_BASE      0x44004400     /* 0x40002400-0x400027ff: LPTIM1 */
#define STM32_LPTIM3_BASE      0x44004800     /* 0x40002400-0x400027ff: LPTIM3 */
#define STM32_LPTIM4_BASE      0x44004c00     /* 0x40002400-0x400027ff: LPTIM4 */
#define STM32_LPTIM5_BASE      0x44005000     /* 0x40002400-0x400027ff: LPTIM5 */
#define STM32_LPTIM6_BASE      0x44005400     /* 0x40002400-0x400027ff: LPTIM6 */
#define STM32_VREF_BASE        0x44007400     /* 0x58003c00-0x58003fff VREF */
#define STM32_RTC_BASE         0x44007800     /* 0x58004000-0x580043ff RTC & BKP */
#define STM32_TAMP_BASE        0x44007c00     /* 0x58004000-0x580043ff RTC & BKP */


/* AHB3 Base Addresses ******************************************************/

#define STM32_PWR_BASE         0x44020800         /* 0x58024800-0x58024bff PWR */
#define STM32_RCC_BASE         0x44020c00     /* 0x58024400-0x580247ff RCC */
#define STM32_EXTI_BASE        0x44022000     /* 0x58000000-0x580003ff EXTI */
#define STM32_DEBUGMCU_BASE    0x44024000


/* AHB4 Base Addresses ******************************************************/

#define STM32_OTFDEC1_BASE     0x46005000     /* 0x48021000-0x480213ff CRYPTO */
#define STM32_SDMMC1_BASE      0x46008400     /* 0x52007000-0x52007fff SDMMC1 */

#define STM32_DLYBSDMMC1_BASE  0x52008000     /* 0x52008000-0x52008fff Delay Block SDMMC1 */
#define STM32_DLYBSDMMC2_BASE  0x48022800     /* 0x48022800-0x48022bff Delay Block SDMMC2 */



#define STM32_CRYPTO_BASE      0x48021000     /* 0x48021000-0x480213ff CRYPTO */
#define STM32_SDMMC2_BASE      0x48022400     /* 0x48022400-0x480227ff SDMMC2 */





#define STM32_SAI3_BASE        0x40016000     /* 0x40016000-0x400163ff SAI3 */
#define STM32_DFSDM1_BASE      0x40017000     /* 0x40017000-0x400173ff DFSDM1 */
#define STM32_HRTIM_BASE       0x40017400     /* 0x40017400-0x400177ff HRTIM */


#define STM32_LTDC_BASE        0x50001000     /* 0x50001000-0x50001fff LTDC */
#define STM32_WWDG1_BASE       0x50003000     /* 0x50003000-0x50003fff WWDG1 */


#define STM32_GPV_BASE         0x51000000     /* 0x51000000-0x510fffff AXI interconnect */
#define STM32_FMC_BASE         0x52004000     /* 0x52004000-0x52004fff FMC control */
#define STM32_NORC_BASE        0x52004000     /*                       NOR/PSRAM controller */
#define STM32_NANDC_BASE       0x52004000     /*                       NAND Flash controller */
#define STM32_SDRAMC_BASE      0x52004000     /*                       SDRAM controller */
#define STM32_QUADSPI_BASE     0x52005000     /* 0x52005000-0x52005fff QUADSPI control */
#define STM32_DLYBQUADSPI_BASE 0x52006000     /* 0x52006000-0x52006fff QUADSPI Delay Block */

/* APB4 Base Addresses ******************************************************/

#define STM32_SYSCFG_BASE      0x58000400     /* 0x58000400-0x580007ff SYSCFG */
#define STM32_COMP12_BASE      0x58003800     /* 0x58003800-0x58003bff COMP1-COMP2 */
#define STM32_IWDG1_BASE       0x58004800     /* 0x58004800-0x58004bff IWDG1 */
#define STM32_SAI4_BASE        0x58005400     /* 0x58005400-0x580057ff SAI4 */



#define STM32_DMAMUX2_BASE     0x48025800     /* 0x58025800-0x58025bff DMAMUX2 */
#define STM32_ADC3_BASE        0x48026000     /* 0x58026000-0x580263ff ADC3 */
/* #define STM32_HSEM_BASE        0x58026400     /1* 0x58026400-0x580267ff HSEM *1/ */


#define STM32_SPDIFRX_BASE     0x40004000     /* 0x40004000-0x400043ff: SPDIFRX */
#define STM32_SWPMI_BASE       0x40008800     /* 0x40008800 - 0x40008bff SWPMI Section */
#define STM32_OPAMP_BASE       0x40009000     /* 0x40009000 - 0x400093ff OPAMP Section */
#define STM32_MDIOS_BASE       0x40009400     /* 0x40009400 - 0x400097ff MDIOS Section */
#define STM32_CANCCU_BASE      0x4000a800     /* 0x4000a800 - 0x4000abff CAN CCU Section */
#define STM32_CANRAM_BASE      0x4000ac00     /* 0x4000ac00 - 0x4000d3ff CAN Message RAM */

#define STM32_SYSMEM_UID       0x1ff1e800

// 115 Table3
#define STM32_DMAMUX1_BASE     0x40020800     /* 0x40020800-0x40020bff DMAMUX1 */
#define STM32_OTGHS_BASE       0x40040000     /* 0x40040000-0x4007ffff USB1 OTG HS/FS */
#define STM32_OTGFS_BASE       0x40080000     /* 0x40080000-0x400bffff USB2 OTG FS */
/* The DBGMCU registers are accessible to the debugger via the APB-D bus
 * at base address 0xe00e1000. They are also accessible by the processor
 * core at base address 0x5c001000.
 */


#endif /* __ARCH_ARM_SRC_STM32H5_HARDWARE_STM32H5X3XX_MEMORYMAP_H */
