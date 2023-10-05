/****************************************************************************
 * arch/arm/src/stm32h5/stm32h5x3xx_rcc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "stm32_pwr.h"
#include "hardware/stm32_syscfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* FLASH wait states */

#if !defined(BOARD_FLASH_WAITSTATES)
#  error BOARD_FLASH_WAITSTATES not defined
#elif BOARD_FLASH_WAITSTATES < 0 || BOARD_FLASH_WAITSTATES > 15
#  error BOARD_FLASH_WAITSTATES is out of range
#endif

/* Voltage output scale (default to Scale 1 mode) */

#ifndef STM32_PWR_VOS_SCALE
#  define STM32_PWR_VOS_SCALE PWR_D3CR_VOS_SCALE_1
#endif

#if !defined(BOARD_FLASH_PROGDELAY)
#  if (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_1) || \
      (STM32_PWR_VOS_SCALE == PWR_D3CR_VOS_SCALE_0)
#    if STM32_SYSCLK_FREQUENCY <= 70000000 && BOARD_FLASH_WAITSTATES == 0
#      define BOARD_FLASH_PROGDELAY  0
#    elif STM32_SYSCLK_FREQUENCY <= 140000000 && BOARD_FLASH_WAITSTATES == 1
#      define BOARD_FLASH_PROGDELAY  10
#    elif STM32_SYSCLK_FREQUENCY <= 185000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  1
#    elif STM32_SYSCLK_FREQUENCY <= 210000000 && BOARD_FLASH_WAITSTATES == 2
#      define BOARD_FLASH_PROGDELAY  2
#    elif STM32_SYSCLK_FREQUENCY <= 225000000 && BOARD_FLASH_WAITSTATES == 3
#      define BOARD_FLASH_PROGDELAY  2
#    else
#      define BOARD_FLASH_PROGDELAY  2
#    endif
#  else
#    define BOARD_FLASH_PROGDELAY    2
#  endif
#endif

/* PLL are only enabled if the P,Q or R outputs are enabled. */

#undef USE_PLL1
#if STM32_PLLCFG_PLL1CFG & (RCC_PLL1CFGR_PLL1PEN | RCC_PLL1CFGR_PLL1QEN | \
                            RCC_PLL1CFGR_PLL1REN)
#  define USE_PLL1
#endif

#undef USE_PLL2
#if STM32_PLLCFG_PLL2CFG & (RCC_PLL2CFGR_PLL2PEN | RCC_PLL2CFGR_PLL2QEN | \
                            RCC_PLL2CFGR_PLL2REN)
#  define USE_PLL2
#endif

#undef USE_PLL3
#if STM32_PLLCFG_PLL3CFG & (RCC_PLL3CFGR_PLL3PEN | RCC_PLL3CFGR_PLL3QEN | \
                            RCC_PLL3CFGR_PLL3REN)
#  define USE_PLL3
#endif

#if defined(STM32_BOARD_USEHSI) && !defined(STM32_BOARD_HSIDIV)
#error When HSI is used, you have to define STM32_BOARD_HSIDIV in board/include/board.h
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x0000002b, STM32_RCC_CFGR1);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON | RCC_CR_HSI48ON |
              RCC_CR_CSION | RCC_CR_PLL1ON |
              RCC_CR_PLL2ON | RCC_CR_PLL3ON |
              RCC_CR_HSIDIV_MASK);

  /* Set HSI predivider to default (4, 16MHz) */

  regval |= RCC_CR_HSIDIV_4;

  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  /* putreg32(RCC_PLL1CFGR_RESET, STM32_RCC_PLL1CFGR); */
  putreg32(0x0, STM32_RCC_PLL1CFGR);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIER);
}

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);

#ifdef CONFIG_STM32H5_GDMA1
  /* GDMA 1 clock enable */

  regval |= RCC_AHB1ENR_GDMA1EN;
#endif

#ifdef CONFIG_STM32H5_GDMA2
  /* GDMA 2 clock enable */

  regval |= RCC_AHB1ENR_GDMA2EN;
#endif

#ifdef CONFIG_STM32H5_FLITF
  regval |= RCC_AHB1ENR_FLITFEN;
#endif

#ifdef CONFIG_STM32H5_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32H5_CORDIC
  regval |= RCC_AHB1ENR_CORDICEN;
#endif

#ifdef CONFIG_STM32H5_FMAC
  regval |= RCC_AHB1ENR_FMACEN;
#endif

#ifdef CONFIG_STM32H5_RAMCFG
  regval |= RCC_AHB1ENR_RAMCFGEN;
#endif

#ifdef CONFIG_STM32H5_ETHMAC
  /* Enable ethernet clocks */

  regval |= (RCC_AHB1ENR_ETH1MACEN | RCC_AHB1ENR_ETH1TXEN |
             RCC_AHB1ENR_ETH1RXEN);
#endif

#ifdef CONFIG_STM32H5_TZSC1
  regval |= RCC_AHB1ENR_TZSC1EN;
#endif

#ifdef CONFIG_STM32H5_BKPRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB1ENR_BKPRAMEN;
#endif

#ifdef CONFIG_STM32H5_DCACHE
  regval |= RCC_AHB1ENR_DCACHEEN;
#endif

#ifdef CONFIG_STM32H5_SRAM1
  regval |= RCC_AHB1ENR_SRAM1EN;
#endif

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

  /* Enable GPIO, GPIOB, ... GPIOI */

#if STM32H5_NGPIO > 0
  regval |= (RCC_AHB2ENR_GPIOAEN
#if STM32H5_NGPIO > 1
             | RCC_AHB2ENR_GPIOBEN
#endif
#if STM32H5_NGPIO > 2
             | RCC_AHB2ENR_GPIOCEN
#endif
#if STM32H5_NGPIO > 3
             | RCC_AHB2ENR_GPIODEN
#endif
#if STM32H5_NGPIO > 4
             | RCC_AHB2ENR_GPIOEEN
#endif
#if (STM32H5_NGPIO > 5) && (defined(CONFIG_STM32H5_HAVE_GPIOF))
             | RCC_AHB2ENR_GPIOFEN
#endif
#if (STM32H5_NGPIO > 6) && (defined(CONFIG_STM32H5_HAVE_GPIOG))
             | RCC_AHB2ENR_GPIOGEN
#endif
#if STM32H5_NGPIO > 7
             | RCC_AHB2ENR_GPIOHEN
#endif
#if STM32H5_NGPIO > 8
             | RCC_AHB2ENR_GPIOIEN
#endif
    );
#endif

#ifdef CONFIG_STM32H5_ADC
  regval |= RCC_AHB2ENR_ADCEN;
#endif

#ifdef CONFIG_STM32H5_HASH
  regval |= RCC_AHB2ENR_HASHEN;
#endif

#ifdef CONFIG_STM32H5_RNG
  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32H5_PKA
  regval |= RCC_AHB2ENR_PKAEN;
#endif

#ifdef CONFIG_STM32H5_SAES
  regval |= RCC_AHB2ENR_SAESEN;
#endif

#ifdef CONFIG_STM32H5_SRAM2
  regval |= RCC_AHB2ENR_SRAM2EN;
#endif

#ifdef CONFIG_STM32H5_SRAM3
  regval |= RCC_AHB2ENR_SRAM3EN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
}


/****************************************************************************
 * Name: rcc_enableahb4
 *
 * Description:
 *   Enable selected AHB4 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb4(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB4ENR register to enabled the
   * selected AHB4 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB4ENR);

#ifdef CONFIG_STM32H5_OTFDEC1
  regval |= RCC_AHB4ENR_OTFDEC1EN;
#endif

#ifdef CONFIG_STM32H5_SDMMC1
  regval |= RCC_AHB4ENR_SDMMC1EN;
#endif

#ifdef CONFIG_STM32H5_SDMMC2
  regval |= RCC_AHB4ENR_SDMMC2EN;
#endif

#ifdef CONFIG_STM32H5_FMC
  regval |= RCC_AHB4ENR_FMCEN;
#endif

#ifdef CONFIG_STM32H5_OCTOSPI1
  regval |= RCC_AHB4ENR_OCTOSPI1EN;
#endif

  putreg32(regval, STM32_RCC_AHB4ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1L/HENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1LENR);

#ifdef CONFIG_STM32H5_TIM2
  regval |= RCC_APB1LENR_TIM2EN;
#endif

#ifdef CONFIG_STM32H5_TIM3
  regval |= RCC_APB1LENR_TIM3EN;
#endif

#ifdef CONFIG_STM32H5_TIM3
  regval |= RCC_APB1LENR_TIM4EN;
#endif

#ifdef CONFIG_STM32H5_TIM5
  regval |= RCC_APB1LENR_TIM5EN;
#endif

#ifdef CONFIG_STM32H5_TIM6
  regval |= RCC_APB1LENR_TIM6EN;
#endif

#ifdef CONFIG_STM32H5_TIM7
  regval |= RCC_APB1LENR_TIM7EN;
#endif


#ifdef CONFIG_STM32H5_TIM12
  regval |= RCC_APB1LENR_TIM12EN;
#endif

#ifdef CONFIG_STM32H5_TIM13
  regval |= RCC_APB1LENR_TIM13EN;
#endif

#ifdef CONFIG_STM32H5_TIM14
  regval |= RCC_APB1LENR_TIM14EN;
#endif

#ifdef CONFIG_STM32H5_WWDG
  regval |= RCC_APB1LENR_WWDGEN;
#endif

#ifdef CONFIG_STM32H5_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1LENR_SPI2EN;
#endif

#ifdef CONFIG_STM32H5_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1LENR_SPI3EN;
#endif

#ifdef CONFIG_STM32H5_USART2
  /* USART2 clock enable */

  regval |= RCC_APB1LENR_USART2EN;
#endif

#ifdef CONFIG_STM32H5_USART3
  /* USART2 clock enable */

  regval |= RCC_APB1LENR_USART3EN;
#endif

#ifdef CONFIG_STM32H5_UART4
  /* UART4 clock enable */

  regval |= RCC_APB1LENR_UART4EN;
#endif

#ifdef CONFIG_STM32H5_UART5
  /* UART5 clock enable */

  regval |= RCC_APB1LENR_UART5EN;
#endif

#ifdef CONFIG_STM32H5_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1LENR_I2C1EN;
#endif

#ifdef CONFIG_STM32H5_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1LENR_I2C2EN;
#endif

#ifdef CONFIG_STM32H5_I3C1
  /* I3C1 clock enable */

  regval |= RCC_APB1LENR_I3C1EN;
#endif

#ifdef CONFIG_STM32H5_CRS
  regval |= RCC_APB1LENR_CRSEN;
#endif

#ifdef CONFIG_STM32H5_USART6
  /* USART6 clock enable */

  regval |= RCC_APB1LENR_USART6EN;
#endif

#ifdef CONFIG_STM32H5_USART10
  /* USART10 clock enable */

  regval |= RCC_APB1LENR_USART10EN;
#endif

#ifdef CONFIG_STM32H5_USART11
  /* USART11 clock enable */

  regval |= RCC_APB1LENR_USART11EN;
#endif

#ifdef CONFIG_STM32H5_CEC
  regval |= RCC_APB1LENR_CECEN;
#endif

#ifdef CONFIG_STM32H5_UART7
  /* UART7 clock enable */

  regval |= RCC_APB1LENR_UART7EN;
#endif

#ifdef CONFIG_STM32H5_UART8
  /* UART8 clock enable */

  regval |= RCC_APB1LENR_UART8EN;
#endif

  putreg32(regval, STM32_RCC_APB1LENR);   /* Enable APB1L peripherals */

  regval = getreg32(STM32_RCC_APB1HENR);

#ifdef CONFIG_STM32H5_UART9
  /* UART9 clock enable */

  regval |= RCC_APB1HENR_UART9EN;
#endif

#ifdef CONFIG_STM32H5_UART12
  /* UART12 clock enable */

  regval |= RCC_APB1HENR_UART12EN;
#endif

#ifdef CONFIG_STM32H5_DTS
  regval |= RCC_APB1HENR_DTSEN;
#endif

#ifdef CONFIG_STM32H5_LPTIM2
  regval |= RCC_APB1HENR_LPTIM2EN;
#endif

#ifdef CONFIG_STM32H5_FDCAN
  /* FDCAN clock enable */

  regval |= RCC_APB1HENR_FDCANEN;
#endif

#ifdef CONFIG_STM32H5_UCPD1
  regval |= RCC_APB1HENR_UCPD1EN;
#endif

  putreg32(regval, STM32_RCC_APB1HENR);   /* Enable APB1H peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32H5_TIM1
  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32H5_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32H5_TIM8
  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32H5_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32H5_TIM15
  regval |= RCC_APB2ENR_TIM15EN;
#endif

#ifdef CONFIG_STM32H5_TIM16
  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32H5_TIM17
  regval |= RCC_APB2ENR_TIM17EN;
#endif

#ifdef CONFIG_STM32H5_SPI4
  /* SPI4 clock enable */

  regval |= RCC_APB2ENR_SPI4EN;
#endif

#ifdef CONFIG_STM32H5_SPI6
  /* SPI6 clock enable */

  regval |= RCC_APB2ENR_SPI6EN;
#endif

#ifdef CONFIG_STM32H5_SAI1
  regval |= RCC_APB2ENR_SAI1EN;
#endif

#ifdef CONFIG_STM32H5_SAI2
  regval |= RCC_APB2ENR_SAI2EN;
#endif

#ifdef CONFIG_STM32H5_USB
  /* USB clock enable */

  regval |= RCC_APB2ENR_USBEN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb3
 *
 * Description:
 *   Enable selected APB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB3ENR register to enabled the
   * selected APB3 peripherals.
   */

  regval = getreg32(STM32_RCC_APB3ENR);

#ifdef CONFIG_STM32H5_SBS
  regval |= RCC_APB3ENR_SBSEN;
#endif

#ifdef CONFIG_STM32H5_SPI5
  /* SPI5 clock enable */

  regval |= RCC_APB3ENR_SPI5EN;
#endif

#ifdef CONFIG_STM32H5_LPUART1
  regval |= RCC_APB3ENR_LPUART1EN;
#endif

#ifdef CONFIG_STM32H5_I2C3
  regval |= RCC_APB3ENR_I2C3EN;
#endif

#ifdef CONFIG_STM32H5_I2C4
  regval |= RCC_APB3ENR_I2C4EN;
#endif

#ifdef CONFIG_STM32H5_LPTIM1
  regval |= RCC_APB3ENR_LPTIM1EN;
#endif

#ifdef CONFIG_STM32H5_LPTIM3
  regval |= RCC_APB3ENR_LPTIM3EN;
#endif

#ifdef CONFIG_STM32H5_LPTIM4
  regval |= RCC_APB3ENR_LPTIM4EN;
#endif

#ifdef CONFIG_STM32H5_LPTIM5
  regval |= RCC_APB3ENR_LPTIM5EN;
#endif

#ifdef CONFIG_STM32H5_LPTIM6
  regval |= RCC_APB3ENR_LPTIM6EN;
#endif

#ifdef CONFIG_STM32H5_VREFBUF
  regval |= RCC_APB3ENR_VREFBUFEN;
#endif

#ifdef CONFIG_STM32H5_RTCAPB
  regval |= RCC_APB3ENR_RTCAPBEN;
#endif

  putreg32(regval, STM32_RCC_APB3ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb4();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_enableapb3();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIGITAL;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /// stm hal code start
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIGITAL;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 250;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /// stm hal code end







#ifdef STM32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */

  /* Set HSI predivider to board specific value */

  regval |= STM32_BOARD_HSIDIV;

  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#else /* if STM32_BOARD_USEHSE */
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
#ifdef STM32_HSEBYP_ENABLE          /* May be defined in board.h header file */
  regval |= RCC_CR_HSEBYP;          /* Enable HSE clock bypass */
#else
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
#endif
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#ifdef CONFIG_STM32H5_HSI48
  /* Enable HSI48 */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSI48ON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI48 is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSI48RDY) == 0)
    {
    }
#endif

#ifdef CONFIG_STM32H5_CSI
  /* Enable CSI */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_CSION;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the CSI is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) == 0)
    {
    }
#endif


  #if 1

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* AHB Prescaler. Devision factor of rcc_hclk */
  
      regval = getreg32(STM32_RCC_CFGR2);
      regval &= ~RCC_CFGR2_HPRE_MASK;
      regval |= STM32_RCC_CFGR2_HPRE;
      putreg32(regval, STM32_RCC_CFGR2);

      /* Set PCLK1 */

      regval = getreg32(STM32_RCC_CFGR2);
      regval &= ~RCC_CFGR2_PPRE1_MASK;
      regval |= STM32_RCC_CFGR2_PPRE1;
      putreg32(regval, STM32_RCC_CFGR2);

      /* Set PCLK2 */

      regval = getreg32(STM32_RCC_CFGR2);
      regval &= ~RCC_CFGR2_PPRE2_MASK;
      regval |= STM32_RCC_CFGR2_PPRE2;
      putreg32(regval, STM32_RCC_CFGR2);

      /* Set PCLK3 */

      regval = getreg32(STM32_RCC_CFGR2);
      regval &= ~RCC_CFGR2_PPRE3_MASK;
      regval |= STM32_RCC_CFGR2_PPRE3;
      putreg32(regval, STM32_RCC_CFGR2);

#ifdef CONFIG_STM32H5_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval = getreg32(STM32_RCC_CFGR1);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR1);
#endif

      /* Configure PLL123 clock source and multipiers */

/* PLL source: HSI, CSI, HSE
 * Pre-devider: M
 * Post-devider: P, Q, R
 *

/**
  * @brief  Macro to configures the main PLL (PLL1) clock source, multiplication and division factors.
  * @note   This function must be used only when the main PLL1 is disabled.
  *
  * @param  __PLL1SOURCE__: specifies the PLL entry clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_PLL1_SOURCE_CSI: CSI oscillator clock selected as PLL1 clock entry
  *            @arg RCC_PLL1_SOURCE_HSI: HSI oscillator clock selected as PLL1 clock entry
  *            @arg RCC_PLL1_SOURCE_HSE: HSE oscillator clock selected as PLL1 clock entry
  * @note   This clock source (__PLL1SOURCE__) is the clock source for PLL1 (main PLL) and is different
            from PLL2 & PLL3 clock sources.
  *
  * @param  __PLL1M__: specifies the division factor for PLL VCO input clock
  *          This parameter must be a number between 1 and 63.
  * @note   You have to set the PLL1M parameter correctly to ensure that the VCO input
  *         frequency ranges from 1 to 16 MHz.
  *
  * @param  __PLL1N__: specifies the multiplication factor for PLL VCO output clock
  *          This parameter must be a number between 4 and 512.
  * @note   You have to set the PLL1N parameter correctly to ensure that the VCO
  *         output frequency is between 150 and 420 MHz (when in medium VCO range) or
  *         between 192 and 836 MHZ (when in wide VCO range)
  *
  * @param  __PLL1P__: specifies the division factor for system  clock.
  *          This parameter must be a number between 2 and 128 (where odd numbers not allowed)
  *
  * @param  __PLL1Q__: specifies the division factor for peripheral kernel clocks
  *          This parameter must be a number between 1 and 128
  *
  * @param  __PLL1R__: specifies the division factor for peripheral kernel clocks
  *          This parameter must be a number between 1 and 128
  *
  * @retval None
  */
#define __HAL_RCC_PLL1_CONFIG(__PLL1SOURCE__, __PLL1M__, __PLL1N__, __PLL1P__, __PLL1Q__, __PLL1R__) \
  do{ MODIFY_REG(RCC->PLL1CFGR, (RCC_PLL1CFGR_PLL1SRC | RCC_PLL1CFGR_PLL1M), \
                   ((__PLL1SOURCE__) << RCC_PLL1CFGR_PLL1SRC_Pos) | ((__PLL1M__) << RCC_PLL1CFGR_PLL1M_Pos));\
    WRITE_REG(RCC->PLL1DIVR , ( (((__PLL1N__) - 1U ) & RCC_PLL1DIVR_PLL1N) | \
                                ((((__PLL1P__) - 1U ) << RCC_PLL1DIVR_PLL1P_Pos) & RCC_PLL1DIVR_PLL1P) | \
                                ((((__PLL1Q__) - 1U) << RCC_PLL1DIVR_PLL1Q_Pos) & RCC_PLL1DIVR_PLL1Q) | \
                                ((((__PLL1R__) - 1U) << RCC_PLL1DIVR_PLL1R_Pos) & RCC_PLL1DIVR_PLL1R))); \
  } while(0)










    #if 0
#ifdef STM32_BOARD_USEHSI
      regval = (RCC_PLLCKSELR_PLLSRC_HSI |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#else /* if STM32_BOARD_USEHSE */
      regval = (RCC_PLLCKSELR_PLLSRC_HSE |
                STM32_PLLCFG_PLL1M |
                STM32_PLLCFG_PLL2M |
                STM32_PLLCFG_PLL3M);
#endif
      /* Each PLL offers 3 outputs with post-dividers (PLLxP/PLLxQ/PLLxR) */

      /* Configure PLL1 dividers */

      regval = (STM32_PLL1CFG_PLL1N |
                STM32_PLL1CFG_PLL1P |
                STM32_PLL1CFG_PLL1Q |
                STM32_PLL1CFG_PLL1R);
      putreg32(regval, STM32_RCC_PLL1DIVR);

      /* Configure PLL2 dividers */

      regval = (STM32_PLL2CFG_PLL2N |
                STM32_PLL2CFG_PLL2P |
                STM32_PLL2CFG_PLL2Q |
                STM32_PLL2CFG_PLL2R);
      putreg32(regval, STM32_RCC_PLL2DIVR);

      /* Configure PLL3 dividers */

      regval = (STM32_PLL3CFG_PLL3N |
                STM32_PLL3CFG_PLL3P |
                STM32_PLL3CFG_PLL3Q |
                STM32_PLL3CFG_PLL3R);
      putreg32(regval, STM32_RCC_PLL3DIVR);

      /* Configure PLLs */

      regval = (STM32_PLL1CFG_PLL1CFG |
                STM32_PLL1CFG_PLL2CFG |
                STM32_PLL1CFG_PLL3CFG);
      putreg32(regval, STM32_RCC_PLL1CFGR);

      regval = getreg32(STM32_RCC_CR);
#if defined(USE_PLL1)
      /* Enable the PLL1 */

      regval |= RCC_CR_PLL1ON;
#endif

#if defined(USE_PLL2)
      /* Enable the PLL2 */

      regval |= RCC_CR_PLL2ON;
#endif

#if defined(USE_PLL3)
      /* Enable the PLL3 */

      regval |= RCC_CR_PLL3ON;
#endif
      putreg32(regval, STM32_RCC_CR);

#if defined(USE_PLL1)
      /* Wait until the PLL1 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL1RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL2)
      /* Wait until the PLL2 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL2RDY) == 0)
        {
        }
#endif

#if defined(USE_PLL3)
      /* Wait until the PLL3 is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLL3RDY) == 0)
        {
        }
#endif
      /* We must write the lower byte of the PWR_CR3 register is written once
       * after POR and it shall be written before changing VOS level or
       * ck_sys clock frequency. No limitation applies to the upper bytes.
       *
       * Programming data corresponding to an invalid combination of
       * LDOEN and BYPASS bits will be ignored: data will not be written,
       * the written-once mechanism will lock the register and any further
       * write access will be ignored. The default supply configuration will
       * be kept and the ACTVOSRDY bit in PWR control status register 1
       * (PWR_CSR1) will go on indicating invalid voltage levels.
       *
       * N.B. The system shall be power cycled before writing a new value.
       */

#if defined(CONFIG_STM32H5_PWR_DIRECT_SMPS_SUPPLY)
      regval = getreg32(STM32_PWR_CR3);
      regval &= ~(STM32_PWR_CR3_BYPASS | STM32_PWR_CR3_LDOEN |
          STM32_PWR_CR3_SMPSEXTHP | STM32_PWR_CR3_SMPSLEVEL_MASK);
      regval |= STM32_PWR_CR3_SCUEN;
      putreg32(regval, STM32_PWR_CR3);
#else
      regval = getreg32(STM32_PWR_CR3);
      regval |= STM32_PWR_CR3_LDOEN | STM32_PWR_CR3_SCUEN;
      putreg32(regval, STM32_PWR_CR3);
#endif

      /* Set the voltage output scale */

      regval = getreg32(STM32_PWR_D3CR);
      regval &= ~STM32_PWR_D3CR_VOS_MASK;
      regval |= STM32_PWR_VOS_SCALE;
      putreg32(regval, STM32_PWR_D3CR);

      while ((getreg32(STM32_PWR_D3CR) & STM32_PWR_D3CR_VOSRDY) == 0)
        {
        }

      /* See Reference manual Section 5.4.1, System supply startup */

      while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ACTVOSRDY) == 0)
        {
        }

      /* Configure FLASH wait states */

      regval = FLASH_ACR_WRHIGHFREQ(BOARD_FLASH_PROGDELAY) |
               FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);

      putreg32(regval, STM32_FLASH_ACR);
    #endif
    
#if 0
      /* Select the PLL1P as system clock source */

      regval = getreg32(STM32_RCC_CFGR1);
      regval &= ~RCC_CFGR1_SW_MASK;
      regval |= RCC_CFGR1_SW_PLL1;
      putreg32(regval, STM32_RCC_CFGR1);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_SWS_MASK) !=
             RCC_CFGR1_SWS_PLL1)
        {
        }
  #endif
    }
#endif // #if 0
}
