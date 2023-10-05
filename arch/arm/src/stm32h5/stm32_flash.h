/****************************************************************************
 * arch/arm/src/stm32h5/stm32_flash.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_flash.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32h5_flash_getopt
 *
 * Description:
 *   Returns the current flash option bytes from the FLASH_OPTSR_CR register.
 *
 ****************************************************************************/

uint32_t stm32h5_flash_getopt(void);

/****************************************************************************
 * Name: stm32h5_flash_optmodify
 *
 * Description:
 *   Modifies the current flash option bytes, given bits to set and clear.
 *
 ****************************************************************************/

void stm32h5_flash_optmodify(uint32_t clear, uint32_t set);

/****************************************************************************
 * Name: stm32h5_flash_swapbanks
 *
 * Description:
 *   Swaps banks 1 and 2 in the processor's memory map.  Takes effect
 *   the next time the system is reset.
 *
 ****************************************************************************/

void stm32h5_flash_swapbanks(void);

/****************************************************************************
 * Name: stm32h5_flash_lock
 *
 * Description:
 *   Locks a bank
 *
 ****************************************************************************/

int stm32h5_flash_lock(void);

/****************************************************************************
 * Name: stm32h5_flash_unlock
 *
 * Description:
 *   Unlocks a bank
 *
 ****************************************************************************/

int stm32h5_flash_unlock(void);

/****************************************************************************
 * Name: stm32h5_flash_writeprotect
 *
 * Description:
 *   Enable or disable the write protection of a flash sector.
 *
 ****************************************************************************/

int stm32h5_flash_writeprotect(size_t page, bool enabled);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_FLASH_H */
