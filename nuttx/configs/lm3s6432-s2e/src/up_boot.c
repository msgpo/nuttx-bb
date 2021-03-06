/************************************************************************************
 * configs/lm3s6432-s2e/src/up_boot.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "up_internal.h"
#include "lm3s_internal.h"
#include "lm3s6432s2e_internal.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#if !defined(CONFIG_UART1_DISABLE) && !defined(CONFIG_SSI0_DISABLE)
#  error Only one of UART1 and SSI0 can be enabled on this board.
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lm3s_boardinitialize
 *
 * Description:
 *   All LM3S architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 ************************************************************************************/

void lm3s_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SSI is not disabled, and 2) the weak function
   * lm3s_ssiinitialize() has been brought into the link.
   */

#if !defined(CONFIG_SSI0_DISABLE)
  if (lm3s_ssiinitialize)
    {
      lm3s_ssiinitialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  up_ledinit();
#endif

  /* Configure serial transciever */
  
  lm3s_configgpio(XCVR_INV_GPIO);
  lm3s_configgpio(XCVR_ENA_GPIO);
  lm3s_configgpio(XCVR_ON_GPIO);
  lm3s_configgpio(XCVR_OFF_GPIO);
}
