/****************************************************************************
 * boards/risc-v/bl602/hifive1-revb/src/bl602_buttons.c
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

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <nuttx/arch.h>

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include "bl602_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define GPIO_BTN (GPIO_MODE_INIRQ | GPIO_INT_BOTH | GPIO_PULLUP | GPIO_PIN12)
//#define GPIO_BTN (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN12)
#define GPIO_BTN (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN12)
#define BTN_IRQ  GPIO_PIN12
//#define BTN_IRQ  BL602_IRQ_GPIO_INT0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
#warning  Maybe this is/should be done by bl602_gpio_initialize from board.h
//  uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

//  bl602_expander_set_intmod(BTN_IRQ, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
//  bl602_gpio_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);

//  bl602_configgpio(GPIO_BTN);

  return 1;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT  definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint8_t ret = 0;
  int i = 0;
  int n = 0;
printf("Starting BOARD_buttons()");
  bool b0 = bl602_gpioread(GPIO_BTN);

  for (i = 0; i < 10; i++)
    {
      up_mdelay(1); /* TODO */

      bool b1 = bl602_gpioread(GPIO_BTN);

      if (b0 == b1)
        {
          n++;
        }
      else
        {
          n = 0;
        }

      if (3 == n)
        {
          break;
        }

      b0 = b1;
    }

  iinfo("b=%d n=%d\n", b0, n);

  /* Low value means that the button is pressed */

  if (!b0)
    {
      ret = 0x1;
    }

printf("From BOARD_buttons() returning %d", ret);
  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;
  ASSERT(id == 0);
printf("Registering BUTTON irq handler@ %p ID %d, BTN_IRQ %d\n", irqhandler, id, BTN_IRQ);
  if (NULL != irqhandler)
    {
      /* Attach the new button handler. */

//      ret = irq_attach(BTN_IRQ, irqhandler, arg);
      ret = irq_attach(BL602_IRQ_GPIO_INT0, irqhandler, arg);
//      ret = irq_attach(BL602_IRQ_GPIO_INT0, irqhandler, arg);
//      bl602_gpio_intmask(BTN_IRQ, 0);



      /* Then make sure that interrupts are enabled on the pin */

      up_enable_irq(BTN_IRQ);
    }
  else
    {
      up_disable_irq(BTN_IRQ);
    }

  return ret;
}
#endif
