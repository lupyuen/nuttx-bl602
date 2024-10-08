/****************************************************************************
 * boards/risc-v/bl602/bl602evb/include/board.h
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

#ifndef __BOARDS_RISCV_BL602_BL602EVB_INCLUDE_BOARD_H
#define __BOARDS_RISCV_BL602_BL602EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include BL602 header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin Definitions below must sync with bl602_bringup.c: bl602_gpio_inputs / outputs / interrupts / other_pins */

#ifndef CONFIG_IOEXPANDER_BL602_EXPANDER
/* GPIO Configuration (Not used for BL602 GPIO Expander) */

#define BOARD_NGPIOIN     1 /* Amount of GPIO Input pins */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOINT    1 /* Amount of GPIO Input w/ Interruption pins */

/* Busy Pin for PineDio SX1262 */

#define BOARD_GPIO_IN1    (GPIO_INPUT | GPIO_FLOAT | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN10)

/* SPI Chip Select for PineDio SX1262 */

#define BOARD_GPIO_OUT1   (GPIO_OUTPUT | GPIO_PULLUP | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN15)

/* GPIO Interrupt (DIO1) for PineDio SX1262 */

#define BOARD_GPIO_INT1   (GPIO_INPUT | GPIO_FLOAT | \
                            GPIO_FUNC_SWGPIO | GPIO_PIN19)
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */

#ifdef CONFIG_BL602_UART0
/* UART Configuration */

#define BOARD_UART_0_RX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN7)
#define BOARD_UART_0_TX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN16)
#endif  /* CONFIG_BL602_UART0 */

#ifdef TODO  /* Remember to check for duplicate pins! */
#define BOARD_UART_1_RX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN3)
#define BOARD_UART_1_TX_PIN (GPIO_INPUT | GPIO_PULLUP | \
                              GPIO_FUNC_UART | GPIO_PIN4)
#endif  /* TODO */

#ifdef TODO  /* Remember to check for duplicate pins! */
/* PWM Configuration */

#define BOARD_PWM_CH0_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN0)
#define BOARD_PWM_CH1_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN1)
#define BOARD_PWM_CH2_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN2)
#define BOARD_PWM_CH3_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN3)
#define BOARD_PWM_CH4_PIN (GPIO_OUTPUT | GPIO_PULLDOWN | \
                            GPIO_FUNC_PWM | GPIO_PIN4)
#endif  /* TODO */

#ifdef CONFIG_BL602_I2C0
/* I2C for PineDio Stack: SCL, SDA */

#define BOARD_I2C_SCL (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_I2C | GPIO_PIN2)
#define BOARD_I2C_SDA (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_I2C | GPIO_PIN1)
#endif  /* CONFIG_BL602_I2C0 */

#ifdef CONFIG_BL602_SPI0
/* SPI for PineDio Stack: Chip Select (unused), MOSI, MISO, SCK */

#define BOARD_SPI_CS   (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN8)  /* Unused */
#define BOARD_SPI_MOSI (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN13)
#define BOARD_SPI_MISO (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN0)
#define BOARD_SPI_CLK  (GPIO_INPUT | GPIO_PULLUP | GPIO_FUNC_SPI | GPIO_PIN11)
#endif  /* CONFIG_BL602_SPI0 */

#ifdef CONFIG_LCD_ST7789
/* ST7789 for PineDio Stack: Chip Select, Reset and Backlight */

#define BOARD_LCD_DEVID SPIDEV_DISPLAY(0)  /* SPI Device ID: 0x40000 */
#define BOARD_LCD_SWAP  0    /* Don't swap MISO/MOSI */
#define BOARD_LCD_BL_INVERT  /* Backlight is active when Low */
#define BOARD_LCD_CS  (GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN20)
#define BOARD_LCD_RST (GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN3)
#define BOARD_LCD_BL  (GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN21)
#endif  /* CONFIG_LCD_ST7789 */

#ifdef CONFIG_BL602_SPI0
/* SX1262 for PineDio Stack: Chip Select, Busy, DIO1 */

#define BOARD_SX1262_DEVID 1  /* SPI Device ID */
#define BOARD_SX1262_SWAP  1  /* Swap MISO/MOSI */
#define BOARD_SX1262_CS   (GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN15)
#define BOARD_SX1262_BUSY (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN10)
#define BOARD_SX1262_DIO1 (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN19)
#endif  /* CONFIG_BL602_SPI0 */

#ifdef CONFIG_BL602_SPI0
/* SPI Flash for PineDio Stack: Chip Select */

#define BOARD_FLASH_DEVID 2  /* SPI Device ID */
#define BOARD_FLASH_SWAP  1  /* Swap MISO/MOSI */
#define BOARD_FLASH_CS (GPIO_OUTPUT | GPIO_PULLUP | GPIO_FUNC_SWGPIO | GPIO_PIN14)
#endif  /* CONFIG_BL602_SPI0 */

#ifdef CONFIG_INPUT_CST816S
/* CST816S Touch Controller for PineDio Stack: GPIO Interrupt */

#define BOARD_TOUCH_INT (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN9)
#endif  /* CONFIG_INPUT_CST816S */

/* Push Button for PineDio Stack */

#define BOARD_BUTTON_INT (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN12)

/* Identify as PineDio Stack if both ST7789 and CST816S are present */

#if defined(CONFIG_LCD_ST7789) && defined(CONFIG_INPUT_CST816S)
#define PINEDIO_STACK_BL604
#endif /* CONFIG_LCD_ST7789 && CONFIG_INPUT_CST816S */

/* Columns in the SPI Device Table */

#define DEVID_COL 0  /* SPI Device ID */
#define SWAP_COL  1  /* 1 if MISO/MOSI should be swapped, else 0 */
#define CS_COL    2  /* SPI Chip Select Pin */
#define NUM_COLS  3  /* Number of columns in SPI Device Table */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: litex_boardinitialize
 ****************************************************************************/

void bl602_boardinitialize(void);

#ifdef CONFIG_BL602_SPI0
void bl602_spi_deselect_devices(void);
const int32_t *bl602_spi_get_device(uint32_t devid);
#endif  /* CONFIG_BL602_SPI0 */

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISC_V_BL602_BL602EVB_INCLUDE_BOARD_H */
