/***************************************************************************
 * arch/arm64/src/qemu/qemu_serial.c
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
 ***************************************************************************/

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>
#include <nuttx/serial/serial.h>

#include "arm64_arch.h"
#include "arm64_internal.h"
#include "qemu_serial.h"
#include "arm64_arch_timer.h"
#include "qemu_boot.h"
#include "arm64_gic.h"

#ifdef USE_SERIALDRIVER

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

// Use PinePhone Allwinner A64 UART (instead of QEMU PL011)
#define PINEPHONE_UART

// UART0 IRQ Number for PinePhone Allwinner A64 UART
#define UART_IRQ 32

// UART0 Base Address for PinePhone Allwinner A64 UART
#define UART_BASE_ADDRESS 0x01C28000

/* Which UART with be tty0/console and which tty1-4?  The console will
 * always be ttyS0.  If there is no console then will use the lowest
 * numbered UART.
 */

/* First pick the console and ttys0.  This could be any of UART1-5 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port         /* UART1 is console */
#  define TTYS0_DEV       g_uart1port         /* UART1 is ttyS0 */
#  define UART1_ASSIGNED  1
#endif

#ifndef PINEPHONE_UART
#define PL011_BIT_MASK(x, y)  (((2 << (x)) - 1) << (y))

/* PL011 Uart Flags Register */
#define PL011_FR_CTS                    BIT(0)  /* clear to send - inverted */
#define PL011_FR_DSR                    BIT(1)  /* data set ready - inverted
                                                 */
#define PL011_FR_DCD                    BIT(2)  /* data carrier detect -
                                                 * inverted */
#define PL011_FR_BUSY                   BIT(3)  /* busy transmitting data */
#define PL011_FR_RXFE                   BIT(4)  /* receive FIFO empty */
#define PL011_FR_TXFF                   BIT(5)  /* transmit FIFO full */
#define PL011_FR_RXFF                   BIT(6)  /* receive FIFO full */
#define PL011_FR_TXFE                   BIT(7)  /* transmit FIFO empty */
#define PL011_FR_RI                     BIT(8)  /* ring indicator - inverted */

/* PL011 Integer baud rate register */
#define PL011_IBRD_BAUD_DIVINT_MASK     0xff /* 16 bits of divider */

/* PL011 Fractional baud rate register */
#define PL011_FBRD_BAUD_DIVFRAC         0x3f
#define PL011_FBRD_WIDTH                6u

/* PL011 Receive status register / error clear register */
#define PL011_RSR_ECR_FE                BIT(0)  /* framing error */
#define PL011_RSR_ECR_PE                BIT(1)  /* parity error */
#define PL011_RSR_ECR_BE                BIT(2)  /* break error */
#define PL011_RSR_ECR_OE                BIT(3)  /* overrun error */

#define PL011_RSR_ERROR_MASK            (PL011_RSR_ECR_FE | PL011_RSR_ECR_PE | \
                                         PL011_RSR_ECR_BE | PL011_RSR_ECR_OE)

/* PL011 Line Control Register  */
#define PL011_LCRH_BRK                  BIT(0)  /* send break */
#define PL011_LCRH_PEN                  BIT(1)  /* enable parity */
#define PL011_LCRH_EPS                  BIT(2)  /* select even parity */
#define PL011_LCRH_STP2                 BIT(3)  /* select two stop bits */
#define PL011_LCRH_FEN                  BIT(4)  /* enable FIFOs */
#define PL011_LCRH_WLEN_SHIFT           5       /* word length */
#define PL011_LCRH_WLEN_WIDTH           2
#define PL011_LCRH_SPS                  BIT(7)  /* stick parity bit */

#define PL011_LCRH_WLEN_SIZE(x)         ((x) - 5)

#define PL011_LCRH_FORMAT_MASK          (PL011_LCRH_PEN | PL011_LCRH_EPS |     \
                                         PL011_LCRH_SPS |                      \
                                         PL011_BIT_MASK(PL011_LCRH_WLEN_WIDTH, \
                                                        PL011_LCRH_WLEN_SHIFT))

#define PL011_LCRH_PARTIY_EVEN          (PL011_LCRH_PEN | PL011_LCRH_EPS)
#define PL011_LCRH_PARITY_ODD           (PL011_LCRH_PEN)
#define PL011_LCRH_PARITY_NONE          (0)

/* PL011 Control Register */
#define PL011_CR_UARTEN                 BIT(0)  /* enable uart operations */
#define PL011_CR_SIREN                  BIT(1)  /* enable IrDA SIR */
#define PL011_CR_SIRLP                  BIT(2)  /* IrDA SIR low power mode */
#define PL011_CR_LBE                    BIT(7)  /* loop back enable */
#define PL011_CR_TXE                    BIT(8)  /* transmit enable */
#define PL011_CR_RXE                    BIT(9)  /* receive enable */
#define PL011_CR_DTR                    BIT(10) /* data transmit ready */
#define PL011_CR_RTS                    BIT(11) /* request to send */
#define PL011_CR_Out1                   BIT(12)
#define PL011_CR_Out2                   BIT(13)
#define PL011_CR_RTSEn                  BIT(14) /* RTS hw flow control enable
                                                 */
#define PL011_CR_CTSEn                  BIT(15) /* CTS hw flow control enable
                                                 */

/* PL011 Interrupt Fifo Level Select Register */
#define PL011_IFLS_TXIFLSEL_SHIFT       0   /* bits 2:0 */
#define PL011_IFLS_TXIFLSEL_WIDTH       3
#define PL011_IFLS_RXIFLSEL_SHIFT       3   /* bits 5:3 */
#define PL011_IFLS_RXIFLSEL_WIDTH       3

/* PL011 Interrupt Mask Set/Clear Register */
#define PL011_IMSC_RIMIM                BIT(0)  /* RTR modem interrupt mask */
#define PL011_IMSC_CTSMIM               BIT(1)  /* CTS modem interrupt mask */
#define PL011_IMSC_DCDMIM               BIT(2)  /* DCD modem interrupt mask */
#define PL011_IMSC_DSRMIM               BIT(3)  /* DSR modem interrupt mask */
#define PL011_IMSC_RXIM                 BIT(4)  /* receive interrupt mask */
#define PL011_IMSC_TXIM                 BIT(5)  /* transmit interrupt mask */
#define PL011_IMSC_RTIM                 BIT(6)  /* receive timeout interrupt
                                                 * mask */
#define PL011_IMSC_FEIM                 BIT(7)  /* framing error interrupt
                                                 * mask */
#define PL011_IMSC_PEIM                 BIT(8)  /* parity error interrupt mask
                                                 */
#define PL011_IMSC_BEIM                 BIT(9)  /* break error interrupt mask
                                                 */
#define PL011_IMSC_OEIM                 BIT(10) /* overrun error interrupt
                                                 * mask */

#define PL011_IMSC_ERROR_MASK           (PL011_IMSC_FEIM |                   \
                                         PL011_IMSC_PEIM | PL011_IMSC_BEIM | \
                                         PL011_IMSC_OEIM)

#define PL011_IMSC_MASK_ALL             (PL011_IMSC_OEIM | PL011_IMSC_BEIM | \
                                         PL011_IMSC_PEIM | PL011_IMSC_FEIM | \
                                         PL011_IMSC_RIMIM |                  \
                                         PL011_IMSC_CTSMIM |                 \
                                         PL011_IMSC_DCDMIM |                 \
                                         PL011_IMSC_DSRMIM |                 \
                                         PL011_IMSC_RXIM | PL011_IMSC_TXIM | \
                                         PL011_IMSC_RTIM)
#endif  //  !PINEPHONE_UART

/***************************************************************************
 * Private Types
 ***************************************************************************/

/* UART PL011 register map structure */

struct pl011_regs
{
  uint32_t dr;   /* data register */
  union
  {
    uint32_t rsr;
    uint32_t ecr;
  };

  uint32_t reserved_0[4];
  uint32_t fr;   /* flags register */
  uint32_t reserved_1;
  uint32_t ilpr;
  uint32_t ibrd;
  uint32_t fbrd;
  uint32_t lcr_h;
  uint32_t cr;
  uint32_t ifls;
  uint32_t imsc;
  uint32_t ris;
  uint32_t mis;
  uint32_t icr;
  uint32_t dmacr;
};

struct pl011_config
{
  volatile struct pl011_regs *uart;
  uint32_t sys_clk_freq;
};

/* Device data structure */

struct pl011_data
{
  uint32_t baud_rate;
  bool sbsa;
};

struct pl011_uart_port_s
{
  struct pl011_data data;
  struct pl011_config config;
  unsigned int irq_num;
  bool is_console;
};

/***************************************************************************
 * Private Functions
 ***************************************************************************/

#ifdef PINEPHONE_UART
//  Declarations for PinePhone A64 UART
static int a64_uart_setup(struct uart_dev_s *dev);
static void a64_uart_shutdown(struct uart_dev_s *dev);
static int a64_uart_attach(struct uart_dev_s *dev);
static void a64_uart_detach(struct uart_dev_s *dev);
static int a64_uart_ioctl(struct file *filep, int cmd, unsigned long arg);
static int a64_uart_receive(struct uart_dev_s *dev, unsigned int *status);
static void a64_uart_rxint(struct uart_dev_s *dev, bool enable);
static bool a64_uart_rxavailable(struct uart_dev_s *dev);
static void a64_uart_send(struct uart_dev_s *dev, int ch);
static void a64_uart_txint(struct uart_dev_s *dev, bool enable);
static bool a64_uart_txready(struct uart_dev_s *dev);
static bool a64_uart_txempty(struct uart_dev_s *dev);
static int a64_uart_irq_handler(int irq, void *context, void *arg);
#endif  //  PINEPHONE_UART

#ifndef PINEPHONE_UART

static void pl011_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->cr |= PL011_CR_UARTEN;
}

static void pl011_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->cr &= ~PL011_CR_UARTEN;
}

static void pl011_enable_fifo(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->lcr_h |= PL011_LCRH_FEN;
}

static void pl011_disable_fifo(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->lcr_h &= ~PL011_LCRH_FEN;
}

static int pl011_set_baudrate(const struct pl011_uart_port_s *sport,
                              uint32_t clk, uint32_t baudrate)
{
  const struct pl011_config *config = &sport->config;

  /* Avoiding float calculations, bauddiv is left shifted by 6 */

  uint64_t bauddiv =
      (((uint64_t)clk) << PL011_FBRD_WIDTH) / (baudrate * 16U);

  /* Valid bauddiv value
   * uart_clk (min) >= 16 x baud_rate (max)
   * uart_clk (max) <= 16 x 65535 x baud_rate (min)
   */

  if ((bauddiv < (1U << PL011_FBRD_WIDTH)) ||
      (bauddiv > (65535U << PL011_FBRD_WIDTH)))
    {
      return -EINVAL;
    }

  config->uart->ibrd    = bauddiv >> PL011_FBRD_WIDTH;
  config->uart->fbrd    = bauddiv & ((1U << PL011_FBRD_WIDTH) - 1U);

  ARM64_DMB();

  /* In order to internally update the contents of ibrd or fbrd, a
   * lcr_h write must always be performed at the end
   * ARM DDI 0183F, Pg 3-13
   */

  config->uart->lcr_h = config->uart->lcr_h;

  return 0;
}

static void pl011_irq_tx_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_TXIM;
}

static void pl011_irq_tx_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~PL011_IMSC_TXIM;
}

static void pl011_irq_rx_enable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc |= PL011_IMSC_RXIM | PL011_IMSC_RTIM;
}

static void pl011_irq_rx_disable(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  config->uart->imsc &= ~(PL011_IMSC_RXIM | PL011_IMSC_RTIM);
}

static int pl011_irq_tx_complete(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;

  /* check for TX FIFO empty */

  return config->uart->fr & PL011_FR_TXFE;
}

static int pl011_irq_rx_ready(const struct pl011_uart_port_s *sport)
{
  const struct pl011_config *config = &sport->config;
  const struct pl011_data   *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_RXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_RXIM) &&
         (!(config->uart->fr & PL011_FR_RXFE));
}

/***************************************************************************
 * Name: qemu_pl011_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ***************************************************************************/

static bool qemu_pl011_txready(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;

  if (!data->sbsa && !(config->uart->cr & PL011_CR_TXE))
    {
      return false;
    }

  return (config->uart->imsc & PL011_IMSC_TXIM) &&
         pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: qemu_pl011_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ***************************************************************************/

static bool qemu_pl011_txempty(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  return pl011_irq_tx_complete(sport);
}

/***************************************************************************
 * Name: qemu_pl011_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ***************************************************************************/

static void qemu_pl011_send(struct uart_dev_s *dev, int ch)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;

  config->uart->dr = ch;
}

/***************************************************************************
 * Name: qemu_pl011_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ***************************************************************************/

static bool qemu_pl011_rxavailable(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;

  if (!data->sbsa &&
      (!(config->uart->cr & PL011_CR_UARTEN) ||
       !(config->uart->cr & PL011_CR_RXE)))
    {
      return false;
    }

  return (config->uart->fr & PL011_FR_RXFE) == 0U;
}

/***************************************************************************
 * Name: qemu_pl011_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ***************************************************************************/

static void qemu_pl011_rxint(struct uart_dev_s *dev, bool enable)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  if (enable)
    {
      pl011_irq_rx_enable(sport);
    }
  else
    {
      pl011_irq_rx_disable(sport);
    }
}

/***************************************************************************
 * Name: qemu_pl011_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ***************************************************************************/

static void qemu_pl011_txint(struct uart_dev_s *dev, bool enable)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  if (enable)
    {
      pl011_irq_tx_enable(sport);
    }
  else
    {
      pl011_irq_tx_disable(sport);
    }
}

/***************************************************************************
 * Name: qemu_pl011_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ***************************************************************************/

static int qemu_pl011_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  unsigned int              rx;

  rx = config->uart->dr;

  *status = 0;

  return rx;
}

/***************************************************************************
 * Name: qemu_pl011_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *   for current qemu configure,
 *
 ***************************************************************************/

static int qemu_pl011_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;
  UNUSED(filep);
  UNUSED(arg);

  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

/***************************************************************************
 * Name: qemu_pl011_irq_handler (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It should cal
 *   uart_transmitchars or uart_receivechar to perform the appropriate data
 *   transfers.
 *
 ***************************************************************************/

static int qemu_pl011_irq_handler(int irq, void *context, void *arg)
{
  struct uart_dev_s         *dev = (struct uart_dev_s *)arg;
  struct pl011_uart_port_s  *sport;
  UNUSED(irq);
  UNUSED(context);

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  sport = (struct pl011_uart_port_s *)dev->priv;

  if (pl011_irq_rx_ready(sport))
    {
      uart_recvchars(dev);
    }

  if (qemu_pl011_txready(dev))
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/***************************************************************************
 * Name: qemu_pl011_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ***************************************************************************/

static void qemu_pl011_detach(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s *sport = (struct pl011_uart_port_s *)dev->priv;

  up_disable_irq(sport->irq_num);
  irq_detach(sport->irq_num);
}

/***************************************************************************
 * Name: qemu_pl011_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.
 *   This method is called when the serial port is opened.
 *   Normally, this is just after the setup() method is called,
 *   however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method
 *   (unless the hardware supports multiple levels of interrupt
 *   enabling).  The RX and TX interrupts are not enabled until
 *   the txint() and rxint() methods are called.
 *
 ***************************************************************************/

static int qemu_pl011_attach(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport;
  struct pl011_data         *data;
  int                       ret;

  sport = (struct pl011_uart_port_s *)dev->priv;
  data  = &sport->data;

  ret = irq_attach(sport->irq_num, qemu_pl011_irq_handler, dev);
  arm64_gic_irq_set_priority(sport->irq_num, IRQ_TYPE_LEVEL, 0);

  if (ret == OK)
    {
      up_enable_irq(sport->irq_num);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }

  if (!data->sbsa)
    {
      pl011_enable(sport);
    }

  return ret;
}

/***************************************************************************
 * Name: qemu_pl011_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ***************************************************************************/

static void qemu_pl011_shutdown(struct uart_dev_s *dev)
{
  UNUSED(dev);
  sinfo("%s: call unexpected\n", __func__);
}

static int qemu_pl011_setup(struct uart_dev_s *dev)
{
  struct pl011_uart_port_s  *sport  = (struct pl011_uart_port_s *)dev->priv;
  const struct pl011_config *config = &sport->config;
  struct pl011_data         *data   = &sport->data;
  int                       ret;
  uint32_t                  lcrh;
  irqstate_t                i_flags;

  i_flags = up_irq_save();

  /* If working in SBSA mode, we assume that UART is already configured,
   * or does not require configuration at all (if UART is emulated by
   * virtualization software).
   */

  if (!data->sbsa)
    {
      /* disable the uart */

      pl011_disable(sport);
      pl011_disable_fifo(sport);

      /* Set baud rate */

      ret = pl011_set_baudrate(sport, config->sys_clk_freq,
                               data->baud_rate);
      if (ret != 0)
        {
          up_irq_restore(i_flags);
          return ret;
        }

      /* Setting the default character format */

      lcrh  = config->uart->lcr_h & ~(PL011_LCRH_FORMAT_MASK);
      lcrh  &= ~(BIT(0) | BIT(7));
      lcrh  |= PL011_LCRH_WLEN_SIZE(8) << PL011_LCRH_WLEN_SHIFT;
      config->uart->lcr_h = lcrh;

      /* Enabling the FIFOs */

      pl011_enable_fifo(sport);
    }

  /* initialize all IRQs as masked */

  config->uart->imsc    = 0U;
  config->uart->icr     = PL011_IMSC_MASK_ALL;

  if (!data->sbsa)
    {
      config->uart->dmacr = 0U;
      ARM64_ISB();
      config->uart->cr  &= ~(BIT(14) | BIT(15) | BIT(1));
      config->uart->cr  |= PL011_CR_RXE | PL011_CR_TXE;
      ARM64_ISB();
    }

  up_irq_restore(i_flags);

  return 0;
}

#endif  //  !PINEPHONE_UART

/***************************************************************************
 * Private Data
 ***************************************************************************/

/* Serial driver UART operations */

#ifndef PINEPHONE_UART
//  Serial driver UART operations for QEMU PL011
static const struct uart_ops_s g_uart_ops =
{
  .setup    = qemu_pl011_setup,
  .shutdown = qemu_pl011_shutdown,
  .attach   = qemu_pl011_attach,
  .detach   = qemu_pl011_detach,
  .ioctl    = qemu_pl011_ioctl,
  .receive  = qemu_pl011_receive,
  .rxint    = qemu_pl011_rxint,
  .rxavailable = qemu_pl011_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = qemu_pl011_send,
  .txint    = qemu_pl011_txint,
  .txready  = qemu_pl011_txready,
  .txempty  = qemu_pl011_txempty,
};
#endif  //  !PINEPHONE_UART

#ifdef PINEPHONE_UART
//  Serial driver UART operations for PinePhone Allwinner A64 UART
static const struct uart_ops_s g_uart_ops =
{
  .setup    = a64_uart_setup,
  .shutdown = a64_uart_shutdown,
  .attach   = a64_uart_attach,
  .detach   = a64_uart_detach,
  .ioctl    = a64_uart_ioctl,
  .receive  = a64_uart_receive,
  .rxint    = a64_uart_rxint,
  .rxavailable = a64_uart_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol    = NULL,
#endif
  .send     = a64_uart_send,
  .txint    = a64_uart_txint,
  .txready  = a64_uart_txready,
  .txempty  = a64_uart_txempty,
};
#endif  //  PINEPHONE_UART

/* This describes the state of the uart1 port. */

static struct pl011_uart_port_s g_uart1priv =
{
  .data   =
    {
      .baud_rate  = CONFIG_UART1_BAUD,
      .sbsa       = false,
    },

  .config =
    {
      .uart           = (volatile struct pl011_regs *)CONFIG_QEMU_UART_BASE,
      .sys_clk_freq   = 24000000,
    },

    .irq_num       = CONFIG_QEMU_UART_IRQ,
    .is_console   = 1,
};

/* I/O buffers */

#ifdef CONFIG_QEMU_UART_PL011

static char                 g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char                 g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

static struct uart_dev_s    g_uart1port =
{
  .recv  =
    {
      .size   = CONFIG_UART1_RXBUFSIZE,
      .buffer = g_uart1rxbuffer,
    },

  .xmit  =
    {
      .size   = CONFIG_UART1_TXBUFSIZE,
      .buffer = g_uart1txbuffer,
    },

  .ops   = &g_uart_ops,
  .priv  = &g_uart1priv,
};

#endif

/***************************************************************************
 * Public Functions
 ***************************************************************************/

/***************************************************************************
 * Name: qemu_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before arm_serialinit.
 *
 ***************************************************************************/

void qemu_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function imx8_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
#ifndef PINEPHONE_UART
  qemu_pl011_setup(&CONSOLE_DEV);
#endif  //  !PINEPHONE_UART

#ifdef PINEPHONE_UART
  a64_uart_setup(&CONSOLE_DEV);
#endif  //  PINEPHONE_UART
#endif
}

/* Used to assure mutually exclusive access up_putc() */

/* static sem_t g_putc_lock = SEM_INITIALIZER(1); */

/***************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc((uint8_t)ch);
  return ch;
}

/***************************************************************************
 * Name: arm64_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that imx_earlyserialinit was called previously.
 *
 ***************************************************************************/

void arm64_serialinit(void)
{
  int ret;

  ret = uart_register("/dev/console", &CONSOLE_DEV);
  if (ret < 0)
    {
      sinfo("error at register dev/console, ret =%d\n", ret);
    }

  ret = uart_register("/dev/ttyS0", &TTYS0_DEV);

  if (ret < 0)
    {
      sinfo("error at register dev/ttyS0, ret =%d\n", ret);
    }
}

///////////////////////////////////////////////////////////////////////////////
// PinePhone Allwinner A64 UART

#ifdef PINEPHONE_UART

// Setup PinePhone Allwinner A64 UART
static int a64_uart_setup(struct uart_dev_s *dev)
{
  return 0;
}

// Shutdown PinePhone Allwinner A64 UART
static void a64_uart_shutdown(struct uart_dev_s *dev)
{
  // Should never be called
  UNUSED(dev);
  sinfo("%s: call unexpected\n", __func__);
}

// Attach Interrupt Handler for PinePhone Allwinner A64 UART
static int a64_uart_attach(struct uart_dev_s *dev)
{
  int ret;

  // Attach UART Interrupt Handler
  ret = irq_attach(UART_IRQ, a64_uart_irq_handler, dev);

  // Set Interrupt Priority in GIC v2
  arm64_gic_irq_set_priority(UART_IRQ, IRQ_TYPE_LEVEL, 0);

  // Enable UART Interrupt
  if (ret == OK)
    {
      up_enable_irq(UART_IRQ);
    }
  else
    {
      sinfo("error ret=%d\n", ret);
    }
  return ret;
}

// Detach Interrupt Handler for PinePhone Allwinner A64 UART
static void a64_uart_detach(struct uart_dev_s *dev)
{
  // Disable UART Interrupt
  up_disable_irq(UART_IRQ);

  // Detach UART Interrupt
  irq_detach(UART_IRQ);
}

// I/O Control for PinePhone Allwinner A64 UART
static int a64_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;
  UNUSED(filep);
  UNUSED(arg);
  switch (cmd)
    {
      case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }
  return ret;
}

// Receive data from PinePhone Allwinner A64 UART
static int a64_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
  // Read from UART Receiver Buffer Register (UART_RBR)
  // Offset: 0x0000
  const uint8_t *uart_rbr = (const uint8_t *) (UART_BASE_ADDRESS + 0x00);

  // Data byte received on the serial input port . The data in this register is
  // valid only if the Data Ready (DR) bit in the UART Line Status Register
  // (UART_LCR) is set.
  //
  // If in FIFO mode and FIFOs are enabled (UART_FCR[0] set to one), this
  // register accesses the head of the receive FIFO. If the receive FIFO is full
  // and this register is not read before the next data character arrives, then
  // the data already in the FIFO is preserved, but any incoming data are lost
  // and an overrun error occurs.
  return *uart_rbr;
}

// Enable or disable Receive Interrupts for PinePhone Allwinner A64 UART
static void a64_uart_rxint(struct uart_dev_s *dev, bool enable)
{
  // Write to UART Interrupt Enable Register (UART_IER)
  // Offset: 0x0004
  uint8_t *uart_ier = (uint8_t *) (UART_BASE_ADDRESS + 0x04);

  // Bit 0: Enable Received Data Available Interrupt (ERBFI)
  // This is used to enable/disable the generation of Received Data Available Interrupt and the Character Timeout Interrupt (if in FIFO mode and FIFOs enabled). These are the second highest priority interrupts.
  // 0: Disable
  // 1: Enable
  if (enable) { *uart_ier |= 0b00000001; }
  else        { *uart_ier &= 0b11111110; }
}

// Return true if Receive FIFO is not empty for PinePhone Allwinner A64 UART
static bool a64_uart_rxavailable(struct uart_dev_s *dev)
{
  // Read from UART Line Status Register (UART_LSR)
  // Offset: 0x0014
  const uint8_t *uart_lsr = (const uint8_t *) (UART_BASE_ADDRESS + 0x14);

  // Bit 0: Data Ready (DR)
  // This is used to indicate that the receiver contains at least one character in
  // the RBR or the receiver FIFO.
  // 0: no data ready
  // 1: data ready
  // This bit is cleared when the RBR is read in non-FIFO mode, or when the
  // receiver FIFO is empty, in FIFO mode.
  return (*uart_lsr) & 1;  // DR=1 if data is ready
}

// Send one byte to PinePhone Allwinner A64 UART
static void a64_uart_send(struct uart_dev_s *dev, int ch)
{
  uint8_t *uart0_base_address = (uint8_t *) UART_BASE_ADDRESS;
  *uart0_base_address = ch;
}

// Enable or disable Transmit Interrupts for PinePhone Allwinner A64 UART
static void a64_uart_txint(struct uart_dev_s *dev, bool enable)
{
  // Write to UART Interrupt Enable Register (UART_IER)
  // Offset: 0x0004
  uint8_t *uart_ier = (uint8_t *) (UART_BASE_ADDRESS + 0x04);

  // Bit 1: Enable Transmit Holding Register Empty Interrupt (ETBEI)
  // This is used to enable/disable the generation of Transmitter Holding Register Empty Interrupt. This is the third highest priority interrupt.
  // 0: Disable
  // 1: Enable
  if (enable) { *uart_ier |= 0b00000010; }
  else        { *uart_ier &= 0b11111101; }
}

// Return true if Transmit FIFO is not full for PinePhone Allwinner A64 UART
static bool a64_uart_txready(struct uart_dev_s *dev)
{
  // Read from UART_LSR
  // Offset: 0x0014
  const uint8_t *uart_lsr = (const uint8_t *) (UART_BASE_ADDRESS + 0x14);

  // Transmit FIFO is ready if THRE=1 (bit 5 of LSR)
  return (*uart_lsr & 0x20) != 0;
}

// Return true if Transmit FIFO is empty for PinePhone Allwinner A64 UART
static bool a64_uart_txempty(struct uart_dev_s *dev)
{
  return a64_uart_txready(dev);
}

// Interrupt Handler for PinePhone Allwinner A64 UART
static int a64_uart_irq_handler(int irq, void *context, void *arg)
{
  // Get the UART Device
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  UNUSED(irq);
  UNUSED(context);
  DEBUGASSERT(dev != NULL && dev->priv != NULL);

  // Read UART Interrupt Identity Register (UART_IIR)
  // Offset: 0x0008 
  const uint8_t *uart_iir = (const uint8_t *) (UART_BASE_ADDRESS + 0x08);

  // Bits 3:0: Interrupt ID
  // This indicates the highest priority pending interrupt which can be one of the following types:
  // 0000: modem status
  // 0001: no interrupt pending
  // 0010: THR empty
  // 0100: received data available
  // 0110: receiver line status
  // 0111: busy detect
  // 1100: character timeout
  // Bit 3 indicates an interrupt can only occur when the FIFOs are enabled and used to distinguish a Character Timeout condition interrupt.
  uint8_t int_id = (*uart_iir) & 0b1111;

  // 0100: If received data is available...
  if (int_id == 0b0100) {
    // Receive the data
    uart_recvchars(dev);

  // 0010: If THR is empty (Transmit Holding Register)...
  } else if (int_id == 0b0010) {
    // Transmit the data
    uart_xmitchars(dev);

  }
  return OK;
}

#endif  //  PINEPHONE_UART

#else /* USE_SERIALDRIVER */

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc((uint8_t)ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
