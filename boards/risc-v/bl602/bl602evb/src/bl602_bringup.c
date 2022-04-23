/****************************************************************************
 * boards/risc-v/bl602/bl602evb/src/bl602_bringup.c
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
#include <nuttx/timers/oneshot.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <syslog.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/buttons.h>
#include <nuttx/timers/rtc.h>
#include <bl602_tim_lowerhalf.h>
#include <bl602_oneshot_lowerhalf.h>
#include <bl602_pwm_lowerhalf.h>
#include <bl602_wdt_lowerhalf.h>
#include <bl602_glb.h>
#include <bl602_gpio.h>
#include <bl602_i2c.h>
#include <bl602_spi.h>
#include <bl602_rtc.h>

#if defined(CONFIG_BL602_SPIFLASH)
#include <bl602_spiflash.h>
#endif

#if defined(CONFIG_BL602_BLE_CONTROLLER)
#include <nuttx/kmalloc.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>
#include <nuttx/mm/circbuf.h>
#if defined(CONFIG_UART_BTH4)
#include <nuttx/serial/uart_bth4.h>
#endif
#endif /* CONFIG_BL602_BLE_CONTROLLER */

#ifdef CONFIG_FS_ROMFS
#include <nuttx/drivers/ramdisk.h>

#define BL602_XIP_START_ADDR    (0x23000000)
#define BL602_XIP_OFFSET        (*(volatile uint32_t *)0x4000B434)
#define BL602_ROMFS_FLASH_ADDR  (0x1C0000)
#define BL602_ROMFS_XIP_ADDR    (BL602_XIP_START_ADDR \
                                 + BL602_ROMFS_FLASH_ADDR \
                                 - BL602_XIP_OFFSET)
#endif /* CONFIG_FS_ROMFS */

#ifdef CONFIG_RF_SPI_TEST_DRIVER
#include <nuttx/rf/spi_test_driver.h>
#endif /* CONFIG_RF_SPI_TEST_DRIVER */

#ifdef CONFIG_SENSORS_BME280
#include <nuttx/sensors/bme280.h>
#endif /* CONFIG_SENSORS_BME280 */

#ifdef CONFIG_SENSORS_BMP280
#include <nuttx/sensors/bmp280.h>
#endif /* CONFIG_SENSORS_BMP280 */

#ifdef CONFIG_LCD_DEV
#  include <nuttx/board.h>
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_LCD_ST7789
#include <nuttx/lcd/st7789.h>
#include "../boards/risc-v/bl602/bl602evb/include/board.h"
#include "riscv_internal.h"
#endif /* CONFIG_LCD_ST7789 */

#ifdef CONFIG_INPUT_CST816S
/* I2C Address of CST816S Touch Controller */

#define CST816S_DEVICE_ADDRESS 0x15
#include <nuttx/input/cst816s.h>
#endif /* CONFIG_INPUT_CST816S */

#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/bl602_expander.h>
FAR struct ioexpander_dev_s *bl602_expander = NULL;
static int button_isr_handler(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, FAR void *arg); //// TODO
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */

#include "chip.h"
#include "../include/board.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BL602_SPI0
/* SPI Device Table: SPI Device ID, Swap MISO/MOSI, Chip Select */

static const int32_t bl602_spi_device_table[] =
{
#ifdef BOARD_LCD_DEVID  /* ST7789 Display */
  BOARD_LCD_DEVID, BOARD_LCD_SWAP, BOARD_LCD_CS,
#endif  /* BOARD_LCD_DEVID */

#ifdef BOARD_SX1262_DEVID  /* LoRa SX1262 */
  BOARD_SX1262_DEVID, BOARD_SX1262_SWAP, BOARD_SX1262_CS,
#endif  /* BOARD_SX1262_DEVID */

#ifdef BOARD_FLASH_DEVID  /* SPI Flash */
  BOARD_FLASH_DEVID, BOARD_FLASH_SWAP, BOARD_FLASH_CS,
#endif  /* BOARD_FLASH_DEVID */

  /* Must end with Default SPI Device */

  -1, 1, BOARD_SPI_CS,  /* Swap MISO/MOSI */
};
#endif  /* CONFIG_BL602_SPI0 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_BL602_SPI0
/****************************************************************************
 * Name: bl602_spi_validate_devices
 *
 * Description:
 *   Validate the SPI Device Table
 *
 ****************************************************************************/

static void bl602_spi_validate_devices(void)
{
  int len;
  int i;

  /* All columns must be populated */

  len = sizeof(bl602_spi_device_table) / sizeof(bl602_spi_device_table[0]);
  DEBUGASSERT(len % NUM_COLS == 0);

  /* Validate every row */

  for (i = 0; i < len; i += NUM_COLS)
    {
      int32_t devid;
      int32_t swap;
      
      devid = bl602_spi_device_table[i + DEVID_COL];
      swap = bl602_spi_device_table[i + SWAP_COL];

      /* Validate Device ID and Swap */

      DEBUGASSERT(devid >= -1);
      DEBUGASSERT(swap == 0 || swap == 1);
    }

  /* TODO: Verify that all Device IDs are unique */

}

/****************************************************************************
 * Name: bl602_spi_deselect_devices
 *
 * Description:
 *   Set Chip Select to High for all devices in the SPI Device Table
 *
 ****************************************************************************/

void bl602_spi_deselect_devices(void)
{
  int len;
  int i;

  /* Validate the SPI Device Table */

  bl602_spi_validate_devices();

  /* Get all devices in the SPI Device Table, including default device */

  len = sizeof(bl602_spi_device_table) / sizeof(bl602_spi_device_table[0]);
  for (i = 0; i < len; i += NUM_COLS)
    {
      int32_t cs;

      /* Configure Chip Select as GPIO and set to High */

      cs = bl602_spi_device_table[i + CS_COL];
      bl602_configgpio(cs);
      bl602_gpiowrite(cs, true);
    }
}

/****************************************************************************
 * Name: bl602_spi_get_device
 *
 * Description:
 *   Return the device from the SPI Device Table
 *
 ****************************************************************************/

const int32_t *bl602_spi_get_device(uint32_t devid)
{
  int len;
  int i;

  /* Find the device in the SPI Device Table, or return default device */

  len = sizeof(bl602_spi_device_table) / sizeof(bl602_spi_device_table[0]);
  for (i = 0; i < len; i += NUM_COLS)
    {
      int32_t id;
      
      id = bl602_spi_device_table[i + DEVID_COL];
      if (id == -1 || id == devid)
        {
          return &bl602_spi_device_table[i];
        }
    }

  DEBUGPANIC();  /* Never comes here */
  return NULL;
}
#endif  /* CONFIG_BL602_SPI0 */

#if defined(CONFIG_BL602_WIRELESS)
extern int bl602_net_initialize(void);
#endif

#if defined(CONFIG_BL602_BLE_CONTROLLER)
struct bthci_s
{
  struct bt_driver_s drv;
  int id;
  int fd;
  sq_entry_t link;
};

struct uart_rxchannel
{
  void (*callback)(void *, uint8_t);
  void *dummy;
  uint32_t remain_size;
  uint8_t *remain_data;
};

struct uart_env_tag
{
  struct uart_rxchannel rx;
};

static struct bthci_s *hci_dev;
static struct circbuf_s circbuf_rd;
static struct uart_env_tag uart_env;

static int bthci_send(struct bt_driver_s *drv,
                      enum bt_buf_type_e type,
                      void *data,
                      size_t len);
static int bthci_open(struct bt_driver_s *drv);
static void bthci_close(struct bt_driver_s *drv);
static int bthci_receive(uint8_t *data, uint32_t len);

static int bthci_register(void);
extern void rw_main_task_post_from_fw(void);
extern void bl602_hci_uart_api_init(void *ble_uart_read,
                                    void *ble_uart_write);

static void ble_uart_read(uint8_t *bufptr,
                          uint32_t size,
                          void (*callback)(void *, uint8_t),
                          void *dummy)
{
  irqstate_t flags;

  if (!bufptr || !size || !callback)
    return;

  if (circbuf_used(&circbuf_rd) >= size)
    {
      flags = enter_critical_section();
      size_t nread = circbuf_read(&circbuf_rd, bufptr, size);
      leave_critical_section(flags);
      if (nread != size)
        {
          printf("%s\n", __func__);
        }

      callback(dummy, 0);

      /* rw_main_task_post_from_fw(); */

      return;
    }

  uart_env.rx.callback = callback;
  uart_env.rx.dummy = dummy;
  uart_env.rx.remain_size = size;
  uart_env.rx.remain_data = bufptr;
}

static void ble_uart_write(const uint8_t *bufptr,
                           uint32_t size,
                           void (*callback)(void *, uint8_t),
                           void *dummy)
{
  if (!bufptr || !size || !callback)
    return;

  bthci_receive((uint8_t *)bufptr, size);

  callback(dummy, 0);

  return;
}

static int bthci_send(struct bt_driver_s *drv,
                      enum bt_buf_type_e type,
                      void *data,
                      size_t len)
{
  char *hdr = (char *)data - drv->head_reserve;
  void (*callback)(void *, uint8_t) = NULL;
  void *dummy = NULL;
  int nlen;
  int rlen;
  irqstate_t flags;

  if (type == BT_CMD)
    {
      *hdr = H4_CMD;
    }
  else if (type == BT_ACL_OUT)
    {
      *hdr = H4_ACL;
    }
  else if (type == BT_ISO_OUT)
    {
      *hdr = H4_ISO;
    }
  else
    {
      return -EINVAL;
    }

  /* Host send to controller */

  flags = enter_critical_section();
  nlen = circbuf_write(&circbuf_rd, hdr, len + H4_HEADER_SIZE);
  if (uart_env.rx.remain_size &&
      circbuf_used(&circbuf_rd) >= uart_env.rx.remain_size)
    {
      /* Read data */

      rlen = circbuf_read(&circbuf_rd,
                          uart_env.rx.remain_data,
                          uart_env.rx.remain_size);
      if (rlen < uart_env.rx.remain_size)
        {
          printf("bthci_send rlen is error\n");
        }

      /* printf("Rx len[%d]\n", len); */

      uart_env.rx.remain_data += rlen;
      uart_env.rx.remain_size -= rlen;

      callback = uart_env.rx.callback;
      dummy = uart_env.rx.dummy;

      if (callback != NULL && !uart_env.rx.remain_size)
        {
          uart_env.rx.callback = NULL;
          uart_env.rx.dummy = NULL;
          callback(dummy, 0);
        }
    }

  leave_critical_section(flags);

  return nlen;
}

static void bthci_close(struct bt_driver_s *drv)
{
}

static int bthci_receive(uint8_t *data, uint32_t len)
{
  enum bt_buf_type_e type;

  if (len <= 0)
    {
      return len;
    }

  if (data[0] == H4_EVT)
    {
      type = BT_EVT;
    }
  else if (data[0] == H4_ACL)
    {
      type = BT_ACL_IN;
    }
  else if (data[0] == H4_ISO)
    {
      type = BT_ISO_IN;
    }
  else
    {
      return -EINVAL;
    }

  return bt_netdev_receive(&hci_dev->drv,
                           type,
                           data + H4_HEADER_SIZE,
                           len - H4_HEADER_SIZE);
}

static int bthci_open(struct bt_driver_s *drv)
{
  return OK;
}

static struct bthci_s *bthci_alloc(void)
{
  /* Register the driver with the Bluetooth stack */

  struct bthci_s *dev;
  struct bt_driver_s *drv;

  dev = (struct bthci_s *)kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->id = 0;
  dev->fd = -1;
  drv = &dev->drv;
  drv->head_reserve = H4_HEADER_SIZE;
  drv->open = bthci_open;
  drv->send = bthci_send;
  drv->close = bthci_close;

  return dev;
}

int bthci_register(void)
{
  int ret;

  hci_dev = bthci_alloc();
  if (hci_dev == NULL)
    {
      return -ENOMEM;
    }

  #if defined(CONFIG_UART_BTH4)
  ret = uart_bth4_register("/dev/ttyHCI0", &hci_dev->drv);
  #elif defined(CONFIG_NET_BLUETOOTH)
  ret = bt_netdev_register(&hci_dev->drv);
  #elif defined(BL602_BLE_CONTROLLER)
    #error "Must select CONFIG_UART_BTH4 or CONFIG_NET_BLUETOOTH"
  #endif
  if (ret < 0)
    {
      printf("register faile[%d] errno %d\n", ret, errno);
      kmm_free(hci_dev);
    }

  return ret;
}

void bl602_hci_uart_init(uint8_t uartid)
{
  int ret;

  if (uartid)
    return;

  ret = circbuf_init(&circbuf_rd, NULL, 512);
  if (ret < 0)
    {
      circbuf_uninit(&circbuf_rd);
      return;
    }

  bl602_hci_uart_api_init(ble_uart_read, ble_uart_write);

  bthci_register();
  rw_main_task_post_from_fw();
  return;
}
#endif /* CONFIG_BL602_BLE_CONTROLLER */

/****************************************************************************
 * Name: bl602_bringup
 ****************************************************************************/

int bl602_bringup(void)
{
#if defined(CONFIG_TIMER) && defined(CONFIG_ONESHOT) && \
  defined(CONFIG_BL602_TIMER1)
  struct oneshot_lowerhalf_s *os = NULL;
#endif
#if defined(CONFIG_BL602_SPIFLASH)
  struct mtd_dev_s *mtd_part = NULL;
  const char *path = "/dev/mtdflash";
#endif
#ifdef CONFIG_I2C
  struct i2c_master_s *i2c_bus;
#endif
#ifdef CONFIG_SPI
  struct spi_dev_s *spi_bus;
#endif
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#if defined(CONFIG_TIMER)
#if defined(CONFIG_BL602_TIMER0)
  ret = bl602_timer_initialize("/dev/timer0", 0);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer0 Driver: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_BL602_TIMER1) && !defined(CONFIG_ONESHOT)
  ret = bl602_timer_initialize("/dev/timer1", 1);
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "Failed to initialize /dev/timer1 Driver: %d\n", ret);
      return ret;
    }
#elif defined(CONFIG_BL602_TIMER1) && defined(CONFIG_ONESHOT)
  os = oneshot_initialize(1, 1);
  if (os == NULL)
    {
      syslog(LOG_DEBUG, "ERROR: oneshot_initialize failed\n");
    }
  else
    {
#ifdef CONFIG_CPULOAD_ONESHOT
      /* Configure the oneshot timer to support CPU load measurement */

      nxsched_oneshot_extclk(os);

#else
      ret = oneshot_register("/dev/oneshot", os);
      if (ret < 0)
        {
          syslog(LOG_DEBUG,
            "ERROR: Failed to register oneshot at /dev/oneshot: %d\n", ret);
        }
#endif
    }
#endif
#endif

#ifdef CONFIG_PWM
  struct pwm_lowerhalf_s *pwm;

  /* Initialize PWM and register the PWM driver */

  pwm = bl602_pwminitialize(0);
  if (pwm == NULL)
    {
      syslog(LOG_DEBUG, "ERROR: bl602_pwminitialize failed\n");
    }
  else
    {
      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          syslog(LOG_DEBUG, "ERROR: pwm_register failed: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_WATCHDOG
  ret = bl602_wdt_initialize(CONFIG_WATCHDOG_DEVPATH);
  if (ret < 0)
    {
      syslog(LOG_DEBUG, "ERROR: bl602_wdt_initialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = bl602_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_IOEXPANDER_BL602_EXPANDER
  /* Must load BL602 GPIO Expander before other drivers */

  bl602_expander = bl602_expander_initialize();
  if (bl602_expander == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Expander: %d\n", ret);
      return -ENOMEM;
    }

  /* Register pin drivers */

  /* Touch Panel (GPIO 9): a non-inverted, falling-edge interrupting pin */
  {
    gpio_pinset_t pinset = BOARD_TOUCH_INT;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);
  }

  /* Push Button (GPIO 12): a non-inverted, falling-edge interrupting pin */
  {
    #define BOARD_BUTTON_INT (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN12)
    gpio_pinset_t pinset = BOARD_BUTTON_INT;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);

    #warning TODO: Move IOEP_ATTACH to Button Handler
    // void *handle = IOEP_ATTACH(bl602_expander,
    //                            (ioe_pinset_t)1 << gpio_pin,
    //                            button_isr_handler,
    //                            NULL);  ////  TODO
    // DEBUGASSERT(handle != NULL);
  }

  /* SX1262 Busy (GPIO 10): a non-inverted, input pin */
  {
    #define BOARD_SX1262_BUSY (GPIO_INPUT | GPIO_FLOAT | GPIO_FUNC_SWGPIO | GPIO_PIN10)
    gpio_pinset_t pinset = BOARD_SX1262_BUSY;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INPUT_PIN, gpio_pin);
  }

  /* SX1262 Chip Select (GPIO 15): a non-inverted, output pin */
  {
    gpio_pinset_t pinset = BOARD_SX1262_CS;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_OUTPUT_PIN, gpio_pin);
  }

  /* SX1262 Interupt (GPIO 19): a non-inverted, falling-edge interrupt */
  {
    gpio_pinset_t pinset = BOARD_GPIO_INT1;
    uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

    #warning TODO: Move bl602_configgpio to GPIO Expander
    ret = bl602_configgpio(pinset);
    DEBUGASSERT(ret == OK);

    #warning TODO: Move gpio_lower_half to GPIO Expander
    gpio_lower_half(bl602_expander, gpio_pin, GPIO_INTERRUPT_PIN, gpio_pin);

    IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                    (FAR void *)IOEXPANDER_VAL_FALLING);
  }
#endif /* CONFIG_IOEXPANDER_BL602_EXPANDER */

#ifdef CONFIG_I2C
  i2c_bus = bl602_i2cbus_initialize(0);
  i2c_register(i2c_bus, 0);
#endif

#ifdef CONFIG_SPI
  spi_bus = bl602_spibus_initialize(0);
  spi_register(spi_bus, 0);
#endif

#ifdef CONFIG_BL602_SPIFLASH
  mtd_part = bl602_spiflash_alloc_mtdpart();

  if (!mtd_part)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return -1;
    }

  /* Register the MTD driver so that it can be accessed from the  VFS */

  ret = register_mtddriver(path, mtd_part, 0777, NULL);
  if (ret < 0)
    {
      syslog(LOG_DEBUG, "ERROR: Failed to regitser MTD: %d\n", ret);
      return -1;
    }

  /* Mount the SPIFFS file system */

#ifdef CONFIG_FS_LITTLEFS
  ret = nx_mount(path, "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_DEBUG,
        "ERROR: Failed to mount littlefs at /data: %d\n", ret);
      return -1;
    }

#endif /* CONFIG_FS_LITTLEFS */
#endif /* CONFIG_BL602_SPIFLASH */

#ifdef CONFIG_BL602_WIRELESS
  bl602_set_em_sel(BL602_GLB_EM_8KB);

  bl602_net_initialize();
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the BL602 lower-half RTC driver */

  struct rtc_lowerhalf_s *lower;

  lower = bl602_rtc_lowerhalf_initialize();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n",
                 ret);
        }
    }
#endif

#if defined(CONFIG_BL602_BLE_CONTROLLER)
  bl602_hci_uart_init(0);
#endif /* CONFIG_BL602_BLE_CONTROLLER */

#ifdef CONFIG_FS_ROMFS
  /* Create a ROM disk for the /sbin filesystem */

  ret = romdisk_register(0, BL602_ROMFS_XIP_ADDR,
                         512,
                         512);
  if (ret < 0)
    {
      _err("ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount("/dev/ram0",
                  "/sbin",
                  "romfs", MS_RDONLY, NULL);
      if (ret < 0)
        {
          _err("ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
               "dev/ram0",
               "/sbin", ret);
        }
    }
#endif /* CONFIG_FS_ROMFS */

#ifdef CONFIG_RF_SPI_TEST_DRIVER

  /* Init SPI bus again */

  struct spi_dev_s *spitest = bl602_spibus_initialize(0);
  if (!spitest)
    {
      _err("ERROR: Failed to initialize SPI %d bus for SPI Test Driver\n", 0);
    }

  /* Register the SPI Test Driver */

  ret = spi_test_driver_register("/dev/spitest0", spitest, 0);
  if (ret < 0)
    {
      _err("ERROR: Failed to register SPI Test Driver\n");
    }

#endif /* CONFIG_RF_SPI_TEST_DRIVER */

#ifdef CONFIG_SENSORS_BME280

  /* Init I2C bus for BME280 */

  struct i2c_master_s *bme280_i2c_bus = bl602_i2cbus_initialize(0);
  if (!bme280_i2c_bus)
    {
      _err("ERROR: Failed to get I2C%d interface\n", 0);
    }

  /* Register the BME280 driver */

  ret = bme280_register(0, bme280_i2c_bus);
  if (ret < 0)
    {
      _err("ERROR: Failed to register BME280\n");
    }
#endif /* CONFIG_SENSORS_BME280 */

#ifdef CONFIG_SENSORS_BMP280

  /* Init I2C bus for BMP280 */

  struct i2c_master_s *bmp280_i2c_bus = bl602_i2cbus_initialize(0);
  if (!bmp280_i2c_bus)
    {
      _err("ERROR: Failed to get I2C%d interface\n", 0);
    }

  /* Register the BMP280 driver */

  ret = bmp280_register(0, bmp280_i2c_bus);
  if (ret < 0)
    {
      _err("ERROR: Failed to register BMP280\n");
    }
#endif /* CONFIG_SENSORS_BMP280 */

#ifdef CONFIG_LCD_DEV

  /* Initialize the LCD driver */

  ret = board_lcd_initialize();
  if (ret < 0)
    {
      _err("ERROR: board_lcd_initialize() failed: %d\n", ret);
    }

  /* Register the LCD driver */

  ret = lcddev_register(0);
  if (ret < 0)
    {
      _err("ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif /* CONFIG_LCD_DEV */

#ifdef CONFIG_INPUT_CST816S

  /* Init I2C bus for CST816S */

  struct i2c_master_s *cst816s_i2c_bus = bl602_i2cbus_initialize(0);
  if (!cst816s_i2c_bus)
    {
      _err("ERROR: Failed to get I2C%d interface\n", 0);
    }

  /* Register the CST816S driver */

  ret = cst816s_register("/dev/input0", cst816s_i2c_bus, CST816S_DEVICE_ADDRESS);
  if (ret < 0)
    {
      _err("ERROR: Failed to register CST816S\n");
    }
#endif /* CONFIG_INPUT_CST816S */

  return ret;
}

#ifdef CONFIG_LCD_ST7789

/* SPI Port Number for LCD */
#define LCD_SPI_PORTNO 0

/* SPI Bus for LCD */
static struct spi_dev_s *st7789_spi_bus;

/* LCD Device */
static struct lcd_dev_s *g_lcd = NULL;

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  st7789_spi_bus = bl602_spibus_initialize(LCD_SPI_PORTNO);
  if (!st7789_spi_bus)
    {
      lcderr("ERROR: Failed to initialize SPI port %d for LCD\n", LCD_SPI_PORTNO);
      return -ENODEV;
    }

  /* Pull LCD_RESET high */

  bl602_configgpio(BOARD_LCD_RST);
  bl602_gpiowrite(BOARD_LCD_RST, false);
  up_mdelay(1);
  bl602_gpiowrite(BOARD_LCD_RST, true);
  up_mdelay(10);

  /* Set full brightness */

  bl602_configgpio(BOARD_LCD_BL);
#ifdef BOARD_LCD_BL_INVERT  /* Backlight is active when Low */
  bl602_gpiowrite(BOARD_LCD_BL, false);
#else   /* Backlight is active when High */
  bl602_gpiowrite(BOARD_LCD_BL, true);
#endif  /* BOARD_LCD_BL_INVERT */

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  g_lcd = st7789_lcdinitialize(st7789_spi_bus);
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n", LCD_SPI_PORTNO,
      devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n", LCD_SPI_PORTNO, devno);
      return g_lcd;
    }

  return NULL;
}
#endif  //  CONFIG_LCD_ST7789

static int button_isr_handler(FAR struct ioexpander_dev_s *dev,
                              ioe_pinset_t pinset, FAR void *arg)
{
  #warning TODO: Move button_isr_handler to Button Handler
  gpioinfo("Button Pressed\n");
  return 0;
}