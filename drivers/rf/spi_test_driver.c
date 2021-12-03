/****************************************************************************
 * drivers/rf/spi_test_driver.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/* SPI Test Driver */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/rf/ioctl.h>
#include <nuttx/rf/attenuator.h>
#include <nuttx/rf/spi_test_driver.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPI) && defined(CONFIG_RF_SPI_TEST_DRIVER)

#ifndef CONFIG_SPI_TEST_DRIVER_SPI_FREQUENCY
#  define CONFIG_SPI_TEST_DRIVER_SPI_FREQUENCY 1000000
#endif

////#define SPI_TEST_DRIVER_SPI_MODE (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0,CPHA=0 */
#define SPI_TEST_DRIVER_SPI_MODE (SPIDEV_MODE1) /* SPI Mode 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_test_driver_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int spi_test_driver_open(FAR struct file *filep);
static int spi_test_driver_close(FAR struct file *filep);
static ssize_t spi_test_driver_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t spi_test_driver_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int spi_test_driver_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_spi_test_driver_fops =
{
  spi_test_driver_open,
  spi_test_driver_close,
  spi_test_driver_read,
  spi_test_driver_write,
  NULL,  /* Seek not implemented */
  spi_test_driver_ioctl,
  NULL   /* Poll not implemented */
};

static char recv_buffer[256];  /* Buffer for SPI response */

static int recv_buffer_len = 0;  /* Length of SPI response */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_test_driver_configspi
 *
 * Description:
 *   Configure the SPI instance
 *
 ****************************************************************************/

static inline void spi_test_driver_configspi(FAR struct spi_dev_s *spi)
{
  printf("spi_test_driver_configspi\n");
  DEBUGASSERT(spi != NULL);

  /* Set SPI Mode (Polarity and Phase) and Transfer Size (8 bits) */

  SPI_SETMODE(spi, SPI_TEST_DRIVER_SPI_MODE);
  SPI_SETBITS(spi, 8);

  /* Set SPI Hardware Features and Frequency */

  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_SPI_TEST_DRIVER_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: spi_test_driver_set_attenuation
 *
 * Description:
 *   Set the attenuation level in dB (16.16 bits fixed point).
 *
 ****************************************************************************/

static void spi_test_driver_set_attenuation(FAR struct spi_test_driver_dev_s *priv,
                                      b16_t attenuation)
{
  printf("spi_test_driver_set_attenuation\n");
  DEBUGASSERT(priv != NULL);

  SPI_LOCK(priv->spi, true);

  spi_test_driver_configspi(priv->spi);

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Convert the attenuation value from 16.16 bits to 5.1 bits. */

  SPI_SEND(priv->spi, (uint8_t)(attenuation >> 15));

  /* Send a pulse to the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, true);
  up_udelay(1);
  SPI_SELECT(priv->spi, priv->spidev, false);

  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: spi_test_driver_open
 *
 * Description:
 *   This function is called whenever the device is opened.
 *
 ****************************************************************************/

static int spi_test_driver_open(FAR struct file *filep)
{
  printf("spi_test_driver_open\n");
  DEBUGASSERT(filep != NULL);
  return OK;
}

/****************************************************************************
 * Name: spi_test_driver_close
 *
 * Description:
 *   This function is called whenever the device is closed.
 *
 ****************************************************************************/

static int spi_test_driver_close(FAR struct file *filep)
{
  printf("spi_test_driver_close\n");
  DEBUGASSERT(filep != NULL);
  return OK;
}

/****************************************************************************
 * Name: spi_test_driver_write
 *
 * Description:
 *   Write the buffer to the device.
 ****************************************************************************/

static ssize_t spi_test_driver_write(FAR struct file *filep,
                               FAR const char *buffer,
                               size_t buflen)
{
  printf("spi_test_driver_write: buflen=%u\n  ", buflen);
  for (int i = 0; buffer != NULL && i < buflen; i++) 
    {
      printf("%02x ", buffer[i]);
    }
  printf("\n");

  DEBUGASSERT(buflen <= sizeof(recv_buffer));
  DEBUGASSERT(filep  != NULL);
  DEBUGASSERT(buffer != NULL);

  /* Get the SPI interface */

  FAR struct inode *inode = filep->f_inode;
  FAR struct spi_test_driver_dev_s *priv = inode->i_private;

  /* Lock the SPI bus and configure the SPI interface */

  SPI_LOCK(priv->spi, true);
  spi_test_driver_configspi(priv->spi);

  /* Select the SPI device (unused for BL602) */

  SPI_SELECT(priv->spi, priv->spidev, true);

  /* Transmit buffer to SPI device and receive the response */

  SPI_EXCHANGE(priv->spi, buffer, recv_buffer, buflen);
  recv_buffer_len = buflen;

  printf("spi_test_driver_write: received\n  ");
  for (int i = 0; i < buflen; i++) 
    {
      printf("%02x ", recv_buffer[i]);
    }
  printf("\n");

  /* Deselect the SPI device (unused for BL602) */

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Unlock the SPI bus */

  SPI_LOCK(priv->spi, false);

  return buflen;
}

/****************************************************************************
 * Name: spi_test_driver_read
 *
 * Description:
 *   Return the data received from the device.
 ****************************************************************************/

static ssize_t spi_test_driver_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  printf("spi_test_driver_read: buflen=%u\n", buflen);
  DEBUGASSERT(filep  != NULL);
  DEBUGASSERT(buffer != NULL);

  /* Copy the SPI response to the buffer */

  DEBUGASSERT(recv_buffer_len >= 0);
  DEBUGASSERT(recv_buffer_len <= buflen);
  memcpy(buffer, recv_buffer, recv_buffer_len);

  /* Return the number of bytes read */

  return recv_buffer_len;
}

/****************************************************************************
 * Name: spi_test_driver_ioctl
 *
 * Description:
 *   Execute ioctl commands for the device.
 ****************************************************************************/

static int spi_test_driver_ioctl(FAR struct file *filep,
                           int cmd,
                           unsigned long arg)
{
  printf("spi_test_driver_ioctl: cmd=0x%x, arg=0x%lx\n", cmd, arg);
  DEBUGASSERT(filep != NULL);

  FAR struct inode *inode = filep->f_inode;
  FAR struct spi_test_driver_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      /* TODO: Handle ioctl commands */

      case RFIOC_SETATT:
        {
          FAR struct attenuator_control *att =
            (FAR struct attenuator_control *)((uintptr_t)arg);
          DEBUGASSERT(att != NULL);
          spi_test_driver_set_attenuation(priv, att->attenuation);
        }
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_test_driver_register
 *
 * Description:
 *   Register the spi_test_driver character device as 'devpath' during NuttX startup.
 *
 ****************************************************************************/

int spi_test_driver_register(FAR const char *devpath,
                       FAR struct spi_dev_s *spi,
                       int spidev)
{
  FAR struct spi_test_driver_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(spi != NULL);

  /* Initialize the device structure */

  priv = (FAR struct spi_test_driver_dev_s *)
      kmm_malloc(sizeof(struct spi_test_driver_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi    = spi;
  priv->spidev = spidev;

  /* Clear the LE pin */

  SPI_SELECT(priv->spi, priv->spidev, false);

  /* Register the character driver */

  ret = register_driver(devpath, &g_spi_test_driver_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif
