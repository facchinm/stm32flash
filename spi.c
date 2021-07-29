/*
  stm32flash - Open Source ST STM32 flash program for *nix
  Copyright (C) 2014 Antonio Borneo <borneo.antonio@gmail.com>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "compiler.h"
#include "serial.h"
#include "port.h"


#if !defined(__linux__)

static port_err_t spi_open(struct port_interface __unused *port,
			   struct port_options __unused *ops)
{
	return PORT_ERR_NODEV;
}

struct port_interface port_spi = {
	.name	= "spi",
	.open	= spi_open,
};

#else

#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

struct spi_priv {
	int fd;
	uint8_t mode, lsb, bits;
  uint32_t speed;
};

static port_err_t spi_open(struct port_interface *port,
			   struct port_options *ops)
{
	struct spi_priv *h;
	int fd, ret;
	unsigned long funcs;
	uint8_t mode, lsb, bits;
  uint32_t speed = 2500000;

	/* 1. check device name match */
	if (strncmp(ops->device, "/dev/spidev", strlen("/dev/spidev")))
		return PORT_ERR_NODEV;

	/* 2. open it */
	h = calloc(sizeof(*h), 1);
	if (h == NULL) {
		fprintf(stderr, "End of memory\n");
		return PORT_ERR_UNKNOWN;
	}
	fd = open(ops->device, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Unable to open special file \"%s\"\n",
			ops->device);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	/* 3.5. Check capabilities */
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &lsb);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret < 0) {
		fprintf(stderr, "I2C ioctl(WRITE_MODE) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(READ_BIT_PER_WORD) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret < 0) {
		fprintf(stderr, "SPI ioctl(WRITE_SPEED) error %d\n", errno);
		close(fd);
		free(h);
		return PORT_ERR_UNKNOWN;
	}

	h->fd = fd;
	h->mode = mode;
	h->lsb = lsb;
	h->bits = bits;
	h->speed = speed;
	port->private = h;
	return PORT_ERR_OK;
}

static port_err_t spi_close(struct port_interface *port)
{
	struct spi_priv *h;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	close(h->fd);
	free(h);
	port->private = NULL;
	return PORT_ERR_OK;
}

ssize_t spi_transfer(int fd, void *out, void *in, size_t len)
{
    struct spi_ioc_transfer msgs[2] = {};
    int nmsg = 1;

  	uint8_t buf[1] = { 0x5A };

  	msgs[0].tx_buf = buf;
  	msgs[0].rx_buf = NULL;
  	msgs[0].len = 1;

    msgs[1].tx_buf = out;
    msgs[1].rx_buf = in;
    msgs[1].len = len;
    if(ioctl(fd, SPI_IOC_MESSAGE(2), &msgs) < 0)
        return -1;
    return len;
}

static port_err_t spi_read(struct port_interface *port, void *buf,
			   size_t nbyte)
{
	struct spi_priv *h;
	int ret;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	ret = spi_transfer(h->fd, NULL, buf, nbyte);
	if (ret != (int)nbyte)
		return PORT_ERR_UNKNOWN;
	return PORT_ERR_OK;
}

static port_err_t spi_write(struct port_interface *port, void *buf,
			    size_t nbyte)
{
	struct spi_priv *h;
	int ret;

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return PORT_ERR_UNKNOWN;
	ret = spi_transfer(h->fd, buf, NULL, nbyte);

	if (ret != (int)nbyte)
		return PORT_ERR_UNKNOWN;
	return PORT_ERR_OK;
}

static port_err_t spi_gpio(struct port_interface __unused *port,
			   serial_gpio_t __unused n,
			   int __unused level)
{
	return PORT_ERR_OK;
}

static const char *spi_get_cfg_str(struct port_interface *port)
{
	struct spi_priv *h;
	static char str[50];

	h = (struct spi_priv *)port->private;
	if (h == NULL)
		return "INVALID";
	snprintf(str, sizeof(str), "mode:%d lsb:%d bits:%d speed:%lu", h->mode, h->lsb, h->bits, h->speed);
	return str;
}

static port_err_t spi_flush(struct port_interface __unused *port)
{
	/* We shouldn't need to flush I2C */
	return PORT_ERR_OK;
}

struct port_interface port_spi = {
	.name	= "spi",
	//.flags	= PORT_SYNC_5A,
	.open	= spi_open,
	.close	= spi_close,
	.flush  = spi_flush,
	.read	= spi_read,
	.write	= spi_write,
	.gpio	= spi_gpio,
	.get_cfg_str	= spi_get_cfg_str,
};

#endif
