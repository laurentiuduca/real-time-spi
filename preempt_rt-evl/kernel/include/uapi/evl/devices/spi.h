/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _EVL_SPI_H
#define _EVL_SPI_H

#include <linux/types.h>

struct evl_spi_config {
	__u32 speed_hz;
	__u16 mode;
	__u8 bits_per_word;
};

struct evl_spi_iobufs {
	__u32 io_len;
	__u32 i_offset;
	__u32 o_offset;
	__u32 map_len;
};

#define EVL_SPI_IOCBASE	'S'
#define SPI_RTIOC_SET_CONFIG		_IOW(EVL_SPI_IOCBASE, 0, struct evl_spi_config)
#define SPI_RTIOC_GET_CONFIG		_IOR(EVL_SPI_IOCBASE, 1, struct evl_spi_config)
#define SPI_RTIOC_SET_IOBUFS		_IOR(EVL_SPI_IOCBASE, 2, struct evl_spi_iobufs)
#define SPI_RTIOC_TRANSFER		_IO(EVL_SPI_IOCBASE, 3)
#define SPI_RTIOC_TRANSFER_N		_IOR(EVL_SPI_IOCBASE, 4, int)

#endif /* !_EVL_SPI_H */
