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
#ifndef _evl_spi_master_H
#define _evl_spi_master_H

#include <linux/rtmutex.h>
#include <uapi/evl/devices/spi.h>
#include "spi-device.h"

#define MASTER_MAX_MSG_SIZE (PAGE_SIZE / 2)

struct class;
struct device_node;
struct evl_spi_master;
struct spi_master;

struct evl_spi_master_ops {
	int (*open)(struct evl_spi_remote_slave *slave);
	void (*close)(struct evl_spi_remote_slave *slave);
	int (*configure)(struct evl_spi_remote_slave *slave);
	void (*chip_select)(struct evl_spi_remote_slave *slave,
			    bool active);
	int (*set_iobufs)(struct evl_spi_remote_slave *slave,
			  struct evl_spi_iobufs *p);
	int (*mmap_iobufs)(struct evl_spi_remote_slave *slave,
			   struct vm_area_struct *vma);
	void (*mmap_release)(struct evl_spi_remote_slave *slave);
	int (*transfer_iobufs)(struct evl_spi_remote_slave *slave);
	int (*transfer_iobufs_n)(struct evl_spi_remote_slave *slave, int len);
	ssize_t (*write)(struct evl_spi_remote_slave *slave,
			 const void *tx, size_t len);
	ssize_t (*read)(struct evl_spi_remote_slave *slave,
			 void *rx, size_t len);
	struct evl_spi_remote_slave *(*attach_slave)
		(struct evl_spi_master *master,
		 struct spi_device *spi, 
		 struct device *dev);
	void (*detach_slave)(struct evl_spi_remote_slave *slave);
};

struct evl_spi_master {
	int subclass;
	const struct evl_spi_master_ops *ops;
	struct spi_master *kmaster;
	int initialized;
	void *rx_buf, *tx_buf;
	struct {	/* Internal */
		struct list_head slaves;
		struct list_head next;
		raw_spinlock_t lock;
		struct rt_mutex bus_lock;
		struct evl_spi_remote_slave *cs;
	};
};

#define evl_spi_alloc_master(__dev, __type, __mptr)			\
	__evl_spi_alloc_master(__dev, sizeof(__type),			\
				offsetof(__type, __mptr))		\

struct evl_spi_master *
__evl_spi_alloc_master(struct device *dev, size_t size, int off);

int __evl_spi_setup_driver(struct evl_spi_master *master);

int evl_spi_add_master(struct evl_spi_master *master);

void evl_spi_remove_master(struct evl_spi_master *master);

int spi_master_open(struct evl_spi_remote_slave *slave);
void spi_master_close(struct evl_spi_remote_slave *slave);
ssize_t spi_master_read_rt(struct evl_spi_remote_slave *slave,
				  void __user *u_buf, size_t len);
ssize_t spi_master_write_rt(struct evl_spi_remote_slave *slave,
				   const void __user *u_buf, size_t len);
int spi_master_ioctl_nrt(struct evl_spi_remote_slave *slave,
				unsigned int request, void *arg);
int spi_master_ioctl_rt(struct evl_spi_remote_slave *slave,
			       unsigned int request, void *arg);
int spi_master_mmap(struct evl_spi_remote_slave *slave,
						   struct vm_area_struct *vma);

#endif /* !_evl_spi_master_H */
