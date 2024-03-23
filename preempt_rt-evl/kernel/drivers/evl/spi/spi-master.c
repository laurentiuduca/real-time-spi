/**
 * Copyright (C) 2020 Laurentiu-Cristian Duca 
 * <laurentiu [dot] duca [at] gmail [dot] com>
 * Port to EVL.
 *
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include "spi-master.h"

static int update_slave_config(struct evl_spi_remote_slave *slave,
			       struct evl_spi_config *config)
{
	struct evl_spi_config old_config;
	struct evl_spi_master *master = slave->master;
	int ret;

	rt_mutex_lock(&master->bus_lock);

	old_config = slave->config;
	slave->config = *config;
	ret = slave->master->ops->configure(slave);
	if (ret) {
		slave->config = old_config;
		rt_mutex_unlock(&master->bus_lock);
		return ret;
	}

	rt_mutex_unlock(&master->bus_lock);

#if 0	
	raw_printk(
		 "configured mode %d, %s%s%s%s%u bits/w, %u Hz max\n",
		 (int) (slave->config.mode & (SPI_CPOL | SPI_CPHA)),
		 (slave->config.mode & SPI_CS_HIGH) ? "cs_high, " : "",
		 (slave->config.mode & SPI_LSB_FIRST) ? "lsb, " : "",
		 (slave->config.mode & SPI_3WIRE) ? "3wire, " : "",
		 (slave->config.mode & SPI_LOOP) ? "loopback, " : "",
		 slave->config.bits_per_word,
		 slave->config.speed_hz);
#endif	
	return 0;
}

int spi_master_open(struct evl_spi_remote_slave *slave)
{
	struct evl_spi_master *master = slave->master;

	if (master->ops->open)
		return master->ops->open(slave);
		
	return 0;
}

void spi_master_close(struct evl_spi_remote_slave *slave)
{
	struct evl_spi_master *master = slave->master;
	unsigned long flags;

	raw_spin_lock_irqsave(&master->lock, flags);

	if (master->cs == slave)
		master->cs = NULL;

	raw_spin_unlock_irqrestore(&master->lock, flags);

	if (master->ops->close)
		master->ops->close(slave);
}

static int do_chip_select(struct evl_spi_remote_slave *slave)
{				/* master->bus_lock held */
	struct evl_spi_master *master = slave->master;
	unsigned long flags;
	int state;

	if (slave->config.speed_hz == 0)
		return -EINVAL; /* Setup is missing. */

	/* Serialize with spi_master_close() */
	raw_spin_lock_irqsave(&master->lock, flags);
	
	if (master->cs != slave) {
		if (gpio_is_valid(slave->cs_gpio)) {
			state = !!(slave->config.mode & SPI_CS_HIGH);
			gpiod_set_raw_value(slave->cs_gpiod, state);
		} else
			master->ops->chip_select(slave, true);
		master->cs = slave;
	}

	raw_spin_unlock_irqrestore(&master->lock, flags);

	return 0;
}

static void do_chip_deselect(struct evl_spi_remote_slave *slave)
{				/* master->bus_lock held */
	struct evl_spi_master *master = slave->master;
	unsigned long flags;
	int state;

	raw_spin_lock_irqsave(&master->lock, flags);

	if (gpio_is_valid(slave->cs_gpio)) {
		state = !(slave->config.mode & SPI_CS_HIGH);
		gpiod_set_raw_value(slave->cs_gpiod, state);
	} else
		master->ops->chip_select(slave, false);

	master->cs = NULL;

	raw_spin_unlock_irqrestore(&master->lock, flags);
}

int spi_master_ioctl_rt(struct evl_spi_remote_slave *slave,
			       unsigned int request, void *arg)
{
	struct evl_spi_master *master = slave->master;
	struct evl_spi_config config;
	int ret, len;

	switch (request) {
	case SPI_RTIOC_SET_CONFIG:
		ret = raw_copy_from_user(&config,
					       arg, sizeof(config));
		if (ret == 0)
			ret = update_slave_config(slave, &config);
		break;
	case SPI_RTIOC_GET_CONFIG:
		rt_mutex_lock(&master->bus_lock);
		config = slave->config;
		rt_mutex_unlock(&master->bus_lock);
		ret = raw_copy_to_user(arg,
					     &config, sizeof(config));
		break;
	case SPI_RTIOC_TRANSFER:
		ret = -EINVAL;
		if (master->ops->transfer_iobufs) {
			rt_mutex_lock(&master->bus_lock);
			ret = do_chip_select(slave);
			if (ret == 0) {
				ret = master->ops->transfer_iobufs(slave);
				do_chip_deselect(slave);
			}
			rt_mutex_unlock(&master->bus_lock);
		}
		break;
	case SPI_RTIOC_TRANSFER_N:
		ret = -EINVAL;
		if (master->ops->transfer_iobufs_n) {
			len = (int)arg;
			rt_mutex_lock(&master->bus_lock);
			ret = do_chip_select(slave);
			if (ret == 0) {
				ret = master->ops->transfer_iobufs_n(slave, len);
				do_chip_deselect(slave);
			}
			rt_mutex_unlock(&master->bus_lock);
		}
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

int spi_master_ioctl_nrt(struct evl_spi_remote_slave *slave,
				unsigned int request, void *arg)
{
	struct evl_spi_master *master = slave->master;
	struct evl_spi_iobufs iobufs;
	int ret = 0;

	switch (request) {
	case SPI_RTIOC_SET_IOBUFS:
		ret = copy_from_user(&iobufs, arg, sizeof(iobufs));
		if(ret)
			return ret;
		/*
		 * No transfer can happen without I/O buffers being
		 * set, and I/O buffers cannot be reset, therefore we
		 * need no serialization with the transfer code here.
		 */
		mutex_lock(&slave->ctl_lock);
		ret = master->ops->set_iobufs(slave, &iobufs);
		mutex_unlock(&slave->ctl_lock);
		if (ret == 0)
			ret = copy_to_user(arg, &iobufs, sizeof(iobufs));
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

ssize_t spi_master_read_rt(struct evl_spi_remote_slave *slave,
				  void __user *u_buf, size_t len)
{
	struct evl_spi_master *master = slave->master;
	void *rx;
	int ret;

	if (len == 0)
		return 0;
	if (len > MASTER_MAX_MSG_SIZE)
		return -EMSGSIZE;
	
	rx = master->rx_buf;

	rt_mutex_lock(&master->bus_lock);
	ret = do_chip_select(slave);
	if (ret == 0) {
		ret = master->ops->read(slave, rx, len);
		do_chip_deselect(slave);
	}
	rt_mutex_unlock(&master->bus_lock);
	if (ret > 0)
		ret = raw_copy_to_user(u_buf, rx, ret);

	return ret;
}

ssize_t spi_master_write_rt(struct evl_spi_remote_slave *slave,
				   const void __user *u_buf, size_t len)
{
	struct evl_spi_master *master = slave->master;
	void *tx;
	int ret;

	if (len == 0)
		return 0;
	if (len > MASTER_MAX_MSG_SIZE)
		return -EMSGSIZE;

	tx = master->tx_buf;

	ret = raw_copy_from_user(tx, u_buf, len);
	if (ret == 0) {
		rt_mutex_lock(&master->bus_lock);
		ret = do_chip_select(slave);
		if (ret == 0) {
			ret = master->ops->write(slave, tx, len);
			do_chip_deselect(slave);
		}
		rt_mutex_unlock(&master->bus_lock);
	}
	
	return ret;
}

static void iobufs_vmopen(struct vm_area_struct *vma)
{
	struct evl_spi_remote_slave *slave = vma->vm_private_data;

	atomic_inc(&slave->mmap_refs);
}

static void iobufs_vmclose(struct vm_area_struct *vma)
{
	struct evl_spi_remote_slave *slave = vma->vm_private_data;

	if (atomic_dec_and_test(&slave->mmap_refs)) {
		slave->master->ops->mmap_release(slave);
	}
}

static struct vm_operations_struct iobufs_vmops = {
	.open = iobufs_vmopen,
	.close = iobufs_vmclose,
};

int spi_master_mmap(struct evl_spi_remote_slave *slave,
						   struct vm_area_struct *vma)
{
	int ret;

	if (slave->master->ops->mmap_iobufs == NULL)
		return -EINVAL;

	ret = slave->master->ops->mmap_iobufs(slave, vma);
	if (ret)
		return ret;

	atomic_inc(&slave->mmap_refs);

	if (slave->master->ops->mmap_release) {
		vma->vm_ops = &iobufs_vmops;
		vma->vm_private_data = slave;
	}

	return 0;
}

struct evl_spi_master *
__evl_spi_alloc_master(struct device *dev, size_t size, int off)
{
	struct evl_spi_master *master;
	struct spi_master *kmaster;

	kmaster = spi_alloc_master(dev, size);
	if (kmaster == NULL)
		return NULL;
	
	master = (void *)(kmaster + 1) + off;
	master->kmaster = kmaster;
	master->initialized = 0;
	spi_master_set_devdata(kmaster, master);

	return master;
}
EXPORT_SYMBOL_GPL(__evl_spi_alloc_master);

int __evl_spi_setup_driver(struct evl_spi_master *master)
{
	master->initialized = 1;
	master->cs = NULL;

	INIT_LIST_HEAD(&master->slaves);
	raw_spin_lock_init(&master->lock);
	rt_mutex_init(&master->bus_lock);

	return 0;
}

static int spi_transfer_one_unimp(struct spi_master *master,
				  struct spi_device *spi,
				  struct spi_transfer *tfr)
{
	return -ENODEV;
}

int evl_spi_add_master(struct evl_spi_master *master)
{
	struct spi_master *kmaster = master->kmaster;

	master->tx_buf = kmalloc(MASTER_MAX_MSG_SIZE, GFP_KERNEL);
	if(master->tx_buf == NULL)
		return -ENOMEM;
	master->rx_buf = kmalloc(MASTER_MAX_MSG_SIZE, GFP_KERNEL);
	if(master->rx_buf == NULL)
		return -ENOMEM;
	
	/*
	 * Prevent the transfer handler to be called from the regular
	 * SPI stack, just in case.
	 */
	kmaster->transfer_one = spi_transfer_one_unimp;

	/*
	 * Add the core SPI driver, devices on the bus will be
	 * enumerated, handed to spi_device_probe().
	 */
	return spi_register_master(kmaster);
}
EXPORT_SYMBOL_GPL(evl_spi_add_master);

void evl_spi_remove_master(struct evl_spi_master *master)
{
	kfree(master->tx_buf);
	kfree(master->rx_buf);
	//rt_mutex_destroy(&master->bus_lock); // not available
    //WARN_ON(rt_mutex_is_locked(master->bus_lock));
    #ifdef CONFIG_DEBUG_RT_MUTEXES
        master->bus_lock->magic = NULL;
    #endif

	spi_unregister_master(master->kmaster);
}
EXPORT_SYMBOL_GPL(evl_spi_remove_master);

MODULE_LICENSE("GPL");
