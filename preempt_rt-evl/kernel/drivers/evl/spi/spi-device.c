// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Simple userspace interface to real-time SPI device drivers
 *
 * Copyright (C) 2006 SWAPP
 *	Andrea Paterniani <a.paterniani@swapp-eng.it>
 * Copyright (C) 2007 David Brownell (simplification, cleanup)
 * Copyright (C) 2020 Laurentiu-Cristian Duca 
 * <laurentiu [dot] duca [at] gmail [dot] com>
 * (real-time support)
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/gpio.h>

#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <linux/uaccess.h>
#include "spi-master.h"

/*
 * This supports access to SPI devices using normal userspace I/O calls.
 * Note that while traditional UNIX/POSIX I/O semantics are half duplex,
 * and often mask message boundaries, full SPI support requires full duplex
 * transfers.  There are several kinds of internal message boundaries to
 * handle chipselect management and other protocol options.
 *
 * SPI has a character major number assigned.  We allocate minor numbers
 * dynamically using a bitmask.  You must use hotplug tools, such as udev
 * (or mdev with busybox) to create and destroy the /dev/spidevB.C device
 * nodes, since there is no fixed association of minor numbers with any
 * particular SPI bus or device.
 */

#define N_SPI_MINORS			32	/* ... up to 256 */

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

/*-------------------------------------------------------------------------*/

static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct evl_spidevice *evl_spidev;
	
	evl_spidev = filp->private_data;

	return spi_master_read_rt(evl_spidev->spidev->slave, buf, count);
}

static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct evl_spidevice *evl_spidev;
	
	evl_spidev = filp->private_data;

	return spi_master_write_rt(evl_spidev->spidev->slave, buf, count);
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct evl_spidevice *evl_spidev;
	
	evl_spidev = filp->private_data;

	switch(cmd) {
	case SPI_RTIOC_SET_CONFIG:
	case SPI_RTIOC_GET_CONFIG:
	case SPI_RTIOC_TRANSFER:
	case SPI_RTIOC_TRANSFER_N:
		return spi_master_ioctl_rt(evl_spidev->spidev->slave, cmd, (void *)arg);
	case SPI_RTIOC_SET_IOBUFS:
		return spi_master_ioctl_nrt(evl_spidev->spidev->slave, cmd, (void *)arg);
	default:
		return -EINVAL;
	}
}

/*  mmap handler to map kernel space to user space  
 *
 */
static int spidev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct evl_spidevice	*evl_spidev;

	evl_spidev = filp->private_data;
	return spi_master_mmap(evl_spidev->spidev->slave, vma);
}

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct evl_spidevice *evl_spidev;
	spidev_t	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status) {
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
		goto err_find_dev;
	}

	evl_spidev = kzalloc(sizeof(*evl_spidev), GFP_KERNEL);
	evl_spidev->spidev = spidev;
	spidev->users++;
		
	filp->private_data = evl_spidev;
	mutex_unlock(&device_list_lock);
	
	spi_master_open(spidev->slave);

err_find_dev:
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct evl_spidevice *evl_spidev;

	mutex_lock(&device_list_lock);
	evl_spidev = filp->private_data;
	filp->private_data = NULL;
	evl_spidev->spidev->users--;
	mutex_unlock(&device_list_lock);

	spi_master_close(evl_spidev->spidev->slave);
	
	return 0;
}

static const struct file_operations spidev_fops = {
	.owner =	THIS_MODULE,
	.unlocked_ioctl = 	spidev_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.read = spidev_read,
	.write = spidev_write,
	.mmap = spidev_mmap,
};

/*-------------------------------------------------------------------------*/

int evl_spi_add_remote_slave(struct evl_spi_remote_slave *slave,
			      struct evl_spi_master *master,
			      struct spi_device *spi, 
				  struct device *dev)
{
	struct spi_master *kmaster = master->kmaster;
	int ret;
	unsigned long flags;
	
	memset(slave, 0, sizeof(*slave));
	slave->chip_select = spi->chip_select;
	slave->config.bits_per_word = spi->bits_per_word;
	slave->config.speed_hz = spi->max_speed_hz;
	slave->config.mode = spi->mode;
	slave->master = master;
	slave->dev = dev;
	
	if (gpio_is_valid(spi->cs_gpio))
		slave->cs_gpio = spi->cs_gpio;
	else {
		slave->cs_gpio = -ENOENT;
		if (kmaster->cs_gpios)
			slave->cs_gpio = kmaster->cs_gpios[spi->chip_select];
	}

	if (gpio_is_valid(slave->cs_gpio)) {
		ret = gpio_request(slave->cs_gpio, slave->dev->kobj.name);
		if (ret)
			goto fail;
		slave->cs_gpiod = gpio_to_desc(slave->cs_gpio);
		if (slave->cs_gpiod == NULL) {
			ret = -EINVAL;
			goto fail;
		}
	}
	
	mutex_init(&slave->ctl_lock);

	raw_spin_lock_irqsave(&master->lock, flags);
	list_add_tail(&slave->next, &master->slaves);
	raw_spin_unlock_irqrestore(&master->lock, flags);

	return 0;
	
fail:
	return ret;
}
EXPORT_SYMBOL_GPL(evl_spi_add_remote_slave);

void evl_spi_remove_remote_slave(struct evl_spi_remote_slave *slave)
{
	struct evl_spi_master *master = slave->master;
	unsigned long flags;
	
	if (gpio_is_valid(slave->cs_gpio))
		gpio_free(slave->cs_gpio);

	mutex_destroy(&slave->ctl_lock);
	raw_spin_lock_irqsave(&master->lock, flags);
	list_del(&slave->next);
	raw_spin_unlock_irqrestore(&master->lock, flags);
}
EXPORT_SYMBOL_GPL(evl_spi_remove_remote_slave);

/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *spidev_class;
static int major, minor=0;

static const struct of_device_id spidev_dt_ids[] = {
	{
		.compatible = "spidev",
	},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, spidev_dt_ids);

/*-------------------------------------------------------------------------*/

static int spidev_probe(struct spi_device *spi)
{
	struct evl_spi_master *master;
	spidev_t	*spidev;
	int			status, ret;
	struct device *dev;

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	if (minor < N_SPI_MINORS) {
		spidev->devt = MKDEV(major, minor);
		dev = device_create(spidev_class, &spi->dev, spidev->devt,
				    spidev, "evl-spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		minor++;
		list_add(&spidev->device_entry, &device_list);

		master = spi_master_get_devdata(spi->master);
		if (master->initialized == 0) {
			ret = __evl_spi_setup_driver(master);
			if (ret)
				return ret;
		}
		spidev->slave = master->ops->attach_slave(master, spi, dev);
		spidev->slave->dev = dev;
		if (IS_ERR(spidev->slave))
			return PTR_ERR(spidev->slave);

		spi_set_drvdata(spi, spidev);
	}

	mutex_unlock(&device_list_lock);

	if (status < 0)
		kfree(spidev);

	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	spidev_t	*spidev = spi_get_drvdata(spi);

	spidev->slave->master->ops->detach_slave(spidev->slave);
	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);

	return 0;
}

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"evl_spi_device",
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};

/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;
	
	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	major = register_chrdev(0, "spidev-rt", &spidev_fops);
	if (major < 0) {
		printk(KERN_ERR "%s: register_chrdev error\n", __func__);
		return major;
	}
	
	spidev_class = class_create(THIS_MODULE, "evl_spi_device");
	if (IS_ERR(spidev_class)) {
		printk(KERN_ERR "%s: class_create error\n", __func__);
		unregister_chrdev(major, spidev_spi_driver.driver.name);
		return PTR_ERR(spidev_class);
	}

	status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		printk(KERN_ERR "%s: spi_register_driver error\n", __func__);
		class_destroy(spidev_class);
		unregister_chrdev(major, spidev_spi_driver.driver.name);
	}
	return status;
}
module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(major, spidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_DESCRIPTION("EVL real-time SPI device driver interface");
MODULE_LICENSE("GPL");
