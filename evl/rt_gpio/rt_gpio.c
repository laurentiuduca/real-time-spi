#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/sched.h>

#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/moduleparam.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/semaphore.h>

#include <linux/gpio.h>
#include <evl/flag.h>

static DEFINE_MUTEX(access_mutex);
static struct evl_file efile;

static u8 gpio_in_id = 115, gpio_out_id = 48;
static int irq_line, n_interrupts;

// the first device number
static dev_t first_device;
// the character device structure
static struct cdev c_dev;
// the device class
static struct class *cl; 
static struct device *dev;

static struct gpio_desc *gpiod_in, *gpiod_out;
static struct evl_flag interrupt_done;

static irqreturn_t rt_interrupt_handler(int irq, void *data)
{
#if 0
	int val = 0;
	val = gpiod_get_raw_value(gpiod_in);
	gpiod_set_raw_value(gpiod_out, val);
#endif
	n_interrupts++;
	
	/* signal that the interrupt has come */
	evl_raise_flag(&interrupt_done);

	//raw_printk("%s: n_interrupts=%d val=%d\n", __func__, n_interrupts, val);
	return IRQ_HANDLED;
}

static int rt_gpio_open(struct inode *i, struct file *f)
{
	int ret;
	mutex_lock(&access_mutex);
	ret = evl_open_file(&efile, f);
	return ret;
}

static int rt_gpio_close(struct inode *i, struct file *f)
{
	evl_release_file(&efile);
	mutex_unlock(&access_mutex);
	return 0;
}

static ssize_t rt_gpio_read(struct file *filp, char __user *buf, size_t len)
{
	int ret;
	
	//raw_printk("rt_gpio: read()\n");
	
	/* wait for interrupt */
	ret = evl_wait_flag(&interrupt_done);
	if (ret)
		return ret;

	if (gpiod_get_raw_value(gpiod_in) == 1) {
		ret = raw_copy_to_user(buf, "1", 2);
	} else {
		ret = raw_copy_to_user(buf, "0", 2);
	}

	if (ret < 0)
		return ret;

    return 2;
}

static ssize_t rt_gpio_write(struct file *filp, const char __user *buf, size_t len)
{
	int ret;
    u8 buf_kernel[100];
	
	//raw_printk("rt_gpio: write()\n");
	
	// read from user space and set new gpio output value
    ret = raw_copy_from_user(buf_kernel, buf, len);
    if (ret < 0)
		return ret;

	if (buf_kernel[0] == '0')
		gpiod_set_raw_value(gpiod_out, 0);
	else
		gpiod_set_raw_value(gpiod_out, 1);

	return len;
}

static struct file_operations rt_gpio_fops =
{
	.owner = THIS_MODULE,
	.open = rt_gpio_open,
	.release = rt_gpio_close,
	.oob_read = rt_gpio_read,
	.oob_write = rt_gpio_write
};

static int __init rt_gpio_init(void) /* Constructor */
{
	int ret;
	
	printk(KERN_INFO "Registering rt_gpio ...\n");
	if ((ret = alloc_chrdev_region(&first_device, 0, 1, "rt_gpio_region")) < 0) {
		printk(KERN_ERR "alloc_chrdev_region error\n");
		return ret;
	}
	if ((cl = class_create(THIS_MODULE, "rt_gpio_chardrv")) == NULL) {
		printk(KERN_ERR "class_create error\n");
		unregister_chrdev_region(first_device, 1);
		return -1;
	}
	if ((dev = device_create(cl, NULL, first_device, NULL, "rt_gpio_device")) == NULL) {
		printk(KERN_ERR "device_create error\n");
		class_destroy(cl);
		unregister_chrdev_region(first_device, 1);
		return -1;
	}
	cdev_init(&c_dev, &rt_gpio_fops);
	if ((ret = cdev_add(&c_dev, first_device, 1)) < 0) {
		printk(KERN_ERR "cdev_add error\n");
		device_destroy(cl, first_device);
		class_destroy(cl);
		unregister_chrdev_region(first_device, 1);
		return ret;
	}
	if (!gpio_is_valid(gpio_out_id) || !gpio_is_valid(gpio_in_id)) {
		printk(KERN_ERR "gpio_is_valid failed\n");
        return ret;
	}
	if ((ret = gpio_request_one(gpio_in_id, GPIOF_DIR_IN, THIS_MODULE->name)) < 0) {
		printk(KERN_ERR "gpio_request_one(gpio_in_id) error\n");
		return ret;
	}
	gpiod_in = gpio_to_desc(gpio_in_id);
	gpiod_out = gpio_to_desc(gpio_out_id);
	if ((gpiod_in == NULL) || (gpiod_out == NULL)) {
		printk(KERN_ERR "gpiod null \n");
		goto rollback_gpio_in;
	}
#if 0
	if(gpiod_in->gdev->chip->can_sleep ||
	   gpiod_out->gdev->chip->can_sleep)  {
		printk(KERN_ERR "gpiod can_sleep \n");
	}
#endif
#if 0
	if(gpiod_direction_input(gpiod_in) < 0) {
		printk(KERN_ERR "gpio_direction_input error\n");
		goto rollback_gpio_in;
	}
#endif
	if(gpiod_direction_output(gpiod_out, 0) < 0) {
		printk(KERN_ERR "gpio_direction_output error\n");
		goto rollback_gpio_in;
	}
	if ((irq_line = gpiod_to_irq(gpiod_in)) < 0){
		printk(KERN_ERR "gpio %d cannot be used as interrupt\n", gpio_in_id);
		goto rollback_gpio_in;
	}
#if 0	
    if ((ret = irq_set_irq_type(irq_n, IRQ_TYPE_EDGE_RISING)) < 0) {
		printk(KERN_ERR "error irq_set_irq_type\n");
		goto rollback_gpio_in;
	}
#endif
	if ((ret = request_irq(irq_line, rt_interrupt_handler,
		IRQ_TYPE_EDGE_RISING | IRQF_OOB, "rt_gpio interrupt\n", (void *)1)) < 0) 
	{
		printk(KERN_ERR "request_irq failed\n");
		//raw_printk("request_irq failed\n");
		goto rollback_gpio_in;
	}

	evl_init_flag(&interrupt_done);
	
	printk(KERN_INFO "rt_gpio done registering\n");
	//raw_printk("rt_gpio done registering\n");
	return 0;

rollback_gpio_in:	
	gpio_free(gpio_in_id);
#if 0
rollback_gpio_out:
#endif
	gpio_free(gpio_out_id);
	
	return -1;
}
 
static void __exit rt_gpio_exit(void)
{
	free_irq(irq_line, (void*)1);
	gpio_free(gpio_in_id);
	gpio_free(gpio_out_id);
	cdev_del(&c_dev);
	device_destroy(cl, first_device);
	class_destroy(cl);
	unregister_chrdev_region(first_device, 1);
	printk(KERN_INFO "rt_gpio unregistered\n");
}

module_init(rt_gpio_init);
module_exit(rt_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Laurentiu-Cristian Duca");
MODULE_DESCRIPTION("evl gpio in-out for bbb");

