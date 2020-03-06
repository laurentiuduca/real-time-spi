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

// To view trace_printk() output:
// cat /sys/kernel/debug/tracing/trace
// after mounting debugfs:
// mount -t debugfs none /sys/kernel/debug


static u8 gpio_in_id = 115, gpio_out_id = 48;
static int irq_line, n_interrupts, interrupt_done;
static wait_queue_head_t wq;

static dev_t first_device; // the first device number
static struct cdev c_dev; // the character device structure
static struct class *cl; // the device class

int val = 0;
static irqreturn_t non_rt_interrupt_handler(int irq, void *data)
{
	n_interrupts++;
	
	val = !val;
	gpio_set_value(gpio_out_id, val);
	//trace_printk("%s: n_interrupts=%d\n", __func__, n_interrupts);
	return IRQ_HANDLED;
}

static int non_rt_gpio_open(struct inode *i, struct file *f)
{
	return 0;
}

static int non_rt_gpio_close(struct inode *i, struct file *f)
{
	return 0;
}

static ssize_t non_rt_gpio_read(struct file *f, char __user *buf, size_t
	len, loff_t *off)
{
	int ret;
	//trace_printk("non_rt_gpio: read()\n");
	interrupt_done = 0;
	/* wait for interrupt */
	wait_event_interruptible(wq, (interrupt_done > 0));

	if (gpio_get_value(gpio_in_id) == 1) {
		ret = copy_to_user(buf, "1", 2);
	} else {
		ret = copy_to_user(buf, "0", 2);
	}

	if (ret < 0)
		return ret;

    return 2;
}

static ssize_t non_rt_gpio_write(struct file *f, const char __user *buf,
	size_t len, loff_t *off)
{
	int ret;
    u8 buf2[100];
	//trace_printk("non_rt_gpio: write()\n");
	
	// read from user space and set new gpio output value
    ret = copy_from_user(buf2, buf, len);
    if (ret < 0)
		return ret;

	if (buf2[0] == '0')
		gpio_set_value(gpio_out_id, 0);
	else
		gpio_set_value(gpio_out_id, 1);

	return len;
}

static struct file_operations non_rt_gpio_fops =
{
	.owner = THIS_MODULE,
	.open = non_rt_gpio_open,
	.release = non_rt_gpio_close,
	.read = non_rt_gpio_read,
	.write = non_rt_gpio_write
};
 
static int __init non_rt_gpio_init(void) /* Constructor */
{
	int ret;
	
	printk(KERN_INFO "Registering non_rt_gpio ...\n");
	if ((ret = alloc_chrdev_region(&first_device, 0, 1, "non_rt_gpio_region")) < 0) {
		printk(KERN_ERR "alloc_chrdev_region error\n");
		return ret;
	}
	if ((cl = class_create(THIS_MODULE, "non_rt_gpio_chardrv")) == NULL) {
		printk(KERN_ERR "class_create error\n");
		unregister_chrdev_region(first_device, 1);
		return -1;
	}
	if (device_create(cl, NULL, first_device, NULL, "non_rt_gpio_device") == NULL) {
		printk(KERN_ERR "device_create error\n");
		class_destroy(cl);
		unregister_chrdev_region(first_device, 1);
		return -1;
	}
	cdev_init(&c_dev, &non_rt_gpio_fops);
	if ((ret = cdev_add(&c_dev, first_device, 1)) < 0) {
		printk(KERN_ERR "cdev_add error\n");
		device_destroy(cl, first_device);
		class_destroy(cl);
		unregister_chrdev_region(first_device, 1);
		return ret;
	}

	if ((ret = gpio_request_one(gpio_out_id, GPIOF_DIR_OUT, THIS_MODULE->name)) < 0) {
		printk(KERN_ERR "gpio_request_one(gpio_out_id) failed\n");
        return ret;
	}

	if ((ret = gpio_request_one(gpio_in_id, GPIOF_DIR_IN, THIS_MODULE->name)) < 0) {
		printk(KERN_ERR "gpio_request_one(gpio_in_id) error\n");
		goto rollback_gpio_out;
	}
	if ((irq_line = gpio_to_irq(gpio_in_id)) < 0){
		printk(KERN_ERR "gpio %d cannot be used as interrupt\n", gpio_in_id);
		goto rollback_gpio_in;
	}
#if 0	
    if ((ret = irq_set_irq_type(irq_n, IRQ_TYPE_EDGE_RISING)) < 0) {
		printk(KERN_ERR "error irq_set_irq_type\n");
		goto rollback_gpio_in;
	}
#endif
	if ((ret = request_irq(irq_line, non_rt_interrupt_handler,
		IRQ_TYPE_EDGE_RISING | IRQF_NO_THREAD, "non_rt_gpio interrupt\n", (void *)1)) < 0) 
	{
		printk(KERN_ERR "request_irq failed\n");
		//trace_printk("request_irq failed\n");
		goto rollback_gpio_in;
	}

	init_waitqueue_head(&wq);
	
	printk(KERN_INFO "non_rt_gpio done registering\n");
	//trace_printk("non_rt_gpio done registering\n");
	return 0;

rollback_gpio_in:	
	gpio_free(gpio_in_id);
rollback_gpio_out:
	gpio_free(gpio_out_id);
	
	return -1;
}
 
static void __exit non_rt_gpio_exit(void)
{
	free_irq(irq_line, (void*)1);
	gpio_free(gpio_in_id);
	gpio_free(gpio_out_id);
	cdev_del(&c_dev);
	device_destroy(cl, first_device);
	class_destroy(cl);
	unregister_chrdev_region(first_device, 1);
	printk(KERN_INFO "non_rt_gpio unregistered\n");
}

module_init(non_rt_gpio_init);
module_exit(non_rt_gpio_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Laurentiu-Cristian Duca");
MODULE_DESCRIPTION("real-time gpio driver for bbb");

