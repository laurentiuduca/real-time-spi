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
#include <linux/swait.h>

#include <linux/gpio.h>

// To view trace_printk() output:
// cat /sys/kernel/debug/tracing/trace
// after mounting debugfs:
// mount -t debugfs none /sys/kernel/debug

//static DEFINE_MUTEX(access_mutex);

static u8 gpio_in_id = 115, gpio_out_id = 48;
static int irq_line, n_interrupts, interrupt_done = 0;

static dev_t first_device; // the first device number
static struct cdev c_dev; // the character device structure
static struct class *cl; // the device class
static struct device *dev;

static struct gpio_desc *gpiod_in, *gpiod_out;

static struct swait_queue_head head_swait;

static irqreturn_t rt_interrupt_handler(int irq, void *data)
{
#if 0
	static int val = 0;
	val = !val;
	gpiod_set_raw_value(gpiod_out, val);
#endif
	n_interrupts++;

	/* use swait.h model to signal interrupt arrived */
	interrupt_done = 1;
	smp_mb();
 	if (swait_active(&head_swait))
		swake_up_one(&head_swait);

	//trace_printk("%s: n_interrupts=%d\n", __func__, n_interrupts);
	return IRQ_HANDLED;
}

static int rt_gpio_open(struct inode *i, struct file *f)
{
	/* single thread only */
	//mutex_lock(&access_mutex);
	return 0;
}

static int rt_gpio_close(struct inode *i, struct file *f)
{
	//mutex_unlock(&access_mutex);
	return 0;
}

static ssize_t rt_gpio_read(struct file *f, char __user *buf, size_t
	len, loff_t *off)
{
	int ret;
	DECLARE_SWAITQUEUE(swait);

	//trace_printk("rt_gpio: read()\n");
	/* wait for interrupt using swait.h model */
	for (;;) {
		prepare_to_swait_exclusive(&head_swait, &swait, TASK_INTERRUPTIBLE);
 		/* smp_mb() from set_current_state() */
 		if (interrupt_done)
 			break;
 		schedule();
 	}
	finish_swait(&head_swait, &swait);
	interrupt_done = 0;
	smp_mb();
	
	if (gpiod_get_raw_value(gpiod_in) == 1) {
		ret = copy_to_user(buf, "1", 2);
	} else {
		ret = copy_to_user(buf, "0", 2);
	}

	if (ret < 0)
		return ret;

    return 2;
}

static ssize_t rt_gpio_write(struct file *f, const char __user *buf,
	size_t len, loff_t *off)
{
	int ret;
    u8 buf_kernel[100];
	//trace_printk("rt_gpio: write()\n");

	// read from user space and set new gpio output value
    ret = copy_from_user(buf_kernel, buf, len);
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
	.read = rt_gpio_read,
	.write = rt_gpio_write
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
	if(gpiod_direction_input(gpiod_in) < 0) {
		printk(KERN_ERR "gpio_direction_input error\n");
		goto rollback_gpio_in;
	}
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
		IRQ_TYPE_EDGE_RISING | IRQF_NO_THREAD, "rt_gpio interrupt\n", (void *)1)) < 0) 
	{
		printk(KERN_ERR "request_irq failed\n");
		//raw_printk("request_irq failed\n");
		goto rollback_gpio_in;
	}

	init_swait_queue_head(&head_swait);
	
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
MODULE_DESCRIPTION("real-time gpio driver for bbb");

