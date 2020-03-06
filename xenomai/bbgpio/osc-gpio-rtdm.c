#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <rtdm/driver.h>


static unsigned int irq_num;
// bbb gpios that work on PREEMPT_RT linux 4.19.59 and 5.4.5 with default dts
static unsigned int gpio_out = 48;
static unsigned int gpio_in = 115;
static bool value = false;
static rtdm_irq_t irq_handle;
static int num_of_intr = 0;


static int gpio_irq_handler(rtdm_irq_t * irq)
{
    value = !value;
    gpio_set_value(gpio_out, value); // toggle the led everytime irq handler is invoked
    trace_printk("GPIO interrupt \n");
    num_of_intr++;
    return RTDM_IRQ_HANDLED;
}


static int __init rtdm_init (void)
{
    int err;

    if ((err = gpio_request(gpio_in, THIS_MODULE->name)) != 0) {
        printk(" gpio_request gpio_in failed ! \n");
        return err;
    }

    if ((err = gpio_direction_input(gpio_in)) != 0) {
        printk(" gpio_direction_input gpio_in failed ! \n");
        gpio_free(gpio_in);

        return err;
    }

    irq_num = gpio_to_irq(gpio_in);

    printk(" IRQ number %d !  \n",irq_num);

    if ((err = gpio_request(gpio_out, THIS_MODULE->name)) != 0) {
        printk(" gpio_request gpio_out failed ! \n");
        gpio_free(gpio_in);
        return err;
    }

    if ((err = gpio_direction_output(gpio_out, 0)) != 0) {
        printk(" gpio_direction_input gpio_out failed ! \n");
        gpio_free(gpio_out);
        gpio_free(gpio_in);
        return err;
    }

    err = irq_set_irq_type(irq_num,  IRQF_TRIGGER_RISING);

    if(err) {
        gpio_free(gpio_out);
        gpio_free(gpio_in);
        printk(" irq_set_irq_type failed ! \n");
        return err;
    }

    err = rtdm_irq_request(&irq_handle,irq_num,(rtdm_irq_handler_t)gpio_irq_handler,
		RTDM_IRQTYPE_EDGE,THIS_MODULE->name, NULL);

    printk("after request irq = %d \n",irq_handle.irq);

    if(err) {
        gpio_free(gpio_out);
        gpio_free(gpio_in);
        printk(" rtdm_irq_request failed ! \n");
        return err;
    }

    err = rtdm_irq_enable(&irq_handle);

    if (err < 0) {
        printk("rtdm_irq_enable failed \n");
        return err;
    }
    return 0;


}



static void __exit rtdm_exit (void)
{
    rtdm_irq_free(&irq_handle);
    gpio_free(gpio_out);
    gpio_free(gpio_in);

    printk("The number of intr is %d \n",num_of_intr);
}


module_init(rtdm_init);
module_exit(rtdm_exit);
MODULE_LICENSE("GPL");
