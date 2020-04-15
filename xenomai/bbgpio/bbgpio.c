#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <rtdm/rtdm.h>
#include <rtdm/driver.h>

#include "bbgpio.h"

//! struct with data representing a GPIO device.
struct bbgpio_data {
    //! the direction of the GPIO
    int dir;
    //! the the edge the GPIO IRQ will trigger on
    int edge;
    //! the GPIO channel number
    int gpio_n;
    //! how long a read function will wait for an IRQ
    nanosecs_rel_t timeout;
    //! the RTDM IRQ channel
    rtdm_irq_t irq;
    //! the RTDM event used by IRQ handler to signal any waiting tasks
    rtdm_event_t irq_event;
};

/*!
 * Method for handling an open action. Depending on the oflags, the GPIO
 * will be opened as an input, output or input IRQ.
 * @param fd rtdm file descriptor pointer
 * @param oflags the value given when opening the file
 * @return 0 on success, otherwise an error code
 */
static int bbgpio_open(struct rtdm_fd *fd, int oflags);
/*!
 * Method for handling a read action. For input GPIO the current
 * GPIO value is returned. IRQ GPIO will block until an interrupt
 * occurs or a timeout. GPIO output channels return error.
 * @param fd rtdm file descriptor pointer
 * @param buf user space read buffer
 * @param size size of user read buffer
 * @return 0 on success, otherwise an error code
 */
static ssize_t bbgpio_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size);
/*!
 * Method for handling a write action. For output GPIO the value provided
 * is set on the GPIO output. GPIO input and IRQ GPIO channels return error.
 * @param fd rtdm file descriptor pointer
 * @param buf user space write buffer
 * @param size size of user write buffer
 * @return 0 on success, otherwise an error code
 */
static ssize_t bbgpio_write_rt(struct rtdm_fd *fd, const void __user *buf, size_t size);
/*!
 * Method for handling a ioctl action.
 * DIR GET return the direction of all GPIO channels.
 * EDGE GET return the edge of IRQ GPIO channels, otherwise error.
 * EDGE SET set the edge of IRQ GPIO channels, otherwise error.
 * TIMEOUT GET return the timeout of IRQ GPIO channels, otherwise error.
 * TIMEOUT SET set the timeout of IRQ GPIO channels, otherwise error.
 * @param fd rtdm file descriptor pointer
 * @param request the ioctl request code
 * @param arg pointer to input or output data
 * @return 0 on success, otherwise an error code
 */
static int bbgpio_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void *arg);
/*!
 * Method for handling a close action, which will disable the GPIO device.
 * @param fd rtdm file descriptor pointer
 */
static void bbgpio_close(struct rtdm_fd *fd);
/*!
 * IRQ handler, will trigger an event, which wake up waiting read methods.
 * @param irq the RTDM IRQ channel
 * @return always 1
 */
static int bbgpio_interrupt(rtdm_irq_t *irq);
/*!
 * Initialize a GPIO channel of the given number. Does not activate the GPIO.
 * @param n GPIO number (in the list of this driver)
 * @return 0 on success, otherwise an error code
 */ 
static int bbgpio_device_init(int n);
/*
 * Exit a GPIO channel of a given number.
 * @param n GPIO number (in the list of this driver)
 */
static void bbgpio_device_exit(int n);


//! initialization of GPIO data struct array, with all GPIO that can be used
//! without changing the mux settings
static struct bbgpio_data _bbgpio_data[] = {
    {.gpio_n = 20},
    {.gpio_n = 26},
    {.gpio_n = 27},
    {.gpio_n = 44},
    {.gpio_n = 45},
    {.gpio_n = 46},
    {.gpio_n = 47},
    {.gpio_n = 48},
    {.gpio_n = 60},
    {.gpio_n = 61},
    {.gpio_n = 65},
    {.gpio_n = 66},
    {.gpio_n = 67},
    {.gpio_n = 68},
    {.gpio_n = 69},
    {.gpio_n = 112},
    {.gpio_n = 115},
    {.gpio_n = 117},
    {.gpio_n = 86},
    {.gpio_n = 87},
    {.gpio_n = 10},
    {.gpio_n = 9},
    {.gpio_n = 8},
    {.gpio_n = 78},
    {.gpio_n = 76},
    {.gpio_n = 74},
    {.gpio_n = 72},
    {.gpio_n = 88},
    {.gpio_n = 89},
    {.gpio_n = 11},
    {.gpio_n = 79},
    {.gpio_n = 77},
    {.gpio_n = 75},
    {.gpio_n = 73},
    {.gpio_n = 38},
    {.gpio_n = 39},
    {.gpio_n = 34},
    {.gpio_n = 35},
    {.gpio_n = 63},
    {.gpio_n = 62},
    {.gpio_n = 37},
    {.gpio_n = 36},
    {.gpio_n = 33},
    {.gpio_n = 32},
};

#define RTDM_SUBCLASS_GPIO_IRQ 4711

//! the driver for the Beaglebone GPIO RTDM driver
static struct rtdm_driver _bbgpio_driver = {
    .profile_info = RTDM_PROFILE_INFO("bbgpio", RTDM_CLASS_EXPERIMENTAL,
			RTDM_SUBCLASS_GPIO_IRQ, 42),
            //RTDM_SUBCLASS_GENERIC, 42),
    .device_flags = RTDM_NAMED_DEVICE, // | RTDM_EXCLUSIVE,
    .device_count = ARRAY_SIZE(_bbgpio_data),
    .context_size = sizeof(struct bbgpio_data),
    .ops = {
        .open = bbgpio_open,
        .read_rt = bbgpio_read_rt,
        .write_rt = bbgpio_write_rt,
        .ioctl_rt = bbgpio_ioctl_rt,
        .close = bbgpio_close,
    },
};

//! initialization of GPIO device struct array, with all GPIO that can be used
//! without changing the mux settings with name indicating pin position
static struct rtdm_device _bbgpio_devices[] = {
    {
        .driver = &_bbgpio_driver,
        .label = "gpio20_P9_41",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio26_P8_14",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio27_P8_17",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio44_P8_12",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio45_P8_11",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio46_P8_16",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio47_P8_15",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio48_P9_15",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio60_P9_12",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio61_P8_26",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio65_P8_18",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio66_P8_07",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio67_P8_08",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio68_P8_10",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio69_P8_09",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio112_P9_13",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio115_P9_27",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio117_P9_25",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio86_P8_27_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio87_P8_29_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio10_P8_31_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio9_P8_33_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio8_P8_35_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio78_P8_37_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio76_P8_39_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio74_P8_41_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio72_P8_43_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio88_P8_28_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio89_P8_30_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio11_P8_32_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio79_P8_38_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio77_P8_40_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio75_P8_42_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio73_P8_44_lcd",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio38_P8_03_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio39_P8_04_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio34_P8_05_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio35_P8_06_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio63_P8_20_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio62_P8_21_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio37_P8_22_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio36_P8_23_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio33_P8_24_mmc",
    },
    {
        .driver = &_bbgpio_driver,
        .label = "gpio32_P8_25_mmc",
    },
};


static int n_interrupts = 0;
static int bbgpio_interrupt(rtdm_irq_t *irq)
{
    struct bbgpio_data *data =  rtdm_irq_get_arg(irq, struct bbgpio_data);

	n_interrupts++;
	//trace_printk("%s: n_interrupts=%d\n", __func__, n_interrupts);
    // wake up any read methods that are waiting for the interrupt
	rtdm_event_signal(&data->irq_event);
    //rtdm_event_pulse(&data->irq_event);
    return RTDM_IRQ_HANDLED;
}

static int bbgpio_open(struct rtdm_fd *fd, int oflags)
{
    int ret = 0;
    int irq_n;
    struct rtdm_dev_context *context;
    struct bbgpio_data *data;

    context = rtdm_fd_to_context(fd);
    data = (struct bbgpio_data*)context->device->device_data;

    data->dir = oflags;
    data->edge = 0;

    switch (data->dir) {
    case BBGPIO_DIR_IN:
        // request input GPIO
        ret = gpio_request_one(data->gpio_n, GPIOF_DIR_IN, THIS_MODULE->name);
        if (ret < 0) {
            goto rollback_request;
        }

        break;

    case BBGPIO_DIR_OUT:
        // request output GPIO
        ret = gpio_request_one(data->gpio_n, GPIOF_DIR_OUT, THIS_MODULE->name);
        if (ret < 0) {
            goto rollback_request;
        }

        break;

    case BBGPIO_DIR_IRQ:
        irq_n = gpio_to_irq(data->gpio_n);

        // request GPIO IRQ
        ret = gpio_request_one(data->gpio_n, GPIOF_DIR_IN, THIS_MODULE->name);
        if (ret < 0) {
            goto rollback_request;
        }

        // set trigger on rising edge as default
        data->edge = BBGPIO_IOCTL_EDGE_RISING;
        ret = irq_set_irq_type(irq_n, IRQ_TYPE_EDGE_RISING);
        if (ret < 0) {
            goto rollback_irq;
        }

        // request IRQ
        ret = rtdm_irq_request(&data->irq, irq_n, bbgpio_interrupt,
                RTDM_IRQTYPE_EDGE, THIS_MODULE->name, (void*)data);
        if (ret < 0) {
            goto rollback_irq;
        }

        // enable IRQ
        ret = rtdm_irq_enable(&data->irq);
        if (ret < 0) {
            goto rollback_irq_enable;
        }

        // initialize the event that will be used to signal waiting read methods.
        rtdm_event_init(&data->irq_event, 0);

        break;

    default:
        return -EPERM;
    }

    return 0; /* success */

rollback_irq_enable:
    rtdm_irq_free(&data->irq);
rollback_irq:
    gpio_free(data->gpio_n);
rollback_request:
    return ret;
}

static ssize_t bbgpio_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size) {
    s16 ret;
    struct rtdm_dev_context *context;
    struct bbgpio_data *data;
	int val;
	
    context = rtdm_fd_to_context(fd);
    data = (struct bbgpio_data*)context->device->device_data;

    switch (data->dir) {
    case BBGPIO_DIR_IRQ:
		//trace_printk("%s(): n_interrupts=%d\n", __func__, n_interrupts);
        // wait for an interrupt; timeout is nanoseconds
        //ret = rtdm_event_timedwait(&(data->irq_event), data->timeout, NULL);
		ret = rtdm_event_wait(&(data->irq_event));
        if (ret < 0) {
			rtdm_printk("%s(): error rtdm_event_wait\n", __func__);
            return ret;
        }

        // no break, do the same as for input
			
    case BBGPIO_DIR_IN:
        // read GPIO value and return to user space
        // is performed for both input and IRQ

        if ((val = gpio_get_value(data->gpio_n)) != 0) {
            ret = rtdm_safe_copy_to_user(fd, buf, "1", 2);
        } else {
            ret = rtdm_safe_copy_to_user(fd, buf, "0", 2);
        }
        if (ret < 0) {
			rtdm_printk("%s(): error rtdm_safe_copy_to_user\n", __func__);
            return ret;
        }
		//trace_printk("%s(): val=%d\n", __func__, val);
        return 2;
    
    default:
        return -EPERM;
    }
}

static ssize_t bbgpio_write_rt(struct rtdm_fd *fd, const void __user *buf, size_t size)
{
    s16 ret;
    u8 buf2[100];
    struct rtdm_dev_context *context;
    struct bbgpio_data *data;

    context = rtdm_fd_to_context(fd);
    data = (struct bbgpio_data*)context->device->device_data;

    switch (data->dir) {
    case BBGPIO_DIR_OUT:
        // read from user space and set new GPIO output value
        ret = rtdm_safe_copy_from_user(fd, buf2, buf, size);
        if (ret < 0) {
            return ret;
        }
        buf2[size] = 0;

        if (buf2[0] == '0') {
            gpio_set_value(data->gpio_n, 0);
        } else {
            gpio_set_value(data->gpio_n, 1);
        }

        return size;
    
    default:
        return -EPERM;
    }
}

static int bbgpio_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void *arg)
{
    struct rtdm_dev_context *context;
    struct bbgpio_data *data;

    context = rtdm_fd_to_context(fd);
    data = (struct bbgpio_data*)context->device->device_data;

    switch (request) {
    case BBGPIO_IOCTL_DIR_GET:
        // return the direction of the GPIO channel
        return rtdm_safe_copy_to_user(fd, arg, &(data->dir), sizeof(data->dir));

    case BBGPIO_IOCTL_EDGE_GET:
        if (data->dir != BBGPIO_DIR_IRQ) {
            return -EPERM;
        }
        // if channel is IRQ, return its current edge
        return rtdm_safe_copy_to_user(fd, arg, &(data->edge), sizeof(data->edge));

    case BBGPIO_IOCTL_EDGE_SET:
        if (data->dir != BBGPIO_DIR_IRQ) {
			printk(KERN_ERR "data->dir(%d) != BBGPIO_DIR_IRQ(%d)\n", data->dir, BBGPIO_DIR_IRQ);
            return -EPERM;
        }

        // if channel is IRQ, set the edge that trigger interrupt
        data->edge = (int)arg;
        switch (data->edge) {
        case BBGPIO_IOCTL_EDGE_RISING:
            return irq_set_irq_type(gpio_to_irq(data->gpio_n), IRQ_TYPE_EDGE_RISING);
        case BBGPIO_IOCTL_EDGE_FALLING:
            return irq_set_irq_type(gpio_to_irq(data->gpio_n), IRQ_TYPE_EDGE_FALLING);
        case BBGPIO_IOCTL_EDGE_BOTH:
            return irq_set_irq_type(gpio_to_irq(data->gpio_n), IRQ_TYPE_EDGE_BOTH);
        default:
            return -EINVAL;
        }

    case BBGPIO_IOCTL_TIMEOUT_GET:
        if (data->dir != BBGPIO_DIR_IRQ) {
            return -EPERM;
        }
        // if channel is IRQ, return its current interrupt timeout
        return rtdm_safe_copy_to_user(fd, arg, &data->timeout, sizeof(data->timeout));

    case BBGPIO_IOCTL_TIMEOUT_SET:
        if (data->dir != BBGPIO_DIR_IRQ) {
            return -EPERM;
        }

        // if channel is IRQ, set the interrupt timeout
		data->timeout = (int)arg;
		return 0;

    default:
        return -ENOTTY;
    }
}

static void bbgpio_close(struct rtdm_fd *fd)
{
    struct rtdm_dev_context *context;
    struct bbgpio_data *data;

    // free GPIO 
    context = rtdm_fd_to_context(fd);
    data = (struct bbgpio_data*)context->device->device_data;

    if (data->dir == BBGPIO_DIR_IRQ) {
        rtdm_irq_free(&data->irq);
    }

    gpio_free(data->gpio_n);
}

static int bbgpio_device_init(int n)
{
    int ret;

    // register the RTDM device
    _bbgpio_devices[n].device_data = &_bbgpio_data[n];
    ret = rtdm_dev_register(&_bbgpio_devices[n]);
    if (ret < 0) {
        rtdm_printk(KERN_ERR "BBGPIO ERROR initialize %i\n", n);
        goto rollback_dev_register;
    }

    return 0; // success

rollback_dev_register:
    return ret;
}


static void bbgpio_device_exit(int n)
{
    // free the RTDM device
    rtdm_dev_unregister(&_bbgpio_devices[n]);
}

int __init bbgpio_init(void)
{
    u16 i;
    s16 ret;

    // initialize all the defined RTDM GPIO devices
    for (i = 0; i < ARRAY_SIZE(_bbgpio_data); i++) {
        ret = bbgpio_device_init(i);
        if (ret < 0) {
            return -1;
        }
    }

    rtdm_printk(KERN_INFO "BBGPIO initialized\n");

    return 0; // success
}

void __exit bbgpio_exit(void)
{
    u16 i;

    // remove all the defined RTDM GPIO devices
    for (i = 0; i < ARRAY_SIZE(_bbgpio_devices); i++) {
        bbgpio_device_exit(i);
    }

    rtdm_printk(KERN_INFO "BBGPIO exit\n");
}

module_init(bbgpio_init);
module_exit(bbgpio_exit);

MODULE_AUTHOR("Øyvind Netland <oyvind@netland.name>");
MODULE_DESCRIPTION("RTDM GPIO module for Beaglebone");
MODULE_LICENSE("GPL");
