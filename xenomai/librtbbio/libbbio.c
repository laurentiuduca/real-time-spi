#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <alchemy/task.h>

#include "bbgpio.h"
//#include <bbpwm.h>

#include "libbbio.h"

int bbgpio_open_input(char *gpio)
{
	return __RT(open(gpio, BBGPIO_DIR_IN));
}

int bbgpio_open_output(char *gpio)
{
	return __RT(open(gpio, BBGPIO_DIR_OUT));
}

int bbgpio_open_irq(char *gpio)
{
	return __RT(open(gpio, BBGPIO_DIR_IRQ));
}

int bbgpio_get(int dev)
{
	int ret;
	char buf[10];

	ret = __RT(read(dev, buf, 10));
	if (ret < 0) {
		return ret;
	}

	buf[ret] = 0;
	return atoi(buf);
}

int bbgpio_set(int dev, int value)
{
	char buf[10];

	if (value < 0) {
		return -EINVAL;
	}

	if (value > 1) {
		return -EINVAL;
	}

	sprintf(buf, "%d", value);
	return __RT(write(dev, buf, 10));
}


int bbgpio_dir_get(int dev)
{
	int ret;
	int dir;
	ret = __RT(ioctl(dev, BBGPIO_IOCTL_DIR_GET, &dir));

	if (ret < 0) {
		return ret;
	}

    if (dir == BBGPIO_DIR_IN) {
        return DIR_IN;
    } else if (dir == BBGPIO_DIR_OUT) {
        return DIR_OUT;
    } else if (dir == BBGPIO_DIR_IRQ) {
        return DIR_IRQ;
    } else {
        return -1;
    }
}

int bbgpio_edge_get(int dev)
{
	int ret;
	int edge;
	ret = __RT(ioctl(dev, BBGPIO_IOCTL_EDGE_GET, &edge));

	if (ret < 0) {
		return ret;
	}

   if (edge == BBGPIO_IOCTL_EDGE_RISING) {
        return EDGE_RISING;
    } else if (edge == BBGPIO_IOCTL_EDGE_FALLING) {
        return EDGE_FALLING;
    } else if (edge == BBGPIO_IOCTL_EDGE_BOTH) {
        return EDGE_BOTH;
    } else {
        return -1;
    }
}

int bbgpio_edge_set(int dev, int edge)
{
    int edge_ioctl;

    if (edge == EDGE_RISING) {
        edge_ioctl = BBGPIO_IOCTL_EDGE_RISING;
    } else if (edge == EDGE_FALLING) {
        edge_ioctl = BBGPIO_IOCTL_EDGE_FALLING;
    } else if (edge == EDGE_BOTH) {
        edge_ioctl = BBGPIO_IOCTL_EDGE_BOTH;
    } else {
        return -1;
    }

	return __RT(ioctl(dev, BBGPIO_IOCTL_EDGE_SET, &edge_ioctl));
}

nanosecs_rel_t bbgpio_timeout_get(int dev)
{
	int ret;
    nanosecs_rel_t timeout_ns;
	ret = __RT(ioctl(dev, BBGPIO_IOCTL_TIMEOUT_GET, &timeout_ns));

	if (ret < 0) {
		return ret;
	}

	return timeout_ns;
}

int bbgpio_timeout_set(int dev, nanosecs_rel_t timeout_ns)
{
	return __RT(ioctl(dev, BBGPIO_IOCTL_TIMEOUT_SET, &timeout_ns));
}

void bbgpio_close(int dev)
{
	__RT(close(dev));
}
/*
int bbpwm_open(char *pwm, int period_ms)
{
	return __RT(open(pwm, period_ms));
}

int bbpwm_get_period(int dev)
{
	int ret;
	int period;
	ret = __RT(ioctl(dev, BBPWM_IOCTL_PERIOD_GET, &period));

	if (ret < 0) {
		return ret;
	}

	return period;
}

int bbpwm_get_duty_a(int dev)
{
	int ret;
	int duty;
	ret = __RT(ioctl(dev, BBPWM_IOCTL_DUTYA_GET, &duty));

	if (ret < 0) {
		return ret;
	}

	return duty;
}

int bbpwm_get_duty_ns_a(int dev)
{
	int ret;
	int duty_ns;
	ret = __RT(ioctl(dev, BBPWM_IOCTL_DUTYA_NS_GET, &duty_ns));

	if (ret < 0) {
		return ret;
	}

	return duty_ns;
}

int bbpwm_get_duty_b(int dev)
{
	int ret;
	int duty;
	ret = __RT(ioctl(dev, BBPWM_IOCTL_DUTYB_GET, &duty));

	if (ret < 0) {
		return ret;
	}

	return duty;
}

int bbpwm_get_duty_ns_b(int dev)
{
	int ret;
	int duty_ns;
	ret = __RT(ioctl(dev, BBPWM_IOCTL_DUTYB_NS_GET, &duty_ns));

	if (ret < 0) {
		return ret;
	}

	return duty_ns;
}

int bbpwm_set_duty_a(int dev, int duty)
{
	return __RT(ioctl(dev, BBPWM_IOCTL_DUTYA_SET, &duty));
}

int bbpwm_set_duty_ns_a(int dev, int duty_ns)
{
	return __RT(ioctl(dev, BBPWM_IOCTL_DUTYA_NS_SET, &duty_ns));
}

int bbpwm_set_duty_b(int dev, int duty)
{
	return __RT(ioctl(dev, BBPWM_IOCTL_DUTYB_SET, &duty));
}

int bbpwm_set_duty_ns_b(int dev, int duty_ns)
{
	return __RT(ioctl(dev, BBPWM_IOCTL_DUTYB_NS_SET, &duty_ns));
}

int bbpwm_close(int dev)
{
	return __RT(close(dev));
}
*/
