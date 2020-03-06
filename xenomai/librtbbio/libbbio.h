#ifndef INCLUDE_LIBBBIO_H_
#define INCLUDE_LIBBBIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <alchemy/task.h>

#include <rtdm/rtdm.h>

//! code indicating GPIO input direction
#define DIR_IN             1
//! code indicating GPIO output direction
#define DIR_OUT            2
//! code indicating GPIO IRQ input direction
#define DIR_IRQ            3

//! code indicating GPIO IRQ rising edge
#define EDGE_RISING        1
//! code indicating GPIO IRQ falling edge
#define EDGE_FALLING       2
//! code indicating GPIO IRQ both edges
#define EDGE_BOTH          3

int bbgpio_open_input(char *gpio);
int bbgpio_open_output(char *gpio);
int bbgpio_open_irq(char *gpio);
int bbgpio_get(int dev);
int bbgpio_set(int dev, int value);
int bbgpio_dir_get(int dev);
int bbgpio_edge_get(int dev);
int bbgpio_edge_set(int dev, int edge);
nanosecs_rel_t bbgpio_timeout_get(int dev);
int bbgpio_timeout_set(int dev, nanosecs_rel_t timout_ms);
void bbgpio_close(int dev);

int bbpwm_open(char *pwm, int period_ms);
int bbpwm_get_period(int dev);
int bbpwm_get_duty_a(int dev);
int bbpwm_get_duty_ns_a(int dev);
int bbpwm_get_duty_b(int dev);
int bbpwm_get_duty_ns_b(int dev);
int bbpwm_set_duty_a(int dev, int duty);
int bbpwm_set_duty_us_a(int dev, int duty_ns);
int bbpwm_set_duty_b(int dev, int duty);
int bbpwm_set_duty_us_b(int dev, int duty_ns);
int bbpwm_close(int dev);

#endif /* INCLUDE_LIBBBIO_H_ */



