#include <stdio.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/mman.h>
#include <alchemy/task.h>

#include "libbbio.h"




/*static void bbio_gpio_input(char *dev)
{
	int fd = bbgpio_open_input(dev);
    printf("bbio test read input %s %i\n", dev, bbgpio_get(fd));
    bbgpio_close(fd);
}

static void bbio_gpio_output(char *dev, int value)
{
	int fd = bbgpio_open_input(dev);
	printf("bbio test set output %s to value %i\n", dev, value);
	bbgpio_set(fd, value);
	rt_task_sleep(1000000000);
	bbgpio_close(fd);
}

static void bbio_gpio_irq(char *dev)
{

}

static void bbio_pwm(char *dev)
{

}*/


static void bbgpio_input_test(char *input, char *output)
{
	int fd_input;
	int fd_output;

	if ((input == NULL) || (output == NULL)) {
		rt_printf("missing arguments - bbio_test -t <input/irq/pwm> (-i <input dev> -o <output dev> -p <pwm dev>)\n");
		exit(EXIT_FAILURE);
	}

	rt_printf("do input test - %s - %s\n", input, output);

	fd_input = bbgpio_open_input(input);
	if (fd_input < 0) {
		rt_printf("fails to open input GPIO %i\n", fd_input);
		exit(EXIT_FAILURE);
	}

	fd_output = bbgpio_open_output(output);
	if (fd_output < 0) {
		rt_printf("fails to open output GPIO %i\n", fd_output);
		exit(EXIT_FAILURE);
	}

	rt_printf("read input %i\n", bbgpio_get(fd_input));
	rt_task_sleep(100000000);
	rt_printf("set high %i\n", bbgpio_set(fd_output, 1));
	rt_task_sleep(100000000);
	rt_printf("read input high %i\n", bbgpio_get(fd_input));
	rt_task_sleep(100000000);
	rt_printf("set low %i\n", bbgpio_set(fd_output, 0));
	rt_task_sleep(100000000);
	rt_printf("read input low %i\n", bbgpio_get(fd_input));

	bbgpio_close(fd_input);
	bbgpio_close(fd_output);
}

static void bbgpio_irq_test(char *input, char *output)
{

}

int main(int argc, char **argv)
{
	int ret;
	int c;
	char *test = NULL;
	char *input = NULL;
	char *output = NULL;
	RT_TASK main_task;

	opterr = 0;

	while ((c = getopt(argc, argv, "t:i:o:")) != -1) {
		switch(c) {
		case 't':
			test = optarg;
			break;
		case 'i':
			input = optarg;
			break;
		case 'o':
			output = optarg;
			break;
		default:
			printf("incorrect arguments\n");
			exit(EXIT_FAILURE);
		}
	}

	if (test == NULL) {
		printf("missing arguments - bbio_test -t <input/irq/pwm> (-i <input dev> -o <output dev> -p <pwm dev>)\n");
		exit(EXIT_FAILURE);
	}

	ret = rt_task_shadow(&main_task, NULL, 60, 0);
	if (ret < 0) {
		printf("making xenomai task failed : %s\n", strerror(-ret));
		exit(EXIT_FAILURE);
	}

	if (!strncmp(test, "input", 5)) {
		// normal input test
		bbgpio_input_test(input, output);
	} else	if (!strncmp(test, "irq", 3)) {
		// normal irq test
		bbgpio_irq_test(input, output);
	} else {
		rt_printf("missing arguments - bbio_test -t <input/irq/pwm> (-i <input dev> -o <output dev> -p <pwm dev>)\n");
	}

	exit(EXIT_SUCCESS);
}


