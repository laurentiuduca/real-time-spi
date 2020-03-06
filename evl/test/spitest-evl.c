/*
 * Author: Laurentiu-Cristian Duca
 * Date: 2020/02/08
 */

#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <stdio.h>
#include <error.h>
#include <semaphore.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <evl/evl.h>
#include <evl/proxy.h>
#include <evl/thread.h>
#include <linux/spi/spidev.h>
// this is from uapi/evl/devices/spi.h
#include <spi.h>
#include <pthread.h>

#define DEFAULT_DEV_NAME "/dev/evl-spidev0.0"

#define MAX_SIZE 16
#define N_TESTS	 1

unsigned char *i_area, *o_area;
unsigned char r_area[MAX_SIZE], w_area[MAX_SIZE];
int fd, efd;
int transfer_size = MAX_SIZE;
int ret, speed_hz = 500000;
struct evl_spi_config config;
struct evl_spi_iobufs iobufs;
void *p;
__u32 mode;

static int do_spi_loop(int fd, int transfer_size, int val)
{
	int i, ret;
	int pass;

	for(i = 0; i < transfer_size; i++) {
		i_area[i] = 0;
		o_area[i] = i;
		r_area[i] = 0;
		w_area[i] = i;
	}

	mode = T_WOSS; /* warn on stage switch */
	ret = oob_ioctl(efd, EVL_THRIOC_SET_MODE, &mode);
	if(ret < 0) {
		evl_printf("EVL_THRIOC_SET_MODE error %d\n", ret);
		return ret;
	}
	
	if ((ret = oob_ioctl(fd, SPI_RTIOC_TRANSFER_N, transfer_size)) != 0) {
		evl_printf("SPI_RTIOC_TRANSFER_N returns %d, %s\n", ret, strerror(errno));
	}

	mode = T_WOSS; /* warn on stage switch */
	ret = oob_ioctl(efd, EVL_THRIOC_CLEAR_MODE, &mode);
	if(ret < 0) {
		evl_printf("EVL_THRIOC_SET_MODE error %d\n", ret);
		return ret;
	}
	
	evl_printf("SPI_RTIOC_TRANSFER_N: transfer_size=%d\n", transfer_size);
	pass = 1;
	for(i = 0; i < transfer_size; i++) {
		if(i_area[i] != o_area[i])
			pass = 0;
		evl_printf("%x, %x; ", 
			   o_area[i], i_area[i]);
		if(i % 16 == 0)
			evl_printf("\n");
	}
	evl_printf("\n\tpass=%d\n", pass);

#if 0
	evl_printf("writing ... \n");
	if((ret = write(fd, w_area, transfer_size)) != transfer_size) {
		evl_printf("write returns %d \n", ret);
		//return ret;
	}
	
	evl_printf("write: \n");
	for(i = 0; i < transfer_size; i++) {
		evl_printf("%x, %x; ", 
			   w_area[i], r_area[i]);
		if(i % 16 == 0)
			evl_printf("\n");
	}
	evl_printf("\n");

	evl_printf("reading ... \n");
	if((ret = read (fd, r_area, transfer_size)) != transfer_size) {
		evl_printf("read returns %d \n", ret);
		//return ret;
	}
	
	evl_printf("read: \n");
	for(i = 0; i < transfer_size; i++) {
		evl_printf("%x, %x; ", 
			   w_area[i], r_area[i]);
		if(i % 16 == 0)
			evl_printf("\n");
	}
	evl_printf("\n");
#endif	
	return 0;
}

static int configure(char * str)
{	
	iobufs.io_len = transfer_size;
	if ((ret = ioctl(fd, SPI_RTIOC_SET_IOBUFS, &iobufs))) {
		evl_printf("SPI_RTIOC_SET_IOBUFS returns %d \n", ret);
		return ret;
	}

	p = mmap(0, getpagesize(), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED)
		return -EINVAL;

	evl_printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u \n",
			 iobufs.i_offset, iobufs.i_offset + transfer_size - 1,
			 iobufs.o_offset, iobufs.o_offset + transfer_size - 1,
			 iobufs.map_len);

	i_area = p + iobufs.i_offset;
	o_area = p + iobufs.o_offset;

	config.mode = SPI_MODE_0;
	config.bits_per_word = 8;
	config.speed_hz = speed_hz;
	if ((ret = oob_ioctl(fd, SPI_RTIOC_SET_CONFIG, &config))) {
		evl_printf("SPI_RTIOC_SET_CONFIG error: %d %s\n", ret, strerror(errno));
		return ret;
	}

	if ((ret = oob_ioctl(fd, SPI_RTIOC_GET_CONFIG, &config))) {
		evl_printf("SPI_RTIOC_GET_CONFIG error: %d \n", ret);
		return ret;
	}

	evl_printf("speed=%u hz, mode=%#x, bits=%u \n",
			 config.speed_hz, config.mode, config.bits_per_word);
	
	return 0;
}

static void sigdebug_handler(int sig, siginfo_t *si, void *context)
{
    evl_sigdebug_handler(sig, si, context);
}

static int run_spi_transfer(char * str)
{
	const char *device = str;
	int ret, i;

	fd = open(device, O_RDWR);
	if (fd < 0) {
		ret = -errno;
		evl_printf("cannot open device %s \n", device);
		return ret;
	}

	ret = configure(str);
	if(ret < 0)
		return ret;

	for(i = 1; i <= N_TESTS; i++)
		do_spi_loop(fd, transfer_size, i);

	// cleaning up
	if((ret = munmap(p, iobufs.map_len))) {
		evl_printf("munmap returns %d\n", ret);
		return ret;
	}

	ret = close(fd);
	return ret;	
}

int main(int argc, char *const argv[])
{
	int ret;
	struct sigaction sa;
    struct sched_param sp;

	/* setup oob to ib signal */
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = sigdebug_handler;
	sa.sa_flags = SA_SIGINFO;
	sigaction(SIGDEBUG, &sa, NULL);

	/* become real time */
    if((ret = mlockall(MCL_FUTURE|MCL_CURRENT)) < 0) {
        printf("mlockall failed: %s\n", strerror(ret));
        return -1;
    }
    sp.sched_priority = 80;
    if((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) != 0) {
        printf("Failed to set stepper thread to real-time priority: %s\n", strerror(ret));
        return -1;
    }
	
	/* Attach the current thread to the EVL core. */
	efd = evl_attach_self("spitest-evl-%d", getpid());

	evl_printf("Thread %d in evl\n", efd);

	if(argc < 2) {
		evl_printf("using default dev name: %s \n", DEFAULT_DEV_NAME);
		ret = run_spi_transfer(DEFAULT_DEV_NAME);
	} else {
		evl_printf("using dev name: %s \n", argv[1]);
		ret = run_spi_transfer(argv[1]);
	} 
	evl_printf("run_spi_transfer returns %d\n", ret);

	/* Then detach it. */
	evl_detach_self();
	
	return 0;
}
