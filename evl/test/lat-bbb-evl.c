// Author: Laurentiu-Cristian Duca
// License: GNU GPL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/mman.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <error.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <evl/evl.h>
#include <evl/proxy.h>
#include <evl/thread.h>
#include <linux/spi/spidev.h>
// this is from uapi/evl/devices/spi.h
#include <spi.h>
#include <pthread.h>

#define DEBUG_APPLICATION 0
#define debug_printf(pr_str) do { if (DEBUG_APPLICATION) evl_printf pr_str; } while(0) 

#define FPGA_CLOCK_FREQ 50000000.00
#define EVL_SPI_BUFFER_SIZE	100
#define EVL_SPI_SPEED_HZ	(2*1000*1000)
#define EVL_N_TESTS	(1000*1000)

#define pinInputIRQ	int_dev_fd
#define pinOutput	intack_dev_fd

static unsigned char *rx_buffer, *tx_buffer;
// mmap memory area
void *p;
struct evl_spi_config config;
struct evl_spi_iobufs iobufs;
__u32 mode;

/** Handle to the rtdm spi driver instance. */
int spi_fd = -1, int_dev_fd = -1, intack_dev_fd = -1;
int efd;
unsigned short int int_lat[EVL_N_TESTS], spi_lat[EVL_N_TESTS];
FILE *f;

/**
 * Open and configure spi device
 * @return 0 in case of success, a negative value otherwise.
 */
int open_spi_device(char* spi_dev_name) 
{
	int ret;

	/* Open device */
	evl_printf("%s O_RDWR\n", spi_dev_name);
	spi_fd = open(spi_dev_name, O_RDWR);
	if (spi_fd < 0) {
		evl_printf("%s: Could not open spi device, open has failed with %d (%s).\n", 
			   __FUNCTION__, errno, strerror(errno));
		return -1;
	} else {
		evl_printf("%s: Device opened.\n", __FUNCTION__);
	}

	/* Configure device */
	iobufs.io_len = EVL_SPI_BUFFER_SIZE;
	if ((ret = ioctl(spi_fd, SPI_RTIOC_SET_IOBUFS, &iobufs))) {
		perror("SPI_RTIOC_SET_IOBUFS");
		evl_printf("SPI_RTIOC_SET_IOBUFS returns %d \n", ret);
		return ret;
	}

	p = mmap(NULL, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, spi_fd, 0);
	if (p == MAP_FAILED)
		return -EINVAL;

	evl_printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u \n",
			 iobufs.i_offset, iobufs.i_offset + EVL_SPI_BUFFER_SIZE - 1,
			 iobufs.o_offset, iobufs.o_offset + EVL_SPI_BUFFER_SIZE - 1,
			 iobufs.map_len);

	rx_buffer = p + iobufs.i_offset;
	tx_buffer = p + iobufs.o_offset;

	config.mode = SPI_MODE_0;
	config.bits_per_word = 8;
	config.speed_hz = EVL_SPI_SPEED_HZ;

	if ((ret = oob_ioctl(spi_fd, SPI_RTIOC_SET_CONFIG, &config))) {
		evl_printf("SPI_RTIOC_SET_CONFIG error: %d %s\n", ret, strerror(errno));
		return ret;
	}
	if ((ret = oob_ioctl(spi_fd, SPI_RTIOC_GET_CONFIG, &config))) {
		evl_printf("SPI_RTIOC_GET_CONFIG error: %d %s\n", ret, strerror(errno));
		return ret;
	}

	evl_printf("speed=%u hz, mode=%#x, bits=%u \n",
			 config.speed_hz, config.mode, config.bits_per_word);		

	evl_printf("%s: Device successfully configured.\n", __FUNCTION__);
	return 0;
}

int configure_pins(char *int_dev_name, char *intack_dev_name)
{
	int ret;

	/* Open device for interrupt input */
	evl_printf("%s O_RDWR\n", int_dev_name);
	ret = open(int_dev_name, O_RDWR);
	if (ret < 0) {
		evl_printf("%s: Could not open device %s for interrupt input, open has failed with %d (%s).\n", 
			   __FUNCTION__, int_dev_name, errno, strerror(errno));
		return -1;
	} else {
		evl_printf("%s: Device for interrupt input opened.\n", __FUNCTION__);
		int_dev_fd = ret;
	}

	intack_dev_fd = int_dev_fd;
	
	evl_printf("%s: success\n", __func__);
	return 0;
}

int pinGet(int pin)
{
	int retval, err;
	int val=-1;
	char str_val[2];
	retval = oob_read(pin, str_val, 2);
	if (retval < 0) {
		err = errno;
		perror("pinGet");
		evl_printf("%s errno %d retval %d couldn't read pin.\n", 
			   __func__, err, retval);
		return retval;
	}
	val = str_val[0] == '0' ? 0 : 1;
	return val;
}

void pinSet(int pin, int val)
{
	int retval;
	char str_val[2]={'0', '0'};
	str_val[0] = val ? '1' : '0';
	retval = oob_write(pin, str_val, 2);
	if(retval < 0){
		evl_printf("%s error:%d couldn't write pin: %s\n",
				   __func__, retval, strerror(errno));
	}
}

static void sigdebug_handler(int sig, siginfo_t *si, void *context)
{
    evl_sigdebug_handler(sig, si, context);
}

static void sigint_signal_handler(int unused) 
{
	int ret;
	evl_printf("%s\n", __func__);
	//pthread_setmode_np(PTHREAD_WARNSW, 0, NULL);
    pinSet(pinOutput, 1);
	if((ret = munmap(p, getpagesize()))) {
		evl_printf("munmap returns %d\n", ret);
	}    
	close(spi_fd);
	close(int_dev_fd);
	if(int_dev_fd != intack_dev_fd)
		close(intack_dev_fd);
	
	evl_detach_self();
	exit(0);
}

int spi_write_and_read(int size)
{
	int ret, err;
	if ((ret = oob_ioctl(spi_fd, SPI_RTIOC_TRANSFER_N, size)) != 0) {
		err = errno;
		perror("error SPI_RTIOC_TRANSFER_N");
		evl_printf("SPI_RTIOC_TRANSFER_N returns %d, errno=%d \n", ret, err);
		sigint_signal_handler(err);
	}
	return ret;
}

int main(int argc, char *argv[])
{
	int ret, n=EVL_N_TESTS;
	int i=0, j=0, interrupt=0;
	int int_ack_latency, max_int_ack_latency=0,
		spi_latency, max_spi_latency=0;
	struct sigaction sa;
    struct sched_param sp;
	long double mean=0;

	if(argc >= 2)
		n = atoi(argv[1]);
	evl_printf("using n=%d\n", n);
	evl_printf("Set signal handlers ...\n");
    //signal(SIGXCPU, sixcpu_signal_handler);
	/* setup oob to ib signal */
	sigemptyset(&sa.sa_mask);
	sa.sa_sigaction = sigdebug_handler;
	sa.sa_flags = SA_SIGINFO;
	sigaction(SIGDEBUG, &sa, NULL);
	signal(SIGINT, sigint_signal_handler);
	
	evl_printf("Setup this thread as a real time thread\n");
  	if((ret = mlockall(MCL_FUTURE|MCL_CURRENT)) < 0) {
       	evl_printf("mlockall failed: %s\n", strerror(ret));
       	return -1;
   	}
    sp.sched_priority = 80;
    if((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) != 0) {
        evl_printf("Failed to set stepper thread to real-time priority: %s\n", strerror(ret));
		return -1;
    }
	//pthread_setmode_np(0, PTHREAD_WARNSW, NULL);
    //sched_setscheduler(getpid(), SCHED_FIFO, &sp);
    //printf("SCHED_FIFO=%d, current_scheduler=%d\n", SCHED_FIFO, sched_getscheduler(0));
	/* Attach the current thread to the EVL core. */
	efd = evl_attach_self("lat-bbb-evl-%d", getpid());
	evl_printf("Thread %d in evl\n", efd);

	/* Open gpio pins */
	//if((ret = configure_pins("/dev/rtdm/gpio115_P9_27", "/dev/rtdm/gpio48_P9_15")) < 0)
	if((ret = configure_pins("/dev/rt_gpio_device", "none")) < 0)
		return ret;
	
	/* Open spi device */
    //ret = open_spi_device("/dev/rtdm/spi1/slave1.0");
	ret = open_spi_device("/dev/evl-spidev1.0");
    if (ret < 0) {
            evl_printf("%s: Error on spi device, exiting.\n", __FUNCTION__);
            return -1;
    }

	evl_printf("Set interrupt_ack <= 1...\n");
	mode = T_WOSS; /* warn on stage switch */
	ret = oob_ioctl(efd, EVL_THRIOC_SET_MODE, &mode);
	if(ret < 0) {
		evl_printf("EVL_THRIOC_SET_MODE error %d\n", ret);
		return ret;
	}
	//printf("\n ok \n");
	//write(1, "\n ok \n", strlen("\n ok \n"));
	pinSet(pinOutput, 1);
	mode = T_WOSS; /* warn on stage switch */
	ret = oob_ioctl(efd, EVL_THRIOC_CLEAR_MODE, &mode);
	if(ret < 0) {
		evl_printf("EVL_THRIOC_CLEAR_MODE error %d\n", ret);
		return ret;
	}
	
    evl_printf("Transfer one octet to spi in order to move latency.v in state 0 ...\n");
    tx_buffer[0] = 0xE6;
	spi_write_and_read(1);
	evl_printf("\t...done\n");

	do {
		debug_printf(("j=%d \n", j));
	        
	    debug_printf(("interrupt_ack <= 0, in order to exit state 0 ..."
					  " and then read interrupt\n"));
		pinSet(pinOutput, 0);
		
      	interrupt=0;
		do {
	        interrupt = pinGet(pinInputIRQ);
			if(!interrupt)
				debug_printf(("interrupt=0, read again\n"));
			else if(interrupt < 0) {
				evl_printf("interrupt = %d\n", interrupt);
				return -1;
				sigint_signal_handler(1);
			}
		} while (!interrupt);
       	debug_printf(("interrupt read value: %d\n", interrupt));
		
        if(interrupt) {
           	debug_printf(("ack interrupt ...\n"));
            pinSet(pinOutput, 1);
       	}

		// Do stuff 
		debug_printf(("Write from tx buffer...\n"));
		tx_buffer[0] = 0; tx_buffer[1] = 1; tx_buffer[2] = 2; tx_buffer[3] = 3;
		tx_buffer[4] = 0; tx_buffer[5] = 1; tx_buffer[6] = 2; tx_buffer[7] = 3;
		spi_write_and_read(8);

		for(i = 0; i < 8; i++)
			debug_printf(("rx_buffer[%d]=%02x ", i, rx_buffer[i]));
        debug_printf(("\n"));
		int_ack_latency = *(int*)rx_buffer;
		spi_latency = *(int*)&rx_buffer[4];
		debug_printf(("int_ack_latency=%x, spi_latency=%x \n", int_ack_latency, spi_latency));
		if(int_ack_latency > max_int_ack_latency)
			max_int_ack_latency = int_ack_latency;
		if(spi_latency > max_spi_latency)
			max_spi_latency = spi_latency;
		mean += int_ack_latency * ((double)1.0/n);
		int_lat[j] = int_ack_latency;
		spi_lat[j] = spi_latency;
		
		j++;
		/* feedback */
		if(((j % 10000) == 0) || (j == n) || (j == 1000))
			evl_printf("j=%d: max_int_ack_latency= %d => %.2fus (mean=%.2fus) max_spi_latency= %d => %.2fus \n", 
				j, max_int_ack_latency, ((double)max_int_ack_latency/FPGA_CLOCK_FREQ)*1E6, 
				(mean/FPGA_CLOCK_FREQ)*1E6,
				max_spi_latency, ((double)max_spi_latency/FPGA_CLOCK_FREQ)*1E6);
	} while(j < n);

	/* write latency data to file */
	f = fopen("latency.csv", "w");
	if(f != NULL) {
		for (j = 0; j < n; j++)
			fprintf(f, "%.2f, %.2f\n",
					int_lat[j]*((double)1E6/FPGA_CLOCK_FREQ), spi_lat[j]*((double)1E6/FPGA_CLOCK_FREQ));
		fclose(f);
	} else {
		printf("fopen error\n");
	}
	
	// Clean up
	sigint_signal_handler(0);
	
	evl_printf("Program done.\n");
	return 0;
}

