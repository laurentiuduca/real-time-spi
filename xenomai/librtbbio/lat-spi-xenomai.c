// Author: Laurentiu-Cristian Duca
// License: GNU GPL

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <rtdm/gpio.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <error.h>
#include <semaphore.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <rtdm/rtdm.h>
#include <rtdm/spi.h>
#include <smokey/smokey.h>

#include "bbgpio.h"

#define DEBUG_APPLICATION 0
#define debug_printf(pr_str) do { if (DEBUG_APPLICATION) printf pr_str; } while(0) 

#define FPGA_CLOCK_FREQ 50000000.00
#define XENOLAT_SPI_BUFFER_SIZE	100
#define XENOLAT_SPI_SPEED_HZ	(2*1000*1000)
#define XENOLAT_N_TESTS	(1000*1000)

#define pinInputIRQ	int_dev_fd
#define pinOutput	intack_dev_fd

static unsigned char *rx_buffer, *tx_buffer;
// mmap memory area
void *p;
struct rtdm_spi_iobufs iobufs;

/** Handle to the rtdm spi driver instance. */
static int spi_fd = -1, int_dev_fd = -1, intack_dev_fd = -1;

unsigned short int int_lat[XENOLAT_N_TESTS], spi_lat[XENOLAT_N_TESTS];
FILE *f;

/**
 * Open and configure spi device
 * @return 0 in case of success, a negative value otherwise.
 */
int open_spi_device(char* spi_dev_name) 
{
	int ret;
	struct rtdm_spi_config config;

	/* Open device */
	spi_fd = open(spi_dev_name, O_RDWR);
	if (spi_fd < 0) {
		printf("%s: Could not open spi device, open has failed with %d (%s).\n", 
			   __FUNCTION__, errno, strerror(errno));
		return -1;
	} else {
		printf("%s: Device opened.\n", __FUNCTION__);
	}

	/* Configure device */
	iobufs.io_len = XENOLAT_SPI_BUFFER_SIZE;
	if ((ret = ioctl(spi_fd, SPI_RTIOC_SET_IOBUFS, &iobufs))) {
		perror("SPI_RTIOC_SET_IOBUFS");
		printf("SPI_RTIOC_SET_IOBUFS returns %d \n", ret);
		return ret;
	}

	p = mmap(NULL, iobufs.map_len, PROT_READ|PROT_WRITE, MAP_SHARED, spi_fd, 0);
	if (p == MAP_FAILED)
		return -EINVAL;

	printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u \n",
			 iobufs.i_offset, iobufs.i_offset + XENOLAT_SPI_BUFFER_SIZE - 1,
			 iobufs.o_offset, iobufs.o_offset + XENOLAT_SPI_BUFFER_SIZE - 1,
			 iobufs.map_len);

	rx_buffer = p + iobufs.i_offset;
	tx_buffer = p + iobufs.o_offset;

	config.mode = SPI_MODE_0;
	config.bits_per_word = 8;
	config.speed_hz = XENOLAT_SPI_SPEED_HZ;
	if ((ret = ioctl(spi_fd, SPI_RTIOC_SET_CONFIG, &config))) {
		printf("SPI_RTIOC_SET_CONFIG error: %d \n", ret);
		return ret;
	}

	if ((ret = ioctl(spi_fd, SPI_RTIOC_GET_CONFIG, &config))) {
		printf("SPI_RTIOC_GET_CONFIG error: %d \n", ret);
		return ret;
	}

	printf("speed=%u hz, mode=%#x, bits=%u \n",
			 config.speed_hz, config.mode, config.bits_per_word);		

	printf("%s: Device successfully configured.\n", __FUNCTION__);
	return 0;
}

int configure_pins(char *int_dev_name, char *intack_dev_name)
{
	int ret;

	/* Open device for interrupt input */
	printf("BBGPIO_DIR_IRQ\n");
	ret = open(int_dev_name, BBGPIO_DIR_IRQ);
	if (ret < 0) {
		printf("%s: Could not open device %s for interrupt input, open has failed with %d (%s).\n", 
			   __FUNCTION__, int_dev_name, errno, strerror(errno));
		return -1;
	} else {
		printf("%s: Device for interrupt input opened.\n", __FUNCTION__);
		int_dev_fd = ret;
	}

	if ((ret = ioctl(int_dev_fd, BBGPIO_IOCTL_EDGE_SET, BBGPIO_IOCTL_EDGE_RISING)) != 0) {
		printf("%s: error %d, %s\n", __func__, errno, strerror(errno));
		return -1;
	}
	
	// timeout 0 for infinite
	if ((ret = ioctl(int_dev_fd, BBGPIO_IOCTL_TIMEOUT_SET, 0)) != 0) {
		perror("error BBGPIO_IOCTL_TIMEOUT_SET");
		return -1;
	}
	
	/* Open device for interrupt ack */
	ret = open(intack_dev_name, BBGPIO_DIR_OUT);
	if (ret < 0) {
		printf("%s: Could not open device for interrupt ack, open has failed with %d (%s).\n", 
			   __FUNCTION__, errno, strerror(errno));
		return -1;
	} else {
		printf("%s: Device for interrupt ack opened.\n", __FUNCTION__);
		intack_dev_fd = ret;
	}
	
	printf("%s: success\n", __func__);
	return 0;
}

int pinGet(int pin)
{
	int retval, err;
	int val=-1;
	char str_val[2];
	retval = read(pin, str_val, 2);
	if (retval < 0) {
		err = errno;
		perror("pinGet");
		printf("%s errno %d retval %d couldn't read pin.\n", 
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
	retval = write(pin, str_val, 2);
	if(retval < 0){
		printf("%s error:%d couldn't write pin.\n", __func__, retval);
	}
}

static void sixcpu_signal_handler(int unused) 
{
	printf("received SIGXCPU\n");
}

static void sigint_signal_handler(int unused) 
{
	int ret;
	printf("%s\n", __func__);
	pthread_setmode_np(PTHREAD_WARNSW, 0, NULL);
    pinSet(pinOutput, 1);
	if((ret = munmap(p, iobufs.map_len))) {
		printf("munmap returns %d\n", ret);
	}    
	close(spi_fd);
	close(int_dev_fd);
	if(int_dev_fd != intack_dev_fd)
		close(intack_dev_fd);
	exit(0);
}

int spi_write_and_read(int size)
{
	int ret, err;
	if ((ret = ioctl(spi_fd, SPI_RTIOC_TRANSFER_N, size)) != 0) {
		err = errno;
		perror("error SPI_RTIOC_TRANSFER_N");
		printf("SPI_RTIOC_TRANSFER_N returns %d, errno=%d \n", ret, err);
		sigint_signal_handler(err);
	}
	return ret;
}

int main() 
{
	int ret, n=XENOLAT_N_TESTS;
	int i=0, j=0, interrupt=0;
	int int_ack_latency, max_int_ack_latency=0,
		spi_latency, max_spi_latency=0;
	long double mean=0;
	
	/* Open gpio pins */
	if((ret = configure_pins("/dev/rtdm/gpio115_P9_27", "/dev/rtdm/gpio48_P9_15")) < 0)
	//if((ret = configure_pins("/dev/rt_gpio_device", "none")) < 0)
		return ret;
	
	/* Open spi device */
    ret = open_spi_device("/dev/rtdm/spi1/slave1.0");
    if (ret < 0) {
            printf("%s: Could not open spi device, exiting.\n", __FUNCTION__);
            return -1;
    }

	printf("Set signal handlers ...\n");
    signal(SIGXCPU, sixcpu_signal_handler);
	signal(SIGINT, sigint_signal_handler);
	printf("Setup this thread as a real time thread\n");
  	if((ret = mlockall(MCL_FUTURE|MCL_CURRENT)) < 0) {
       	printf("mlockall failed: %s\n", strerror(ret));
       	return -1;
   	}
    struct sched_param sp;
    sp.sched_priority = 80;
    if((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) != 0) {
        printf("Failed to set stepper thread to real-time priority: %s\n", strerror(ret));
		return -1;
    }
	pthread_setmode_np(0, PTHREAD_WARNSW, NULL);

	printf("Set interrupt_ack <= 1...\n");
	pinSet(pinOutput, 1);

    printf("Transfer one octet to spi in order to move latency.v in state 0 ...\n");
    tx_buffer[0] = 0xE6;
	spi_write_and_read(1);
	printf("\t...done\n");

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
				printf("interrupt = %d\n", interrupt);
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
			printf("j=%d: max_int_ack_latency= %d => %.2fus (mean=%.2Lfus) max_spi_latency= %d => %.2fus \n", 
				j, max_int_ack_latency, ((double)max_int_ack_latency/FPGA_CLOCK_FREQ)*1E6, 
				(mean/FPGA_CLOCK_FREQ)*1E6,
				max_spi_latency, ((double)max_spi_latency/FPGA_CLOCK_FREQ)*1E6);
	} while(j < n);

	pthread_setmode_np(PTHREAD_WARNSW, 0, NULL);
	
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
	
	printf("Program done.\n");
	return 0;
}

