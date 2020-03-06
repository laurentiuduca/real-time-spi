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
#include <linux/spi/spidev.h>
#include <pthread.h>

#define DEBUG_APPLICATION 0
#define debug_printf(pr_str) do { if (DEBUG_APPLICATION) printf pr_str; } while(0) 

#define FPGA_CLOCK_FREQ 50000000.00
#define LAT_SPI_BUFFER_SIZE	100
#define LAT_SPI_SPEED_HZ	(2*1000*1000)
#define LAT_N_TESTS	(1000*1000)

#define pinInputIRQ	int_dev_fd
#define pinOutput	intack_dev_fd

static unsigned char rx_buffer[LAT_SPI_BUFFER_SIZE], tx_buffer[LAT_SPI_BUFFER_SIZE];

/** Handle to the spi driver instance. */
int spi_fd = -1, int_dev_fd = -1, intack_dev_fd = -1;
int efd;
unsigned short int int_lat[LAT_N_TESTS], spi_lat[LAT_N_TESTS];
FILE *f;

/**
 * Open and configure spi device
 * @return 0 in case of success, a negative value otherwise.
 */
int open_spi_device(char* spi_dev_name) 
{
    int mode, speed, a, b;
    unsigned char i;
    int bits = 8;

	/* Open device */
	spi_fd = open(spi_dev_name, O_RDWR);
	if (spi_fd < 0) {
		printf("%s: Could not open spi device, open has failed with %d (%s).\n", 
			   __FUNCTION__, errno, strerror(errno));
		return -1;
	} else {
		printf("%s: Device %s opened.\n", __FUNCTION__, spi_dev_name);
	}
	
	/*
     * spi mode: mode 0
     * Initial clock state low, and the first edge is rising.
     */
    mode = SPI_MODE_0;
    a = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode); /* write mode */
    b = ioctl(spi_fd, SPI_IOC_RD_MODE, &mode); /* read mode */
    if ((a < 0) || (b < 0)) {
	    perror("can't set spi mode");
    	return -1;
    }
    /*
     * Clock max speed in Hz
     */
    speed = LAT_SPI_SPEED_HZ; 
    a = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); /* Write speed */
    b = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed); /* Read speed */
    if ((a < 0) || (b < 0)) {
		perror("fail to set max speed hz");
		return -1;
    }
    /*
     * setting SPI to MSB first.
     * Here, 0 means "not to use LSB first".
     * In order to use LSB first, argument should be > 0
     */
    i = 0;
    a = ioctl(spi_fd, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(spi_fd, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0)) {
        perror("Fail to set LSB/MSB first\n");
		return -1;
	}
    /*
     * setting SPI to 8 bits per word
     */
    bits = 8;
    a = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    b = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if ((a < 0) || (b < 0)) {
        perror("Fail to set bits per word\n");
		return -1;
    }
	
	printf("%s: Device successfully configured.\n", __FUNCTION__);
    return 0;
}

int configure_pins(char *int_dev_name, char *intack_dev_name)
{
	int ret;

	/* Open device for interrupt input */
	printf("%s O_RDWR\n", int_dev_name);
	ret = open(int_dev_name, O_RDWR);
	if (ret < 0) {
		printf("%s: Could not open device %s for interrupt input, open has failed with %d (%s).\n", 
			   __FUNCTION__, int_dev_name, errno, strerror(errno));
		return -1;
	} else {
		printf("%s: Device for interrupt input opened.\n", __FUNCTION__);
		int_dev_fd = ret;
	}

	intack_dev_fd = int_dev_fd;
	
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
		printf("%s error:%d couldn't write pin: %s\n",
				   __func__, retval, strerror(errno));
	}
}

static void sigint_signal_handler(int unused) 
{
	int ret;
	printf("%s\n", __func__);
	//pthread_setmode_np(PTHREAD_WARNSW, 0, NULL);
    pinSet(pinOutput, 1);
    
	close(spi_fd);
	close(int_dev_fd);
	if(int_dev_fd != intack_dev_fd)
		close(intack_dev_fd);
	exit(0);
}

int spi_write_and_read(int size)
{
    int ret;
    struct spi_ioc_transfer tr[1] = {
        [0] = {
            .tx_buf = (unsigned long)tx_buffer,
            .rx_buf = (unsigned long)rx_buffer,
            .len = size,
            .cs_change = 1, /* We need CS to change */
            .delay_usecs = 0, /* wait after this transfer */
            .bits_per_word = 8,
        },
    };
	// SPI_IOC_MESSAGE(1) means spi_ioc_transfer contains one transfer
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret != size){
        perror("can't send spi message");
        sigint_signal_handler(1);
    }
	return ret;
}

int main(int argc, char *argv[])
{
	int ret, n=LAT_N_TESTS;
	int i=0, j=0, interrupt=0;
	int int_ack_latency, max_int_ack_latency=0,
		spi_latency, max_spi_latency=0;
	struct sigaction sa;
    struct sched_param sp;
	long double mean=0;

	if(argc >= 2)
		n = atoi(argv[1]);
	if(n > LAT_N_TESTS) {
		printf("n > LAT_N_TESTS\n");
		return -1;
	}
	printf("using n=%d\n", n);

	printf("Set signal handlers ...\n");
	signal(SIGINT, sigint_signal_handler);

	printf("Setup this thread as a real time thread\n");
  	if((ret = mlockall(MCL_FUTURE|MCL_CURRENT)) < 0) {
       	printf("mlockall failed: %s\n", strerror(ret));
       	return -1;
   	}
    sp.sched_priority = 98;
    if((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) != 0) {
        printf("Failed to set lat thread to real-time priority: %s\n", strerror(ret));
		return -1;
    }
    sched_setscheduler(getpid(), SCHED_FIFO, &sp);
    printf("SCHED_FIFO=%d, current_scheduler=%d\n", SCHED_FIFO, sched_getscheduler(0));
	
	/* Open gpio pins */
	//if((ret = configure_pins("/dev/rtdm/gpio115_P9_27", "/dev/rtdm/gpio48_P9_15")) < 0)
	if((ret = configure_pins("/dev/rt_gpio_device", "none")) < 0)
		return ret;
	
	/* Open spi device */
    //ret = open_spi_device("/dev/rtdm/spi1/slave1.0");
	ret = open_spi_device("/dev/spidev1.0");
    if (ret < 0) {
            printf("%s: Error on spi device, exiting.\n", __FUNCTION__);
            return -1;
    }

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
			printf("j=%d: max_int_ack_latency= %d => %.2fus (mean=%.2fus) max_spi_latency= %d => %.2fus \n", 
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
	
	printf("Program done.\n");
	return 0;
}

