/*
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *  
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
#include <unistd.h>
#include <fcntl.h>
#include <smokey/smokey.h>
#include <linux/spi/spidev.h>
#include <rtdm/spi.h>

#define DEFAULT_DEV_NAME "/dev/rtdm/spi0/slave0.0"

#define MAX_SIZE 360
#define N_TESTS	 50

static unsigned char *i_area, *o_area;
static unsigned char r_area[MAX_SIZE], w_area[MAX_SIZE];

static int do_spi_loop(int fd, int transfer_size, int val)
{
	int i, ret, err;
	int pass;

	for(i = 0; i < transfer_size; i++) {
		i_area[i] = 0;
		o_area[i] = val;
		r_area[i] = 0;
		w_area[i] = val;
	}

	if ((ret = ioctl(fd, SPI_RTIOC_TRANSFER_N, transfer_size)) != 0) {
		err = errno;
		perror("error SPI_RTIOC_TRANSFER_N");
		printf("SPI_RTIOC_TRANSFER_N returns %d, errno=%d \n", ret, err);
	}
	
	printf("SPI_RTIOC_TRANSFER_N: transfer_size=%d val=%d\n", transfer_size, val);
	pass = 1;
	for(i = 0; i < transfer_size; i++) {
		if(i_area[i] != o_area[i])
			pass = 0;
#if 0
		printf("%x, %x; ", 
			   o_area[i], i_area[i]);
		if(i % 16 == 0)
			printf("\n");
#endif
	}
	printf("\n\tpass=%d\n", pass);

#if 0
	printf("writing ... \n");
	if((ret = write(fd, w_area, transfer_size)) != transfer_size) {
		printf("write returns %d \n", ret);
		//return ret;
	}
	
	printf("write: \n");
	for(i = 0; i < transfer_size; i++) {
		printf("%x, %x; ", 
			   w_area[i], r_area[i]);
		if(i % 16 == 0)
			printf("\n");
	}
	printf("\n");

	printf("reading ... \n");
	if((ret = read (fd, r_area, transfer_size)) != transfer_size) {
		printf("read returns %d \n", ret);
		//return ret;
	}

	
	printf("read: \n");
	for(i = 0; i < transfer_size; i++) {
		printf("%x, %x; ", 
			   w_area[i], r_area[i]);
		if(i % 16 == 0)
			printf("\n");
	}
	printf("\n");
#endif	
	return 0;
}

static int run_spi_transfer(char * str)
{
	int fd, ret, speed_hz = 5000000;
	struct rtdm_spi_config config;
	struct rtdm_spi_iobufs iobufs;
	const char *device = str;
	void *p;
	int transfer_size;
	int i;
	
	fd = open(device, O_RDWR);
	if (fd < 0) {
		ret = -errno;
		printf("cannot open device %s \n", device);
		return ret;
	}

		transfer_size = MAX_SIZE;
		iobufs.io_len = transfer_size;
		if ((ret = ioctl(fd, SPI_RTIOC_SET_IOBUFS, &iobufs))) {
			printf("SPI_RTIOC_SET_IOBUFS returns %d \n", ret);
			return ret;
		}

		p = mmap(NULL, iobufs.map_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
		if (!__Fassert(p == MAP_FAILED))
			return -EINVAL;

		printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u \n",
				 iobufs.i_offset, iobufs.i_offset + transfer_size - 1,
				 iobufs.o_offset, iobufs.o_offset + transfer_size - 1,
				 iobufs.map_len);

		i_area = p + iobufs.i_offset;
		o_area = p + iobufs.o_offset;

		config.mode = SPI_MODE_0;
		config.bits_per_word = 8;
		config.speed_hz = speed_hz;
		if ((ret = ioctl(fd, SPI_RTIOC_SET_CONFIG, &config))) {
			printf("SPI_RTIOC_SET_CONFIG error: %d \n", ret);
			return ret;
		}

		if ((ret = ioctl(fd, SPI_RTIOC_GET_CONFIG, &config))) {
			printf("SPI_RTIOC_GET_CONFIG error: %d \n", ret);
			return ret;
		}

		printf("speed=%u hz, mode=%#x, bits=%u \n",
				 config.speed_hz, config.mode, config.bits_per_word);

		for(i = 0; i < N_TESTS; i++)
			do_spi_loop(fd, transfer_size, i);

		// cleaning up
		if((ret = munmap(p, iobufs.map_len))) {
			printf("munmap returns %d\n", ret);
			return ret;
		}
	//}
	ret = close(fd);
	return ret;
}

int main(int argc, char *const argv[])
{
	int ret;
	
	if(argc < 2) {
		printf("using default dev name: %s \n", DEFAULT_DEV_NAME);
		ret = run_spi_transfer(DEFAULT_DEV_NAME);
	} else {
		printf("using dev name: %s \n", argv[1]);
		ret = run_spi_transfer(argv[1]);
	} 
	printf("run_spi_transfer returns %d\n", ret);
	
	return 0;
}
