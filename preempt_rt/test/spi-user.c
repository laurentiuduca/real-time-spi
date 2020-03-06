 #include <stdint.h>
 #include <unistd.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <fcntl.h>
 #include <sys/ioctl.h>
 #include <linux/types.h>
 #include <linux/spi/spidev.h>

#define TRANSFER_SIZE 1

static int pabort(const char *s)
{
    perror(s);
    return -1;
}

static int spi_device_setup(int fd)
{
    int mode, speed, a, b;
    unsigned char i;
    int bits = 8;
    /*
     * spi mode: mode 0
     * Initial clock state low, and the first edge is rising.
     */
    mode = SPI_MODE_0;
    a = ioctl(fd, SPI_IOC_WR_MODE, &mode); /* write mode */
    b = ioctl(fd, SPI_IOC_RD_MODE, &mode); /* read mode */
    if ((a < 0) || (b < 0)) {
        return pabort("can't set spi mode");
    }
    /*
     * Clock max speed in Hz
     */
    speed = 50000; 
    a = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed); /* Write speed */
    b = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed); /* Read speed */
    if ((a < 0) || (b < 0)) {
        return pabort("fail to set max speed hz");
    }
    /*
     * setting SPI to MSB first.
     * Here, 0 means "not to use LSB first".
     * In order to use LSB first, argument should be > 0
     */
    i = 0;
    a = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &i);
    b = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &i);
    if ((a < 0) || (b < 0)) {
        return pabort("Fail to set LSB/MSB first\n");
    }
    /*
     * setting SPI to 8 bits per word
     */
    bits = 8;
    a = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    b = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    if ((a < 0) || (b < 0)) {
        return pabort("Fail to set bits per word\n");
    }
    return 0;
}

static void do_transfer(int fd)
{
    int i, ret;
    char txbuf[TRANSFER_SIZE] = {0, };
    char rxbuf[TRANSFER_SIZE] = {0, };
    //char cmd_buff = 0x61;
    struct spi_ioc_transfer tr[1] = {
        [0] = {
            .tx_buf = (unsigned long)txbuf,
            .rx_buf = (unsigned long)rxbuf,
            .len = TRANSFER_SIZE,
            .cs_change = 1, /* We need CS to change */
            .delay_usecs = 0, /* wait after this transfer */
            .bits_per_word = 8,
        },
#if 0
        [1] = {
            .tx_buf = (unsigned long)txbuf,
            .rx_buf = (unsigned long)rxbuf,
            .len = sizeof(txbuf),
            .bits_per_word = 8,
        },
#endif
    };
    for(i = 0; i < TRANSFER_SIZE; i++) {
	    rxbuf[i] = 0;
	    txbuf[i] = i+1;
    }
	// SPI_IOC_MESSAGE(1) means spi_ioc_transfer contains one transfer
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    printf("ioctl returns %d \n", ret);
    if (ret != TRANSFER_SIZE){
        perror("can't send spi message");
        exit(1);
    }
    for (i = 0; i < TRANSFER_SIZE; i++) {
        printf("i=%d rx=%.2X, ", i, rxbuf[i]);
	if((i % 10) == 0)
		printf("\n");
    }
    printf("\n");
#if 0
    for (i = 0; i < sizeof(txbuf); i++)
        printf("i=%d rx=%.2X \n", i, rxbuf[i]);
#endif
    
    printf("\n");
}

int main(int argc, char **argv)
{
    char *device = "/dev/spidev0.0";
    //char dest[16]="abcdefghabcdefgh";
    int fd;
    int error; //, i, n;
    
    if(argc >= 2) {
        printf("argc=%d argv[1]=%s \n", argc, argv[1]);
        device = argv[1];
    }
    printf("device=%s \n", device);
    fd = open(device, O_RDWR);
    if (fd < 0)
        return pabort("Can't open device ");
    error = spi_device_setup(fd);
    if (error) {
        printf("error spi_device_setup\n");
        exit (1);
    }
    do_transfer(fd);
/*
    n = strlen(device);
    printf("write: %s \n", device);
    write(fd, device, n);
    read(fd, dest, n);
    printf("read: '%s': ", dest);
    for(i = 0; i < n; i++) {
#if 0
	    if(device[i] != dest[i]) {
		    printf("device != dest\n");
		    break;
	    }
#endif
	printf("'%c'=%x ", dest[i], dest[i]);
    }
    printf("\n");
//    if(i == n)
//	    printf("device == dest\n");
*/    
    close(fd);
    return 0;
}

