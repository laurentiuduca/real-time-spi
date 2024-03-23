
ccflags-$(CONFIG_EVL_DRIVERS_SPI_DEBUG) := -DDEBUG

obj-$(CONFIG_EVL_DRIVERS_SPI) += evl_spi.o

evl_spi-y := spi-master.o spi-device.o

obj-$(CONFIG_EVL_DRIVERS_SPI_BCM2835) += evl_spi_bcm2835.o
obj-$(CONFIG_EVL_DRIVERS_SPI_SUN6I) += evl_spi_sun6i.o
obj-$(CONFIG_EVL_DRIVERS_SPI_OMAP2_MCSPI_RT) += evl_spi_omap2_mcspi_rt.o

evl_spi_bcm2835-y := spi-bcm2835.o
evl_spi_sun6i-y := spi-sun6i.o
evl_spi_omap2_mcspi_rt-y := spi-omap2-mcspi-rt.o
