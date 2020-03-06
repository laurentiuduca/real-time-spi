export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.:/usr/xenomai/lib
./bbio_test -t input -i /dev/rtdm/gpio60_P9_12 -o /dev/rtdm/gpio48_P9_15

