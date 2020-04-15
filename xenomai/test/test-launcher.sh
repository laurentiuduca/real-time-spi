stress-ng --cpu 1 --cache 2 --io 2 --vm 2 --vm-bytes 128M --aggressive --maximize --timeout 200m &
#cat /dev/zero > /dev/null &
sleep 1
./lat-spi-xenomai.out
#/usr/bin/cyclictest --smp -p98 -m -i 100000

