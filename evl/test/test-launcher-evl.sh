stress-ng --cpu 1 --cache 2 --io 2 --vm 2 --vm-bytes 128M --aggressive --maximize --timeout 200m &
# cat /dev/zero > /dev/null &
# on the host run:
# sudo ping -i 0.01 board-IP
sleep 1
./lat-bbb-evl.out

