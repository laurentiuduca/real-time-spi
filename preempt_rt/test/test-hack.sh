set -x
while :; sleep 1; do hackbench -g 40 -l 30000 >/dev/null 2>&1; done&
dd if=/dev/zero of=/dev/null bs=32M &
./lat-preempt_rt.out

