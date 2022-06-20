#!/bin/bash

echo "Aerate basement."

echo "$(date "+%d.%m.%Y %H:%M"): Manually open windows to aerate basement." > /dev/shm/wincon.log

# check if there is already a instance of this script running
script_name=$(basename -- "$0")

if pidof -x "$script_name" -o $$ > /dev/null;
then
	echo "There is already a instance running to aerate the basement."
	exit 1
fi

# stop wincon daemon
systemctl stop wincon

# open basement windows
for i in {13,6,5};
do
	echo 0 > /sys/class/gpio/gpio${i}/value;
	sleep 15;
done

sleep 300 # wait 5 minutes
#sleep 1 # debug
#for i in {13,6,5};
#do
#	echo 1 > /sys/class/gpio/gpio${i}/value;
#	sleep 1;
#done

# restart wincon daemon
systemctl start wincon
