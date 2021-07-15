#!/bin/bash

sec=1
cnt=0
name=armorDetector
root=$(pwd)
Thread=`ps -ef | grep $name | grep -v "grep"`
cd root/build
make clean && make -j8
while [ 1 ]
do
count=`ps -ef | grep $name | grep -v "grep" | wc -l`
echo "Thread count: $count"
echo "Expection count: $cnt"
if [ $count -gt 1 ]; then
	echo "The $name is still alive!"
	sleep $sec
else 
	echo "Starting $name..."
	sudo chmod 666 /dev/ttyUSB0
	sudo chmod 666 /dev/ttyUSB1
    	cd root/build
    	gnome-terminal -x bash -c "./$name;exec bash;"
    echo "$name has started!"		
	sleep $sec
	((cnt=cnt+1))
	if [ $cnt -gt 9 ]; then
		reboot
	fi
fi
done
