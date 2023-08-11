#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  agv-driver"
echo "ldlidar usb connection as /dev/agv-driver , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy agv-driver.rules to  /etc/udev/rules.d/"
sudo cp ./agv-driver.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "
