#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  agv-driver"
echo "sudo rm   /etc/udev/rules.d/agv-driver.rules"
sudo rm   /etc/udev/rules.d/agv-driver.rules
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish  delete"
