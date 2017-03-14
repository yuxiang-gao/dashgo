#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  aiui"
echo "AIUI usb connection as /dev/aiui , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy aiui.rules to  /etc/udev/rules.d/"
#echo "`rospack find rplidar_ros`/scripts/aiui.rules"
sudo cp ./aiui.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
