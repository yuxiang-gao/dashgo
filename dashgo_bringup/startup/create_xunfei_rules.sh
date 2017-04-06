#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  xunfei"
echo "xunfei usb connection as /dev/xunfei , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy xunfei.rules to  /etc/udev/rules.d/"
#echo "`rospack find rplidar_ros`/scripts/xunfei.rules"
sudo cp ./xunfei.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish "
