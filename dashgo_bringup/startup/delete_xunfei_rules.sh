#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  aiui"
echo "sudo rm   /etc/udev/rules.d/aiui.rules"
sudo rm   /etc/udev/rules.d/aiui.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
