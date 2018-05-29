#!/bin/bash

echo "Loading Trollbot udev rules"
sudo cp `rospack find trollbot`/scripts/trollbot.rules  /etc/udev/rules.d
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "Completed"
