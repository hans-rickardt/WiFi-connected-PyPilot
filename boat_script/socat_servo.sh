#!/bin/bash
HOST=pypilot_servo.
PORT=23

sleep 30

sudo home/pi/boat_script/wifi_2.5.sh
while true
do
#sudo chmod 777 /dev
sleep 30
#/usr/bin/socat pty,link=/dev/ttySRV0,raw,echo=0,ignoreeof=0,nonblock=1 tcp:pypilot_servo:23  
#/usr/bin/socat pty,link=/dev/ttySRV0,raw,echo=0, tcp:pypilot_servo:23  
sudo /usr/bin/socat pty,link=/dev/ttySRV0,raw,echo=0,mode=666 tcp:$HOST:$PORT  
sleep 10
sudo chmod 666 /dev/ttySRV0
done
