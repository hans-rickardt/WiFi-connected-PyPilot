
# Project Title

A brief description of what this project does and who it's for

WiFi connected PyPilot Servo and WiFi Remote
## Authors

https://github.com/hans-rickardt


## Background


 I have sinse many years used a Raspberry Pi with 7” Display running OpenCPN + Signal-K, connected to my old NMEA0183 equipment getting GPS, Wind, Log and depth. But it also act as an WiFI access point with 4G antenna mounted in the mast 18 meters up in the mast on my sailing boat.

Three years ago I upgraded instrument to Raymarine Axion 9” and CAN-Bus connected equipment. Upgraded Raspberry Pi with CAN-Bus and AIS receiver and IMU compass MPU-925x that is i2c connected.

https://www.electrokit.com/produkt/pican2-can-bus-board-for-raspberry-pi-2-3/
https://shop.wegmatt.com/products/daisy-hat-ais-receiver



I was inspired by a very well documented OpenPlotter project that hade the same functionality as my own but even more cool integrations like PyPilot. Decided to upgrade to OpenPlotter. 
https://openplotter.readthedocs.io/en/3.x.x/

I convert my autopilot old Autohelm 3000 to PyPilot and keep the motor.
Found a nice project but it is a lot of cables and I have WiFi already.   
https://open-boat-projects.org/en/pypilot/
## WiFi PyPilot Servo

My plan for this project is to to do and WiFi connected PyPilot 

Motor controller is based on but have removed the display that I don’t need.
https://github.com/McNugget6750/pypilot/tree/master/arduino/motor

To keep it simple I decided to use the same Adruino Nano and the serial port RX, TX conneced to an WiFI capable controller ESP-01S using Ardruino “exampleWiFiTelnetToSerial.ino”.
Need only a few resistors for interface between 5V to 3.3V for RX, TX signals levels, using Speed 4800Baud.
The flow is ESP-01S connect to WiFi and listening on Port 23 ( Telnet ), when a client connect it will transfer all data to Ardruino Nano as is was USB connected.
 To get PyPilot detect my WiFi connected SerialPort I use a Linix/Unix tool “socat” that can act as Multipurpose relay. A start script in systemd start my “socat.sh” script that connect to ESP-01S, when it is connected it a virtual tty “/dev/ttySRV0” that is a link to a “/dev/pts/?”.
Then you configure PyPilot to use “/dev/ttySRV0” .
Other component needed 5V and 3.3V regulator.


https://store.arduino.cc/products/arduino-nano
https://www.amazon.se/AZDelivery-WiFi-Modul-kompatibel-Raspberry-inklusive/dp/B01LK83TX0

https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiTelnetToSerial/WiFiTelnetToSerial.ino

https://raspberry.redbrain.me/Scripte/Tutorial_socat.html

https://github.com/hans-rickardt/WiFi-connected-PyPilot/blob/main/images/IMG_3077.jpeg

## Screenshots

![App Screenshot](https://github.com/hans-rickardt/WiFi-connected-PyPilot/blob/main/images/IMG_3077.jpeg)



## WiFi PyPilot Remote

Next task was to create an WiFi connected remote based on WEMOS LOLIN D32 that have support for WiFi and battery, and 2.8” LCD with touch panel from Olimex.
Was able to debug the API using ”pypilot_client” and ”tcpdump”. 
It is build with ”Arduino IDE 2.0.4”, using mainly standard libarery’s. 

## Usage
```
pilot3_4.ino:
This is the main start up module.
The D32 is a dual core SOC therefore I divided the task in to “WiFI + API” and display + touch screen.  
```

```
WiFi_remote.cpp:
This module start WiFi and connecting to ”Pypilot-server” at port 23322, send parameters to get several event from Pypilot-server like ”ap.heading”. Listens on event and will analyse and update messages buffers to the display task.
```

```
Graf.cpp:
The module is main display task, scans for event from keyboard and messages buffers that wifi_remote.cpp create
Other:   
Was not able to get a stable touch screen using the standard library therefore included STMPE610 driver from Olimex. Added two fonts

```

```
Function:
The screen is divided 6 keypad where the top mid is setup where you can select compass, gps, true wind and wind , switch to Tack mode

```

https://www.wemos.cc/en/latest/d32/d32.html
 https://www.olimex.com/Products/Modules/LCD/MOD-LCD2-8RTP/open-source-hardware
 
## Screenshots

![App Screenshot](https://github.com/hans-rickardt/WiFi-connected-PyPilot/blob/main/images/box1.jpg)
![App Screenshot](https://github.com/hans-rickardt/WiFi-connected-PyPilot/blob/main/images/box2.png)
![App Screenshot](https://github.com/hans-rickardt/WiFi-connected-PyPilot/blob/main/images/box3.png)

