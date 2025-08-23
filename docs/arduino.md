<h2>Arduino IDE</h2>

Download IDE from https://www.arduino.cc/en/software

Under File > Preferences, set Additional boards manager URLs to https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json,https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json

Under Tools > Manage Libraries, add these libraries:
 * ESP32Servo by Kevin Harrington

Under Tools > Board > Board Manager, add these boards
* esp32_bluepad32 by Richard Quesada

Under Tools > Board, select esp32_bluepad32 > ESP 32 Dev Module

Connect ESP32 Development Kit with USB cable
Under Tools > Port, select COM port used

Open MiniFork_Bluepad3.0.ino file and click right arrow on top tool bar to download code to ESP32

<h2>Controls</h2>

* Left joystick y-axis controls speed and x-axis controls steering
* Right joystick y-axis controls raising/lowering mast carriage
* DPAD up/down control mast tilt
* "A" button controls lights on/off
* "B" button executes "wiggle" function to move motors and flash lights (if you have more than 1 vehicle, this makes it easy to identify which
one is paired with this controller.
* L1 and R1 bumper buttons adjust the steering trim "center" point.

<h2>Rate Limited Steering</h2>
Steering is dampered (rate limited).  Continuing to hold in one direction will apply greater turning.  It will also not slam back to centered when
releasing joystick, but will slowly move back to center.  Right and left wheels move at different rate to give better turning performance.  All
of these combined will allow the forklift to make very sharp turns at low speed, but more gradual turns at higher speeds.
