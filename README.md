# Esp32SSControl
This includes the real-time code to control servos using a Polulo Maestro and serial input from an outside source containing coordinates of the target.
The Polulo Maestro must be used and set to 9600 buad rate. https://www.pololu.com/blog/456/maestro-servo-controller-arduino-library
The Arduino IDE must also be set up to be able to upload the code to the ESP32. The ESP32 is great for this, running with two processors at a higher clock frequency than an Arduino.
It also includes bluetooth and WiFi which can be useful for many applications.
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/ has more information about how to set the ESP32 up on the arduino IDE.
Sometimes the ESP32 requires that you hold the boot button in order for the program to upload.

This was used for the control of a laser turret and so has a a couple interrupts that control the laser as well as whether the entire system is on.
If these are not set high or low then a floating error will cause the program not to work. 
