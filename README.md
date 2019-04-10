# Esp32SSControl Readme for the Servo Control section of the Project
This includes the real-time code to control servos using a Polulo Maestro and serial input from an outside source containing coordinates of the target. This was used as the control of a system that had both a laser and camera connected to the system, turned by the servos to aim at a target. The coordinates of the target were found using oject detection software in the Tensorflow library. The goal was to turn the servos so the target was on the center of the screen. Once it was, an interrupt was set high so the laser turned on. That software was run on a raspberry pi, though as long as the serial data is sent the same, any hardware can work that can run object detection software i.e. Nvidia Jetson, or personal laptop. 

The Polulo Maestro must be used and set to 9600 buad rate. https://www.pololu.com/blog/456/maestro-servo-controller-arduino-library
The Arduino IDE must also be set up to be able to upload the code to the ESP32. The ESP32 is great for this, running with two processors at a higher clock frequency than an Arduino.
It also includes bluetooth and WiFi which can be useful for many applications.
https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/ has more information about how to set the ESP32 up on the arduino IDE.
Sometimes the ESP32 requires that you hold the boot button in order for the program to upload. 
In my case I used this ESP32 https://www.amazon.com/HiLetgo-ESP-WROOM-32-Development-Microcontroller-Integrated/dp/B0718T232Z and the board type I selected on Ardunio IDE is the Node32s. 

This was used for the control of a laser turret and so has a couple interrupts that control the laser as well as whether the entire system is on. If these are not set high or low then a floating error will cause the program not to work. I used a npn transistor to short the gnd of the laser to the gnd of the system which turns the laser on when the gate is high.
 

In order to control the two servos using the Polulo Maestro using this code, the TX pin on the ESP32 should be connected to the RX pin on the Maestro, the gnds connected, and the vin connected to either the Servo battery or the 5V source on the ESP32. 
There are then ports for the source and gnd for the servos that must be connected to a 9V battery. 

I used JSON to send and recieve data, though if I had more time I would perfect sending data more quickly without it. You must download the arduino json library most commonly used. I was recieving commands from a raspberry pi that also had the JSON library downloaded so that the transfer was pretty simple.


Math:
The coordinates I recieve in include the x and y coordinates of the target, the center x and y of the camera screen, and the total angle of the camera capturing the location of the target. Here is the math included to find how far the servos should turn:
-First we must find the total points per angle:
  (centerx * 2) / totalanglex = points per angle x.
  ex: (300 * 2) = pixels per screen in the x direction 
  600 points / 90 degrees = 6.667 points per angle or points/angle
  
-Second we must divide the difference in location relative to the center of the screen by the points per angle to get the angle change.
  (targetx - centerx) / points per angle x = change in angle.
  ex: (265 - 300) / 6.667 = -5.25 degrees
  
-Then we need to know what number to send to the Polulo. The range of the values that can be sent to the polulo is between 2000 and 10000, 2000 being the angle zero and 10000 being the angle 270. 
  (10000 - 2000) / 270 = 29.62963 which is the val per angle.
  
-Lastly we multiply the val per angle by the change in angle to get change in val to send to the Polulo. 
In this case, I keep track of the current location (angles) of the servos and add the change in val to the current location. 
This then moves the servos to the object found on the camera.
  ex: -5.25 * 29.6296 = -155.55 + current x => -155.55 + 6000 = 5844.45 = new 1/4th microsecond signal sent to the Polulo. This uses pulse width modulation which is why the value is using microseconds.
  
Using a different camera requires the values of center x and y (which can be used to find the total x and y) to be sent along with the angles of the camera. All of this is in the code provided.
NOTE: If the angle is unknown, a small single may take the servos longer to bring the object to the center of the camera, but a larger angle will cause the system to become unstable until the object is no longer on the screen. 



