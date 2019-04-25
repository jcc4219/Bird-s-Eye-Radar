## mission_basic_sensor1.py
This program is supposed to perform an autonomous mission for the drone to fly and search for trash. 

**Required Python packages**
 * numpy: For the array calculations.
 * serial: To read the serial data from the radar.
 * time: To wait until more data is generated.
 * array: For arrays
 * Picamera: To take images using picamera
 * RPi.GPIO: For raspberry pi pins to read from sensor. 
 * pymavlink and mavutil: To communicate between raspberry pi and pixhawk
 * math: To do mathimatical calculations
 * dronekit: To import functions to help with mission planning in code
 
**HOW TO USE**
* Download the required packages.
* Make all neccesary connections between raspberry pi, pixhawk 2, mmW radar, and ultrasonic sensor. Connect the usb connection from pi to mmW radar. Connect tx/rx gpio pins from pi to telemetry port of pixhawk 2.
* Plug battery in to power all components.
* Make sure pi is connected to same wifi internet as companion computer such as a laptop.
* Connect to pi with laptop through ssh
* Run the code

## mmW_Trash_Detection.py
This program is a stand-alone version of the mmW sensor portion of "mission_basic_sensor1.py". It allows the user to run only the sensor without starting a mission on the drone.

**Required Python packages**
* numpy: For the array calculations.
* serial: To read the serial data from the radar.
* time: To wait until more data is generated.
* array: For arrays
* Picamera: To take images using picamera

**HOW TO USE**
* Download the required packages.
* Connect the RaspberryPi and mmW radar sensor via USB.
* Power both the RaspberryPi and mmW radar with their appropriate power connections (either through a wall plug or LiPo battery mounted on the drone.
* Establish ssh connection between RaspberryPi and laptop
* Navigate to the appropriate folder on the RaspberryPi and run the code using the command "python mmW_Trash_Detection.py"


## Subsystem Descriptions
Jonathan Chin - Subsystem: Drone Navigation
Current Status: I have the communication between the pixhawk and raspberry pi working. I also have a working simulation of the projected flight path of the drone working. The drone's main base is built and a few things need to be added before flight testing.

Cori Teague - Subsystem: Object Detection
Current Status: I wrote a python script that uses range data read from the TI AWR1443 (mmW Radar sensor) to detect the presence of trash based on the change in depth detected. Once a change in depth within a predetermined threshold is detected, the Pi Camera module is opened and an image is taken of the ground below.

Ali Warsi - Subsystem: Image Processing 
Current Status: Using Tensorflow I have trained it to detect 5 different types of trash. I am currently righting a script to automate the process instead of using the terminal window. 
