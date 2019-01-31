Jonathan Chin - Subsystem: Drone Navigation
Current Status: I have the communication between the pixhawk and raspberry pi working. I also have a working simulation of the projected flight path of the drone working. The drone's main base is built and a few things need to be added before flight testing.

Cori Teague - Subsystem: Object Detection
Current Status: I wrote a python script that reads range data from the TI IWR1443 (mmW Radar sensor) detects the presence of trash based on the change in depth detected. I am working toward connecting the IWR1443 and the raspberry pi to process the data from the sensor during the drone's flight and send a signal telling the drone to hover over the piece of trash.

Ali Warsi - Subsystem: Image Processing 
Current Status: Using Tensorflow I have trained it to detect 5 different types of trash. I am currently righting a script to automate the process instead of using the terminal window. 
