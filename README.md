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

## Image Classifier and GUI

################################################
# Prework                                                                                                                                           
################################################                                                                                                                                                                            
# git clone https://github.com/pypa/pip                                                                                                                                                                                     
# cd pip                                                                                                                                                                                                                    
# sudo python setup.py install
# sudo pip install docker-py 


################################################
# Steps to run and display data
################################################

1. First enable docker on mac 
2. Then kick off automate.py using command 
   python automate.py
   a) Ctrl D the first time to esc the docker bash 
3. Now need to run copy_container.py in another shell with command:
   python copy_container.py | tee container_id.txt  
4. Now in the docker terminal kick off the following commannds:
   a)python retrain.py --bottleneck_dir=bottlenecks --how_many_training_steps=500 --model_dir=inception --summaries_dir=training_summaries/long --output_graph=retrained_graph.pb --output_labels=retrained_labels.txt --image_dir=trash
   b)python label_image.py test/Test/redcup.jpg > results.log    *****Image path must match the one being passed to the gui*****
5. From the non-docker terminal:
   Run python copy_results.py it will copy the results into a csv file on desktop
6. Now initiate the gui from the non docker terminal 
   python3 main_gui_BER.py
7. Click start and reenter the same path as in 4b) and hit Enter 


#############################################
# Copy a new image into the flow 
#############################################
1. Add new image to the test folder: 
cp <source> /Users/Nida/tf_files_2/test/Test/.
2. Same path can be passed to the gui

#############################################
# Retraining a new image
#############################################
1. Create a folder on desktop with traing images
2. Go into /Users/tf_files/trash and add the new folder name 
3. edit copy_container.py point to new folder
4. Retrain 4a


## Subsystem Descriptions
Jonathan Chin - Subsystem: Drone Navigation
Current Status: I have the communication between the pixhawk and raspberry pi working. I also have a working simulation of the projected flight path of the drone working. The drone's main base is built and a few things need to be added before flight testing.

Cori Teague - Subsystem: Object Detection
Current Status: I wrote a python script that uses range data read from the TI AWR1443 (mmW Radar sensor) to detect the presence of trash based on the change in depth detected. Once a change in depth within a predetermined threshold is detected, the Pi Camera module is opened and an image is taken of the ground below.

Ali Warsi - Subsystem: Image Classifier
Current Status: Using Tensorflow and a pretraining model called inception I was able to train 6 different types of trash. This includes a plastic bag, red plastic cup, a cardboard box, a plastic fork, a coke can and a plastic bottle. Then I parsed the csv file generated by the retrain script on to a GUI creaded in python. The results are displayed in a bar graph along with GPS coordinates of the trash.
