#!/usr/bin/python3.7.2                                                                                                                                                                             

#################################################
# This script will copy the reference images to 
# docker container for training
#################################################

#################################################
# python copy_results.py
#################################################
import os


os.system("cd /Users/Nida/")
os.system("docker ps")
array =  []
with open('container_id.txt', 'r') as file:
    for line in file:
        array.append(line)

for x in range(len(array)): 
    print array[x], 
    
if (len(array) == 1):
    print "No running container"
elif (len(array) > 1):
    container_id = array[1].split()[0]
    print container_id
    os.system("docker cp " + container_id +":/tf_files_2/results.log ~/Desktop/results.csv")
    
