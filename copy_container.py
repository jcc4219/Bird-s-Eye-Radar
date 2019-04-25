#!/usr/bin/python3.7.2                                                                                                                                                                             

#################################################
# This script will copy the reference images to 
# docker container for training
#################################################

#################################################
# python copy.py | tee container_id.txt
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
    os.system("docker cp ~/Desktop/Coke_cans " + container_id +":/tf_files_2/trash/.")
    os.system("docker cp ~/Desktop/Plastic_bottles " + container_id +":/tf_files_2/trash/.")
    os.system("docker cp ~/Desktop/Disposable_fork " + container_id +":/tf_files_2/trash/.")
    os.system("docker cp ~/Desktop/Plastic_bags " + container_id +":/tf_files_2/trash/.")
    os.system("docker cp ~/Desktop/Red_cup " + container_id +":/tf_files_2/trash/.")
    os.system("docker cp ~/Desktop/Cardboard_box " + container_id +":/tf_files_2/trash/.")
