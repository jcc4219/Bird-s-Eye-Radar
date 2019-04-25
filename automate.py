#!/usr/bin/python3.7.2
import os
import docker

################################################
# Prework
################################################
# git clone https://github.com/pypa/pip
# cd pip
# sudo python setup.py install
# sudo pip install docker-py
################################################
##  Check docker is installed correctly and 
##  run tensorflow
################################################
os.system("cd /Users/Nida/")
os.system("docker run hello-world")
os.system("docker run -it tensorflow/tensorflow:1.1.0 bash")
os.system("docker ps")
################################################
## Ctrl D manually on the terminal to exit 
## docker.  Rest of the code will execute.
###############################################
os.system("cd /Users/Nida/code/tensorflow_hh")
os.system("docker run -it --volume ${HOME}/tf_files_2:/tf_files_2 --workdir /tf_files_2 --publish 6006:6006 tensorflow/tensorflow:1.1.0 bash")




#os.system("python retrain.py --bottleneck_dir=bottlenecks --how_many_training_steps=500 --model_dir=inception --summaries_dir=training_summaries/long --output_graph=retrained_graph.pb --output_labels=retrained_labels.txt --image_dir=trash")
################################################
##  Download retrain script
################################################
# In a different shell call the copy_directory.py
#  script to  copy the traing files files in
 
client = docker.from_env()
print client
print client.containers.list()
#print client.containers.run('copy', 'cp ') 
client.containers.run('retrain', 'python retrain.py --bottleneck_dir=bottlenecks --how_many_training_steps=500 --model_dir=inception --summaries_dir=training_summaries/long --output_graph=retrained_graph.pb --output_labels=retrained_labels.txt --image_dir=trash')
#client.containers.run('retrain', 'curl -0 https://raw.githubusercontent.com/tensorflow/tensorflow/r1.1/tensorflow/examples/image_retraining/retrain.py')

