#!/bin/sh

# Need to add LD_LIBRARY_PATH environment variable
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rot1bp/.mujoco/mujoco200/bin
# Path of the compiled xml file of the GTPR model
directory=$(dirname -- $(readlink -fn -- "$0"))
xml_path=$directory/FolderB/envs/compiled.xml

if [ "$1" = train ];
then
   echo "Training of a model has begun!"
   python3 main.py $1
elif [ "$1" = sin ];
then
   echo "Sinus performance has started!"
   python3 main.py $1
elif [ "$1" = simu ];
then
   echo "Simulation of $xml_path file has begun"
   cd /home/rot1bp/.mujoco/mujoco200/bin/
   # Using the simulation application of mujoco200
   /home/rot1bp/.mujoco/mujoco200/bin/simulate $xml_path
else
   echo "None of the condition met"
fi