#!/bin/sh
# argumentumok megadása
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rot1bp/.mujoco/mujoco200/bin
echo "First arg: $1"

if [ $1 = 'train' ]
then
   echo "Training of a model has been started!"
   python3 main.py
elif [ $1 = 'simu' ]
then
   cd /home/rot1bp/.mujoco/mujoco200/bin/
   /home/rot1bp/.mujoco/mujoco200/bin/simulate /home/rot1bp/Desktop/work_custom_mujoco_project/OE/gtpr/FolderB/envs/compiled.xml
else
   echo "None of the condition met"
fi