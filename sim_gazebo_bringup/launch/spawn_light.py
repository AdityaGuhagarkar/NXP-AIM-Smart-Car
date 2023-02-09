import os
from time import sleep

spawn_green="gz model --spawn-file=/home/manish/ros2ws/nxp_gazebo/models/traffic_light_green/model.sdf --model-name=lightGreen -x 0.780034 -y 2.566224 -z 0.328665 -R -3.138687 -P 0.000255 -Y 1.606558"
del_green="gz model -m lightGreen -d"

while(True):	
	sleep(10)
	os.system(spawn_green)
	sleep(10)
	os.system(del_green)	
