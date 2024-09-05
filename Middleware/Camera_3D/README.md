# Camera 3D

## Installation
Clone sick visionary repo to catkin workspace
<pre>cd ~/autonomous_car/src
git clone https://github.com/SICKAG/sick_visionary_ros.git #clone Repo URL </pre>

## Operation

1. ROS run cmd
<pre>roslaunch sick_visionary_ros sick_visionary-t_mini.lauch</pre>

2. Visialize the results in RViz
<pre>rosrun rviz rviz</pre>

## Important notes
Open SOPAS ET on windows and see 2 different IP addresses: the camera IP and PC network adapter IP. 
Change the ethernet IP to be similar to PC network adapter IP.
Then run with camera IP. 
