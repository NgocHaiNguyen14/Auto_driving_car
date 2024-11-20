# LiDAR

## Installation
<pre>sudo apt update
sudo apt-get install ros-noetic-sick-scan-xd</pre>

## Operation

<pre>roslaunch sick_scan_xd sick_lms_1xx.launch </pre>
Load data to text file - realtime
<pre>rostopic echo /cloud > lidar_data_Hai_test.txt</pre>
Datapoint virtualization by Rviz
<rviz>
Then select "Add" and choose "By topic" / "cloud" / "PointCloud2"

**Remember to connect Ethernet of Lidar to CH2 of the AdvancedTech**
