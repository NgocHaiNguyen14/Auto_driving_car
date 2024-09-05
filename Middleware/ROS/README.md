# ROS 

## Installation
1. Install Ubuntu 20.04
2. Install ROS noetic
link: http://wiki.ros.org/noetic/Installation/Ubuntu
2.1 Setup your sources.list
<pre>sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'</pre>
2.2 Set up your keys
<pre>sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -</pre>
2.3 Install
<pre>sudo apt update
sudo apt install ros-noetic-desktop-full</pre>


## Tutorial

link: https://www.youtube.com/watch?v=Qk4vLFhvfbI&list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q
