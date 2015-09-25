#! /bin/bash
echo Installing ROS...
sudo apt-get install python-wxglade
wget https://github.com/downloads/jraedler/Polygon2/Polygon-2.0.5.zip
unzip Polygon-2.0.5.zip
cd Polygon-2.0.5
sudo python setup.py install
cd ..
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
. ~/.bashrc
source /opt/ros/indigo/setup.bash
sudo apt-get install python-setuptools python-pip
sudo pip install -U rosinstall vcstools rosdep
sudo easy_install -U rosinstall vcstools rosdep
sudo apt-get install ros-indigo-pr2-desktop
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
. ~/.bashrc
source /opt/ros/indigo/setup.bash
sudo apt-get install python-rsvg
PATHSM='/opt/ros/indigo/stacks/simulator_gazebo/gazebo_worlds'
sudo cp ltlmop_map.world $PATHSM/worlds/ltlmop_map.world
sudo chmod 777 $PATHSM/worlds/ltlmop_map.world
sudo chmod 777 $PATHSM/worlds
sudo chmod 777 $PATHSM/launch
sudo cp ltlmop.launch $PATHSM/launch/ltlmop.launch
sudo chmod 777 $PATHSM/launch/ltlmop.launch
PATHLOC='/opt/ros/indigo/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.0.2/Media/materials'
sudo chmod 777 /opt/ros/indigo/stacks/simulator_gazebo/gazebo/gazebo/share/gazebo-1.0.2/Media/materials/textures
sudo chmod 777 $PATHLOC/scripts/Gazebo.material
if ! grep -q ltlmop $PATHLOC/scripts/Gazebo.material 
	then sudo cat ltlmop.material >> $PATHLOC/scripts/Gazebo.material 
fi
ROS_WORKSPACE=/opt/ros/indigo
sudo apt-get install openjdk-7-jre
sudo apt-get install openjdk-7-jdk
cd ../../../../etc/jtlv
sh build.sh
cd ../../lib/handlers/robots/ROS
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
. ~/.bashrc
source /opt/ros/indigo/setup.bash
