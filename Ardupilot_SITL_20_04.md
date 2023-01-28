# Installing Ardupilot and MAVProxy Ubuntu 20.04

## Clone ArduPilot

In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

## Install dependencies:
```
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

reload profile
```
. ~/.profile
```

## Checkout Latest Copter Build
```
git checkout Copter-4.0.4
git submodule update --init --recursive
```

Submodule update might fail if some firewalls do not allow ssh access. You can tell git to unilaterally use https through the following command:
```
git config --global url."https://github.com/".insteadOf git@github.com:
git config --global url."https://".insteadOf git://
```

Run SITL (Software In The Loop) once to set params:
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

***You can also do this to build the repo if you are facing problems***
```
./waf configure --board sitl
./waf copter
```

After the build is successful you can check if the SITL is working using the following commands
```
cd ~/ardupilot/ArduCopter1
../Tools/autotest/sim_vehicle.py --map --console
```

# Installing Gazebo and ArduPilot Plugin

## Install Gazebo [***18.04-20.04***]

Setup your computer to accept software from http://packages.osrfoundation.org:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```

Setup keys:
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```

Reload software list:
```
sudo apt update
```

Install Gazebo:
### Ubuntu [***18.04***]
```
sudo apt install gazebo9 libgazebo9-dev
```
### Ubuntu [***20.04***]
```
sudo apt-get install gazebo11 libgazebo11-dev
```

for more detailed instructions for installing gazebo checkout http://gazebosim.org/tutorials?tut=install_ubuntu


## Install Gazebo plugin for APM (ArduPilot Master) :
```
cd ~
git clone --recurse https://github.com/khancyr/ardupilot_gazebo.git
cd ardupilot_gazebo
```
***Ubuntu 18.04 only*** checkout dev
```
git checkout dev
```
build and install plugin
```
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
```
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
```
Set paths for models:
```
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc
. ~/.bashrc
```

## Run Simulator

***NOTE the iris_arducopter_runway is not currently working in gazebo11.***

In one Terminal (Terminal 1), run Gazebo:
```
gazebo --verbose ~/ardupilot_gazebo/worlds/iris_arducopter_runway.world
```

In another Terminal (Terminal 2)(use Ctrl+Shift+T to open another terminal), run SITL:
```
cd ~/ardupilot/ArduCopter/
../Tools/autotest/sim_vehicle.py -f gazebo-iris --console --map
```

# Installing QGroundControl on Ubuntu Linux 20.04 LTS

On a new terminal enter
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
```
Logout and login again to enable the change to user permissions.
***You can use this command if you are a command line nerd (remember to save any current progress as this instantly logs you out).***
```
loginctl terminate-user $USER
```

Download the latest QGroundControl.AppImage (you can check [here]{https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html} for the latest version)
```
cd ~ # Or download the Image in any directory of your choice
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```
Change permissions and run 
```
chmod +x ./QGroundControl.AppImage 
./QGroundControl.AppImage  (or double click)
```

## Run SITL and connect with Q Ground

In another terminal run SITL and QGround will automatically connect.
```
cd ~/ardupilot/ArduCopter/
sim_vehicle.py
```

# Install ROS and Setup Catkin Workspace

We will be installing **ROS Noetic**

## 1. Install ROS

   - Do _Desktop-full Install_
   - Follow all the steps from the given website

   First, install **ROS Noetic** using the following instructions: http://wiki.ros.org/noetic/Installation/Ubuntu


## 2. Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:
```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

Then, initialize the catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

## 3. Dependencies installation

Install `mavros` and `mavlink` from source:
```
cd ~/catkin_ws
wstool init ~/catkin_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
If the catkin build fails and the error says
```
module 'enum' has no attribute 'IntFlag'
```
Simply uninstall enum34 from the environment which is overriding built-in enum in Python. This package was probably installed by something as for forward compatibility which is no longer needed with Python 3.6+
```
pip uninstall -y enum34
```

Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

install geographiclib dependancy 
```
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Now follow the [ROS tutorials][http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem] to learn how to create ROS packages.






