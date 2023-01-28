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

After the build is successful you can check if the SITL is working using the following commands
```
cd ~/ardupilot/ArduCopter1
../Tools/autotest/sim_vehicle.py --map --console
```

