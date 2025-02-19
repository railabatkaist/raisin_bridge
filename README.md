# raisin_bridge
ros2 packages that works as a bridge between raisin_network and ros2

## Prerequisite
```
vi ~/.bashrc
```
add 
```
export RAISIN_WS=$YOUR_RAISIN_WS
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$RAISIN_WS/install/lib
```
to the end.

## Build
Create individual workspace for the project
```
mkdir $RAISIN_BRIDGE_WS
cd $RAISIN BRIDGE_WS
mkdir src
cd src
git clone git@github.com:railabatkaist/raisin_bridge.git
python raisin_bridge/update_interfaces.py
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## How to use
Before building, you can modify messages to bridge by editing src/raisin_bridge.cpp
```
cd $RAISIN BRIDGE_WS
source install/setup.bash
ros2 run raisin_bridge raisin_bridge_node
```
