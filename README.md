# raisin_bridge
ros2 packages that works as a bridge between raisin_network and ros2

## Prerequisite
```
vi ~/.bashrc
```
add 
```
export RAISIN_WS=$YOUR_RAISIN_WS
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
python3 raisin_bridge/update_interfaces.py
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
## you can add "--packages-up-to $INTERFACE_conversion" to build necessary packages only
```

Build will take more than 5 minutes, but you don't have to build everytime.

It is is required only when there's a update in raisin's msgs. 

[```python raisin_bridge/update_interfaces.py```](https://github.com/railabatkaist/raisin_bridge/blob/main/update_interfaces.py#L289)

It scans all msg files under ```$RAISIN_WS/install/messages``` and make cmake project under ```generated``` so that it can build dynamic library to bridge raisin and ros2.
## How to use
Before running, you can modify messages to bridge by editing ```config/params.yaml```.

You can either use TCP, WEBSOCKET. You can also modify id of the bridge, and the peer network(network to exchange messages).
```
cd $RAISIN BRIDGE_WS
source install/setup.bash
source $RAISIN_WS/ld_prefix_path.sh
ros2 launch raisin_bridge raisin_bridge_launch.py
```
