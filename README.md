# simple_teleop

### Run in Command Line
```
# publish twist messages into the default MavRos  topic: /cmd_vel
ros2 run simple_teleop teleop
# OR
ros2 run simple_teleop teleop --ros-args -p topic:=/drone0/cmd_vel
```

### Default Arguments:

```topic = /cmd_vel```, 
```step = 0.2```, 
```publish_rate = 10```
