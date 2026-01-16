# simple_teleop

Simplifies drone control for camera calibration purposes as example.

### Control Robot via simple teleop command:
```
Use Arrows to control linear velocities:

    ↑ / ↓ - X axis - Move Forward & Backwards
    ← / → - Y axis - Move Left & Right

    w/s - Z axis - Move UP & DOWN
    a/d - Z axis - Rotate/YAW Left & Right

    esc: QUIT,
    other key: STOP movement

```

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


### Requires Python module `pynput` for getting keyboard events
