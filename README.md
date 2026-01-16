# simple_teleop

Simplifies drone control for camera calibration purposes as example.

Once teleop command is run the tab can be switched to a different one, as thankfully for `pynput` all keyboard events are listening in background mode and command terminal is not required to be active. 

It is suitable for camera calibration purpose, because we can focus on RVIZ Images and not on linux terminal.

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


#### Requires Python module `pynput` for getting keyboard events.

#### Note: Following key presses increase linear or angular speed by the provided step argument.

#### Inspirated by origin project: https://github.com/tonynajjar/keyboard_teleop
