# Voice-Driven Robotic Manipulation (ROS 2)

This project implements a voice-controlled pick-and-place robotic arm pipeline using ROS 2.
It combines object detection (OpenCV), speech command interpretation, coordinate-to-joint mapping, and serial motor control.
The current phase focuses on fixed-workspace operation and establishes a base for future LLM-guided planning and mobile manipulation.

## To Reproduce

```bash
cd ~/tmr
source ros2_env/bin/activate
colcon build --packages-select rtk
source install/setup.bash
ros2 launch rtk project.launch.py camera_id:=4
```

Optional manual control test:

```bash
ros2 launch rtk manual_control.launch.py
```
