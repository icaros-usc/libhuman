# LibHuman

## Examples 

### LibAda examples

- roslaunch libada simulation.launch
- rviz  
- $ cd /home/ros_dev/ros_ws/src/libada/python/tests
- /home/ros_dev/ros_ws/src/libada/python/tests$ python3 simple_trajectories.py

### libAda ready to fix

Try to fix the bug in libhuman/src/Human.cpp
- roslaunch libhuman simulation.launch
- ~/ros_ws# catkin build
- ~/ros_ws# cd devel/bin/
- ~/ros_ws/devel/bin# ./simple_load

Troubleshooting:
- ModuleNotFoundError: No module named 'moveit_ros_planning_interface'
    - $ apt update && apt install ros-noetic-moveit

