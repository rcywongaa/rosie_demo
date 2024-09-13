# Setup
1.  Install ROS2
1.  In this directory, run
    ```
    rosdep install -r -y --from-paths . --ignore-src
    ```
1.  Install HEBI python API
    ```
    pip install --user hebi-py==2.6.2
    ```

# Build
```
source /opt/ros/humble/setup.bash && colcon build
```
# Run
```
source install/setup.bash && ros2 launch rosie_demo bringup_launch.yaml
```
