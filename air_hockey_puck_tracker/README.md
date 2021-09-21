# air_hockey_puck_tracker

Kalman Filter is used to track and predict puck's movement.

Required package
* Eigen3
* Kalman Filter: https://github.com/mherb/kalman/tree/master

Test Kalman Filter Only

```asm
roslaunch air_hockey_gazebo air_hockey_gazebo.launch 
roslaunch air_hockey_puck_tracker air_hockey_puck_tracker.launch
```

Collision model between puck, table and mallet is following:

* Partridge, C. B., & Spong, M. W. (2000). Control of Planar Rigid Body Sliding with Impacts and Friction. International Journal of Robotics Research, 19(4), 336–348. https://doi.org/10.1177/02783640022066897

