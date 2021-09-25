# Installation

Create a new catkin workspace and clone the repository into the src folder. 
Install all the dependencies listed below and build the project. 
Do not forget to source the setup.* file.

# Usage

Starting the baseline agent can be done by starting the gazebo simulator and the baseline argent
with the following commands:
```
roslaunch air_hockey_gazebo air_hockey_gazebo.launch
roslaunch air_hockey_baseline_agent air_hockey_baseline_agent.launch use_back_iiwa:=true
```

# Dependencies

1. iiwas_core https://github.com/PuzeLiu/iiwas_core
2. OSQP https://osqp.org/
3. OSQP Eigen https://github.com/robotology/osqp-eigen
4. NLopt https://nlopt.readthedocs.io
5. Coin-or-CLP (apt-get install coinor-libclp-dev)
6. Eigen3 https://eigen.tuxfamily.org
7. Kalman Filter https://github.com/mherb/kalman
8. Ros Packages: rqt 
9. Python Packages: pyqt, pydot
