#Installation
Create a new catkin workspace and clone the repository into the src folder. 
Install all the dependencies listed below and build the project. 
Do not forget to source the setup.* file.

#Usage
Starting the baseline agent can be done by starting the gazebo simulator and the baseline argent
with the following commands:
```
roslaunch air_hockey_gazebo air_hockey_gazebo.launch
roslaunch air_hockey_baseline_agent air_hockey_baseline_agent.launch use_back_iiwa:=true
```

#Dependencies
1. OSQP https://osqp.org/
2. OSQP Eigen https://robotology.github.io/osqp-eigen/doxygen/doc/html/index.html
3. NLopt https://nlopt.readthedocs.io/en/latest/
4. Coin-or-CLP (apt-get install coinor-libclp-dev)
5. Eigen3 https://eigen.tuxfamily.org/index.php?title=Main_Page
6. Kalman Filter https://github.com/mherb/kalman (clone recursive)
7. Ros Packages: rqt 
8. Python Packages: pyqt, pydot
