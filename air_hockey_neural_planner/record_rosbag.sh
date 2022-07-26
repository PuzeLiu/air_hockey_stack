duration=${1:-100}
rosbag record --duration=${duration} /iiwa_front/bspline_adrc_joint_trajectory_controller/state /iiwa_front/joint_states /tf \
              /neural_planner/status /neural_planner/plan_trajectory
