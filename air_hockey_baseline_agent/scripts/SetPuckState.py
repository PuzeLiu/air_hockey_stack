import numpy as np
import rospy
import gazebo_ros
from gazebo_msgs.srv import SetModelState,SetModelStateRequest



def main():
    rospy.init_node("air_hockey_set_model", anonymous=True)
    set_puck_state_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name ='puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'

    x_array = [0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
    y_array = [-0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35]
    # x_array = [0.8]
    # y_array = [0.35]
    for x in x_array:
        for y in y_array:
            set_puck_state.model_state.pose.position.x = x - 1.956 / 2
            set_puck_state.model_state.pose.position.y = y
            set_puck_state.model_state.pose.position.z = 0.

            set_puck_state_service(set_puck_state)
            rospy.sleep(3)
            if x >= 0.8:
                rospy.sleep(2)

            set_puck_state.model_state.pose.position.x = 0.4
            set_puck_state.model_state.pose.position.y = 0.
            set_puck_state.model_state.pose.position.z = 0.

            set_puck_state_service(set_puck_state)
            rospy.sleep(2)

if __name__ == "__main__":
    main()