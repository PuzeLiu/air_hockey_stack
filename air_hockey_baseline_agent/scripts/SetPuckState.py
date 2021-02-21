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

    x_array = np.linspace(0.25, 0.9, 10, endpoint=True)
    y_array = np.linspace(-0.38, 0.38, 10, endpoint=True)
    x_array = [0.89]
    for x in x_array:
        for y in y_array:
            set_puck_state.model_state.pose.position.x = x - 1.956 / 2
            set_puck_state.model_state.pose.position.y = y
            set_puck_state.model_state.pose.position.z = 0.

            set_puck_state_service(set_puck_state)
            rospy.sleep(3)
            if x > 0.8:
                rospy.sleep(2)

            set_puck_state.model_state.pose.position.x = 0.4
            set_puck_state.model_state.pose.position.y = 0.
            set_puck_state.model_state.pose.position.z = 0.

            set_puck_state_service(set_puck_state)
            rospy.sleep(3)

if __name__ == "__main__":
    main()