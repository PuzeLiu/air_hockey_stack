import numpy as np
import rospy
import gazebo_ros
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from air_hockey_baseline_agent.srv import SetTacticsService, SetTacticsServiceRequest


def main():
    rospy.init_node("air_hockey_set_model", anonymous=True)
    set_puck_state_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    set_tactic_service = rospy.ServiceProxy("/iiwa_front/set_tactic", SetTacticsService)

    tableLength = rospy.get_param("/air_hockey/table_length", 1.956)
    tableWidth = rospy.get_param("/air_hockey/table_width", 1.042)

    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name = 'puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'

    set_tactic = SetTacticsServiceRequest()
    set_tactic.tactic = "SMASH"

    x_array = [0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8]
    y_array = [-0.35, -0.3, -0.25, -0.2, -0.15, -0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35]
    # x_array = [0.8]
    # y_array = [0.35]
    while True:
        i = input("Press Enter to continue...")
        if i == 'r':
            set_puck_state.model_state.pose.position.x = np.random.normal(- tableLength/3, 0.2, 1)
            set_puck_state.model_state.pose.position.y = np.random.normal(0, 0.4, 1)
            set_puck_state.model_state.pose.position.z = 0.

            set_puck_state_service(set_puck_state)
        elif i == 's':
            set_tactic.tactic = "SMASH"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
        elif i =='c':
            set_tactic.tactic = "CUT"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
        elif i =='p':
            set_tactic.tactic = "REPEL"
            set_tactic_service(set_tactic)
            rospy.sleep(2)


if __name__ == "__main__":
    main()