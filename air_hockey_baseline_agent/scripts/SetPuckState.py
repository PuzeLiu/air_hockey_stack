import numpy as np
import rospy
import gazebo_ros
from air_hockey_referee.srv import StartGame
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from air_hockey_baseline_agent.srv import SetTacticsService, SetTacticsServiceRequest
from sensor_msgs.msg import JointState

currentJointState = np.zeros((7, 1))
initJointState = np.zeros((7, 1))
puckState = np.zeros((6, 1))
lastStrategy = ''


def set_puck_again():
    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name = 'puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'
    print(puckState)
    set_puck_state.model_state.pose.position.x = puckState[0]
    set_puck_state.model_state.pose.position.y = puckState[1]
    set_puck_state.model_state.pose.position.z = 0.

    set_puck_state.model_state.twist.linear.x = puckState[3]
    set_puck_state.model_state.twist.linear.y = puckState[4]
    return set_puck_state


def set_puck_defend(table_w):
    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name = 'puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'
    x = np.random.uniform(0.2, 0.55, 1)
    y = np.random.uniform(-table_w/2, table_w/2, 1)
    set_puck_state.model_state.pose.position.x = x
    set_puck_state.model_state.pose.position.y = y
    set_puck_state.model_state.pose.position.z = 0.

    dx = np.random.uniform(-1.5, -0.5, 1)
    dy = np.random.normal(0.5, 0.1, 1)
    set_puck_state.model_state.twist.linear.x = dx
    set_puck_state.model_state.twist.linear.y = dy
    global puckState
    puckState[0] = x
    puckState[1] = y
    puckState[3] = dx
    puckState[4] = dy
    return set_puck_state


def set_puck_smash(table_w, table_l):

    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name = 'puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'
    x = np.random.uniform(-table_l/2 + 0.2, -table_l/2 + 0.55, 1)
    y = np.random.uniform(-table_w/2 - 0.1, table_w/2, 1)
    set_puck_state.model_state.pose.position.x = x
    set_puck_state.model_state.pose.position.y = y
    set_puck_state.model_state.pose.position.z = 0.
    global puckState
    puckState[0] = x
    puckState[1] = y
    print("setting puck state to", puckState)
    return set_puck_state


def callback(data):
    if len(data.position) == 7:
        global currentJointState
        currentJointState = np.asarray(data.position)


def main():
    rospy.init_node("air_hockey_set_model", anonymous=True)
    set_puck_state_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    start_game_service = rospy.ServiceProxy("/air_hockey_referee/start_game", StartGame)
    set_tactic_service = rospy.ServiceProxy("/iiwa_front/set_tactic", SetTacticsService)
    global lastStrategy

    tableLength = rospy.get_param("/air_hockey/table_length", 1.956)
    tableWidth = rospy.get_param("/air_hockey/table_width", 1.042)

    set_puck_state = SetModelStateRequest()
    set_puck_state.model_state.model_name = 'puck'
    set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'

    set_tactic = SetTacticsServiceRequest()
    start_game_service()

    while True:
        rospy.Subscriber("/iiwa_front/joint_states", JointState, callback)
        i = input("Press Enter to continue...")
        if i == 'sm':
            set_puck_state_service(set_puck_smash(tableWidth, tableLength))
            rospy.sleep(2)
            set_tactic.smashStrategy = 0
            lastStrategy = "SMASH"
            set_tactic.tactic = "SMASH"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'sl':
            set_puck_state_service(set_puck_smash(tableWidth,tableLength))
            rospy.sleep(2)
            set_tactic.smashStrategy = 1
            lastStrategy = "SMASH"
            set_tactic.tactic = "SMASH"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'sr':
            set_puck_state_service(set_puck_smash(tableWidth, tableLength))
            rospy.sleep(2)
            set_tactic.smashStrategy = 2
            lastStrategy = "SMASH"
            set_tactic.tactic = "SMASH"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'c':
            set_puck_state_service(set_puck_defend(tableWidth))
            rospy.sleep(0.5)

            lastStrategy = "CUT"
            set_tactic.tactic = "CUT"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'p':
            lastStrategy = "PREPARE"
            set_tactic.tactic = "PREPARE"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'e':
            lastStrategy = "REPEL"
            set_tactic.tactic = "REPEL"
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)
        elif i == 'a':
            print('Again')
            set_puck_state_service(set_puck_again())
            rospy.sleep(0.5)
            set_tactic.tactic = lastStrategy
            set_tactic_service(set_tactic)
            rospy.sleep(2)
            set_tactic_service(set_tactic)


if __name__ == "__main__":
    main()