import numpy as np
import rospy
import gazebo_ros
from air_hockey_referee.srv import StartGame, StopGame
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from air_hockey_baseline_agent.srv import SetTacticsService, SetTacticsServiceRequest
from sensor_msgs.msg import JointState


class TacticsDebugger():
    def __init__(self, agent_ns="/iiwa_front"):
        self.agent_ns = agent_ns
        self.set_puck_state_client = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.start_game_client = rospy.ServiceProxy("/air_hockey_referee/start_game", StartGame)
        self.set_tactic_client = rospy.ServiceProxy(self.agent_ns + "/set_tactic", SetTacticsService)

        self.table_length = rospy.get_param("/air_hockey/table_length", 1.948)
        self.table_width = rospy.get_param("/air_hockey/table_width", 1.038)
        self.puck_radius = rospy.get_param("/air_hockey/puck_radius", 0.03165)
        self.mallet_radius = rospy.get_param("/air_hockey/mallet_radius", 0.04815)
        self.goal_width = rospy.get_param("/air_hockey/goal_width", 0.25)
        self.tactics_switch_time = rospy.get_param("/air_hockey/agent/min_tactic_switch_time", 0.5)

        self.puck_state_srv = SetModelStateRequest()
        self.puck_state_srv.model_state.model_name = 'puck'
        self.puck_state_srv.model_state.reference_frame = 'air_hockey_table::Table'

        self.set_tactic_srv = SetTacticsServiceRequest()
        self.reset_puck = True

    def start_game(self):
        self.start_game_client()
        rospy.sleep(3.0)
        rospy.loginfo("start the game")

    def start(self):
        while not rospy.is_shutdown():
            tactic = input("Wait for Tactics: "
                           "\n\t h: HIT / SMASH, "
                           "\n\t c: CUT / DEFEND, "
                           "\n\t p: PREPARE, "
                           "\n\t r: REPEL,"
                           "\n\t q: Quit,"
                           "\n\t z: Turn On/Off Reset Puck."
                           "\n Press Enter to Continue: ")
            if tactic == 'h':
                if self.reset_puck:
                    puck_pos = np.random.uniform([-self.table_length / 2 + 0.2, -self.table_width / 2 + 0.1],
                                                 [-0.3, self.table_width / 2 - 0.1])
                    self.set_puck_state(puck_pos[0], puck_pos[1], 0., 0.)
                    rospy.sleep(0.5)
                self.set_tactic_srv.tactic = "SMASH"
                self.set_tactic_srv.smashStrategy = 0
                self.set_tactic_client(self.set_tactic_srv)
            elif tactic == 'c':
                self.set_tactic_srv.tactic = 'CUT'
                self.set_tactic_client(self.set_tactic_srv)
                if self.reset_puck:
                    puck_pos = np.random.uniform([0.3, -self.table_width / 2 + 0.1],
                                                 [self.table_length / 2 - 0.2, self.table_width / 2 - 0.1])
                    puck_vel = np.random.uniform([-2.0, -0.8], [-0.5, 0.8])
                    self.set_puck_state(puck_pos[0], puck_pos[1], puck_vel[0], puck_vel[1])
            elif tactic == 'p':
                if self.reset_puck:
                    puck_pos = np.random.uniform([-self.table_length / 2 + self.puck_radius, -self.table_width / 2 + 0.1],
                                                 [self.table_length / 2 - 0.2, self.table_width / 2 - 0.1])
                    self.set_puck_state(puck_pos[0], puck_pos[1], 0., 0.)
                    rospy.sleep(0.5)
                self.set_tactic_srv.tactic = 'PREPARE'
                self.set_tactic_client(self.set_tactic_srv)
            elif tactic == 'r':
                self.set_tactic_srv.tactic = 'REPEL'
                self.set_tactic_client(self.set_tactic_srv)
                if self.reset_puck:
                    puck_pos = np.random.uniform([0.3, -self.table_width / 2 + 0.1],
                                                 [self.table_length / 2 - 0.2, self.table_width / 2 - 0.1])
                    target_pos = np.random.uniform([-self.table_length / 2, -self.goal_width / 2],
                                                   [-self.table_length / 2, self.goal_width / 2])
                    puck_vel = target_pos - puck_pos
                    puck_vel = puck_vel / np.linalg.norm(puck_vel) * np.random.uniform(0.5, 2.0)
                    self.set_puck_state(puck_pos[0], puck_pos[1], puck_vel[0], puck_vel[1])
            elif tactic == 'z':
                self.reset_puck = not self.reset_puck
                rospy.loginfo("\nReset puck: {}".format("On" if self.reset_puck else "Off"))
            elif tactic == 'q':
                rospy.loginfo("Exit the node.")
                break
            rospy.sleep(self.tactics_switch_time)

    def set_puck_state(self, x, y, dx, dy):
        set_puck_state = SetModelStateRequest()
        set_puck_state.model_state.model_name = 'puck'
        set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'
        set_puck_state.model_state.pose.position.x = x
        set_puck_state.model_state.pose.position.y = y
        set_puck_state.model_state.pose.position.z = 0.

        set_puck_state.model_state.twist.linear.x = dx
        set_puck_state.model_state.twist.linear.y = dy
        return self.set_puck_state_client(set_puck_state)


def main():
    rospy.init_node("air_hockey_set_puck_state", anonymous=True)
    tactics_debugger = TacticsDebugger()
    tactics_debugger.start_game()
    tactics_debugger.start()


if __name__ == "__main__":
    main()
