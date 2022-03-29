import datetime
import os

import numpy as np
import rospy
from air_hockey_referee.srv import StartGame, PauseGame
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from air_hockey_baseline_agent.srv import SetTacticsService, SetTacticsServiceRequest
from air_hockey_puck_tracker.srv import PuckTrackerResetService, PuckTrackerResetServiceRequest
from geometry_msgs.msg import TransformStamped
import tf2_ros


class TestStrategy:
    def __init__(self):

        self.malletStates = []
        self.trajectory = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.set_puck_state_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.set_tactic_service = rospy.ServiceProxy("/iiwa_front/set_tactic", SetTacticsService)
        self.reset_tracker_srv = rospy.ServiceProxy('/iiwa_front/reset_puck_tracker', PuckTrackerResetService)
        start_game_service = rospy.ServiceProxy("/air_hockey_referee/start_game", StartGame)
        pause_game_service = rospy.ServiceProxy("/air_hockey_referee/pause_game", PauseGame)

        self.tableLength = rospy.get_param("/air_hockey/table_length", 1.956)
        self.tableWidth = rospy.get_param("/air_hockey/table_width", 1.042)
        self.goalWidth = rospy.get_param("/air_hockey/goal_width", 0.25)

        self.set_puck_state = SetModelStateRequest()
        self.set_puck_state.model_state.model_name = 'puck'
        self.set_puck_state.model_state.reference_frame = 'air_hockey_table::Table'

        start_game_service()
        pause_game_service()

    def set_puck_repel(self, vel=2.):
        x = self.tableLength/2 - 0.1
        y = np.random.uniform(-self.tableWidth/2 + 0.1, self.tableWidth - 0.1, 1)
        self.set_puck_state.model_state.pose.position.x = x
        self.set_puck_state.model_state.pose.position.y = y
        self.set_puck_state.model_state.pose.position.z = 0.

        dx = -self.tableLength/2 - x
        dy = 0 - y
        v = np.asarray([dx, dy])
        v = v/np.linalg.norm(v)
        v = v * vel
        self.set_puck_state.model_state.twist.linear.x = v[0]
        self.set_puck_state.model_state.twist.linear.y = v[1]

        try:
            req = PuckTrackerResetServiceRequest()
            req.state.x = x
            req.state.y = y
            req.state.dx = v[0]
            req.state.dy = v[1]
            req.state.theta = 0.
            req.state.dtheta = 0.

            self.reset_tracker_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def set_puck_defend(self, vel=2.):
        x = self.tableLength/2 - 0.1
        y = np.random.uniform(-self.tableWidth/2 + 0.1, self.tableWidth/2-0.1, 1)
        self.set_puck_state.model_state.pose.position.x = x
        self.set_puck_state.model_state.pose.position.y = y
        self.set_puck_state.model_state.pose.position.z = 0.

        gx = np.random.uniform(-self.tableLength/4, -0.1)
        gy_left = self.tableWidth
        gy_right = -self.tableWidth
        choice = np.random.randint(0, 2)
        dx = gx - x
        dy = (gy_left if choice < 1 else gy_right) - y
        v = np.asarray([dx, dy])
        v = v/np.linalg.norm(v)
        v = v * vel
        self.set_puck_state.model_state.twist.linear.x = v[0]
        self.set_puck_state.model_state.twist.linear.y = v[1]

        try:
            req = PuckTrackerResetServiceRequest()
            req.state.x = x
            req.state.y = y
            req.state.dx = v[0]
            req.state.dy = v[1]
            req.state.theta = 0.
            req.state.dtheta = 0.

            self.reset_tracker_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def set_puck_smash(self):
        x = np.random.uniform(-self.tableLength/2 + 0.2, -self.tableLength/2 + 0.8, 1)
        y = np.random.uniform(-self.tableWidth/2 + 0.1, self.tableWidth/2 - 0.1, 1)
        self.set_puck_state.model_state.pose.position.x = x
        self.set_puck_state.model_state.pose.position.y = y
        self.set_puck_state.model_state.pose.position.z = 0.
        self.set_puck_state.model_state.twist.linear.x = 0.
        self.set_puck_state.model_state.twist.linear.y = 0.
        self.set_puck_state.model_state.twist.linear.z = 0.

        try:
            req = PuckTrackerResetServiceRequest()
            req.state.x = x
            req.state.y = y
            req.state.dx = 0.
            req.state.dy = 0.
            req.state.theta = 0.
            req.state.dtheta = 0.

            self.reset_tracker_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def set_puck_prepare(self):
        x = np.random.uniform(-self.tableLength/2, -self.tableLength/2 + 0.6, 1)
        y1 = np.random.uniform(-self.tableWidth/2 + 0.035, -self.tableWidth/2 + 0.15, 1)
        y2 = np.random.uniform(self.tableWidth/2 - 0.15, self.tableWidth/2 - 0.035, 1)
        choice = np.random.randint(0, 2)
        y = y1 if choice < 1 else y2
        self.set_puck_state.model_state.pose.position.x = x
        self.set_puck_state.model_state.pose.position.y = y
        self.set_puck_state.model_state.pose.position.z = 0.
        self.set_puck_state.model_state.twist.linear.x = 0.
        self.set_puck_state.model_state.twist.linear.y = 0.
        self.set_puck_state.model_state.twist.linear.z = 0.
        try:
            req = PuckTrackerResetServiceRequest()
            req.state.x = x
            req.state.y = y
            req.state.dx = 0.
            req.state.dy = 0.
            req.state.theta = 0.
            req.state.dtheta = 0.

            self.reset_tracker_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    @staticmethod
    def computeVel(lastTf, puckTf):
        dt = (lastTf.header.stamp - puckTf.header.stamp).to_sec()
        dx = (lastTf.transform.translation.x - puckTf.transform.translation.x) / dt
        dy = (lastTf.transform.translation.y - puckTf.transform.translation.y) / dt
        vel = np.linalg.norm(np.asarray([dx, dy]))
        return vel

    def goal(self, trans):
        if trans.x < -(self.tableLength / 2 + 1e-2) and abs(trans.y) < (self.goalWidth / 2) - 0.03165:
            return True
        elif trans.x > (self.tableLength / 2 + 1e-2) and abs(trans.y) < self.goalWidth / 2 - 0.03165:
            return True
        else:
            return False

    def detectEnd(self):
        end = False
        begin = rospy.Time.now()
        lastTf = TransformStamped()
        tmp_traj = []
        tmp_mallet_traj = []
        while not end:
            malletTf = self.tfBuffer.lookup_transform("Table", "F_striker_mallet_tip", rospy.Time(0), rospy.Duration(1))
            puckTf = self.tfBuffer.lookup_transform("Table", "Puck", rospy.Time(0), rospy.Duration(1))
            tmp_traj.append(np.array([puckTf.transform.translation.x, puckTf.transform.translation.y, puckTf.header.stamp.to_sec()]))
            tmp_mallet_traj.append(np.array([malletTf.transform.translation.x, malletTf.transform.translation.y, malletTf.transform.translation.z, malletTf.header.stamp.to_sec()]))
            if abs(puckTf.transform.translation.y - self.set_puck_state.model_state.pose.position.y.real) < 1e-3:
                #puck is not moving
                if puckTf.header.stamp.to_sec() - begin.to_sec() > 5:
                    print("End because of Timeout")
                    end = True
            elif not (lastTf.header.stamp - puckTf.header.stamp).is_zero() and puckTf.header.stamp.to_sec() - begin.to_sec() > 1:
                vel = self.computeVel(lastTf, puckTf)
                if vel < 0.001:
                    print("End because of Velocity")
                    end = True
            if self.goal(puckTf.transform.translation):
                print("End because of GOOOAL")
                end = True
            lastTf = puckTf
            rospy.sleep(1/120)
        self.trajectory.append(tmp_traj)
        self.malletStates.append(tmp_mallet_traj)
        return end

    def testSmash(self, iterations, smashStrategy):
        set_tactic = SetTacticsServiceRequest()
        for i in range(iterations):
            print("Iteration: ", i)
            rospy.sleep(2)
            self.set_puck_smash()
            self.set_puck_state_service(self.set_puck_state)
            rospy.sleep(1)
            set_tactic.smashStrategy = smashStrategy
            self.lastStrategy = strategy
            set_tactic.tactic = 'SMASH'
            self.set_tactic_service(set_tactic)
            self.detectEnd()

    def testDefend(self, iterations, velocity= 1.):
        set_tactic = SetTacticsServiceRequest()
        for i in range(iterations):
            rospy.sleep(3)
            self.set_puck_defend(velocity)
            self.set_puck_state_service(self.set_puck_state)
            rospy.sleep(0.4)
            self.lastStrategy = strategy
            set_tactic.tactic = 'CUT'
            self.set_tactic_service(set_tactic)
            self.detectEnd()

    def testRepel(self, iterations, velocity=1.5):
        set_tactic = SetTacticsServiceRequest()
        for i in range(iterations):
            print("Iteration: ", i)
            rospy.sleep(3)
            self.set_puck_repel(velocity)
            set_tactic.tactic = 'REPEL'
            self.set_puck_state_service(self.set_puck_state)
            self.set_tactic_service(set_tactic)
            self.detectEnd()

    def testPrepare(self, iterations):
        set_tactic = SetTacticsServiceRequest()
        for i in range(iterations):
            print("Iteration: ", i)
            rospy.sleep(1)
            self.set_puck_prepare()
            self.set_puck_state_service(self.set_puck_state)
            rospy.sleep(1)
            set_tactic.tactic = 'PREPARE'
            self.set_tactic_service(set_tactic)
            self.detectEnd()



if __name__ == "__main__":
    rospy.init_node("air_hockey_strategy_test", anonymous=True)

    iterations = 50
    smashStrategy = 2

    testStrategy = TestStrategy()

    #Set strategy string for file and call test function
    strategy = 'PREPARE'
    testStrategy.testPrepare(iterations)
    print("END")

    logdir = os.path.join('logs/tests/', strategy, str(smashStrategy) + datetime.datetime.now().strftime("%d-%m-%Y-%H-%M"))
    if not os.path.exists(logdir):
        os.makedirs(logdir)

    np.save(logdir +"/log_trajectory", testStrategy.trajectory, allow_pickle=True)
    np.save(logdir +"/log_malletTrajectory", testStrategy.malletStates, allow_pickle=True)



