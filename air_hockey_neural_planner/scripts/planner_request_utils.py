import numpy as np
from air_hockey_neural_planner.msg import PlannerRequest


def unpack_planner_request(msg):
    q_0 = np.array(msg.q_0)
    q_dot_0 = np.array(msg.q_dot_0)
    q_ddot_0 = np.array(msg.q_ddot_0)
    end_point = msg.end_point
    x_d = end_point.x
    y_d = end_point.y
    th_d = msg.hitting_angle
    return x_d, y_d, th_d, q_0, q_dot_0, q_ddot_0
