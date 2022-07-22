import numpy as np
from air_hockey_neural_planner.msg import PlannerRequest


def unpack_planner_request(msg):
    q_0 = np.array(msg.q_0)
    q_dot_0 = np.array(msg.q_dot_0)
    q_ddot_0 = np.array(msg.q_ddot_0)
    hit_point = msg.hit_point
    x_hit = hit_point.x
    y_hit = hit_point.y
    end_point = msg.end_point
    x_end = end_point.x
    y_end = end_point.y
    th_hit = msg.hit_angle
    expected_time = msg.expected_time
    tactic = msg.tactic
    return tactic, x_hit, y_hit, th_hit, q_0, q_dot_0, q_ddot_0, x_end, y_end, expected_time
