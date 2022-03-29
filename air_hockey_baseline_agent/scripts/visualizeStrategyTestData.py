import rosbag
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import numpy as np

def removeShortTraj(dataSet):
    cleanDataSet = []
    for data in dataSet:
        if len(data) > 100:
            begin = data[0]
            if begin[0] > -t_len/2 + 0.2 and begin[0] < -t_len/2 + 0.8 and abs(begin[1]) < t_w/2 - 0.1:
                cleanDataSet.append(data)
    return cleanDataSet

def goal(data, g_dist):
    flag = True
    for x, y, t in data:
        if x >= t_len/2 - 0.035:
            if flag:
                g_dist.append(y)
                flag = False
            if abs(y) <= g_w/2 - 0.035:
                return True
    return False

def robot_moves(malletStates):
    begin = malletStates[0][:3]
    for state in malletStates[1:]:
        diff = begin - state[:3]
        dist = np.linalg.norm(diff)
        if dist > 0.05:
            return True
    return False

def robot_hits_puck(puckState, malletStates):
    for i, state in enumerate(puckState):
        for mallet in malletStates:
            if abs(state[-1] - mallet[-1]) < 0.05:
                diff = state[:2] - mallet[:2]
                dist = np.linalg.norm(diff)
                if dist <= (0.049 + 0.032):
                    return True
    return False

def time_til_defendLine(p, param):
    for state in p:
        if state[0] <= param:
            dt = state[-1] - p[0][-1]
            return dt
    return np.nan

def vel_on_defendLine(puckState, defendline):
    for index, state in enumerate(puckState):
        if state[0] - defendline < 0.02:
            dt = puckState[index + 3][-1] - state[-1]
            dx = puckState[index + 3][0] - state[0]
            dy = puckState[index + 3][1] - state[1]
            vel = np.linalg.norm(np.array([dx, dy]))/dt
            if vel is None:
                return np.nan
            return vel
    return np.nan

def remove_from_both_by_select(puckState, malletStates, selects):
    n_puck = []
    n_mallet = []
    for i, sel in enumerate(selects):
        if sel:
            n_puck.append(puckState[i])
            n_mallet.append(malletStates[i])
    return n_puck, n_mallet


# 1: in eigener seite
# 2: mitte um geschlagen werden zu können
def smash_area(data):
    ret = 0
    for (x, y, t), (x1, y1, t1) in zip(data[:-1], data[1:]):
        dt = (t1 - t)
        dx = (x1 - x) / dt
        dy = (y1 - y) / dt
        vel = np.linalg.norm(np.asarray([dx, dy]))
        if vel < 0.05:
            #puck is very slow
            if x < 0:
                ret = 1
                if abs(y) <= t_w/2 - 0.2:
                    ret = 2
                    break
    return ret

def prep_smash_area(data):
    ret = 0
    for (x, y, t), (x1, y1, t1) in zip(data[:-1], data[1:]):
        dt = (t1 - t)
        dx = (x1 - x) / dt
        dy = (y1 - y) / dt
        vel = np.linalg.norm(np.asarray([dx, dy]))
        if vel < 0.05:
            #puck is very slow
            if x < 0:
                ret = 1
                if abs(y) <= t_w/2 - 0.2:
                    ret = 2
                    break
            else:
                ret = 0
                break
    return ret





def defend_analysis():
    print("_____CUT______")

    puck = np.load("logs/tests/CUT/17-02-2022-22-38/log_trajectory.npy", allow_pickle=True)
    jointState = np.load("logs/tests/CUT/17-02-2022-22-38/log_malletTrajectory.npy", allow_pickle=True)

    print(len(puck), len(jointState))
    print([len(d) for d in puck])
    goals = [goal(p, []) for p in puck]
    print(goals.count(True)/len(goals))
    moves = [robot_moves(d) for d in jointState]
    puck, jointState = remove_from_both_by_select(puck, jointState, moves)
    print(moves)
    goals = [goal(p, []) for p in puck]
    print(goals.count(True)/len(goals))
    hits = [robot_hits_puck(p, m) for p,m in zip(puck, jointState)]
    print(hits)
    puck, _ = remove_from_both_by_select(puck, jointState, hits)
    end = [smash_area(p) for p in puck]
    vels = np.array([vel_on_defendLine(p, -t_len/2 + 0.2) for p in puck])

    time = np.array([time_til_defendLine(p, -t_len/2 + 0.2) for p in puck])
    print(end)
    print(vels)
    print("puck hit ratio: ", hits.count(True)/len(hits))
    print("success rate smash area: ", end.count(2)/len(end))
    print("success rate own area: ", (end.count(1) + end.count(2))/len(end))
    print("velocities: ", np.nanmean(vels))
    print("std of vel: ", np.nanstd(vels))
    print("time: ", np.nanmean(time))
    print("std of time: ", np.nanstd(time))



def repel_analysis():
    print("_____REPEL______")

    puck = np.load("logs/tests/REPEL/18-02-2022-11-25/log_trajectory.npy", allow_pickle=True)
    jointState = np.load("logs/tests/REPEL/18-02-2022-11-25/log_malletTrajectory.npy", allow_pickle=True)

    print(len(puck), len(jointState))
    print([len(d) for d in puck])

    goals = [goal(p, []) for p in puck]
    print(goals.count(True)/len(goals))
    exit()
    moves = [robot_moves(d) for d in jointState]
    puck, jointState = remove_from_both_by_select(puck, jointState, moves)
    print(moves)
    hits = [robot_hits_puck(p, m) for p,m in zip(puck, jointState)]
    print(hits)
    puck, _ = remove_from_both_by_select(puck, jointState, hits)
    dist = []
    g = [goal(p, dist) for p in puck]
    vels = np.array([vel_on_defendLine(p[50:], t_len/2 - 0.2) for p in puck])
    print(vels)
    print("puck hit ratio: ", hits.count(True)/len(hits))
    print("success hit goal: ", g.count(True)/len(g))
    g_dist = np.asarray(dist)
    print(np.mean(g_dist), np.std(g_dist))
    print("velocities: ", np.nanmean(vels))
    print("std of vel: ", np.nanstd(vels))





def prepare_analysis():
    print("_____PREPARE______")

    puck = np.load("logs/tests/PREPARE/123-02-2022-18-25/log_trajectory.npy", allow_pickle=True)
    puck1 = np.load("logs/tests/PREPARE/121-02-2022-11-44/log_trajectory.npy", allow_pickle=True)
    puck = np.append(puck, puck1)
    jointState = np.load("logs/tests/PREPARE/123-02-2022-18-25/log_malletTrajectory.npy", allow_pickle=True)
    joint1 = np.load("logs/tests/PREPARE/121-02-2022-11-44/log_malletTrajectory.npy", allow_pickle=True)
    jointState = np.append(jointState, joint1)

    print(len(puck), len(jointState))
    print([len(d) for d in puck])
    moves = [robot_moves(d) for d in jointState]
    puck, jointState = remove_from_both_by_select(puck, jointState, moves)
    print(moves)
    hits = [robot_hits_puck(p, m) for p,m in zip(puck, jointState)]
    print(hits)
    puck, _ = remove_from_both_by_select(puck, jointState, hits)


    end = [prep_smash_area(p) for p in puck[:-10]]
    print(end)
    print("puck hit ratio: ", hits.count(True)/len(hits))
    print("success rate smash area: ", end.count(2)/len(end))
    print("success rate own area: ", (end.count(1) + end.count(2))/len(end))


if __name__ == "__main__":
    t_len = 1.956
    t_w = 1.042
    g_w = 0.25
    defend_analysis()
    #data = np.load("logs/tests/PREPARE/16-02-2022-14-23/log_trajectory.npy", allow_pickle=True)
    #smash = np.load("logs/tests/SMASH/012-02-2022-18-44.npy",allow_pickle=True)
    #smash1 = np.load("logs/tests/SMASH/113-02-2022-10-59.npy",allow_pickle=True)
    #smash2 = np.load("logs/tests/SMASH/213-02-2022-13-50.npy",allow_pickle=True)
    data = np.load("logs/tests/PREPARE/123-02-2022-18-25/log_trajectory.npy", allow_pickle=True)
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    for i in range(len(data)):
        ax.plot([d[0] for d in data[i]], [d[1] for d in data[i]],  color='orange')
    #for i in range(len(smash)):
    #    smash[i] = smash[i][:1]
    #    ax.plot([d[0] for d in smash[i]], [d[1] for d in smash[i]], '.', color='green')
    #for i in range(len(smash1)):
    #    smash1[i] = smash1[i][:1]
    #    ax.plot([d[0] for d in smash1[i]], [d[1] for d in smash1[i]], '.', color='green')
    #for i in range(len(jointState)):
    #    ax.plot([d[0] for d in jointState[i]], [d[1] for d in jointState[i]])
    ax.vlines(-t_len/2, -t_w/2, t_w/2)
    ax.vlines(t_len/2, -t_w/2, t_w/2)
    ax.vlines(-t_len/2, -g_w/2, g_w/2, colors='r')
    ax.vlines(t_len/2, -g_w/2, g_w/2, colors='r')
    ax.vlines(0, -t_w/2, t_w/2, alpha=0.3)
    ax.vlines(-t_len/2 + 0.2, -t_w/2, t_w/2, colors='c')
    ax.vlines(t_len/2 - 0.2, -t_w/2, t_w/2, colors='c')
    ax.hlines(-t_w/2, -t_len/2, t_len/2)
    ax.hlines(t_w/2, -t_len/2, t_len/2)
    ax.add_patch(
        patches.Rectangle(
            (-t_len/2 + 0.2, -t_w/2 + 0.1),
            0.6,
            t_w-0.2,
            edgecolor='b',
            facecolor='b',
            alpha=0.2,
            fill=True
        ))
    ax.add_patch(
        patches.Wedge(
            (-t_len/2, 0),
            t_len/2 - (0.156/2),
            0,
            360,
            alpha=0.5,
            color='orange',
            width=0.01
        ))
    plt.show()
