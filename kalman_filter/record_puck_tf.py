import rospy
import tf
import numpy as np
import sys, termios, tty, select
import os
from kalman_filter.utils import makedirs

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def info():
    print("=========================\n"
          "| Data Recording Tool   |\n"
          "| [r]: Start recording  |\n"
          "| [s]: Stop recording   |\n"
          "| [esc]: Close          |\n"
          "=========================")


def record():
    rate = rospy.Rate(120.0)

    data = []
    record = False
    has_data = False

    info()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    try:
        tty.setcbreak(sys.stdin.fileno())

        while not rospy.is_shutdown():
            if record:
                try:
                    (trans_puck, rot_puck) = listener.lookupTransform("/Puck", "/world", rospy.Time(0))
                    (trans_mallet, rot_mallet) = listener.lookupTransform("/Mallet", "/world", rospy.Time(0))
                    data.append(trans_puck + rot_puck + trans_mallet + rot_mallet)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("can not look up transform")
                    continue

            if isData():
                c = sys.stdin.read(1)
                if c == "r":
                    print("Start recording data")
                    record = True
                    has_data = False
                    data = []

                elif c == "s" and record:
                    record = False
                    has_data = True
                    print("Stop decording data")
                    print("Whether to save data of length {}? [y]es / [n]o".format(len(data)))

                elif c == "y" and has_data:
                    i = 0
                    while os.path.exists(os.path.join(directory, "data_{}.npy".format(i))):
                        i += 1

                    file_name = "data_{}.npy".format(i)
                    file_dir = os.path.join(directory, file_name)
                    print("Data saved in {}".format(file_dir))

                    np.save(file_dir, data)

                    has_data = False
                    data = []
                    info()

                elif c == "n" and has_data:
                    print("Data dropped")
                    info()
                    data = []
                    has_data = False

                elif c == chr(27):  # x1b is ESC
                    print("Stop")
                    break

                else:
                    print("Unknown input")

            rate.sleep()
    finally:
        print("Closing all data recorders")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == "__main__":
    directory = os.path.abspath(__file__ + "/../data")
    data_type = "mallet"
    directory = os.path.join(directory, data_type)
    makedirs(directory)

    rospy.init_node("puck_tf_record")

    listener = tf.TransformListener()

    record()
