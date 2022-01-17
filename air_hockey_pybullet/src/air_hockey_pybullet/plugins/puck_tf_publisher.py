import geometry_msgs.msg
import numpy as np
import rospy
import tf2_ros
from air_hockey_pybullet.srv import ResetPuck, ResetPuckRequest, ResetPuckResponse


class PuckTfPublisher:
    def __init__(self, pybullet, namespace, model_spec, **kwargs):
        self.pb = pybullet
        self.namespace = namespace
        self.model_id = model_spec['model_id']

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()
        self.tf_msg.header.frame_id = 'world'
        self.tf_msg.child_frame_id = 'Puck'

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.reset_puck_server = rospy.Service(namespace + "/reset_puck_state", ResetPuck,
                                               self.reset_puck_cb)

    def execute(self, sim_time):
        self.tf_msg.header.stamp = sim_time
        state = self.pb.getLinkState(self.model_id, 0)
        self.tf_msg.transform.translation.x = state[0][0]
        self.tf_msg.transform.translation.y = state[0][1]
        self.tf_msg.transform.translation.z = state[0][2]
        self.tf_msg.transform.rotation.x = state[1][0]
        self.tf_msg.transform.rotation.y = state[1][1]
        self.tf_msg.transform.rotation.z = state[1][2]
        self.tf_msg.transform.rotation.w = state[1][3]

        self.br.sendTransform(self.tf_msg)

    def reset_puck_cb(self, req: ResetPuckRequest):
        world2parent_pos = np.array([0., 0., 0.])
        world2parent_quat = np.array([0., 0., 0., 1.])
        if req.transform.header.frame_id != '':

            world2parent = self.tf_buffer.lookup_transform('world', req.transform.header.frame_id, rospy.Time(0))
            world2parent_pos = np.array([world2parent.transform.translation.x,
                                         world2parent.transform.translation.y,
                                         world2parent.transform.translation.z])
            world2parent_quat = np.array([world2parent.transform.rotation.x,
                                          world2parent.transform.rotation.y,
                                          world2parent.transform.rotation.z,
                                          world2parent.transform.rotation.w])

        parent2puck_pos = np.array([req.transform.transform.translation.x,
                                    req.transform.transform.translation.y,
                                    req.transform.transform.translation.z])
        parent2puck_quat = np.array([req.transform.transform.rotation.x,
                                     req.transform.transform.rotation.y,
                                     req.transform.transform.rotation.z,
                                     req.transform.transform.rotation.w])

        self.pb.resetBasePositionAndOrientation(self.model_id,
                                                *self.pb.multiplyTransforms(world2parent_pos, world2parent_quat,
                                                                            parent2puck_pos, parent2puck_quat))
        return ResetPuckResponse()
