import tf2_ros
import geometry_msgs.msg


class PuckTfPublisher:
    def __init__(self, pybullet, namespace, model_spec, **kwargs):
        self.pb = pybullet
        self.namespace = namespace
        self.model_id = model_spec['model_id']

        self.br = tf2_ros.TransformBroadcaster()
        self.tf_msg = geometry_msgs.msg.TransformStamped()
        self.tf_msg.header.frame_id = 'world'
        self.tf_msg.child_frame_id = 'Puck'

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
