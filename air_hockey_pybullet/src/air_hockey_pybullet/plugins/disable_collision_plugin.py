import rospy
from pybullet_ros.plugins.post_processor import PostProcessor


class AvoidCollsionProcessor(PostProcessor):
    def load(self):
        # Puck collision: Group: 11, Mask: 11
        self.set_collision_model(self.models['puck']['model_id'], group=3, mask=3)
        # Robot collision: Group 10, Mask: 01. Disable self collision
        self.set_collision_model(self.models['iiwa_front']['model_id'], group=2, mask=1)
        self.set_collision_model(self.models['iiwa_back']['model_id'], group=2, mask=1)
        # Table collision: Group 01, Mask: 01
        self.set_collision_model(self.models['air_hockey_table']['model_id'], group=2, mask=1)

    def set_collision_model(self, model_id, group, mask):
        for i in range(self.pb.getNumJoints(model_id)):
            self.pb.setCollisionFilterGroupMask(model_id, i, group, mask)
