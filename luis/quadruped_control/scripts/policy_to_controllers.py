#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class PolicyRelay:
    def __init__(self):
        self.pubs = {}          # topic -> Publisher
        self.name_to_idx = None # joint_name -> index in JointState arrays
        rospy.Subscriber('/joint_targets_rl', JointState, self.cb, queue_size=1)

    def cb(self, msg: JointState):

        if (self.name_to_idx is None) or (len(self.name_to_idx) != len(msg.name)):
            self.name_to_idx = {n: i for i, n in enumerate(msg.name)}
            rospy.loginfo("Relay: mapeados %d joints: %s", len(self.name_to_idx), ','.join(self.name_to_idx.keys()))


        for jname, idx in self.name_to_idx.items():
            topic = f'/{jname}_position_controller/command'
            pub = self.pubs.get(topic)
            if pub is None:
                pub = rospy.Publisher(topic, Float64, queue_size=1)
                self.pubs[topic] = pub

            pos = msg.position[idx] if idx < len(msg.position) else 0.0
            pub.publish(pos)

if __name__ == '__main__':
    rospy.init_node('policy_to_controllers')
    PolicyRelay()
    rospy.loginfo("Relay policy→controllers active")
    rospy.spin()

