#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

model_q0 = {
    "FL_HAA": 0.0,
    "FR_HAA": 0.0,
    "HL_HAA": 0.0,
    "HR_HAA": 0.0,
    "FL_HFE": 0.4102,
    "FR_HFE": 0.4102,
    "HL_HFE": -0.6981,
    "HR_HFE": -0.6981,
    "FL_KFE": -1.2716,
    "FR_KFE": -1.2716,
    "HL_KFE": 1.676,
    "HR_KFE": 1.676,
    "HL_AFE": -1.7219,
    "HR_AFE": -1.7219,
}
action_scale = 0.5


class PolicyRelay:
    def __init__(self):
        self.pubs = {}  # topic -> Publisher
        self.name_to_idx = None  # joint_name -> index in JointState arrays
        rospy.Subscriber("/joint_targets_rl", JointState, self.cb, queue_size=1)

        time.sleep(0.5)
        for joint in model_q0.keys():
            topic = f"/{joint}_position_controller/command"
            if topic not in self.pubs:
                self.pubs[topic] = rospy.Publisher(topic, Float64, queue_size=1)

        # 3. On publie les cibles
        for joint, target_pos in model_q0.items():
            topic = f"/{joint}_position_controller/command"
            self.pubs[topic].publish(Float64(target_pos))
        rospy.loginfo("Posture initiale model_q0 envoyée aux contrôleurs PID.")

    def cb(self, msg: JointState):
        if (self.name_to_idx is None) or (len(self.name_to_idx) != len(msg.name)):
            self.name_to_idx = {n: i for i, n in enumerate(msg.name)}
            rospy.loginfo(
                "Relay: mapeados %d joints: %s",
                len(self.name_to_idx),
                ",".join(self.name_to_idx.keys()),
            )

        for jname, idx in self.name_to_idx.items():
            topic = f"/{jname}_position_controller/command"
            pub = self.pubs.get(topic)
            if pub is None:
                pub = rospy.Publisher(topic, Float64, queue_size=1)
                self.pubs[topic] = pub

            pos = msg.position[idx] if idx < len(msg.position) else 0.0

            pub.publish(pos)


if __name__ == "__main__":
    rospy.init_node("policy_to_controllers")
    PolicyRelay()
    rospy.loginfo("Relay policy→controllers active")
    rospy.spin()
