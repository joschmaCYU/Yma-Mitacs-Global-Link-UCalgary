#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


MAP = {
    "FL_HAA": "/FL_HAA_position_controller/command",
    "FL_HFE": "/FL_HFE_position_controller/command",
    "FL_KFE": "/FL_KFE_position_controller/command",

    "FR_HAA": "/FR_HAA_position_controller/command",
    "FR_HFE": "/FR_HFE_position_controller/command",
    "FR_KFE": "/FR_KFE_position_controller/command",

    "HL_HAA": "/HL_HAA_position_controller/command",
    "HL_HFE": "/HL_HFE_position_controller/command",
    "HL_KFE": "/HL_KFE_position_controller/command",
    "HL_AFE": "/HL_AFE_position_controller/command",

    "HR_HAA": "/HR_HAA_position_controller/command",
    "HR_HFE": "/HR_HFE_position_controller/command",
    "HR_KFE": "/HR_KFE_position_controller/command",
    "HR_AFE": "/HR_AFE_position_controller/command",
}

class JointCommander:
    def __init__(self):
        rospy.init_node("joint_commander")


        self.input_topic = rospy.get_param("~input_topic", "/joint_targets_selected")


        self.pubs = {name: rospy.Publisher(topic, Float64, queue_size=10)
                     for name, topic in MAP.items()}

        rospy.Subscriber(self.input_topic, JointState, self.cb)
        rospy.loginfo("joint_commander started (listening to %s)", self.input_topic)
        rospy.spin()

    def cb(self, msg: JointState):

        idx = {n: i for i, n in enumerate(msg.name)}


        missing = []
        for jname, pub in self.pubs.items():
            i = idx.get(jname, None)
            if i is None or i >= len(msg.position):
                missing.append(jname)
                continue
            pub.publish(Float64(msg.position[i]))

        if missing:
            rospy.logwarn_throttle(2.0, "joint_commander: missing values for %s", missing)

if __name__ == "__main__":
    JointCommander()
