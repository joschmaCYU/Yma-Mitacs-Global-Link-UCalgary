#!/usr/bin/env python3
import rospy, time
from std_msgs.msg import Float64MultiArray

N = int(rospy.get_param("~N", 12))                
AMP = float(rospy.get_param("~amp", 0.08))         
T_ON = float(rospy.get_param("~t_on", 1.0))        
T_OFF = float(rospy.get_param("~t_off", 1.0))      
TOPIC = rospy.get_param("~topic", "/policy_action")
RATE = float(rospy.get_param("~rate", 100.0))

rospy.init_node("index_probe")
pub = rospy.Publisher(TOPIC, Float64MultiArray, queue_size=10)
rate = rospy.Rate(RATE)

def send(vec):
    pub.publish(Float64MultiArray(data=vec))

base = [0.0]*N
time.sleep(0.5); send(base)

for i in range(N):
    rospy.logwarn("PROBING INDEX %d" % i)
    v = base[:]; v[i] = AMP
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < T_ON and not rospy.is_shutdown():
        send(v); rate.sleep()
    t0 = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - t0 < T_OFF and not rospy.is_shutdown():
        send(base); rate.sleep()
