#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel:
    def __init__(self):
        rospy.init_node("teleop_joy_cmdvel")

        # ---------------- Parameters ----------------
        # Axis indices (according to the controller)
        self.axis_vx   = rospy.get_param("~axis_vx", 1)   # Left stick vertical (forward/backward)
        self.axis_vy   = rospy.get_param("~axis_vy", 0)   # Left stick horizontal (lateral)
        self.axis_yaw  = rospy.get_param("~axis_yaw", 3)  # Left stick horizontal (lateral)

        # Invert sign per axis (fixes cases like “forward = negative”, etc.)
        self.invert_vx  = rospy.get_param("~invert_vx", True)   
        self.invert_vy  = rospy.get_param("~invert_vy", False)
        self.invert_yaw = rospy.get_param("~invert_yaw", False)

        # Deadzone
        self.scale_vx   = rospy.get_param("~scale_vx", 0.3)   # m/s
        self.scale_vy   = rospy.get_param("~scale_vy", 0.2)   # m/s
        self.scale_yaw  = rospy.get_param("~scale_yaw", 0.8)  # rad/s
        self.deadzone   = rospy.get_param("~deadzone", 0.15)

        # Enable button (deadman). -1 = always enabled
        self.enable_button = rospy.get_param("~enable_button", -1)

        # Publishers / Subscribers
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/joy", Joy, self.cb_joy)

        rospy.loginfo("teleop_joy_cmdvel listo → /cmd_vel | axes(vx=%d, vy=%d, yaw=%d) invert(vx=%s, vy=%s, yaw=%s)",
                      self.axis_vx, self.axis_vy, self.axis_yaw,
                      self.invert_vx, self.invert_vy, self.invert_yaw)

        rospy.spin()

    def _deadzone(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def cb_joy(self, msg: Joy):
        # Deadman button if configured

        if self.enable_button >= 0:
            ok = (self.enable_button < len(msg.buttons) and msg.buttons[self.enable_button] == 1)
            if not ok:
                self.pub.publish(Twist())  
                return

        # Read joystick raw axis values

        ax_vx  = msg.axes[self.axis_vx]  if self.axis_vx  < len(msg.axes) else 0.0
        ax_vy  = msg.axes[self.axis_vy]  if self.axis_vy  < len(msg.axes) else 0.0
        ax_yaw = msg.axes[self.axis_yaw] if self.axis_yaw < len(msg.axes) else 0.0

        # Deadzone
        ax_vx  = self._deadzone(ax_vx)
        ax_vy  = self._deadzone(ax_vy)
        ax_yaw = self._deadzone(ax_yaw)

        # Invert if applicable
        if self.invert_vx:   ax_vx  = -ax_vx
        if self.invert_vy:   ax_vy  = -ax_vy
        if self.invert_yaw:  ax_yaw = -ax_yaw

        # Scale to physical units
        tw = Twist()
        tw.linear.x  = ax_vx  * self.scale_vx
        tw.linear.y  = ax_vy  * self.scale_vy
        tw.angular.z = ax_yaw * self.scale_yaw

        self.pub.publish(tw)
        rospy.loginfo_throttle(1.0, "/cmd_vel → vx=%.2f  vy=%.2f  yaw=%.2f", tw.linear.x, tw.linear.y, tw.angular.z)

if __name__ == "__main__":
    try:
        JoyToCmdVel()
    except rospy.ROSInterruptException:
        pass
