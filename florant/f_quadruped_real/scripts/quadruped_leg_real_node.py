#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Project Name : ContinuO - Quadruped Robot
Description : This node controls one leg of the quadruped in real hardware, by subscribing to joint target commands and sending position setpoints to the motors via CAN bus.
Author : Florent Pralong
Date of creation : 10/06/2026
Version : 1.0
"""

# ---------------------------
# IMPORTS
# ---------------------------

from cProfile import label
import os
import time
import threading
import rospy
import myactuator_rmd_py as rmd
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class QuadrupedLegRealNode:

    # ---------------------------
    # INITIALIZATION
    # ---------------------------

    def __init__(self):
        rospy.init_node("quadruped_leg_real_node")

        # ---------------------------
        # PARAMETERS
        # ---------------------------

        # Indicate if a send cycle take too much time
        self.step_err_no = 0
        self.infoMaxTime = rospy.get_param("~infoMaxTime", 0.1)

        # Legs definition
        self.legs = {
            "front_right": [5, 3, 1],
            "front_left": [6, 4, 2],
            "hind_right": [17, 15, 13, 11],
            "hind_left": [16, 14, 12, 10],
        }

        self.leg_can = {
            "front_right": "can0",
            "front_left": "can1",
            "hind_right": "can2",
            "hind_left": "can3",
        }

        self.joint_to_motor = {
            "front_right": {
                "FR_HAA": 1,
                "FR_HFE": 3,
                "FR_KFE": 5,
            },
            "front_left": {
                "FL_HAA": 2,
                "FL_HFE": 4,
                "FL_KFE": 6,
            },
            "hind_right": {
                "HR_HAA": 11,
                "HR_HFE": 13,
                "HR_KFE": 15,
                "HR_AFE": 17,
            },
            "hind_left": {
                "HL_HAA": 10,
                "HL_HFE": 12,
                "HL_KFE": 14,
                "HL_AFE": 16,
            },
        }

        #--------------------------
        # Configuration parameters (in the launch file)

        # Parameter for selecting the active leg
        self.active_leg = rospy.get_param("~active_leg", "hind_right")

        if self.active_leg not in self.legs:
            raise RuntimeError("Unknown active_leg: {}".format(self.active_leg))
        
        self.inter_motor_delay = rospy.get_param("~inter_motor_delay", 0.02)
        self.command_speed = rospy.get_param("~command_speed", 30)
        self.command_accel = int(rospy.get_param("~command_accel", 2000))
        self.command_decel = int(rospy.get_param("~command_decel", 2000))
        self.dry_run = rospy.get_param("~dry_run", True)
        self.inv_motor_command = rospy.get_param("~inv_motor_command", False)
        self.default_max_amplitude_deg = rospy.get_param("~default_max_amplitude_deg", 20.0)
        self.max_amplitude_deg = rospy.get_param("~max_amplitude_deg", {})

        # ---------------------------
        # Other parameters
        
        self.can_target = self.leg_can[self.active_leg]
        self.motor_IDs = self.legs[self.active_leg]
        self.target = {motor_id: 0.0 for motor_id in self.motor_IDs}
        self.motor = {}
        self.can = {}
        self.startup_pos = {}
        self.target_lock = threading.Lock()
        self.received_first_target = False
        self.emergency_stop = False
        self.latest_target_stamp = None
        self.max_target_age = rospy.get_param("~max_target_age", 1.0)
        self.last_sent_target = {motor_id: None for motor_id in self.motor_IDs}
        self.min_target_delta_deg = rospy.get_param("~min_target_delta_deg", 1.0)

        # Needed positions definition
        self.init_pos = {
            1: 0,
            2: 0,
            3: -66.5,
            4: 66.5,
            5: 107.1,
            6: -107.1,
            10: 0,
            11: 0,
            12: -50.1,
            13: 50.1,
            14: 84,
            15: -84,
            16: -81.3,
            17: 81.3
        }

        self.offsets_startup_pos = {
            1: 0,
            2: 0,
            3: -90,
            4: 90,
            5: 180,
            6: -180,
            10: 0,
            11: 0,
            12: -90,
            13: 90,
            14: 180,
            15: -180,
            16: -180,
            17: 180
        }

        # ---------------------------
        # INITIALIZATION
        # ---------------------------

        # 01-CAN bus and motors initialization
        self.init_canports()
        self.motor, self.motor_IDs, self.can = self.init_motors(self.active_leg)
        self.check_targets(self.motor_IDs, self.target)
        self.pos_accel = rmd.actuator_state.AccelerationType(0)
        self.pos_decel = rmd.actuator_state.AccelerationType(1)

        # 02-Motors startup: read initial position and set accelerations
        for motor_id in self.motor_IDs:
            
            time.sleep(self.inter_motor_delay)

            # Reading actual motor position (onTheGroundRef)
            rospy.loginfo("Reading motor %s", motor_id)

            self.startup_pos[motor_id] = self.read_stable_startup_position(
                self.motor,
                motor_id
            )

            rospy.loginfo("OK motor %s startup_pos=%s", motor_id, self.startup_pos[motor_id])

            # Set acceleration defined in 01
            self.init_retry_call(
                "Set acceleration motor {}".format(motor_id),
                self.set_motor_acceleration,
                self.motor,
                motor_id,
                self.command_accel,
                self.pos_accel
            )

            rospy.loginfo("OK motor %s accel=%s", motor_id, self.command_accel)

            # Set deceleration defined in 01
            self.init_retry_call(
                "Set deceleration motor {}".format(motor_id),
                self.set_motor_acceleration,
                self.motor,
                motor_id,
                self.command_decel,
                self.pos_decel
            )

            rospy.loginfo("OK motor %s decel=%s", motor_id, self.command_decel)

        # 03-Move motors to initial position before starting to listen to commands
        
        self.move_to_initial_position()

        # 04-Wait to ensure motors are in position 

        time.sleep(10.0)

        # Apply offsets between real startup position (onTheGroundRef) and control policy startup position (isaacLabRef)

        self.apply_startup_position_offsets()

        # ---------------------------
        # ROS Interface
        # ---------------------------
        # Publisher

        self.joint_state_pub = rospy.Publisher(
            "/joint_states",
            JointState,
            queue_size=1
        )

        self.motor_to_joint = {}

        for joint_name, motor_id in self.joint_to_motor[self.active_leg].items():
            self.motor_to_joint[motor_id] = joint_name

        self.last_joint_position = {motor_id: 0.0 for motor_id in self.motor_IDs}
        self.last_joint_velocity = {motor_id: 0.0 for motor_id in self.motor_IDs}

        # Subscriber

        rospy.Subscriber(
            "/joint_targets_rl",
            JointState,
            self.joint_targets_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/control_tick",
            Header,
            self.control_tick_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/emergency_stop",
            Bool,
            self.emergency_stop_callback,
            queue_size=1
        )

        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "quadruped_leg_real_node ready: leg=%s, can=%s, motors=%s",
            self.active_leg,
            self.can_target,
            self.motor_IDs
        )

    # ---------------------------
    # ROS CALLBACK FUNCTIONS
    # ---------------------------

    # Define what happen when a change is detected on the /joint_targets_rl topic
    def joint_targets_callback(self, msg):
        name_to_position = dict(zip(msg.name, msg.position))

        new_target = {}

        for joint_name, motor_id in self.joint_to_motor[self.active_leg].items():
            #Convert from radians to degrees, and apply inversion if needed
            if joint_name in name_to_position:
                    
                target_deg = round(name_to_position[joint_name] * 180.0 / 3.141592653589793)

                # Apply inversion on motors 10 and 1 (error on the usd ?)
                if motor_id == 10 or motor_id == 1:
                    target_deg = -target_deg

                # Apply inversion if needed
                if self.inv_motor_command:
                    target_deg = -target_deg

                # Limit motion around startup position: [-max_amp, +max_amp]
                target_deg = self.clamp_motor_target(motor_id, target_deg)

                new_target[motor_id] = target_deg

        with self.target_lock:
            for motor_id, target_value in new_target.items():
                self.target[motor_id] = target_value

            if new_target:
                self.received_first_target = True
                self.latest_target_stamp = rospy.Time.now()

    # Define what happen when a change is detected on the /emergency_stop topic
    def emergency_stop_callback(self, msg):
        if not msg.data:
            return

        if self.emergency_stop:
            return

        self.emergency_stop = True

        rospy.logfatal("%s: EMERGENCY STOP RECEIVED", self.active_leg)

        # Stop all motors when an emergency stop is published
        for motor_id in self.motor_IDs:
            try:
                rospy.logfatal("%s: stopping motor %s", self.active_leg, motor_id)
                self.motor[motor_id].stopMotor()
                time.sleep(self.inter_motor_delay)
            except Exception as exc:
                rospy.logerr(
                    "%s: failed to stop motor %s during emergency stop: %s",
                    self.active_leg,
                    motor_id,
                    exc
                )

    # Define what happen when a change is detected on the /control_tick topic
    def control_tick_callback(self, msg):
        if self.emergency_stop:
            return

        if not self.received_first_target:
            return

        with self.target_lock:
            target_copy = dict(self.target)
            target_stamp = self.latest_target_stamp

        if target_stamp is None:
            return

        # Calcul the age of the target and send a warning if it is > max age
        age = (rospy.Time.now() - target_stamp).to_sec()

        if age > self.max_target_age:
            rospy.logwarn_throttle(
                1.0,
                "%s: target too old: %.3f s",
                self.active_leg,
                age
            )
            return

        t1 = time.perf_counter()

        # Positions update
        for motor_id in self.motor_IDs:
            
            time.sleep(self.inter_motor_delay)

            absolute_target = self.startup_pos[motor_id] + target_copy[motor_id]

            last = self.last_sent_target[motor_id]

            if last is None:
                calculated_speed = self.command_speed
            else:
                if abs(absolute_target - last) < self.min_target_delta_deg:
                    continue

                calculated_speed = self.command_speed

            self.last_sent_target[motor_id] = absolute_target

            # [Dry] run execut all the code except the CAN send command
            if self.dry_run:
                rospy.loginfo(
                    "[DRY RUN] tick=%d leg=%s motor=%d startup=%.3f target=%.3f absolute=%.3f speed=%d",
                    msg.seq,
                    self.active_leg,
                    motor_id,
                    self.startup_pos[motor_id],
                    target_copy[motor_id],
                    absolute_target,
                    calculated_speed
                )

            #[Real dry]
            else:
                self.retry_call(
                    "Send position {} motor {}".format(self.active_leg, motor_id),
                    self.send_position,
                    self.motor,
                    motor_id,
                    absolute_target,
                    calculated_speed
                )

                rospy.loginfo(
                    "[REAL RUN] tick=%d leg=%s motor=%d startup=%.3f target=%.3f absolute=%.3f speed=%d",
                    msg.seq,
                    self.active_leg,
                    motor_id,
                    self.startup_pos[motor_id],
                    target_copy[motor_id],
                    absolute_target,
                    calculated_speed
                )


        # Positions read
        for motor_id in self.motor_IDs:
            try:
                raw_pos_deg = self.retry_call(
                    "Read position {} motor {}".format(self.active_leg, motor_id),
                    self.read_motor_position,
                    self.motor,
                    motor_id
                )

                raw_vel_rpm = self.retry_call(
                    "Read velocity {} motor {}".format(self.active_leg, motor_id),
                    self.read_motor_velocity,
                    self.motor,
                    motor_id
                )

                joint_pos_deg = raw_pos_deg - self.startup_pos[motor_id]

                self.last_joint_position[motor_id] = self.deg_to_rad(joint_pos_deg)
                self.last_joint_velocity[motor_id] = self.rpm_to_rad_s(raw_vel_rpm)

            except Exception as exc:
                rospy.logwarn(
                    "%s: failed to read joint state motor %s after command: %s",
                    self.active_leg,
                    motor_id,
                    exc
                )
                
        # Publish the readen positions
        self.publish_joint_states(rospy.Time.now())

        # Monitor the cycle time
        dt = time.perf_counter() - t1

        if dt >= self.infoMaxTime:
            self.step_err_no += 1
            rospy.logwarn(
                "%s: execution longer than intended timestep: %.4f s, count=%d",
                self.active_leg,
                dt,
                self.step_err_no
            )

 
    # ---------------------------
    # HARDWARE FUNCTIONS
    # ---------------------------

    # CAN initialization
    def init_canports(self):
        rospy.loginfo("Initializing CAN interface %s", self.can_target)

        os.system("sudo ip link set {} down".format(self.can_target))
        time.sleep(0.1)

        os.system("sudo ip link set {} up type can bitrate 1000000".format(self.can_target))
        time.sleep(0.1)

    # Motors initialization
    def init_motors(self, active_leg):
        motor = {}
        can = {}

        can_name = self.leg_can[active_leg]
        can[active_leg] = rmd.CanDriver(can_name)

        for motor_id in self.legs[active_leg]:
            motor[motor_id] = rmd.ActuatorInterface(can[active_leg], motor_id)

        return motor, self.legs[active_leg], can

    # Check if a target is defined for each motor
    def check_targets(self, active_motor_ids, target):
        for motor_id in active_motor_ids:
            if motor_id not in target:
                raise RuntimeError("No target defined for motor {}".format(motor_id))

    # Send a CAN command and retry if the request fail
    def retry_call(self, label, func, *args, max_attempts=2, delay=0.001):
        for attempt in range(1, max_attempts + 1):
            try:
                return func(*args)

            except rmd.can.BusError:
                rospy.logerr("%s: BusError attempt %d/%d", label, attempt, max_attempts)
                time.sleep(delay)

            except rmd.ProtocolException as exc:
                rospy.logerr(
                    "%s: ProtocolException attempt %d/%d: %s",
                    label,
                    attempt,
                    max_attempts,
                    exc
                )
                time.sleep(delay)

        raise RuntimeError("{}: failed after {} attempts".format(label, max_attempts))

    # Same as above but parameters are different
    def init_retry_call(self, label, func, *args, max_attempts=8, delay=0.5):
        try:    
            return self.retry_call(
                label,
                func,
                *args,
                max_attempts=max_attempts,
                delay=delay
            )
        except RuntimeError:
            self.fatal_init_error(label)

    # Read the positions of the motors and check if the position readed is stable
    def read_stable_startup_position(self, motor, motor_id, samples=3, max_spread_deg=4.0):
        values = []

        for i in range(samples):
            time.sleep(self.inter_motor_delay)

            value = self.init_retry_call(
                "Read startup position motor {} sample {}".format(motor_id, i + 1),
                self.read_motor_position,
                motor,
                motor_id
            )
            values.append(value)
        

        spread = max(values) - min(values)

        if spread > max_spread_deg:
            self.fatal_init_error(
                "Motor {} unstable startup position readings: {}".format(motor_id, values)
            )

        return sum(values) / len(values)
    
    # Apply the offset between real startup position (onTheGroundRef) and control policy startup position (isaacLabRef)
    def apply_startup_position_offsets(self):
        rospy.loginfo("%s: applying startup position offsets", self.active_leg)

        for motor_id in self.motor_IDs:

            if motor_id not in self.offsets_startup_pos:
                rospy.logwarn(
                    "%s: no startup offset defined for motor %s",
                    self.active_leg,
                    motor_id
                )
                continue

            old_pos = self.startup_pos[motor_id]
            offset = self.offsets_startup_pos[motor_id]

            self.startup_pos[motor_id] = old_pos + offset

            rospy.loginfo(
                "%s: motor %d startup_pos %.3f -> %.3f (offset %+0.3f)",
                self.active_leg,
                motor_id,
                old_pos,
                self.startup_pos[motor_id],
                offset
            )

    # Stop the node in case of an error during the init
    def fatal_init_error(self, message):
        rospy.logfatal("%s: FATAL INIT ERROR: %s", self.active_leg, message)

        try:
            rospy.logfatal("%s: stopping motors before shutdown", self.active_leg)
            for motor_id in self.motor_IDs:
                try:
                    self.motor[motor_id].stopMotor()
                    time.sleep(self.inter_motor_delay)
                except Exception as exc:
                    rospy.logerr(
                        "%s: failed to stop motor %s during fatal error: %s",
                        self.active_leg,
                        motor_id,
                        exc
                    )
        finally:
            raise RuntimeError("{}: {}".format(self.active_leg, message))

    # Read the startup position (called in read_stable_startup_position)
    def read_startup_position(self, motor, motor_id):
        return motor[motor_id].getMultiTurnAngle()

    # Set acceleration and deceration during the init
    def set_motor_acceleration(self, motor, motor_id, value, accel_type):
        return motor[motor_id].setAcceleration(value, accel_type)

    # Send a position to a motor
    def send_position(self, motor, motor_id, target, speed):
        return motor[motor_id].sendPositionAbsoluteSetpoint(target, speed) 
    
    # Move to init position during the initialization
    def move_to_initial_position(self):
        rospy.loginfo("%s: moving to initial position", self.active_leg)

        for motor_id in self.motor_IDs:
            time.sleep(self.inter_motor_delay)

            if motor_id not in self.init_pos:
                rospy.logwarn("%s: no init position defined for motor %s", self.active_leg, motor_id)
                continue

            absolute_target = self.startup_pos[motor_id] + self.init_pos[motor_id]

            if self.dry_run:
                rospy.loginfo(
                    "[DRY RUN INIT] leg=%s motor=%d startup=%.3f init_offset=%.3f absolute=%.3f speed=%d",
                    self.active_leg,
                    motor_id,
                    self.startup_pos[motor_id],
                    self.init_pos[motor_id],
                    absolute_target,
                    self.command_speed
                )
            else:
                self.init_retry_call(
                    "Move to init position {} motor {}".format(self.active_leg, motor_id),
                    self.send_position,
                    self.motor,
                    motor_id,
                    absolute_target,
                    self.command_speed
                )

    # Check that the target in the defined range
    def clamp_motor_target(self, motor_id, target_deg):
        motor_key_str = str(motor_id)

        if motor_key_str in self.max_amplitude_deg:
            max_amp = self.max_amplitude_deg[motor_key_str]

        elif motor_id in self.max_amplitude_deg:
            max_amp = self.max_amplitude_deg[motor_id]

        else:
            max_amp = self.default_max_amplitude_deg

        max_amp = abs(float(max_amp))

        if target_deg > max_amp:
            rospy.logwarn_throttle(
                1.0,
                "%s motor %s target %.2f deg limited to %.2f deg",
                self.active_leg,
                motor_id,
                target_deg,
                max_amp
            )
            return max_amp

        if target_deg < -max_amp:
            rospy.logwarn_throttle(
                1.0,
                "%s motor %s target %.2f deg limited to %.2f deg",
                self.active_leg,
                motor_id,
                target_deg,
                -max_amp
            )
            return -max_amp

        return target_deg
    
    # Maths functions
    def deg_to_rad(self, value_deg):
        return value_deg * 3.141592653589793 / 180.0
    
    def rpm_to_rad_s(self, value_rpm):
        return value_rpm * 2.0 * 3.141592653589793 / 60.0


    # Read a position (why the same as read_startuo_pos!!!!!!!!!!)
    def read_motor_position(self, motor, motor_id):
        return motor[motor_id].getMultiTurnAngle()

    # Read motor velocity
    def read_motor_velocity(self, motor, motor_id):
        return motor[motor_id].getMotorStatus2().shaft_speed

    # Publish positions on /joint_states
    def publish_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = ""

        for motor_id in self.motor_IDs:
            joint_name = self.motor_to_joint[motor_id]

            msg.name.append(joint_name)
            msg.position.append(self.last_joint_position[motor_id])
            msg.velocity.append(self.last_joint_velocity[motor_id])

        self.joint_state_pub.publish(msg)


    # ---------------------------
    # MAIN LOOP
    # ---------------------------   

    def run(self):
          rospy.spin()

        
    # ---------------------------
    # SHUTDOWN / FINALLY
    # ---------------------------

    def shutdown(self):
        rospy.loginfo("Shutdown %s: resetting accelerations", self.active_leg)

        time.sleep(2)

        # Reset acceleration and decelaration
        for motor_id in self.motor_IDs:
            time.sleep(self.inter_motor_delay)

            try:
                self.init_retry_call(
                    "Reset acceleration motor {}".format(motor_id),
                    self.set_motor_acceleration,
                    self.motor,
                    motor_id,
                    10000,
                    self.pos_accel
                )

                self.init_retry_call(
                    "Reset deceleration motor {}".format(motor_id),
                    self.set_motor_acceleration,
                    self.motor,
                    motor_id,
                    10000,
                    self.pos_decel
                )

            except Exception as exc:
                rospy.logerr("Shutdown failed for motor %s: %s", motor_id, exc)

# ---------------------------
# Entrypoint and node startup
# ---------------------------

if __name__ == "__main__":
    try:
        node = QuadrupedLegRealNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
