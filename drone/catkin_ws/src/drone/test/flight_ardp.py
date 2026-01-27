#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
import math
from collections import deque

import rospy
from sensor_msgs.msg import Range
from mavros_msgs.msg import State, StatusText, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Quaternion


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def quat_identity():
    # level attitude, yaw = 0
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


class FlightNoCamRangeArduPilot:
    """
    ArduCopter + MAVROS:
      - Mode: GUIDED_NOGPS (default) or GUIDED
      - Control: /mavros/setpoint_raw/attitude (AttitudeTarget), only thrust is used
      - Altitude feedback: /rangefinder/range (sensor_msgs/Range)
    """

    def __init__(self, args):
        self.args = args
        self.state = State()
        self.range_m = None
        self.range_ts = 0.0
        self.last_statustext = deque(maxlen=50)

        self.pub_att = rospy.Publisher(
            "/mavros/setpoint_raw/attitude",
            AttitudeTarget,
            queue_size=20
        )

        rospy.Subscriber("/mavros/state", State, self.cb_state)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.cb_statustext)
        rospy.Subscriber("/rangefinder/range", Range, self.cb_range)

        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")

        self.srv_arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.srv_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # ---------------- callbacks ----------------

    def cb_state(self, msg):
        self.state = msg

    def cb_statustext(self, msg):
        self.last_statustext.append(f"[{msg.severity}] {msg.text}")

    def cb_range(self, msg):
        if not math.isnan(msg.range) and not math.isinf(msg.range):
            self.range_m = msg.range
            self.range_ts = time.time()

    # ---------------- helpers ----------------

    def wait_connected(self):
        rospy.loginfo("[STATE] Waiting for FCU connection...")
        while not rospy.is_shutdown():
            if self.state.connected:
                rospy.loginfo("[STATE] FCU connected")
                return True
            rospy.sleep(0.1)
        return False

    def wait_range(self):
        rospy.loginfo("[ALT] Waiting for rangefinder data...")
        while not rospy.is_shutdown():
            if self.range_m is not None and (time.time() - self.range_ts) < 1.0:
                rospy.loginfo(f"[ALT] Rangefinder OK, initial height: {self.range_m:.2f} m")
                return True
            rospy.sleep(0.1)
        return False

    def publish_thrust(self, thrust_0_1: float):
        """
        AttitudeTarget thrust: float32 [0..1]
        We keep level attitude (identity quaternion) and command only thrust.
        """
        msg = AttitudeTarget()
        msg.header.stamp = rospy.Time.now()
        msg.orientation = quat_identity()

        # Ignore body rates (we don't command them)
        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0

        # type_mask bits (per MAVROS):
        #  - ignore body rate: 1|2|4 = 7
        #  - we DO provide orientation, so don't ignore it
        msg.type_mask = 7

        msg.thrust = float(clamp(thrust_0_1, 0.0, 1.0))
        self.pub_att.publish(msg)

    def set_mode(self, mode: str):
        rospy.loginfo(f"[MODE] Setting mode {mode}")
        try:
            res = self.srv_mode(custom_mode=mode)
            if hasattr(res, "mode_sent") and not res.mode_sent:
                rospy.logwarn("[MODE] Mode not accepted (mode_sent=false)")
            return res
        except rospy.ServiceException as e:
            rospy.logerr(f"[MODE] set_mode failed: {e}")
            return None

    def arm(self, value: bool):
        try:
            return self.srv_arm(value)
        except rospy.ServiceException as e:
            rospy.logerr(f"[ARM] arming failed: {e}")
            return None

    # ---------------- main logic ----------------

    def run(self):
        r = rospy.Rate(self.args.rate)

        if not self.wait_connected():
            return
        if not self.wait_range():
            return

        # Pre-stream setpoints (ArduPilot/MAVROS requires setpoint stream before OFFBOARD-like control)
        rospy.loginfo("[STATE] Pre-stream attitude targets...")
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < self.args.prestream:
            self.publish_thrust(0.0)
            r.sleep()

        # Mode
        self.set_mode(self.args.mode)

        # Arm loop
        rospy.loginfo("[ARM] Arming...")
        while not rospy.is_shutdown() and not self.state.armed:
            self.publish_thrust(0.0)
            self.arm(True)
            r.sleep()

        if not self.state.armed:
            rospy.logerr("[ARM] Failed to arm")
            return

        rospy.loginfo("[ARM] ✔ ARMED")

        # ---------- TAKEOFF ----------
        rospy.loginfo(f"[TAKEOFF] Target altitude: {self.args.target:.2f} m")
        reached = False
        hold_start = None

        while not rospy.is_shutdown():
            h = self.range_m
            if h is None:
                self.publish_thrust(0.0)
                r.sleep()
                continue

            err = self.args.target - h

            rospy.loginfo_throttle(
                0.5,
                f"[ALT] Current: {h:.2f} m | Error: {err:+.2f} m"
            )

            if abs(err) < self.args.deadband:
                if not reached:
                    rospy.loginfo("[TAKEOFF] ✔ Target altitude reached")
                    reached = True
                    hold_start = time.time()

                self.publish_thrust(self.args.hover)
            else:
                thr = self.args.hover + self.args.kp * err
                self.publish_thrust(thr)

            if reached:
                elapsed = time.time() - hold_start
                rospy.loginfo_throttle(
                    1.0,
                    f"[HOLD] Holding altitude ({elapsed:.1f}/{self.args.hold:.1f} s)"
                )
                if elapsed >= self.args.hold:
                    rospy.loginfo("[HOLD] ✔ Hold complete")
                    break

            r.sleep()

        # ---------- LAND ----------
        rospy.loginfo("[LAND] Starting descent")
        while not rospy.is_shutdown():
            h = self.range_m
            if h is None:
                self.publish_thrust(0.0)
                r.sleep()
                continue

            rospy.loginfo_throttle(0.5, f"[LAND] Height: {h:.2f} m")

            if h <= self.args.disarm_alt:
                break

            # constant descend thrust (below hover)
            self.publish_thrust(max(0.0, self.args.hover - self.args.land_step))
            r.sleep()

        rospy.loginfo("[LAND] Cutting thrust")
        for _ in range(int(self.args.rate * 1.0)):
            self.publish_thrust(0.0)
            r.sleep()

        rospy.loginfo("[DISARM] Disarming")
        self.arm(False)

        rospy.loginfo("[DONE] Flight finished")


def main():
    p = argparse.ArgumentParser()

    # ArduPilot modes: "GUIDED" or "GUIDED_NOGPS" typically
    p.add_argument("--mode", default="GUIDED_NOGPS")

    p.add_argument("--rate", type=float, default=25.0)

    p.add_argument("--target", type=float, default=1.2)
    p.add_argument("--hold", type=float, default=8.0)

    # thrust parameters are 0..1
    p.add_argument("--hover", type=float, default=0.55, help="Hover thrust (0..1)")
    p.add_argument("--kp", type=float, default=0.35, help="P gain in thrust units per meter error")
    p.add_argument("--deadband", type=float, default=0.06)

    p.add_argument("--disarm_alt", type=float, default=0.12)

    p.add_argument("--prestream", type=float, default=1.5)

    # descent tuning
    p.add_argument("--land_step", type=float, default=0.12, help="How much thrust below hover during landing")

    args = p.parse_args()

    rospy.init_node("flight_no_cam_range_ardupilot", anonymous=True)
    FlightNoCamRangeArduPilot(args).run()


if __name__ == "__main__":
    main()
