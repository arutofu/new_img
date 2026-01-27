#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import time
import math
from collections import deque

import rospy
from sensor_msgs.msg import Range
from mavros_msgs.msg import State, ManualControl, StatusText
from mavros_msgs.srv import CommandBool, SetMode


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class FlightNoCamRange:

    def __init__(self, args):
        self.args = args
        self.state = State()
        self.range_m = None
        self.range_ts = 0.0
        self.last_statustext = deque(maxlen=50)

        self.pub_manual = rospy.Publisher(
            "/mavros/manual_control/send",
            ManualControl,
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

    def wait_range(self):
        rospy.loginfo("[ALT] Waiting for rangefinder data...")
        while not rospy.is_shutdown():
            if self.range_m is not None and (time.time() - self.range_ts) < 1.0:
                rospy.loginfo(f"[ALT] Rangefinder OK, initial height: {self.range_m:.2f} m")
                return True
            rospy.sleep(0.1)

    def publish_manual(self, z):
        msg = ManualControl()
        msg.z = int(clamp(z, 0, 1000))
        self.pub_manual.publish(msg)

    # ---------------- main logic ----------------

    def run(self):
        r = rospy.Rate(self.args.rate)

        self.wait_connected()
        self.wait_range()

        rospy.loginfo("[STATE] Pre-stream manual control...")
        t0 = time.time()
        while time.time() - t0 < self.args.prestream:
            self.publish_manual(0)
            r.sleep()

        rospy.loginfo(f"[MODE] Setting mode {self.args.mode}")
        self.srv_mode(custom_mode=self.args.mode)

        rospy.loginfo("[ARM] Arming...")
        while not self.state.armed:
            self.publish_manual(0)
            self.srv_arm(True)
            r.sleep()

        rospy.loginfo("[ARM] ✔ ARMED")

        # ---------- TAKEOFF ----------
        rospy.loginfo(f"[TAKEOFF] Target altitude: {self.args.target:.2f} m")
        reached = False
        hold_start = None

        while not rospy.is_shutdown():
            h = self.range_m
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

                self.publish_manual(self.args.hover)
            else:
                thr = self.args.hover + self.args.kp * err
                self.publish_manual(thr)

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
            rospy.loginfo_throttle(0.5, f"[LAND] Height: {h:.2f} m")

            if h <= self.args.disarm_alt:
                break

            thr = self.args.hover - 150
            self.publish_manual(thr)
            r.sleep()

        rospy.loginfo("[LAND] Cutting throttle")
        for _ in range(20):
            self.publish_manual(0)
            r.sleep()

        rospy.loginfo("[DISARM] Disarming")
        self.srv_arm(False)

        rospy.loginfo("[DONE] Flight finished")


# ---------------- entry ----------------

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", default="ALTCTL")
    p.add_argument("--rate", type=float, default=25)
    p.add_argument("--target", type=float, default=1.2)
    p.add_argument("--hold", type=float, default=8.0)
    p.add_argument("--hover", type=float, default=350)
    p.add_argument("--kp", type=float, default=700)
    p.add_argument("--deadband", type=float, default=0.06)
    p.add_argument("--disarm_alt", type=float, default=0.12)
    p.add_argument("--prestream", type=float, default=1.5)

    args = p.parse_args()
    rospy.init_node("flight_no_cam_range", anonymous=True)
    FlightNoCamRange(args).run()


if __name__ == "__main__":
    main()
