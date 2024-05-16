#!/usr/bin/env python
# Copyright 2019-2020 Lucas Walter
# BSD Licensed
#
# Move a tf around with dynamic reconfigure
# Also velocity controls

import copy

import rospy
import tf

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


def transform_stamped(parent, child, stamp, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    ts = TransformStamped()
    ts.header.stamp = stamp
    ts.header.frame_id = parent
    ts.child_frame_id = child
    ts.transform.translation.x = x
    ts.transform.translation.y = y
    ts.transform.translation.z = z
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    ts.transform.rotation.x = quat[0]
    ts.transform.rotation.y = quat[1]
    ts.transform.rotation.z = quat[2]
    ts.transform.rotation.w = quat[3]
    return ts


class DDRtoTF(object):
    def __init__(self):
        self.tf_pub = rospy.Publisher('/tf', TFMessage, queue_size=4)

        self.x = None
        self.y = None
        self.z = None

        self.roll = None
        self.pitch = None
        self.yaw = None

        self.config = None
        self.stored_config = None
        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("frame_id", "frame id", "map")
        self.ddr.add_variable("child_frame_id", "frame id", "frame")
        scale = rospy.get_param("~scale", 10.0)
        self.ddr.add_variable("x", "x", 0.0, -scale, scale)
        self.ddr.add_variable("y", "y", 0.0, -scale, scale)
        self.ddr.add_variable("z", "z", 0.0, -scale, scale)
        sc = 100.0
        self.ddr.add_variable("base_x", "x above is relative to this base value", 0.0, -scale * sc, scale * sc)
        self.ddr.add_variable("base_y", "y above is relative to this base value", 0.0, -scale * sc, scale * sc)
        self.ddr.add_variable("base_z", "z above is relative to this base value", 0.0, -scale * sc, scale * sc)
        angle_scale = rospy.get_param("~angle_scale", 3.2)
        self.ddr.add_variable("roll", "roll", 0.0, -angle_scale, angle_scale)
        self.ddr.add_variable("pitch", "pitch", 0.0, -angle_scale, angle_scale)
        self.ddr.add_variable("yaw", "yaw", 0.0, -angle_scale, angle_scale)
        self.ddr.add_variable("enable_live_rpy", "update rpy live, doesn't work with velocity mode", True)
        self.ddr.add_variable("zero", "zero", False)
        self.ddr.add_variable("store", "store", False)
        self.ddr.add_variable("reset", "reset", False)

        vel_scale = rospy.get_param("~vel_scale", 1.0)
        self.ddr.add_variable("vx", "x velocity", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("vy", "y velocity", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("vz", "z velocity", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("angular_x", "roll rate", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("angular_y", "pitch rate", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("angular_z", "yaw rate", 0.0, -vel_scale, vel_scale)
        self.ddr.add_variable("enable_velocity", "enable velocity", False)

        self.ddr.add_variable("use_bound", "use bounds", False)
        self.ddr.add_variable("bound_x", "x +/- bound", scale, 0.0, scale)
        self.ddr.add_variable("bound_y", "y +/- bound", scale, 0.0, scale)
        self.ddr.add_variable("bound_z", "z +/- bound", scale, 0.0, scale)

        self.ddr.add_variable("use_expected_time", "use expected time, may not work well with sim time", False)
        self.ddr.start(self.config_callback)
        # TODO(lucasw) maybe only set reset=True when use_sim_time is true
        # (can that be detected without using get_param("/use_sim_time")?)
        self.timer = rospy.Timer(rospy.Duration(0.033), self.update, reset=True)

    def config_callback(self, config, level):
        if self.x is None:
            self.x = config.x
            self.y = config.y
            self.z = config.z
            self.roll = config.roll
            self.pitch = config.pitch
            self.yaw = config.yaw
        if self.stored_config is None:
            self.stored_config = config
        if config.zero:
            config.zero = False
            config.x = 0.0
            config.y = 0.0
            config.z = 0.0
            config.roll = 0.0
            config.pitch = 0.0
            config.yaw = 0.0
            self.x = config.x
            self.y = config.y
            self.z = config.z
            self.roll = config.roll
            self.pitch = config.pitch
            self.yaw = config.yaw
        if config.reset:
            config.reset = False
            config = self.stored_config
            self.x = config.x
            self.y = config.y
            self.z = config.z
            self.roll = config.roll
            self.pitch = config.pitch
            self.yaw = config.yaw
        if config.enable_velocity and (self.config is None or not self.config.enable_velocity):
            self.x = config.base_x + config.x
            self.y = config.base_y + config.y
            self.z = config.base_z + config.z
            self.roll = config.roll
            self.pitch = config.pitch
            self.yaw = config.yaw
        if config.enable_velocity and config.enable_live_rpy:
            rospy.logwarn("disabling live rpy in favor of velocity mode")
            config.enable_live_rpy = False
        if config.enable_live_rpy:
            self.roll = config.roll
            self.pitch = config.pitch
            self.yaw = config.yaw

        if self.config is not None:
            # latch last velocity defined position
            if self.config.enable_velocity and not config.enable_velocity:
                config.base_x = self.x
                config.base_y = self.y
                config.base_z = self.z
                config.x = 0.0
                config.y = 0.0
                config.z = 0.0

                # TODO(lucasw) base_roll etc.?

        if config.store:
            config.store = False
            self.stored_config = config
        self.config = config
        return config

    def clip(self, pos, bound):
        if bound <= 0.0:
            return pos
        if pos > bound:
            pos -= 2.0 * bound
        if pos < -bound:
            pos += 2.0 * bound
        return pos

    def update(self, event):
        config = copy.deepcopy(self.config)

        if config.use_expected_time:
            last = event.last_expected
            cur = event.current_expected
        else:
            last = event.last_real
            cur = event.current_real

        if cur == last:
            # TODO(lucasw) not sure why this happens, but it does- maybe only with bag playback?
            rospy.logwarn_throttle(8.0, "duplicate stamps")
            return

        if config.enable_velocity:
            if last is not None:
                # TODO(lucasw) this shouldn't happen unless last_expected is changed to last_real above?
                if last > cur:
                    rospy.logwarn(f"time jump backwards {last.to_sec():0.2f} {cur.to_sec():0.2f}"
                                  + f", delta {(last - cur).to_sec():0.2f}")
                    last = cur
                    return
                elif last == cur:
                    rospy.logwarn(f"time didn't advance {last.to_sec():0.2f} {cur.to_sec():0.2f}")
                    return
                else:
                    dt = (cur - last).to_sec()
                    self.x += config.vx * dt
                    self.y += config.vy * dt
                    self.z += config.vz * dt

                    self.roll += config.angular_x * dt
                    self.pitch += config.angular_y * dt
                    self.yaw += config.angular_z * dt
        else:
            self.x = config.base_x + config.x
            self.y = config.base_y + config.y
            self.z = config.base_z + config.z

        if config.use_bound:
            self.x = self.clip(self.x, config.bound_x)
            self.y = self.clip(self.y, config.bound_y)
            self.z = self.clip(self.z, config.bound_z)

        ts = transform_stamped(config.frame_id, config.child_frame_id, cur,
                               self.x, self.y, self.z,
                               self.roll, self.pitch, self.yaw)
        rospy.logdebug_throttle(1.0, f"{config.base_x:0.2f} {config.x:0.2f} -> {self.x:0.2f}")

        tfm = TFMessage()
        tfm.transforms.append(ts)
        self.tf_pub.publish(tfm)


if __name__ == '__main__':
    rospy.init_node('ddr_to_tf')
    ddr_to_tf = DDRtoTF()
    rospy.spin()
