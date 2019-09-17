#!/usr/bin/env python
# Copyright 2019 Lucas Walter
# BSD Licensed

import rospy
import tf
import tf2_ros

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

def transform_stamped(parent, child, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    ts = TransformStamped()
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

        self.config = None
        self.stored_config = None
        self.ddr = DDynamicReconfigure("")
        self.ddr.add_variable("frame_id", "frame id", "map")
        self.ddr.add_variable("child_frame_id", "frame id", "frame")
        scale = rospy.get_param("~scale", 10.0)
        self.ddr.add_variable("x", "x", 0.0, -scale, scale)
        self.ddr.add_variable("y", "y", 0.0, -scale, scale)
        self.ddr.add_variable("z", "z", 0.0, -scale, scale)
        scale = rospy.get_param("~angle_scale", 3.2)
        self.ddr.add_variable("roll", "roll", 0.0, -scale, scale)
        self.ddr.add_variable("pitch", "pitch", 0.0, -scale, scale)
        self.ddr.add_variable("yaw", "yaw", 0.0, -scale, scale)
        self.ddr.add_variable("zero", "zero", False)
        self.ddr.add_variable("store", "store", False)
        self.ddr.add_variable("reset", "reset", False)
        self.ddr.start(self.config_callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update)

    def config_callback(self, config, level):
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
        if config.reset:
            config.reset = False
            config = self.stored_config
        if config.store:
            config.store = False
            self.stored_config = config
        self.config = config
        return config

    def update(self, event):
        config = self.config
        ts = transform_stamped(config.frame_id, config.child_frame_id,
                               config.x, config.y, config.z,
                               config.roll, config.pitch, config.yaw)
        ts.header.stamp = event.current_real

        tfm = TFMessage()
        tfm.transforms.append(ts)
        self.tf_pub.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('ddr_to_tf')
    ddr_to_tf = DDRtoTF()
    rospy.spin()
