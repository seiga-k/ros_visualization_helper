#!/usr/bin/env python2
#encoding: UTF-8
import string
import sys
import threading
import math

import rospy
import rosgraph
import roslib
import roslib.msgs
import roslib.message
import roslib.names
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion

class RosQuaternionTranslatorException(Exception):
    pass

class ROSQuat(object):
    def __init__(self, topic, start_time, mode):
        self.name = topic
        self.start_time = start_time
        self.error = None
        self.lock = threading.Lock()
        self.mode = mode
        topic_type, real_topic, fields = self._get_topic_type(topic)
        rospy.loginfo("type : " + topic_type)
        rospy.loginfo("real : " + real_topic)
        rospy.loginfo("fields : " + fields)
        self.field = fields.strip('/')
        if topic_type is not None:
            data_class = roslib.message.get_message_class(topic_type)
            self.sub = rospy.Subscriber(real_topic, data_class, self._ros_cb)
        else:
            self.error = RosTopicTranslatorException("Can not resolve topic type of %s" % topic)
        self.pubx = rospy.Publisher("~" + topic.strip('/') + "/x", Float32, queue_size=10)
        self.puby = rospy.Publisher("~" + topic.strip('/') + "/y", Float32, queue_size=10)
        self.pubz = rospy.Publisher("~" + topic.strip('/') + "/z", Float32, queue_size=10)
        rospy.loginfo("Publish to " + "~" + topic.strip('/'))
            
    def close(self):
        self.sub.unregister()

    def _ros_cb(self, msg):
#        print(msg)
        quat = getattr(msg, self.field)
#        print(quat)
        x = getattr(quat, "x")
        y = getattr(quat, "y")
        z = getattr(quat, "z")
        w = getattr(quat, "w")
#        val = self._get_data(msg)
        rpy = euler_from_quaternion([w, x, y, z])
#        print(rpy)
        k = 1.0
        if self.mode :
            k = 180.0 / math.pi
        self.pubx.publish(rpy[2] * k)
        self.puby.publish(rpy[1] * k)
        self.pubz.publish(rpy[0] * k)
            
    def _get_topic_type(self, topic):
        try:
            master = rosgraph.Master('/rostt')
            val = master.getTopicTypes()
        except:
            raise RosTopicTranslatorException("unable to get list of topics from master")
        matches = [(t, t_type) for t, t_type in val if t == topic or topic.startswith(t + '/')]
        if matches:
            t, t_type = matches[0]
            if t_type == roslib.names.ANYTYPE:
                return None, None, None
            if t_type == topic:
                return t_type, None
            return t_type, t, topic[len(t):]
        else:
            return None, None, None
        
if __name__ == '__main__':
    rospy.init_node('quaternion_translator')
    rospy.loginfo("Start quaternion_translator node")
    if rospy.has_param("~input"):
        topic_name = rospy.get_param("~input")
        rospy.loginfo("Transfer a " + topic_name + " topic")
    else:
        rospy.loginfo("No input parameter!!")
        sys.exit(1)
        
    if rospy.has_param("~degree_mode"):
        deg_mode = rospy.get_param("~degree_mode")
    else:
        deg_mode = True
   
    start_time = rospy.get_time()
    rosquat = {}
    
    rosquat[topic_name] = ROSQuat(topic_name, start_time, deg_mode)
    if rosquat[topic_name].error is not None:
        rospy.loginfo(str(rosquat[topic_name].error))
        del rosquat[topic_name]
    else:
        rospy.loginfo("Input topic set up correctly")
    rospy.spin()