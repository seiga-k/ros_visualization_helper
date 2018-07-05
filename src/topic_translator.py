#!/usr/bin/env python2
#encoding: UTF-8
import string
import sys
import threading

import rospy
import rosgraph
import roslib
import roslib.msgs
import roslib.message
import roslib.names
from rqt_py_common import topic_helpers
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class RosTopicTranslatorException(Exception):
    pass

class ROSData(object):
    def __init__(self, topic, start_time):
        self.name = topic
        self.start_time = start_time
        self.error = None
        self.lock = threading.Lock()
        topic_type, real_topic, fields = self._get_topic_type(topic)
        #rospy.loginfo("type : " + topic_type)
        #rospy.loginfo("real : " + real_topic)
        #rospy.loginfo("fields : " + fields)
        if topic_type is not None:
            self.field_evals = self._generate_field_evals(fields)
            data_class = roslib.message.get_message_class(topic_type)
            self.sub = rospy.Subscriber(real_topic, data_class, self._ros_cb)
        else:
            self.error = RosTopicTranslatorException("Can not resolve topic type of %s" % topic)        
        self.pub = rospy.Publisher("~" + topic.strip('/'), Float32, queue_size=10)
        rospy.loginfo("Publish to " + "~" + topic.strip('/'))
            
    def close(self):
        self.sub.unregister()

    def _ros_cb(self, msg):
        val = self._get_data(msg)
        #rospy.loginfo(val)
        self.pub.publish(val)

    def _get_data(self, msg):
        val = msg
        try:
            if not self.field_evals:
                if isinstance(val, Bool):
                    # extract boolean field from bool messages
                    val = val.data
                return float(val)
            for f in self.field_evals:
                val = f(val)
            return float(val)
        except IndexError:
            self.error = RosTopicTranslatorException("[%s] index error for: %s" % (self.name, str(val).replace('\n', ', ')))
        except TypeError:
            self.error = RosTopicTranslatorException("[%s] value was not numeric: %s" % (self.name, val))
            
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

    def _field_eval(self, field_name):
        def fn(f):
            return getattr(f, field_name)
        return fn

    def _generate_field_evals(self, fields):
        try:
            evals = []
            fields = [f for f in fields.split('/') if f]
            for f in fields:
                if '[' in f:
                    field_name, rest = f.split('[')
                    slot_num = string.atoi(rest[:rest.find(']')])
                    evals.append(_array_eval(field_name, slot_num))
                else:
                    evals.append(self._field_eval(f))
            return evals
        except Exception as e:
            raise RosTopicTranslatorException("cannot parse field reference [%s]: %s" % (fields, str(e)))

if __name__ == '__main__':
    rospy.init_node('topic_translator')
    rospy.loginfo("Start topic_translator node")
    if rospy.has_param("~input"):
        topic_name = rospy.get_param("~input")
        rospy.loginfo("Transfer a " + topic_name + " topic")
    else:
        rospy.loginfo("No input parameter!!")
        sys.exit(1)
   
    start_time = rospy.get_time()
    rosdata = {}
    
    rosdata[topic_name] = ROSData(topic_name, start_time)
    if rosdata[topic_name].error is not None:
        rospy.loginfo(str(rosdata[topic_name].error))
        del rosdata[topic_name]
    else:
        rospy.loginfo("Input topic set up correctly")
    rospy.spin()