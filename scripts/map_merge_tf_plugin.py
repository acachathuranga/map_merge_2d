#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf
import tf.transformations
from tf2_msgs.msg import TFMessage
import numpy as np

transform_publisher = None
tf_listener = None
map_frame = "map"
submap_name = "multijackal_03"
external_map_pose_topic = "multijackal_02/drone_map_frame"
transform_output_topic = "multijackal_01/map_merge_2d/submap_transforms"

def pose_callback(pose):
    try:
        (trans,rot) = tf_listener.lookupTransform(map_frame, pose.header.frame_id, rospy.Time(0))
        map_to_pose_tf = tf.transformations.quaternion_matrix(rot)
        map_to_pose_tf[0:3, -1] = trans
        
        pose_local_tf = tf.transformations.quaternion_matrix([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
        pose_local_tf[0:3, -1] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        
        tf_correction = tf.transformations.quaternion_matrix([0.0, 0.0, 1.0, 0.0])
        tf_correction[0:3, -1] = [0.0, 0.0, 0.0]
        
        pose_tf_raw = np.dot(map_to_pose_tf, pose_local_tf)
        pose_tf = np.dot(pose_tf_raw, tf_correction)
        pose_q = tf.transformations.quaternion_from_matrix(pose_tf)
        transform = TransformStamped()
        transform.header.stamp = pose.header.stamp
        transform.header.frame_id = map_frame
        transform.child_frame_id = submap_name
        transform.header.seq = pose.header.seq
        transform.transform.translation.x = pose_tf[0, 3]
        transform.transform.translation.y = pose_tf[1, 3]
        transform.transform.translation.z = pose_tf[2, 3]
        transform.transform.rotation.x = pose_q[0]
        transform.transform.rotation.y = pose_q[1]
        transform.transform.rotation.z = pose_q[2]
        transform.transform.rotation.w = pose_q[3]
        
        rospy.loginfo("Received %s transform : [%.1f, %.1f, %.1f] [%.1f, %.1f, %.1f, %.1f]"%(submap_name, pose_tf[0,3], pose_tf[1,3], pose_tf[2,3], 
                                                                                             pose_q[0], pose_q[1], pose_q[2], pose_q[3]))
        
        transform_msg = TFMessage()
        transform_msg.transforms.append(transform)
        transform_publisher.publish(transform_msg)
        
    except Exception as ex:
        rospy.logerr(str(ex))
        rospy.logwarn("%s : External map transform received, but cannot transform from %s to %s"%(rospy.get_name(), pose.header.frame_id, map_frame))
    
def plugin():
    rospy.init_node('map_merge_tf_plugin')
    
    global transform_publisher, tf_listener
    tf_listener = tf.TransformListener()
    transform_publisher = rospy.Publisher(transform_output_topic, TFMessage, queue_size=10)
    rospy.Subscriber(external_map_pose_topic, PoseStamped, pose_callback)
    
    rospy.loginfo("Started map_merge_tf_plugin")
    rospy.spin()
    

if __name__ == '__main__':
    plugin()
