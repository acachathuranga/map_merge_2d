#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid

map1_topic="/multijackal_01/map"
map2_topic="/multijackal_02/map"
map1_delay = 0.0
map2_delay = 0.0
map1_msg_count = 0
map2_msg_count = 0

def map1_cb(data):
    global map1_delay, map1_msg_count
    msg_delay = rospy.Time.now().to_sec() -data.header.stamp.to_sec()
    map1_delay = 0.8 * msg_delay + 0.2 * map1_delay
    map1_msg_count = map1_msg_count + 1
    rospy.loginfo("Map1 Delay %f      :  count %d", map1_delay, map1_msg_count)
    
def map2_cb(data):
    global map2_delay, map2_msg_count
    msg_delay = rospy.Time.now().to_sec() - data.header.stamp.to_sec()
    map2_delay = 0.8 * msg_delay + 0.2 * map2_delay
    map2_msg_count = map2_msg_count + 1
    rospy.loginfo("Map2 Delay %f      :  count %d", map2_delay, map2_msg_count)
    
def latency_monitor():
    rospy.init_node('latency_mon', anonymous=True)
    rospy.loginfo("Latency Monitor")

    rospy.Subscriber(map1_topic, OccupancyGrid, map1_cb)
    rospy.Subscriber(map2_topic, OccupancyGrid, map2_cb)
    rospy.spin()

if __name__ == '__main__':
    latency_monitor()
