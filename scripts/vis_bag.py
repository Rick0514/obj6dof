import rosbag, rospkg
import rospy
import numpy as np
import matplotlib.pyplot as plt

import time

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path

rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('obj6dof')

print(pkg_dir)

rospy.init_node('vis_bag')

bag = rosbag.Bag('{}/../pcd/pm/vis.bag'.format(pkg_dir))

# lidar_tsp = []
# image_tsp = []

mappub = rospy.Publisher('/globalmap', PointCloud2, queue_size=5, latch=True)
pathpub = rospy.Publisher('/path', Path, queue_size=5, latch=True)

for topic, msg, t in bag.read_messages():
    if topic == '/globalmap':
        mappub.publish(msg)
    elif topic == '/path':
        pathpub.publish(msg)
    elif topic.startswith('/avia'):
        # print('pub {}'.format(topic))
        rospy.Publisher(topic, PointCloud2, queue_size=5).publish(msg)
    # elif topic == '/image':
    #     image_tsp.append(msg.header.stamp.to_sec())
    time.sleep(0.2)
    

rospy.spin()

bag.close()
