import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

bagfn = '/home/rick/code/ai-imu-dr/data/bag/sim.bag'

odom_msg = []
imu_msg = []

with rosbag.Bag(bagfn, 'r') as in_bag:
    for _, msg, t in in_bag.read_messages("/odom_baselink"):
        # t x y z qx qy qz qw
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        msg = [t.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        odom_msg.append(msg)

    for _, msg, t in in_bag.read_messages("/imu/data"):
        # t w acc
        p = msg.angular_velocity
        q = msg.linear_acceleration
        msg = [t, p.x, p.y, p.z, q.x, q.y, q.z]
        imu_msg.append(msg)
        print(t.to_sec())

# print(odom_msg)

