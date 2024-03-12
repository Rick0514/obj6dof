import rosbag
import rospy
import numpy as np
import matplotlib.pyplot as plt


bag = rosbag.Bag('sim.bag')

lidar_tsp = []
image_tsp = []

for topic, msg, t in bag.read_messages():
    if topic == '/scan':
        lidar_tsp.append(msg.header.stamp.to_sec())
    elif topic == '/image':
        image_tsp.append(msg.header.stamp.to_sec())

bag.close()

plt.figure(1)

nl = len(lidar_tsp)
ni = len(image_tsp)

plt.scatter(np.arange(nl), lidar_tsp, label='livox')
plt.scatter(np.arange(ni), image_tsp, label='cam')

plt.ylabel('timestamp/s')

plt.grid()
plt.legend()
plt.show()
