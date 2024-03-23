import numpy as np
import tf.transformations as tfs

import rospy, tf2_ros
from geometry_msgs.msg import TransformStamped

to_rad = np.math.pi / 180

# mat is lidar to cam -> lT_a
mat = np.loadtxt('extrin.txt', skiprows=1)

# print(mat)

# turn to gazebo_cam to lidar
# el = to_rad * np.array([-90, 0, -90])
# Rcam = tfs.euler_matrix(el[0], el[1], el[2])    # bT_a
# # mat = np.linalg.inv(mat)       # lT_a
# mat = np.matmul(mat, Rcam.T)   # lT_a * aT_b = lT_b

# rpy = np.array(tfs.euler_from_matrix(mat[:3, :3]))
trans = mat[:3, 3]
# print('rpy(rad): {}'.format(rpy))
# print('rpy(deg): {}'.format(rpy * 180 / np.math.pi))
# print('trans: {}'.format(trans))


# el = to_rad * np.array([180, 90, 0])
# Rcam = tfs.euler_matrix(el[0], el[1], el[2], 'rxyz')
# print(Rcam[:3, :3])




rospy.init_node('vis_extrinsic')

broadcaster = tf2_ros.StaticTransformBroadcaster()
tfs_msg = TransformStamped()

tfs_msg.header.frame_id = 'avia'
tfs_msg.header.stamp = rospy.Time.now()
tfs_msg.child_frame_id = 'cam'

# pub front first
tfs_msg.transform.translation.x = trans[0]
tfs_msg.transform.translation.y = trans[1]
tfs_msg.transform.translation.z = trans[2]
q = tfs.quaternion_from_matrix(mat)
tfs_msg.transform.rotation.x = q[0]
tfs_msg.transform.rotation.y = q[1]
tfs_msg.transform.rotation.z = q[2]
tfs_msg.transform.rotation.w = q[3]

broadcaster.sendTransform(tfs_msg)

rospy.spin()