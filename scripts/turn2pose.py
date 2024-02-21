import time
import rospy

import tf.transformations as tfs
import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped


T_str = []
with open('./data-results-cam.txt', 'r') as f:
    lines = f.readlines()
    
    idx = 0
    while idx < len(lines):
        if lines[idx].startswith('baseline'):
            T_str.append([lines[idx+1], lines[idx+2]])
        idx += 1

res = open('./poses.txt', 'w')
for i in range(5):
    res.write('{:.6f} '.format(0.0))
res.write('{:.6f}\n'.format(0.0))

rot0 = np.eye(3)
trans0 = np.zeros((3, 1))

T = []

for each in T_str:
    # parse q
    qs = each[0].strip()
    qs = qs[qs.find('[')+1:qs.find(']')].split(' ')
    
    q = []
    for e in qs:
        if len(e):
            q.append(float(e))
    q[1] = -q[1]

    # parse t
    ts = each[1].strip()
    ts = ts[ts.find('[')+1:ts.find(']')].split(' ')
    
    t = []
    for e in ts:
        if len(e):
            t.append(float(e))
        
    rot1 = tfs.quaternion_matrix(q)[:3, :3].T
    trans0 = trans0 - np.matmul(rot0, np.matmul(rot1, np.array(t).reshape(3, 1)))
    rot0 = np.matmul(rot0, rot1)
    
    rpy = tfs.euler_from_matrix(rot0)
    print(np.array(rpy) * 180.0 / np.math.pi)
    res.write('{} {} {} '.format(*rpy))
    xyz = list(trans0.reshape(3,))
    res.write('{} {} {}\n'.format(*xyz))

    h_rot0 = np.eye(4)
    h_rot0[:3, :3] = rot0
    T.append([tfs.quaternion_from_matrix(h_rot0), trans0.reshape(3,)])

rospy.init_node('vis_extrinsic')

broadcaster = tf2_ros.StaticTransformBroadcaster()
tfs_msg = TransformStamped()

tfs_msg.header.frame_id = 'map'
tfs_msg.header.stamp = rospy.Time.now()
tfs_msg.child_frame_id = 'front_cam'

# pub front first
tfs_msg.transform.translation.x = 0.0
tfs_msg.transform.translation.y = 0.0
tfs_msg.transform.translation.z = 2.0
rpy = list(np.math.pi * np.array([-90, 0, -90]) / 180)
q = tfs.quaternion_from_euler(*rpy)
tfs_msg.transform.rotation.x = q[0]
tfs_msg.transform.rotation.y = q[1]
tfs_msg.transform.rotation.z = q[2]
tfs_msg.transform.rotation.w = q[3]

broadcaster.sendTransform(tfs_msg)

frames = ['right_cam', 'back_cam', 'left_cam']

for i in range(3):

    tfs_msg.header.frame_id = 'front_cam'
    tfs_msg.header.stamp = rospy.Time.now()
    tfs_msg.child_frame_id = frames[i]

    # pub front first
    tfs_msg.transform.translation.x = T[i][1][0]
    tfs_msg.transform.translation.y = T[i][1][1]
    tfs_msg.transform.translation.z = T[i][1][2]
    tfs_msg.transform.rotation.x = T[i][0][0]
    tfs_msg.transform.rotation.y = T[i][0][1]
    tfs_msg.transform.rotation.z = T[i][0][2]
    tfs_msg.transform.rotation.w = T[i][0][3]
    
    broadcaster.sendTransform(tfs_msg)

    time.sleep(0.5)

rospy.spin()