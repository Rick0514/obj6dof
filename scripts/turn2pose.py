import time, math
import rospy

import tf.transformations as tfs
import numpy as np

import tf2_ros
from geometry_msgs.msg import TransformStamped


# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    _EPS = 1.0
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

T_str = []
with open('./data-results.txt', 'r') as f:
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
    
    rpy = euler_from_matrix(rot0)
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
time.sleep(0.5)

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