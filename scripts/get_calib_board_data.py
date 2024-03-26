#!/usr/bin/env python

import os, sys, termios, tty, select, time
import Queue
import threading

import cv2
import numpy as np

import rospy, rosbag, rospkg
from cv_bridge import CvBridge
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState
from std_srvs.srv import Empty, EmptyRequest

from geometry_msgs.msg import Quaternion, Point
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_euler

bridge = CvBridge()
cam1_queue = Queue.Queue(25)
cam2_queue = Queue.Queue(25)

HelpInfo = """
Control Platform!
    ---------------------------
    Moving around:
        W/S: X-Axis
        A/D: Y-Axis
        X/Z: Z-Axis

        Q/E: Yaw
        I/K: Pitch
        J/L: Roll

        C: Capture image
        R: Reset RPY

        Ctrl+C: Exit
"""

t_step = 0.02  # in meters
r_step = np.deg2rad(5)

moveTargetSrv = None
pau = None
unpau = None
# translation and rotation key mappings to the state changes
# for translation [x, y ,z] + [offset_x, offset_y, offset_z]
# for rotation [roll, pitch, yaw] + [offset_roll, offset_pitch, offset_yaw]
t_step_key_map = {'w': np.array([t_step, 0, 0]), 's': np.array([-t_step, 0, 0]),
                  'a': np.array([0, t_step, 0]), 'd': np.array([0, -t_step, 0]),
                  'x': np.array([0, 0, t_step]), 'z': np.array([0, 0, -t_step])}

r_step_key_map = {'i': np.array([r_step, 0, 0]), 'k': np.array([-r_step, 0, 0]),
                  'j': np.array([0, r_step, 0]), 'l': np.array([0, -r_step, 0]),
                  'q': np.array([0, 0, r_step]), 'e': np.array([0, 0, -r_step])}


# def clear_queue():
#     for i in range(img_queue.qsize()):
#         img_queue.get_nowait()

# subscribe to images and when 'S' is pressed save the image
def image_callback(cam_queue, image):
    if cam_queue.full():
        cam_queue.get_nowait()

    cam_queue.put_nowait([image.header.stamp.to_sec(), image])
    # print('queue size: {}'.format(cam_queue.qsize()))


def change_link_state(t, quat):
    global moveTargetSrv
    
    pau.call(EmptyRequest())    
    try:
        state = LinkState()
        state.link_name = 'Checkboard67::main'
        state.pose.position = Point(t[0], t[1], t[2])
        state.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        moveTargetSrv(state)
        # print(status)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)
    
    unpau.call(EmptyRequest())

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':

    print(HelpInfo)

    rospy.init_node('get_checkboard_data')

    # bag_name = 'fe1.bag'

    try:
        rospy.wait_for_service('/gazebo/pause_physics', 1)
        rospy.wait_for_service('/gazebo/set_link_state', 1)
    except rospy.ROSException as e:
        print("Error %s" % e)
        sys.exit(1)

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('obj6dof')

    save_dir = pkg_path + '/../imgs'
    if not os.path.exists(save_dir):
        os.mkdir(save_dir)
    else:
        for e in os.listdir(save_dir):
            os.unlink(os.path.join(save_dir, e))
    # else:
    #     save_dir = pkg_path + '/../bag'
    #     if not os.path.exists(save_dir):
    #         os.mkdir(save_dir)
    #     bag = rosbag.Bag(os.path.join(save_dir, bag_name), 'w')

    settings = termios.tcgetattr(sys.stdin)

    moveTargetSrv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
    # cam1, cam2
    sub1 = rospy.Subscriber('/hik/image', Image, (lambda img : image_callback(cam1_queue, img)), queue_size=10)
    # sub2 = rospy.Subscriber('/left_cam_fisheye/image_raw', Image, (lambda img : image_callback(cam2_queue, img)), queue_size=10)
    # important for update rendering smoothly
    pau = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpau = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    # pau.call(EmptyRequest())

    img_counter = 0
    tr = np.array([3.0, 0, 1.0])
    rpy = np.array(np.deg2rad([0, -90, 0]))

    change_link_state(tr, quaternion_from_euler(*rpy))

    try:
        while not rospy.is_shutdown():
            key = getKey(None)

            if key in t_step_key_map:
                tr += t_step_key_map[key]
                change_link_state(tr, quaternion_from_euler(*rpy))
                print('tr: {}, rpy: {}'.format(tr, rpy))
            elif key in r_step_key_map:
                rpy += r_step_key_map[key]
                change_link_state(tr, quaternion_from_euler(*rpy))
                print('tr: {}, rpy: {}'.format(tr, rpy))
            elif key == 'c':
                # cv2.imwrite(filename, img_queue.get(), [int(cv2.IMWRITE_JPEG_QUALITY), 100])
                # if bag is None:
                if not cam1_queue.empty():
                    img = bridge.imgmsg_to_cv2(cam1_queue.get()[1], "bgr8")
                    img_name = '{}/{}.png'.format(save_dir, img_counter)
                    cv2.imwrite(img_name, img)
                    img_counter += 1
                    print('save to file: {}'.format(img_counter))

                # if not cam1_queue.empty() and not cam2_queue.empty():
                #     # sync
                #     c1_t = cam1_queue.queue[0][0]
                #     c2_t = cam2_queue.queue[0][0]
                #     if c1_t > c2_t:
                #         while abs(c1_t - cam2_queue.queue[0][0]) > 0.1:
                #             cam2_queue.get_nowait()
                #     else:
                #         while abs(c2_t - cam1_queue.queue[0][0]) > 0.1:
                #             cam1_queue.get_nowait()
                    
                #     # c1_name = '{}/c1_{}.jpg'.format(save_dir, img_counter)
                #     # c2_name = '{}/c2_{}.jpg'.format(save_dir, img_counter)
                #     bag.write('/cam1/image', cam1_queue.get()[1])
                #     bag.write('/cam2/image', cam2_queue.get()[1])

                #     print('write to bag: {}'.format(img_counter))
                #     img_counter += 1
                    # print('Saved file: {}'.format(c2_name))

            elif key == 'r':  # reset roll-pitch-yaw
                rpy = np.array([0.0, -1.571, 0.0])
                change_link_state(tr, quaternion_from_euler(*rpy))

            elif key == '\x03':
                break
    
    except Exception as e:
        print(e)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # bag.close()