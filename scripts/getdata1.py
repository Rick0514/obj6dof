#!/usr/bin/env python

import os, sys, termios, tty, select
import Queue

import cv2
import numpy as np

import rospy, rosbag
from cv_bridge import CvBridge

from sensor_msgs.msg import Image

def make_cb(cam_queue):
    def image_callback(image):
        if cam_queue.full():
            cam_queue.get_nowait()
        cam_queue.put_nowait([image.header.stamp.to_sec(), image])

    return image_callback    

class GetData:

    HelpInfo = """
    Get calib data!
              a
              |
        d --------- b
              |
              c
        a: choose a, b pair
        b: choose b, c pair
        ...
        r: record image
        ctrl+c: exit
    """

    bridge = CvBridge()

    cam_id = ['a', 'b', 'c', 'd']

    def __init__(self):
        
        print(self.HelpInfo)

        self.topics = ['/front_cam_fisheye/image_raw', 
                    '/right_cam_fisheye/image_raw',
                    '/back_cam_fisheye/image_raw',
                    '/left_cam_fisheye/image_raw']        
        self.freq = 10

        self.chosen_id = 0
        self.img_cnt = [1] * 4
        self.img_oc = [1] * 4
        self.subs = []
        self.queues = []
        for i in range(4):
            self.queues.append(Queue.Queue(20))
            sub = rospy.Subscriber(self.topics[i], Image,
                        make_cb(self.queues[i]),
                        queue_size=10)
            self.subs.append(sub)

        self.bag = rosbag.Bag('./data.bag', 'w')

    def set_choose_id(self, i):
        self.chosen_id = i % 4

    def save_sync_images(self):
        
        q1_idx = self.chosen_id
        q2_idx = (self.chosen_id + 1) % 4

        if not self.queues[q1_idx].empty() and not self.queues[q2_idx].empty():
            # sync
            c1_t = self.queues[q1_idx].queue[0][0]
            c2_t = self.queues[q2_idx].queue[0][0]
            sync_t = max(c1_t, c2_t)
            if c1_t > c2_t:
                while abs(c1_t - self.queues[q2_idx].queue[0][0]) > 1.0 / self.freq:
                    self.queues[q2_idx].get_nowait()
            else:
                while abs(c2_t - self.queues[q1_idx].queue[0][0]) > 1.0 / self.freq:
                    self.queues[q1_idx].get_nowait()
            
            # cv2.imwrite('./feimg/cam{}_{}.jpg'.format(q1_idx, self.img_cnt[q
            # cv2.imwrite('./feimg/cam{}_{}.jpg'.format(q2_idx, self.img_cnt[q2_idx]), 1_idx]), self.bridge.imgmsg_to_cv2(self.queues[q1_idx].get()[1], "bgr8"))self.bridge.imgmsg_to_cv2(self.queues[q2_idx].get()[1], "bgr8"))
            self.bag.write(self.topics[q1_idx], self.queues[q1_idx].get()[1], rospy.Time.from_seconds(sync_t))
            self.bag.write(self.topics[q2_idx], self.queues[q2_idx].get()[1], rospy.Time.from_seconds(sync_t))

            print('At {}s pair ({},{}) get ({},{}) imgs'.format(sync_t, q1_idx, q2_idx, self.img_cnt[q1_idx], self.img_cnt[q2_idx]))
            self.img_cnt[q1_idx] += 1
            self.img_cnt[q2_idx] += 1

        else:
            print('some queue is empty, check you camera!')

    def save_one_image(self):
        idx = self.chosen_id
        img = self.bridge.imgmsg_to_cv2(self.queues[idx].get()[1], "bgr8")
        cv2.imwrite('./feimg/cam{}_{}.jpg'.format(idx, self.img_oc[idx]), img)
        self.img_oc[idx] += 1

    def exit(self):
        self.bag.close()

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

    rospy.init_node('get_calib_data')
    
    settings = termios.tcgetattr(sys.stdin)

    gd = GetData()

    print("cur dir: {}".format(os.getcwd()))
    if not os.path.exists('./feimg'):
        os.mkdir('./feimg')

    try:
        while not rospy.is_shutdown():
            key = getKey(None)

            if key in GetData.cam_id:
                idx = GetData.cam_id.index(key)
                gd.set_choose_id(idx)
                print('({},{}) cam pair is chosen!'.format(idx, (idx + 1) % 4))
            elif key == 'r':
                gd.save_one_image()
            elif key == 's':
                gd.save_sync_images()
            elif key == '\x03':
                break
    
    except Exception as e:
        print(e)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    gd.exit()