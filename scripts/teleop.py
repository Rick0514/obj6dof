#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from std_srvs.srv import Empty, EmptyRequest

from geometry_msgs.msg import Twist, Pose
import angles
import tf.transformations as tts
import numpy as np

import sys, select, termios, tty

rad2deg = 1.0 / angles.pi * 180
deg2rad = 1.0 / rad2deg

HelpInfo = """
Control Your Vehicle!
    ---------------------------
    Moving around:
        W/S: X-Axis
        A/D: Y-Axis
        X/Z: Z-Axis

        Q/E: Yaw
        I/K: Pitch
        J/L: Roll

    Trans Speed:
    Slow / Fast: 1 / 2

    Rot Speed:
    Slow / Fast: 3 / 4
    
    Record PC: R

    CTRL-C to quit
"""
notMove = np.array([0.0, 0.0, 0.0])
transBindings = {
    'w': np.array([1., 0., 0.]),
    's': np.array([-1., 0., 0.]),
    'a': np.array([0., 1., 0.]),
    'd': np.array([0., -1., 0.]),
    'x': np.array([0., 0., 1.]),
    'z': np.array([0., 0., -1.]),
}
rotBindings = {
    'j': np.array([1., 0., 0.]),
    'l': np.array([-1., 0., 0.]),
    'i': np.array([0., 1., 0.]),
    'k': np.array([0., -1., 0.]),
    'q': np.array([0., 0., 1.]),
    'e': np.array([0., 0., -1.]),
}

speedBindings={
    '1': np.array([-0.1, 0]),
    '2': np.array([0.1, 0]),
    '3': np.array([0, -0.1 * deg2rad]),
    '4': np.array([0, 0.1 * deg2rad])
}

class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('/target_pose', Pose, queue_size = 1)
        self.xyz = np.array([0.0, 0.0, 1.0])
        self.rpy = np.array([0.0, 0.0, 0.0])
        self.trans_speed = 0.1
        self.rot_speed = 0.5 * deg2rad
        
        self.condition = threading.Condition()
        self.done = False

        self.start()

    def printSpeed(self):
        print("trans speed: {}, rot speed: {}".format(self.trans_speed, self.rot_speed * rad2deg))
    
    def printPose(self):
        print("pose xyz: [{}, {}, {}]\npose rpy: [{}, {}, {}]".format(self.xyz[0], self.xyz[1], \
            self.xyz[2], self.rpy[0], self.rpy[1], self.rpy[2]))

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, dxyz, drpy, dspeed):
        self.condition.acquire()
        R = tts.euler_matrix(self.rpy[0], self.rpy[1], self.rpy[2], 'rxyz')[:3, :3]
        self.xyz += self.trans_speed * np.matmul(R, dxyz.reshape((3, 1))).reshape((3,))
        self.rpy += self.rot_speed * drpy
        self.rpy[2] = angles.normalize_angle_positive(self.rpy[2])
        self.rpy[:2] = np.clip(self.rpy[:2], deg2rad * np.array([-30, -30]), deg2rad * np.array([30, 30]))
        self.trans_speed += dspeed[0]
        self.rot_speed += dspeed[1]
        # Notify publish thread that we have a new message.
        self.printPose()
        self.printSpeed()
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(notMove, notMove, [0, 0])
        self.join()

    def run(self):
        pose = Pose()
        while not self.done:
            self.condition.acquire()
            self.condition.wait()

            # Copy state into twist message.
            pose.position.x = self.xyz[0]
            pose.position.y = self.xyz[1]
            pose.position.z = self.xyz[2]
            q = tts.quaternion_from_euler(self.rpy[0], self.rpy[1], self.rpy[2], 'rxyz')
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            self.condition.release()

            # Publish.
            self.publisher.publish(pose)

    def excite_rp(self):
        roll = deg2rad * np.random.uniform(-20, 20, 10)
        pitch = deg2rad * np.random.uniform(-20, 20, 10)
        roll = np.append(roll, 0)
        pitch = np.append(pitch, 0)

        for k in range(len(roll)):
            p = Pose()
            p.position.x = self.xyz[0]
            p.position.y = self.xyz[1]
            p.position.z = self.xyz[2]
            q = tts.quaternion_from_euler(roll[k], pitch[k], 0, 'rxyz')
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]

            self.publisher.publish(p)
            rospy.sleep(1.2)
        
        self.rpy = np.zeros(3,)
        rospy.logwarn("finished!!")


def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    rec_srv = None
    if len(sys.argv) > 1 and sys.argv[1] == "getdata":
        rec_srv = rospy.ServiceProxy('/get_data', Empty)
        rec_srv.wait_for_service()

    pub_thread = PublishThread()

    try:
        pub_thread.wait_for_subscribers()

        print(HelpInfo)
        pub_thread.printPose()
        pub_thread.printSpeed()

        while(1):
            key = getKey(key_timeout)
            if key in transBindings.keys():
                pub_thread.update(transBindings[key], notMove, [0, 0])
            elif key in rotBindings.keys():
                pub_thread.update(notMove, rotBindings[key], [0, 0])
            elif key in speedBindings.keys():
                pub_thread.update(notMove, notMove, speedBindings[key]) 
            elif key == 'm':
                pub_thread.excite_rp()
            elif key == 'r' and rec_srv is not None:
                rec_srv.call(EmptyRequest())
                print('call get data!!')
            else:
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    finally:
        pub_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
