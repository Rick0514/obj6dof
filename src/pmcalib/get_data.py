import os
import rospy, rospkg
import json
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg as LivoxMsg
from nav_msgs.msg import Odometry

import pcl_ros

# get multiple lidar data
class GetData:
    
    def __init__(self):
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('obj6dof')

        src_path = '{}/src/pmcalib'.format(pkg_path)
        self.save_dir = '{}/../pcd/pm'
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

        self.odom_file = open('{}/odom.txt'.format(self.save_dir), 'w')

        self.lidar_topic = None
        self.conf = None
        with open(os.path.join(src_path, 'config.json'), 'r') as f:
            self.conf = json.load(f)
            # print(conf)
            self.lidar_topic = self.conf['lidars_topic']

        self.sub = []
        self.pc = []
        self.odom = None
        self.cnt = 0
        pctype = PointCloud2 if self.conf['pctype'] else LivoxMsg
        for i in range(len(self.lidar_topic)):
            self.pc.append(pctype())
            self.sub.append(
                rospy.Subscriber(self.lidar_topic[i], pctype, callback=self.make_pc_callback(self.pc[i]), queue_size=1)
            )

        self.sub.append(rospy.Subscriber(self.conf['odom'], Odometry, callback=self.odom_callback, queue_size=10))

        self.server = rospy.Service('/get_data', Empty, self.handler)


    def make_pc_callback(self, pc):
        def cb(msg):
            pc = msg
        return cb
    
    def odom_callback(self, msg):
        self.odom = msg

    def handler(self, req):
        # save pc and odom to file
        # odom to numpy
        # self.odom : Odometry
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        self.odom_file.write('{} {} {} {} {} {} {}\n', q.x, q.y, q.z, q.w, p.x, p.y, p.z)
        self.odom_file.flush()
        
        # save pc
        for i in range(len(self.pc)):
            # strip '/'
            pc_name = '{}/{}_{}.pcd'.format(self.save_dir, self.lidar_topic[i].strip('/'), self.cnt)
            pcl_ros.point_cloud2.msg_to_pointcloud2(self.pc[i]).to_file(pc_name)

        self.cnt += 1 

        return EmptyResponse()

if __name__ == '__main__':

    rospy.init_node('get_data_node')
    
    g = GetData()

    rospy.spin()