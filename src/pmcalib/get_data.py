import os
import rospy, rospkg
import json
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg as LivoxMsg

from nav_msgs.msg import Odometry

# get multiple lidar data
class GetData:
    
    def __init__(self):
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('obj6dof')

        src_path = '{}/src/pmcalib'.format(pkg_path)

        lidar_topic = None
        self.conf = None
        with open(os.path.join(src_path, 'config.json'), 'r') as f:
            self.conf = json.load(f)
            # print(conf)
            lidar_topic = self.conf['lidars_topic']

        self.sub = []
        self.pc = []
        pctype = PointCloud2 if self.conf['pctype'] else LivoxMsg
        for i in range(len(lidar_topic)):
            self.pc.append(pctype())
            self.sub.append(
                rospy.Subscriber(lidar_topic[i], pctype, callback=self.make_pc_callback(self.pc[i]), queue_size=1)
            )

        self.sub.append(rospy.Subscriber(self.conf['odom'], Odometry, callback=self.odom_callback, queue_size=10))

        self.server = rospy.Service('/get_data', Empty, self.handler)


    def make_pc_callback(self, pc):
        def cb(msg):
            pc = msg
        return cb
    
    def odom_callback(self, msg):
        pass
        
    def handler(self, req):
        pass


GetData()