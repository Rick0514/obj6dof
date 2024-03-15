import os
import rospy, rospkg, rosbag
import yaml
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg as LivoxMsg
from nav_msgs.msg import Odometry

# get multiple lidar data
class GetData:
    
    def __init__(self):
        
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('obj6dof')
        print('pkg_path: {}'.format(pkg_path))
        src_path = '{}/src/pmcalib'.format(pkg_path)
        self.save_dir = '{}/../../../pcd/pm'.format(src_path)
        if not os.path.exists(self.save_dir):
            os.mkdir(self.save_dir)

        self.lidar_topic = None
        self.conf = None
        with open(os.path.join(src_path, 'config.yaml'), 'r') as f:
            self.conf = yaml.safe_load(f)
            # print(conf)
            self.lidar_topic = self.conf['lidar_topic']

        self.sub = []
        self.pc = []
        self.odom = None
        self.cnt = 0
        pctype = PointCloud2 if self.conf['pctype'] == 0 else LivoxMsg
        for i in range(len(self.lidar_topic)):
            self.pc.append(pctype())
            self.sub.append(
                rospy.Subscriber(self.lidar_topic[i], pctype, callback=self.make_pc_callback(i), queue_size=1)
            )

        self.sub.append(rospy.Subscriber(self.conf['odom'], Odometry, callback=self.odom_callback, queue_size=10))

        self.server = rospy.Service('/get_data', Empty, self.handler)

        self.bag = rosbag.Bag('{}/test1.bag'.format(self.save_dir), 'w')

    def make_pc_callback(self, idx):
        def cb(msg):
            self.pc[idx] = msg
        return cb
    
    def odom_callback(self, msg):
        self.odom = msg

    def handler(self, req):
        # save odom
        self.bag.write(self.conf['odom'], self.odom)
        # save pc
        for i in range(len(self.pc)):
            self.bag.write(self.lidar_topic[i], self.pc[i])

        self.cnt += 1

        return EmptyResponse()

    def exit(self):
        self.bag.close()

if __name__ == '__main__':

    rospy.init_node('get_data_node')
    
    g = GetData()

    rospy.spin()

    g.exit()

    