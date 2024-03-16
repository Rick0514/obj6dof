#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/bundled/ranges.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

// parse bag
std::shared_ptr<spdlog::logger> lg;
std::string pkg_dir;

using Pose6d = Eigen::Isometry3d;
using Qd = Eigen::Quaterniond;

using pt_t = pcl::PointXYZI;
using pc_t = pcl::PointCloud<pt_t>;
using pc_ptr = pc_t::Ptr;

template <typename T>
using vec_t = std::vector<T>;

template <typename T>
using vvec_t = std::vector<std::vector<T>>;

int main(int argc, char** argv)
{
    lg = spdlog::default_logger();

    // Get the directory of a specific ROS package
    pkg_dir = ros::package::getPath("obj6dof");
    // remove /.
    if(pkg_dir.size() > 2)    pkg_dir = pkg_dir.substr(0, pkg_dir.size() - 2);
    lg->info("pkg_dir: {}", pkg_dir);

    // 0. read config.yaml
    std::ifstream inf(pkg_dir + "/src/pmcalib/config.yaml");
    YAML::Node conf_yml = YAML::Load(inf);
    auto odom_topic = conf_yml["odom"].as<std::string>();
    auto lidar_topic = conf_yml["lidar_topic"].as<vec_t<std::string>>();
    lg->info("lidar_topic: {}", lidar_topic);

    // 1. load gt map
    // pc_t::Ptr global_map = pcl::make_shared<pc_t>();
    std::string data_dir = pkg_dir + "/../pcd/pm/";
    // pcl::io::loadPCDFile<pt_t>(data_dir + "map.pcd", *global_map);

    // 2. parse rosbag and fetch all data
    rosbag::Bag bag;
    bag.open(data_dir + "test1.bag", rosbag::bagmode::Read);

    vvec_t<pc_ptr> vv_pc(lidar_topic.size(), vec_t<pc_ptr>{});
    vec_t<Pose6d> v_odom;

    vec_t<std::string> topics{odom_topic};
    for(auto&& e : lidar_topic) topics.push_back(e);

    for(const rosbag::MessageInstance& m : rosbag::View(bag, rosbag::TopicQuery(topics)))
    {   
        std::string tp = m.getTopic();
        auto cur_time =  m.getTime();

        if(tp == odom_topic){
            nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();

            auto p = odom->pose.pose.position;
            auto q = odom->pose.pose.orientation;
            Qd eq(q.w, q.x, q.y, q.z);

            Pose6d pose = Pose6d::Identity();
            pose.rotate(eq);
            pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
            v_odom.emplace_back(pose);
            continue;
        }

        for(int i=0; i<lidar_topic.size(); i++){
            if(tp == lidar_topic[i]){
                sensor_msgs::PointCloud2ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
                auto cloud = std::make_shared<pc_t>();
                pcl::fromROSMsg(*msg, *cloud);
                pcl_conversions::toPCL(msg->header.stamp, cloud->header.stamp);
                vv_pc[i].emplace_back(cloud);
            }
        }
    }

    // 3. contruct optimization problem
    

    inf.close();    
    return 0;
}