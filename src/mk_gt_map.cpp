#include <iostream>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "pcp.hpp"

using namespace std;

using PType = pcl::PointXYZ;
using PCType = pcl::PointCloud<PType>;

class MkGtMap
{
private:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub, odom_sub;
    ros::Publisher pc_pub;

    // obtain a window of size one sec
    deque<nav_msgs::Odometry> odom_queue;
    deque<sensor_msgs::PointCloud2ConstPtr> pc_queue;

    PCType::Ptr global_map;

    string save_file;
    float grid_size;

    Eigen::Matrix4d T_L_I;

    VoxelDownSample* voxelgrid;

public:
    MkGtMap()
    {
        std::string pkg_dir = ros::package::getPath("obj6dof");
        global_map = pcl::make_shared<PCType>();

        // load param
        nh.param<float>("/mkpm/grid_size", grid_size, 0.2f);
        voxelgrid = new VoxelDownSample(grid_size);

        string lidar_topic, odom_topic;
        nh.param<string>("/mkpm/lidar_topic", lidar_topic, "/scan");
        nh.param<string>("/mkpm/odom_topic", odom_topic, "/odom");
        printf("lidar topic: %s\n", lidar_topic.c_str());

        string save_name;
        nh.param<string>("/mkpm/save_pcd", save_name, "");
        save_file = pkg_dir + save_name;
        printf("save file: %s\n", save_file.c_str());

        pc_sub = nh.subscribe(lidar_topic, 5, &MkGtMap::pc_cb, this);        
        odom_sub = nh.subscribe(odom_topic, 1000, &MkGtMap::odom_cb, this);        
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 5);

        // xyz rpy
        vector<float> ext;
        nh.param<vector<float>>("/mkpm/ext", ext, vector<float>{});
        T_L_I.setIdentity();
        for(int i=0; i<3; i++)  T_L_I(i, 3) = ext[i];
        tf2::Quaternion q;
        q.setRPY(ext[3], ext[4], ext[5]);
        Eigen::Quaterniond eq(q.w(), q.x(), q.y(), q.z());
        T_L_I.topLeftCorner(3, 3) = eq.toRotationMatrix();
    }

    void pc_cb(const sensor_msgs::PointCloud2ConstPtr& ptr)
    {
        static double pc_time = 0;
        // 1s upddate once
        if(ptr->header.stamp.toSec() - pc_time < 0.5)   return;

        pc_queue.push_back(ptr);
        pc_time = ptr->header.stamp.toSec();
        
        while(!pc_queue.empty() && !odom_queue.empty() && pc_queue.front()->header.stamp < odom_queue.front().header.stamp)
            pc_queue.pop_front();
        
        if(pc_queue.empty())    return;

        auto& rptr = pc_queue.front();
        double pc_stamp = rptr->header.stamp.toSec();

        nav_msgs::Odometry o;
        o.header.stamp = rptr->header.stamp;
        auto it = upper_bound(odom_queue.begin(), odom_queue.end(), o, [](const nav_msgs::Odometry& a,
        const nav_msgs::Odometry& b){ return a.header.stamp < b.header.stamp; });

        if(it != odom_queue.begin() && it != odom_queue.end()){
            nav_msgs::Odometry left = *prev(it);
            nav_msgs::Odometry right = *it;

            double lt = left.header.stamp.toSec();
            double rt = right.header.stamp.toSec();
            double r = (pc_stamp - lt) / (rt - lt);

            printf("lt(%f), rt(%f), r(%f)\n", lt, rt, r);
            
            // 1. interpolate
            tf2::Quaternion left_q, right_q;
            tf2::fromMsg(left.pose.pose.orientation, left_q);
            tf2::fromMsg(right.pose.pose.orientation, right_q);
            tf2::Quaternion inter_q = left_q.slerp(right_q, r);

            Eigen::Quaterniond eq(inter_q.w(), inter_q.x(), inter_q.y(), inter_q.z());

            tf2::Vector3 left_tr, right_tr;
            tf2::fromMsg(left.pose.pose.position, left_tr);
            tf2::fromMsg(right.pose.pose.position, right_tr);
            auto tr = left_tr + r * (right_tr - left_tr);
            
            Eigen::Vector3d e_tr(tr.x(), tr.y(), tr.z());
            
            Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
            trans.topLeftCorner(3, 3) = eq.toRotationMatrix();
            trans.topRightCorner(3, 1) = e_tr;

            trans = trans * T_L_I;

            // 2. trans pc to des
            PCType pc;
            pcl::fromROSMsg(*rptr, pc);
            filterPCByRange<PType>(pc, 200.0f);
            pcl::transformPointCloud(pc, pc, trans.cast<float>());
            
            // 3. add to global map
            *global_map += pc;
            printf("before filter pc size: %d, ", global_map->size());
            // down sample preventing duplucate points
            voxelgrid->filter<PType>(global_map, global_map);
            printf("filtered pc size: %d\n", global_map->size());
            fflush(stdout);

            // 4. publish
            sensor_msgs::PointCloud2 vis_pc;
            pcl::toROSMsg(*global_map, vis_pc);
            vis_pc.header.frame_id = "map";
            vis_pc.header.stamp = ros::Time::now();
            pc_pub.publish(vis_pc);
        }else{
            if(odom_queue.empty()){
                cout << "odom queue empty..." << endl;
            }else{
                printf("pc_stamp: %f, odom_front: %f, odom_back: %f\n", pc_stamp,
                    odom_queue.front().header.stamp.toSec(),
                    odom_queue.back().header.stamp.toSec());
                fflush(stdout);
            }
        }
    }

    void odom_cb(const nav_msgs::OdometryConstPtr& ptr){
        if(odom_queue.size() > 100) odom_queue.pop_front();
        odom_queue.push_back(*ptr);
    }

    void savePC()
    {
        voxelgrid->filter<PType>(global_map, global_map);
        pcl::io::savePCDFileBinary(save_file, *global_map);
    }

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mk_gt_map");

    MkGtMap mgm;

    ros::spin();

    mgm.savePC();

    return 0;
}

