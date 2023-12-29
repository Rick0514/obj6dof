#include <iostream>
#include <vector>
#include <deque>
#include <ros/ros.h>

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
#include <pcl/filters/voxel_grid.h>

using namespace std;

using PType = pcl::PointXYZ;
using PCType = pcl::PointCloud<PType>;

class MkGtMap
{
private:
    ros::NodeHandle nh;
    ros::Subscriber pc_sub, odom_sub;
    ros::Publisher pc_pub;

    deque<sensor_msgs::PointCloud2ConstPtr> pc_queue;
    deque<nav_msgs::Odometry> odom_queue;

    PCType::Ptr global_map;

    string save_file;
    const float grid_size = 0.5f;

    Eigen::Matrix4d T_L_I;

public:
    MkGtMap()
    {
        pc_sub = nh.subscribe("/lidar_points", 5, &MkGtMap::pc_cb, this);        
        odom_sub = nh.subscribe("/odom", 1000, &MkGtMap::odom_cb, this);        
        pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/globalmap", 1);

        global_map = pcl::make_shared<PCType>();
        save_file = "/home/rick/chore/obj6dof/pcd/gmap.pcd";

        // xyz rpy
        vector<double>ext{0.1, 0.2, 0.3, 0, 0, 0};
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
        if(ptr->header.stamp.toSec() - pc_time < 1.0)   return;

        pc_queue.push_back(ptr);
        pc_time = ptr->header.stamp.toSec();
        
        if(pc_queue.size() > 3){
            nav_msgs::Odometry o;
            o.header.stamp.fromSec(pc_time);
            auto it = upper_bound(odom_queue.begin(), odom_queue.end(), o, [](const nav_msgs::Odometry& a,
            const nav_msgs::Odometry& b){ return a.header.stamp < b.header.stamp; });

            if(it != odom_queue.begin() && it != odom_queue.end()){
                nav_msgs::Odometry left = *prev(it);
                nav_msgs::Odometry right = *it;

                double lt = left.header.stamp.toSec();
                double rt = right.header.stamp.toSec();
                double r = (pc_time - lt) / (rt - lt);

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
                pcl::fromROSMsg(*ptr, pc);
                pcl::transformPointCloud(pc, pc, trans.cast<float>());
                
                // 3. add to global map
                *global_map += pc;

                // 4. publish
                sensor_msgs::PointCloud2 vis_pc;
                pcl::toROSMsg(*global_map, vis_pc);
                vis_pc.header.frame_id = "map";
                vis_pc.header.stamp = ros::Time::now();
                pc_pub.publish(vis_pc);
            }
            
            while(!odom_queue.empty() && pc_time > odom_queue.front().header.stamp.toSec())  odom_queue.pop_front();
            pc_queue.pop_front();

        }
    }

    void odom_cb(const nav_msgs::OdometryConstPtr& ptr){
        odom_queue.push_back(*ptr);
    }

    void savePC()
    {
        pcl::VoxelGrid<PType> voxelgrid;
        voxelgrid.setLeafSize(grid_size, grid_size, grid_size);
        voxelgrid.setInputCloud(global_map);
        voxelgrid.filter(*global_map);
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

