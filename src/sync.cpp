#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

using namespace std;

static ofstream tum;

void cb(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::ImuConstPtr& imu){
    // t px py pz qx qy qz qw gx gy gz ax ay az
    static double last_time = -1;
    double t = ros::Time::now().toSec();
    if(last_time < 0)   last_time = t;
    if(abs(t - last_time) < 1e-3)   return;

    auto p = odom->pose.pose.position;
    auto q = odom->pose.pose.orientation;
    auto gr = imu->angular_velocity;
    auto ac = imu->linear_acceleration;
    tum << t << " " << p.x << " " << p.y << " " << p.z
        << " " << q.x << " " << q.y << " " << q.z << " " << q.w << " "
        << gr.x << " " << gr.y << " " << gr.z << " "
        << ac.x << " " << ac.y << " " << ac.z << endl;
    
    last_time = t;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_node");
    ros::NodeHandle nh;

    string odom_topic = "/odom_baselink";
    string imu_topic = "/imu/data";

    tum.open("/home/rick/code/ai-imu-dr/data/0.txt");
    if(!tum.is_open()){
        cerr << "tum file is not opened!!" << endl;
        return -1;
    }

    message_filters::Subscriber<nav_msgs::Odometry> sync_odom;
    message_filters::Subscriber<sensor_msgs::Imu> sync_imu;
    typedef message_filters::sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Imu> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(100), sync_odom, sync_imu);

    sync_odom.subscribe(nh, odom_topic, 100);    
    sync_imu.subscribe(nh, imu_topic, 100);
    sync.registerCallback(cb);

    ros::spin();

    tum.close();

    return 0;
}