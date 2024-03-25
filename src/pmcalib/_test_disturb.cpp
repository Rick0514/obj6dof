#include "disturb.hpp"

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

using Pose6 = Eigen::Isometry3d;
using V3 = Eigen::Vector3d;
using Qd = Eigen::Quaterniond;

double to_rad = M_PI / 180.0; 


void checkTransform()
{
    AddNoiseToPose antp;

    Pose6 po;
    po.setIdentity();

    double* arr = AddNoiseToPose::pose2Array(po);
    
    vector<float> v1{0, 0, 0, 1, 0, 0, 0};
    for(int i=0; i<7; i++){
        checkNear("check identity pose to arr", v1[i], arr[i]);
    }

    // axis is must be normalized!!!!!!!!
    Eigen::AngleAxisd aga(0.3, Eigen::Vector3d(1, 2, 3).normalized());
    po.linear() = aga.toRotationMatrix();
    po.translation() = Eigen::Vector3d(1, 1, 1);

    // cout << Qd(po.linear()).coeffs().transpose() << endl;
    arr = AddNoiseToPose::pose2Array(po);
    Pose6 po1 = AddNoiseToPose::array2Pose(arr);
    // for(int i=0; i<4; i++)  cout << arr[i] << " ";
    // cout << endl;

    for(int i=0; i<16; i++){
        checkNear(string("turn back trans ") + to_string(i), po.matrix().data()[i], po1.matrix().data()[i]);
    }

    // cout << "-- po -- \n" << po.matrix() << endl;
    // cout << "-- po1 -- \n" << po1.matrix() << endl;
}

void checkGN()
{
    AddNoiseToPose antp;

    for(int i=0; i<10; i++){
        cout << antp.getNoise(0) << " ";
    }
    cout << endl;
}

void checkAddNoiseToFrame()
{
    double ang_std = 10.0 * to_rad;
    double trans_std = 0.05;

    AddNoiseToPose antp(ang_std, trans_std);

    Eigen::AngleAxisd aga(45.0 * to_rad, Eigen::Vector3d(1, 1, 1).normalized());
    Pose6 p0;
    p0.setIdentity();
    p0.linear() = aga.toRotationMatrix();

    double* src = AddNoiseToPose::pose2Array(p0);
    double* dst = antp.addNoiseToAngleAndTrans(src);

    tf2_ros::StaticTransformBroadcaster broadcaster;
    
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "map";
    ts.child_frame_id = "src";
    ts.transform.rotation.x = src[0];
    ts.transform.rotation.y = src[1];
    ts.transform.rotation.z = src[2];
    ts.transform.rotation.w = src[3];
    ts.transform.translation.x = src[4];
    ts.transform.translation.y = src[5];
    ts.transform.translation.z = src[6];

    broadcaster.sendTransform(ts);

    ts.child_frame_id = "dst";
    ts.transform.rotation.x = dst[0];
    ts.transform.rotation.y = dst[1];
    ts.transform.rotation.z = dst[2];
    ts.transform.rotation.w = dst[3];
    ts.transform.translation.x = dst[4];
    ts.transform.translation.y = dst[5];
    ts.transform.translation.z = dst[6];

    broadcaster.sendTransform(ts);
    ros::spin();

}

int main(int argc, char *argv[])
{
    // checkTransform();
    // checkGN();

    ros::init(argc, argv, "static_brocast");
    checkAddNoiseToFrame();

    return 0;
}
