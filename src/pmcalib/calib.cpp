#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/bundled/ranges.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>

#include <ceres/ceres.h>

#include "disturb.hpp"

static double to_rad = M_PI / 180.0; 

// parse bag
std::shared_ptr<spdlog::logger> lg;
std::string pkg_dir;

using Pose6 = Eigen::Isometry3d;
using V3 = Eigen::Vector3d;
using Qd = Eigen::Quaterniond;
using plane_t = Eigen::Vector4d;

using pt_t = pcl::PointXYZI;
using pc_t = pcl::PointCloud<pt_t>;
using pc_ptr = pc_t::Ptr;

template <typename T>
using vec_t = std::vector<T>;

template <typename T>
using vvec_t = std::vector<std::vector<T>>;

int dg_rb_size = 0;

struct OptimParams
{
    // the first one is ext
    OptimParams(int n) {
        poses.resize(n+1);
        for(int i=0; i<n+1; i++){
            poses[i] = new double[7];
        }
    }

    // OptimParams(int n, ceres::Problem& pro_) : pro(pro_)
    // {
    //     poses.resize(n+1);
    //     for(int i=0; i<n+1; i++){
    //         poses[i] = new double[7];
    //         pro.AddParameterBlock(poses[i], 4, lq);
    //         pro.AddParameterBlock(&(poses[i][4]), 3);
    //     }
    // }

    void initParams(const YAML::Node& conf, const vec_t<Pose6>& v_odom)
    {
        auto gt_ext = conf["gt_ext"].as<vec_t<double>>();
        auto b2l_ext = conf["b2l_ext"].as<vec_t<double>>();

        Pose6 b2l = AddNoiseToPose::array2Pose(b2l_ext.data());

        AddNoiseToPose ap(10.0 * to_rad, 0.1);
        // AddNoiseToPose ap(0, 0);

        double* tmp = ap.addNoiseToAngleAndTrans(gt_ext.data());
        std::memcpy(poses[0], tmp, 7 * sizeof(double));

        AddNoiseToPose ap1(10.0 * to_rad, 0.1);
        // AddNoiseToPose ap1(0, 0);
        for(int i=0; i<v_odom.size(); i++){
            tmp = ap1.addNoiseToAngleAndTrans(AddNoiseToPose::pose2Array(v_odom[i] * b2l));
            std::memcpy(poses[i+1], tmp, 7 * sizeof(double));
        }
    }

    Pose6 getPose(int idx, bool is_src){
        
        Pose6 pos;
        pos.setIdentity();
        
        Qd q = Eigen::Map<Qd>(poses[idx]);
        pos.linear() = q.toRotationMatrix();
        pos.translation() = Eigen::Map<V3>(&(poses[idx][4]));

        if(!is_src){
            return pos * getPose(0, true);    
        }

        return pos;
    }

    // ceres::Problem& pro;
    vec_t<double*> poses;
};

struct SrcICPError
{
    SrcICPError(const plane_t& pl_, const pt_t& pt_) : pl(pl_), pt(pt_) {}

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const plane_t& pl_, const pt_t& pt_) {
        return (new ceres::AutoDiffCostFunction<SrcICPError, 1, 4, 3>(new SrcICPError(pl_, pt_)));
    }

    template <typename T>
    bool operator()(const T* const q_, const T* const t_, T* residuals) const {
        // pose 0~3 q, 4~6, t
        Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(q_);
        Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(t_);

        Eigen::Matrix<T,4,1> ep; ep << T(pt.x), T(pt.y), T(pt.z), T(1.0);
        // ep.block<3, 1>(0) = q * ep.block<3, 1>(0) + t;
        ep.template head<3>() = q * ep.template head<3>() + t;

        residuals[0] = ep.dot(pl.cast<T>());

        return true;
    }

    plane_t pl;
    pt_t pt;
};

struct TarICPError
{
    TarICPError(const plane_t& pl_, const pt_t& pt_) : pl(pl_), pt(pt_) {}

    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const plane_t& pl_, const pt_t& pt_) {
        return (new ceres::AutoDiffCostFunction<TarICPError, 1, 4, 3, 4, 3>(new TarICPError(pl_, pt_)));
    }

    template <typename T>
    bool operator()(const T* const q1_, const T* const t1_, const T* const q2_, const T* const t2_, T* residuals) const {
        // pose 0~3 q, 4~6, t
        Eigen::Quaternion<T> q1 = Eigen::Map<const Eigen::Quaternion<T>>(q1_);
        Eigen::Matrix<T,3,1> t1 = Eigen::Map<const Eigen::Matrix<T,3,1>>(t1_);

        Eigen::Quaternion<T> q2 = Eigen::Map<const Eigen::Quaternion<T>>(q2_);
        Eigen::Matrix<T,3,1> t2 = Eigen::Map<const Eigen::Matrix<T,3,1>>(t2_);

        q2 = q1 * q2;
        t2 = q1 * t2 + t1;

        Eigen::Matrix<T,4,1> ep; ep << T(pt.x), T(pt.y), T(pt.z), T(1.0);
        // ep.head<3>() = q2 * ep.head<3>() + t2;
        ep.template head<3>() = q2 * ep.template head<3>() + t2;

        residuals[0] = ep.dot(pl.cast<T>());

        return true;
    }

    plane_t pl;
    pt_t pt;
};

template<typename T, int NUM_MATCH_POINTS>
bool esti_plane(plane_t &pca_result, const vec_t<pt_t> &point, const T &threshold)
{
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }

    return true;
}

void addResidualBlock(int idx, pc_ptr& pc, pcl::KdTreeFLANN<pt_t>& kdtree, OptimParams& op, ceres::Problem& pro, bool is_src)
{
    Pose6 initpose = op.getPose(idx, is_src);

    for(const auto& pt : pc->points)
    {
        // to world
        pt_t wpt = pcl::transformPoint<pt_t>(pt, initpose.cast<float>());

        std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        kdtree.nearestKSearch(wpt, 5, k_indices, k_sqr_distances);

        if(k_indices.size() == 5)
        {
            plane_t pabcd;
            vec_t<pt_t> points_near(5);
            for(int i=0; i<5; i++)  points_near[i] = kdtree.getInputCloud()->at(k_indices[i]);

            if (esti_plane<float, 5>(pabcd, points_near, 0.1))   //(planeValid)
            {
                float pd2 = pabcd(0) * wpt.x + pabcd(1) * wpt.y + pabcd(2) * wpt.z + pabcd(3);
                V3 p_body(pt.x, pt.y, pt.z);
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

                if (s > 0.9) {
                    dg_rb_size++;

                    if(is_src){
                        pro.AddResidualBlock(SrcICPError::Create(pabcd, pt), nullptr, op.poses[idx], &(op.poses[idx][4]));
                    }else{
                        pro.AddResidualBlock(TarICPError::Create(pabcd, pt), nullptr, op.poses[idx], &(op.poses[idx][4]), op.poses[0], &(op.poses[0][4]));
                    }
                }
            }
        }
    }
}

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

    float ds_size;
    ds_size = conf_yml["ds_size"].as<float>();
    pcl::VoxelGrid<pt_t> ds;
    ds.setLeafSize(ds_size, ds_size, ds_size);

    // 1. load gt map and make kdtree
    pc_t::Ptr global_map = pcl::make_shared<pc_t>();
    std::string data_dir = pkg_dir + conf_yml["pcd_file"].as<std::string>();
    pcl::io::loadPCDFile<pt_t>(data_dir, *global_map);
    ds.setInputCloud(global_map);
    ds.filter(*global_map);
    pcl::KdTreeFLANN<pt_t> kdtree;
    kdtree.setInputCloud(global_map);

    // 2. parse rosbag and fetch all data
    rosbag::Bag bag;
    data_dir = pkg_dir + conf_yml["bag_file"].as<std::string>();
    bag.open(data_dir, rosbag::bagmode::Read);

    vvec_t<pc_ptr> vv_pc(lidar_topic.size(), vec_t<pc_ptr>{});
    vec_t<Pose6> v_odom;

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

            Pose6 pose = Pose6::Identity();
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
                lg->info("{} get pts: {}", tp, cloud->size());
            }
        }
    }

    // 3. contruct and solve optimization problem
    // 3.1 init ext, test 2 lidars case first
    // prepare data block
    int N = v_odom.size();
    // OptimParams op(N, problem);
    OptimParams op(N);
    op.initParams(conf_yml, v_odom);

    int iterN = conf_yml["iter"].as<int>();

    for(int iter=0; iter<iterN; iter++){   

        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        // options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = true;
        
        ceres::LocalParameterization *lq = new ceres::QuaternionParameterization();

        // add to problem
        for(int i=0; i<N+1; i++){
            problem.AddParameterBlock(op.poses[i], 4, lq);
            problem.AddParameterBlock(&(op.poses[i][4]), 3);
        }
        // remove all residual blocks
        // vec_t<ceres::ResidualBlockId> rb_v;
        // problem.GetResidualBlocks(&rb_v);
        // for(auto& id : rb_v)    problem.RemoveResidualBlock(id);

        for(int i=0; i<N; i++){
            pc_ptr src = vv_pc[0][i];
            pc_ptr tar = vv_pc[1][i];

            // maybe downsample a bit?
            // downsample xxx
            pc_ptr src_ = pcl::make_shared<pc_t>();
            pc_ptr tar_ = pcl::make_shared<pc_t>();
            
            ds.setInputCloud(src);
            ds.filter(*src_);
            ds.setInputCloud(tar);
            ds.filter(*tar_);

            // add residuals
            addResidualBlock(i+1, src_, kdtree, op, problem, true);
            addResidualBlock(i+1, tar_, kdtree, op, problem, false);
        }   

        // lg->info("residual block size: {}", dg_rb_size);
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
    }

    // 4. show result
    rosbag::Bag obag(pkg_dir + conf_yml["out_bag"].as<std::string>(), rosbag::bagmode::Write);
    // 4.1 write globalmap
    ros::Time rt_st(1.0);
    sensor_msgs::PointCloud2 ros_globalmap;
    pcl::toROSMsg(*global_map, ros_globalmap);
    ros_globalmap.header.frame_id = "map";
    ros_globalmap.header.stamp = rt_st;
    obag.write("/globalmap", rt_st, ros_globalmap);
    // 4.2 write all pose path and pc
    double step_time = 0.5;
    // write path
    nav_msgs::Path pa;
    pa.header.frame_id = "map";
    pa.header.stamp = rt_st;

    for(int i=0; i<N; i++){
        Pose6 p1 = op.getPose(i+1, true);
        pc_t pc;
        pcl::transformPointCloud<pt_t>(*(vv_pc[0][i]), pc, p1.matrix().cast<float>());
        sensor_msgs::PointCloud2 rpc;
        pcl::toROSMsg(pc, rpc);
        rpc.header.frame_id = "map";
        rpc.header.stamp = ros::Time(1.0 + i * step_time);
        obag.write(lidar_topic[0], rpc.header.stamp, rpc);

        Pose6 p2 = op.getPose(i+1, false);
        pcl::transformPointCloud<pt_t>(*(vv_pc[1][i]), pc, p2.matrix().cast<float>());
        pcl::toROSMsg(pc, rpc);
        rpc.header.frame_id = "map";
        rpc.header.stamp = ros::Time(1.0 + i * step_time);
        obag.write(lidar_topic[1], rpc.header.stamp, rpc);

        auto& odom = v_odom[i];

        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = lidar_topic[0];
        ps.header.stamp = ros::Time(1.0 + i * step_time);
        ps.pose.position.x = odom.translation().x();
        ps.pose.position.y = odom.translation().y();
        ps.pose.position.z = odom.translation().z();

        Qd q(odom.rotation());
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();

        pa.poses.push_back(ps);
    }
    obag.write("/path", rt_st, pa);

    inf.close();   
    bag.close();
    obag.close();
    
    return 0;
}