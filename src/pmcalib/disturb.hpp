#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <random>

// check float or double
template <typename T1, typename T2>
void checkNear(std::string s, T1 a, T2 b){
    if(std::abs(a - b) > 1e-9){
        printf("[%s] fail!\n", s.c_str());
        fflush(stdout);
    }
}

class AddNoiseToPose
{
protected:
    using Pose6 = Eigen::Isometry3d;
    using V3 = Eigen::Vector3d;
    using Qd = Eigen::Quaterniond;

    float ang_std, trans_std;

    std::random_device rd{};
    std::mt19937 gen{rd()};

public:

    AddNoiseToPose() : ang_std(0.0), trans_std(0.0) {}

    AddNoiseToPose(float ang_std_, float trans_std_) : ang_std(ang_std_), trans_std(trans_std_) {}

    double getNoise(double std)
    {
        std::normal_distribution<double> d(0.0, std);
        return d(gen);
    }

    // noted: arr should be stored as xyzw xyz
    static Pose6 array2Pose(double* arr)
    {
        Pose6 po;
        Qd q = Eigen::Map<Qd>(arr).normalized();
        
        po.setIdentity();
        po.linear() = q.toRotationMatrix();
        po.translation() = Eigen::Map<V3>(&(arr[4]));
        
        #ifdef TEST_DISTURB
            checkNear("array2pose, q.w", arr[3], q.w());
            checkNear("array2pose, q.x", arr[0], q.x());
        #endif
      
        return po;
    }

    static double* pose2Array(const Pose6& p)
    {
        double* arr = new double[7];

        // note: linear is not the same as rotation!!!!!!
        Qd q(p.linear());
        std::memcpy(arr, q.coeffs().data(), 4 * sizeof(double));
        std::memcpy(&(arr[4]), p.translation().data(), 3 * sizeof(double));

    #ifdef TEST_DISTURB
        checkNear("pose2Array, q.w", arr[3], q.w());
        checkNear("pose2Array, q.x", arr[0], q.x());
    #endif

    #ifdef TEST_DISTURB
        checkNear("pose2Array, x.x", arr[4], p.translation().x());
        checkNear("pose2Array, x.y", arr[5], p.translation().y());
    #endif

        // for(int i=0; i<4; i++)  printf("%f ", q.coeffs().data()[i]);
        // printf("\n");

        return arr;
    }

    double* addNoiseToAngleAndTrans(double* src)
    {
        Qd q = Eigen::Map<Qd>(src);
        Eigen::AngleAxisd src_aa(q);
        double ang = src_aa.angle();
        V3 v = src_aa.axis();

        // add noise
        ang += getNoise(ang_std);
        
        double* dst = new double[7];

        Eigen::AngleAxisd dst_aa(ang, v);
        Qd qd(dst_aa);
        std::memcpy(dst, qd.coeffs().data(), 4 * sizeof(double));
        for(int i=4; i<7; i++){
            dst[i] = src[i] + getNoise(trans_std);
        }

        return dst;
    }
    
    void addNoiseToAngleAxisAndTrans()
    {
        AddNoiseToPose antp;
    }
    
};