#pragma once

#include <Eigen/Core>
#include <pcl/common/centroid.h>

template <typename T>
using PC = pcl::PointCloud<T>;

template <typename T>
void filterPCByRange(PC<T>& pc, float r)
{
    PC<T> pc_tmp;
    pc_tmp.reserve(pc.size());

    for(const auto& pt : pc.points){
        if(pt.x > -r && pt.x < r &&
           pt.y > -r && pt.y < r && 
           pt.z > -r && pt.z < r)   pc_tmp.push_back(pt);
    }
    pcl::copyPointCloud(pc_tmp, pc);
}

class VoxelDownSample
{
protected:

    float _grid_size;
    float _inverse_grid_size;

    Eigen::Array3f _max_p, _min_p;
    
    const float limit_abnormal_range = 200.0f;
    
public:
    VoxelDownSample() = default;
    VoxelDownSample(float gs) : _grid_size(gs) {
        _inverse_grid_size = 1.0f / _grid_size;
    }

    template <typename PointType>
    void filter(const typename PC<PointType>::ConstPtr& cloud, typename PC<PointType>::Ptr& out)
    {
        if(cloud->empty())  return;

        out->header = cloud->header;
        out->sensor_origin_ = cloud->sensor_origin_;
        out->sensor_orientation_ = cloud->sensor_orientation_;

        _max_p = cloud->points[0].getArray3fMap();
        _min_p = _max_p;

        // find max and min first
        for(const auto& p : cloud->points){
            pcl::Array3fMapConst pt = p.getArray3fMap();
            _max_p = _max_p.max(pt);
            _min_p = _min_p.min(pt);
        }

        Eigen::Array3i max_b, min_b;
        min_b[0] = static_cast<int> (std::floor (_min_p[0] * _inverse_grid_size));
        max_b[0] = static_cast<int> (std::floor (_max_p[0] * _inverse_grid_size));
        min_b[1] = static_cast<int> (std::floor (_min_p[1] * _inverse_grid_size));
        max_b[1] = static_cast<int> (std::floor (_max_p[1] * _inverse_grid_size));
        min_b[2] = static_cast<int> (std::floor (_min_p[2] * _inverse_grid_size));
        max_b[2] = static_cast<int> (std::floor (_max_p[2] * _inverse_grid_size));

        int dx = max_b[0] - min_b[0] + 1;
        int dy = max_b[1] - min_b[1] + 1;

        std::unordered_map<size_t, std::vector<size_t>> _map;

        for(size_t i=0; i<cloud->size(); i++){
            const auto& pt = cloud->points[i];
            size_t ijk0 = static_cast<size_t> (std::floor (pt.x * _inverse_grid_size) - static_cast<float> (min_b[0]));
            size_t ijk1 = static_cast<size_t> (std::floor (pt.y * _inverse_grid_size) - static_cast<float> (min_b[1]));
            size_t ijk2 = static_cast<size_t> (std::floor (pt.z * _inverse_grid_size) - static_cast<float> (min_b[2]));

            size_t idx = ijk0 + ijk1 * dx + ijk2 * dx * dy;

            if(_map.count(idx) == 0){
                _map.emplace(idx, std::vector<size_t>{i});
            }else{
                _map[idx].emplace_back(i);
            }
        }
        
        PC<PointType>* out_ = out.get();
        PC<PointType> out_tmp;

        if(cloud.get() == out.get()){
            printf("equal addr\n");    
            out_ = &out_tmp;
        }

        out_->clear();
        out_->reserve(_map.size());

        for(const auto& [_, vb] : _map){
            pcl::CentroidPoint<PointType> centroid;

            // fill in the accumulator with leaf points
            for (auto&& idx : vb)
                centroid.add(cloud->points[idx]);  
            
            PointType pt;
            centroid.get(pt);
            out_->emplace_back(pt);
        }

        if(cloud.get() == out.get())    pcl::copyPointCloud(out_tmp, *out);
    }

    std::string getMaxMin()
    {
        std::stringstream ss;
        ss << "min: " << _min_p.transpose() << " "
           << "max: " << _max_p.transpose() << std::endl;
        return ss.str();
    }
};
