/*
 * Copyright (C) 2017 chapulina
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#pragma once

#include <thread>
#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/PID.hh>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ignition/math/Pose3.hh>

namespace gazebo
{

    class PosePID
    {
    private:
        // assume mass is 1.0
        float m_max_acc{5.0};
        float m_max_wacc{M_PI/3};
        
        float xyz_pid_p{10.0};
        float xyz_pid_d{20.0};

        float rpy_pid_p{0.02};
        float rpy_pid_d{0.1};

        std::vector<common::PID> xyzPIDvec;    
        std::vector<common::PID> rpyPIDvec;     // rxyz
        
        typedef ignition::math::Vector3d ig3d;

    public:
        PosePID(){
            common::PID xyz_pid(xyz_pid_p, 0, xyz_pid_d, 0, 0, m_max_acc, -m_max_acc);
            common::PID rpy_pid(rpy_pid_p, 0, rpy_pid_d, m_max_wacc, -m_max_wacc);
            for(int i=0; i<3; i++){
                xyzPIDvec.emplace_back(xyz_pid);
                rpyPIDvec.emplace_back(rpy_pid);
            }  
        }
        
        // force torque
        std::pair<ig3d, ig3d> Update(const ig3d& xyz_err, const ig3d& rpy_err, double dt)
        {
            std::pair<ig3d, ig3d> res;
            for(int i=0; i<3; i++){
                res.first[i] = xyzPIDvec[i].Update(xyz_err[i], dt);
                res.second[i] = rpyPIDvec[i].Update(rpy_err[i], dt);
            }
            return res;
        }
    };

	class GAZEBO_VISIBLE SimMotion : public ModelPlugin
	{
		// Documentation inherited
	public:
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual ~SimMotion();
		/// \brief Main loop to update the pose
	private:
		void OnUpdate();
		void OnRosUpdate();
		void GetTargetPose(const geometry_msgs::PoseConstPtr& msg);
        void ScanRosParam(const ros::WallTimerEvent& e);

		/// \brief All the event connections.
	private:
		std::vector<event::ConnectionPtr> connections;

	private: physics::WorldPtr world;

		/// \brief Pointer to the model
	private:

		physics::ModelPtr model;
        std::string m_link_name;
		
		ros::NodeHandlePtr m_nh_ptr;
		std::unique_ptr<std::thread> m_ros_thread_ptr;
		ros::Subscriber m_pose_sub;

        std::mutex m_pid_lock;
        std::unique_ptr<PosePID> m_pid_ptr;
        
        std::mutex m_pose_lock;
        ignition::math::Pose3d m_target_pose;

        ros::WallTimer m_scan_rosparam_timer;

	};
}
