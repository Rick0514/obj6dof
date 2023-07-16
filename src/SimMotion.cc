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


#include <ignition/math/Rand.hh>
#include <gazebo/physics/physics.hh>
#include "SimMotion.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SimMotion)


SimMotion::~SimMotion(){
	// Finalize the controller
	this->m_nh_ptr->shutdown();
	if(m_ros_thread_ptr->joinable())	m_ros_thread_ptr->join();
}
/////////////////////////////////////////////////
void SimMotion::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	if (!ros::isInitialized())
	{
		std::string msg = "ros is not init yet!"; 
		ROS_FATAL(msg.c_str());
		throw std::runtime_error(msg);
	}

    m_link_name = "link";   
    if (_sdf->HasElement("link"))
    {
        m_link_name = _sdf->Get<std::string>("link");
    }
    gzmsg << "get link name: " << m_link_name << std::endl;

    m_target_pose.Set(0, 0, 0.5, 0, 0, 0);

	gzmsg << "init pid control" << std::endl;
    m_pid_ptr.reset(new PosePID());

	gzmsg << "loading libSimMotion.so..." << std::endl;
	m_nh_ptr = boost::make_shared<ros::NodeHandle>();
	m_pose_sub = m_nh_ptr->subscribe("/target_pose", 1, &SimMotion::GetTargetPose, this);

	this->world = _model->GetWorld();
	
	this->model = _model;
	this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
		std::bind(&SimMotion::OnUpdate, this)));

    // m_scan_rosparam_timer = m_nh_ptr->createWallTimer(ros::WallDuration(0.2), &SimMotion::ScanRosParam, this);
	m_ros_thread_ptr.reset(new std::thread(&SimMotion::OnRosUpdate, this));
}

/////////////////////////////////////////////
void SimMotion::OnUpdate()
{
    static double last_time{0};
    auto cur_time = this->world->SimTime().Double();
    if(last_time == 0){
        last_time = cur_time;
        return;
    }

    auto dt = cur_time - last_time;
    last_time = cur_time;

	// 1. get error
    auto true_pose = this->model->WorldPose();
    m_pose_lock.lock();
    auto err_xyz = true_pose.Pos() - m_target_pose.Pos();
    auto err_q = m_target_pose.Rot().Inverse() * true_pose.Rot();
    m_pose_lock.unlock();
    ignition::math::Vector3d err_rpy(err_q.Roll(), err_q.Pitch(), err_q.Yaw());

    // 2. update pid to get output
    m_pid_lock.lock();
    auto cmd = m_pid_ptr->Update(err_xyz, err_rpy, dt);
    m_pid_lock.unlock();
    
    // 3. apply force to link
    double g = this->world->Gravity().Z();
    auto link = this->model->GetLink(m_link_name);
    cmd.first.Z() -= g;
    link->AddForce(cmd.first);
    cmd.second = true_pose.Rot() * cmd.second;
    link->AddTorque(cmd.second);
}

void SimMotion::GetTargetPose(const geometry_msgs::PoseConstPtr& msg){
    std::lock_guard<std::mutex> lk(m_pose_lock);
    ignition::math::Vector3d p(msg->position.x, msg->position.y, msg->position.z);
    ignition::math::Quaterniond q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    m_target_pose.Pos() = p;
    m_target_pose.Rot() = q;
}

void SimMotion::OnRosUpdate()
{
	while(m_nh_ptr->ok()){
		ros::spinOnce();
		std::this_thread::yield();
	}
}

void SimMotion::ScanRosParam(const ros::WallTimerEvent& e)
{
    // ros::param::get("/pid/p", m_pid_p);
    // ros::param::get("/pid/d", m_pid_d);

    // auto true_pose = this->model->WorldPose();
    // m_pose_lock.lock();
    // auto diff_pose = m_target_pose.Inverse() * true_pose;
    // m_pose_lock.unlock();

    // gzmsg << "true pose: " << true_pose << std::endl;
    // gzmsg << "diff pose: " << diff_pose << std::endl;

    // std::lock_guard<std::mutex> lk(m_pid_lock);
    // m_z_axis_pid_ptr->SetPGain(m_pid_p);
    // m_z_axis_pid_ptr->SetDGain(m_pid_d);
}