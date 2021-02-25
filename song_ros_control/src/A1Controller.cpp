//
// Created by han on 2021/2/20.
//

#include "song_ros_control/A1Controller.h"

#include <memory>
A1Controller::A1Controller()
{
    memset(&last_cmd_, 0, sizeof(song_msgs::MotorCmd));
    memset(&last_state_, 0, sizeof(sensor_msgs::JointState));
}

A1Controller::~A1Controller()
{
    sub_cmd_.shutdown();
}

bool A1Controller::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    name_space = n.getNamespace();

    urdf::Model urdf; // Get URDF info about joint
    if (!urdf.initParamWithNodeHandle("robot_description", n)){
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    n.getParam("joint_name_list", joint_name_list_);

    for (int i = 0; i < joint_name_list_.size(); ++i) {
        ROS_INFO_STREAM(joint_name_list_[i] << " \n ");
        joint_name_to_index_[joint_name_list_[i]] = i;

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name_list_[i]);
        if (!joint_urdf) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name_list_[i].c_str());
            return false;
        }
        hardware_interface::JointHandle joint = robot->getHandle(joint_name_list_[i]);
        joint_list_.push_back(joint);
    }

    sub_cmd_ = n.subscribe("command", 20, &A1Controller::cmd_cb, this);

    robot_status_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::JointState>>(
            n, name_space + "/state", 1);
    return true;
}

void A1Controller::starting(const ros::Time& time)
{
    for (int i = 0; i < joint_name_list_.size(); ++i) {
        last_cmd_.tau[i] = 0.0;
        last_cmd_.joint_name[i] = joint_name_list_[i];
    }
    command_.initRT(last_cmd_);
}

void A1Controller::cmd_cb(const song_msgs::MotorCmdConstPtr& msg)
{
    last_cmd_.tau = msg->tau;
    last_cmd_.joint_name = msg->joint_name;

    command_.writeFromNonRT(last_cmd_);
}

void A1Controller::update(const ros::Time& time, const ros::Duration& period)
{
    last_cmd_ = *(command_.readFromRT());

    for (int i = 0; i != 12; ++i){
        joint_list_[joint_name_to_index_[last_cmd_.joint_name[i]]].setCommand(last_cmd_.tau[i]);
    }

    if(robot_status_pub_ && robot_status_pub_->trylock()){
        last_state_.name.resize(12);
        last_state_.position.resize(12);
        last_state_.velocity.resize(12);
        last_state_.effort.resize(12);
        for (int i = 0; i != 12; ++i){
            last_state_.name[i] = joint_name_list_[i];
            last_state_.position[i] = joint_list_[i].getPosition();
            last_state_.velocity[i] = joint_list_[i].getVelocity();
            last_state_.effort[i] = joint_list_[i].getEffort();
        }
        robot_status_pub_->msg_.position = last_state_.position;
        robot_status_pub_->msg_.velocity = last_state_.velocity;
        robot_status_pub_->msg_.effort = last_state_.effort;
        //robot_status_pub_->msg_ = time;
        robot_status_pub_->msg_.header.stamp = time;
        robot_status_pub_->unlockAndPublish();
    }
}

PLUGINLIB_EXPORT_CLASS(A1Controller, controller_interface::ControllerBase);

