//
// Created by han on 2021/2/20.
//

#ifndef SONG_ROS_CONTROL_A1CONTROLLER_H
#define SONG_ROS_CONTROL_A1CONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <memory>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>

#include <song_msgs/MotorCmd.h>
#include <song_msgs/MotorState.h>

#include <pluginlib/class_list_macros.h>

class A1Controller: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    A1Controller();
    ~A1Controller() override;
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    virtual void stopping(){};

    void cmd_cb(const song_msgs::MotorCmdConstPtr& msg);

private:
    std::string name_space;

    song_msgs::MotorCmd last_cmd_;
    song_msgs::MotorState last_state_;

    ros::Subscriber sub_cmd_;

    std::unique_ptr<realtime_tools::RealtimePublisher<song_msgs::MotorState>> robot_status_pub_;
    std::vector<std::string> joint_name_list_;
    std::vector<hardware_interface::JointHandle> joint_list_;
    std::map<std::string, int> joint_name_to_index_;

    realtime_tools::RealtimeBuffer<song_msgs::MotorCmd> command_;

};

#endif //SONG_ROS_CONTROL_A1CONTROLLER_H
