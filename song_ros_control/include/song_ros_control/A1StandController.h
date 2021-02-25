//
// Created by han on 2021/2/24.
//

#ifndef SONG_ROS_CONTROL_A1STANDCONTROLLER_H
#define SONG_ROS_CONTROL_A1STANDCONTROLLER_H

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <memory>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/package.h>

//#include <song_msgs/MotorCmd.h>
#include <song_msgs/MotorState.h>
#include <nav_msgs/Odometry.h>

#include <pluginlib/class_list_macros.h>

#include "song_ros_control/wbc/controller/OscStandController.h"

#include "drake/multibody/plant/multibody_plant.h"



class A1StandController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    A1StandController();
    ~A1StandController() override;
    bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    virtual void stopping(){};

    void inertial_cb(const nav_msgs::OdometryConstPtr& msg);

private:
    std::string name_space;

    nav_msgs::Odometry global_state_;
    song_msgs::MotorState last_state_;

    nav_msgs::Odometry global_state__;

    ros::Subscriber sub_cmd_;

    std::unique_ptr<realtime_tools::RealtimePublisher<song_msgs::MotorState>> robot_status_pub_;
    std::vector<std::string> joint_name_list_;
    std::vector<hardware_interface::JointHandle> joint_list_;
    std::map<std::string, int> joint_name_to_index_;

    std::vector<std::string> output_order_;

    realtime_tools::RealtimeBuffer<nav_msgs::Odometry> command_;

    std::vector<double> w_com_;
    Eigen::Matrix3d W_com_;

    std::vector<double> k_p_com_;
    Eigen::Matrix3d K_p_com_;

    std::vector<double> k_d_com_;
    Eigen::Matrix3d K_d_com_;

    std::vector<double> w_pelvis_;
    Eigen::Matrix3d W_pelvis_;

    std::vector<double> k_p_pelvis_;
    Eigen::Matrix3d K_p_pelvis_;

    std::vector<double> k_d_pelvis_;
    Eigen::Matrix3d K_d_pelvis_;

    std::shared_ptr<OscStandController> osc_;
    std::shared_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    std::unique_ptr<drake::systems::Context<double>> context_;
};


#endif //SONG_ROS_CONTROL_A1STANDCONTROLLER_H
