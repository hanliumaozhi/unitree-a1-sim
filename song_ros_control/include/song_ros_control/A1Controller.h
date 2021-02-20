//
// Created by han on 2021/2/20.
//

#ifndef SONG_ROS_CONTROL_A1CONTROLLER_H
#define SONG_ROS_CONTROL_A1CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <song_msgs/MotorCmd.h>


class A1Controller: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    A1Controller();
    ~A1Controller();
    virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& period);
    virtual void stopping();

private:
    song_msgs::MotorCmd last_cmd;

};

#endif //SONG_ROS_CONTROL_A1CONTROLLER_H
