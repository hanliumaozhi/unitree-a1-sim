//
// Created by han on 2021/2/20.
//

#include "song_ros_control/A1Controller.h"
A1Controller::A1Controller()
{
    memset(&last_cmd, 0, sizeof(song_msgs::MotorCmd));
}

A1Controller::~A1Controller()
{

}

bool A1Controller::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{

}



