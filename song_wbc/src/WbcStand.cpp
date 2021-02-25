//
// Created by han on 2021/2/22.
//

#include <iostream>
#include <ros/package.h>
#include <memory>

#include "song_wbc/controller/OscStandController.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

std::shared_ptr<OscStandController> osc;
ros::Publisher motor_cmd;

void callback(const sensor_msgs::JointStateConstPtr& motor_state, const nav_msgs::OdometryConstPtr& odo_data)
{
    song_msgs::MotorCmd new_cmd;
    osc->update(motor_state, odo_data, new_cmd);
    motor_cmd.publish(new_cmd);
}

int main(int argc, char** argv)
{
    std::string path = ros::package::getPath("song_wbc");
    std::string urdf_path = path + "/urdf/a1.urdf";

    drake::multibody::MultibodyPlant<double> plant(0.0);
    addA1Multibody(&plant, urdf_path);
    plant.Finalize();

    auto context = plant.CreateDefaultContext();

    osc = std::make_shared<OscStandController>(plant, context.get());

    double w_contact_relax = 20000;

    osc->SetWeightOfSoftContactConstraint(w_contact_relax);

    double mu = 0.8;
    osc->SetContactFriction(mu);

    auto left_toe = LeftToeFront(plant);
    auto left_heel = LeftToeRear(plant);
    auto right_toe = RightToeFront(plant);
    auto right_heel = RightToeRear(plant);


    osc->AddContactPoint(left_toe);
    osc->AddContactPoint(left_heel);
    osc->AddContactPoint(right_toe);
    osc->AddContactPoint(right_heel);

    int n_v = plant.num_velocities();
    Eigen::MatrixXd Q_accel = 0.01 * Eigen::MatrixXd::Identity(n_v, n_v);
    osc->SetAccelerationCostForAllJoints(Q_accel);

    Eigen::Matrix3d W_com;
    W_com << 2000, 0, 0, 0, 2000, 0, 0, 0, 1000;

    Eigen::Matrix3d K_p_com;
    K_p_com << 22, 0, 0, 0, 20, 0, 0, 0, 20;
    Eigen::Matrix3d K_d_com;
    K_d_com << 0.5, 0, 0, 0, 0.4, 0, 0, 0, 2;

    ComTrackingData center_of_mass_traj("com_traj", K_p_com, K_d_com,
                                        W_com,
                                        plant);
    osc->AddAllLegTrackingData(&center_of_mass_traj);

    Eigen::Matrix3d W_pelvis;
    W_pelvis << 200, 0, 0, 0, 200, 0, 0, 0, 200;

    Eigen::Matrix3d K_p_pelvis;
    K_p_pelvis << 10, 0, 0, 0, 100, 0, 0, 0, 10;

    Eigen::Matrix3d K_d_pelvis;
    K_d_pelvis << 0.5,  0, 0, 0, 10, 0, 0, 0, 10;

    // Pelvis rotation tracking
    RotTaskSpaceTrackingData pelvis_rot_traj(
            "base_rot_traj", K_p_pelvis, K_d_pelvis,
            W_pelvis, plant);
    pelvis_rot_traj.AddFrameToTrack("base");
    osc->AddAllLegTrackingData(&pelvis_rot_traj);

    osc->Build();

    ros::init(argc, argv, "stand_node");

    ros::NodeHandle nh;

    motor_cmd = nh.advertise<song_msgs::MotorCmd>("/a1_gazebo/A1_controller/command", 1);

    message_filters::Subscriber<sensor_msgs::JointState> state_sub(nh, "/a1_gazebo/A1_controller/state", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odo_sub(nh, "/ground_truth/state", 1);
    message_filters::TimeSynchronizer<sensor_msgs::JointState, nav_msgs::Odometry> sync(state_sub, odo_sub, 2);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}
