//
// Created by han on 2021/2/25.
//

#ifndef SONG_ROS_CONTROL_OSCTRACKINGDATARAW_H
#define SONG_ROS_CONTROL_OSCTRACKINGDATARAW_H


#include <Eigen/Dense>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

class ComTrackingDataRaw
{
public:
    ComTrackingDataRaw(const std::string& name, const Eigen::MatrixXd& K_p,
                       const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
                       const drake::multibody::MultibodyPlant<double>& plant);


    bool Update(const Eigen::VectorXd& x,
                const drake::systems::Context<double>& context,
                const Eigen::Vector3d& position_d);

    void PrintFeedbackAndDesiredValues(const Eigen::VectorXd& dv);

    void SaveYddotCommandSol(const Eigen::VectorXd& dv);
    std::string name_;

    Eigen::MatrixXd K_p_;
    Eigen::MatrixXd K_d_;
    Eigen::MatrixXd W_;

    Eigen::MatrixXd J_;
    Eigen::VectorXd JdotV_;
    Eigen::Vector3d yddot_command_;

    Eigen::Vector3d error_y_;
    Eigen::Vector3d error_ydot_;
    Eigen::Vector3d y_;
    Eigen::Vector3d ydot_;

    // Desired output
    Eigen::Vector3d y_des_;
    Eigen::Vector3d ydot_des_;
    Eigen::Vector3d yddot_des_;
    Eigen::Vector3d yddot_des_converted_;

    // Commanded acceleration after feedback terms

    // Osc solution
    Eigen::VectorXd yddot_command_sol_;

    const drake::multibody::MultibodyPlant<double>& plant_;

    // World frames
    const drake::multibody::BodyFrame<double>& world_;
};


#endif //SONG_ROS_CONTROL_OSCTRACKINGDATARAW_H
