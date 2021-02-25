//
// Created by han on 2021/2/25.
//

#include "song_ros_control/wbc/OSC/OscTrackingDataRaw.h"

using std::cout;
using std::endl;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::Isometry3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;
using std::vector;


ComTrackingDataRaw::ComTrackingDataRaw(const std::string& name, const Eigen::MatrixXd& K_p,
                                       const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
                                       const drake::multibody::MultibodyPlant<double>& plant):
                                       name_(name),
                                       K_p_(K_p),
                                       K_d_(K_d),
                                       W_(W),
                                       plant_(plant),
                                       world_(plant.world_frame()){}

bool ComTrackingDataRaw::Update(const Eigen::VectorXd& x,
                           const drake::systems::Context<double>& context,
                           const Eigen::Vector3d& position_d)
{
    std::cout<<1<<std::endl;
    y_des_ = position_d;
    ydot_des_ = Eigen::Vector3d(0, 0, 0);
    yddot_des_ = Eigen::Vector3d(0, 0, 0);

    std::cout<<2<<std::endl;
    // update y and error
    //y_ = plant_.CalcCenterOfMassPosition(context);

    std::cout<<22<<std::endl;
    error_y_ = y_des_ - y_;

    std::cout<<3<<std::endl;
    // update doty and error
    MatrixXd J_w_spr(3, plant_.num_velocities());
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
            context, JacobianWrtVariable::kV, world_, world_,
            &J_w_spr);
    ydot_ = J_w_spr * x.tail(plant_.num_velocities());
    error_ydot_ = ydot_des_ - ydot_;
    std::cout<<4<<std::endl;
    // update yddotdes
    yddot_des_converted_ = yddot_des_;

    std::cout<<5<<std::endl;
    // update J
    J_ = MatrixXd::Zero(3, plant_.num_velocities());
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
            context, JacobianWrtVariable::kV, world_, world_, &J_);

    std::cout<<6<<std::endl;
    //updtae jdotv
    JdotV_ = plant_.CalcBiasCenterOfMassTranslationalAcceleration(
            context, JacobianWrtVariable::kV, world_, world_);

    yddot_command_ =
            yddot_des_converted_ + K_p_ * (error_y_) + K_d_ * (error_ydot_);

    return true;

}

void ComTrackingDataRaw::PrintFeedbackAndDesiredValues(const VectorXd &dv) {
    cout << name_ << ":\n";
    cout << "  y = " << y_.transpose() << endl;
    cout << "  y_des = " << y_des_.transpose() << endl;
    cout << "  error_y = " << error_y_.transpose() << endl;
    cout << "  ydot = " << ydot_.transpose() << endl;
    cout << "  ydot_des = " << ydot_des_.transpose() << endl;
    cout << "  error_ydot_ = " << error_ydot_.transpose() << endl;
    cout << "  yddot_des_converted = " << yddot_des_converted_.transpose()
         << endl;
    cout << "  yddot_command = " << yddot_command_.transpose() << endl;
    cout << "  yddot_command_sol = " << (J_ * dv + JdotV_).transpose() << endl;
}

void ComTrackingDataRaw::SaveYddotCommandSol(const Eigen::VectorXd& dv)
{
    yddot_command_sol_ = J_ * dv + JdotV_;
}
