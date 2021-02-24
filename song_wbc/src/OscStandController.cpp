//
// Created by han on 2021/2/23.
//

#include "song_wbc/controller/OscStandController.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

OscStandController::OscStandController(const drake::multibody::MultibodyPlant<double> &plant,
                                       drake::systems::Context<double> *context,
                                       bool print_info) :
        plant_(plant),
        context_(context),
        world_(plant.world_frame()),
        is_print_info_(print_info) {
    n_q_ = plant_.num_positions();
    n_v_ = plant_.num_velocities();
    n_u_ = plant_.num_actuators();

    VectorXd u_min(n_u_);
    VectorXd u_max(n_u_);
    for (drake::multibody::JointActuatorIndex i(0); i < n_u_; ++i) {
        u_min(i) = -plant_.get_joint_actuator(i).effort_limit();
        u_max(i) = plant_.get_joint_actuator(i).effort_limit();
    }
    u_min_ = u_min;
    u_max_ = u_max;

    /*auto position_dict = makeNameToPositionsMap(plant_);
    for (auto& [key, value]: position_dict) {
        std::cout << key << " has value " << value << std::endl;
    }

    auto input_name = makeNameToActuatorsMap(plant_);
    for (auto& [key, value]: input_name) {
        std::cout << key << " has value " << value << std::endl;
    }*/
    /*
     * L_calf_joint has value 16
FL_hip_joint has value 8
FL_thigh_joint has value 12
FR_calf_joint has value 15
FR_hip_joint has value 7
FR_thigh_joint has value 11
RL_calf_joint has value 18
RL_hip_joint has value 10
RL_thigh_joint has value 14
RR_calf_joint has value 17
RR_hip_joint has value 9
RR_thigh_joint has value 13
base_qw has value 0
base_qx has value 1
base_qy has value 2
base_qz has value 3
base_x has value 4
base_y has value 5
base_z has value 6
FL_calf_motor has value 5
FL_hip_motor has value 3
FL_thigh_motor has value 4
FR_calf_motor has value 2
FR_hip_motor has value 0
FR_thigh_motor has value 1
RL_calf_motor has value 11
RL_hip_motor has value 9
RL_thigh_motor has value 10
RR_calf_motor has value 8
RR_hip_motor has value 6
RR_thigh_motor has value 7
     */
}

void OscStandController::CheckCostSettings() {
    if (W_input_.size() != 0) {
        DRAKE_DEMAND((W_input_.rows() == n_u_) && (W_input_.cols() == n_u_));
    }
    if (W_joint_accel_.size() != 0) {
        DRAKE_DEMAND((W_joint_accel_.rows() == n_v_) &&
                     (W_joint_accel_.cols() == n_v_));
    }
}
void OscStandController::CheckConstraintSettings() {
    if (!all_contacts_.empty()) {
        DRAKE_DEMAND(mu_ != -1);
    }
}

void OscStandController::Build(){
    // Checker
    CheckCostSettings();
    CheckConstraintSettings();

    /*for (auto tracking_data : *tracking_data_vec_) {
        tracking_data->CheckOscTrackingData();
    }*/

    // Construct QP
    prog_ = std::make_unique<drake::solvers::MathematicalProgram>();

    // Size of decision variable
    n_h_ = 0;

    n_c_ = 3 * all_contacts_.size();
    n_c_active_ = 0;
    for (auto evaluator : all_contacts_) {
        n_c_active_ += 3;
    }

    // Initialize solution
    dv_sol_ = std::make_unique<Eigen::VectorXd>(n_v_);
    u_sol_ = std::make_unique<Eigen::VectorXd>(n_u_);
    lambda_c_sol_ = std::make_unique<Eigen::VectorXd>(n_c_);
    lambda_h_sol_ = std::make_unique<Eigen::VectorXd>(n_h_);
    epsilon_sol_ = std::make_unique<Eigen::VectorXd>(n_c_active_);
    dv_sol_->setZero();
    u_sol_->setZero();
    lambda_c_sol_->setZero();
    lambda_h_sol_->setZero();
    epsilon_sol_->setZero();

    // Add decision variables
    dv_ = prog_->NewContinuousVariables(n_v_, "dv");
    u_ = prog_->NewContinuousVariables(n_u_, "u");
    lambda_c_ = prog_->NewContinuousVariables(n_c_, "lambda_contact");
    lambda_h_ = prog_->NewContinuousVariables(n_h_, "lambda_holonomic");
    epsilon_ = prog_->NewContinuousVariables(n_c_active_, "epsilon");

    // Add constraints
    // 1. Dynamics constraint
    dynamics_constraint_ =
            prog_->AddLinearEqualityConstraint(
                            MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_),
                            VectorXd::Zero(n_v_), {dv_, lambda_c_, lambda_h_, u_})
                    .evaluator()
                    .get();

    // 2. Contact constraint
    if (all_contacts_.size() > 0) {
        if (w_soft_constraint_ <= 0) {
            contact_constraints_ =
                    prog_->AddLinearEqualityConstraint(MatrixXd::Zero(n_c_active_, n_v_),
                                                       VectorXd::Zero(n_c_active_), dv_)
                            .evaluator()
                            .get();
        } else {
            // Relaxed version:
            contact_constraints_ =
                    prog_->AddLinearEqualityConstraint(
                                    MatrixXd::Zero(n_c_active_, n_v_ + n_c_active_),
                                    VectorXd::Zero(n_c_active_), {dv_, epsilon_})
                            .evaluator()
                            .get();
        }
    }
    // 3. Friction constraint (approximated friction cone)
    if (!all_contacts_.empty()) {
        VectorXd mu_neg1(2);
        VectorXd mu_1(2);
        VectorXd one(1);
        mu_neg1 << mu_, -1;
        mu_1 << mu_, 1;
        one << 1;
        for (unsigned int j = 0; j < all_contacts_.size(); j++) {
            friction_constraints_.push_back(
                    prog_->AddLinearConstraint(mu_neg1.transpose(), 0,
                                               std::numeric_limits<double>::infinity(),
                                               {lambda_c_.segment(3 * j + 2, 1),
                                                lambda_c_.segment(3 * j + 0, 1)})
                            .evaluator()
                            .get());
            friction_constraints_.push_back(
                    prog_->AddLinearConstraint(mu_1.transpose(), 0,
                                               std::numeric_limits<double>::infinity(),
                                               {lambda_c_.segment(3 * j + 2, 1),
                                                lambda_c_.segment(3 * j + 0, 1)})
                            .evaluator()
                            .get());
            friction_constraints_.push_back(
                    prog_->AddLinearConstraint(mu_neg1.transpose(), 0,
                                               std::numeric_limits<double>::infinity(),
                                               {lambda_c_.segment(3 * j + 2, 1),
                                                lambda_c_.segment(3 * j + 1, 1)})
                            .evaluator()
                            .get());
            friction_constraints_.push_back(
                    prog_->AddLinearConstraint(mu_1.transpose(), 0,
                                               std::numeric_limits<double>::infinity(),
                                               {lambda_c_.segment(3 * j + 2, 1),
                                                lambda_c_.segment(3 * j + 1, 1)})
                            .evaluator()
                            .get());
            friction_constraints_.push_back(
                    prog_->AddLinearConstraint(one.transpose(), 0,
                                               std::numeric_limits<double>::infinity(),
                                               lambda_c_.segment(3 * j + 2, 1))
                            .evaluator()
                            .get());
        }
    }
    // 4. Input constraint
    if (with_input_constraints_) {
        prog_->AddLinearConstraint(MatrixXd::Identity(n_u_, n_u_), u_min_, u_max_,
                                   u_);
    }

    // Add cost
    // 1. input cost
    if (W_input_.size() > 0) {
        prog_->AddQuadraticCost(W_input_, VectorXd::Zero(n_u_), u_);
    }
    // 2. acceleration cost
    if (W_joint_accel_.size() > 0) {
        prog_->AddQuadraticCost(W_joint_accel_, VectorXd::Zero(n_v_), dv_);
    }
    // 3. Soft constraint cost
    if (w_soft_constraint_ > 0) {
        prog_->AddQuadraticCost(
                w_soft_constraint_ * MatrixXd::Identity(n_c_active_, n_c_active_),
                VectorXd::Zero(n_c_active_), epsilon_);
    }

    // 4. All leg Tracking cost
    for (unsigned int i = 0; i < all_leg_data_vec_->size(); i++) {
        all_leg_tracking_cost_.push_back(prog_->AddQuadraticCost(MatrixXd::Zero(n_v_, n_v_),
                                                                 VectorXd::Zero(n_v_), dv_)
                                                 .evaluator()
                                                 .get());
    }
}

void OscStandController::AddContactPoint(const ContactData evaluator)
{
    all_contacts_.push_back(evaluator);
}

void OscStandController::AddAllLegTrackingData(OscTrackingData* tracking_data)
{
    all_leg_data_vec_->push_back(tracking_data);
}

void OscStandController::update(const song_msgs::MotorStatePtr& motor_state, const nav_msgs::OdometryConstPtr& odo_data)
{
    //construction drake data;

}