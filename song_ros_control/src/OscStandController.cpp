//
// Created by han on 2021/2/23.
//

#include "song_ros_control/wbc/controller/OscStandController.h"

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;

OscStandController::OscStandController(std::shared_ptr<drake::multibody::MultibodyPlant<double>> plant,
                                       drake::systems::Context<double> *context,
                                       bool print_info) :
        plant_(plant),
        context_(context),
        world_(plant->world_frame()),
        is_print_info_(print_info) {
    n_q_ = plant_->num_positions();
    n_v_ = plant_->num_velocities();
    n_u_ = plant_->num_actuators();

    VectorXd u_min(n_u_);
    VectorXd u_max(n_u_);
    for (drake::multibody::JointActuatorIndex i(0); i < n_u_; ++i) {
        u_min(i) = -plant_->get_joint_actuator(i).effort_limit();
        u_max(i) = plant_->get_joint_actuator(i).effort_limit();
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

void OscStandController::AddAllLegTrackingData(ComTrackingDataRaw* tracking_data)
{
    all_leg_data_vec_->push_back(tracking_data);
}

void OscStandController::update(const song_msgs::MotorStatePtr& motor_state, const nav_msgs::OdometryConstPtr& odo_data, song_msgs::MotorCmd& motor_cmd)
{
    //construction drake data;
    /*Eigen::VectorXd x(7+12);
    Eigen::VectorXd dx(6+12);
    VectorXd x_(plant_->num_positions() + plant_->num_velocities());

    x_ << x, dx;

    // update
    x(0) = odo_data->pose.pose.orientation.w;
    x(1) = odo_data->pose.pose.orientation.x;
    x(2) = odo_data->pose.pose.orientation.y;
    x(3) = odo_data->pose.pose.orientation.z;
    x(4) = odo_data->pose.pose.position.x;
    x(5) = odo_data->pose.pose.position.y;
    x(6) = odo_data->pose.pose.position.z;
    for (int i = 0; i < 12; ++i) {
        x(7+i) = motor_state->q[i];
    }

    dx(0) = odo_data->twist.twist.angular.x;
    dx(1) = odo_data->twist.twist.angular.y;
    dx(2) = odo_data->twist.twist.angular.z;
    dx(3) = odo_data->twist.twist.linear.x;
    dx(4) = odo_data->twist.twist.linear.y;
    dx(5) = odo_data->twist.twist.linear.z;

    for (int i = 0; i < 12; ++i) {
        dx(6+i) = motor_state->dq[i];
    }

    SetPositionsIfNew(*plant_, x, context_);
    SetVelocitiesIfNew(*plant_, dx, context_);

    if (is_print_info_){
        std::cout<<motor_state->header.stamp<<"  "<<odo_data->header.stamp<<std::endl;
    }

    MatrixXd J_c = MatrixXd::Zero(n_c_, n_v_);
    MatrixXd J_c_active = MatrixXd::Zero(n_c_active_, n_v_);
    VectorXd JdotV_c_active = VectorXd::Zero(n_c_active_);

    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
        J_c.block(3 * i, 0,3, n_v_) = get_contact_jacobin(all_contacts_[i], *plant_, *context_);
    }

    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
        auto contact_i = all_contacts_[i];
        J_c_active.block(3 * i, 0,3, n_v_) = J_c.block(3 * i, 0,3, n_v_);
        JdotV_c_active.block(3 * i, 0,3, n_v_) =
                get_contact_jacobinDotTimesV(all_contacts_[i], *plant_, *context_);
    }

    // Get M, f_cg, B matrices of the manipulator equation
    MatrixXd B = plant_->MakeActuationMatrix();
    MatrixXd M(n_v_, n_v_);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
    VectorXd bias(n_v_);
    plant_->CalcBiasTerm(*context_, &bias);
    VectorXd grav = plant_->CalcGravityGeneralizedForces(*context_);
    bias = bias - grav;

    // Update constraints
    // 1. Dynamics constraint
    ///    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_h + B*u
    /// -> M*dv - J_c^T*lambda_c - J_h^T*lambda_h - B*u == - bias
    /// -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_h, u]^T = - bias
    MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_);
    A_dyn.block(0, 0, n_v_, n_v_) = M;
    A_dyn.block(0, n_v_, n_v_, n_c_) = -J_c.transpose();
    //A_dyn.block(0, n_v_ + n_c_, n_v_, n_h_) = -J_h.transpose();
    A_dyn.block(0, n_v_ + n_c_ + n_h_, n_v_, n_u_) = -B;
    dynamics_constraint_->UpdateCoefficients(A_dyn, -bias);

    // 2. Contact constraint
    MatrixXd A_c = MatrixXd::Zero(n_c_active_, n_v_ + n_c_active_);
    A_c.block(0, 0, n_c_active_, n_v_) = J_c_active;
    A_c.block(0, n_v_, n_c_active_, n_c_active_) =
            MatrixXd::Identity(n_c_active_, n_c_active_);
    contact_constraints_->UpdateCoefficients(A_c, -JdotV_c_active);

    // Update Cost

    for (unsigned int i = 0; i < all_leg_data_vec_->size(); i++) {
        auto tracking_data = all_leg_data_vec_->at(i);
        if (tracking_data->GetName() == "com_traj") {
            Vector3d com_position(-0.00633, 0.000827254, 0.2786);
            tracking_data->Update(
                    x_, *context_,
                    drake::trajectories::PiecewisePolynomial<double>(com_position), motor_state->header.stamp.toSec(), -1);
        }

        if (tracking_data->GetName() == "base_rot_traj") {
            VectorXd pelvis_desired_quat(4);
            pelvis_desired_quat << 1, 0, 0, 0;
            tracking_data->Update(
                    x_, *context_,
                    drake::trajectories::PiecewisePolynomial<double>(pelvis_desired_quat), motor_state->header.stamp.toSec(), -1);
        }

        const VectorXd &ddy_t = tracking_data->GetYddotCommand();
        const MatrixXd &W = tracking_data->GetWeight();
        const MatrixXd &J_t = tracking_data->GetJ();
        const VectorXd &JdotV_t = tracking_data->GetJdotTimesV();
        // The tracking cost is
        // 0.5 * (J_*dv + JdotV - y_command)^T * W * (J_*dv + JdotV - y_command).
        // We ignore the constant term
        // 0.5 * (JdotV - y_command)^T * W * (JdotV - y_command),
        // since it doesn't change the result of QP.
        all_leg_tracking_cost_.at(i)->UpdateCoefficients(
                J_t.transpose() * W * J_t, J_t.transpose() * W * (JdotV_t - ddy_t));
    }

    const drake::solvers::MathematicalProgramResult result = Solve(*prog_);

    // Extract solutions
    *dv_sol_ = result.GetSolution(dv_);
    *u_sol_ = result.GetSolution(u_);
    *lambda_c_sol_ = result.GetSolution(lambda_c_);
    *lambda_h_sol_ = result.GetSolution(lambda_h_);
    *epsilon_sol_ = result.GetSolution(epsilon_);

    for (auto tracking_data : *all_leg_data_vec_) {
        if (tracking_data->IsActive()) tracking_data->SaveYddotCommandSol(*dv_sol_);
    }

    motor_cmd.joint_name[0] = "FR_hip_joint";
    motor_cmd.joint_name[1] = "FR_thigh_joint";
    motor_cmd.joint_name[2] = "FR_calf_joint";

    motor_cmd.joint_name[3] = "FL_hip_joint";
    motor_cmd.joint_name[4] = "FL_thigh_joint";
    motor_cmd.joint_name[5] = "FL_calf_joint";

    motor_cmd.joint_name[6] = "RR_hip_joint";
    motor_cmd.joint_name[7] = "RR_thigh_joint";
    motor_cmd.joint_name[8] = "RR_calf_joint";

    motor_cmd.joint_name[9] = "RR_hip_joint";
    motor_cmd.joint_name[10] = "RR_thigh_joint";
    motor_cmd.joint_name[11] = "RR_calf_joint";


    for (int i = 0; i < 12; ++i) {
        motor_cmd.tau[i] = (*u_sol_)(i);
    }*/
}

Eigen::VectorXd OscStandController::update(Eigen::VectorXd x, Eigen::VectorXd dx, Eigen::VectorXd x_, double t)
{
    SetPositionsIfNew(*plant_, x, context_);
    SetVelocitiesIfNew(*plant_, dx, context_);

    std::cout<<111<<std::endl;

    MatrixXd J_c = MatrixXd::Zero(n_c_, n_v_);
    MatrixXd J_c_active = MatrixXd::Zero(n_c_active_, n_v_);
    VectorXd JdotV_c_active = VectorXd::Zero(n_c_active_);

    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
        J_c.block(3 * i, 0,3, n_v_) = get_contact_jacobin(all_contacts_[i], *plant_, *context_);
    }

    for (unsigned int i = 0; i < all_contacts_.size(); i++) {
        auto contact_i = all_contacts_[i];
        J_c_active.block(3 * i, 0,3, n_v_) = J_c.block(3 * i, 0,3, n_v_);
        JdotV_c_active.segment(3 * i, 3) =
                get_contact_jacobinDotTimesV(all_contacts_[i], *plant_, *context_);
    }

    std::cout<<222<<std::endl;

    // Get M, f_cg, B matrices of the manipulator equation
    MatrixXd B = plant_->MakeActuationMatrix();
    MatrixXd M(n_v_, n_v_);
    plant_->CalcMassMatrixViaInverseDynamics(*context_, &M);
    VectorXd bias(n_v_);
    plant_->CalcBiasTerm(*context_, &bias);
    VectorXd grav = plant_->CalcGravityGeneralizedForces(*context_);
    bias = bias - grav;

    // Update constraints
    // 1. Dynamics constraint
    ///    M*dv + bias == J_c^T*lambda_c + J_h^T*lambda_h + B*u
    /// -> M*dv - J_c^T*lambda_c - J_h^T*lambda_h - B*u == - bias
    /// -> [M, -J_c^T, -J_h^T, -B]*[dv, lambda_c, lambda_h, u]^T = - bias
    MatrixXd A_dyn = MatrixXd::Zero(n_v_, n_v_ + n_c_ + n_h_ + n_u_);
    A_dyn.block(0, 0, n_v_, n_v_) = M;
    A_dyn.block(0, n_v_, n_v_, n_c_) = -J_c.transpose();
    //A_dyn.block(0, n_v_ + n_c_, n_v_, n_h_) = -J_h.transpose();
    A_dyn.block(0, n_v_ + n_c_ + n_h_, n_v_, n_u_) = -B;
    dynamics_constraint_->UpdateCoefficients(A_dyn, -bias);

    // 2. Contact constraint
    MatrixXd A_c = MatrixXd::Zero(n_c_active_, n_v_ + n_c_active_);
    A_c.block(0, 0, n_c_active_, n_v_) = J_c_active;
    A_c.block(0, n_v_, n_c_active_, n_c_active_) =
            MatrixXd::Identity(n_c_active_, n_c_active_);
    contact_constraints_->UpdateCoefficients(A_c, -JdotV_c_active);

    std::cout<<333<<std::endl;

    // Update Cost


    for (unsigned int i = 0; i < all_leg_data_vec_->size(); i++) {
        std::cout<<444<<std::endl;
        auto tracking_data = all_leg_data_vec_->at(i);
        std::cout<<10000<<std::endl;
        if (i == 0) {
            std::cout<<555<<std::endl;
            Vector3d com_position(-0.00633, 0.000827254, 0.2786);
            tracking_data->y_ = plant_->CalcCenterOfMassPosition(context_);
            std::cout<<55<<std::endl;
            tracking_data->Update(
                    x_, *context_, com_position);
            std::cout<<666<<std::endl;
        }

        /*std::cout<<888<<std::endl;
        VectorXd ddy_t = tracking_data->yddot_command_;
        MatrixXd W = tracking_data->W_;
        MatrixXd J_t = tracking_data->J_;
        VectorXd JdotV_t = tracking_data->JdotV_;*/
        std::cout<<999<<std::endl;
        // The tracking cost is
        // 0.5 * (J_*dv + JdotV - y_command)^T * W * (J_*dv + JdotV - y_command).
        // We ignore the constant term
        // 0.5 * (JdotV - y_command)^T * W * (JdotV - y_command),
        // since it doesn't change the result of QP.
        std::cout<<tracking_data->J_.transpose() * tracking_data->W_ * tracking_data->J_<<std::endl;
        std::cout<<tracking_data->J_.transpose() * tracking_data->W_ * (tracking_data->JdotV_ - tracking_data->yddot_command_)<<std::endl;
        all_leg_tracking_cost_.at(i)->UpdateCoefficients(
                tracking_data->J_.transpose() * tracking_data->W_ * tracking_data->J_, tracking_data->J_.transpose() * tracking_data->W_ * (tracking_data->JdotV_ - tracking_data->yddot_command_));
    }

    const drake::solvers::MathematicalProgramResult result = Solve(*prog_);

    // Extract solutions
    *dv_sol_ = result.GetSolution(dv_);
    *u_sol_ = result.GetSolution(u_);
    *lambda_c_sol_ = result.GetSolution(lambda_c_);
    *lambda_h_sol_ = result.GetSolution(lambda_h_);
    *epsilon_sol_ = result.GetSolution(epsilon_);

    for (auto tracking_data : *all_leg_data_vec_) {
        tracking_data->SaveYddotCommandSol(*dv_sol_);
    }

    return (*u_sol_);
}