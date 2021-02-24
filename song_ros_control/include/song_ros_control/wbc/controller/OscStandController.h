//
// Created by han on 2021/2/23.
//

#ifndef SONG_WBC_OSCSTANDCONTROLLER_H
#define SONG_WBC_OSCSTANDCONTROLLER_H

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

#include "song_ros_control/wbc/Utils.h"
#include "song_ros_control/wbc/OSC/OscTrackingData.h"
#include "song_msgs/MotorState.h"
#include "song_msgs/MotorCmd.h"
#include "nav_msgs/Odometry.h"

class OscStandController {
public:
    OscStandController(const drake::multibody::MultibodyPlant<double>& plant,
                       drake::systems::Context<double>* context,
                       bool print_info=false);

    void Build();

    void SetWeightOfSoftContactConstraint(double w_soft_constraint) {
        w_soft_constraint_ = w_soft_constraint;
    }

    void SetContactFriction(double mu) { mu_ = mu; }

    // Cost methods
    void SetInputCost(const Eigen::MatrixXd& W) { W_input_ = W; }

    void SetAccelerationCostForAllJoints(const Eigen::MatrixXd& W) {
        W_joint_accel_ = W;
    }

    void AddContactPoint(const ContactData evaluator);

    void AddAllLegTrackingData(OscTrackingData* tracking_data);

    void update(const song_msgs::MotorStatePtr& motor_state, const nav_msgs::OdometryConstPtr& odo_data, song_msgs::MotorCmd& motor_cmd);


private:
    const drake::multibody::MultibodyPlant<double>& plant_;

    const drake::multibody::BodyFrame<double>& world_;

    drake::systems::Context<double>* context_;

    int n_q_;
    int n_v_;
    int n_u_;

    // Size of holonomic constraint and total/active contact constraints
    int n_h_;
    int n_c_;
    int n_c_active_;

    bool is_print_info_;

    // mu_ and w_soft_constraint
    double mu_ = -1;  // Friction coefficients
    double w_soft_constraint_ = -1;

    Eigen::MatrixXd W_input_;        // Input cost weight
    Eigen::MatrixXd W_joint_accel_;  // Joint acceleration cost weight


    std::map<int, std::set<int>> contact_indices_map_ = {};
    // All contacts (used in contact constraints)
    std::vector<ContactData> all_contacts_ = {};

    // MathematicalProgram
    std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
    // Decision variables
    drake::solvers::VectorXDecisionVariable dv_;
    drake::solvers::VectorXDecisionVariable u_;
    drake::solvers::VectorXDecisionVariable lambda_c_;
    drake::solvers::VectorXDecisionVariable lambda_h_;
    drake::solvers::VectorXDecisionVariable epsilon_;
    // Cost and constraints
    drake::solvers::LinearEqualityConstraint* dynamics_constraint_;
    drake::solvers::LinearEqualityConstraint* holonomic_constraint_;
    drake::solvers::LinearEqualityConstraint* contact_constraints_;
    std::vector<drake::solvers::LinearConstraint*> friction_constraints_;

    std::vector<drake::solvers::QuadraticCost*> all_leg_tracking_cost_;

    // OSC solution
    std::unique_ptr<Eigen::VectorXd> dv_sol_;
    std::unique_ptr<Eigen::VectorXd> u_sol_;
    std::unique_ptr<Eigen::VectorXd> lambda_c_sol_;
    std::unique_ptr<Eigen::VectorXd> lambda_h_sol_;
    std::unique_ptr<Eigen::VectorXd> epsilon_sol_;

    // OSC constraint members
    bool with_input_constraints_ = true;

    // robot input limits
    Eigen::VectorXd u_min_;
    Eigen::VectorXd u_max_;

    std::unique_ptr<std::vector<OscTrackingData*>> all_leg_data_vec_ =
            std::make_unique<std::vector<OscTrackingData*>>();

    // Osc checkers and constructor-related methods
    void CheckCostSettings();
    void CheckConstraintSettings();

};


#endif //SONG_WBC_OSCSTANDCONTROLLER_H
