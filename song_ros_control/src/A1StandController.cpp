//
// Created by han on 2021/2/24.
//

#include "song_ros_control/A1StandController.h"

A1StandController::A1StandController()
{
    memset(&global_state_, 0, sizeof(song_msgs::MotorCmd));
    memset(&last_state_, 0, sizeof(song_msgs::MotorState));
}

A1StandController::~A1StandController()
{
    sub_cmd_.shutdown();
}

bool A1StandController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    name_space = n.getNamespace();

    std::string path = ros::package::getPath("song_ros_control");
    std::string urdf_path = path + "/urdf/a1.urdf";

    plant_ = std::make_shared<drake::multibody::MultibodyPlant<double>>(0.0);
    addA1Multibody(plant_.get(), urdf_path);
    plant_->Finalize();

    context_ = plant_->CreateDefaultContext();

    osc_ = std::make_shared<OscStandController>(plant_, context_.get());

    double w_contact_relax = 20000;

    osc_->SetWeightOfSoftContactConstraint(w_contact_relax);

    double mu = 0.8;
    osc_->SetContactFriction(mu);

    auto left_toe = LeftToeFront(*plant_);
    auto left_heel = LeftToeRear(*plant_);
    auto right_toe = RightToeFront(*plant_);
    auto right_heel = RightToeRear(*plant_);


    osc_->AddContactPoint(left_toe);
    osc_->AddContactPoint(left_heel);
    osc_->AddContactPoint(right_toe);
    osc_->AddContactPoint(right_heel);

    int n_v = plant_->num_velocities();
    Eigen::MatrixXd Q_accel = 0.01 * Eigen::MatrixXd::Identity(n_v, n_v);
    osc_->SetAccelerationCostForAllJoints(Q_accel);

    urdf::Model urdf; // Get URDF info about joint
    if (!urdf.initParamWithNodeHandle("robot_description", n)){
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    n.getParam("joint_name_list", joint_name_list_);
    n.getParam("W_com", w_com_);
    W_com_ = Eigen::Map<Eigen::Matrix3d>(w_com_.data());

    n.getParam("W_pelvis", w_pelvis_);
    W_pelvis_ = Eigen::Map<Eigen::Matrix3d>(w_pelvis_.data());

    n.getParam("K_p_com", k_p_com_);
    K_p_com_ = Eigen::Map<Eigen::Matrix3d>(k_p_com_.data());

    n.getParam("K_d_com", k_d_com_);
    K_d_com_ = Eigen::Map<Eigen::Matrix3d>(k_d_com_.data());

    n.getParam("K_p_pelvis", k_p_pelvis_);
    K_p_pelvis_ = Eigen::Map<Eigen::Matrix3d>(k_p_pelvis_.data());

    n.getParam("K_d_pelvis", k_d_pelvis_);
    K_d_pelvis_ = Eigen::Map<Eigen::Matrix3d>(k_d_pelvis_.data());


    ComTrackingData center_of_mass_traj("com_traj", K_p_com_, K_d_com_,
                                        W_com_,
                                        *plant_);
    osc_->AddAllLegTrackingData(&center_of_mass_traj);

    RotTaskSpaceTrackingData pelvis_rot_traj(
            "base_rot_traj", K_p_pelvis_, K_d_pelvis_,
            W_pelvis_, *plant_);
    pelvis_rot_traj.AddFrameToTrack("base");
    osc_->AddAllLegTrackingData(&pelvis_rot_traj);

    osc_->Build();


    for (int i = 0; i < joint_name_list_.size(); ++i) {
        ROS_INFO_STREAM(joint_name_list_[i] << " \n ");
        joint_name_to_index_[joint_name_list_[i]] = i;

        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name_list_[i]);
        if (!joint_urdf) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name_list_[i].c_str());
            return false;
        }
        hardware_interface::JointHandle joint = robot->getHandle(joint_name_list_[i]);
        joint_list_.push_back(joint);
    }

    sub_cmd_ = n.subscribe("/ground_truth/state", 20, &A1StandController::inertial_cb, this);

    robot_status_pub_ = std::make_unique<realtime_tools::RealtimePublisher<song_msgs::MotorState>>(
            n, name_space + "/state", 1);
    return true;
}

void A1StandController::starting(const ros::Time& time)
{
    for (int i = 0; i < joint_name_list_.size(); ++i) {
        global_state_.pose.pose.orientation.w = 1;
        global_state_.pose.pose.position.x = -0.006337;
        global_state_.pose.pose.position.y = 0.00082725;
        global_state_.pose.pose.position.z = 0.278652;
        last_state_.q[i] = 0.0;
        last_state_.dq[i] = 0.0;
        last_state_.tau[i] = 0.0;
        last_state_.joint_name[i] = joint_name_list_[i];
    }
    command_.initRT(global_state_);
}

void A1StandController::inertial_cb(const nav_msgs::OdometryConstPtr& msg)
{
    global_state_.pose.pose.orientation = msg->pose.pose.orientation;
    global_state_.pose.pose.position = msg->pose.pose.position;

    global_state_.twist.twist.angular = msg->twist.twist.angular;
    global_state_.twist.twist.linear = msg->twist.twist.linear;

    command_.writeFromNonRT(global_state_);
}

void A1StandController::update(const ros::Time& time, const ros::Duration& period)
{
    global_state_ = *(command_.readFromRT());

    /*for (int i = 0; i != 12; ++i){
        joint_list_[joint_name_to_index_[last_cmd_.joint_name[i]]].setCommand(last_cmd_.tau[i]);
    }*/

    if(robot_status_pub_ && robot_status_pub_->trylock()){
        for (int i = 0; i != 12; ++i){
            last_state_.joint_name[i] = joint_name_list_[i];
            last_state_.q[i] = joint_list_[i].getPosition();
            last_state_.dq[i] = joint_list_[i].getVelocity();
            last_state_.tau[i] = joint_list_[i].getEffort();
        }
        robot_status_pub_->msg_.q = last_state_.q;
        robot_status_pub_->msg_.dq = last_state_.dq;
        robot_status_pub_->msg_.tau = last_state_.tau;
        //robot_status_pub_->msg_ = time;
        robot_status_pub_->msg_.header.stamp = time;
        robot_status_pub_->unlockAndPublish();
    }
}

PLUGINLIB_EXPORT_CLASS(A1StandController, controller_interface::ControllerBase);
