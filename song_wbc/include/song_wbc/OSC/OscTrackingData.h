//
// Created by han on 2021/2/23.
//

#ifndef SONG_WBC_OSCTRACKINGDATA_H
#define SONG_WBC_OSCTRACKINGDATA_H


#include <Eigen/Dense>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>


class OscTrackingData {
public:
    static constexpr int kSpaceDim = 3;
    static constexpr int kQuaternionDim = 4;

    OscTrackingData(const std::string& name, int n_y, int n_ydot,
                    const Eigen::MatrixXd& K_p, const Eigen::MatrixXd& K_d,
                    const Eigen::MatrixXd& W,
                    const drake::multibody::MultibodyPlant<double>& plant_);

    // Update() updates the caches. It does the following things in order:
    //  - update track_at_current_state_
    //  - update desired output
    //  - update feedback output (Calling virtual methods)
    //  - update command output (desired output with pd control)
    // Inputs/Arguments:
    //  - `x_w_spr`, state of the robot (with spring)
    //  - `context_w_spr`, plant context of the robot (without spring)
    //  - `x_wo_spr`, state of the robot (with spring)
    //  - `context_wo_spr`, plant context of the robot (without spring)
    //  - `traj`, desired trajectory
    //  - `t`, current time
    //  - `finite_state_machine_state`, current finite state machine state
    bool Update(const Eigen::VectorXd& x,
                const drake::systems::Context<double>& context,
                const drake::trajectories::Trajectory<double>& traj, double t,
                int finite_state_machine_state);

    // Getters for debugging
    const Eigen::VectorXd& GetY() const { return y_; }
    const Eigen::VectorXd& GetYDes() const { return y_des_; }
    const Eigen::VectorXd& GetErrorY() const { return error_y_; }
    const Eigen::VectorXd& GetYdot() const { return ydot_; }
    const Eigen::VectorXd& GetYdotDes() const { return ydot_des_; }
    const Eigen::VectorXd& GetErrorYdot() const { return error_ydot_; }
    const Eigen::VectorXd& GetYddotDes() const { return yddot_des_; }
    const Eigen::VectorXd& GetYddotDesConverted() const {
        return yddot_des_converted_;
    }
    const Eigen::VectorXd& GetYddotCommandSol() const {
        return yddot_command_sol_;
    }

    // Getters used by osc block
    const Eigen::MatrixXd& GetJ() const { return J_; }
    const Eigen::VectorXd& GetJdotTimesV() const { return JdotV_; }
    const Eigen::VectorXd& GetYddotCommand() const { return yddot_command_; }
    const Eigen::MatrixXd& GetWeight() const { return W_; }

    // Getters
    const std::string& GetName() const { return name_; };
    int GetYDim() const { return n_y_; };
    int GetYdotDim() const { return n_ydot_; };
    bool IsActive() const { return track_at_current_state_; }

    void SaveYddotCommandSol(const Eigen::VectorXd& dv);

    // Print feedback and desired values
    void PrintFeedbackAndDesiredValues(const Eigen::VectorXd& dv);

    // Finalize and ensure that users construct OscTrackingData class
    // correctly.
    void CheckOscTrackingData();

protected:
    int GetStateIdx() const { return state_idx_; };
    void AddState(int state);

    // Feedback output, Jacobian and dJ/dt * v
    Eigen::VectorXd error_y_;
    Eigen::VectorXd error_ydot_;
    Eigen::VectorXd y_;
    Eigen::VectorXd ydot_;
    Eigen::MatrixXd J_;
    Eigen::VectorXd JdotV_;

    // Desired output
    Eigen::VectorXd y_des_;
    Eigen::VectorXd ydot_des_;
    Eigen::VectorXd yddot_des_;
    Eigen::VectorXd yddot_des_converted_;

    // Commanded acceleration after feedback terms
    Eigen::VectorXd yddot_command_;
    // Osc solution
    Eigen::VectorXd yddot_command_sol_;

    // `state_` is the finite state machine state when the tracking is enabled
    // If `state_` is empty, then the tracking is always on.
    std::vector<int> state_;

    /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
    /// but in the optimization it uses `plant_wo_spr_`. The reason of using
    /// MultibodyPlant without springs is that the OSC cannot track desired
    /// acceleration instantaneously when springs exist. (relative degrees of 4)
    const drake::multibody::MultibodyPlant<double>& plant_;

    // World frames
    const drake::multibody::BodyFrame<double>& world_;

private:
    // Check if we should do tracking in the current state
    void UpdateTrackingFlag(int finite_state_machine_state);

    // Updaters of feedback output, jacobian and dJ/dt * v
    virtual void UpdateYAndError(
            const Eigen::VectorXd& x_w_spr,
            const drake::systems::Context<double>& context_w_spr) = 0;
    virtual void UpdateYdotAndError(
            const Eigen::VectorXd& x_w_spr,
            const drake::systems::Context<double>& context_w_spr) = 0;
    virtual void UpdateYddotDes() = 0;
    virtual void UpdateJ(
            const Eigen::VectorXd& x_wo_spr,
            const drake::systems::Context<double>& context_wo_spr) = 0;
    virtual void UpdateJdotV(
            const Eigen::VectorXd& x_wo_spr,
            const drake::systems::Context<double>& context_wo_spr) = 0;

    // Finalize and ensure that users construct OscTrackingData derived class
    // correctly.
    virtual void CheckDerivedOscTrackingData() = 0;

    // Trajectory name
    std::string name_;

    // Dimension of the traj
    int n_y_;
    int n_ydot_;

    // PD control gains
    Eigen::MatrixXd K_p_;
    Eigen::MatrixXd K_d_;

    // Cost weights
    Eigen::MatrixXd W_;

    // Store whether or not the tracking data is active
    bool track_at_current_state_;
    int state_idx_ = 0;
};

/// ComTrackingData is used when we want to track center of mass trajectory.
class ComTrackingData final : public OscTrackingData {
public:
    ComTrackingData(const std::string& name, const Eigen::MatrixXd& K_p,
                    const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
                    const drake::multibody::MultibodyPlant<double>& plant);

    //  ComTrackingData() {}  // Default constructor

    // If state is not specified, it will track COM for all states
    void AddStateToTrack(int state);

private:
    void UpdateYAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYdotAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYddotDes() final;
    void UpdateJ(const Eigen::VectorXd& x,
                 const drake::systems::Context<double>& context) final;
    void UpdateJdotV(const Eigen::VectorXd& x,
                     const drake::systems::Context<double>& context) final;

    void CheckDerivedOscTrackingData() final;
};

// TaskSpaceTrackingData is still a virtual class
class TaskSpaceTrackingData : public OscTrackingData {
public:
    TaskSpaceTrackingData(
            const std::string& name, int n_y, int n_ydot, const Eigen::MatrixXd& K_p,
            const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
            const drake::multibody::MultibodyPlant<double>& plant);

protected:
    // `body_index_w_spr` is the index of the body
    // `body_index_wo_spr` is the index of the body
    // If `body_index_w_spr_` is empty, `body_index_wo_spr_` replaces it.
    std::vector<drake::multibody::BodyIndex> body_index_;
    std::vector<const drake::multibody::BodyFrame<double>*> body_frames_;
};

/// TransTaskSpaceTrackingData is used when we want to track a trajectory
/// (translational position) in the task space.

/// AddPointToTrack() should be called to specify what is the point that
/// follows the desired trajectory.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndPointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// pt_on_body_'s if state_ is not empty.
/// This also means that AddPointToTrack and AddStateAndPointToTrack cannot be
/// called one after another for the same TrackingData.
class TransTaskSpaceTrackingData final : public TaskSpaceTrackingData {
public:
    TransTaskSpaceTrackingData(
            const std::string& name, const Eigen::MatrixXd& K_p,
            const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
            const drake::multibody::MultibodyPlant<double>& plant);

    void AddPointToTrack(
            const std::string& body_name,
            const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());
    void AddStateAndPointToTrack(
            int state, const std::string& body_name,
            const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());

private:
    void UpdateYAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYdotAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYddotDes() final;
    void UpdateJ(const Eigen::VectorXd& x,
                 const drake::systems::Context<double>& context) final;
    void UpdateJdotV(const Eigen::VectorXd& x,
                     const drake::systems::Context<double>& context) final;

    void CheckDerivedOscTrackingData() final;

    // `pt_on_body` is the position w.r.t. the origin of the body
    std::vector<Eigen::Vector3d> pts_on_body_;
};

/// RotTaskSpaceTrackingData is used when we want to track a trajectory
/// (rotational position) in the task space. The desired position must be
/// expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndFrameToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// frame_pose_'s if state_ is not empty.
/// This also means that AddFrameToTrack and AddStateAndFrameToTrack cannot be
/// called one after another for the same TrackingData.
class RotTaskSpaceTrackingData final : public TaskSpaceTrackingData {
public:
    RotTaskSpaceTrackingData(
            const std::string& name, const Eigen::MatrixXd& K_p,
            const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
            const drake::multibody::MultibodyPlant<double>& plant);

    void AddFrameToTrack(
            const std::string& body_name,
            const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());
    void AddStateAndFrameToTrack(
            int state, const std::string& body_name,
            const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

private:
    void UpdateYAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYdotAndError(
            const Eigen::VectorXd& x,
            const drake::systems::Context<double>& context) final;
    void UpdateYddotDes() final;
    void UpdateJ(const Eigen::VectorXd& x,
                 const drake::systems::Context<double>& context) final;
    void UpdateJdotV(const Eigen::VectorXd& x,
                     const drake::systems::Context<double>& context) final;

    void CheckDerivedOscTrackingData() final;

    // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
    // which follows the desired rotation.
    std::vector<Eigen::Isometry3d> frame_pose_;
};

#endif //SONG_WBC_OSCTRACKINGDATA_H
