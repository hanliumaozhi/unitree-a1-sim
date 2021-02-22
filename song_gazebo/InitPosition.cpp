//
// Created by han on 2021/2/21.
//

#include "InitPosition.h"

#include <ignition/math/Vector3.hh>

using ignition::math::Vector3d;
using ignition::math::Quaterniond;


namespace gazebo {

    void InitPosition::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf_elem) {
      model_ = _parent;

      Vector3d position = Vector3d(0, 0, 0.3);
      Quaterniond quat = Quaterniond(1, 0, 0, 0);
      model_->SetLinkWorldPose(ignition::math::Pose3d(position, quat),"base");

      std::array<physics::JointPtr, 12> joint_list;
      joint_list = {
              model_->GetJoint("FR_hip_joint"),
              model_->GetJoint("FL_hip_joint"),
              model_->GetJoint("RR_hip_joint"),
              model_->GetJoint("RL_hip_joint"),
              model_->GetJoint("FR_thigh_joint"),
              model_->GetJoint("FL_thigh_joint"),
              model_->GetJoint("RR_thigh_joint"),
              model_->GetJoint("RL_thigh_joint"),
              model_->GetJoint("FR_calf_joint"),
              model_->GetJoint("FL_calf_joint"),
              model_->GetJoint("RR_calf_joint"),
              model_->GetJoint("RL_calf_joint")
      };

        std::array<double, 12> joint_positions = {
                -0.0263951,
                0.0263951,
                -0.0263951,
                0.0263951,
                0.814243,
                0.814243,
                0.814243,
                0.814243,
                -1.53652,
                -1.53652,
                -1.53652,
                -1.53652

        };

        for (int i = 0; i < 12; ++i) {
            joint_list[i]->SetPosition(0, joint_positions[i]);
            joint_list[i]->SetVelocity(0, 0);
        }
    }

    GZ_REGISTER_MODEL_PLUGIN(InitPosition)
}
