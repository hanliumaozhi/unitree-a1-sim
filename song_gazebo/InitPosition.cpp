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

      Vector3d position = Vector3d(0, 0, 2);
      Quaterniond quat = Quaterniond(1, 0, 0, 0);
      model_->SetLinkWorldPose(ignition::math::Pose3d(position, quat),"base");
    }

    GZ_REGISTER_MODEL_PLUGIN(InitPosition)
}
