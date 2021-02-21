//
// Created by han on 2021/2/21.
//

#ifndef SONG_GAZEBO_INITPOSITION_H
#define SONG_GAZEBO_INITPOSITION_H



#include <map>

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {
    class InitPosition : public ModelPlugin {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf_elem);

    private:
        physics::ModelPtr model_;
        std::map<std::string, physics::JointPtr> name_to_joint_map_;
    };
}

#endif //SONG_GAZEBO_INITPOSITION_H
