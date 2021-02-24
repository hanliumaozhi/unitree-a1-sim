//
// Created by han on 2021/2/22.
//

#include "song_ros_control/wbc/Utils.h"


using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using Eigen::Vector3d;
using drake::systems::Context;
using Eigen::VectorXd;
using drake::VectorX;
using std::map;
using std::string;
using std::vector;
using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;

bool AreVectorsEqual(const Eigen::Ref<const VectorXd>& a,
                     const Eigen::Ref<const VectorXd>& b) {
    return a == b;
}

std::pair<const Vector3d, const Frame<double>&> LeftToeFront(
        const MultibodyPlant<double>& plant) {
    return std::pair<const Vector3d, const Frame<double>&>(
            Vector3d(0, 0, -0.22), plant.GetFrameByName("FL_calf"));
}

std::pair<const Vector3d, const Frame<double>&> RightToeFront(
        const MultibodyPlant<double>& plant) {
    return std::pair<const Vector3d, const Frame<double>&>(
            Vector3d(0, 0, -0.22), plant.GetFrameByName("FR_calf"));
}

std::pair<const Vector3d, const Frame<double>&> LeftToeRear(
        const MultibodyPlant<double>& plant) {
    return std::pair<const Vector3d, const Frame<double>&>(
            Vector3d(0, 0, -0.22), plant.GetFrameByName("RL_calf"));
}

std::pair<const Vector3d, const Frame<double>&> RightToeRear(
        const MultibodyPlant<double>& plant) {
    return std::pair<const Vector3d, const Frame<double>&>(
            Vector3d(0, 0, -0.22), plant.GetFrameByName("RR_calf"));
}

void addA1Multibody(
        drake::multibody::MultibodyPlant<double>* plant,
        std::string filename,
        bool floating_base)
{
    Parser parser(plant);

    parser.AddModelFromFile(filename);
    if (!floating_base) {
        plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("base"),
                          drake::math::RigidTransform<double>(Vector3d::Zero()));
    }
}

void SetPositionsIfNew(const MultibodyPlant<double>& plant,
                       const Eigen::Ref<const VectorX<double>>& q,
                       Context<double>* context) {
    if (!AreVectorsEqual(q, plant.GetPositions(*context))) {
        plant.SetPositions(context, q);
    }
}

void SetVelocitiesIfNew(const MultibodyPlant<double>& plant,
                        const Eigen::Ref<const VectorX<double>>& v,
                        Context<double>* context) {
    if (!AreVectorsEqual(v, plant.GetVelocities(*context))) {
        plant.SetVelocities(context, v);
    }
}

map<string, int> makeNameToPositionsMap(const MultibodyPlant<double>& plant) {
    map<string, int> name_to_index_map;
    std::set<int> index_set;
    for (JointIndex i(0); i < plant.num_joints(); ++i) {
        const drake::multibody::Joint<double>& joint = plant.get_joint(i);
        auto name = joint.name();

        if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
            std::vector<JointIndex> index_vector{i};
            auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
            // find index and add
            int selector_index = -1;
            for (int j = 0; j < selectorMatrix.cols(); ++j) {
                if (selectorMatrix(0, j) == 1) {
                    if (selector_index == -1) {
                        selector_index = j;
                    } else {
                        throw std::logic_error("Unable to create selector map.");
                    }
                }
            }
            if (selector_index == -1) {
                std::logic_error("Unable to create selector map.");
            }

            name_to_index_map[name] = selector_index;
            index_set.insert(selector_index);
        }
    }

    // TODO: once RBT fully deprecated, this block can likely be removed, using
    // default coordinate names from Drake.
    auto floating_bodies = plant.GetFloatingBaseBodies();
    DRAKE_THROW_UNLESS(floating_bodies.size() <= 1);
    for (auto body_index : floating_bodies) {
        const auto& body = plant.get_body(body_index);
        DRAKE_ASSERT(body.has_quaternion_dofs());
        int start = body.floating_positions_start();
        // should be body.name() once RBT is deprecated
        std::string name = "base";
        name_to_index_map[name + "_qw"] = start;
        name_to_index_map[name + "_qx"] = start + 1;
        name_to_index_map[name + "_qy"] = start + 2;
        name_to_index_map[name + "_qz"] = start + 3;
        name_to_index_map[name + "_x"] = start + 4;
        name_to_index_map[name + "_y"] = start + 5;
        name_to_index_map[name + "_z"] = start + 6;
        for (int i = 0; i < 7; i++) {
            index_set.insert(start + i);
        }
    }

    for (int i = 0; i < plant.num_positions(); ++i) {
        // if index has not already been captured, throw an error
        if (index_set.find(i) == index_set.end()) {
            DRAKE_THROW_UNLESS(false);
        }
    }

    return name_to_index_map;
}

/// Construct a map between joint names and velocity indices
///     <name,index> such that v(index) has the given name
///  -Only accurately includes joints with a single position and single velocity
///  -Others are included as "state[ind]"
///  -Index mapping can also be used as a state mapping, AFTER
///     an offset of num_positions is applied (assumes x = [q;v])

map<string, int> makeNameToVelocitiesMap(const MultibodyPlant<double>& plant) {
    map<string, int> name_to_index_map;
    std::set<int> index_set;

    for (JointIndex i(0); i < plant.num_joints(); ++i) {
        const drake::multibody::Joint<double>& joint = plant.get_joint(i);
        // TODO(posa): this "dot" should be removed, it's an anachronism from
        // RBT
        auto name = joint.name() + "dot";

        if (joint.num_velocities() == 1 && joint.num_positions() == 1) {
            std::vector<JointIndex> index_vector{i};
            auto selectorMatrix = plant.MakeStateSelectorMatrix(index_vector);
            // find index and add
            int selector_index = -1;
            for (int j = 0; j < selectorMatrix.cols(); ++j) {
                if (selectorMatrix(1, j) == 1) {
                    if (selector_index == -1) {
                        selector_index = j;
                    } else {
                        throw std::logic_error("Unable to create selector map.");
                    }
                }
            }
            if (selector_index == -1) {
                throw std::logic_error("Unable to create selector map.");
            }

            name_to_index_map[name] = selector_index - plant.num_positions();
            index_set.insert(selector_index - plant.num_positions());
        }
    }

    auto floating_bodies = plant.GetFloatingBaseBodies();
    // Remove throw once RBT deprecated
    DRAKE_THROW_UNLESS(floating_bodies.size() <= 1);
    for (auto body_index : floating_bodies) {
        const auto& body = plant.get_body(body_index);
        int start = body.floating_velocities_start() - plant.num_positions();
        std::string name = "base";  // should be body.name() once RBT is deprecated
        name_to_index_map[name + "_wx"] = start;
        name_to_index_map[name + "_wy"] = start + 1;
        name_to_index_map[name + "_wz"] = start + 2;
        name_to_index_map[name + "_vx"] = start + 3;
        name_to_index_map[name + "_vy"] = start + 4;
        name_to_index_map[name + "_vz"] = start + 5;
        for (int i = 0; i < 6; i++) {
            index_set.insert(start + i);
        }
    }

    for (int i = 0; i < plant.num_velocities(); ++i) {
        // if index has not already been captured, throw an error
        if (index_set.find(i) == index_set.end()) {
            DRAKE_THROW_UNLESS(false);
        }
    }

    return name_to_index_map;
}

map<string, int> makeNameToActuatorsMap(const MultibodyPlant<double>& plant) {
    map<string, int> name_to_index_map;
    for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
        const drake::multibody::JointActuator<double>& actuator =
                plant.get_joint_actuator(i);
        auto name = actuator.name();

        if (actuator.joint().num_velocities() == 1 &&
            actuator.joint().num_positions() == 1) {
            std::vector<JointActuatorIndex> index_vector{i};
            auto selectorMatrix = plant.MakeActuatorSelectorMatrix(index_vector);

            // find index and add
            int selector_index = -1;
            for (int j = 0; j < selectorMatrix.rows(); ++j) {
                if (selectorMatrix(j, 0) == 1) {
                    if (selector_index == -1) {
                        selector_index = j;
                    } else {
                        throw std::logic_error("Unable to create selector map.");
                    }
                }
            }
            if (selector_index == -1) {
                throw std::logic_error("Unable to create selector map.");
            }

            name_to_index_map[name] = selector_index;
        }
    }
    return name_to_index_map;
}

Eigen::MatrixXd get_contact_jacobin(ContactData contact, const drake::multibody::MultibodyPlant<double>& plant, const drake::systems::Context<double>& context)
{
    Eigen::MatrixXd J(3, plant.num_velocities());

    plant.CalcJacobianTranslationalVelocity(context, drake::multibody::JacobianWrtVariable::kV, contact.second,
                                            contact.first, plant.world_frame(), plant.world_frame(), &J);

    return J;
}

Eigen::MatrixXd get_contact_jacobinDotTimesV(ContactData contact, const drake::multibody::MultibodyPlant<double>& plant, const drake::systems::Context<double>& context)
{
    const drake::multibody::Frame<double>& world = plant.world_frame();

    Eigen::MatrixXd Jdot_times_V = plant.CalcBiasTranslationalAcceleration(
            context, drake::multibody::JacobianWrtVariable::kV, contact.second,
            contact.first, world, world);

    return  Jdot_times_V;
}
