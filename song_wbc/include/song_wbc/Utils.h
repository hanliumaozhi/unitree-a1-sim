//
// Created by han on 2021/2/22.
//

#ifndef SONG_WBC_UTILS_H
#define SONG_WBC_UTILS_H

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>

std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
LeftToeFront(const drake::multibody::MultibodyPlant<double>& plant);

std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
RightToeFront(const drake::multibody::MultibodyPlant<double>& plant);

std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
LeftToeRear(const drake::multibody::MultibodyPlant<double>& plant);

std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>
RightToeRear(const drake::multibody::MultibodyPlant<double>& plant);

void addA1Multibody(
        drake::multibody::MultibodyPlant<double>* plant,
        std::string filename,
        bool floating_base = true);

using ContactData = std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>;

void SetPositionsIfNew(const drake::multibody::MultibodyPlant<double>& plant,
                       const Eigen::Ref<const drake::VectorX<double>>& q,
                       drake::systems::Context<double>* context);

/// Update an existing MultibodyPlant context, setting corresponding velocities.
/// Will only set if value if changed from current value.
void SetVelocitiesIfNew(const drake::multibody::MultibodyPlant<double>& plant,
                        const Eigen::Ref<const drake::VectorX<double>>& f,
                        drake::systems::Context<double>* context);


/// Given a MultiBodyTree, builds a map from position name to position index
std::map<std::string, int> makeNameToPositionsMap(
        const drake::multibody::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from velocity name to velocity index
std::map<std::string, int> makeNameToVelocitiesMap(
        const drake::multibody::MultibodyPlant<double>& plant);

/// Given a MultiBodyTree, builds a map from actuator name to actuator index
std::map<std::string, int> makeNameToActuatorsMap(
        const drake::multibody::MultibodyPlant<double>& plant);


#endif //SONG_WBC_UTILS_H
