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


#endif //SONG_WBC_UTILS_H
