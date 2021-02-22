//
// Created by han on 2021/2/22.
//

#include "song_wbc/Utils.h"


using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;

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