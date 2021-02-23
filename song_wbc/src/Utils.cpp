//
// Created by han on 2021/2/22.
//

#include "song_wbc/Utils.h"


using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
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