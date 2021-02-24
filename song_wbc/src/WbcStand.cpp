//
// Created by han on 2021/2/22.
//

#include <iostream>
#include <ros/package.h>

#include "song_wbc/controller/OscStandController.h"

int main()
{
    std::string path = ros::package::getPath("song_wbc");
    std::string urdf_path = path + "/urdf/a1.urdf";

    drake::multibody::MultibodyPlant<double> plant(0.0);
    addA1Multibody(&plant, urdf_path);
    plant.Finalize();

    auto context = plant.CreateDefaultContext();

    OscStandController osc(plant, context.get());

    std::cout<<urdf_path<<std::endl;
    return 0;
}
