//
// Created by han on 2021/2/22.
//

#include <iostream>
#include <ros/package.h>

int main()
{
    std::string path = ros::package::getPath("song_wbc");
    std::cout<<path<<std::endl;
    return 0;
}
