#ifndef ROBOT_COMMON_HPP
#define ROBOT_COMMON_HPP

#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>

class TControlPoint{

public:
    int Id;
    int Link2Id;
    std::string Type;
    std::string Label;
    double Velocity;
    std::vector<double> joint;



};

#endif // ROBOT_COMMON_HPP
