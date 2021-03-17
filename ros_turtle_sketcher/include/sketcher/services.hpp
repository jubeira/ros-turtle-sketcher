/*
 * services.hpp
 *
 *  Created on: Oct 19, 2016
 *      Author: juan
 */

#ifndef TURTLE_SKETCHER_SRC_INCLUDE_SERVICES_HPP_
#define TURTLE_SKETCHER_SRC_INCLUDE_SERVICES_HPP_

#include <string>
#include "ros/ros.h"
#include "turtlesim/Pose.h"

namespace tsrv {

bool kill(ros::NodeHandle& n, std::string name);
std::string spawn(ros::NodeHandle& n, turtlesim::Pose& pose, std::string name);

}

#endif /* TURTLE_SKETCHER_SRC_INCLUDE_SERVICES_HPP_ */
