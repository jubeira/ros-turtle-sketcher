#include <ros/duration.h>
#include "sketcher/services.hpp"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"

using namespace std;

namespace tsrv {

/**
 * kill
 * Kills a target turtle from turtleSim.
 * @param n NodeHandle reference.
 * @param name Target turtle to vanish.
 * @return true if successful, false if not.
 */
bool kill(ros::NodeHandle& n, string name)
{
    ros::service::waitForService("/kill", ros::Duration(2.0));
    ros::ServiceClient killClient = n.serviceClient<turtlesim::Kill>("kill");
    turtlesim::Kill ksrv;
    ksrv.request.name = name;
    return killClient.call(ksrv);
}

/**
 * spawn
 * Makes a new turtle at target pose.
 * @param n NodeHandle reference
 * @param pose Pose where turtle will appear.
 * @param name Name of the turtle to create.
 * @return New turtle's name.
 */

string spawn(ros::NodeHandle& n, turtlesim::Pose& pose, string name)
{
    ros::ServiceClient spawnClient = n.serviceClient<turtlesim::Spawn>("spawn");

    turtlesim::Spawn srv;

    srv.request.x = pose.x;
    srv.request.y = pose.y;
    srv.request.theta = pose.theta;
    srv.request.name = name;

    spawnClient.call(srv);

    return srv.response.name;
}

}
