#include "services.hpp"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"

using namespace std;

namespace tsrv {

bool kill(ros::NodeHandle& n, string name)
{

	ros::ServiceClient killClient = n.serviceClient<turtlesim::Kill>("kill");

	turtlesim::Kill ksrv;

	ksrv.request.name = name;

	return killClient.call(ksrv);
}

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


/*
	ros::ServiceClient killClient = n.serviceClient<turtlesim::Kill>("kill");

	turtlesim::Kill ksrv;

	ksrv.request.name = "turtle1";

	if (killClient.call(ksrv) == false) {
		cout << "couldnt kill turtle" << endl;
	}

	cout << "response: " << ksrv.response << endl;

	ros::ServiceClient spawnClient = n.serviceClient<turtlesim::Spawn>("spawn");

	turtlesim::Spawn srv;

	srv.request.x = 2;
	srv.request.y = 2;
	srv.request.theta = 1.57;

	spawnClient.call(srv);

	cout << "new spawned name:" << srv.response.name << endl;

	ksrv.request.name = srv.response.name;
	killClient.call(ksrv);

	cout << "turtle 2 kill attempt: " << ksrv.response << endl;
*/

}
