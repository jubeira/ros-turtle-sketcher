#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"

#include <iostream>
#include "math2d.hpp"


using namespace std;
using namespace m2d;

struct ts_point {
	double x;
	double y;
};

//vector<ts_point> points = {{10, 10}, {1, 10}, {1, 1}, {10, 1}};

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
bool subscribed = false;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
void setDesiredOrientation(double desired_angle_rad);
void moveToGoal(turtlesim::Pose& goal_pose, double dist_tol);
void aimAndMove(turtlesim::Pose& goal_pose, double dist_tol, double line_straightness_tol = 0.0001);
void aimToGoal(turtlesim::Pose& goal_pose, double ang_tol);


void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg);

extern bool parseFile(string fileName, vector<double>& points);

int main(int argc, char** argv)
{
	if (argc != 2) {
		cout << "Bad arguments" << endl;
		return -1;
	}

	vector<double> points;

	if (parseFile(argv[1], points) == false) {
		cout << "Error opening file" << endl;
		return -1;
	}

	ros::init(argc, argv, "robot_cleaner");
	ros::NodeHandle n;

	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
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
	ros::Rate loop_rate(100);

	while (!subscribed)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	cout << "Subscribed; pos: " << turtlesim_pose.x << ", " << turtlesim_pose.y << ", " << turtlesim_pose.theta << endl;

	for(auto it = points.begin(); it != points.end(); ++it) {
		turtlesim::Pose pose;
		pose.x = *it;
		++it;
		pose.y = *it;
		aimAndMove(pose, 0.01);
	}

	return 0;
}



void move(double speed, double distance, bool isForward)
{
	geometry_msgs::Twist vel_msg;

	if (isForward) {
		vel_msg.linear.x = abs(speed);
	} else {
		vel_msg.linear.x = -abs(speed);
	}
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0;

	ros::Rate loop_rate(100);

	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_distance < distance);

	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);

}

void rotate(double angular_speed, double relative_angle, bool clockwise)
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise) {
		vel_msg.angular.z = -abs(angular_speed);
	} else {
		vel_msg.angular.z = abs(angular_speed);
	}

	double t0 = ros::Time::now().toSec();
	double current_angle = 0;

	ros::Rate loop_rate(100);

	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	} while(current_angle < relative_angle);

	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

}

void setDesiredOrientation(double desired_angle_rad)
{
	double relative_angle_rad = normalizeAngle(desired_angle_rad - normalizeAngle(turtlesim_pose.theta));

	bool clockwise = relative_angle_rad < 0 ? true : false;
	cout << desired_angle_rad << ", " << turtlesim_pose.theta << ", " << relative_angle_rad << endl;
	rotate(abs(relative_angle_rad), abs(relative_angle_rad), clockwise);
}

void moveToGoal(turtlesim::Pose& goal_pose, double dist_tol)
{
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(10);

	double error_begin, error_end, target_angle;

	do {
		error_begin = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

//		cout << "Err begin: " << error_begin << endl;

		vel_msg.linear.x = 1.5 * error_begin;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		target_angle = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));
		cout << "Target angle: " << target_angle << " current angle: " << normalizeAngle(turtlesim_pose.theta) << endl;
		vel_msg.angular.z = 4 * normalizeAngle((target_angle - normalizeAngle(turtlesim_pose.theta)));

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
		error_end = getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

//		cout << "Error: " << error_end << endl;

	} while (ros::ok() && (error_end > dist_tol));

	cout << "End move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();

}

void aimToGoal(turtlesim::Pose& goal_pose, double ang_tol)
{
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(10);

	double error_begin, error_end, target_angle;

	target_angle = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));

	do {
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4 * normalizeAngle((target_angle - normalizeAngle(turtlesim_pose.theta)));

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

//		cout << "Error: " << error_end << endl;

	} while (ros::ok() && (abs(normalizeAngle(turtlesim_pose.theta) - target_angle) > ang_tol));

	cout << "End aim goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();

}


void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg)
{
	subscribed = true;
//	cout << "PoseCallback called" << endl;
	turtlesim_pose.x = pose_msg->x;
	turtlesim_pose.y = pose_msg->y;
	turtlesim_pose.theta = pose_msg->theta;
}

void aimAndMove(turtlesim::Pose& goal_pose, double dist_tol, double line_straightness_tol)
{
	double desired_angle_rad = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));
	cout << "Desired angle: " << desired_angle_rad << endl;
//	setDesiredOrientation(desired_angle_rad);
	aimToGoal(goal_pose, line_straightness_tol);

	cout << "Real angle: " << turtlesim_pose.theta << endl;
	cout << "Real normalized angle: " << normalizeAngle(turtlesim_pose.theta) << endl;


	moveToGoal(goal_pose, dist_tol);
}


