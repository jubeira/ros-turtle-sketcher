#include "geometry_msgs/Twist.h"
#include <iostream>
#include "math2d.hpp"
#include "services.hpp"

#define STD_TURTLE_NAME "turtle1"

using namespace std;

/*
 * Global definitions
 */
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
bool subscribed = false;

/*
 * Function prototypes
 */

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double relative_angle, bool clockwise);
void setDesiredOrientation(double desired_angle_rad);
void moveToGoal(turtlesim::Pose& goal_pose, double dist_tol);
void aimAndMove(turtlesim::Pose& goal_pose, double dist_tol, double line_straightness_tol = 0.0001);
void aimToGoal(turtlesim::Pose& goal_pose, double ang_tol);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg);

extern bool parseFile(string fileName, vector<double>& points);

/*
 * Application
 */

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

	cout << "Vector size: " << points.size() << endl;

	if (points.size() < 4) {
		cout << "Insert at least two (x, y) coordinates to draw" << endl;
		return -1;
	}

	ros::init(argc, argv, "turtle_sketcher");
	ros::NodeHandle n;

	string turtleName = STD_TURTLE_NAME;

	tsrv::kill(n, turtleName);

	turtlesim::Pose initialPose;
	initialPose.x = points[0];
	initialPose.y = points[1];

	tsrv::spawn(n, initialPose, turtleName);

	pose_subscriber = n.subscribe("/" + turtleName + "/pose", 10, poseCallback);
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/" + turtleName + "/cmd_vel", 10);

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

	tsrv::kill(n, turtleName);

	return 0;
}

/**
 * rotate
 * Makes global turtle move along its X axis at a certain speed for a specified distance. No tolerance specified.
 * @param speed Movement speed (x).
 * @param distance Distance to travel.
 * @param isForward Movement direction.
 */

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

/**
 * rotate
 * Makes global turtle rotate a relative angle at a certain speed. No tolerance specified.
 * @param angular_speed Rotation speed.
 * @param relative_angle Angle of rotation (relative to starting angle).
 * @param clockwise Rotation direction.
 */

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

/**
 * setDesiredOrientation
 * Rotates up to an absolute orientation. No tolerance is specified; use only when precision is not needed.
 * @param desired_angle_rad Absolute angle.
 */

void setDesiredOrientation(double desired_angle_rad)
{
	double relative_angle_rad = m2d::normalizeAngle(desired_angle_rad - m2d::normalizeAngle(turtlesim_pose.theta));

	bool clockwise = relative_angle_rad < 0 ? true : false;
	cerr << desired_angle_rad << ", " << turtlesim_pose.theta << ", " << relative_angle_rad << endl;
	rotate(abs(relative_angle_rad), abs(relative_angle_rad), clockwise);
}

/**
 * aimAndMove
 * Moves global turtle to goal_pose using a proportional controller.
 * @param goal_pose Target pose to reach (x, y only).
 * @param dist_tol Distance tolerance allowed to target pose.
 */

void moveToGoal(turtlesim::Pose& goal_pose, double dist_tol)
{
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(10);

	double error_begin, error_end, target_angle;

	do {
		error_begin = m2d::getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

		vel_msg.linear.x = 2.5 * error_begin;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		target_angle = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));
		cerr << "Target angle: " << target_angle << " current angle: " << m2d::normalizeAngle(turtlesim_pose.theta) << endl;
		vel_msg.angular.z = 3 * m2d::normalizeAngle((target_angle - m2d::normalizeAngle(turtlesim_pose.theta)));

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
		error_end = m2d::getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);

	} while (ros::ok() && (error_end > dist_tol));

	cerr << "End move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();

}

/**
 * /**
 * aimToGoal
 * Makes global turtle face the target point.
 * @param goal_pose Target pose to reach (x, y only).
 * @param ang_tol Angle tolerance when facing target point (in radians).
 */

void aimToGoal(turtlesim::Pose& goal_pose, double ang_tol)
{
	geometry_msgs::Twist vel_msg;
	ros::Rate loop_rate(10);

	double error_begin, error_end, target_angle;

	target_angle = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));

	do {
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4 * m2d::normalizeAngle((target_angle - m2d::normalizeAngle(turtlesim_pose.theta)));

		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();

	} while (ros::ok() && (abs(m2d::normalizeAngle(turtlesim_pose.theta) - target_angle) > ang_tol));

	cerr << "End aim goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
	ros::spinOnce();
	loop_rate.sleep();

}

/**
 * poseCallback
 * Updates global turtle's pose.
 */

void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg)
{
	subscribed = true;
	turtlesim_pose.x = pose_msg->x;
	turtlesim_pose.y = pose_msg->y;
	turtlesim_pose.theta = pose_msg->theta;
}

/**
 * aimAndMove
 * Moves global turtle to goal_pose in a straight line, facing the target point before moving.
 * @param goal_pose Target pose to reach (x, y only)
 * @param dist_tol Distance tolerance allowed to target pose.
 * @param line_straightness_tol Angle tolerance when facing target point (in radians).
 */

void aimAndMove(turtlesim::Pose& goal_pose, double dist_tol, double line_straightness_tol)
{
	double desired_angle_rad = atan2((goal_pose.y-turtlesim_pose.y), (goal_pose.x - turtlesim_pose.x));
	cout << "Desired angle: " << desired_angle_rad << endl;
	aimToGoal(goal_pose, line_straightness_tol);

	cerr << "Real angle: " << turtlesim_pose.theta << endl;
	cerr << "Real normalized angle: " << m2d::normalizeAngle(turtlesim_pose.theta) << endl;


	moveToGoal(goal_pose, dist_tol);
}


