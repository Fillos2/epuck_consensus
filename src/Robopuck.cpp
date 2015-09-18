/*
 * Robopuck.cpp
 *
 *  Created on: 21 ago 2015
 *      Author: filippo
 */

#include <epuck_consensus/Robopuck.h>
#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
using namespace std;
Robopuck::Robopuck(){
	b0=0.035;
	x=y=theta=xvel=yvel=0;
	num=-1;
	axle_=0.053;
	double v=0.1288;
	vMax_.first = 1000;
	vMax_.second = 1000;
	flag_=false;
}

void Robopuck::set(int _num,ros::NodeHandle * nh){
	if(flag_ )return;

	num = _num;

	string codice;

	if (num == 0)
		codice = "2041";
	else if(num == 1)
		codice = "2034";
	else if(num == 2)
		codice = "2037";
	else if(num == 3)
		codice = "2042";

	pub_= nh->advertise<geometry_msgs::Twist>("/epuck"+codice+"/cmd_vel", 1000);
	flag_=true;
}
Robopuck::~Robopuck() {
	// TODO Auto-generated destructor stub
}

int Robopuck::invia(){
	geometry_msgs::Twist msg;

	msg.linear.x =( cos(theta) * xvel + sin(theta) * yvel);
	msg.angular.z =( -1 / b0 * sin(theta) * xvel + 1 / b0 * cos(theta) * yvel);
	this->satVel(msg);

	/*ROS_INFO_STREAM("lin");
	ROS_INFO_STREAM(msg.linear.x);
	ROS_INFO_STREAM(msg.angular.z);
*/
	pub_.publish(msg);
	return 1;
}
