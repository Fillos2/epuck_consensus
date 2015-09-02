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
}

void Robopuck::set(int _num,ros::NodeHandle * nh){
	num = _num;

	string codice;

	if (num == 0)
		codice = "2034";
	else if(num == 1)
		codice = "2041";
	else if(num == 2)
		codice = "2037";

	pub_= nh->advertise<geometry_msgs::Twist>("/epuck"+codice+"/cmd_vel", 1000);
}
Robopuck::~Robopuck() {
	// TODO Auto-generated destructor stub
}

int Robopuck::invia(){
	geometry_msgs::Twist msg;

	msg.linear.x=cos(theta)*xvel + sin(theta)*yvel;
	msg.angular.z=-(sin(theta)/b0)*xvel + (cos(theta)/b0)*yvel;
	pub_.publish(msg);
	return 1;
}
