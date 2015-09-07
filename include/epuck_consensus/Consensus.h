/*
 * Consensus.h
 *
 *  Created on: 19 ago 2015
 *      Author: filippo
 */

#ifndef EPUCK_CONSENSUS_SRC_CONSENSUS_H_
#define EPUCK_CONSENSUS_SRC_CONSENSUS_H_
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <string>
#include "epuck_consensus/Robopuck.h"
#include <image_transport/image_transport.h>
using namespace std;
class Consensus {
public:
	Consensus();
	virtual ~Consensus();
private:
	ros::NodeHandle nh_;
	ros::Publisher spd;
	Robopuck Robots[3];
	int L_[3][3];
	int b_x[3];
	int b_y[3];
	int b_z[3];
	int x_[3];
	int y_[3];
	int z_[3];
	image_transport::ImageTransport it;
	image_transport::Subscriber image_sub;

	void inizializzaMat();
	void aspettaok();
	void acquisisciPos();
	void calcola_vel();
	void image_callback(const sensor_msgs::ImageConstPtr& msg);
};


#endif /* EPUCK_CONSENSUS_SRC_CONSENSUS_H_ */
