/*
 * Robopuck.h
 *
 *  Created on: 21 ago 2015
 *      Author: filippo
 */

#ifndef EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_
#define EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_
#include "ros/ros.h"
class Robopuck {
public:
	Robopuck();
	void set(int _num,ros::NodeHandle * nh);
	virtual ~Robopuck();
	double x;
	double y;
	double theta;
	double b0;
	double xvel;
	double yvel;
	int invia();
private:
	int num;
	ros::Publisher pub_;


};

#endif /* EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_ */
