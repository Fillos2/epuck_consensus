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
#include <epuck_tracking/bots.h>
#include <omni_msgs/OmniFeedback.h>
#include <omni_msgs/OmniState.h>
#include <omni_msgs/OmniButtonEvent.h>

using namespace std;
class Consensus {
public:
	Consensus();
	virtual ~Consensus();
private:
	ros::NodeHandle nh_;
	Robopuck Robots[8];
	float L_[8][8];
	float b_x[8];
	float b_y[8];
	float x_[8];
	float y_[8];
	ros::Subscriber bots_sub;
	ros::Subscriber phantom_state_sub;
	ros::Subscriber phantom_button_sub;
	ros::Publisher  phantom_feedback_pub;
	bool button_state_flag;
	bool teleop;
	float teleop_x;
	float teleop_y;
	float teleop_zero_x;
	float teleop_zero_y;
	bool in_state;

	void inizializzaMat();
	void aspettaok();
	void acquisisciPos(epuck_tracking::bots bots);
	void calcola_vel(epuck_tracking::bots bots);
	void bots_callback(const epuck_tracking::bots& msg);
	void state_callback(const omni_msgs::OmniState& state);
	void button_callback(const omni_msgs::OmniButtonEvent& button);

};


#endif /* EPUCK_CONSENSUS_SRC_CONSENSUS_H_ */
