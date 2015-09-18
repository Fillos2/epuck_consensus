/*
 * Consensus.cpp
 *
 *  Created on: 19 ago 2015
 *      Author: filippo
 */

#include "epuck_consensus/Consensus.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "epuck_consensus/Robopuck.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include "math.h"
#include <iostream>
#include <epuck_tracking/bots.h>
using namespace std;
Consensus::Consensus() {
	inizializzaMat();
	aspettaok();
	button_state_flag = false;
	teleop=false;
	in_state = false;
	teleop_zero_x=teleop_zero_y=0.0;
	bots_sub = nh_.subscribe("epuck_tracking/bots", 1, &Consensus::bots_callback, this);
	phantom_state_sub = nh_.subscribe("/phantom/state",1,&Consensus::state_callback,this);
	phantom_button_sub = nh_.subscribe("/phantom/button",1,&Consensus::button_callback,this);
	phantom_feedback_pub = nh_.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback",1000);
	while(ros::ok()){
		ros::spin();
	}

}

Consensus::~Consensus() {
	
}
void Consensus::bots_callback(const epuck_tracking::bots& msg){
	ROS_INFO_STREAM("bots");
	acquisisciPos(msg);
	calcola_vel(msg);

}

void Consensus::inizializzaMat(){
	int i,j;
	for(i=0;i<8;i++){
		for(j=0;j<8;j++){
			if(i==j)
				L_[i][j]=1.0;
			else
				L_[i][j]=0.0;
			//ROS_INFO_STREAM(L_[i][j]);
		}
	}
	x_[0]=0.5;
	y_[0]=0.5;

	x_[1]=0.5;
	y_[1]=-0.5;

	x_[2]=-0.5;
	y_[2]=0.5;

	x_[3]=-0.5;
	y_[3]=-0.5;

	x_[4]=0.8;
	y_[4]=0.2;

	x_[5]=0.0;
	y_[5]=0.6;

	x_[6]=0.0;
	y_[6]=0.0;

	x_[7]=0.0;
	y_[7]=0.0;



	for (i = 0; i < 8; i++) {
		ROS_INFO_STREAM(i);
		b_x[i] = 0.0;
		b_y[i] = 0.0;

		for (j = 0; j < 8; j++) {
			//ROS_INFO_STREAM(i);

			b_x[i] += L_[i][j] * x_[j];

			b_y[i] += L_[i][j] * y_[j];
	    }
		//b_x[i]/=2.0;
		//b_y[i]/=2.0;
		ROS_INFO_STREAM("b_x"+boost::to_string(i)+" = "+boost::to_string(b_x[i]));
		ROS_INFO_STREAM("b_y"+boost::to_string(i)+" = "+boost::to_string(b_y[i]));

	}

}

void Consensus::acquisisciPos(epuck_tracking::bots bots){
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::PoseStamped poseMsg;
	ros::Time now = ros::Time(0);


	for(int i=0;i<bots.bots.size();i++){
		Robots[bots.bots[i]].set(bots.bots[i],&nh_);
		try{
			listener.waitForTransform("board","linear"+boost::to_string(bots.bots[i]),now, ros::Duration(0.5));
			listener.lookupTransform("board","linear"+boost::to_string(bots.bots[i]),now, transform);

		}
		catch (tf::TransformException& ex){
			ROS_ERROR("%s",ex.what());
			//ros::Duration(1.0).sleep();
		}

		tf::poseTFToMsg(transform, poseMsg.pose);
		Robots[i].x=poseMsg.pose.position.x;
		Robots[i].y=poseMsg.pose.position.y;

		Robots[i].theta = tf::getYaw(poseMsg.pose.orientation);

		//ROS_INFO_STREAM("position"+boost::to_string(i)+"  "+boost::to_string(Robots[i].x)+"   "+boost::to_string(Robots[i].y)+"   "+boost::to_string(Robots[i].theta));

	}
}

void Consensus::calcola_vel(epuck_tracking::bots bots){
	double sumx,sumy;
	omni_msgs::OmniFeedback forcefeedback;


	for(int i=0;i<bots.bots.size();i++){
		sumx=0,sumy=0;
		for(int j=0;j<bots.bots.size();j++){
			if(i!=j){
				sumx+=Robots[bots.bots[i]].x-Robots[bots.bots[j]].x;
				sumy+=Robots[bots.bots[i]].y-Robots[bots.bots[j]].y;
			}
			Robots[bots.bots[i]].xvel=-sumx+ b_x[bots.bots[i]];
			Robots[bots.bots[i]].yvel=-sumy+ b_y[bots.bots[i]];

		}
		if(teleop && bots.bots[i]==0){
			ROS_INFO_STREAM("force feedback");
			forcefeedback.force.x= Robots[bots.bots[i]].xvel*2;
			forcefeedback.force.y= Robots[bots.bots[i]].yvel*2;
			forcefeedback.force.z=0.0;
			phantom_feedback_pub.publish(forcefeedback);
			Robots[bots.bots[i]].xvel += teleop_x/75;
			Robots[bots.bots[i]].yvel += teleop_y/75;
		}
		Robots[bots.bots[i]].xvel/=5;
		Robots[bots.bots[i]].yvel/=5;
		Robots[bots.bots[i]].invia();

	}
}
void Consensus::state_callback(const omni_msgs::OmniState& state){
	if(state.end_effector_out_of_inkwell){
		teleop_x =state.pose.position.x-teleop_zero_x;
		teleop_y =state.pose.position.y-teleop_zero_y;
		ROS_INFO_STREAM("out");
	}
	else{
		teleop_x = teleop_y = 0.0;
		ROS_INFO_STREAM("in");
	}
	in_state=state.end_effector_out_of_inkwell;
	//ROS_INFO_STREAM(in_state);

}
void Consensus::button_callback(const omni_msgs::OmniButtonEvent& button){
	omni_msgs::OmniFeedback msg;
	msg.force.x=0.0;
	msg.force.y=0.0;
	if(in_state){
		ROS_INFO_STREAM(button.grey_button);
		if(button.grey_button == 0){
			if(teleop==false){
				teleop=true;
				teleop_zero_x=teleop_x;
				teleop_zero_y=teleop_y;
				ROS_INFO_STREAM("teleop on");
			}
			else{
				teleop=false;
				teleop_zero_x=teleop_zero_y=0.0;
				phantom_feedback_pub.publish(msg);
				ROS_INFO_STREAM("teleop off");
			}
		}
	}
	ROS_INFO_STREAM("teleop ");
}
void Consensus::aspettaok(){
	ROS_INFO("Premi un tasto per cominciare");
	getchar();
}
