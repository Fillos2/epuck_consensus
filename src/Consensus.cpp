/*
 * Consensus.cpp
 *
 *  Created on: 19 ago 2015
 *      Author: filippo
 */

#include "epuck_consensus/Consensus.h"
#include "geometry_msgs/Twist.h"
#include "epuck_consensus/Robopuck.h"
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <string>
#include "math.h"
#include <iostream>
#include <image_transport/image_transport.h>
using namespace std;
Consensus::Consensus():it(nh_) {
	inizializzaMat();
	aspettaok();
	image_sub = it.subscribe("epuck_tracking/result", 1, &Consensus::image_callback, this);
	while(ros::ok()){
		ros::spin();
	}

}

Consensus::~Consensus() {
	
}
void Consensus::image_callback(const sensor_msgs::ImageConstPtr& msg){
	acquisisciPos();
	calcola_vel();
	for(int i=0;i<3;i++)
	Robots[i].invia();
}

void Consensus::inizializzaMat(){
	int i,j;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			if(i==j)
				L_[i][j]=2;
			else
				L_[i][j]=-1;
		}
	}
	x_[0]=0;
	x_[1]=1;
	x_[2]=2;

	y_[0]=0;
	y_[1]=0;
	y_[2]=1;

	z_[0]=1;
	z_[1]=1;
	z_[2]=1;

	for (i = 0; i < 3; i++) {
		b_x[i] = 0;
		for (j = 0; j < 3; j++) {
			b_x[i] = b_x[i] + L_[i][j] * x_[j];
	    }
	}
	for (i = 0; i < 3; i++) {
		b_y[i] = 0;
	 	for (j = 0; j < 3; j++) {
	 		b_y[i] = b_y[i] + L_[i][j] * y_[j];
	 	}
	 }

	for (i = 0; i < 3; i++) {
		b_z[i] = 0;
	 	for (j = 0; j < 3; j++) {
	 		b_z[i] = b_z[i] + L_[i][j] * z_[j];
	 	}
	 }
}

void Consensus::acquisisciPos(){
	tf::TransformListener listener;
	tf::StampedTransform transform;
	geometry_msgs::TransformStamped msg;
	ros::Time now = ros::Time(0);
	for(int i=0;i<3;i++){
		Robots[i].set(i,&nh_);
		try{
			listener.waitForTransform("board", boost::to_string(i),now, ros::Duration(3.0));
			listener.lookupTransform("board", boost::to_string(i),now, transform);

		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			//ros::Duration(1.0).sleep();
		}

		tf::transformStampedTFToMsg(transform, msg);
		Robots[i].x=msg.transform.translation.x;
		Robots[i].y=msg.transform.translation.y;
		geometry_msgs::Quaternion Q = msg.transform.rotation;
		Robots[i].theta = asin((double)Q.z)/2;
	}
}

void Consensus::calcola_vel(){
	double sumx=0,sumy=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			if(i!=j){
				sumx+=Robots[i].x-Robots[j].x;
				sumy+=Robots[i].y-Robots[j].y;
			}
			Robots[i].xvel=-sumx + b_x[i];
			Robots[i].yvel=-sumy + b_y[i];
		}


	}
}
void Consensus::aspettaok(){
	ROS_INFO("Premi un tasto per cominciare");
	getchar();
}
