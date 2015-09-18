/*
 * Robopuck.h
 *
 *  Created on: 21 ago 2015
 *      Author: filippo
 */

#ifndef EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_
#define EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#ifndef SIGN
#define SIGN(a) (((a)>=(0))?(1):(-1))
#endif //SIGN

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
	double axle_;
	int invia();
private:
	std::pair<double, double> vMax_;
	int num;
	bool flag_;
	ros::Publisher pub_;
	void satVel(geometry_msgs::Twist& vel)
	  {
	    double lin, rot,left_f,right_f;
	    double wr,wl;
	    lin = vel.linear.x;
	    rot = vel.angular.z;

	    wr=((2*lin)+(0.053*rot))/(0.041);
	    wl=((2*lin)-(0.053*rot))/(0.041);
	    right_f=(1000/6.28)*wr;
	    left_f=(1000/6.28)*wl;
	    // Compute the maximum term between the linear and the angular speeds
	    std::pair<double, unsigned short> res = this->whosMax(fabs(right_f), fabs(left_f));

	    switch (res.second)
	    {
	      case 1:
	        right_f = SIGN(right_f) * vMax_.first;
	        left_f = left_f / res.first ;

	        break;
	      case 2:
	        right_f = right_f / res.first;
	        left_f = SIGN(left_f) * vMax_.second;
	        break;
	      default:
	        break;
	    }
	    wr=right_f*(6.28/1000);
	    wl=left_f*(6.28/1000);
	    vel.linear.x=(wr+wl)*(0.041/4);
	    vel.angular.z=(wr-wl)*(0.041/2)/0.053;

	    return;
	  }

	std::pair<double, unsigned short> whosMax(double a, double b)
	  {
	    std::pair<double, unsigned short> res;

	    double A = a / vMax_.first;
	    double B = b / vMax_.second;

	    if ((A > B) && (A > 1))
	    {
	      res.first = A;
	      res.second = 1;
	    }
	    else if ((B > A) && (B > 1))
	    {
	      res.first = B;
	      res.second = 2;
	    }
	    else
	    {
	      res.first = 1;
	      res.second = 3;
	    }
	    return (res);
	  }

};

#endif /* EPUCK_CONSENSUS_INCLUDE_EPUCK_CONSENSUS_ROBOPUCK_H_ */
