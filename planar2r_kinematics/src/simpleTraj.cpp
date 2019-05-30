#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
#include <fstream>
	/* Write your code her for publishing to /pubJointStates topic
	** The message type is sensor_msgs/JointState 
	** The name field should be an array of names of all four joints
	** The header.stamp field should be ros::Time::now() 
	** The position field should be an array of double values
	** Keep filling the values inside the while(ros::ok()) loop
	** Elapsed time can be calculated as:
	** ros::Time start = ros::Time::now();
	** double diff = (ros::Time::now() - start).toSec();
	** Make the values sinusodial depending on variable diff or anything you like
	** Publish the msg 
	** The lines to be changed or added are marked*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "simepleTraj");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	double l1, l2;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	IRlibrary::Planar2R obj2r;
	obj2r.setLinks(l1, l2);
	std::ofstream fobj;
	fobj.open("~/trajError.txt");
	while (ros::ok())
	{

		diff = (ros::Time::now() - start).toSec();
		if(diff < 30) {
			double amp = 0.9 * ((l1 + l2) - std::fabs(l1 - l2))/2.;
			double midRad = ((l1 + l2) + std::fabs(l1 - l2))/2.;
			double rad = amp * cos(4 * diff/2.) + midRad;
			double ang = diff/2.;
			IRlibrary::Vec2 xy;
			xy << rad * cos(ang), midRad * sin(ang);
			obj2r.setXY(xy);
			auto q = obj2r.getConfig();
			obj2r.setConfig(q);
			auto xy1 = obj2r.getXY();
			double error_normSquared = (xy - xy1).squaredNorm();
			fobj << xy[0] << " " << xy[1] <<" " << error_normSquared<< std::endl;
			new_state.header.stamp = ros::Time::now();
			new_state.position[0] = q[0];
			new_state.position[1] = q[1];
			configPub.publish(new_state);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	fobj.close();
	return 0;
}
