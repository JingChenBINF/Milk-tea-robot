#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
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
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise<sensor_msgs::JointState>("/pubJointStates", 1); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	while (ros::ok())
	{
		diff = (ros::Time::now() - start).toSec(); // Complete this
		new_state.header.stamp = ros::Time::now();
		new_state.position[0] = M_PI * cos(diff); // Complete this
		new_state.position[1] = M_PI * sin(diff); // Complete this
		/* Something important was published here */
		configPub.publish(new_state);
		/* This was important as well, something spinning */
		ros::spinOnce();
		/* Something related to sleep was here */
		sleep(0.1);
	}
	return 0;
}
