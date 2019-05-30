#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar3R.hpp>
#include <fstream>
#include <iostream>
#include <ros/package.h>

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
double randomFloat(double a, double b) 
{
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}

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
	new_state.name = {"joint1", "joint2", "joint3"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff), 0.0};
	double l1 = 0.0;
	double l2 = 0.0;
	double l3 = 0.0;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	n.getParam("link_lengths/l3", l3);
	IRlibrary::Planar3R obj3r(l1, l2, l3);
	std::string pkg_path = ros::package::getPath("planar3r_kinematics");

	std::ofstream file_error_aik, file_error_nik, file_aik1, file_aik2, file_nik, file_time;
	file_error_aik.open(pkg_path + "/data/planar3rTrajError_aik.txt");
	file_error_nik.open(pkg_path + "/data/planar3rTrajError_nik.txt");
	file_aik1.open(pkg_path + "/data/analytical_ik_1.txt");
	file_aik2.open(pkg_path + "/data/analytical_ik_2.txt");
	file_nik.open(pkg_path + "/data/numerical_ik.txt");
	file_time.open(pkg_path + "/data/time.txt");
	double ang = 0.0;
	long num_of_ik_queries = 0;
	double nik_time = 0.0;
	double aik_time = 0.0;
	while (ros::ok())
	{
		diff = (ros::Time::now() - start).toSec();
		double rad = 3.0;
		ang += 0.1;
		double range_ang = IRlibrary::wrapTo2PI(ang);
		IRlibrary::Vec3 xy;
		xy << rad * cos(range_ang), rad * sin(range_ang), range_ang;
		obj3r.setX(xy);

		/* 
			analytical 
		*/
		// analytical ik
		std::vector<IRlibrary::Vec3> analytical_ik;
	 	ros::Time start = ros::Time::now();
		if (!obj3r.inverseKinematics(xy, analytical_ik))
			ROS_ERROR("Failed to get IK analytically");
		ros::Time end = ros::Time::now();
		aik_time += (end-start).toNSec();

		if (ang < 6.29)
		{
			file_aik1 << analytical_ik[1][0] << " " << analytical_ik[1][1] << " " << analytical_ik[1][2] << std::endl;
			file_aik2 << analytical_ik[0][0] << " " << analytical_ik[0][1] << " " << analytical_ik[0][2] << std::endl;
		}
		
		// fk of analytical ik
		obj3r.setConfig(analytical_ik[1]);
		IRlibrary::Vec3 xy1 = obj3r.getX();
		IRlibrary::Vec3 error = xy - xy1;
		if (ang < 6.29)
			file_error_aik << pow(error[0], 2) + pow(error[1], 2) << " " << error[2] << std::endl;

		/* 
			numerical 
		*/
		// numerical ik
		Eigen::VectorXd numerical_ik(3);
		numerical_ik.setZero();
		IRlibrary::Vec3 initial_thetaList;
		initial_thetaList[0] = analytical_ik[1][0] + randomFloat(-0.2, 0.2);
		initial_thetaList[1] = analytical_ik[1][1] + randomFloat(-0.2, 0.2);
		initial_thetaList[2] = analytical_ik[1][2] + randomFloat(-0.2, 0.2);
		start = ros::Time::now();
		if (!obj3r.inverseKinematics_numerical(initial_thetaList, xy, numerical_ik))
			ROS_ERROR("Numerical IK failed");
		// numerical_ik = obj3r.getConfig();
		end = ros::Time::now();
		nik_time += (end-start).toNSec();

		if (ang < 6.29)
			file_nik << numerical_ik[0] << " " << numerical_ik[1] << " " << numerical_ik[2] << std::endl;

		// fk of numerical ik
		IRlibrary::Vec3 nik(numerical_ik[0], numerical_ik[1], numerical_ik[2]);
		obj3r.setConfig(nik);
		xy1 = obj3r.getX();
		error = xy - xy1;
		if (ang < 6.29)
			file_error_nik << pow(error[0], 2) + pow(error[1], 2) << " " << error[2] << std::endl;

		/* 
			visualize in rviz
		*/
		new_state.header.stamp = ros::Time::now();
		// ik1
		// new_state.position[0] = analytical_ik[0][0];
		// new_state.position[1] = analytical_ik[0][1];
		// new_state.position[2] = analytical_ik[0][2];

		// ik2
		new_state.position[0] = analytical_ik[1][0];
		new_state.position[1] = analytical_ik[1][1];
		new_state.position[2] = analytical_ik[1][2];
		
		// new_state.position[0] = numerical_ik[0];
		// new_state.position[1] = numerical_ik[1];
		// new_state.position[2] = numerical_ik[2];
		configPub.publish(new_state);
		ros::spinOnce();
		loop_rate.sleep();

		num_of_ik_queries++;

		if (diff > 100.0)
			break;
	}
	file_time << num_of_ik_queries << " " << nik_time / 1e6 << " ms " << aik_time /1e6 << " ms "<< std::endl;
	file_error_aik.close();
	file_error_nik.close();
	file_aik1.close();
	file_aik2.close();
	file_nik.close();
	return 0;
}
