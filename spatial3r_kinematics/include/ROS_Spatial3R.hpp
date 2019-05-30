#include "ros/ros.h"
#include "Spatial3R.hpp"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class ROS_Spatial3R {
	private:
		IRlibrary::Spatial3R obj3R;
		ros::Publisher fkCheckPub;
		ros::Subscriber configSub;
		std_msgs::Bool fk_check;
		tf::TransformListener listener;
	public:
		ROS_Spatial3R(ros::NodeHandle n) {
			double l1, l2, l3;
			n.getParam("link_lengths/l1", l1);
			n.getParam("link_lengths/l2", l2);
			n.getParam("link_lengths/l3", l3);
			obj3R.setLinks(l1, l2, l3);
			configSub = n.subscribe("/joint_states", 5, &ROS_Spatial3R::configCallBack, this);
			fkCheckPub = n.advertise <std_msgs::Bool> ("/fkCheck", 5);
		}

		void configCallBack(const sensor_msgs::JointState::ConstPtr &msg) {
			ros::Time then = msg->header.stamp;
			IRlibrary::Vec3 q {msg->position[0], msg->position[1], msg->position[2]};
			obj3R.setConfig(q);
			auto x = obj3R.getX();
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(x[0], x[1], x[2]) );
			tf::Quaternion quat;
			auto axis_angle = obj3R.getAxisAngle();
			tf::Vector3 axis_rot (axis_angle.omega[0], axis_angle.omega[1], axis_angle.omega[2]);
			double ang = axis_angle.theta;
			quat.setRotation(axis_rot, ang);
			transform.setRotation(quat);
			static tf::TransformBroadcaster br;
			tf::StampedTransform transform_endEffector;

			ros::Duration(0.01).sleep();
			try{
				listener.lookupTransform("/base_link", "/endEffector",  
						then, transform_endEffector);
				/* transform.setRotation(transform_endEffector.getRotation()); */
				if(abs(transform_endEffector.getOrigin().x() - x[0]) < 1e-5 && abs(transform_endEffector.getOrigin().y() - x[1]) < 1e-5 && abs(transform_endEffector.getOrigin().z() - x[2]) < 1e-5) {
					fk_check.data = true;
				}
				else 
					fk_check.data = false;
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			br.sendTransform(tf::StampedTransform(transform, then, "/base_link", "/fk_endEffector"));
			fkCheckPub.publish(fk_check);
		}
};
