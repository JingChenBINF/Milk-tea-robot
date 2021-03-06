#include "ros/ros.h"
#include "Planar3R.hpp"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class ROS_Planar3R {
	private:
		IRlibrary::Planar3R obj3R;
		ros::Publisher fkCheckPub;
		ros::Subscriber configSub;
		std_msgs::Bool fk_check;
		tf::TransformListener listener;
	public:
		ROS_Planar3R(ros::NodeHandle n) {
			double l1, l2, l3;
			n.getParam("link_lengths/l1", l1);
			n.getParam("link_lengths/l2", l2);
			n.getParam("link_lengths/l3", l3);
			obj3R.setLinks(l1, l2, l3);
			configSub = n.subscribe("/joint_states", 5, &ROS_Planar3R::configCallBack, this);
			fkCheckPub = n.advertise <std_msgs::Bool> ("/fkCheck", 5);
		}

		void configCallBack(const sensor_msgs::JointState::ConstPtr &msg) {
			ros::Time then = msg->header.stamp;
			IRlibrary::Vec3 q {msg->position[0], msg->position[1], msg->position[2]};
			obj3R.setConfig(q);
			auto x = obj3R.getX();
			tf::Transform transform;
			transform.setOrigin( tf::Vector3(x[0], x[1], 0.0) );
			tf::Quaternion quat;
			quat.setRPY(0, 0, x[2]);
			transform.setRotation(quat);
			static tf::TransformBroadcaster br;
			br.sendTransform(tf::StampedTransform(transform, then, "/base_link", "/fk_endEffector"));
			tf::StampedTransform transform_endEffector;

			ros::Duration(0.01).sleep();
			try{
				listener.lookupTransform("/base_link", "/endEffector",  
						then, transform_endEffector);
			}
			catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
			if(abs(transform_endEffector.getOrigin().x() - x[0]) < 1e-10 && abs(transform_endEffector.getOrigin().y() - x[1]) < 1e-10) {
				std::cout << "equation x " << x[0] << " ros x " << transform_endEffector.getOrigin().x() << std::endl;
				std::cout << "equation y " << x[1] << " ros y " << transform_endEffector.getOrigin().y() << std::endl;

				fk_check.data = true;
			}
			else 
				fk_check.data = false;
			fkCheckPub.publish(fk_check);
		}
};
