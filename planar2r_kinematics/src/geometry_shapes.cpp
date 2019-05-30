#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

visualization_msgs::Marker marker;
visualization_msgs::Marker bigCircle;
visualization_msgs::Marker smallCircle;
ros::Publisher marker_pub;

void fkCheckCallBack(const std_msgs::Bool &msg) {
	if(msg.data == true) {
		marker.color.g = 1.0f;
		marker.color.r = 0.0f;
		marker.text = "Forward Kinematics is correct";
	}
	else {
		marker.color.g = 0.0f;
		marker.color.r = 1.0f;
		marker.text = "Please check Forward Kinematics";
	}
	marker_pub.publish(marker);
	marker_pub.publish(bigCircle);
	marker_pub.publish(smallCircle);
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
	double l1, l2;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = -6;
	marker.pose.position.y = 5;
	marker.pose.position.z = 1;
	marker.text = "Ready for Forward Kinematics";

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 0.3;

	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	bigCircle.type = smallCircle.type = visualization_msgs::Marker::LINE_STRIP;

	bigCircle.header.frame_id = "/base_link";
	smallCircle.header.frame_id = "/base_link";
	bigCircle.ns = smallCircle.ns = "workspace";
	bigCircle.id = 1;
	smallCircle.id = 2;
	bigCircle.action = smallCircle.action = visualization_msgs::Marker::ADD;
	bigCircle.scale.x = smallCircle.scale.x = 0.05;
	double largerRad = l1 + l2;
	double smallerRad = abs(l1 - l2);

	bigCircle.color.r = 0.25; smallCircle.color.r = 0.25;
	bigCircle.color.g = 0.25; smallCircle.color.g = 0.25;
	bigCircle.color.b = 0.25; smallCircle.color.b = 0.25;
	bigCircle.color.a = 1.0; smallCircle.color.a = 1.0;
	for (uint32_t i = 0; i <= 90; ++i)
	{
		float x1 = largerRad * sin(i / 90.0f * 2 * M_PI);
		float y1 = largerRad * cos(i / 90.0f * 2 * M_PI);

		float x2 = smallerRad * sin(i / 90.0f * 2 * M_PI);
		float y2 = smallerRad * cos(i / 90.0f * 2 * M_PI);

		geometry_msgs::Point p1, p2;
		p1.x = x1; p1.y = y1; p1.z = -0.1;
		p2.x = x2; p2.y = y2; p2.z = -0.1;

		bigCircle.points.push_back(p1);
		smallCircle.points.push_back(p2);
	}
	marker_pub.publish(bigCircle);
	marker_pub.publish(smallCircle);
	marker.lifetime = ros::Duration();
	bigCircle.lifetime = ros::Duration();
	smallCircle.lifetime = ros::Duration();

	ros::Subscriber fkCheckSub;
	ros::Subscriber configSub;
	while (ros::ok())
  {
		fkCheckSub = n.subscribe("/fkCheck", 5, &fkCheckCallBack);
		ros::spin();
  }
}
