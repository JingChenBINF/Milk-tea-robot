#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;
visualization_msgs::Marker bigCircle, smallCircle, midCircle1, midCircle2;
void drawWorkspace(ros::NodeHandle n) {
	double l1, l2, l3;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	n.getParam("link_lengths/l3", l3);
	bigCircle.type = smallCircle.type = midCircle1.type = midCircle2.type = visualization_msgs::Marker::LINE_STRIP;
	bigCircle.header.frame_id = "/base_link";
	bigCircle.header.stamp = ros::Time::now();
	smallCircle.header.frame_id = "/base_link";
	smallCircle.header.stamp = ros::Time::now();
	midCircle1.header.frame_id = "/base_link";
	midCircle1.header.stamp = ros::Time::now();
	midCircle2.header.frame_id = "/base_link";
	midCircle2.header.stamp = ros::Time::now();
	bigCircle.ns = smallCircle.ns = midCircle1.ns = midCircle2.ns = "basic_shapes";
	bigCircle.id = 1;
	smallCircle.id = 2;
	midCircle1.id = 3;
	midCircle2.id = 4;
	bigCircle.action = visualization_msgs::Marker::ADD;
	smallCircle.action = visualization_msgs::Marker::ADD;
	midCircle1.action = visualization_msgs::Marker::ADD;
	midCircle2.action = visualization_msgs::Marker::ADD;
	bigCircle.scale.x = smallCircle.scale.x = midCircle1.scale.x = midCircle2.scale.x = 0.05;
	double largerRad = l1 + l2 + l3;
	double smallerRad = abs(-l1 + l2 + l3);
	double midRad1 = abs(l1 + l2 - l3);
	double midRad2 = abs(l1 - l2 + l3);
	// Line strip is blue
	bigCircle.color.r = 0.25; smallCircle.color.r = 0.25;
	bigCircle.color.g = 0.25; smallCircle.color.g = 0.25;
	bigCircle.color.b = 0.25; smallCircle.color.b = 0.25;
	bigCircle.color.a = 1.0; smallCircle.color.a = 1.0;

	midCircle1.color.r = 0.25; midCircle2.color.r = 0.25;
	midCircle1.color.g = 0.25; midCircle2.color.g = 0.25;
	midCircle1.color.b = 0.25; midCircle2.color.b = 0.25;
	midCircle1.color.a = 1.0;  midCircle2.color.a = 1.0;
	for (uint32_t i = 0; i <= 90; ++i)
	{
		float x1 = largerRad * sin(i / 90.0f * 2 * M_PI);
		float y1 = largerRad * cos(i / 90.0f * 2 * M_PI);

		float x2 = smallerRad * sin(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);
		float y2 = smallerRad * cos(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);

		float x3 = midRad1 * sin(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);
		float y3 = midRad1 * cos(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);

		float x4 = midRad2 * sin(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);
		float y4 = midRad2 * cos(1.0f * M_PI / 90.0 + i / 90.0f * 2 * M_PI);

		geometry_msgs::Point p1, p2, p3, p4;
		p1.x = x1; p1.y = y1; p1.z = -0.1;
		p2.x = x2; p2.y = y2; p2.z = -0.1;
		p3.x = x3; p3.y = y3; p3.z = -0.1;
		p4.x = x4; p4.y = y4; p4.z = -0.1;

		bigCircle.points.push_back(p1);
		smallCircle.points.push_back(p2);
		midCircle1.points.push_back(p3);
		midCircle2.points.push_back(p4);
	}
}

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
	marker_pub.publish(midCircle1);
	marker_pub.publish(midCircle2);
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;

	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 0;

	marker.type = shape;

	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = -9;
	marker.pose.position.y = 7;
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
	drawWorkspace(n);

	marker.lifetime = ros::Duration();
	bigCircle.lifetime = ros::Duration();
	smallCircle.lifetime = ros::Duration();
	midCircle1.lifetime = ros::Duration();
	midCircle2.lifetime = ros::Duration();
	ros::Subscriber fkCheckSub;
	while (ros::ok())
  {
		fkCheckSub = n.subscribe("/fkCheck", 5, &fkCheckCallBack);
		ros::spin();
  }
}
