
#include "ROS_Planar3R.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forwardKinematics");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ROS_Planar3R ros_3r(n);
	ros::spin();

	while (ros::ok())
	{
		/* ros_3r.posePublish(); */
		/* ros::spinOnce(); */
		/* loop_rate.sleep(); */
	}


	return 0;
}
