
#include "ROS_Planar2R.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "forwardKinematics");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ROS_Planar2R ros_2r(n);
	ros::spin();

	while (ros::ok())
	{
		/* ros_2r.posePublish(); */
		/* ros::spinOnce(); */
		/* loop_rate.sleep(); */
	}


	return 0;
}
