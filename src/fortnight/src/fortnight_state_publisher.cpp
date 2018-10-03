#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "fortnight_state_publisher");
	ros::NodeHandle node;

	ros::Rate loop_rate(30); // in Hz

	while (ros::ok()) {
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(0.2, -0.2, 0.004) );
		tf::Quaternion q;
		q.setRPY(M_PI, 0, -M_PI/4);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "laser_link"));

		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}

	ros::spin();
	return 0;
};
