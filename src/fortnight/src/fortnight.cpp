#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
	//ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "fortnight");
	ros::NodeHandle nh;

	ros::Publisher vel_p = nh.advertise<geometry_msgs::Twist>("fortnight/output/cmd_vel", 1);
	ros::Subscriber odom_s = nh.subscribe("fortnight/input/odom", 1000, odom_callback);

	ros::Rate loop_rate(10); // in Hz

	int count = 0;
	while (ros::ok()) {
		geometry_msgs::Twist msg;

		msg.linear.x = 0.1;
		msg.angular.z = -0.2;

		ROS_INFO("speed %f rotation: %f", msg.linear.x, msg.angular.z);

		vel_p.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		count++;
	}

	return 0;
}
