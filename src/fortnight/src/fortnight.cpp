#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "states.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

ros::Publisher twitter_p;
ros::Publisher docker_p;
ros::Publisher grabber_p;

uint32_t twitter_state = 0;
uint32_t docker_state = 0;
bool  grabber_pressed = 0;
void twitter_callback(const std_msgs::UInt32::ConstPtr &msg) {
	twitter_state = msg->data;
}
void docker_callback(const std_msgs::UInt32::ConstPtr &msg) {
	docker_state = msg->data;
}
void grabber_callback(const std_msgs::UInt32::ConstPtr &msg) {
	grabber_pressed = msg->data;
}

void twitter_do () {
	std_msgs::UInt32 at_home_msg;
	at_home_msg.data = COMMAND_ENABLE;
	twitter_p.publish(at_home_msg);
}

void docker_do () {
	std_msgs::UInt32 at_home_msg;
	at_home_msg.data = COMMAND_ENABLE;
	docker_p.publish(at_home_msg);
}
double point_yaw(double x, double y) {
	double yaw = atan(y/x);
	if (x < 0)
		yaw += M_PI;
	return yaw;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool go_to(double x, double y, double ori_x, double ori_y) {
  static MoveBaseClient ac("move_base", true);

	//tell the action client that we want to spin a thread by default
	move_base_msgs::MoveBaseGoal goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;

	double yaw = point_yaw(ori_x, ori_y);
	tf::Quaternion quat;
	quat.setRPY(0, 0, yaw);
	quaternionTFToMsg(quat, goal.target_pose.pose.orientation);

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Hooray, the base moved 1 meter forward");
		return true; 
	}
	else {
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		return false;
	}

}


// See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "fortnight");
	ros::NodeHandle nh;

	ros::Publisher vel_p = nh.advertise<geometry_msgs::Twist>("fortnight/output/cmd_vel", 1);

	ros::Rate loop_rate(10); // in Hz


	twitter_p   = nh.advertise<std_msgs::UInt32>("twitter/input/action", 10);
	grabber_p   = nh.advertise<std_msgs::UInt32>("grabber/input/action", 10);

	ros::Subscriber twitter_s = nh.subscribe("twitter/output/state", 10, twitter_callback);
	ros::Subscriber docker_s = nh.subscribe("twitter/output/state", 40, docker_callback);
	ros::Subscriber grabber_s = nh.subscribe("twitter/output/start_button", 40, grabber_callback);

	int state = -1; // 0 go to grab
	// 1 do grab :: twitter
	// 99 do grab :: twitter
	while (ros::ok()) {
		if ((state == -1 ) && (grabber_pressed)) {
			state = 0;
		}
		if (state == 0) {
			if (go_to(2.5, 1.6, 0, 1)) {
				state = 1;
			}
			else {
				state = 99;
			}
		}
		else if (state == 1) {
			twitter_do();
			if (twitter_state == STATE_CANCELLED) {
				state == 99;
			}
			if (twitter_state == STATE_FINISHED) {
				state == 2;
			}
		}
		else if (state == 2) {
			if (go_to(-1, 0, 1, -1)) {
				state = 3;
			}
			else {
				state = 99;
			}
		}
		else if (state == 3) {
			docker_do();
			if (docker_state == D_STATE_CANCELLED) {
				state == 99;
			}
			if (docker_state == D_STATE_FINISHED) {
				state == 0;
			}
		}
		else if (state == 99) {
			go_to(-2, 0, -1, -1);
			state = 0;
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
