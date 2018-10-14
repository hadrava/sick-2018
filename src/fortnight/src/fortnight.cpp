#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "states.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

ros::Publisher twitter_p;
ros::Publisher docker_p;
ros::Publisher grabber_p;
ros::Publisher goal_p;

	bool arrived;
	bool failed ;
uint32_t twitter_state = 0;
uint32_t docker_state = 0;
bool  grabber_pressed = 0;
void twitter_callback(const std_msgs::UInt32::ConstPtr &msg) {
	twitter_state = msg->data;
}
void docker_callback(const std_msgs::UInt32::ConstPtr &msg) {
	docker_state = msg->data;
}
void grabber_callback(const std_msgs::Bool::ConstPtr &msg) {
	grabber_pressed = msg->data;
}
void goal_callback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {

	int sz = msg->status_list.size() ;
	if (sz) {
	int status = msg->status_list[sz-1].status;
	if (status == 3) { 
		arrived = true;
		failed = false;
	}
	if (status == 1) { 
		arrived = false;
		failed = false;
	}
	if (status == 0) { 
		arrived = false;
		failed = false;
	}
	if (status == 4) { 
		arrived = false;
		failed = true;
	}
	ROS_INFO("move_status %i", status);
	}
}

void twitter_do () {
	std_msgs::UInt32 at_home_msg;

	at_home_msg.data = COMMAND_ENABLE;
	twitter_p.publish(at_home_msg);
}
void twitter_donot () {
	std_msgs::UInt32 at_home_msg;

	at_home_msg.data = COMMAND_RESET;
	twitter_p.publish(at_home_msg);
}


void docker_do () {
	std_msgs::UInt32 at_home_msg;
	at_home_msg.data = COMMAND_ENABLE;
	docker_p.publish(at_home_msg);
}
void docker_donot () {
	std_msgs::UInt32 at_home_msg;
	at_home_msg.data = COMMAND_RESET;
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
	static int seq = 0;
	arrived = false;
	failed = false;

  //static MoveBaseClient ac("move_base", true);

	//tell the action client that we want to spin a thread by default
  geometry_msgs::PoseStamped goal;

	//we'll send a goal to the robot to move 1 meter forward
	goal.header.frame_id = "map";
	goal.header.stamp = ros::Time::now();
	goal.header.seq = seq++;

	goal.pose.position.x = x;
	goal.pose.position.y = y;

	double yaw = point_yaw(ori_x, ori_y);
	tf::Quaternion quat;
	quat.setRPY(0, 0, yaw);
	quaternionTFToMsg(quat, goal.pose.orientation);

	ROS_INFO("Sending goal");
	//ac.sendGoal(goal);

	goal_p.publish(goal);



	/*
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
		ROS_INFO("Hooray, the base moved 1 meter forward");
		return true; 
	}
	else {
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		return false;
	}
	*/

}


// See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "fortnight");
	ros::NodeHandle nh;

	ros::Publisher vel_p = nh.advertise<geometry_msgs::Twist>("fortnight/output/cmd_vel", 1);

	ros::Rate loop_rate(10); // in Hz


	twitter_p   = nh.advertise<std_msgs::UInt32>("twitter/input/enable", 10);
	grabber_p   = nh.advertise<std_msgs::UInt32>("grabber/input/action", 10);
	docker_p   = nh.advertise<std_msgs::UInt32>("docker/input/enable", 10);
	goal_p   = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

	ros::Subscriber twitter_s = nh.subscribe("twitter/output/state", 10, twitter_callback);
	ros::Subscriber docker_s = nh.subscribe("docker/output/state", 40, docker_callback);
	ros::Subscriber grabber_s = nh.subscribe("grabber/output/start_button", 40, grabber_callback);
	ros::Subscriber goal_s = nh.subscribe("move_base/status", 40, goal_callback);

	int state = 2; // 0 go to grab
	// 1 do grab :: twitter
	// 99 do grab :: twitter
	while (ros::ok()) {
		if ((state == -1 ) && (grabber_pressed)) {
			state = 0;
		}
		else if (state == 0) {
			docker_donot();
			go_to(2.5, 1.6, 0, 1);
			state = 81;
		}
		else if (state == 81) {
			if (arrived)
				state = 1;
			if (failed)
				state = 99;
		}
		else if (state == 1) {
			twitter_do();
			if (twitter_state == STATE_CANCELLED) {
				state == 99;
			}
			if (twitter_state == STATE_FINISHED) {
				state == 2;
			}
			if (twitter_state == STATE_LEAVING) {
				state == 2;
			}
		}
		else if (state == 2) {
			twitter_donot();
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
			twitter_donot();
			docker_donot();
			go_to(-2, 0, -1, -1);
			state = 0;
		}

		ROS_INFO("state: %i", state);
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
