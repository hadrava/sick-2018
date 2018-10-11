#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>

#include "grabber-actions.h"

ros::Publisher at_home_p;
uint32_t last_state = GRABBER_DISABLED;
bool at_home = true;

void hw_set_prepare() { }
void hw_set_grab() { }
void hw_set_home() { }
bool hw_get_at_home() { return true; }

void action_callback(const std_msgs::UInt32::ConstPtr &msg) {
	if (msg->data == GRABBER_PREPARE) {
		if (last_state != GRABBER_PREPARE) {
			hw_set_prepare();
		}
		last_state = GRABBER_PREPARE;
	}
	if (msg->data == GRABBER_GRAB) {
		if (last_state != GRABBER_GRAB) {
			hw_set_grab();
		}
		last_state = GRABBER_GRAB;
	}
	if (msg->data == GRABBER_DISABLED) {
		if (last_state != GRABBER_DISABLED) {
			hw_set_home();
		}
		last_state = GRABBER_GRAB;
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "grabber");
	ros::NodeHandle nh;

	at_home_p = nh.advertise<std_msgs::Bool>("grabber/output/at_home", 10);
	ros::Subscriber action_s = nh.subscribe("grabber/input/action", 10, action_callback);

	ros::Rate loop_rate(30); // in Hz

	while (ros::ok()) {
		std_msgs::Bool at_home_msg;
		at_home_msg.data = hw_get_at_home();
		if (last_state == GRABBER_DISABLED)
			at_home_msg.data = true;
		if (last_state == GRABBER_PREPARE)
			at_home_msg.data = false;
		at_home_p.publish(at_home_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
