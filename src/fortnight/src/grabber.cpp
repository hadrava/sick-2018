#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "grabber-actions.h"

ros::NodeHandle *nh_p;
ros::Publisher at_home_p;
ros::Publisher start_button_p;

bool at_home = true;
bool start_button = false;

int fd = -1;

int error_count = 0;
ros::Time last_error_stamp;

#define MY_ERROR(...)   do { \
	if ((ros::Time::now() - last_error_stamp).toSec() > 3) { \
		error_count = 0; \
	}\
	if (error_count < 5) { \
		error_count++; \
		ROS_ERROR(__VA_ARGS__); \
		last_error_stamp = ros::Time::now(); \
		if (error_count >= 5) \
			ROS_WARN("Suppressing errors for 3 seconds"); \
	} \
} while (0)

void hw_send_state(uint32_t data) {
	if (fd == -1) {
		MY_ERROR("Serial port is not open, check port settings");
		return;
	}

	uint8_t byte = data;
	int c = write(fd, &byte, 1);
	if (c != 1) {
		MY_ERROR("Write failed!");
		close(fd);
		fd = -1;
	}
}

void parse_data(uint8_t *buff) {
	if (buff[0] != 0x09) {
		MY_ERROR("Accepted corrupted packet: 0x%02x 0x%02x 0x%02x", buff[0], buff[1], buff[2]);
		return;
	}
	at_home = (buff[1] & 0xF7) == 0xA0;
	start_button = (buff[1] & 0x08) == 0x08;

	ROS_INFO("Valid packet: 0x%02x 0x%02x 0x%02x", buff[0], buff[1], buff[2]);
}

uint8_t buff[3];
int buff_index = 0;

void hw_receive_data() {
	if (fd == -1) {
		MY_ERROR("Serial port is not open, check port settings");
		return;
	}

	int ret;
	do {
		ret = read(fd, &buff[buff_index], 1);
		if (ret == 1) {
			if ((buff_index == 0) && (buff[0] != 0x09)) {
				ROS_WARN("Received corrupted data: 0x%02x", buff[0]);
				continue;
			}
			buff_index++;
			if (buff_index == 3) {
				buff_index = 0;
				parse_data(buff);
			}
		}
	} while (ret == 1);
}


int set_interface_attribs(int fd, int speed) {
	// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		MY_ERROR("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		MY_ERROR("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

bool real_init(const std::string &dev_name) {
	// ignored
	//int baud_rate;
	//nh_p->getParam("grabber/baud_rate", baud_rate);

	fd = open(dev_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NDELAY | O_NONBLOCK);

	if (fd == -1) {
		MY_ERROR("Unable to open serial_device");
		return false;
	}
	if (set_interface_attribs(fd, B115200)) {
		close(fd);
		fd = -1;
		return false;
	}
	buff_index = 0;
	return true;

}
void serial_init() {
	std::string device_name;

	nh_p->getParam("grabber/device", device_name);
	if (real_init(device_name))
		return;

	nh_p->getParam("grabber/alternative_device1", device_name);
	if (real_init(device_name))
		return;

	nh_p->getParam("grabber/alternative_device2", device_name);
	if (real_init(device_name))
		return;
}


void action_callback(const std_msgs::UInt32::ConstPtr &msg) {
	hw_send_state(msg->data);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "grabber");
	ros::NodeHandle nh;
	nh_p = &nh;

	// TODO something other than 8n1


	at_home_p = nh.advertise<std_msgs::Bool>("grabber/output/at_home", 10);
	start_button_p = nh.advertise<std_msgs::Bool>("grabber/output/start_button", 10);
	ros::Subscriber action_s = nh.subscribe("grabber/input/action", 10, action_callback);

	ros::Rate loop_rate(30); // in Hz

	while (ros::ok()) {
		if (fd == -1) {
			serial_init();
		}

		hw_receive_data();

		std_msgs::Bool at_home_msg;
		at_home_msg.data = at_home;
		at_home_p.publish(at_home_msg);

		std_msgs::Bool start_button_msg;
		start_button_msg.data = start_button;
		start_button_p.publish(start_button_msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
