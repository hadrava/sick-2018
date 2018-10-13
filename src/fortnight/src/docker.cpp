#include <math.h>
#include <limits>
#include <vector>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>

#include "states.h"
#include "grabber-actions.h"

#define SWAP(a,b) do { double tmp = (a); (a) = (b); (b) = tmp; } while(0)

ros::Publisher marker_p;
ros::Publisher cmd_vel_p;
ros::Publisher grabber_p;
tf::TransformListener *tf_lp;
geometry_msgs::PolygonStamped storage_box_polygon;

struct box_observation {
	ros::Time stamp;
	geometry_msgs::PointStamped a_in_odom;
	geometry_msgs::PointStamped b_in_odom;
	geometry_msgs::PointStamped center_in_odom;
	tf::Quaternion orientation;
};

uint32_t state;

std::vector<struct box_observation> history;


// special parameter:
//                        storage_box_polygon: [ ]
float param_storage_box_yaw; // direction of storage box (this is direction of long side), we will arive at this direction

float param_x_offset_entering; // first point to which we should arrive (relative to detected storage box
float param_y_offset_entering;

float param_x_offset; // second point, we are disposing balls at this position
float param_y_offset;

float param_x_offset_leaving; // third point, used for leaving storage box area, should not be in collision with walls or box
float param_y_offset_leaving;

int param_detect_avg_size; // from how many measurements we should compute the average (20)
float param_detect_history_max_duration; // load 1.5? (copy from twitter)

float param_max_ang_speed; // robot maximal angular speed (1.7)
float param_min_ang_speed; // robot maximal angular speed offset
float param_control_yaw_speed_coef; // P coefficient of error in P regulator of turning speed
float param_manipulation_angle; // 15 degrees = cca 0.25

float param_forward_speed; // fixed forward speed
float param_y_error_to_yaw_speed_coef; // y to yaw
float param_yaw_error_to_yaw_speed_coef; // yaw to yaw

float param_release_wait; //  3
float param_box_edge_length; //  0.4
float param_box_edge_width; //  0.05 (unused)

#define LASER_HZ 15


void marker(uint32_t marker_id, const geometry_msgs::PoseStamped &pose_stamped, double length = 1.0, bool color = false);

void marker(uint32_t marker_id, const geometry_msgs::PoseStamped &pose_stamped, double length, bool color) {
	static ros::NodeHandle nh;
	visualization_msgs::Marker marker_m;
	marker_m.header.frame_id = pose_stamped.header.frame_id;
	marker_m.header.stamp = pose_stamped.header.stamp;

	marker_m.ns = "docker_markers";
	marker_m.id = marker_id;

	marker_m.type = visualization_msgs::Marker::ARROW;
	marker_m.action = visualization_msgs::Marker::ADD;

	marker_m.pose = pose_stamped.pose;

	marker_m.scale.x = length;
	marker_m.scale.y = 0.02;
	marker_m.scale.z = 0.02;

	marker_m.color.r = 0.0f;
	marker_m.color.g = color;
	marker_m.color.b = 1.0f;
	marker_m.color.a = 1.0f;

	marker_m.lifetime = ros::Duration(0.2);

	marker_p.publish(marker_m);
}

double point_yaw(double x, double y) {
	double yaw = atan(y/x);
	if (x < 0)
		yaw += M_PI;
	return yaw;
}


bool valid_history() {
	if (history.size() < param_detect_avg_size)
		return false;

	int sz = history.size();
	ros::Duration real_dur = history[sz - 1].stamp - history[sz - param_detect_avg_size].stamp;
	//return (real_dur.toSec() < (sz - 1) * (1.0/LASER_HZ) * param_detect_history_max_duration);
	return real_dur.toSec() * LASER_HZ < (sz - 1) * param_detect_history_max_duration;
}


bool point_inside_polygon(const geometry_msgs::PointStamped &pt, const geometry_msgs::PolygonStamped &poly) {
	int count = 0;
	for (int i = 0; i < poly.polygon.points.size(); i++) {
		int j = i + 1;
		j = j % poly.polygon.points.size();
		int k = i;


		if (poly.polygon.points[j].x == poly.polygon.points[k].x)
			continue;
		if (poly.polygon.points[j].x > poly.polygon.points[k].x) {
			SWAP(j, k);
		}
		if (pt.point.x < poly.polygon.points[j].x)
			continue;
		if (poly.polygon.points[k].x <= pt.point.x)
			continue;

		double sx = pt.point.x - poly.polygon.points[j].x;
		double x = poly.polygon.points[k].x - poly.polygon.points[j].x;

		double sy = pt.point.y - poly.polygon.points[j].y;
		double y = poly.polygon.points[k].y - poly.polygon.points[j].y;
		//ROS_INFO("sx x sy y  %f, %f, %f, %f", sx, x, sy, y);
		if (y*sx > sy * x) { // if (y/x*sx > sy)
			count++;
		}
	}

	//ROS_INFO("poly_pt  %f, %f, %i", pt.point.x, pt.point.y, count);
	return count % 2;
}


void enable_callback(const std_msgs::UInt32::ConstPtr &msg) {
	if (msg->data == COMMAND_RESET) {
		state = D_STATE_DISABLED;
	}
	else if (msg->data == COMMAND_ENABLE) {
		if (state == D_STATE_DISABLED)
			state = D_STATE_START;
	}
	else { // COMMAND DISABLE
		if ((state == D_STATE_START) || (state == D_STATE_ROTATE_TO_1)) {
			state = D_STATE_CANCELLED;
		}
		else {
			state = D_STATE_MOVE_TO_3;
		}
	}
}

bool grabber_at_home = true;
void grabber_callback(const std_msgs::Bool::ConstPtr &msg) { // grabber is at home
	grabber_at_home = msg->data;
}


tf::Quaternion orientation_to_vector(tf::Quaternion rot, double length) {
	tf::Quaternion ret(length, 0, 0, 0);
	ret = rot * ret * rot.inverse();
	return ret;
}

bool rotation_control(const geometry_msgs::PointStamped &next_point_in_base) { // returns true if next_point is exactly* in front of us, *) within manipulation_angle
	bool we_are_locked_on = false;

	double yaw_ideal = point_yaw(1, 0);
	double yaw_tr = point_yaw(next_point_in_base.point.x, next_point_in_base.point.y);

	double yaw_error = yaw_tr - yaw_ideal;
	if (yaw_error > M_PI)
		yaw_error -= 2 * M_PI;
	if (yaw_error < - M_PI)
		yaw_error += 2 * M_PI;
	if ((yaw_error < param_manipulation_angle) && (yaw_error > -param_manipulation_angle)) {
		we_are_locked_on = true;
	}

	ROS_INFO("rotation control: real yaw %f, ideal yaw: %f, diff: %f", yaw_tr, yaw_ideal, yaw_error);

	geometry_msgs::Twist cmd_vel_m;
	cmd_vel_m.linear.x = 0;
	cmd_vel_m.angular.z = param_control_yaw_speed_coef * yaw_error;
	if (cmd_vel_m.angular.z < 0)
		cmd_vel_m.angular.z -= param_min_ang_speed;
	else
		cmd_vel_m.angular.z += param_min_ang_speed;

	if (cmd_vel_m.angular.z < -param_max_ang_speed)
		cmd_vel_m.angular.z = -param_max_ang_speed;
	if (cmd_vel_m.angular.z > param_max_ang_speed)
		cmd_vel_m.angular.z = param_max_ang_speed;

	ROS_INFO("rotation: speed %f rotation: %f, locked on: %i", cmd_vel_m.linear.x, cmd_vel_m.angular.z, (int) we_are_locked_on);
	cmd_vel_p.publish(cmd_vel_m);

	return we_are_locked_on;
}

bool forward_control(const geometry_msgs::PointStamped &next_point_in_base, const geometry_msgs::PointStamped &shifted_next_point_in_base) {
	ROS_INFO("forward_control: x: %f y: %f", next_point_in_base.point.x, next_point_in_base.point.y);
	geometry_msgs::Twist cmd_vel_m;

	if (next_point_in_base.point.x < 0) {
		cmd_vel_m.linear.x = 0;
		cmd_vel_m.angular.z = 0;
		cmd_vel_p.publish(cmd_vel_m);

		return true; // point reached
	}
	else {
		double y_error_yaw_speed = param_y_error_to_yaw_speed_coef * next_point_in_base.point.y;

		double yaw_error = point_yaw(shifted_next_point_in_base.point.x - next_point_in_base.point.x, shifted_next_point_in_base.point.y - next_point_in_base.point.y);
		if (yaw_error > M_PI)
			yaw_error -= 2 * M_PI;
		if (yaw_error < - M_PI)
			yaw_error += 2 * M_PI;
		double yaw_error_yaw_speed = yaw_error * param_yaw_error_to_yaw_speed_coef;

		cmd_vel_m.linear.x = param_forward_speed;
		cmd_vel_m.angular.z = y_error_yaw_speed + yaw_error_yaw_speed;

		if (cmd_vel_m.angular.z < - param_max_ang_speed)
			cmd_vel_m.angular.z = - param_max_ang_speed;
		if (cmd_vel_m.angular.z > param_max_ang_speed)
			cmd_vel_m.angular.z = param_max_ang_speed;

		ROS_INFO("forward_controll speed %f rotation: %f (y_e*c: %f, yaw_e*c: %f)", param_forward_speed, y_error_yaw_speed + yaw_error_yaw_speed, y_error_yaw_speed, yaw_error_yaw_speed);
		cmd_vel_p.publish(cmd_vel_m);
	}
	return false;
}

bool both_controls_with_caclulations(const geometry_msgs::PointStamped &center_in_odom, const tf::Quaternion &orientation, double x_offset, double y_offset, bool rotate_only) { // returns true if we should change our state (we are locked on)
	bool we_are_locked_on;
	geometry_msgs::PointStamped next_point_in_odom;
	geometry_msgs::PointStamped next_point_in_base;
	next_point_in_odom = center_in_odom;
	tf::Quaternion point(x_offset, y_offset, 0, 0);
	point = orientation * point * orientation.inverse();
	next_point_in_odom.point.x += point.x();
	next_point_in_odom.point.y += point.y();

	geometry_msgs::PointStamped shifted_next_point_in_odom;
	geometry_msgs::PointStamped shifted_next_point_in_base;
	shifted_next_point_in_odom = center_in_odom;
	tf::Quaternion shifted_point(x_offset + 1, y_offset, 0, 0);
	shifted_point = orientation * shifted_point * orientation.inverse();
	shifted_next_point_in_odom.point.x += shifted_point.x();
	shifted_next_point_in_odom.point.y += shifted_point.y();
	try {
		tf_lp->waitForTransform("/odom", "/base_link", next_point_in_odom.header.stamp, ros::Duration(0.1));
		tf_lp->transformPoint("/base_link", next_point_in_odom, next_point_in_base);
		tf_lp->transformPoint("/base_link", shifted_next_point_in_odom, shifted_next_point_in_base);

		if (rotate_only)
			return rotation_control(next_point_in_base);
		else
			return forward_control(next_point_in_base, shifted_next_point_in_base);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
	}

	return false;
}

struct box_observation history_avg(int s_index, int cnt) { // s_index: first point to avg, cnt: count of points
	return history[history.size() - 1];
}

void control_based_on_history() {
	int sz = history.size();
	ROS_INFO("c_b_o_h %i", sz);

	box_observation avg;
	avg = history_avg(sz - param_detect_avg_size, param_detect_avg_size);

	// Display observation marker
	geometry_msgs::PoseStamped arrow;
	arrow.header.frame_id = "/odom";
	arrow.header.stamp = avg.stamp;
	arrow.pose.position = avg.a_in_odom.point;
	quaternionTFToMsg(avg.orientation, arrow.pose.orientation);
	marker(4, arrow, param_box_edge_length, true);


	// Real Control
	double x_offset = 0;
	double y_offset = 0;
	bool rotate_only = false;
	switch (state) {
		case D_STATE_ROTATE_TO_1:
			rotate_only = true;
		case D_STATE_MOVE_TO_1:
			x_offset = param_x_offset_entering;
			y_offset = param_y_offset_entering;
			break;
		case D_STATE_ROTATE_TO_2:
			rotate_only = true;
		case D_STATE_MOVE_TO_2:
			x_offset = param_x_offset;
			y_offset = param_y_offset;
			break;
		case D_STATE_MOVE_TO_3:
			x_offset = param_x_offset_leaving;
			y_offset = param_y_offset_leaving;
			break;
	}
	if ((state == D_STATE_DISPOSE_START) || (state == D_STATE_DISPOSE_WAIT) || (state == D_STATE_DISPOSE_END)) {
		geometry_msgs::Twist cmd_vel_m;
		cmd_vel_m.linear.x = 0;
		cmd_vel_m.angular.z = 0;
		ROS_INFO("disposing, motor stopped");
		cmd_vel_p.publish(cmd_vel_m);

		static ros::Time start_to_wait_stamp;
		if ((state == D_STATE_DISPOSE_START) && !grabber_at_home) {
			start_to_wait_stamp = ros::Time::now();
			state++;
		}
		else if ((state == D_STATE_DISPOSE_WAIT) && ((ros::Time::now() - start_to_wait_stamp).toSec() > param_release_wait)) {
			state++;
		}
		else if ((state == D_STATE_DISPOSE_END) && grabber_at_home) {
			state++;
		}
	}
	else {
		bool finished = both_controls_with_caclulations(avg.center_in_odom, avg.orientation, x_offset, y_offset, rotate_only);
		if (finished)
			state++;
	}
}



double metrics_of_point_acording_to_vector(const geometry_msgs::PointStamped &pt, const tf::Quaternion &vector) {
	return pt.point.x * vector.y() - pt.point.y * vector.x();
}



void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) { //ROS_INFO("I heard: [%s]", msg->data.c_str());
	if ((state == D_STATE_DISABLED) || (state == D_STATE_CANCELLED) || (state == D_STATE_FINISHED)) {
		return;
	}
	if (state == D_STATE_START) {
		history.clear();
		state++;
	}

	int size = msg->ranges.size();

	ros::Time middle_scan_stamp = msg->header.stamp + ros::Duration(size/2 * msg->time_increment);

	geometry_msgs::PolygonStamped storage_box_polygon_in_laser;
	tf::Quaternion middle_storage_box_yaw_in_laser;
	try {
		// convert polygon to laser
		tf_lp->waitForTransform("/map", "/laser", middle_scan_stamp, ros::Duration(0.1));
		geometry_msgs::PointStamped corner_in_map;
		geometry_msgs::PointStamped corner_in_laser;
		corner_in_map.header.stamp = middle_scan_stamp;
		corner_in_map.header.frame_id = "/map";

		storage_box_polygon_in_laser.header.stamp = middle_scan_stamp;
		storage_box_polygon_in_laser.header.frame_id = "/laser";

		for (int i = 0; i < storage_box_polygon.polygon.points.size(); i++) {
			corner_in_map.point.x = storage_box_polygon.polygon.points[i].x;
			corner_in_map.point.y = storage_box_polygon.polygon.points[i].y;
			tf_lp->transformPoint("/laser", corner_in_map, corner_in_laser);
			geometry_msgs::Point32 corner_2;
			corner_2.x = corner_in_laser.point.x;
			corner_2.y = corner_in_laser.point.y;
			corner_2.z = 0;
			storage_box_polygon_in_laser.polygon.points.push_back(corner_2);
		}

		// convert yaw to laser:
		geometry_msgs::PointStamped zero_in_laser;
		geometry_msgs::PointStamped direction_in_laser;
		corner_in_map.point.x = 0;
		corner_in_map.point.y = 0;
		corner_in_map.point.z = 0;
		tf_lp->transformPoint("/laser", corner_in_map, zero_in_laser);
		corner_in_map.point.x = cos(param_storage_box_yaw);
		corner_in_map.point.y = sin(param_storage_box_yaw);
		tf_lp->transformPoint("/laser", corner_in_map, direction_in_laser);

		double storage_box_yaw_in_laser = point_yaw(
				direction_in_laser.point.x - zero_in_laser.point.x,
				direction_in_laser.point.y - zero_in_laser.point.y
				);
		middle_storage_box_yaw_in_laser.setRPY(0, 0, storage_box_yaw_in_laser);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
		return;
	}
	// now we have polygon in laser: storage_box_polygon_in_laser;

	std::vector<geometry_msgs::PointStamped> detected_points_filtered;
	for (int i = 0; i < size; i++) {
		double angle = msg->angle_min + i * msg->angle_increment;
		geometry_msgs::PointStamped point_laser_in_laser;
		point_laser_in_laser.header.frame_id = "/laser";
		point_laser_in_laser.header.stamp = msg->header.stamp + ros::Duration(i * msg->time_increment);

		point_laser_in_laser.point.x = cos(angle) * msg->ranges[i];
		point_laser_in_laser.point.y = sin(angle) * msg->ranges[i];
		if (point_inside_polygon(point_laser_in_laser, storage_box_polygon_in_laser)) {
			detected_points_filtered.push_back(point_laser_in_laser);
		}
	}
	int size_f = detected_points_filtered.size();
	// now we have detected_points_filtered which are only those points inside polygon

	double max_grad = - std::numeric_limits<double>::infinity();
	tf::Quaternion optimal_direction;
	double opt_min_met;
	double opt_max_met;
	int opt_idx;
	for (float y_shift = -1.0; y_shift <= 1.0; y_shift += 0.01) {
		tf::Quaternion tested_direction(1.0, y_shift, 0, 0);
		tested_direction = middle_storage_box_yaw_in_laser * tested_direction * middle_storage_box_yaw_in_laser.inverse();
		// tested_direction is vector x,y, 0,0 in same direction as stripes

		double min_met = std::numeric_limits<double>::infinity();
		double max_met = - std::numeric_limits<double>::infinity();
		for (int i = 0; i < size_f; i++) {
			double met = metrics_of_point_acording_to_vector(detected_points_filtered[i], tested_direction);
			if (min_met > met)
				min_met = met;
			if (max_met < met)
				max_met = met;
		}
		int array[102];
		for (int idx = 0; idx < 101; idx++)
			array[idx] = 0;
		for (int i = 0; i < size_f; i++) {
			double met = metrics_of_point_acording_to_vector(detected_points_filtered[i], tested_direction);
			int idx = 1.0 + ((met - min_met) * 100) / (max_met - min_met);
			array[idx]++;
		}
		double grad = 0;
		int max_array = 0;
		for (int idx = 0; idx < 101; idx++) {
			grad += (array[idx] - array[idx + 1]) * (array[idx] - array[idx + 1]);
			if (max_array < array[idx]) {
				max_array = array[idx];
			}
		}
		if (grad > max_grad) {
			max_grad = grad;
			optimal_direction = tested_direction;
			opt_min_met = min_met;
			opt_max_met = max_met;
			for (int idx = 0; idx < 101; idx++) {
				if (max_array < array[idx] * 2) {
					opt_idx = idx;
					break;
				}
			}
		}
	}
	// Now se have optimal_direction, vector in laser which points to direction with maximal gradient
	// and all opt_{min,max}_met opt_idx

	// filter points and calculate their avg:
	int count_pts = 0;
	double avg_x = 0;
	double avg_y = 0;
	double avg_i = 0;
	for (int i = 0; i < size_f; i++) {
		double met = metrics_of_point_acording_to_vector(detected_points_filtered[i], optimal_direction);
		int idx = 1.0 + ((met - opt_min_met) * 100) / (opt_max_met - opt_min_met);
		if (idx == opt_idx) {
			count_pts++;
			avg_x += detected_points_filtered[i].point.x;
			avg_y += detected_points_filtered[i].point.y;
			avg_i += i;
		}
	}
	avg_x = avg_x / count_pts;
	avg_y = avg_y / count_pts;
	avg_i = avg_i / count_pts;
	// now we have central point in laser and its time


	ros::Time center_stamp = msg->header.stamp + ros::Duration(avg_i * msg->time_increment);
	geometry_msgs::PointStamped center_in_laser;

	center_in_laser.header.frame_id = "/laser";
	center_in_laser.header.stamp = center_stamp;
	center_in_laser.point.x = avg_x;
	center_in_laser.point.y = avg_y;
	center_in_laser.point.z = 0.0;

	geometry_msgs::PointStamped a_in_laser = center_in_laser;
	geometry_msgs::PointStamped b_in_laser = center_in_laser;

	optimal_direction.normalize();
	a_in_laser.point.x -= optimal_direction.x() * param_box_edge_length / 2;
	a_in_laser.point.y -= optimal_direction.y() * param_box_edge_length / 2;

	b_in_laser.point.x += optimal_direction.x() * param_box_edge_length / 2;
	b_in_laser.point.y += optimal_direction.y() * param_box_edge_length / 2;

	geometry_msgs::PointStamped center_in_odom;
	geometry_msgs::PointStamped a_in_odom;
	geometry_msgs::PointStamped b_in_odom;

	try {
		tf_lp->waitForTransform("/laser", "/odom", center_stamp, ros::Duration(0.1));
		tf_lp->transformPoint("/odom", center_in_laser, center_in_odom);
		tf_lp->transformPoint("/odom", a_in_laser, a_in_odom);
		tf_lp->transformPoint("/odom", b_in_laser, b_in_odom);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
		return;
	}

	double yaw = point_yaw(b_in_odom.point.x - a_in_odom.point.x, b_in_odom.point.y - a_in_odom.point.y);
	tf::Quaternion orientation;
	orientation.setRPY(0, 0, yaw);

	struct box_observation newest_obs;
	newest_obs.stamp = center_stamp;
	newest_obs.a_in_odom = a_in_odom;
	newest_obs.b_in_odom = b_in_odom;
	newest_obs.center_in_odom = center_in_odom;
	newest_obs.orientation = orientation;

	history.push_back(newest_obs);

	if ((state >= D_STATE_START) && (state <= D_STATE_MOVE_TO_3) && valid_history()) {
		control_based_on_history();
	}
	else {
		geometry_msgs::Twist cmd_vel_m;
		cmd_vel_m.linear.x = 0;
		cmd_vel_m.angular.z = 0;
		ROS_INFO("speed %f rotation: %f", cmd_vel_m.linear.x, cmd_vel_m.angular.z);
		cmd_vel_p.publish(cmd_vel_m);
	}
}

// See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "docker");

	ros::NodeHandle nh;
	tf::TransformListener tf_l;
	tf_lp = &tf_l;

	state = D_STATE_DISABLED;
	// marker init
	marker_p  = nh.advertise<visualization_msgs::Marker>("docker_marker", 10);
	cmd_vel_p = nh.advertise<geometry_msgs::Twist>("docker/output/cmd_vel", 1);
	grabber_p = nh.advertise<std_msgs::UInt32>("docker/output/grabber_action", 10);

	ros::Publisher polygon_p = nh.advertise<geometry_msgs::PolygonStamped>("docker/output/polygon", 1);
	ros::Publisher state_p   = nh.advertise<std_msgs::UInt32>("docker/output/state", 1);

	std::vector<float> storage_box_polygon_vect;
	nh.getParam("docker/storage_box_polygon", storage_box_polygon_vect);


	ros::Subscriber laser_s = nh.subscribe("docker/input/scan", 10, laser_callback);
	ros::Subscriber enable_s = nh.subscribe("docker/input/enable", 40, enable_callback);
	ros::Subscriber grabber_s = nh.subscribe("docker/input/grabber_at_home", 40, grabber_callback);

	ros::Rate loop_rate(30); // in Hz

	storage_box_polygon.header.frame_id = "/map";
	storage_box_polygon.header.stamp = ros::Time::now();
	for (int i = 0; i + 1 < storage_box_polygon_vect.size(); i += 2) {
		geometry_msgs::Point32 pt;
		pt.x = storage_box_polygon_vect[i];
		pt.y = storage_box_polygon_vect[i + 1];
		pt.z = 0.;
		storage_box_polygon.polygon.points.push_back(pt);
		ROS_INFO("poly_pt  %f, %f", pt.x, pt.y);
	}

	while (ros::ok()) {
		nh.getParam("grabber/storage_box_yaw", param_storage_box_yaw);

		nh.getParam("grabber/x_offset_entering", param_x_offset_entering);
		nh.getParam("grabber/y_offset_entering", param_y_offset_entering);

		nh.getParam("grabber/x_offset", param_x_offset);
		nh.getParam("grabber/y_offset", param_y_offset);

		nh.getParam("grabber/x_offset_leaving", param_x_offset_leaving);
		nh.getParam("grabber/y_offset_leaving", param_y_offset_leaving);

		nh.getParam("grabber/detect_avg_size", param_detect_avg_size);
		nh.getParam("grabber/detect_history_max_duration", param_detect_history_max_duration);

		nh.getParam("grabber/max_ang_speed",          param_max_ang_speed         );
		nh.getParam("grabber/min_ang_speed",          param_min_ang_speed         );
		nh.getParam("grabber/control_yaw_speed_coef", param_control_yaw_speed_coef);
		nh.getParam("grabber/manipulation_angle",     param_manipulation_angle    );

		nh.getParam("grabber/forward_speed",               param_forward_speed              );
		nh.getParam("grabber/y_error_to_yaw_speed_coef",   param_y_error_to_yaw_speed_coef  );
		nh.getParam("grabber/yaw_error_to_yaw_speed_coef", param_yaw_error_to_yaw_speed_coef);

		nh.getParam("grabber/release_wait",    param_release_wait   );
		nh.getParam("grabber/box_edge_length", param_box_edge_length);
		nh.getParam("grabber/box_edge_width",  param_box_edge_width );




		std_msgs::UInt32 state_msg;
		state_msg.data = state;
		state_p.publish(state_msg);

		std_msgs::UInt32 grabber_msg;
		grabber_msg.data = GRABBER_DISABLED;
		if (state == D_STATE_DISPOSE_START)
			grabber_msg.data = GRABBER_RELEASE;
		if (state == STATE_DISABLED) {
			// Do not send action
			// Note: cancelled and finished states sends actions, so remember to reset docker before using grabber from other node
		}
		else {
			grabber_p.publish(grabber_msg);
		}

		storage_box_polygon.header.stamp = ros::Time::now();
		polygon_p.publish(storage_box_polygon);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
