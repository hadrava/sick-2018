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

#define SWAP(a,b) do { float tmp = (a); (a) = (b); (b) = tmp; } while(0)

ros::Publisher marker_p;
ros::Publisher cmd_vel_p;
tf::TransformListener *tf_lp;
geometry_msgs::PolygonStamped transporter_polygon;

struct transporter_observation {
	ros::Time stamp;
	geometry_msgs::PointStamped odom_point;
	double speed;
	geometry_msgs::Quaternion orientation;
	geometry_msgs::Quaternion orientation_speed;
};


uint32_t state;

std::vector<struct transporter_observation> history;


float param_transporter_radius; // radius of transporter base, documentation says: 0.1755
float param_x_offset; // x offset maintained during following
float param_y_offset; // y offset maintained during following should be at least radius of robot + radius of transporter
float param_control_speed_coef; // P coefficient of error in P regulator of forward speed
float param_control_yaw_speed_coef; // P coefficient of error in P regulator of turning speed

int param_detect_avg_size; // from how many measurements we should compute the average (20)
int param_detect_avg_distance; // distance betwwen middles of avg groups (20)
float param_detect_history_max_duration; // time between first and last used transporter measurements should be ideal_difference = (2* param_detect_avg_distance + param_detect_avg_size) * delay between scans; but this is ideal value, if all laser scans produces measurement. this coefficient sets maximal time difference to param_detect_history_max_duration * ideal difference
float param_minimal_transporter_speed; // minimal speed to calculate and predict transporter movement = 1/4 of nominal speed (0.2) = 0.05

bool param_use_fixed_angular_speed; // set to true, if angular speed is too much jumping
float param_transporter_angular_speed; // our transporter: -0.2
float param_speed_avg_size; // number of samples to calculate speed (10)

float param_max_ang_speed; // robot maximal angular speed (1.7)
float param_min_ang_speed; // robot maximal angular speed offset
float param_manipulation_angle; // 15 degrees = cca 0.25
#define LASER_HZ 15


void marker(uint32_t marker_id, const geometry_msgs::PoseStamped &pose_stamped, double length = 1.0, bool color = false);
double point_yaw(double x, double y);


bool valid_history() {
	if (history.size() < param_speed_avg_size)
		return false;

	int last_id = 2 * param_detect_avg_distance + param_detect_avg_size;
	if (history.size() >= last_id) {
		int sz = history.size();
		ros::Duration real_dur = history[sz - 1].stamp - history[sz - last_id].stamp;
		//return (real_dur.toSec() < (last_id - 1) * (1.0/LASER_HZ) * param_detect_history_max_duration);
		return real_dur.toSec() * LASER_HZ < (last_id - 1) * param_detect_history_max_duration;
	}
	else
		return false;
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

	ROS_INFO("poly_pt  %f, %f, %i", pt.point.x, pt.point.y, count);
	return count % 2;
}

void point_inside_polygon_test() {
	// call from main loop
	int id = 1;
	for (float x  = -3; x < -1; x += 0.3) {
	for (float y  = -1; y < 3; y += 0.3) {
	id++;
	geometry_msgs::PointStamped ptx;
	geometry_msgs::PoseStamped pt;
	pt.header.frame_id = "/map";
	pt.header.stamp = ros::Time::now();

	pt.pose.position.x = x;
	pt.pose.position.y = y;
	pt.pose.position.z = 0;

	ptx.point.x = x;
	ptx.point.y = y;
	ptx.point.z = 0;

	tf::Quaternion myQuaternion;
	myQuaternion.setRPY( 0, M_PI/2, 0);
	quaternionTFToMsg(myQuaternion, pt.pose.orientation);
	marker(id, pt, 0.2, point_inside_polygon(ptx, transporter_polygon));

	}
	}
}


void marker(uint32_t marker_id, const geometry_msgs::PoseStamped &pose_stamped, double length, bool color) {
	static ros::NodeHandle nh;
	visualization_msgs::Marker marker_m;
	marker_m.header.frame_id = pose_stamped.header.frame_id;
	marker_m.header.stamp = pose_stamped.header.stamp;

	marker_m.ns = "twitter_markers";
	marker_m.id = marker_id;

	marker_m.type = visualization_msgs::Marker::ARROW;
	marker_m.action = visualization_msgs::Marker::ADD;

	marker_m.pose = pose_stamped.pose;

	marker_m.scale.x = length;
	marker_m.scale.y = 0.02;
	marker_m.scale.z = 0.02;

	marker_m.color.r = !color;
	marker_m.color.g = color;
	marker_m.color.b = 0.0f;
	marker_m.color.a = 1.0f;

	marker_m.lifetime = ros::Duration(0.2);

	marker_p.publish(marker_m);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) { //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

double odom_log_x[64];
double odom_log_y[64];
int odom_log_b = 0;
int odom_log_e = 0;

void enable_callback(const std_msgs::Bool::ConstPtr &msg) {
	if (msg->data) {
		if ((state == STATE_DISABLED) || (state == STATE_CANCELLED) || (state == STATE_FINISHED))
			state = STATE_WAIT_FOR_OBJECT;
	}
	else {
		if (state == STATE_FOLLOWING)
			state = STATE_STORNO;
	}
}


geometry_msgs::PointStamped history_avg(int s_index, int cnt) { // s_index: first point to avg, cnt: count of points
	double x = 0.0;
	double y = 0.0;
	double t = 0.0;
	for (int i = s_index; i < s_index + cnt; i++) {
		x += history[i].odom_point.point.x - history[s_index].odom_point.point.x;
		y += history[i].odom_point.point.y - history[s_index].odom_point.point.y;
		ros::Duration dur = history[i].stamp - history[s_index].stamp;
		t += dur.toSec();
	}
	x /= cnt;
	y /= cnt;
	t /= cnt;

	geometry_msgs::PointStamped ret;
	ret.header = history[s_index].odom_point.header;
	ret.header.stamp = history[s_index].stamp + ros::Duration(t);
	ret.point.x = history[s_index].odom_point.point.x + x;
	ret.point.y = history[s_index].odom_point.point.y + y;
	ret.point.z = 0.0;

	return ret;
}

geometry_msgs::PoseStamped pose_from_two_points(const geometry_msgs::PointStamped &a, const geometry_msgs::PointStamped &b) { // a: new, b: old
	geometry_msgs::PoseStamped ret;
	ret.header = a.header;

	ros::Duration dur = a.header.stamp - b.header.stamp;
	ret.header.stamp = b.header.stamp + ros::Duration(dur.toSec() / 2);
	ret.pose.position.x = (a.point.x + b.point.x) / 2;
	ret.pose.position.y = (a.point.y + b.point.y) / 2;
	ret.pose.position.z = 0.0;

	double dx = a.point.x - b.point.x;
	double dy = a.point.y - b.point.y;

	double yaw = point_yaw(dx, dy);
	tf::Quaternion quat;
	quat.setRPY(0, 0, yaw);
	ROS_INFO("pose_from_two_points: %f %f", yaw, quat.getAngleShortestPath());

	quaternionTFToMsg(quat, ret.pose.orientation);

	return ret;
}

double speed_from_two_poses(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) { // a: new, b: old
	double dx = a.pose.position.x - b.pose.position.x;
	double dy = a.pose.position.y - b.pose.position.y;
	ros::Duration dur = a.header.stamp - b.header.stamp;

	double length = sqrt(dx * dx + dy * dy);

	return length / dur.toSec();
}

tf::Quaternion tf_quat_angle_multiply(const tf::Quaternion &q, double m) {
	tf::Quaternion ret;
	double angle = q.getAngle();
	if (angle > M_PI)
		angle -= 2 * M_PI;
	angle = angle * m;
	ret.setRotation(q.getAxis(), angle);
	return ret;
}

tf::Quaternion ang_speed_from_two_poses(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b) { // a: new, b: old
	tf::Quaternion aq(
			a.pose.orientation.x,
			a.pose.orientation.y,
			a.pose.orientation.z,
			a.pose.orientation.w
			);

	tf::Quaternion bq(
			b.pose.orientation.x,
			b.pose.orientation.y,
			b.pose.orientation.z,
			- b.pose.orientation.w
			);

	tf::Quaternion r = aq * bq;
	ros::Duration dur = a.header.stamp - b.header.stamp;

	tf::Quaternion ret = tf_quat_angle_multiply(r, 1 / dur.toSec());

	/*
	ROS_INFO("ang_speed %f, %f, dur: %f, %f %f %f", angle, ret.getAngleShortestPath(), dur.toSec(), r.getAngleShortestPath(), r.getAngle(), r.w());
	ROS_INFO("aq %f, bq: %f", aq.getAngleShortestPath(), bq.getAngleShortestPath());

	geometry_msgs::PoseStamped ps;
	ps.header.frame_id = "/map";
	ps.header.stamp = ros::Time::now();
	tf::Quaternion x(0, 1, 0, 0);
	tf::Quaternion rot = ret;
	x = rot * x * rot.inverse();

	quaternionTFToMsg(x, ps.pose.orientation);
	ps.pose.position.x = ps.pose.orientation.x;
	ps.pose.position.y = ps.pose.orientation.y;
	ps.pose.position.z = ps.pose.orientation.z;

	ps.pose.orientation.x = rot.x();
	ps.pose.orientation.y = rot.y();
	ps.pose.orientation.z = rot.z();
	ps.pose.orientation.w = rot.w();

	marker(3, ps, 0.2, false);
	*/

	return ret;
}

tf::Quaternion orientation_to_vector(tf::Quaternion rot, double length) {
	tf::Quaternion ret(length, 0, 0, 0);
	ret = rot * ret * rot.inverse();
	return ret;
}

double point_yaw(double x, double y) {
	double yaw = atan(y/x);
	if (x < 0)
		yaw += M_PI;
	return yaw;
}

void control_based_on_history() {
	int sz = history.size();
	ROS_INFO("c_b_o_h %i", sz);

	geometry_msgs::PointStamped avgs[3];
	avgs[0] = history_avg(sz - (param_detect_avg_size + 0 * param_detect_avg_distance), param_detect_avg_size); // newest
	avgs[1] = history_avg(sz - (param_detect_avg_size + 1 * param_detect_avg_distance), param_detect_avg_size);
	avgs[2] = history_avg(sz - (param_detect_avg_size + 2 * param_detect_avg_distance), param_detect_avg_size); // oldest

	geometry_msgs::PoseStamped dir[2];
	dir[0] = pose_from_two_points(avgs[0], avgs[1]);
	dir[1] = pose_from_two_points(avgs[1], avgs[2]);

	marker(1, dir[0], 0.2, false);
	marker(2, dir[1], 0.2, true);

	double apx_speed = speed_from_two_poses(dir[0], dir[1]);
	tf::Quaternion ang_speed(0, 0, 0, 1);
	if (param_use_fixed_angular_speed) {
		ang_speed.setRPY(0, 0, param_transporter_angular_speed);
	}
	else {
		if (apx_speed > param_minimal_transporter_speed) {
			ang_speed = ang_speed_from_two_poses(dir[0], dir[1]);
		}
	}


	double speed = 0.1;

	geometry_msgs::PointStamped pta, ptb;
	pta.header = dir[0].header;
	pta.point = dir[0].pose.position;
	ptb.header = dir[1].header;
	ptb.point = dir[1].pose.position;
	geometry_msgs::PoseStamped temp_dir = pose_from_two_points(pta, ptb);

	tf::Quaternion ori(
			temp_dir.pose.orientation.x,
			temp_dir.pose.orientation.y,
			temp_dir.pose.orientation.z,
			temp_dir.pose.orientation.w
			);
	ros::Duration ori_dur = history[sz - 1].stamp - temp_dir.header.stamp;
	ori = ori * tf_quat_angle_multiply(ang_speed, ori_dur.toSec());

	geometry_msgs::PoseStamped pt;
	pt.header.frame_id = "/odom";
	pt.header.stamp = history[sz - 1].stamp;
	pt.pose.position = history[sz - 1].odom_point.point;
	quaternionTFToMsg(ori, pt.pose.orientation);
	marker(4, pt, speed, true);



	if (state == STATE_ROTATING) {
		geometry_msgs::PointStamped tr_in_base;
		geometry_msgs::PointStamped next_tr_in_base;
		geometry_msgs::PointStamped tr_in_odom;
		geometry_msgs::PointStamped next_tr_in_odom;

		tr_in_odom = history[sz - 1].odom_point;

		tf::Quaternion nori(
				temp_dir.pose.orientation.x,
				temp_dir.pose.orientation.y,
				temp_dir.pose.orientation.z,
				temp_dir.pose.orientation.w
				);
		nori = nori * tf_quat_angle_multiply(ang_speed, ori_dur.toSec() + 1.0 / (LASER_HZ * 2));

		next_tr_in_odom = history[sz - 1].odom_point;
		tf::Quaternion mov = orientation_to_vector(nori, speed / (LASER_HZ * 2));
		next_tr_in_odom.point.x += mov.x();
		next_tr_in_odom.point.y += mov.y();
		try {
			tf_lp->waitForTransform("/odom", "/base_link", pt.header.stamp, ros::Duration(0.1));
			tf_lp->transformPoint("/base_link", tr_in_odom, tr_in_base);
			tf_lp->transformPoint("/base_link", next_tr_in_odom, next_tr_in_base);

			double yaw_ideal = point_yaw(0, -1);
			double yaw_tr = point_yaw(tr_in_base.point.x, tr_in_base.point.y);
			double yaw_next_tr = point_yaw(next_tr_in_base.point.x, next_tr_in_base.point.y);

			double diff_ri = yaw_tr - yaw_ideal;
			if (diff_ri > M_PI)
				diff_ri -= 2 * M_PI;
			double diff_nc = yaw_next_tr - yaw_tr;
			if (diff_nc > M_PI)
				diff_nc -= 2 * M_PI;

			ROS_INFO("real ideal %f %f diff %f", yaw_tr, yaw_ideal, diff_ri);
			ROS_INFO("next curr %f %f diff %f", yaw_next_tr, yaw_tr, diff_nc);

			geometry_msgs::Twist cmd_vel_m;
			cmd_vel_m.linear.x = 0;
			cmd_vel_m.angular.z = diff_nc * LASER_HZ + param_control_yaw_speed_coef * diff_ri;
			if (cmd_vel_m.angular.z < 0)
				cmd_vel_m.angular.z -= param_min_ang_speed;
			else
				cmd_vel_m.angular.z += param_min_ang_speed;
			if (cmd_vel_m.angular.z < -param_max_ang_speed)
				cmd_vel_m.angular.z = -param_max_ang_speed;
			if (cmd_vel_m.angular.z > param_max_ang_speed)
				cmd_vel_m.angular.z = param_max_ang_speed;

			ROS_INFO("speed %f rotation: %f", cmd_vel_m.linear.x, cmd_vel_m.angular.z);
			cmd_vel_p.publish(cmd_vel_m);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
		}
	}

	/*
	if (history.size() >= 3) {
		ros::Duration dur = history[history.size() - 1].stamp - history[history.size() - 3].stamp;

		tf::Quaternion h1_or_inv(
				history[history.size() - 2].orientation.x,
				history[history.size() - 2].orientation.y,
				history[history.size() - 2].orientation.z,
				- history[history.size() - 2].orientation.w
				);

		quat *= h1_or_inv;
		tf::Quaternion speed_norm(
				quat.x(),
				quat.y(),
				quat.z(),
				quat.w() / (dur.toSec() / 2)
			);

		speed_norm.normalize();
		quaternionTFToMsg(speed_norm, history[history.size() - 1].orientation_speed);

	}
	*/
}


void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg) { //ROS_INFO("I heard: [%s]", msg->data.c_str());
	if ((state == STATE_DISABLED) || (state == STATE_CANCELLED) || (state == STATE_FINISHED)) {
		return;
	}
	if (state == STATE_START) {
		history.clear();
		state = STATE_WAIT_FOR_OBJECT;
	}

	int size = msg->ranges.size();

	std::vector<float> median;
	median.push_back(msg->ranges[0]);
	for (int i = 0; i < size - 1; i++) {
		float rng[3];
		rng[0] = msg->ranges[i-1];
		rng[1] = msg->ranges[i];
		rng[2] = msg->ranges[i+1];
		if (rng[0] > rng[1])
			SWAP(rng[0], rng[1]);
		if (rng[1] > rng[2])
			SWAP(rng[1], rng[2]);
		if (rng[0] > rng[1])
			SWAP(rng[0], rng[1]);
		median.push_back(rng[1]);
	}
	median.push_back(msg->ranges[msg->ranges.size() - 1]);

	std::vector<float> inten_median;
	inten_median.push_back(msg->intensities[0]);
	for (int i = 0; i < size - 1; i++) {
		float rng[3];
		rng[0] = msg->intensities[i-1];
		rng[1] = msg->intensities[i];
		rng[2] = msg->intensities[i+1];
		if (rng[0] > rng[1])
			SWAP(rng[0], rng[1]);
		if (rng[1] > rng[2])
			SWAP(rng[1], rng[2]);
		if (rng[0] > rng[1])
			SWAP(rng[0], rng[1]);
		inten_median.push_back(rng[1]);
	}
	inten_median.push_back(msg->intensities[msg->intensities.size() - 1]);


	std::vector<bool> near_previous_beam;
	bool next_near = false;
	for (int i = 0; i < size; i++) {
		bool near = true;;

		if (next_near == false) {
			near = false;
			next_near = true;
		}
		if (!((median[i] >= msg->range_min) && (median[i] <= msg->range_max))) {
			near = false;
			next_near = false;
			//ROS_INFO("throw: i: %i median: %f", i, (double) median[i]);
		}

		if ((near) && (fabsf(median[i] - median[i - 1]) > 0.1)) {
			near = false;
			//ROS_INFO("jump: i: %i median: %f", i, (double) median[i]);
		}
		near_previous_beam.push_back(near);
		//ROS_INFO("i:  %i median: %f, near: %i", i, (double) median[i], int (near));
	}
	near_previous_beam.push_back(false);

	std::vector<int> block_begins;
	std::vector<int> block_ends;
	std::vector<float> block_range_min;
	std::vector<float> block_range_max;
	std::vector<float> block_inten_max;

	int block_size = 0;
	float r_min, r_max, i_max;

	for(int i = 1; i < size + 1; i++) {
		//ROS_INFO("i:  %i near: %i", i, (int) near_previous_beam[i]);
		if (near_previous_beam[i] && !near_previous_beam[i - 1]) {
			r_max = median[i-1];
			r_min = median[i-1];
			i_max = inten_median[i-1];
			block_size = 1;
		}
		if (near_previous_beam[i]) {
			if (r_max < median[i])
				r_max = median[i];
			if (r_min > median[i])
				r_min = median[i];
			if (i_max < inten_median[i])
				i_max = inten_median[i];
			block_size++;
		}
		if (!near_previous_beam[i] && near_previous_beam[i - 1]) {
			if (block_size > 10) {
				block_begins.push_back(i - block_size);
				block_ends.push_back(i);
				block_range_min.push_back(r_min);
				block_range_max.push_back(r_max);
				block_inten_max.push_back(i_max);
				//ROS_INFO("brmi brmx %f %f", r_min, r_max);
				//ROS_INFO("save edge, i:  %i block_size: %i", i, block_size);
			}
			else {
				//ROS_INFO("drop edge, i:  %i block_size: %i", i, block_size);
			}
		}
	}

	std::vector<bool> block_size_ok;
	for(int b = 0; b < block_begins.size(); b++) {
		float range = block_range_max[b] - block_range_min[b];
		double min_angle = msg->angle_min + block_begins[b] * msg->angle_increment;
		double max_angle = msg->angle_min + block_ends[b] * msg->angle_increment;

		float sx = cos(min_angle) * block_range_max[b] - cos(max_angle) * block_range_max[b];
		float sy = sin(min_angle) * block_range_max[b] - sin(max_angle) * block_range_max[b];
		if ((range <= 0.4f) && (sx*sx + sy*sy < 0.4 * 0.4)) {
			block_size_ok.push_back(true);
			//ROS_INFO("green, b:  %i range  %f, sx %f, sy %f", b, range, sx, sy);
		}
		else {
			//ROS_INFO("red, b:  %i range  %f, sx %f, sy %f", b, range, sx, sy);
			block_size_ok.push_back(false);
		}
	}

	// filter block_position (polygon)
	float min_range_max = std::numeric_limits<float>::infinity();
	for (int b = 0; b < block_size_ok.size(); b++) {
		if (block_size_ok[b]) {
			int i = (block_ends[b] + block_begins[b]) / 2;
			double angle = msg->angle_min + i * msg->angle_increment;
			float range = block_range_max[b] - block_range_min[b];
			ros::Time stamp = msg->header.stamp + ros::Duration(i * msg->time_increment);

			// pt to map
			geometry_msgs::PointStamped point_laser;
			geometry_msgs::PointStamped point_map;
			point_laser.header.frame_id = "/laser";
			point_laser.header.stamp = stamp;
			point_laser.point.x = cos(angle) * (block_range_min[b] + range/2 + param_transporter_radius);
			point_laser.point.y = sin(angle) * (block_range_min[b] + range/2 + param_transporter_radius);
			point_laser.point.z = 0;
			try {
				tf_lp->waitForTransform("/laser", "/map", stamp, ros::Duration(0.1));
				tf_lp->transformPoint("/map", point_laser, point_map);
				//ROS_INFO("odom :%f %f", point_odom.point.x, point_odom.point.y);

				block_size_ok[b] = point_inside_polygon(point_map, transporter_polygon);
				if ((block_size_ok[b]) && (min_range_max > block_range_max[b])) {
					min_range_max = block_range_max[b];
				}
			}
			catch (tf::TransformException &ex) {
				ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
				block_size_ok[b] = false;
			}

		}
	}

	/*
	int count_ip = 0;
	for (int b = 0; b < block_begins.size(); b++) {
		count_ip += block_size_ok[b];
	}
	ROS_INFO("l_c: inside polygon count :%i, min_range_max: %f", count_ip, min_range_max);
	*/


	float max_rssi = -100;
	int transporter_block = -1;
	for(int b = 0; b < block_begins.size(); b++) {
		//if (block_size_ok[b]) {
		//	ROS_INFO("l_c: brm[b]: %f", block_range_min[b]);
		//}
		if (block_size_ok[b] && (block_range_min[b] < min_range_max + 0.1)) {
			for (int i = block_begins[b]; i < block_ends[b]; i++) {
				if (max_rssi < inten_median[i]) {
					max_rssi = inten_median[i];
					transporter_block = b;
				}
			}
		}
	}

	//ROS_INFO("l_c: transporter_block :%i", transporter_block);


	bool transporter_detected = false;
	geometry_msgs::PointStamped point_laser;
	geometry_msgs::PointStamped point_odom;
	if (transporter_block != -1) {
		int i_s = 0;
		int i_c = 0;
		double range = 0.;
		for (int i = block_begins[transporter_block]; i < block_ends[transporter_block]; i++) {
			if ((msg->ranges[i] == msg->ranges[i]) && (msg->intensities[i] == max_rssi)) {
				range += msg->ranges[i];
				i_s += i;
				i_c++;
			}
		}
		ROS_INFO("i_c :%i", i_c);
		if (i_c) {
			range /= i_c;

			double i = i_s / i_c;
			double angle = msg->angle_min + i * msg->angle_increment;
			ros::Time stamp = msg->header.stamp + ros::Duration(i * msg->time_increment);

			tf::Quaternion myQuaternion;
			myQuaternion.setRPY( 0, 0, angle);

			geometry_msgs::PoseStamped lp;
			lp.header.frame_id = "/laser";
			lp.header.stamp = stamp;
			lp.pose.position.x = cos(angle) * (range + param_transporter_radius);
			lp.pose.position.y = sin(angle) * (range + param_transporter_radius);
			lp.pose.position.z = 0;
			myQuaternion.setRPY( 0, M_PI/2, 0);
			quaternionTFToMsg(myQuaternion, lp.pose.orientation);
			marker(0, lp, 0.2, false);


			// to odom and back
			point_laser.header.frame_id = "/laser";
			point_laser.header.stamp = stamp;
			point_laser.point.x = cos(angle) * (range + param_transporter_radius);
			point_laser.point.y = sin(angle) * (range + param_transporter_radius);
			point_laser.point.z = 0;

			try {
				tf_lp->waitForTransform("/laser", "/odom", stamp, ros::Duration(0.1));
				tf_lp->transformPoint("/odom", point_laser, point_odom);
				transporter_detected = true;
				ROS_INFO("odom :%f %f", point_odom.point.x, point_odom.point.y);
			}
			catch (tf::TransformException &ex) {
				ROS_ERROR("Failure %s\n", ex.what()); // Print exception which was caught
			}
		}
	}

	if (transporter_detected) {
		struct transporter_observation ob;
		ob.stamp = point_laser.header.stamp;
		point_odom.header.stamp = point_laser.header.stamp;
		ob.odom_point = point_odom;
		history.push_back(ob);
	}

	if (state == STATE_WAIT_FOR_OBJECT) {
		if (valid_history()) {
			state = STATE_ROTATING;
		}
	}
	if ((state == STATE_ROTATING) || (state == STATE_FOLLOWING)) {
		if (valid_history()) {
			control_based_on_history();
		}
	}

	ROS_INFO("l_c %i", __LINE__);

	if ((state == STATE_START) || (state == STATE_WAIT_FOR_OBJECT)) {
		geometry_msgs::Twist cmd_vel_m;
		cmd_vel_m.linear.x = 0;
		cmd_vel_m.angular.z = 0;
		ROS_INFO("speed %f rotation: %f", cmd_vel_m.linear.x, cmd_vel_m.angular.z);
		cmd_vel_p.publish(cmd_vel_m);
	}

	/*
	// show arrows of all potential blocks
	int green_count = 0;
	for(int b = 0; b < block_begins.size(); b++) {
		int i = (block_ends[b] + block_begins[b]) / 2;
		float range = block_range_max[b] - block_range_min[b];
		double angle = msg->angle_min + i * msg->angle_increment;
		ros::Time stamp = msg->header.stamp + ros::Duration(i * msg->time_increment);

		//tf::StampedTransform transform;
		//try{
		//	tf_l.lookupTransform("/base_link", "/laser", stamp, transform);
		//}
		//catch (tf::TransformException ex){
		//	ROS_ERROR("%s",ex.what());
		//	return;
		//}

		tf::Quaternion myQuaternion;
		myQuaternion.setRPY( 0, 0, angle);

		geometry_msgs::PoseStamped lp;
		lp.header.frame_id = "/laser";
		lp.header.stamp = stamp;
		lp.pose.position.x = cos(angle) * block_range_min[b];
		lp.pose.position.y = sin(angle) * block_range_min[b];
		lp.pose.position.z = -0.05;
		quaternionTFToMsg(myQuaternion, lp.pose.orientation);

		if (block_size_ok[b]) {
			green_count++;

			geometry_msgs::PoseStamped lp2;
			lp2.header.frame_id = "/laser";
			lp2.header.stamp = stamp;
			lp2.pose.position.x = cos(angle) * (block_range_min[b] + range/2 + param_transporter_radius);
			lp2.pose.position.y = sin(angle) * (block_range_min[b] + range/2 + param_transporter_radius);
			lp2.pose.position.z = 0;

			myQuaternion.setRPY( 0, M_PI/2, 0);
			quaternionTFToMsg(myQuaternion, lp2.pose.orientation);
			marker(3+b*2, lp2, 0.2, block_size_ok[b]);
		}

		//marker(2+b*2, lp, range, block_size_ok[b]);
		//ROS_INFO("add marker: angle: %f, range: %f", angle, (double) range);
	}
	ROS_INFO("Number of blocks: %lu, green: %i", block_begins.size(), green_count);
	*/


		geometry_msgs::PoseStamped p;
		p.header.frame_id = "/laser";
		p.header.stamp = ros::Time::now();

		p.pose.position.x = 0;
		p.pose.position.y = 0;
		p.pose.position.z = 0;
		p.pose.orientation.x = 0.0;
		p.pose.orientation.y = 0.0;
		p.pose.orientation.z = 0.0;
		p.pose.orientation.w = 1.0;

		//marker(1, p);
}


// See http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
int main(int argc, char **argv) {
	ros::init(argc, argv, "twitter");

	ros::NodeHandle nh;
	tf::TransformListener tf_l;
	tf_lp = &tf_l;

	state = STATE_DISABLED;
	// marker init
	marker_p  = nh.advertise<visualization_msgs::Marker>("twitter_marker", 1);
	cmd_vel_p = nh.advertise<geometry_msgs::Twist>("twitter/output/cmd_vel", 1);

	ros::Publisher polygon_p = nh.advertise<geometry_msgs::PolygonStamped>("twitter/output/polygon", 1);
	ros::Publisher state_p   = nh.advertise<std_msgs::UInt32>("twitter/output/state", 1);

	std::vector<float> transporter_polygon_vect;
	nh.getParam("twitter/transporter_polygon", transporter_polygon_vect);


	//ros::Subscriber odom_s = nh.subscribe("twitter/input/odom", 1000, odom_callback);
	ros::Subscriber laser_s = nh.subscribe("twitter/input/scan", 10, laser_callback);
	ros::Subscriber enable_s = nh.subscribe("twitter/input/enable", 1, enable_callback);

	ros::Rate loop_rate(30); // in Hz

	transporter_polygon.header.frame_id = "/map";
	transporter_polygon.header.stamp = ros::Time::now();
	for (int i = 0; i + 1 < transporter_polygon_vect.size(); i += 2) {
		geometry_msgs::Point32 pt;
		pt.x = transporter_polygon_vect[i];
		pt.y = transporter_polygon_vect[i + 1];
		pt.z = 0.;
		transporter_polygon.polygon.points.push_back(pt);
		ROS_INFO("poly_pt  %f, %f", pt.x, pt.y);
	}

	while (ros::ok()) {
		nh.getParam("twitter/transporter_radius", param_transporter_radius);
		nh.getParam("twitter/x_offset", param_x_offset);
		nh.getParam("twitter/y_offset", param_y_offset);
		nh.getParam("twitter/control_speed_coef", param_control_speed_coef);
		nh.getParam("twitter/control_yaw_speed_coef", param_control_yaw_speed_coef);

		nh.getParam("twitter/detect_avg_size", param_detect_avg_size);
		nh.getParam("twitter/detect_avg_distance", param_detect_avg_distance);
		nh.getParam("twitter/detect_history_max_duration", param_detect_history_max_duration);

		nh.getParam("twitter/minimal_transporter_speed", param_minimal_transporter_speed);
		nh.getParam("twitter/use_fixed_angular_speed", param_use_fixed_angular_speed);
		nh.getParam("twitter/transporter_angular_speed", param_transporter_angular_speed);

		nh.getParam("twitter/speed_avg_size", param_speed_avg_size);

		nh.getParam("twitter/max_ang_speed", param_max_ang_speed);
		nh.getParam("twitter/min_ang_speed", param_min_ang_speed);
		nh.getParam("twitter/manipulation_angle", param_manipulation_angle);




		std_msgs::UInt32 state_msg;
		state_msg.data = state;
		state_p.publish(state_msg);

		transporter_polygon.header.stamp = ros::Time::now();
		polygon_p.publish(transporter_polygon);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
