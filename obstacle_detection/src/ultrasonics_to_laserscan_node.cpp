/**
 * \file ultrasonics_to_laserscan_node.cpp
 * \author: sunil
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <novus_msgs/UltrasonicFeedback.h>

/**
 * Global variables
 */
novus_msgs::UltrasonicFeedback g_ultrasonic_data;
double g_angle_min;
double g_angle_max;
double g_angle_increment;
double g_time_increment;
double g_scan_time;
double g_range_min;
double g_range_max;
double g_voltage_to_range_scale=1;

double g_front_right_x;
double g_front_right_y;
double g_front_right_theta;

double g_front_left_x;
double g_front_left_y;
double g_front_left_theta;

double g_back_right_x;
double g_back_right_y;
double g_back_right_theta;

double g_back_left_x;
double g_back_left_y;
double g_back_left_theta;

bool is_data_received=false;

/**
 * @brief ultrasonic sensor feedback callback
 * @param feedback_ptr feedback pointer
 */
void ultrasonic_feedback_callback(const novus_msgs::UltrasonicFeedbackConstPtr feedback_ptr)
{
		g_ultrasonic_data.front_right = feedback_ptr->front_right/g_voltage_to_range_scale;
		g_ultrasonic_data.front_left = feedback_ptr->front_left/g_voltage_to_range_scale;
		g_ultrasonic_data.back_right = feedback_ptr->back_right/g_voltage_to_range_scale;
		g_ultrasonic_data.back_left = feedback_ptr->back_left/g_voltage_to_range_scale;
}

struct Point
{
	double x;
	double y;
};

double angle_zero_to_2pi(double angle)
{
	if(angle < 0)
		return angle+360;
	else
		return angle;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"ultrasonics_to_laserscan_node");

	ros::NodeHandle nh;
	ros::Subscriber sub_ultrasonic = nh.subscribe("/ultrasonic_feedback",1,ultrasonic_feedback_callback);
	ros::Publisher pub_laserscan = nh.advertise<sensor_msgs::LaserScan>("/ultrasonic_laserscan",1);

	bool is_param_read = true;

	is_param_read = is_param_read && nh.getParam("/ultrasonics/angle_min", g_angle_min);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/angle_max", g_angle_max);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/angle_increment", g_angle_increment);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/time_increment", g_time_increment);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/scan_time", g_scan_time);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/range_min", g_range_min);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/range_max", g_range_max);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/voltage_to_range_scale", g_voltage_to_range_scale);

	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_right_x", g_front_right_x);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_right_y", g_front_right_y);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_right_theta", g_front_right_theta);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_left_x", g_front_left_x);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_left_y", g_front_left_y);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/front_left_theta", g_front_left_theta);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_right_x", g_back_right_x);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_right_y", g_back_right_y);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_right_theta", g_back_right_theta);

	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_left_x", g_back_left_x);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_left_y", g_back_left_y);
	is_param_read = is_param_read && nh.getParam("/ultrasonics/back_left_theta", g_back_left_theta);

	if(!is_param_read)
	{
		ROS_ERROR_ONCE("Could not load all parameters");
		exit(-1);
	}

	ROS_INFO("angle_min:%lf, angle_max:%lf,angle_increment:%lf",g_angle_min, g_angle_max, g_angle_increment);

	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.frame_id="/ultrasonic";
	scan_msg.angle_min = 0;
	scan_msg.angle_max = 2*M_PI;
	scan_msg.angle_increment = g_angle_increment;
	scan_msg.time_increment = g_time_increment;
	scan_msg.scan_time = g_scan_time;
	scan_msg.range_min = g_range_min;
	scan_msg.range_max = g_range_max;

	int no_of_points = (int)2*M_PI/g_angle_increment;
	for (int i=0; i<no_of_points; i++)
	{
		scan_msg.ranges.push_back(0);
	}

	ros::Rate r(30);
	while (ros::ok())
	{
		for (int i=0; i<no_of_points; i++)
		{
			scan_msg.ranges[i]=0;
		}

		double angle = g_angle_min;

		while (angle < g_angle_max)
		{
			Point fr_skewed,fl_skewed,br_skewed,bl_skewed;
			fr_skewed.x = g_ultrasonic_data.front_right * cos(angle);
			fr_skewed.y = g_ultrasonic_data.front_right * sin(angle);

			fl_skewed.x = g_ultrasonic_data.front_left * cos(angle);
			fl_skewed.y = g_ultrasonic_data.front_left * sin(angle);

			br_skewed.x = g_ultrasonic_data.back_right * cos(angle);
			br_skewed.y = g_ultrasonic_data.back_right * sin(angle);

			bl_skewed.x = g_ultrasonic_data.back_left * cos(angle);
			bl_skewed.y = g_ultrasonic_data.back_left * sin(angle);

			Point fr,fl,br,bl;

			double half_fov = 0;//(g_angle_max - g_angle_min)/2;
			fr.x = fr_skewed.x*cos(half_fov) - fr_skewed.y*sin(half_fov);
			fr.y = fr_skewed.x*sin(half_fov) + fr_skewed.y*cos(half_fov);

			fl.x = fl_skewed.x*cos(half_fov) - fl_skewed.y*sin(half_fov);
			fl.y = fl_skewed.x*sin(half_fov) + fl_skewed.y*cos(half_fov);

			br.x = br_skewed.x*cos(half_fov) - br_skewed.y*sin(half_fov);
			br.y = br_skewed.x*sin(half_fov) + br_skewed.y*cos(half_fov);

			bl.x = bl_skewed.x*cos(half_fov) - bl_skewed.y*sin(half_fov);
			bl.y = bl_skewed.x*sin(half_fov) + bl_skewed.y*cos(half_fov);

			int index_i=0;
			Point fr_base, fl_base, br_base, bl_base;
			fr_base.x = fr.x*cos(g_front_right_theta) + fr.y*sin(g_front_right_theta) + g_front_right_x;
			fr_base.y = -fr.x*sin(g_front_right_theta) + fr.y*cos(g_front_right_theta) + g_front_right_y;
			index_i = round(angle_zero_to_2pi(atan2(fr_base.y,fr_base.x)*180/M_PI));
			double dist = sqrt(fr_base.x*fr_base.x + fr_base.y*fr_base.y);
			if(dist < 1)
				scan_msg.ranges[index_i] = dist;

			fl_base.x = fl.x*cos(g_front_left_theta) + fl.y*sin(g_front_left_theta) + g_front_left_x;
			fl_base.y = -fl.x*sin(g_front_left_theta) + fl.y*cos(g_front_left_theta) + g_front_left_y;
			index_i = round(angle_zero_to_2pi(atan2(fl_base.y,fl_base.x)*180/M_PI));
			dist = sqrt(fl_base.x*fl_base.x + fl_base.y*fl_base.y);
			if(dist < 1)
				scan_msg.ranges[index_i] = dist;


			br_base.x = br.x*cos(g_back_right_theta) + br.y*sin(g_back_right_theta) + g_back_right_x;
			br_base.y = -br.x*sin(g_back_right_theta) + br.y*cos(g_back_right_theta) + g_back_right_y;
			index_i = round(angle_zero_to_2pi(atan2(br_base.y,br_base.x)*180/M_PI));
			dist = sqrt(br_base.x*br_base.x + br_base.y*br_base.y);
			if(dist < 1)
				scan_msg.ranges[index_i] = dist;

			bl_base.x = bl.x*cos(g_back_left_theta) + bl.y*sin(g_back_left_theta) + g_back_left_x;
			bl_base.y = -bl.x*sin(g_back_left_theta) + bl.y*cos(g_back_left_theta) + g_back_left_y;
			index_i = round(angle_zero_to_2pi(atan2(bl_base.y,bl_base.x)*180/M_PI));
			dist = sqrt(bl_base.x*bl_base.x + bl_base.y*bl_base.y);
			if(dist < 1)
				scan_msg.ranges[index_i] = dist;

			angle += (g_angle_increment);
		}
		scan_msg.header.stamp = ros::Time::now();
		pub_laserscan.publish(scan_msg);
		is_data_received = false;
		r.sleep();
		ros::spinOnce();
	}
	return 0;
}
