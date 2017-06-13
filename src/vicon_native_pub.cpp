#include <ros/ros.h>
#include <vicon_xb/viconPoseMsg.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

std::string node_name = "vicon_native_pub";
std::string topic_sub_vicon_info = "/vicon_xb/viconPoseTopic";

std::string topic_pub_pose_local = "/vicon/pose_local";
std::string topic_pub_vel_local = "/vicon/twist_local";
std::string topic_pub_vel_body = "/vicon/twist_body";
std::string topic_pub_imu = "/vicon/imu";

geometry_msgs::Pose curr_vicon_pose_local;
geometry_msgs::Vector3 curr_vicon_vel_local;
geometry_msgs::Vector3 curr_vicon_vel_body;
sensor_msgs::Imu curr_imu;

geometry_msgs::PoseStamped pose_local_msg;
geometry_msgs::TwistStamped vel_local_msg;
geometry_msgs::TwistStamped vel_body_msg;
sensor_msgs::Imu imu_msg;

ros::Time last_vicon_time;
double vicon_timeout = 1.5; //sec

cv::Mat zeros6x6 = Mat::zeros(6, 6, CV_64FC1);
cv::Mat zeros6x1 = Mat::zeros(6, 1, CV_64FC1);
cv::Mat ones6x6 = Mat::ones(6, 6, CV_64FC1);
cv::Mat ones6x1 = Mat::ones(6, 1, CV_64FC1);
cv::Mat ones3x6 = Mat::ones(3, 6, CV_64FC1);
cv::Mat ones1x6 = Mat::ones(1, 6, CV_64FC1);

cv::Mat pose_curr(6, 1, CV_64FC1), 		pose_last(6, 1, CV_64FC1);
cv::Mat vel_curr(6, 1, CV_64FC1), 		vel_last(6, 1, CV_64FC1);
cv::Mat acc_curr(6, 1, CV_64FC1), 		acc_last(6, 1, CV_64FC1);

//Util functions
bool is_vicon_timeout();

//Logic functions
void prepare_body_frame_estimates();

//Callback functions;
void vicon_info_cb(const vicon_xb::viconPoseMsg::ConstPtr vicon_info);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	ros::Subscriber sub_vicon_info = nh.subscribe<vicon_xb::viconPoseMsg>(topic_sub_vicon_info, 10, vicon_info_cb);

	ros::Publisher pub_pose_local = nh.advertise<geometry_msgs::PoseStamped>(topic_pub_pose_local, 10);
	ros::Publisher pub_vel_local = nh.advertise<geometry_msgs::TwistStamped>(topic_pub_vel_local, 10);
	ros::Publisher pub_vel_body = nh.advertise<geometry_msgs::TwistStamped>(topic_pub_vel_body, 10);
	ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>(topic_pub_imu, 10);

	ros::Rate loop_rate(10);

	while(ros::ok()){
		if (is_vicon_timeout() == false){
			prepare_body_frame_estimates();
			
			pose_local_msg.pose = curr_vicon_pose_local;
			pose_local_msg.header.stamp = last_vicon_time;

			vel_local_msg.twist.linear = curr_vicon_vel_local;
			vel_local_msg.header.stamp = last_vicon_time;

			vel_body_msg.twist.linear = curr_vicon_vel_body;
			vel_body_msg.header.stamp = last_vicon_time;

			imu_msg = curr_imu;
			curr_imu.header.stamp = last_vicon_time;

			pub_pose_local.publish(pose_local_msg);
			pub_vel_local.publish(vel_local_msg);
			pub_vel_body.publish(vel_body_msg);
			pub_imu.publish(imu_msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void vicon_info_cb(const vicon_xb::viconPoseMsg::ConstPtr vicon_info){
	last_vicon_time = ros::Time::now();

	curr_vicon_pose_local = vicon_info->pose;
	curr_vicon_vel_local = vicon_info->vel;
	curr_imu.orientation = curr_vicon_pose_local.orientation;

	//Updating pose matrix - map frame
	double x, y, z, w;
	double roll, pitch, yaw;

	x = curr_vicon_pose_local.orientation.x;
	y = curr_vicon_pose_local.orientation.y;
	z = curr_vicon_pose_local.orientation.z;
	w = curr_vicon_pose_local.orientation.w;

	tf::Quaternion q(x, y, z, w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	pose_curr.at<double>(0, 0) = curr_vicon_pose_local.position.x;
	pose_curr.at<double>(1, 0) = curr_vicon_pose_local.position.y;
	pose_curr.at<double>(2, 0) = curr_vicon_pose_local.position.z;
	pose_curr.at<double>(3, 0) = roll;
	pose_curr.at<double>(4, 0) = pitch;
	pose_curr.at<double>(5, 0) = yaw;

	//Updating vel matrix - map frame
	vel_curr.at<double>(0, 0) = curr_vicon_vel_local.x;
	vel_curr.at<double>(1, 0) = curr_vicon_vel_local.y;
	vel_curr.at<double>(2, 0) = curr_vicon_vel_local.z;
	vel_curr.at<double>(3, 0) = 0.0;
	vel_curr.at<double>(4, 0) = 0.0;
	vel_curr.at<double>(5, 0) = 0.0;
}

bool is_vicon_timeout(){
	if (ros::Time::now().toSec() - last_vicon_time.toSec() >= vicon_timeout) return true;
	else return false;
}

void prepare_body_frame_estimates(){
	cv::Mat temp_corrected_vel(6, 1, CV_64FC1);
	geometry_msgs::Twist velocity_msg;
	//Form rotation correction matrix
	double z_rotation = pose_curr.at<double>(5, 0); //Map frame to copter frame
	// z_rotation = -z_rotation; 						//Copter frame to map frame

	cv::Mat rotation_correction = Mat::zeros(6, 6, CV_64FC1);
	rotation_correction.at<double>(0, 0) = cos(z_rotation);
	rotation_correction.at<double>(0, 1) = sin(z_rotation);
	rotation_correction.at<double>(1, 0) = -sin(z_rotation);
	rotation_correction.at<double>(1, 1) = cos(z_rotation);
	rotation_correction.at<double>(2, 2) = 1.0;
	rotation_correction.at<double>(3, 3) = 1.0;
	rotation_correction.at<double>(4, 4) = 1.0;
	rotation_correction.at<double>(5, 5) = 1.0;

	//Correct vel command from copter frame to map frame
	gemm(rotation_correction, vel_curr, 1, zeros6x1, 0, temp_corrected_vel);

	//Write into message
	curr_vicon_vel_body.x = temp_corrected_vel.at<double>(0, 0);
	curr_vicon_vel_body.y = temp_corrected_vel.at<double>(1, 0);
	curr_vicon_vel_body.z = temp_corrected_vel.at<double>(2, 0);
}