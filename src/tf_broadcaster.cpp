#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string node_name = "tf_broadcaster";
std::string turtle_name;

std::string child_frame = "uav";
std::string parent_frame = "vicon_local";
std::string pose_topic = "/vicon_xb_node0/mocap/pose";

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& _pose){
  static tf::TransformBroadcaster br;
  ros::Time myTime = _pose->header.stamp;

  tf::Transform transform;
  transform.setOrigin( tf::Vector3(_pose->pose.position.x, _pose->pose.position.y, _pose->pose.position.z));
  tf::Quaternion q(_pose->pose.orientation.x, _pose->pose.orientation.y, _pose->pose.orientation.z,_pose->pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
}

int main(int argc, char** argv){
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");

  nh_param.param<std::string>("pose_topic", pose_topic, pose_topic);
  nh_param.param<std::string>("parent_frame", parent_frame, parent_frame);
  nh_param.param<std::string>("child_frame", child_frame, child_frame);

  ros::Subscriber sub = nh.subscribe(pose_topic, 10, &poseCallback);

  ros::spin();
  return 0;
};