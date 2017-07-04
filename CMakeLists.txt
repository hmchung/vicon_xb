cmake_minimum_required(VERSION 2.8.3)
project(vicon_xb)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	geometry_msgs 
	message_generation 
	serial
	# cv_bridge
	tf 
)

 add_message_files(
   FILES
   viconPoseMsg.msg
   PoseWithTwistStamped.msg
 )

 add_service_files(
   FILES
   viconXbSrv.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(vicon_xb_node src/vicon_xb.cpp)
# add_executable(vicon_native_pub src/vicon_native_pub.cpp)
add_executable(tf_broadcaster src/tf_broadcaster.cpp)


add_dependencies(vicon_xb_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(vicon_xb_node  ${catkin_LIBRARIES})
# target_link_libraries(vicon_native_pub  ${catkin_LIBRARIES} ${OpenCV_LIB})
target_link_libraries(tf_broadcaster  ${catkin_LIBRARIES})