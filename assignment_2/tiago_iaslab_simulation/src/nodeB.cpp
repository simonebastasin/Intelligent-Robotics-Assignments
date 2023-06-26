/*
nodeB implements a service node that performs the detection of the obstacles (only the one with AprilTag IDs) in the current camera frame.
The poses of the tags are transformed and returned with respect to the base_link reference frame.
*/

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
// as service file 
#include "tiago_iaslab_simulation/ObjDetection.h"
// As subscriber to get apriltag detections
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/PoseArray.h"
#include <vector>
#include <string>

using namespace std;


// __________ FUNCTIONS __________


bool detection(tiago_iaslab_simulation::ObjDetection::Request &req, tiago_iaslab_simulation::ObjDetection::Response &res) {
	ROS_INFO("\n----------");
	ROS_INFO("Request received: collecting tag_detections data");
	
	// create a nodehandler to subscribe to tag_detections topic and retrieve tags positions (ONLY ONCE)
	ros::NodeHandle n;
	boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> detection_sharePtr;
	apriltag_ros::AprilTagDetectionArray detection_msg;

	detection_sharePtr = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", n);
	if (detection_sharePtr != NULL) {
		detection_msg = *detection_sharePtr;
	}
	ROS_INFO("AprilTag data obtained");
	int n_tags = detection_msg.detections.size();	

	// getting transformation matrix values to move from camera rf to base_link rf
	tf::TransformListener listener;
	tf::StampedTransform transform;
	ros::Duration(3.0).sleep();
	string camera_rf = detection_msg.detections[0].pose.header.frame_id; 
	const char *cstr = camera_rf.c_str();
	ROS_INFO(cstr);
	listener.lookupTransform("base_footprint", cstr, ros::Time(0), transform);
	ros::Duration(1.0).sleep();
	ROS_INFO("Tranform matrix obtained");
	
	res.pose_array.header.frame_id = "base_footprint";

	// for each detected obstacle
    for(int i = 0; i < n_tags; i++) {
        tf::Pose temp_pose;
        geometry_msgs::Pose converted_pose;
		int id = detection_msg.detections[i].id[0];
		ROS_INFO("ID object detected: %d", id);

        // convert pose: geometry_msgs -> tf
        tf::poseMsgToTF(detection_msg.detections[i].pose.pose.pose, temp_pose);
        // transform pose: to base_link rf
        temp_pose = transform * temp_pose;
        // transform pose: back to geometry_msgs
        tf::poseTFToMsg(temp_pose, converted_pose);
        // push every pose in response vector
        res.pose_array.poses.push_back(converted_pose);
		res.pose_ids.push_back(id);
    }
    ROS_INFO("Objects Detection done: sending back response");
    return true;
}


// __________ MAIN __________


int main(int argc, char** argv) {
    ros::init(argc, argv, "nodeB");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/nodeB_service", detection);
    ROS_INFO("Ready to detect objects");
    ros::spin();
    return 0;
}