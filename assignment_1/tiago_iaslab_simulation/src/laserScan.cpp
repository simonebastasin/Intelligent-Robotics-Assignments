#include "ros/ros.h"
#include "ros/topic.h"
#include "sensor_msgs/LaserScan.h"
#include "tiago_iaslab_simulation/ProxyDetection.h"
#include <boost/shared_ptr.hpp>
#include <vector>

using namespace std;


// given a vector of ranges and a threshold eps the function finds the starting and ending indices for each depression
pair<vector<int>, vector<int>> find_depressions(const vector<float>& arr, float eps) {
	// initialize the lists to store the start and end indices of depressions in the given arr
	vector<int> depressions_start, depressions_end;
	for (int i = 1; i < arr.size() - 1 ; i++) {
		// if current element is smaller than its previous neighbor by eps value it's a depression starting point
		if (arr[i] < arr[i-1] - eps) {
			depressions_start.push_back(i);
		}
		// if current element is smaller than its next neighbor by eps value it's a depression ending point
		else if (arr[i] < arr[i+1] - eps) {
			// if no previous starting point (eg laser cuts in half an object, depression starting point is first element in ranges)
			if (depressions_start.empty()) {
				depressions_start.push_back(0);
			}
			depressions_end.push_back(i);
		}
	}
	// if not end point for last depression it's last index in ranges ((eg laser cuts in half an object)
	if (depressions_start.size() != depressions_end.size()) {
		depressions_end.push_back(arr.size() - 1);
	}
	return make_pair(depressions_start, depressions_end);
}

// to find argmin of the array (depression) passed
int find_argmin(const vector<float>& arr, int start, int end) {
	int argmin = start;
	for (int i = start; i < end + 1; i++) {
		if (arr[i] < arr[argmin]) {
			argmin = i;
		}
	}
	return argmin;
}

pair<float, float> convert_to_cartesian_coords(const pair<float, int>& center, float angle_min, float angle_increment) {
	float x = cos(angle_min + center.second * angle_increment) * center.first;
	float y = sin(angle_min + center.second * angle_increment) * center.first;
	return make_pair(x, y);
}

pair<float, float> convert_ref_frame(const pair<float, float>& rf, float angle, float dx, float dy) {
	float T[4][4] = {{cos(angle), - sin(angle), 0, dx},
						{sin(angle), cos(angle), 0, dy},
						{0, 0, 1, 0},
						{0, 0, 0, 1}};

	vector<float> crrf = {rf.first, rf.second, 0, 1};
	vector<float> result(4);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result[i] += T[i][j] * crrf[j];
		}
	}
	return make_pair(result[0], result[1]); // z=0, last_elem=1
}


bool detection(tiago_iaslab_simulation::ProxyDetection::Request &req, tiago_iaslab_simulation::ProxyDetection::Response &res) {
	ROS_INFO("----------");
	
	// creating a nodehandler to subscribe to scan topic and retrieve ranges values from laser scan ONLY ONCE
	ros::NodeHandle n;
	boost::shared_ptr<sensor_msgs::LaserScan const> laser_msg_sharePtr;
	sensor_msgs::LaserScan laser_msg;
	ROS_INFO("Request received: collecting laser scan data");

	laser_msg_sharePtr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", n);
	if (laser_msg_sharePtr != NULL) {
		laser_msg = *laser_msg_sharePtr;
	}
	vector<float> ranges;
	float angle_inc = laser_msg.angle_increment;
	float angle_min = laser_msg.angle_min;
	float angle_max = laser_msg.angle_max;

	// discarding first 20 and last 20 values as the robot is probably detecting him self (small range values)
	int toDiscard = 20;
	for(int i = toDiscard; i < laser_msg.ranges.size() - toDiscard; i++) {
		ranges.push_back(laser_msg.ranges[i]);
	}
	// now ranges global variable should have been filled in through messageCallback
	ROS_INFO("Laser scan data obtained, detecting obstacle positions");

	// find start and end points of depressions (i.e., obastcles)
	float eps = 0.8; // parameter to detect depressions
	pair<vector<int>, vector<int>> depressions = find_depressions(ranges, eps);
	vector<int> depressions_start = depressions.first;
	vector<int> depressions_end = depressions.second;
	int n_obstacles = depressions_start.size(); // number of obstacles found

	// for each depression find its argmin (i.e., closest point of the obastacle to the robot) and save it
	vector<int> argmins;
	for (int i = 0; i < n_obstacles; i++) {
		int argmin = find_argmin(ranges, depressions_start[i], depressions_end[i]);
		argmins.push_back(argmin);
	}

	// now, while still in polar coordinates we extend ro value by some delta to approximate object center
	vector<pair<float, int>> centers; // vector of pairs (ro, argmin)
	float delta = 0.445; // parameter to detect centers
	for (int i = 0; i < n_obstacles; i++) {
		float shifted_ro = ranges[argmins[i]] + delta;
		int argmin_shifted_back = argmins[i] + toDiscard; // shifting back argmin to consider first deleted range values
		centers.push_back(make_pair(shifted_ro, argmin_shifted_back));
	}

	// convert centers from polar to cartesian coordinates
	vector<pair<float, float>> centers_cart_coords;
	for (int i = 0; i < n_obstacles; i++) {
		centers_cart_coords.push_back(convert_to_cartesian_coords(centers[i], angle_min, angle_inc));
	}

	// constant values as starting coordinates of robot in world reference frame (found from Gazebo map)
	float dx = - 6.579991 + req.x;
	float dy = 1.369981 + req.y;
	// now centers are wrt to robot reference frame, I could apply a transformation matrix to map coordinates to World Reference frame
	// transformation matrix from robot rf to world rf
	vector<pair<float, float>> centers_world_rf;
	for (int i = 0; i < n_obstacles; i++) {
		centers_world_rf.push_back(convert_ref_frame(centers_cart_coords[i], req.yaw_radians, dx, dy));
	}

	// filling in the service response
	res.x_positions.clear(); res.x_positions_wrf.clear();
	res.y_positions.clear(); res.y_positions_wrf.clear();
	for (int i = 0; i < n_obstacles; i++) {
		// robot reference frame
		res.x_positions.push_back(centers_cart_coords[i].first);
		res.y_positions.push_back(centers_cart_coords[i].second);
        // world reference frame
		res.x_positions_wrf.push_back(centers_world_rf[i].first);
		res.y_positions_wrf.push_back(centers_world_rf[i].second);
	}

    ROS_INFO("Detection done: sending back response to proxy");
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "laserScan");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("laserScan_topic", detection);
	ROS_INFO("Ready to receive obastacles detection requests");
	ros::spin();
    return 0;
}