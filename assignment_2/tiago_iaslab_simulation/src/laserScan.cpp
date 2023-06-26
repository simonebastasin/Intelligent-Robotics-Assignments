#include "ros/ros.h"
#include "ros/topic.h"
#include "sensor_msgs/LaserScan.h"
#include "tiago_iaslab_simulation/ProxyDetection.h"
#include <boost/shared_ptr.hpp>
#include <vector>
#include <random>
#include <complex>

using namespace std;


// given a vector of ranges and a threshold eps the function finds the starting and ending indices for each depression
pair<vector<int>, vector<int>> find_depressions(const vector<float>& arr, float eps) {
	// initialize the lists to store the start and end indices of depressions in the given arr
	vector<int> depressions_start, depressions_end;
	bool start_found = false;
	for (int i = 1; i < arr.size() - 1 ; i++) {
		// if current element is smaller than its previous neighbor by eps value it's a depression starting point
		if (arr[i] < arr[i-1] - eps) {
			depressions_start.push_back(i);
			start_found = true;
		}
		// if current element is smaller than its next neighbor by eps value it's a depression ending point
		else if (start_found && arr[i] < arr[i+1] - eps) {
			// if no previous starting point (eg laser cuts in half an object, depression starting point is first element in ranges)
			// if (depressions_start.empty()) depressions_start.push_back(0);
			depressions_end.push_back(i);
			start_found = false;
		}
	}
	/*int j = 0;
	while (j < depressions_end.size()) {
		if (depressions_end[j] < depressions_start[j]) {
  			depressions_end.erase(depressions_end.begin() + j - 1); // erase the j-th element
		}
		else {
			j++;
		}
	}*/
	// if not end point for last depression it's last index in ranges ((eg laser cuts in half an object)
	if (depressions_start.size() != depressions_end.size()) {
		depressions_end.push_back(arr.size() - 1);
	}
	for(int i = 0; i < depressions_end.size(); i++) {
		ROS_INFO("depr = [%d, %d]", depressions_start[i], depressions_end[i]);
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
	float T[4][4] = {{cos(angle), -sin(angle), 0, dx},
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

bool detection2(tiago_iaslab_simulation::ProxyDetection::Request &req, tiago_iaslab_simulation::ProxyDetection::Response &res) {
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
	// discard first 160 and last 160 values, they are useless considering robot pose (corner of room)
	int toDiscard = 160;
	vector<float> ranges;
	float angle_inc = laser_msg.angle_increment;
	float angle_min = laser_msg.angle_min + toDiscard * angle_inc;
	float angle_max = laser_msg.angle_max + toDiscard * angle_inc;;

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

	vector<vector<vector<float>>> values;
	vector<float> center_angles;
	for(int i=n_obstacles-1; i>=0; i--) {
		vector<vector<float>> temp;
		for(int j=depressions_start[i]; j<depressions_end[i]; j++) {
			float angle = angle_min + angle_inc*j;
			float r = ranges[j];
			float xtemp = r*cos(angle);
			float ytemp = r*sin(angle);
			temp.push_back(vector<float>{xtemp,ytemp});
			if(j-depressions_start[i] == (depressions_end[i]-depressions_start[i])/2){
				center_angles.push_back(angle/M_PI*180.0);
			}
		}
		values.push_back(temp);
	}

	int iter = 100;
	vector<pair<float,float>> centers_cart_coords;
	for(int i=0; i<values.size(); i++){
		float xsum = 0;
		float ysum = 0;
		int vsize = values[i].size()/3;
		for(int j=0; j<iter; j++){
			vector<int> rnd_i;
			for(int k=0; k<3; k++) rnd_i.push_back(rand()%vsize + k*vsize);
			complex<float> z1(values[i][rnd_i[0]][0], values[i][rnd_i[0]][1]);
			complex<float> z2(values[i][rnd_i[1]][0], values[i][rnd_i[1]][1]);
			complex<float> z3(values[i][rnd_i[2]][0], values[i][rnd_i[2]][1]);
			complex<float> w = (z3-z1)/(z2-z1);
			complex<float> c = (z1-z2)*(w-pow(abs(w),complex<float>(2,0)))/complex<float>(0,2)/imag(w)-z1;
			xsum += (-real(c));
			ysum += (-imag(c));
		}
		centers_cart_coords.push_back(make_pair(xsum/iter, ysum/iter));
	}

	for(int i=0; i<centers_cart_coords.size(); i++){
		ROS_INFO("coords of %d:   x: %f, y: %f.", i,centers_cart_coords[i].first, centers_cart_coords[i].second);
	}

	// constant values as starting coordinates of robot in world reference frame (found from Gazebo map)
	float dx = + req.x;
	float dy = + req.y;
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
		res.angle_positions.push_back(center_angles[i]);
        // world reference frame
		res.x_positions_wrf.push_back(centers_world_rf[i].first);
		res.y_positions_wrf.push_back(centers_world_rf[i].second);
	}

    ROS_INFO("Detection done: sending back response to proxy");
    return true;
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
	// discard first 160 and last 160 values, they are useless considering robot pose (corner of room)
	int toDiscard = 160;
	vector<float> ranges;
	float angle_inc = laser_msg.angle_increment;
	float angle_min = laser_msg.angle_min + toDiscard * angle_inc;
	float angle_max = laser_msg.angle_max + toDiscard * angle_inc;;

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
		int min_offset = (depressions_end[i] - depressions_start[i]) / 4;
		int argmin = find_argmin(ranges, depressions_start[i] + min_offset, depressions_end[i] - min_offset);
		argmins.push_back(argmin);
	}

	// now, while still in polar coordinates we extend ro value by some delta to approximate object center
	vector<pair<float, int>> centers; // vector of pairs (ro, argmin)
	vector<float> center_angles; // angles in degrees
	float delta = 0.21; // parameter to detect centers
	for (int i = 0; i < n_obstacles; i++) {
		float shifted_ro = ranges[argmins[i]] + delta;
		centers.push_back(make_pair(shifted_ro, argmins[i])); // ro, angle_index
		float center_angle_rrf = (angle_min + argmins[i] * angle_inc) * 180.0 / M_PI;
		center_angles.push_back(center_angle_rrf);
		ROS_INFO("angle: %f", center_angle_rrf);
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
		res.angle_positions.push_back(center_angles[i]);
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
	//ros::ServiceServer service = n.advertiseService("laserScan_topic", detection);
	ros::ServiceServer service = n.advertiseService("laserScan_topic", detection2);
	ROS_INFO("Ready to receive obastacles detection requests");
	ros::spin();
    return 0;
}