#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <string>

using namespace std;

/*
	We approached the exercise by implementing the following node as "subscriber" 
	to the topic named "scan";
	The exercise then asks to compute the position of each person as the point 
	in the middle of the 2 legs.
	We approached this task by implementing k-means clustering algorithm as usual.
	Knowing that kmeans works with k-fixed a priori to generalize to a variable number of people we
	computed the number of legs (from which we can compute the number of people dividing by 2) in a clever way 
	when scanning the ranges vector (see below).
	We assume each person has 2 legs and that legs coordinates in the ranges vector are separated by at least 1 'inf' value. 
	The algorithm thus works with a variable number of people and also if a leg starts at the end of ranges and continues at the beginning.
*/

// Euclidean distance between 2 points
float dist(vector<float> p1, vector<float> p2) {
	return sqrt(pow((p1[0] - p2[0]), 2) + pow((p1[1] - p2[1]), 2));
}

// k-means loss function
float loss(vector<vector<float>> coord, vector<int> label, vector<vector<float>> centroids) {
	float sum = 0;
	for(int i = 0; i < coord.size(); i++) {
		sum += dist(coord[i], centroids[label[i]]);
	}
	return sum;
}

// k-means function
vector<vector<float>> kmeans(vector<vector<float>> coord, int k, vector<vector<float>> centers) {
	vector<int> label(coord.size());
	int iter = 0;
	float prev_loss = INFINITY;

	// each coordinate label initialized based on the closest center
	for(int i = 0; i < coord.size(); i++) {
		int min_index = min_element(centers.begin(), centers.end(),
			[coord, i](vector<float> p1, vector<float> p2) {
				return dist(coord[i], p1) < dist(coord[i], p2);
			}) - centers.begin();
		label[i] = min_index;
	}

	do {
		vector<vector<float>> centroids(k, vector<float>(2, 0.0));
		vector<int> dim(k, 0);
		for(int i = 0; i < coord.size(); i++){
			centroids[label[i]][0] += coord[i][0];
			centroids[label[i]][1] += coord[i][1];
			dim[label[i]]++;
		}
		for(int i = 0; i < centroids.size(); i++) {
			centroids[i][0] /= dim[i];
			centroids[i][1] /= dim[i];
		}
		for(int i = 0; i < coord.size(); i++) {
			int min_index = min_element(centroids.begin(), centroids.end(),
				[coord,i](vector<float> p1, vector<float> p2) {
					return dist(coord[i], p1) < dist(coord[i], p2);
				}) - centroids.begin();
			label[i] = min_index;
		}
		centers = centroids;
		iter++;
	} while(loss(coord, label, centers) - prev_loss > 1e-2 && iter < 100);

	return centers;
}

// void messageCallback(const exercise5::scan_msg::ConstPtr& msg){
void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	float angle_inc = msg->angle_increment;
	float angle_min = msg->angle_min;
	float angle_max = msg->angle_max;
	vector<vector<float>> values;
	vector<vector<float>> starting_centers;
	// to compute k for kmeans in an automatic way (see comment at beginning)
	bool new_leg = true;
	int n_legs = 0;

	for(int i = 0; i < msg->ranges.size(); i++) {
		float r = msg->ranges[i]; // ranges size = 720
		if(r < INFINITY) {
			float x = cos(i * angle_inc) * r;
			float y = sin(i * angle_inc) * r;
			vector<float> coord_values{x, y};
			values.push_back(coord_values);
			if(new_leg) {
				n_legs++;
				// centers initialization for kmeans
				// adding a center every 2 new legs found as the coordinates of the first point of that leg
				if(n_legs % 2 == 0) {
					starting_centers.push_back(coord_values);
				}
				new_leg = false;
			}
		}
		else new_leg = true;
	}

	vector<vector<float>> centers = kmeans(values, n_legs/2, starting_centers);

	string output = "People centers as [x, y]: ";
	for(int i = 0; i < centers.size(); i++) {
		output += "[" + to_string(centers[i][0]) + ", " + to_string(centers[i][1]) + "] ";
	}
	ROS_INFO("%s", output.c_str());
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "laserScan");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan", 1000, messageCallback);
	ros::spin();
	return 0;
}