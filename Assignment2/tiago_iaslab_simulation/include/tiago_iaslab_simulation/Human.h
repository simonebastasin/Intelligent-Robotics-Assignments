#ifndef TIAGO_IASLAB_SIMULATION_HUMAN_H
#define TIAGO_IASLAB_SIMULATION_HUMAN_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <tiago_iaslab_simulation/Objs.h>
#include <algorithm>
#include <vector>

class Human{
    public:
        Human(std::shared_ptr<ros::NodeHandle> nh_ptr);
        ~Human() {}

    private:
        const int MAX_ = 3;
        const int MIN_ = 1;

        std::shared_ptr<ros::NodeHandle> nh_ptr_;
        ros::ServiceServer objects_server_;

        void start();
        bool objService(tiago_iaslab_simulation::Objs::Request &req, tiago_iaslab_simulation::Objs::Response &res);
        int randomNumber();
};

#endif
