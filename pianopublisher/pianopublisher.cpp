#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <string.h>
#include <stdio.h>  
#include <std_msgs/Float64.h>
#include <math.h>
#include "../PianoMover/Graph.h"
#include "../PianoMover/SpecialEuclideanThreeSpace.h"
#include <unistd.h>
#include <chrono>

void update_model_state(ros::ServiceClient set_model_state_client, Configuration * c);

int main(int argc, char **argv) {
    ros::init(argc, argv, "init_model_state");
    ros::NodeHandle nh;
    ros::Duration half_sec(0.5);
    
    // make sure service is available before attempting to proceed, else node will crash
    bool service_ready = false;
    while (!service_ready) {
      service_ready = ros::service::exists("/gazebo/set_model_state",true);
      ROS_INFO("waiting for set_model_state service");
      half_sec.sleep();
    }
    ROS_INFO("set_model_state service exists");

    ros::ServiceClient set_model_state_client = 
       nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    std::string * vehicle = new std::string("/ilab/users/tem97/catkin_ws/src/comprobfall2018-hw2/PianoMover/piano");
    std::string * room = new std::string("/ilab/users/tem97/catkin_ws/src/comprobfall2018-hw2/PianoMover/room");

    Graph * new_roadmap = new Graph(100, 5, 0, 10, 0, 10, 0, 2);

    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    new_roadmap->create_kroad_map(vehicle, room);
    std::forward_list<int> * path = new_roadmap->astar(0, 50);
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

    if (path == nullptr)
        std::cout << "No path." << std::endl;
    else
        std::cout << "Path found." << std::endl;

    std::vector<node*> * nodes = new std::vector<node*>();
    for (int & config : *path) {
        nodes->push_back(new_roadmap->getNodes(config));
    }

    int i = 1;
    Configuration * c1;
    Configuration * c2;
    unsigned int microseconds;
    do {
        c1 = nodes->at(i-1)->node_configuration;
        c2 = nodes->at(i)->node_configuration;

        new_roadmap->configuration_solver->setup_interpolation(c1, c2, new_roadmap->steps_for_interplolation);
        Configuration * current = new_roadmap->configuration_solver->get_next_interpolation_configuration();
        while (current != nullptr) {
            update_model_state(set_model_state_client, current);
            current = new_roadmap->configuration_solver->get_next_interpolation_configuration();
            usleep(16666);
        }
        update_model_state(set_model_state_client, c2);
        i++;
    } while (i < nodes->size());

    update_model_state(set_model_state_client, new_roadmap->getNodes(0)->node_configuration);

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    std::cout << "Total time: " << duration << std::endl;
}

void update_model_state(ros::ServiceClient set_model_state_client, Configuration * c) {
    gazebo_msgs::SetModelState model_state_srv_msg;
    
    // Set no pose and twist for piano.
    model_state_srv_msg.request.model_state.model_name = "piano2";
    model_state_srv_msg.request.model_state.pose.position.x = c->position->x;
    model_state_srv_msg.request.model_state.pose.position.y = c->position->y;
    model_state_srv_msg.request.model_state.pose.position.z = c->position->z;
    
    model_state_srv_msg.request.model_state.pose.orientation.x = c->rotation->x;
    model_state_srv_msg.request.model_state.pose.orientation.y = c->rotation->y;
    model_state_srv_msg.request.model_state.pose.orientation.z = c->rotation->z;
    model_state_srv_msg.request.model_state.pose.orientation.w = c->rotation->w;
    
    
    model_state_srv_msg.request.model_state.twist.linear.x= 0.0; //2cm/sec
    model_state_srv_msg.request.model_state.twist.linear.y= 0.0;
    model_state_srv_msg.request.model_state.twist.linear.z= 0.0;
    
    model_state_srv_msg.request.model_state.twist.angular.x= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.y= 0.0;
    model_state_srv_msg.request.model_state.twist.angular.z= 0.0;
        
    model_state_srv_msg.request.model_state.reference_frame = "world";

    set_model_state_client.call(model_state_srv_msg);
        //make sure service call was successful
        bool result = model_state_srv_msg.response.success;
        if (!result)
            ROS_WARN("service call to set_model_state failed!");
        else
            ROS_INFO("Done");
   
}
