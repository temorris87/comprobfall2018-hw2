//
// Created by Eric Fiore on 10/26/2018.
//

#ifndef ROBOTICS_02_PROBROADMAP_H
#define ROBOTICS_02_PROBROADMAP_H

#include "SpecialEuclideanThreeSpace.h"
#include <vector>
#include <iostream>
#include "Graph.h"
#include "include/PQP.h"

class ProbRoadMap: Graph{

    std::vector<Configuration*> * worldConfiguration;      //holds all the nodes of the roadmap
    const double distance = 0.03;
    const int steps_for_interplolation = 5;

private:
    bool first_node = true;
    int totalNodesToCreate;
    Configuration * get_random_config();     //gets a random configuration
    bool check_collision(Configuration * configuration_one, PQP_Model * vehicle_model, PQP_Model * room_model);   //checks if a configuration is collision free
    node * create_world_node(Configuration * config_to_add);              //creates a random configuration then checks if it is
                                            // collision free if it is collision free it adds node
                                            //to configuration vector
    node * get_closest_neighbors(Configuration * incoming_configuration);  //creates k closest neighbors
    bool connect_config_to_neighbors(Configuration * main_configuration, Configuration * neighbor_configuration);   //checks if a connection if possible
                                                                                                                    //between a configuration and a neighbor
                                                                                                                    //if a connection is possible the connection
                                                                                                                    //is made
public:
    PQP_Model * piano;
    PQP_Model * room;
    ProbRoadMap(int numNodes_to_create, int nearest_neighbors, double min_x, double max_x, double min_y,
                double max_y, double min_z, double max_z);      //constructor to take set up how many nodes to
                                                                // create and how many neighbors each node will have
    void create_road_map(std::string * vehicle, std::string * room);
    void print_road_map();                                           //prints out all nodes and connections
};

#endif //ROBOTICS_02_PROBROADMAP_H
