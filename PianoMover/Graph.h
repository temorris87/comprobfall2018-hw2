//
// Created by Eric Fiore on 10/30/2018.
//

#ifndef PIANOMOVER_GRAPH_H
#define PIANOMOVER_GRAPH_H

#include <forward_list>
#include <queue>
#include <vector>
#include "SpecialEuclideanThreeSpace.h"
#include <string>
#include "MeshReader.h"
#include <algorithm>

struct edge {
    int from_node;
    int to_node;
    double weight;
    std::vector<Configuration*> * interpolations = new std::vector<Configuration*>;
};

struct node {
    int id;
    Configuration * node_configuration;
    std::forward_list<edge*> * edges = new std::forward_list<edge*>;

    // For use with searching algorithms.
    struct node * parent;
    double cost;
    bool in_fringe;
    bool in_closed;
};

struct min_comparator {
    bool operator() (const node * n1, const node * n2) const {
        return n1->cost > n2->cost;
    }
};



class Graph {
public:
    int total_nodes;
    int k_nodes;
    std::vector<node*> * nodes = new std::vector<node*>;
    std::priority_queue<node*, std::vector<node*>, min_comparator> fringe;
    //virtual void build_graph();

private:
    const double distance = 0.2;
    const int steps_for_interplolation = 1000;
    bool first_node = true;
    int totalNodesToCreate;
    Configuration * get_random_config();     //gets a random configuration
    bool check_collision(Configuration * configuration_one, PQP_Model * vehicle_model, PQP_Model * room_model);   //checks if a configuration is collision free
    node * create_world_node(Configuration * config_to_add, int center_id);              //creates a random configuration then checks if it is
                                                                            // collision free if it is collision free it adds node
                                                                            //to configuration vector
    void connect_closest_neighbor(node * connecting_node, int center_id);  //creates k closest neighbors
    bool connect_config_to_neighbors(Configuration * main_configuration, Configuration * neighbor_configuration, std::vector<Configuration*> * interpolation_configs);   //checks if a connection if possible
                                                                            //between a configuration and a neighbor
                                                                            //if a connection is possible the connection
                                                                            //is made
    void add_node_to_fringe(node * n);
    void empty_fringe();
    node * get_node_from_fringe();
    bool fringe_has_nodes();
    std::forward_list<int> * build_path(node * start_node, node * end_node);
    void update_vertex(node * s, node * t, node * goal, edge * e, bool free_direction);
    void remove_all_nodes_from_closed();
    double heuristic_cost(double s_id, double t_id);
    //virtual bool line_of_sight(node * s, node * t);
    std::forward_list<int> * astar(int start_node, int end_node, bool free_direction);
    SpecialEuclideanThreeSpace * configuration_solver;

public:
    PQP_Model * piano;
    PQP_Model * room;
    Graph(int numNodes_to_create, int nearest_neighbors, double min_x, double max_x, double min_y,
                double max_y, double min_z, double max_z);      //constructor to take set up how many nodes to
    // create and how many neighbors each node will have
    void create_kroad_map(std::string * vehicle, std::string * room);
    void print_road_map();                                           //prints out all nodes and connections
    void add_edge(int n, int to_node, double weight, std::vector<Configuration*> * added_edgeInterpolation);
    void remove_edge(int n, int to_node);
    std::forward_list<int> * astar(int start_node, int end_node);
    std::forward_list<int> * fd_astar(int start_node, int end_node);
    void set_configuration(int s_id, Configuration * s_configuration);
    void print_graph();
    void print_graph_with_weights();
    node * getNodes(int id);
    void setNodes(node * addedNode);
    int get_total_nodes();
    int set_total_nodes();
};


#endif //PIANOMOVER_GRAPH_H
