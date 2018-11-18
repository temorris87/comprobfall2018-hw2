//
// Created by Eric Fiore on 10/30/2018.
//

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include "Graph.h"

Graph::Graph(int numNodes, int nearest_neighbors, double min_x, double max_x,
                         double min_y, double max_y, double min_z, double max_z)  {
    //total_nodes = 0;
    total_nodes = 0;
    k_nodes = nearest_neighbors;
    this->totalNodesToCreate = numNodes;
    configuration_solver = new SpecialEuclideanThreeSpace(min_x, max_x, min_y, max_y, min_z, max_z);
}

void Graph::create_kroad_map(std::string * vehicle_string, std::string * room_string) {
    MeshReader * piano_meshes = new MeshReader(*vehicle_string);                //get mesh information for piano
    this->piano = piano_meshes->model;                                          //save piano meshes
    MeshReader * room_meshes = new MeshReader(*room_string);                    //get mesh information for room
    this->room = room_meshes->model;                                            //save room mesh

    for (int i = 0; i < totalNodesToCreate; ) {
        Configuration * center_config = get_random_config();                       //get a random configuration
        bool test_result = check_collision(center_config, this->piano, this->room);           //test if random configuration is in collision
        if (test_result) {                                                      //if random configurat ion is not in collision
            node * center_node = create_world_node(center_config, i);                 //save random configuration
            int num_neigbors = 1;
            while (num_neigbors != this->k_nodes) {
                Configuration * neighbor_config = configuration_solver->get_nearby_random_configuration(center_config);
                if (check_collision(neighbor_config, this->piano, this->room)) {
                    std::vector<Configuration*> * steps_in_between = new std::vector<Configuration*>;
                    bool neighbor_result = connect_config_to_neighbors(center_config, neighbor_config, steps_in_between);
                    std::vector<Configuration*> * reversed_vector = new std::vector<Configuration*>;
                    for (int j = steps_in_between->size()-1; j >= 0; j--){
                        reversed_vector->push_back(steps_in_between->at(j));
                    }

                    if (neighbor_result) {
                        node * neighbor_node = create_world_node(neighbor_config, i);
                        double edge_cost = configuration_solver->get_distance(center_config, neighbor_config, 0.5);
                        add_edge(center_node->id, neighbor_node->id, edge_cost, steps_in_between);
                        add_edge(neighbor_node->id, center_node->id, edge_cost, steps_in_between);
                        connect_closest_neighbor(neighbor_node, center_node->id);
                        num_neigbors++;
                    }
                }
            }
            //node * parent_node = get_closest_neighbors(test_config);            //find closest neighbor
//            if (parent_node != nullptr) {
//                //child_node->parent = parent_node;
//                int edge_cost = configuration_solver->get_distance(parent_node->node_configuration, center_node->node_configuration, 0.5);
//                Graph::add_edge(parent_node->id, center_node->id, edge_cost);
//            }
            i = i + k_nodes;
        }
    }
    //print_road_map();

}

node * Graph::create_world_node(Configuration * config_to_add, int center_id) {
    node *temp = new node;
    temp->node_configuration = config_to_add;
    temp->parent = nullptr;
    temp->cost = 0;
    //double edge_weight = configuration_solver->get_distance(config_to_add, road_map->getNodes(
    //      road_map->get_total_nodes() - 1)->node_configuration, 0.5);
    temp->in_fringe = false;
    if (first_node){
        temp->id = 0;
        first_node = false;
    }
    else
        temp->id = set_total_nodes();
    setNodes(temp);
    return temp;
}

void Graph::connect_closest_neighbor(node * connecting_node, int center_id) {
    for (int i = center_id - 1; i >= 0; i--){
        if (configuration_solver->get_distance(connecting_node->node_configuration, Graph::getNodes(i)->node_configuration, 0.5) <= distance
            && (connecting_node->node_configuration != getNodes(i)->node_configuration)) {

            std::vector<Configuration*> * steps_in_between = new std::vector<Configuration*>;
            bool interpolation_result = connect_config_to_neighbors(connecting_node->node_configuration,
                                                                    getNodes(i)->node_configuration, steps_in_between);
            if (!interpolation_result)
                continue;
            double edge_cost = configuration_solver->get_distance(connecting_node->node_configuration, getNodes(i)->node_configuration, 0.5);
            add_edge(getNodes(i)->id, connecting_node->id, edge_cost, steps_in_between);
        }
    }
}

bool Graph::connect_config_to_neighbors(Configuration * main_configuration,
                                              Configuration * neighbor_configuration, std::vector<Configuration*> * interpolation_configs) {
    PQP_REAL room_orientation[3][3];
    PQP_REAL room_position[3];
    for (int m = 0; m < 3; m++){
        room_position[m] = 0;
        for (int n = 0; n < 3; n++)
            room_orientation[m][n] = 0;
    }
    room_orientation[0][0] = 1;
    room_orientation[1][1] = 1;
    room_orientation[2][2] = 1;

    configuration_solver->setup_interpolation(main_configuration, neighbor_configuration, this->steps_for_interplolation);
    Configuration * interpolation_configuration = configuration_solver->get_next_interpolation_configuration();
    while (interpolation_configuration != nullptr){
        PQP_REAL pqp_position[3];
        PQP_REAL pqp_rotation[3][3];
        configuration_solver->get_pqp_position(interpolation_configuration, pqp_position);
        configuration_solver->get_pqp_rotation(interpolation_configuration, pqp_rotation);
        PQP_CollideResult is_in_collision;
        PQP_Collide(&is_in_collision, pqp_rotation, pqp_position, this->piano,
                    room_orientation, room_position, this->room);

        int colliding = is_in_collision.NumPairs();
        if (colliding > 0)
            return false;
        interpolation_configuration = configuration_solver->get_next_interpolation_configuration();
        if (interpolation_configuration != nullptr) {
            interpolation_configs->push_back(interpolation_configuration);
        }
    }
    return true;
}

bool Graph::check_collision(Configuration * configuration_one, PQP_Model * vehicle_model, PQP_Model * room_model){
    PQP_CollideResult is_in_collision;

    PQP_REAL  room_orientation[3][3];
    PQP_REAL  room_position[3];
    for (int i =0; i < 3; i++){
        room_position[i] = 0;
        for (int j =0; j < 3; j++){
            room_orientation[i][j] = 0;
        }
    }
    room_orientation[0][0] = 1; //rotation
    room_orientation[1][1] = 1;
    room_orientation[2][2] = 1;

    PQP_REAL pqp_position[3];
    PQP_REAL pqp_rotation[3][3];

    configuration_solver->get_pqp_position(configuration_one, pqp_position);
    configuration_solver->get_pqp_rotation(configuration_one, pqp_rotation);

    PQP_Collide(&is_in_collision, pqp_rotation, pqp_position, vehicle_model,
                room_orientation, room_position, room_model);

    int colliding = is_in_collision.NumPairs();
    if (colliding > 0)
        return false;

    return true;
}

void Graph::print_road_map() {
    for (int i = 0; i < Graph::get_total_nodes(); i++){
        std::cout << "Node " << getNodes(i)->id << ":";
        node * temp_node = getNodes(i);
        while (temp_node->parent != nullptr){
            temp_node = temp_node->parent;
            std::cout << " <-- " << temp_node->id;
        }
        std::cout << std::endl;
    }
}

Configuration * Graph::get_random_config() {
    return configuration_solver->get_random_end_configuration();
}

node * Graph::getNodes(int id){
    return nodes->at(id);
}

void Graph::setNodes(node * addedNode){
    nodes->push_back(addedNode);
}

int Graph::get_total_nodes() {
    return total_nodes;
}

int Graph::set_total_nodes() {
    return ++total_nodes;
}

void Graph::add_edge(int n, int to_node, double weight, std::vector<Configuration*> * added_edgeInterpolation) {
    edge * temp = new edge;
    temp->from_node = n;
    temp->to_node = to_node;
    temp->weight = weight;
    temp->interpolations = added_edgeInterpolation;

    nodes->at(n)->edges->push_front(temp);
}

//void Graph::remove_edge(int n, int to_node) {
//    for (auto it = nodes.at(n).edges.begin(); it < nodes.at(n).edges.end(); it++) {
//
//    }
//    nodes.at(n).edges->remove
//}

void Graph::add_node_to_fringe(node * n) {
    fringe.push(n);
}

void Graph::empty_fringe() {
    while (fringe_has_nodes()) {
        get_node_from_fringe();
    }

    for (int i = 0; i < total_nodes; i++)
        nodes->at(i)->in_fringe = false;
}

node * Graph::get_node_from_fringe() {
    node * n = nullptr;

    while (fringe_has_nodes()) {
        n = fringe.top();
        fringe.pop();
        if (not n->in_closed)
            return n;
    }

    return n;
}

bool Graph::fringe_has_nodes() {
    return fringe.size() != 0;
}

void Graph::remove_all_nodes_from_closed() {
    for (int i = 0; i < total_nodes; i++) {
        nodes->at(i)->in_closed = false;
    }
}

std::forward_list<int> * Graph::astar(int start_node, int end_node) {
    return astar(start_node, end_node, false);
}

std::forward_list<int> * Graph::fd_astar(int start_node, int end_node) {
    return astar(start_node, end_node, true);
}

std::forward_list<int> * Graph::astar(int start_node, int end_node, bool free_direction) {
    nodes->at(start_node)->cost = 0;
    nodes->at(start_node)->parent = nodes->at(start_node);
    empty_fringe();
    nodes->at(start_node)->cost = heuristic_cost(start_node, end_node);
    add_node_to_fringe(nodes->at(start_node));
    remove_all_nodes_from_closed();

    while (fringe_has_nodes()) {
        node * s = get_node_from_fringe();
        if (s == nodes->at(end_node)) {
            return build_path(nodes->at(start_node), nodes->at(end_node));
        }
        s->in_closed = true;
        for (auto it = s->edges->begin(); it != s->edges->end(); it++) {
            if (not nodes->at((*it)->to_node)->in_closed) {
                if (not nodes->at((*it)->to_node)->in_fringe) {
                    nodes->at((*it)->to_node)->cost = std::numeric_limits<double>::infinity();
                    nodes->at((*it)->to_node)->parent = nullptr;
                }
                update_vertex(nodes->at((*it)->from_node), nodes->at((*it)->to_node),  nodes->at(end_node), *it, free_direction);
            }
        }
    }

    return nullptr;
}

void Graph::update_vertex(node * s, node * t, node * goal, edge * e, bool free_direction) {
    double h_cost;

/*    if (line_of_sight(s->parent, t)) {
        h_cost = heuristic_cost(t->parent->id, goal->id);
        if (s->parent->cost + h_cost < t->cost) {
            t->cost = s->parent->cost + h_cost;
            t->parent = s->parent;
            add_node_to_fringe(t);
        }
    } */
//    else {
    h_cost = heuristic_cost(t->id, goal->id);
    if (s->cost + e->weight + h_cost < t->cost) {
        t->cost = s->cost + e->weight + h_cost;
        t->parent = s;
        add_node_to_fringe(t);
    }
//    }
}

std::forward_list<int> * Graph::build_path(node * start_node, node * end_node) {
    if (end_node-> parent == nullptr)
        return nullptr;

    std::forward_list<int> * path = new std::forward_list<int>;
    node * temp = end_node;

    while (temp != start_node) {
        if (temp->parent == nullptr)
            return nullptr;
        path->push_front(temp->id);
        temp = temp->parent;
    }

    path->push_front(temp->id);

    return path;
}

void Graph::set_configuration(int s_id, Configuration * s_configuration) {
    nodes->at(s_id)->node_configuration = s_configuration;
}

double Graph::heuristic_cost(double s_id, double t_id) {
    return configuration_solver->get_distance(getNodes(s_id)->node_configuration, getNodes(t_id)->node_configuration, 0.5);
//    double dx = std::abs(nodes->at(s_id)->node_configuration->position->x - nodes->at(t_id)->node_configuration->position->x);
//    double dy = std::abs(nodes->at(s_id)->node_configuration->position->y - nodes->at(t_id)->node_configuration->position->y);
//    double min = dx < dy ? dx : dy;
//    double max = dx < dy ? dy : dx;
//    return std::sqrt(2) * min + max - min;
//    return sqrt(std::pow(nodes->at(s_id)->node_configuration->position->x - nodes->at(t_id)->node_configuration->position->x, 2)
//                + std::pow(nodes->at(s_id)->node_configuration->position->y - nodes->at(t_id)->node_configuration->position->y, 2));
}

void Graph::print_graph() {
    for (auto it_node = nodes->begin(); it_node != nodes->end(); it_node++) {
        std::cout << "Node " << (*it_node)->id << ": ";
        for (auto it_edge = (*it_node)->edges->begin(); it_edge != (*it_node)->edges->end(); it_edge++) {
            std::cout << (*it_edge)->to_node << " ";
        }
        std::cout << std::endl;
    }
}

void Graph::print_graph_with_weights() {
    for (auto it_node = nodes->begin(); it_node != nodes->end(); it_node++) {
        std::cout << "Node " << (*it_node)->id << ": ";
        for (auto it_edge = (*it_node)->edges->begin(); it_edge != (*it_node)->edges->end(); it_edge++) {
            std::cout << "(" << (*it_edge)->to_node << ", " << (*it_edge)->weight << ") ";
        }
        std::cout << std::endl;
    }
}
