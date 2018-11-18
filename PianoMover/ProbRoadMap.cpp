//
// Created by Eric Fiore on 10/30/2018.


#include "ProbRoadMap.h"
#include "MeshReader.h"

SpecialEuclideanThreeSpace * configuration_solver;

ProbRoadMap::ProbRoadMap(int numNodes, int nearest_neighbors, double min_x, double max_x,
        double min_y, double max_y, double min_z, double max_z) : Graph() {
    //total_nodes = 0;
    Graph::total_nodes = 0;
    totalNodesToCreate = numNodes;
    configuration_solver = new SpecialEuclideanThreeSpace(min_x, max_x, min_y, max_y, min_z, max_z);
}

void ProbRoadMap::create_road_map(std::string * vehicle_string, std::string * room_string) {
    MeshReader * piano_meshes = new MeshReader(*vehicle_string);
    this->piano = piano_meshes->model;
    MeshReader * room_meshes = new MeshReader(*room_string);
    this->room = room_meshes->model;

    for (int i = 0; i < totalNodesToCreate; i++) {
        Configuration *test_config = get_random_config();
        bool test_result = check_collision(test_config, piano, room);
        if (test_result) {
            node * child_node = create_world_node(test_config);
            node * parent_node = get_closest_neighbors(test_config);
            if (parent_node != nullptr) {
                child_node->parent = parent_node;
                int edge_cost = configuration_solver->get_distance(parent_node->node_configuration, child_node->node_configuration, 0.5);
                Graph::add_edge(parent_node->id, child_node->id, edge_cost);
            }
        }
    }
    print_road_map();

}

node * ProbRoadMap::create_world_node(Configuration * config_to_add) {
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
        temp->id = Graph::set_total_nodes();
    Graph::setNodes(temp);
    return temp;
}

node * ProbRoadMap::get_closest_neighbors(Configuration *incoming_configuration) {
    int number_of_nodes = Graph::get_total_nodes();
    for (int i = 0; i < number_of_nodes; i++){
        if (configuration_solver->get_distance(incoming_configuration, Graph::getNodes(i)->node_configuration, 0.5) <= distance
            && (incoming_configuration != Graph::getNodes(i)->node_configuration)) {
            bool interpolation_result = connect_config_to_neighbors(incoming_configuration,
                                                                    Graph::getNodes(i)->node_configuration);
            if (!interpolation_result)
                continue;
            return Graph::getNodes(i);
        }
    }
    return nullptr;
}

bool ProbRoadMap::connect_config_to_neighbors(Configuration *main_configuration,
                                              Configuration *neighbor_configuration) {
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
    }
    return true;
}

bool ProbRoadMap::check_collision(Configuration * configuration_one, PQP_Model * vehicle_model, PQP_Model * room_model){
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

void ProbRoadMap::print_road_map() {
    for (int i = 0; i < Graph::get_total_nodes(); i++){
        std::cout << "Node " << Graph::getNodes(i)->id << ":";
        node * temp_node = Graph::getNodes(i);
        while (temp_node->parent != nullptr){
            temp_node = temp_node->parent;
            std::cout << " <-- " << temp_node->id;
        }
        std::cout << std::endl;
    }
}

Configuration * ProbRoadMap::get_random_config() {
    return configuration_solver->get_random_configuration();
}

