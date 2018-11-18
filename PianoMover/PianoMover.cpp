//
// Created by tyson on 10/31/18.
//

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include "SpecialEuclideanThreeSpace.h"
#include "Tests/PianoMoverTests.h"
#include "Graph.h"
#include "MeshReader.h"

int main() {
    //SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 1, 0, 1, 0, 1);
    std::string * vehicle =  new std::string("/home/eric/CLionProjects/robotics-02/PianoMover/piano");
    std::string * room = new std::string("/home/eric/CLionProjects/robotics-02/PianoMover/room");

//    MeshReader * room = new MeshReader("/home/eric/CLionProjects/robotics-02/PianoMover/room");
//    MeshReader * piano = new MeshReader("/home/eric/CLionProjects/robotics-02/PianoMover/piano");
    Graph * new_roadmap = new Graph(1000, 5, -10, 10, -10, 10, 0, 2);
    new_roadmap->create_kroad_map(vehicle, room);
    std::forward_list<int> * path = new_roadmap->astar(0, 950);

    if (path == nullptr)
        std::cout << "No path." << std::endl;
    else
        std::cout << "Path found." << std::endl;

    return 0;
}