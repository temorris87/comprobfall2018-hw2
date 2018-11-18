//
// Created by Eric Fiore on 10/31/2018.
//

#ifndef PIANOMOVER_MESHREADER_H
#define PIANOMOVER_MESHREADER_H

#include <string>
#include <cstring>
#include <fstream>
#include <iostream>
#include "include/PQP.h"

class MeshReader {

public:
    PQP_Model * model;
    MeshReader(std::string fileName);
    PQP_Model * model_reader(std::ifstream & incoming_file);

};


#endif //PIANOMOVER_MESHREADER_H
