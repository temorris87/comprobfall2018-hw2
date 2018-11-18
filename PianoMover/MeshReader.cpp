//
// Created by Eric Fiore on 10/31/2018.
//

#include "MeshReader.h"
#include "include/PQP.h"
#include <string>
//#include <string.h>

MeshReader::MeshReader(std::string incoming_file_name) {
    std::string file_name = incoming_file_name + ".txt";
    char char_file_name[file_name.size() + 1];
    strcpy(char_file_name, file_name.c_str());

    std::ifstream infile;
    infile.open(file_name, std::ifstream::in);
    if(!infile.is_open()){
        std::cout << "error in opening file" << std::endl;
        exit(EXIT_FAILURE);
    }
    model = model_reader(infile);
    infile.close();
}

PQP_Model * MeshReader::model_reader(std::ifstream & incoming_file){
    int num_triangles = 0;
    std::ifstream &file_to_read = incoming_file;
    file_to_read >> num_triangles;
    std::cout << "number of triangles is " << + num_triangles << std::endl;
    PQP_Model * read_model = new PQP_Model;
    read_model->BeginModel();

    for (int i = 0; i < num_triangles; i++){
        PQP_REAL pointOne[3], pointTwo[3], pointThree[3];
        for(int n = 0; n < 3; n++){
            double point = 0;
            file_to_read >> point;
            pointOne[n] = point;
        }
        for(int n = 0; n < 3; n++){
            double point = 0;
            file_to_read >> point;
            pointTwo[n] = point;
        }
        for(int n = 0; n < 3; n++){
            double point = 0;
            file_to_read >> point;
            pointThree[n] = point;
        }
        read_model->AddTri(pointOne, pointTwo, pointThree, i);
    }
    read_model->EndModel();
    return read_model;
}