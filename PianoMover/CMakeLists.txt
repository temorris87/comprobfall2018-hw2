cmake_minimum_required(VERSION 2.8)
project(PianoMover)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Debug)



add_executable(PianoMover
        Graph.cpp
        Graph.h
        SpecialEuclideanThreeSpace.cpp
        SpecialEuclideanThreeSpace.h
        MeshReader.cpp
        MeshReader.h
        ${CMAKE_SOURCE_DIR}/lib/Build.o
        ${CMAKE_SOURCE_DIR}/lib/BV.o
        ${CMAKE_SOURCE_DIR}/lib/PQP.o
        ${CMAKE_SOURCE_DIR}/lib/TriDist.o
        PianoMover.cpp)

target_link_libraries(PianoMover ${CMAKE_SOURCE_DIR}/lib/libPQP.a)
