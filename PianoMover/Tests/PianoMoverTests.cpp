//
// Created by tyson on 10/26/18.
//

#include <iostream>
#include <math.h>
#include "PianoMoverTests.h"
#include "../SpecialEuclideanThreeSpace.h"

/*
bool test_sets_random_numbers() {
    SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 20, 0, 20, 0, 20);

    for (int i = 0; i < 2000000; i++) {
        double r = sets->get_random_in_range(0, 20);
        if (r < 0 || r > 20) {
            std::cout << "Error: " << r << std::endl;
            return false;
        }
    }
    return true;
}
*/

/*
bool test_sets_euclidean_length() {
    SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 1, 0, 1, 0, 1);

    Translation * temp;
    double distance;
    for (int i = 0; i < 20000000; i++) {
        temp = sets->get_random_translation();
        distance = sqrt(pow(temp->x, 2) + pow(temp->y, 2) + pow(temp->z, 2));

        if (distance < 0 || distance > sets->max_distance) {
            std::cout << "Error: " << distance << std::endl;
            return false;
        }
    }

    return true;
}
*/

/*
bool test_sets_quaternion_length() {
    SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 20, 0, 20, 0, 20);

    Quaternion * temp;
    double distance;
    for (int i = 0; i < 2000000; i++) {
        temp = sets->get_random_rotation();
        distance = sqrt(pow(temp->w, 2) + pow(temp->x, 2) + pow(temp->y, 2) + pow(temp->z, 2));

        if (abs(1 - distance) > 0.0001) {
            std::cout << "Error: " << distance << std::endl;
            return false;
        }
    }

    return true;
}
*/

/*
bool test_sets_configuration_distances() {
    SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 1, 0, 1, 0, 1);

    Configuration * c1;
    Configuration * c2;
    double distance;
    for (int i = 0; i < 20000000; i++) {
        c1 = sets->get_random_configuration();
        c2 = sets->get_random_configuration();
        distance = sets->get_distance(c1, c2, 0.3);
        if (distance < 0 || distance > 1) {
            std::cout << "Error for distance: " << distance << std::endl;
            return false;
        }
    }

    return true;
}
*/

/*
void test_sets_view_interpolation() {
    SpecialEuclideanThreeSpace * sets = new SpecialEuclideanThreeSpace(0, 1, 0, 1, 0, 1);

    Configuration * c1 = sets->get_random_configuration();
    Configuration * c2 = sets->get_random_configuration();

    sets->print_configuration(c1);
    sets->print_configuration(c2);

    sets->setup_interpolation(c1, c2, 10);

    Configuration * interpolation = sets->get_next_interpolation_configuration();
    while (interpolation != NULL) {
        sets->print_configuration(interpolation);
        interpolation = sets->get_next_interpolation_configuration();
    }
}
*/
