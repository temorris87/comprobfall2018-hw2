//
// Created by tyson on 10/26/18.
//

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include "SpecialEuclideanThreeSpace.h"

SpecialEuclideanThreeSpace::SpecialEuclideanThreeSpace(double min_x, double max_x,
                                                       double min_y, double max_y,
                                                       double min_z, double max_z) {
    this->min_x = min_x;
    this->max_x = max_x;
    this->min_y = min_y;
    this->max_y = max_y;
    this->min_z = min_z;
    this->max_z = max_z;

    // Longest euclidean distance in a rectangular prism is the diagonal.
    this->max_distance = sqrt(pow((max_x - min_x), 2) + pow((max_y - min_y), 2) + pow((max_z - min_z), 2));

    // Initialize seed for future randomization.
    srand((unsigned int)time(0));


}

//--------------------------------------------------------------------------------------------------------------------
//
// get_distance and helper functions for computing the distance between two configurations.
//
//--------------------------------------------------------------------------------------------------------------------

double SpecialEuclideanThreeSpace::get_distance(Configuration * c1, Configuration * c2, double euclidean_weight) {
    double rotation_weight = 1 - euclidean_weight;
    double euclidean_distance = get_translation_distance(c1->position, c2->position);
    double rotation_distance = get_rotation_distance(c2->rotation, c2->rotation);

    return euclidean_weight * euclidean_distance + rotation_weight * rotation_distance;
}

double SpecialEuclideanThreeSpace::get_translation_distance(Translation * t1, Translation * t2) {
    return sqrt(pow(t1->x - t2->x, 2) + pow(t1->y - t2->y, 2) + pow(t1->z - t2->z, 2)) / max_distance;
}

double SpecialEuclideanThreeSpace::get_rotation_distance(Quaternion * q1, Quaternion * q2) {
    return 1 - abs(quaternion_inner_product(q1, q2));
}

double SpecialEuclideanThreeSpace::quaternion_inner_product(Quaternion * q1, Quaternion* q2) {
    return (q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z);
}

//--------------------------------------------------------------------------------------------------------------------
//
// get_random_configuration and its helper functions.
//
//--------------------------------------------------------------------------------------------------------------------

Configuration * SpecialEuclideanThreeSpace::get_random_configuration() {
    Configuration * configuration = new Configuration();

    configuration->position = get_random_translation();
    configuration->rotation = get_random_rotation();

    return configuration;
}

Configuration * SpecialEuclideanThreeSpace::get_random_end_configuration() {
    Configuration * configuration = new Configuration();

    configuration->position = get_random_end_translation();
    configuration->rotation = get_random_end_rotation();

    return configuration;
}

double SpecialEuclideanThreeSpace::get_random_in_range(double n, double m) {
    double r = ((float) rand()) / RAND_MAX;
    if (m > n)
        return (m - n) * r + n;
    else
        return (n - m) * r + m;
}

Translation * SpecialEuclideanThreeSpace::get_random_translation() {
    Translation * pos = new Translation();

    pos->x = get_random_in_range(min_x, max_x);
    pos->y = get_random_in_range(min_y, max_y);
    pos->z = get_random_in_range(min_z, max_z);

    return pos;
}

Quaternion * SpecialEuclideanThreeSpace::get_random_rotation() {
    Quaternion * rot = new Quaternion();

    double s = get_random_in_range(0, 1);

    double sigma_1 = sqrt(1 - s);
    double sigma_2 = sqrt(s);

    double theta_1 = 2 * M_PI * get_random_in_range(0, 1);
    double theta_2 = 2 * M_PI * get_random_in_range(0, 1);

    rot->w = cos(theta_2) * sigma_2;
    rot->x = sin(theta_1) * sigma_1;
    rot->y = cos(theta_1) * sigma_1;
    rot->z = sin(theta_2) * sigma_2;

    return rot;
}

Translation * SpecialEuclideanThreeSpace::get_random_end_translation() {
    Translation * pos = new Translation();

    pos->x = get_random_in_range(min_x, max_x);
    pos->y = get_random_in_range(min_y, max_y);
    pos->z = get_random_in_range(0.4, 0.4);
}

Quaternion * SpecialEuclideanThreeSpace::get_random_end_rotation() {
    Quaternion * rot = new Quaternion();

    double roll = 0;
    double pitch = 0;
    double yaw = get_random_in_range(0, 2 * M_PI);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    rot->w = cy * cr * cp + sy * sr * sp;
    rot->x = cy * sr * cp - sy * cr * sp;
    rot->y = cy * cr * sp + sy * sr * cp;
    rot->z = sy * cr * cp - cy * sr * sp;
    
    return rot;
}

void SpecialEuclideanThreeSpace::get_pqp_position(Configuration * c, PQP_REAL pos[3]) {
    PQP_REAL * pqp_position[3];

    pos[0] = c->position->x;
    pos[1] = c->position->y;
    pos[2] = c->position->z;
}

void SpecialEuclideanThreeSpace::get_pqp_rotation(Configuration * c, PQP_REAL rot[3][3]) {
    double r = c->rotation->w;
    double i = c->rotation->x;
    double j = c->rotation->y;
    double k = c->rotation->z;

    double i2 = pow(i, 2);
    double j2 = pow(j, 2);
    double k2 = pow(k, 2);

    rot[0][0] = 1 - 2*(j2 + k2);
    rot[0][1] = 2 * (i*j - k*r);
    rot[0][2] = 2 * (i*k + j*r);
    rot[1][0] = 2 * (i*j + k*r);
    rot[1][1] = 1 - 2*(i2 + k2);
    rot[1][2] = 2 * (j*k - i*r);
    rot[2][0] = 2 * (i*k - j*r);
    rot[2][1] = 2 * (j*k + i*r);
    rot[2][2] = 1 - 2*(i2 + j2);
}

//--------------------------------------------------------------------------------------------------------------------
//
// Functions and helper functions for dealing with interpolation between two configurations.
//
//--------------------------------------------------------------------------------------------------------------------

void SpecialEuclideanThreeSpace::setup_interpolation(Configuration *start_config, Configuration *end_config, int n) {
    this->start_configuration = start_config;
    this->previous_configuration = start_config;
    this->end_configuration = end_config;
    this->total_number_of_steps = n;
    this->current_step = 0;
    this->delta_euclidean = build_euclidean_interpolation(start_config->position, end_config->position, n);
}

Configuration * SpecialEuclideanThreeSpace::get_next_interpolation_configuration() {
    this->current_step++;
    if (this->current_step >= this->total_number_of_steps)
        return NULL;

    Configuration * next_configuration = new Configuration();
    next_configuration->position = take_euclidean_interpolation_step(previous_configuration->position);
    next_configuration->rotation = take_rotation_interpolation_step();

    this->previous_configuration = next_configuration;

    return next_configuration;
}

Translation *SpecialEuclideanThreeSpace::build_euclidean_interpolation(Translation *t1, Translation *t2, int n) {
    Translation * delta_euclidean = new Translation();

    delta_euclidean->x = (t2->x - t1->x) / n;
    delta_euclidean->y = (t2->y - t1->y) / n;
    delta_euclidean->z = (t2->z - t1->z) / n;

    return delta_euclidean;
}

Quaternion *SpecialEuclideanThreeSpace::take_rotation_interpolation_step() {
    double epsilon = 0.1; // Determines tolerance for linear vs spherical interpolation.
    Quaternion * q1 = this->start_configuration->rotation;
    Quaternion * q2 = this->end_configuration->rotation;
    int n = this->total_number_of_steps;

    double lambda = quaternion_inner_product(q1, q2);

    double r;
    double s;
    double f = ((float) this->current_step) / n; // Fractional distance per step.

    if (lambda < 0) {
        // The quaternions are pointing in opposite directions, reverse q2.
        q2->w = -1 * q2->w;
        q2->x = -1 * q2->x;
        q2->y = -1 * q2->y;
        q2->z = -1 * q2->z;
        lambda = -1 * lambda;
    }

    // Calculate interpolation factors.
    if (1 - lambda < epsilon) {
        // The quaternions are nearly parallel, so we use linear interpolation.
        r = 1 - f;
        s = f;
    }
    else {
        // Calculate the spherical interpolation factors.
        double alpha = acos(lambda);
        double gamma = 1.0 / sin(alpha);
        r = sin((1 - f) * alpha) * gamma;
        s = sin(f * alpha) * gamma;
    }

    // Set the interpolated quaternion.
    Quaternion * delta_rotation = new Quaternion();
    delta_rotation->w = r * q1->w + s * q2->w;
    delta_rotation->x = r * q1->x + s * q2->x;
    delta_rotation->y = r * q1->y + s * q2->y;
    delta_rotation->z = r * q1->z + s * q2->z;

    // Normalize the result.
    return normalize_quaternion(delta_rotation);
}

Translation *SpecialEuclideanThreeSpace::take_euclidean_interpolation_step(Translation * t) {
    Translation * next_translation = new Translation();

    next_translation->x = t->x + delta_euclidean->x;
    next_translation->y = t->y + delta_euclidean->y;
    next_translation->z = t->z + delta_euclidean->z;

    return next_translation;
}

Quaternion *SpecialEuclideanThreeSpace::normalize_quaternion(Quaternion * q) {
    double length = sqrt(pow(q->w, 2) + pow(q->x, 2) + pow(q->y, 2) + pow(q->z, 2));

    Quaternion * normalized = new Quaternion();
    normalized->w = q->w / length;
    normalized->x = q->x / length;
    normalized->y = q->y / length;
    normalized->z = q->z / length;

    return normalized;
}

//--------------------------------------------------------------------------------------------------------------------
//
// Functions for viewing state of what's going on internally.
//
//--------------------------------------------------------------------------------------------------------------------

void SpecialEuclideanThreeSpace::print_configuration(Configuration * c) {
    Translation * t = c->position;
    Quaternion * q = c->rotation;

    std::cout << "( <" << t->x << ", " << t->y << ", " << t->z << ">, <"
              << q->w << ", " << q->x << ", " << q->y << ", " << q->z << "> )\n";
}

Configuration *SpecialEuclideanThreeSpace::get_nearby_random_configuration(Configuration *c) {
    Configuration * nearby = new Configuration();
    nearby->position = new Translation();

    nearby->position->x = c->position->x + get_random_in_range(-0.1, 0.1);
    nearby->position->y = c->position->y + get_random_in_range(-0.1, 0.1);
    nearby->position->z = c->position->z;

    nearby->rotation = get_random_rotation();

    return nearby;
}
