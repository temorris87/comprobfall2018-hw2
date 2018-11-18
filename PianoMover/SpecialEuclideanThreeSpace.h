//
// Created by tyson on 10/26/18.
//

#ifndef PIANOMOVER_SPECIALEUCLIDEANTHREESPACE_H
#define PIANOMOVER_SPECIALEUCLIDEANTHREESPACE_H

#include <cstddef>
#include "include/PQP.h"

/**
 * Euclidean representation of a point (x,y,z).
 */
struct Translation {
    double x;
    double y;
    double z;
};

/**
 * Quaternion of the form (w,x,y,z).
 */
struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

/**
 * A configuration in SE(3) of the form (T, Q) where T is a translation in euclidean space and Q is a quaternion..
 */
struct Configuration {
    Translation * position;
    Quaternion * rotation;
};

class SpecialEuclideanThreeSpace {

private:
    // Fields
    double min_x, max_x;  // The minimum and maximum euclidean distances along x axis for a configuration.
    double min_y, max_y;  // The minimum and maximum euclidean distances along y axis for a configuration.
    double min_z, max_z;  // The minimum and maximum euclidean distances along z axis for a configuration.
    double max_distance;  // The maximum euclidean distance possible within the configuration space.

    Configuration * start_configuration;    // The start configuration for interpolation.
    Configuration * previous_configuration; // The current in between configuration for interpolation.
    Configuration * end_configuration;      // The end of the line configuration for interpolation.
    int total_number_of_steps;              // Steps requested for interpolation between two configurations.
    int current_step;                       // The current step along the interpolation path.
    Translation * delta_euclidean;          // The euclidean step forward for each step in interpolation.

    /**
     * Compute the euclidean distance for translating between two configurations in SE(3).
     * @param t1 The euclidean point associated with the first configuration point.
     * @param t2 The euclidean point associated with the second configuration point.
     * @return The euclidean distance between two configurations.
     */
    double get_translation_distance(Translation * t1, Translation * t2);

    /**
     * Compute the rotational distance for translating between two configurations in SE(3).
     * @param q1 The rotational configuration for the first configuration point.
     * @param q2 The rotationsl configuration for the second configuration point.
     * @return The rotational distance between two configurations.
     */
    double get_rotation_distance(Quaternion * q1, Quaternion * q2);

    /**
     * The dot product between two quaternions.
     * @param q1 The quaternion associated with the first configuration.
     * @param q2 The quaternion associated with the second configuration.
     * @return The dot product of q1 and q2.
     */
    double quaternion_inner_product(Quaternion * q1, Quaternion* q2);

    /**
     * Get a floating point random number in the range n to m.
     * @param n Lower bound for random double.
     * @param m Upper bound for random double.
     * @return Random double in [n, m]
     */
    double get_random_in_range(double n, double m);

    /**
     * Populate a new Translation with random (x,y,z) coordinates from the configuration space.
     * @return A randomly populated Translation structure.
     */
    Translation * get_random_translation();

    /**
     * Implementation of Algorithm 2 on page 3 in Effective Sampling and Distance Metrics.
     * @return A random unit length quaternion.
     */
    Quaternion * get_random_rotation();

    /**
     * Create a Translation that holds the step size in euclidean space between two configurations.
     * @param t1 Start translation location for interpolation.
     * @param t2 End translation location for interpolation.
     * @param n  The number of steps to break interpolation into.
     * @return A Translation that represents the step that must be taken at each interpolation step to follow path.
     */
    Translation * build_euclidean_interpolation(Translation * t1, Translation * t2, int n);

    /**
     * Move delta_euclidean away from t.
     * @param t Current translation.
     * @return Translation for euclidean interpolation adding delta_euclidean.
     */
    Translation * take_euclidean_interpolation_step(Translation * t);

    /**
     * Move delta_rotation away from q.
     * @param q Current rotation.
     * @return Rotation for spherical interpolation adding delta_rotation.
     */
    Quaternion * take_rotation_interpolation_step();

    /**
     * Normalize a quaternion to length 1.
     * @param q The Quaternion to normalize.
     * @return A normalized Quaternion.
     */
    Quaternion * normalize_quaternion(Quaternion * q);

public:

    Translation * get_random_end_translation();
    Configuration * get_random_end_configuration();
    Quaternion * get_random_end_rotation();

    /**
     * Construct the boundaries in euclidean space that any given configuration cannot violate.
     * @param min_x
     * @param max_x
     * @param min_y
     * @param max_y
     * @param min_z
     * @param max_z
     */
    SpecialEuclideanThreeSpace(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z);

    /**
     * Compute the distance between two points in the configuration space utilizing both the euclidean distance and
     * the rotational distance. The computed distance will be in the range [0,1].
     * @param c1 First configuration in question.
     * @param c2 Second configuration in question.
     * @param euclidean_weight A weight in the range [0,1] that determines the emphasis to be placed on the
     *                         euclidean distance. For example, a value of 0.5 will apply equal emphasis on both
     *                         euclidean distance and rotational distance. A value of 0.2 emphasizes rotation with a
     *                         value of 0.8.
     * @return The distance between two points in SE(3) normalized to the range [0,1].
     */
    double get_distance(Configuration * c1, Configuration * c2, double euclidean_weight);

    /**
     * Construct a random Configuration within the SE(3) utilizing the method described in the Effective Sampling
     * and Distance Metrics paper.
     * @return A random Configuration in SE(3).
     */
    Configuration * get_random_configuration();

    Configuration * get_nearby_random_configuration(Configuration * c);

    /**
     * Prepare the state required for tracking interpolation between two points for n values. This method is to be
     * used before get_next_interpolation_configuration. The interpolation will occur in a linear fashion, not jumping
     * around the line to try to find collisions faster.
     * @param start_config The beginning of the interpolation path.
     * @param end_config   The end of the interpolation path.
     * @param n            The number of configurations steps to be found in between start_config and end_config.
     */
    void setup_interpolation(Configuration * start_config, Configuration * end_config, int n);

    /**
     * Get the next configuration along the interpolation path between start_config and end_config as passed into
     * setup_interpolation. IMPORTANT: For accurate results, setup_interpolation must be called before any two points
     * can be interpolated!
     * @return The next configuration on the path between two configurations in SE(3). The returned configuration will
     *         be NULL if the number interpolations requested is larger than n set in setup_interpolation.
     */
    Configuration * get_next_interpolation_configuration();

    /**
     * Print a configuration of the form (<x,y,z>, <w, x', y', z'>) where <x,y,z> is a euclidean point and
     * <w, x', y', z'> is a quaternion that represents an orientation.
     * @param c The configuration to be printed out.
     */
    void print_configuration(Configuration * c);

    /**
    * Get the position vector that PQP uses for collision detecting.
    * @param c The configuration which needs to be converted.
    */
    void get_pqp_position(Configuration * c, PQP_REAL p[3]);

    /**
     * Get the orientation vector that PQP uses for collision detecting.
     * @param c The configuration which needs to be converted.
     */
    void get_pqp_rotation(Configuration * c, PQP_REAL p[3][3]);

};


#endif //PIANOMOVER_SPECIALEUCLIDEANTHREESPACE_H
