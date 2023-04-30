/**
 * @file sample_generator.cpp
 * @author Tadeáš Zribko
 * @brief  The SampleGenerator class generates points in a given cone using three methods: uniform sampling, random sampling, and lines sampling.
 * @version 0.1
 * @date 2023-04-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef SAMPLE_GENERATOR_H
#define SAMPLE_GENERATOR_H

#include <mrs_lib/param_loader.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeKey.h>
#include <ros/ros.h>

#include <eigen3/Eigen/Core>
#include <vector>

#include "cone.hpp"
#include "point.hpp"

using namespace std;
using namespace Eigen;
namespace fusion_radiation {

using OcTreePtr_t = std::shared_ptr<octomap::OcTree>;

class SampleGenerator {
   public:
    /**
     * @brief Generates uniform samples within the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param collisions A pointer to an OcTree object representing the collisions.
     * @param samples A vector of points to which the generated samples are added.
     */
    static void generateSamplesUniform(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    /**
     * @brief Generates samples on lines in the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param collisions A pointer to an OcTree object representing the collisions.
     * @param samples A vector of points to which the generated samples are added.
     */
    static void generateSamplesLines(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    /**
     * @brief Generates random samples within the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param collisions A pointer to an OcTree object representing the collisions.
     * @param samples A vector of points to which the generated samples are added.
     */
    static void generateSamplesRandom(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    /**
     * @brief Updates the collisions object.
     *
     * @param collisions A pointer to an OcTree object representing the collisions.
     */
    static void updateCollisions(OcTreePtr_t collisions);

    /*
     * @brief Loads the parameters required for generating samples.
     *
     * @param param_loader A reference to the parameter loader object.
     */
    static void loadParameters(mrs_lib::ParamLoader &param_loader);

   private:
    inline static ulong number_of_sample = 0;
    inline static OcTreePtr_t collisions = {};
    inline static const double max_PI = 2 * M_PI;

    /*Parameters*/
    inline static int start_capacity = 150;
    inline static double start_lenght = 5;
    inline static double end_lenght = 15;
    /*Uniform variables*/
    inline static double range = 0.8;
    /*Lines variables*/
    inline static int number_lines = 100;
    /*Random variables*/
    inline static int capacity_lines = 100;
    inline static int number_iterations = 400;
    inline static double start_dist_weight = 0.2;
    inline static double end_dist_weight = 1;
    /**
     * @brief Generates uniform samples within the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param samples A vector of points to which the generated samples are added.
     */
    inline static void sampleConeUniform(const Cone &cone, Points &samples);
    /**
     * @brief Generates samples on lines in the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param samples A vector of points to which the generated samples are added.
     */
    inline static void sampleLines(const Cone &cone, Points &samples);
    /**
     * @brief Generates random samples within the cone.
     *
     * @param cone The cone within which the samples are generated.
     * @param samples A vector of points to which the generated samples are added.
     */
    inline static void sampleConeRandom(const Cone &cone, Points &samples);

    /**
     * @brief Generates samples on a line.
     *
     * @param origin The start point of the line.
     * @param cicle_point A point on the circle of the cone.
     * @param samples A vector of points to which the generated samples are added.
     */
    inline static void sampleOnLine(const Vector3d &origin, const Vector3d &cicle_point, Points &samples);
    /**
     * @brief Check if a given point is in collision with the environment
     *
     * @param point The point to be checked
     * @return true if the point is not in collision, false otherwise
     */
    inline const static bool checkCollisions(const Vector3d &point);
    /**
     * @brief Generates initial samples by setting the collisions object and incrementing the number of samples.
     *
     * @param collisions A pointer to an OcTree object representing the collisions.
     */
    inline static void generateInit(OcTreePtr_t collisions);
    /**
     * @brief Get a point on the cicle of a given cone at a specified angle
     *
     * @param cone The cone to get the point from
     * @param alpha The angle (in radians) on the cicle
     * @return A point on the cicle of the given cone at the specified angle
     */
    inline static const Vector3d getPointOnConeCicle(const Cone &cone, const double &alpha);
};
}  // namespace fusion_radiation

#endif
