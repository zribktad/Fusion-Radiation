
#ifndef FUSION_TEST
#define FUSION_TEST

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <chrono>
#include <cmath>
#include <cone.hpp>
#include <memory>
#include <point.hpp>
#include <random>
#include <sample_generator.hpp>
#include <string>
#include <vector>

#include "cone.hpp"

using namespace Eigen;
using namespace std;

/**
 * @namespace fusion_radiation
 * @brief Namespace for functions related to radiation fusion
 */
namespace fusion_radiation {

using OcTreePtr_t = std::shared_ptr<octomap::OcTree>;

/**
 * @class FusionTest
 * @brief Class to test radiation fusion related functions
 */
class FusionTest {
   public:
    /**
     * @brief Generate an OcTreePtr_t with a plane at z=origin.z()
     * @param resolution The resolution of the OcTree
     * @param range The range of the plane in x and y directions
     * @param origin The origin of the plane
     * @return The generated OcTreePtr_t
     */
    static OcTreePtr_t generateOctomapPlane(double resolution, int range, Vector3d origin);

    /**
     * @brief Generate a vector of points for testing
     * @param size The size of the vector
     * @return The generated vector
     */
    static Points generatorPoints(const uint size);
    /**
     * @brief Compare the time taken by different sampler generation functions
     */
    static void timeCompareSampler();
    /**
     * @brief Measure the time taken by a function to run
     * @tparam Func The type of the function
     * @tparam Args The types of the function arguments
     * @param func The function to measure
     * @param args The arguments to pass to the function
     * @return The time taken by the function in seconds
     */
    template <typename Func, typename... Args>
    static double measure_time(Func&& func, Args&&... args);
    /**
     * @brief Compare the time taken by multiple functions
     * @tparam Funcs The types of the functions
     * @param funcs The functions to compare
     */
    template <typename... Funcs>
    static void compare_time(Funcs&&... funcs);

};  // namespace fusion_radiation

}  // namespace fusion_radiation
#endif