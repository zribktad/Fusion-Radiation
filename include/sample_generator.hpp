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
    static void generateSamplesUniform(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    static void generateSamplesLines(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    static void generateSamplesRandom(const Cone &cone, OcTreePtr_t collisions, Points &samples);
    static void updateCollisions(OcTreePtr_t collisions);
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

    inline static void sampleConeUniform(const Cone &cone, Points &samples);
    inline static void sampleLines(const Cone &cone, Points &samples);
    inline static void sampleConeRandom(const Cone &cone, Points &samples);

    inline static void sampleOnLine(const Vector3d &origin, const Vector3d &cicle_point, Points &samples);
    inline const static bool checkCollisions(const Vector3d &point);
    inline static void generateInit(OcTreePtr_t collisions);
    inline static const Vector3d getPointOnConeCicle(const Cone &cone, const double &alpha);
};
}  // namespace fusion_radiation

#endif
