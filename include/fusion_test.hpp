
#ifndef FUSION_TEST
#define FUSION_TEST

#include <point.hpp>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <chrono>
#include <cmath>
#include <cone.hpp>
#include <memory>
#include <random>
#include <sample_generator.hpp>
#include <string>


#include "cone.hpp"

using namespace Eigen;
using namespace std;

namespace fusion_radiation {

using OcTreePtr_t = std::shared_ptr<octomap::OcTree>;

class FusionTest {
   public:

    static OcTreePtr_t generateOctomapPlane(double resolution, int range, Vector3d origin);


    static Points generatorPoints(const uint size);
    static void timeCompareSampler();
    template <typename Func, typename... Args>
    static double measure_time(Func&& func, Args&&... args);
    template <typename... Funcs>
    static void compare_time(Funcs&&... funcs);

};  // namespace fusion_radiation

}  // namespace fusion_radiation
#endif