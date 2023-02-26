
#ifndef FUSION_TEST
#define FUSION_TEST

#include <point.hpp>
#include <vector>

#include "cone.hpp"

using namespace Eigen;
using namespace std;

namespace fusion_radiation {



class FusionTest {
   public:
    static Points generatorPoints(const uint size);
    static void timeCompareSampler();
    template <typename Func, typename... Args>
    static double measure_time(Func&& func, Args&&... args);
    template <typename... Funcs>
    static void compare_time(Funcs&&... funcs);

};  // namespace fusion_radiation

}  // namespace fusion_radiation
#endif