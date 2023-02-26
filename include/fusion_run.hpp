#ifndef FUSION_RUN
#define FUSION_RUN

#include <mrs_lib/batch_visualizer.h>

#include <iostream>
#include <vector>

#include "cone.hpp"
#include "point.hpp"
#include "point_visualizer.hpp"
#include "sample_filter.hpp"
#include "sample_generator.hpp"

namespace fusion_radiation {

class FusionRun {
   private:
    inline static uint estimate_size = 200;
    inline static uint mark_as_best_size = 200;
    inline static SampleFilter filter = {};

   public:
    static void loadTesting(mrs_lib::ParamLoader& param_loader);
    static void generateSample(const Cone& cone, OcTreePtr_t collisions);
    inline static vector<Vector3d> estimation = {};
};

}  // namespace fusion_radiation

#endif /* FUSION_RUN*/
