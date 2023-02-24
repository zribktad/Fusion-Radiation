#ifndef FUSION_RUN
#define FUSION_RUN

#include <iostream>
#include <vector>
#include <mrs_lib/batch_visualizer.h>


#include "point.hpp"
#include "sample_filter.hpp"
#include "sample_generator.hpp"
#include "point_visualizer.hpp"
#include "cone.hpp"


namespace fusion_radiation {

class FusionRun {

 private:
    inline static uint estimate_size = 200;
    inline static uint mark_as_best_size = 200;
    inline static SampleFilter filter ={};
public:
    static void initTesting(int estimate_size,int mark_as_best_size);
    static void generateSample(const Cone& cone, OcTreePtr_t collisions);




};

}  // namespace fusion_radiation

#endif /* FUSION_RUN*/
