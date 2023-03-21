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
#include "csv_file_writer.hpp"

namespace fusion_radiation {

class FusionRun {
   private:
    inline static SampleFilter filter = {};
    inline static int draw_limit_dataset = 400;
    inline static CSVFileWriter csv_estimations = {"estim"}; 
    inline static CSVFileWriter csv_particles = {"part"}; 

   public:
    static void loadParameters(mrs_lib::ParamLoader& param_loader);
    static void processData(const Cone& cone, OcTreePtr_t collisions);
    inline static vector<Vector3d> estimation = {};
};

}  // namespace fusion_radiation

#endif /* FUSION_RUN*/
