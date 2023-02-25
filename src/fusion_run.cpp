#include "fusion_run.hpp"

#include <iostream>

namespace fusion_radiation {

void FusionRun::loadTesting(mrs_lib::ParamLoader& param_loader) {
    SampleGenerator::loadParameters(param_loader);
    filter.loadParameters(param_loader);
}

void FusionRun::generateSample(const Cone& cone, OcTreePtr_t collisions) {
    Points samples;
    SampleGenerator::generateSamplesUniform(cone, collisions, samples);
    std::cout << " samples:" << samples.size() << endl;
    PointVisualzer::drawPoints(samples, {0.5, 0.5, 0.5, 0.8});
   filter.cicleFilter(samples);
    //get estimation of radiation sources
    vector<Vector3d> estimation;
    filter.estimateManySources(estimation);
    PointVisualzer::drawPoints(estimation, {0, 1, 0, 1});

   const auto& dataset = filter.getDataset();
   
    PointVisualzer::drawPoints(dataset, {0, 0, 1, 0.8}, (const ulong)200);
    Point::writePoints(dataset);
}
}  // namespace fusion_radiation