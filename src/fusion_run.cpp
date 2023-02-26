#include "fusion_run.hpp"

#include <iostream>

namespace fusion_radiation {

void FusionRun::loadParameters(mrs_lib::ParamLoader& param_loader) {
    SampleGenerator::loadParameters(param_loader);
    filter.loadParameters(param_loader);
}

void FusionRun::generateSample(const Cone& cone, OcTreePtr_t collisions) {
    Points samples;
    SampleGenerator::generateSamplesLines(cone, collisions, samples);
    ROS_INFO_STREAM(" New generated samples size:" << samples.size() );
    PointVisualzer::drawPoints(samples, {0.5, 0.5, 0.5, 0.8});
   
//    /*Filter part */
//    filter.CicleFilter(samples);
//     //get estimation of radiation sources
    
//     filter.estimateManySources(estimation);
//     PointVisualzer::drawPoints(estimation, {0, 1, 0, 1});

//    const auto& dataset = filter.getDataset();
//     PointVisualzer::drawPoints(dataset, {0, 0, 1, 0.8}, (const ulong)200);
//    // Point::writePoints(dataset);
}
}  // namespace fusion_radiation