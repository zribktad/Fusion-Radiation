#include "fusion_run.hpp"

#include <iostream>

namespace fusion_radiation {

void FusionRun::loadParameters(mrs_lib::ParamLoader& param_loader) {
    SampleGenerator::loadParameters(param_loader);
    filter.loadParameters(param_loader);

      param_loader.loadParam("sample_filter/draw_limit_dataset", draw_limit_dataset);
}

void FusionRun::processData(const Cone& cone, OcTreePtr_t collisions) {
    Points samples;
    /*Sampling*/
    SampleGenerator::generateSamplesUniform(cone, collisions, samples);
    ROS_INFO_STREAM(" New generated samples size:" << samples.size());

    /*Filter part */
    filter.CircleFilter(samples);

    filter.estimateManySources(estimation);  // get estimation of radiation sources
    const auto& dataset = filter.getDataset();

    /*Drawing*/
    PointVisualzer::drawPoints(samples, {0.5, 0.5, 0.5, 0.8});
    PointVisualzer::drawPoints(estimation, {0, 1, 0, 1});
    PointVisualzer::drawPoints(dataset, {0, 0, 1, 0.8}, (const ulong)draw_limit_dataset);
    // Point::writePoints(dataset);
}
}  // namespace fusion_radiation