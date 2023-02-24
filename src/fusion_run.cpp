#include "fusion_run.hpp"
#include <iostream>

namespace fusion_radiation {

void FusionRun::initTesting(int estimate_size, int mark_as_best_size) {
    FusionRun::estimate_size = estimate_size;
    FusionRun::mark_as_best_size = mark_as_best_size;
}

void FusionRun::generateSample(const Cone& cone, OcTreePtr_t collisions) {
    Points samples;
    SampleGenerator::generateSamplesUniform(cone, collisions, samples);
    std::cout  << " samples:"  << samples.size() << endl;
    PointVisualzer::drawPoints(samples, {0.0, 0.0, 1.0, 0.8},(const ulong)mark_as_best_size);
    filter.cicleFilter(samples);
    
}
}  // namespace fusion_radiation