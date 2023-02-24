#ifndef SAMPLE_FILTER_H
#define SAMPLE_FILTER_H

#include <eigen3/Eigen/Core>
#include <queue>
#include <vector>

#include "KDTree.hpp"
#include "point.hpp"

using namespace std;

namespace fusion_radiation {
class SampleFilter {
   public:
    void cicleFilter(Points &samples);
    void BestNumFilter(Points &samples);
    void estimateManySources(vector<Vector3d> & estimation);
    void setDataset(Points &dataset);
    Points getDataset();

   private:
    Points dataset;
    struct Estimate_sum_t;

    double treshold = 0.5;
    int database_limit = 200;
    int estimation_limit = 200;
    double estiamtion_dist = 3;
    int estimation_min_group_size = 5;
};
}  // namespace fusion_radiation

#endif
