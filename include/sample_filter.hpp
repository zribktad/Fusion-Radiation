#ifndef SAMPLE_FILTER_H
#define SAMPLE_FILTER_H

#include <mrs_lib/param_loader.h>

#include <eigen3/Eigen/Core>
#include <queue>
#include <vector>

#include "KDTree.hpp"
#include "point.hpp"

using namespace std;

namespace fusion_radiation {
class SampleFilter {
   public:
    void loadParameters(mrs_lib::ParamLoader &param_loader);
    void CircleFilter(Points &samples);
    void CircleFilter2(Points &samples);
    void BestOfNumFilter(Points &samples);
    void SumNumFilter(Points &samples);
    void SumOneFilter(Points &samples);
    void SumAllFilter(Points &samples);
    void RandomFilter(Points &samples);
    void WorstOfNumFilter(Points &samples);
    void estimateManySources(vector<Vector3d> &estimation);
    void setDataset(Points &dataset);
    Points getDataset();
    vector<Vector3d> getEstimation();

   private:
    Points dataset;
    vector<Eigen::Vector3d> estimation;
    struct Estimate_sum_t;

    inline void queueToDataset(priority_queue<Point> &queue);

    int dataset_limit = 2000;  // buffer size
    /*Estimations*/
    int estimation_limit = 200;
    double estimation_dist = 3.0;
    int estimation_min_group_size = 5;
    /* Filters  */
    /*cicleFilter*/
    double treshold = 0.5;
    int hit_score = 5;
    int miss_score = -1;
    double hit_position = 0.7;
    double miss_position = 0.95;
    /*SumNumFilter*/
    double nearest_sum_n = 2;
    int queue_sum_n = 1;
    /*BestOfNumFilter*/
    double input_coef_avg_best = 0.4;  //<0, 1>
    int output_size_avg_best = 1;      // N
    /*RandomFilter*/
    double random_sample_coef = 0.33;  // 1/N
    /*WorstOfNumFilter*/
    int input_size_avg_worst = 10;       // N
    double output_coef_avg_worst = 0.4;  //<0, 1>
};

}  // namespace fusion_radiation

#endif
