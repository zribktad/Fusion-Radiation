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
    void SurroundingModel(Points &samples);
    void SurroundingModel2(Points &samples);
    void AverageModel(Points &samples);
    void AverageTreeModel(Points &samples);
    void SumOneModel(Points &samples);
    void SumAllModel(Points &samples);
    void RandomModel(Points &samples);
    void WorstOfNumModel(Points &samples);
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
    int threshold_hit =0;
    double threshold_distance = 0.5;
    int hit_score = 5;
    int miss_score = -1;
    double hit_position = 0.7;
    double miss_position = 0.95;

    /*AverageTreeModel*/
    double nearest_sum_n = 2;
    int queue_sum_n = 1;
    /*AverageModel*/
    double input_coef_avg_best = 0.4;  //<0, 1>
    int output_size_avg_best = 1;      // N
    /*RandomModel*/
    double random_sample_coef = 0.33;  // 1/N
    /*WorstOfNumModel*/
    int input_size_avg_worst = 10;       // N
    double output_coef_avg_worst = 0.4;  //<0, 1>
};

}  // namespace fusion_radiation

#endif
