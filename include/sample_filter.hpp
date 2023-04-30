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
    /**
     * @brief  Loads parameters for the SampleFilter object
     *
     * @param param_loader object for loading parameters
     */
    void loadParameters(mrs_lib::ParamLoader &param_loader);
    /**
     * @brief Calculates weights for each point in the given sample set based on their distance to points in a dataset.
     *
     * The weights are calculated by finding all points in the dataset that are within a certain distance of each point in the sample set.
     * The score of each point in the dataset is increased by the number of nearby points found, and the score of each nearby sample point is increased by 1.
     * The weight of each sample point is then assigned based on the scores of nearby dataset points.
     * Finally, the weights of the dataset points are adjusted based on their scores, and the resulting points are stored in the dataset.
     *
     * @param samples The sample set for which to calculate weights.
     */
    void SurroundingModel(Points &samples);
    /**
     * @brief Calculates weights for each point in the given sample set based on their distance to points in a dataset.
     *
     * The weights are calculated by finding all points in the dataset that are within a certain distance of each point in the sample set.
     * The score of each point in the dataset is increased by the number of nearby points found, and the score of each nearby sample point is increased by 1.
     * The weight of each sample point is then assigned based on the scores of nearby dataset points.
     * Finally, the weights of the dataset points are adjusted based on their scores, and the resulting points are stored in the dataset.
     *
     * @param samples The sample set for which to calculate weights.
     */
    void SurroundingModel2(Points &samples);
    /**
     * @brief Calculate the average model of a set of points.
     *
     * This function calculates the average model of a set of points using a priority queue to sort the points
     * by their distance to other points.
     *
     * @param samples The set of points to calculate the average model of.
     */
    void AverageModel(Points &samples);
    /**
     * @brief Builds an average tree model of the input points.
     *
     * This function builds an average tree model of the input points. It takes a vector of
     * input points, builds a KDTree from the points, and then calculates the average
     * distance from each point to its nearest neighbors. The resulting points are stored
     * in a vector.
     *
     * @param samples The vector of input points to build the model from.
     */
    void AverageTreeModel(Points &samples);
    /**
     * @brief  Apply the SumOneModel method to the given sample set.  For each sample in the set, finds the nearest point in the dataset and calculate the squared distance between them. Updates the weights of the point in the dataset based on the found distances.
     *
     * @param samples   A vector of Points representing the samples to calculate weights for.
     */
    void SumOneModel(Points &samples);
    /**
     * @brief  Apply the SumAllModel method to the given sample set.
     *
     *  For each sample in the set, calculates the squared distance between it and every point in the dataset. Updates the weights of the point in the dataset based on the found distances.
     *
     * @param samples  A vector of Points representing the samples to calculate weights for.
     */
    void SumAllModel(Points &samples);
    /**
     * @brief  Apply the RandomModel method to the given sample set.
     *
     * For each sample in the set, selects a random subset of the dataset and calculates the squared distance between the sample and each point in the subset. Updates the weights of the point in the dataset based on the found distances.
     *
     * @param samples  A vector of Points representing the samples to calculate weights for.
     */
    void RandomModel(Points &samples);
    /**
     * @brief  Calculates the worst of num model for a given set of points This function calculates the worst of num model for a set of points, where the worst of num model is defined as the average distance to the worst num points in the dataset, where num is a parameter specified by the user. The function takes in a reference to a vector of points, calculates the average distance to the worst num points in the dataset for each point in the vector, and updates the weight and coefficient of each point based on this calculation.
     *
     * @param samples A vector of Points representing the samples to calculate weights for.
     */
    void WorstOfNumModel(Points &samples);
    /**
     * @brief Estimate the sources of a sound based on a dataset of recorded samples.
     *
     * This function uses the dataset of recorded samples to estimate the sources of a sound. It groups the
     * samples by proximity and then takes the average of each group to estimate the sources.
     *
     * @param estimation A reference to a vector of Vector3d objects to store the estimated sources in.
     */
    void estimateManySources(vector<Vector3d> &estimation);
    /**
     * @brief Sets the dataset to filter.
     * @param dataset The dataset to filter.
     */
    void setDataset(Points &dataset);
    /**
     * @brief Clear the dataset of SampleFilter.
     *
     * This function clears the dataset of SampleFilter by calling the clear() function on the vector container.
     */
    void clearDataset();
    /**
     * @brief Returns a string representation of the filter's settings.
     *
     * This function returns a string representation of the filter's settings, including dataset_limit, estimation_limit, estimation_dist, estimation_min_group_size, threshold_hit, threshold_distance, hit_score, miss_score, hit_position, miss_position, nearest_sum_n, queue_sum_n, input_coef_avg_best, output_size_avg_best, random_sample_coef, input_size_avg_worst, and output_coef_avg_worst
     *
     * @return A string representation of the filter's settings.
     */
    string get_settings_string();
    /**
     * @brief Gets the filtered dataset.
     * @return The filtered dataset.
     */
    Points getDataset();
    /**
     * @brief Gets the calculated estimation.
     * @return The estimation.
     */
    vector<Vector3d> getEstimation();

   private:
    Points dataset;                     /**< The filtered dataset. */
    vector<Eigen::Vector3d> estimation; /**< The estimation. */
    /**
     * @struct Estimate_sum_t
     * @brief A structure that holds the sum and number of coefficients for an estimate.
     *
     * This structure is used in SampleFilter::estimateManySources() to calculate the average estimate of a group of points.
     * @var Estimate_sum_t::sum
     * The sum of the points in the group.
     * @var Estimate_sum_t::coeff
     * The number of points in the group.
     */
    struct Estimate_sum_t;
    /**
     * @brief Adds the top elements of a priority queue to the filtered dataset.
     * @param queue The priority queue to get elements from.
     */
    inline void queueToDataset(priority_queue<Point> &queue);

   public:
    /**
     *  @The maximum number of points in the filtered dataset.
     */
    int dataset_limit = 2000;
    /*Estimations*/
    /**
     * @brief The maximum number of points to consider for estimation.
     */
    int estimation_limit = 200;
    /**
     * @brief The maximum distance between points to group them for estimation.
     */
    double estimation_dist = 3.0;
    /**
     * @brief The minimum number of points in a group to be considered for estimation.
     */
    int estimation_min_group_size = 5;
    /* Filters  */
    /*cicleFilter*/
    /**
     * @brief The minimum number of points required for a hit.
     */
    int threshold_hit = 0;
    /**
     * @brief The maximum distance from a hit to a point for it to be considered a hit.
     */
    double threshold_distance = 0.5;
    /**
     * @brief The score assigned to a hit.
     */
    int hit_score = 5;
    /**
     * @brief The score assigned to a miss.
     */
    int miss_score = -1;
    /**
     * @brief The position of a hit relative to the sensor.
     */
    double hit_position = 0.7;
    /**
     * @brief The position of a miss relative to the sensor.
     */
    double miss_position = 0.95;

    /*AverageTreeModel*/
    /**
     * @brief The number of nearest points to use in the weighted average for each point in the dataset.
     */
    double nearest_sum_n = 2;
    /**
     * @brief The number of points to use in the weighted average for each point in the dataset.
     */
    int queue_sum_n = 1;

    /*AverageModel*/
    /**
     * @brief The fraction of the dataset to use in the weighted average for each point.
     */
    double input_coef_avg_best = 0.4;  //<0, 1>
    /**
     * @brief The number of points to use in the weighted average for each output point.
     */
    int output_size_avg_best = 1;  // N

    /*RandomModel*/
    /**
     * @brief The fraction of the dataset to use in the random model.
     */
    double random_sample_coef = 0.33;  // 1/N
    /*WorstOfNumModel*/

    /**
     * @brief The number of points to use in the weighted average for each input point.
     */
    int input_size_avg_worst = 10;  // N
    /**
     * @brief The fraction of the dataset to use in the weighted average for each output point.
     */
    double output_coef_avg_worst = 0.4;  //<0, 1>
};

}  // namespace fusion_radiation

#endif
