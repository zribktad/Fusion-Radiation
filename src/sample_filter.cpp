#include "sample_filter.hpp"

#include <cmath>

namespace fusion_radiation {

struct SampleFilter::Estimate_sum_t {
    Vector3d sum;
    uint coeff;
};

void SampleFilter::loadParameters(mrs_lib::ParamLoader &param_loader) {
    param_loader.loadParam("sample_filter/dataset_limit", dataset_limit);
    /* source Estimations*/
    param_loader.loadParam("sample_filter/estimation_limit", estimation_limit);
    param_loader.loadParam("sample_filter/estimation_dist", estimation_dist);
    param_loader.loadParam("sample_filter/estimation_min_group_size", estimation_min_group_size);
    /*cicleFilter*/
    param_loader.loadParam("sample_filter/treshold", treshold);
    /*SumNumFilter*/
    param_loader.loadParam("sample_filter/nearest_sum_n", nearest_sum_n);
    param_loader.loadParam("sample_filter/queue_sum_n", queue_sum_n);
    /*BestOfNumFilter*/
    param_loader.loadParam("sample_filter/input_coef_avg_best", input_coef_avg_best);
    param_loader.loadParam("sample_filter/output_size_avg_best", output_size_avg_best);
    /*RandomFilter*/
    param_loader.loadParam("sample_filter/random_sample_coef", random_sample_coef);
    /*WorstOfNumFilter*/
    param_loader.loadParam("sample_filter/input_size_avg_worst", input_size_avg_worst);
    param_loader.loadParam("sample_filter/output_coef_avg_worst", output_coef_avg_worst);
}

void SampleFilter::estimateManySources(vector<Vector3d> &estimation) {
    const int dataset_size = dataset.size();
    const uint limit = estimation_limit != 0 ? min(dataset_size, estimation_limit) : dataset_size;
    vector<int> group(limit, -1);
    const double ref_distance = estimation_dist * estimation_dist;
    vector<Estimate_sum_t> group_set;
    int group_index = 0;
    for (ulong i = 0; i < limit; i++) {
        auto &start_point = dataset[i].coord;
        if (group[i] == -1) {
            Estimate_sum_t estimate_sum = {Vector3d(start_point.x(), start_point.y(), start_point.z()), 1};
            queue<reference_wrapper<Vector3d>> q;
            q.emplace(start_point);
            while (!q.empty()) {
                const Vector3d &ref_point = q.front();
                q.pop();
                for (ulong j = i + 1; j < limit; j++) {
                    Vector3d &p = dataset[j].coord;
                    if (group[j] == -1) {
                        const double distance = (ref_point - p).squaredNorm();
                        if (distance <= ref_distance) {
                            group[j] = group_index;
                            estimate_sum.sum += p;
                            estimate_sum.coeff++;
                            q.emplace(p);
                        }
                    }
                }
            }
            group_index++;
            group_set.emplace_back(estimate_sum);
        }
    }

    estimation.reserve(group_index);
    for (const auto &itr : group_set) {
        if (itr.coeff >= estimation_min_group_size) {
            estimation.emplace_back(itr.sum / itr.coeff);
        }
    }
    this->estimation = estimation;
}

void SampleFilter::SumNumFilter(Points &samples) {
    KDTree tree(samples);
    priority_queue<Point> dataset_q;
    vector<priority_queue<double, vector<double>, greater<double>>> distances_samples(samples.size());

    for (size_t j = 0; j < dataset.size(); j++) {
        Point &point = dataset[j];
        vector<ulong> indexes;
        vector<double> distances;
        tree.findNearestPoints(point, nearest_sum_n, indexes, distances);
        double sum = 0;
        for (size_t i = 0; i < distances.size(); i++) {
            sum += distances[i];
            auto &d_s = distances_samples[indexes[i]];
            d_s.emplace(distances[i]);
        }

        if (point.weight == INFINITY) {
            point.weight = sum / nearest_sum_n;
            point.coeff = nearest_sum_n;
        } else {
            point.weight = (point.weight * point.coeff + sum) / (point.coeff + nearest_sum_n);
            point.coeff += nearest_sum_n;
        }
        dataset_q.emplace(point);
    }

    for (size_t i = 0; i < samples.size(); i++) {
        auto &d_s = distances_samples[i];
        const auto limit = min(d_s.size(), (ulong)queue_sum_n);
        double sum = 0;
        for (size_t j = 0; j < limit; j++) {
            sum += d_s.top();
            d_s.pop();
        }
        Point &p = samples[i];
        p.weight = sum / limit;
        p.coeff = limit;
        dataset_q.emplace(p);
    }

    queueToDataset(dataset_q);
}

void SampleFilter::BestOfNumFilter(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<vector<double>> ref_new_weights(dataset.size());
    const ulong limit_in = (ulong)(dataset.size() * input_coef_avg_best);
    for (Point &point : samples) {
        vector<double> distance_for_point(dataset.size());
        for (ulong i = 0; i < dataset.size(); ++i) {
            Point &ref_point = dataset[i];
            double distance = point.distanceSquared(ref_point);
            distance_for_point[i] = distance;
            ref_new_weights[i].emplace_back(distance);
        }
        partial_sort(distance_for_point.begin(), distance_for_point.begin() + limit_in, distance_for_point.end());
        double sum_dist = 0;
        for (ulong i = 0; i < limit_in; i++) {
            sum_dist += distance_for_point[i];
        }
        if (sum_dist != 0) {
            point.coeff = limit_in;
            point.weight = sum_dist / limit_in;
        }

        data_priority_queue.emplace(point);
    }

    for (ulong i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        vector<double> &new_weight = ref_new_weights[i];
        const ulong limit_out = new_weight.size() > output_size_avg_best ? output_size_avg_best : new_weight.size();
        partial_sort(new_weight.begin(), new_weight.begin() + limit_out, new_weight.end());
        double sum_dist = 0;
        for (ulong i = 0; i < limit_out; i++) {
            sum_dist += new_weight[i];
        }
        if (sum_dist != 0)
            if (point.weight == INFINITY) {
                point.weight = sum_dist / limit_out;
                point.coeff = limit_out;
            } else {
                point.weight = (point.weight * point.coeff + sum_dist) / (point.coeff + limit_out);
                point.coeff += limit_out;
            }
        data_priority_queue.emplace(point);
    }

    queueToDataset(data_priority_queue);
}

// Choose BEST OF DISTANCE
void SampleFilter::SumOneFilter(Points &samples) {
    KDTree tree(dataset);
    priority_queue<Point> data_priority_queue;
    vector<double> dataset_weights(dataset.size(), INFINITY);

    for (Point &point : samples) {
        vector<ulong> indexes;
        vector<double> distances;
        tree.findNearestPoints(point, 1, indexes, distances);
        if (indexes.size()) {
            const double distanceSQ = distances[0];
            double &dataset_weight = dataset_weights[indexes[0]];

            point.weight = distanceSQ;
            point.coeff = 1;
            data_priority_queue.emplace(point);

            if (distanceSQ < dataset_weight) {
                dataset_weight = distanceSQ;
            }
        }
    }
    for (ulong i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        double &new_weight = dataset_weights[i];

        if (point.weight == INFINITY) {
            point.weight = new_weight;
        } else {
            point.weight = point.weight + (new_weight - point.weight) / (++(point.coeff));
        }
        data_priority_queue.emplace(point);
    }

    queueToDataset(data_priority_queue);
}

// all for all
void SampleFilter::SumAllFilter(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<double> ref_new_weights(dataset.size());
    vector<ulong> ref_new_weights_count(dataset.size());
    const ulong dataset_size = dataset.size();
    for (Point &point : samples) {
        double point_sum = 0;
        for (ulong i = 0; i < dataset_size; ++i) {
            Point &ref_point = dataset[i];
            double distance = point.distanceSquared(ref_point);
            point_sum += distance;
            ref_new_weights[i] += distance;
            ref_new_weights_count[i]++;
        }
        if (!dataset.empty()) {
            point.weight = point_sum / dataset_size;
            point.coeff += dataset_size;
        }

        data_priority_queue.emplace(point);
    }
    for (ulong i = 0; i < dataset_size; ++i) {
        Point &point = dataset[i];
        double &new_weight = ref_new_weights[i];
        ulong new_size = ref_new_weights_count[i];

        if (point.weight == INFINITY) {
            point.coeff = new_size;
            point.weight = new_weight / new_size;
        } else {
            point.weight = (point.weight * point.coeff + new_weight) / (point.coeff + new_size);
            point.coeff += new_size;
        }

        data_priority_queue.emplace(point);
    }

    queueToDataset(data_priority_queue);
}

void SampleFilter::RandomFilter(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<double> ref_new_weights(dataset.size(), INFINITY);

    srand(time(0));
    ulong limit_in = (ulong)dataset.size() * random_sample_coef;
    for (Point &point : samples) {
        for (ulong i = 0; i < limit_in; ++i) {
            ulong index = rand() % dataset.size();
            Point &ref_point = dataset[index];
            double &ref_new_weight = ref_new_weights[index];
            double distance = point.distanceSquared(ref_point);

            if (point.weight == INFINITY) {
                point.weight = distance;
            } else {
                point.weight = point.weight + (distance - point.weight) / (++(point.coeff));
            }
            if (distance < ref_new_weight) {
                ref_new_weight = distance;
            }
        }

        data_priority_queue.emplace(point);
    }

    for (int i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        double &new_weight = ref_new_weights[i];
        if (point.weight == INFINITY) {
            point.weight = new_weight;
        } else {
            point.weight = point.weight + (new_weight - point.weight) / (++(point.coeff));
        }

        data_priority_queue.emplace(point);
    }

    queueToDataset(data_priority_queue);
}

void SampleFilter::WorstOfNumFilter(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<priority_queue<double>> ref_new_weights(dataset.size());
    // const ulong limit_in = (ulong)dataset.size() *input_size_avg_worst ;
    const ulong limit_out = (ulong)samples.size() * output_coef_avg_worst;

    for (Point &point : samples) {
        priority_queue<double> distance_for_point;
        for (ulong i = 0; i < dataset.size(); ++i) {
            Point &ref_point = dataset[i];
            priority_queue<double> &ref_new_weight = ref_new_weights[i];
            double distance = point.distanceSquared(ref_point);
            distance_for_point.emplace(distance);
            ref_new_weight.emplace(distance);
        }
        const ulong limit_in = distance_for_point.size() > input_size_avg_worst ? input_size_avg_worst : distance_for_point.size();
        double sum_dist = 0;
        for (ulong i = 0; i < limit_in; i++) {
            double distance = distance_for_point.top();
            distance_for_point.pop();
            sum_dist += distance;
        }
        if (sum_dist != 0) {
            point.coeff = limit_in;
            point.weight = sum_dist / limit_in;
        }
        data_priority_queue.emplace(point);
    }
    for (ulong i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        priority_queue<double> &new_weight = ref_new_weights[i];
        // const ulong limit = new_weight.size() > output_coef_avg_worst? output_coef_avg_worst: new_weight.size();
        double sum_dist = 0;
        for (ulong i = 0; i < limit_out; i++) {
            double distance = new_weight.top();
            new_weight.pop();
            sum_dist += distance;
        }
        if (sum_dist != 0)
            if (point.weight == INFINITY) {
                point.weight = sum_dist / limit_out;
                point.coeff = limit_out;
            } else {
                point.weight = point.weight + (sum_dist - point.weight) / (point.coeff + limit_out);
                point.coeff += limit_out;
            }
        data_priority_queue.emplace(point);
    }
    queueToDataset(data_priority_queue);
}

void SampleFilter::CicleFilter(Points &samples) {
    // Initialize priority queue and score vectors
    priority_queue<Point> dataset_q;
    vector<double> score_dataset(dataset.size());
    vector<double> score_samples(samples.size());
    KDTree tree(samples);  // Build KD-Tree for efficient nearest-neighbor search
    const ulong dataset_size = dataset.size();
    for (ulong i = 0; i < dataset_size; ++i) {  // Calculate score for each point in the dataset

        vector<ulong> result;
        tree.findPointsWithinDistance(dataset[i], treshold, result);  // Find all points within a certain distance of the current point in the dataset

        score_dataset[i] += result.size();  // Increase the score of the current dataset point by the number of nearby points found

        for (const ulong &index : result) {
            ++score_samples[index];  // Increase the score of each nearby sample point by 1 / can be changed
        }
    }
    const ulong samples_size = samples.size();
    if (dataset_size) {
        // Calculate indices for the "better" and "worst" points in the dataset based on hit_position and miss_position
        const ulong better_index = dataset_size * hit_position - 1;
        const ulong worst_index = dataset_size * miss_position - 1;
        for (ulong j = 0; j < samples_size; j++) {  // Assign weights to each sample point based on nearby dataset point scores
            Point &point = samples[j];
            point.weight = score_samples[j] > 0 ? dataset[better_index].weight : dataset[worst_index].weight;
            dataset_q.emplace(point);  // Add the sample point to the priority queue
        }
    } else {
        for (ulong j = 0; j < samples_size; j++) {  // If there are no points in the dataset, assign weight of 0 to each sample point
            Point &point = samples[j];
            point.weight = 0;
            dataset_q.emplace(point);  // Add the sample point to the priority queue
        }
    }
    for (ulong i = 0; i < dataset_size; ++i) {  // Adjust weights of dataset points based on their scores
        Point &point = dataset[i];
        point.weight -= score_dataset[i] > 0 ? hit_score : miss_score;
        dataset_q.emplace(point);  // Add the dataset point to the priority queue
    }
    // Convert the priority queue to a vector and store it as the new dataset
    queueToDataset(dataset_q);
}

void SampleFilter::CicleFilter2(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<double> score_dataset(dataset.size(), 0);

    for (Point &point : samples) {
        double score_for_point = 0;
        for (ulong i = 0; i < dataset.size(); ++i) {
            Point &ref_point = dataset[i];
            double distance = point.distanceSquared(ref_point);
            if (distance < 0.5) {
                score_for_point++;
                // if(!score_dataset[i])
                score_dataset[i]++;
            } else {
                //   score_for_point--;
                // score_dataset[i]--;
            }
        }
        if (dataset.size()) {
            point.weight = score_for_point > 0 ? dataset[dataset.size() * 0.7 - 1].weight : dataset[dataset.size() * 0.95 - 1].weight;
        }
        data_priority_queue.push(point);
    }

    for (ulong i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        if (point.weight == INFINITY) {
            point.weight = 0;
        }

        point.weight += score_dataset[i] > 0 ? -5 : 1;

        data_priority_queue.push(point);
    }

    queueToDataset(data_priority_queue);
}

void SampleFilter::setDataset(Points &dataset) {
    SampleFilter::dataset = dataset;
}

Points SampleFilter::getDataset() {
    return dataset;
}

inline void SampleFilter::queueToDataset(priority_queue<Point> &queue) {
    const ulong queue_limit = min((int)queue.size(), dataset_limit);
    Points new_dataset(queue_limit);

    for (int i = 0; i < queue_limit; ++i) {
        new_dataset[i] = std::move(queue.top());
        queue.pop();
    }

    dataset = std::move(new_dataset);
}

 vector<Vector3d>  SampleFilter::getEstimation()  {
    return estimation;
}

}  // namespace fusion_radiation