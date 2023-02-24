#include "sample_filter.hpp"

#include <cmath>

namespace fusion_radiation {

struct SampleFilter::Estimate_sum_t {
    Vector3d sum;
    uint coef;
};

void SampleFilter::estimateManySources(vector<Vector3d> &estimation) {
    const int dataset_size = dataset.size();
    const uint limit = min(dataset_size, estimation_limit);
    vector<int> group(limit, -1);

    const double ref_distance = estiamtion_dist * estiamtion_dist;
    vector<Estimate_sum_t> group_set;

    int group_index = 0;
    for (ulong i = 0; i < limit; i++) {
        auto &start_point = dataset[i].coord;

        if (group[i] == -1) {
            Estimate_sum_t estimate_sum = {Vector3d(start_point.x(), start_point.y(), start_point.z()), 1};

            queue<reference_wrapper<Vector3d>> q;
            q.push(start_point);

            while (!q.empty()) {
                const Vector3d &ref_point = q.front();
                q.pop();
                // cout << "    QuE pos: "<<group  << " coord: " << ref_point.transpose() << " \tgroup: "<< group_index<< endl;
                for (ulong j = i + 1; j < limit; j++) {
                    Vector3d &p = dataset[j].coord;
                    if (group[j] == -1) {
                        const double distance = (ref_point - p).squaredNorm();
                        //     cout << "       tmp pos: "<<group  << " coord: " << p.transpose() << " \tgroup: "<< p.group<<"\tdistance: " << distance <<endl;
                        if (distance <= ref_distance) {
                            group[j] = group_index;
                            estimate_sum.sum += p;
                            estimate_sum.coef++;
                            q.push(p);
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
        if (itr.coef >= estimation_min_group_size) {
            estimation.emplace_back(itr.sum / itr.coef);
        }
    }
}
uint dataset_best_num = 0;
void SampleFilter::BestNumFilter(Points &samples) {
    KDTree tree(samples);
    vector<priority_queue<double, vector<double>, greater<double>>> distances_samples(samples.size());
    for (Point &point : dataset) {
        vector<ulong> indexes;
        vector<double> distances;
        tree.findNearestPoints(point, dataset_best_num, indexes, distances);
        double sum = 0;
        for (size_t i = 0; i < distances.size(); i++) {
            sum += distances[i];
            priority_queue<double, vector<double>, greater<double>> &d_s = distances_samples[indexes[i]];
            d_s.emplace(distances[i]);
        }
    }
}

void SampleFilter::cicleFilter(Points &samples) {
    priority_queue<Point> data_priority_queue;
    vector<double> score_dataset(dataset.size(), 0);
    vector<double> score_samples(samples.size(), 0);
    KDTree tree(samples);

    const ulong dateset_size = dataset.size();

    for (ulong i = 0; i < dateset_size; ++i) {
        vector<ulong> result;
        tree.findPointsWithinDistance(dataset[i], treshold, result);
        score_dataset[i] += result.size();
        for (const ulong &index : result) {
            score_samples[index]++;
        }
    }
    const ulong samples_size = samples.size();
    if (dateset_size) {
        const ulong better_index = dateset_size * 0.7 - 1;
        const ulong worst_index = dateset_size * 0.95 - 1;
        for (ulong j = 0; j < samples_size; j++) {
            Point &point = samples[j];
            point.weight = score_samples[j] > 0 ? dataset[better_index].weight : dataset[worst_index].weight;
            data_priority_queue.emplace(point);
        }
    } else {
        for (ulong j = 0; j < samples_size; j++) {
            Point &point = samples[j];
            point.weight = 0;
            data_priority_queue.emplace(point);
        }
    }

    for (ulong i = 0; i < dataset.size(); ++i) {
        Point &point = dataset[i];
        point.weight += score_dataset[i] > 0 ? -5 : 1;
        data_priority_queue.emplace(point);
    }

    const ulong queue_limit = min((int)data_priority_queue.size(), database_limit);

    Points new_dataset(queue_limit);

    for (int i = 0; i < queue_limit; ++i) {
        new_dataset[i] = std::move(data_priority_queue.top());
        data_priority_queue.pop();
    }
    dataset = std::move(new_dataset);
}

void SampleFilter::setDataset(Points &dataset) {
    SampleFilter::dataset = dataset;
}

Points SampleFilter::getDataset() {
    return dataset;
}
}  // namespace fusion_radiation