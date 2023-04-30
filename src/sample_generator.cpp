/**
 * @file sample_generator.cpp
 * @author Tadeáš Zribko
 * @brief  The SampleGenerator class generates points in a given cone using three methods: uniform sampling, random sampling, and lines sampling.
 * @version 0.1
 * @date 2023-04-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "sample_generator.hpp"

#include <random>

namespace fusion_radiation {
using namespace octomap;

void SampleGenerator::loadParameters(mrs_lib::ParamLoader &param_loader) {
    param_loader.loadParam("generator/start_capacity", start_capacity);
    param_loader.loadParam("generator/start_lenght", start_lenght);
    param_loader.loadParam("generator/end_lenght", end_lenght);
    /*random*/
    param_loader.loadParam("generator/uniform/range", range);
    /*lines*/
    param_loader.loadParam("generator/lines/number_lines", number_lines);
    /*random*/
    param_loader.loadParam("generator/random/capacity_lines", capacity_lines);
    param_loader.loadParam("generator/random/number_iterations", number_iterations);
    param_loader.loadParam("generator/random/start_dist_weight", start_dist_weight);
    param_loader.loadParam("generator/random/end_dist_weight", end_dist_weight);
}

inline void SampleGenerator::generateInit(OcTreePtr_t collisions) {
    SampleGenerator::collisions = collisions;
    number_of_sample++;
}

void SampleGenerator::generateSamplesUniform(const Cone &cone, OcTreePtr_t collisions, Points &samples) {
    generateInit(collisions);
    sampleConeUniform(cone, samples);
}

void SampleGenerator::generateSamplesLines(const Cone &cone, OcTreePtr_t collisions, Points &samples) {
    generateInit(collisions);
    sampleLines(cone, samples);
}

void SampleGenerator::generateSamplesRandom(const Cone &cone, OcTreePtr_t collisions, Points &samples) {
    generateInit(collisions);
    sampleConeRandom(cone, samples);
}

void SampleGenerator::updateCollisions(OcTreePtr_t collisions) {
SampleGenerator:
    collisions = collisions;
}

inline const Vector3d SampleGenerator::getPointOnConeCicle(const Cone &cone, const double &alpha) {
    return (cone.direction_norm_side + cone.u_side * cos(alpha) + cone.v_side * sin(alpha));
}

inline void SampleGenerator::sampleConeUniform(const Cone &cone, Points &samples) {
    const double norm_range = range / sin(cone.angle);
    samples.reserve(start_capacity);
    // samples.reserve(static_cast<size_t>((end_lenght - start_lenght) * max_PI / norm_range));

    for (double height = start_lenght; height < end_lenght; height += range) {
        const double step = norm_range / height;
        for (double alpha = 0; alpha < max_PI; alpha += step) {
            const Vector3d &point = cone.origin + height * getPointOnConeCicle(cone, alpha);
            if (checkCollisions(point)) {
                samples.emplace_back(point, INFINITY, number_of_sample);
            }
        }
    }
}

inline void SampleGenerator::sampleLines(const Cone &cone, Points &samples) {
    if (!collisions)
        return;

    samples.reserve(number_lines);

    const double step = max_PI / number_lines;
    for (double alpha = 0; alpha < max_PI; alpha += step) {  // part of cicle <-PI,PI>
        const Vector3d &cicle_point = getPointOnConeCicle(cone, alpha);
        sampleOnLine(cone.origin, cicle_point, samples);
    }
}

// sampling points on line
inline void SampleGenerator::sampleOnLine(const Vector3d &origin, const Vector3d &cicle_point, Points &samples) {  // lines

    const Vector3d start_point = cicle_point * start_lenght + origin;
    const Vector3d end_point = cicle_point * end_lenght + origin;

    octomap::KeyRay key_ray;
    collisions->computeRayKeys(point3d(start_point.x(), start_point.y(), start_point.z()), point3d(end_point.x(), end_point.y(), end_point.z()), key_ray);

    for (const auto &key : key_ray) {
        const OcTreeNode *node = collisions->search(key);
        if (node  && collisions->isNodeOccupied(node)) {
            const point3d coord = collisions->keyToCoord(key);
            Point sample(Vector3d(coord.x(), coord.y(), coord.z()), INFINITY, number_of_sample);
            samples.emplace_back(sample);
        }
    }
}

inline void SampleGenerator::sampleConeRandom(const Cone &cone, Points &samples) {
    // samples.reserve(number_iterations);

    static std::random_device rd;
    static std::mt19937 gen(rd());
    static const vector<double> v{start_lenght, end_lenght};
    static const vector<double> w{start_dist_weight, end_dist_weight};
    static std::piecewise_linear_distribution<double> height_dist(v.begin(), v.end(), w.begin());
    static std::uniform_real_distribution<double> cicle_dist(0, max_PI);
    static std::uniform_int_distribution<> line_dist(0, capacity_lines - 1);

    const auto start = cone.origin;
    vector<Eigen::Vector3d> lines;
    lines.reserve(capacity_lines);

    while (lines.size() < capacity_lines) {
        const auto cicle_p = getPointOnConeCicle(cone, cicle_dist(gen));
        lines.emplace_back(cicle_p);
    }

    const size_t num_samples = number_iterations;
    samples.reserve(num_samples);
    for (size_t j = 0; j < num_samples; j++) {
        const auto d = lines[line_dist(gen)];
        Eigen::Vector3d point = start + height_dist(gen) * d;
        if (checkCollisions(point)) {
            samples.push_back({point, INFINITY, number_of_sample});
        }
    }
}
// This function checks if the given sample collides with the octomap
inline const bool SampleGenerator::checkCollisions(const Vector3d &sample) {
    // Check if the octomap has been loaded

    if (!collisions) {
         ROS_WARN_THROTTLE(1.0,"Octomap not loaded!");
        return true;
    }
    // Convert the sample to octomap coordinates
    const point3d coord(sample.x(), sample.y(), sample.z());
    // Convert the octomap coordinates to an octree key
    OcTreeKey coord_key;
    if (!collisions->coordToKeyChecked(coord, coord_key)) {
        return false;  // out of bounds
    }
    // Search the octomap for the node corresponding to the key
    OcTreeNode *node = collisions->search(coord_key);
    return node && collisions->isNodeOccupied(node);
}

}  // namespace fusion_radiation
