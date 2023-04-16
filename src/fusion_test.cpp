#include "fusion_test.hpp"

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <chrono>
#include <cmath>
#include <cone.hpp>
#include <memory>
#include <random>
#include <sample_generator.hpp>
#include <string>

namespace fusion_radiation {

fusion_radiation::OcTreePtr_t generateRandomOctomap(double resolution, int range, bool plane = true, int num_nodes = 100000) {
    octomap::OcTree tree(resolution);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-range, range);

    
        for (int x = -range; x < range; x++) {
            for (int y = -range; y < range; y++) {
                for (int z = 0; z < 2; z++) {
                    octomap::point3d point(x, y, 0- z);
                    tree.updateNode(point, true);
                }
            }
        }

    if (!plane) {
        for (size_t i = 0; i < num_nodes; i++) {
            octomap::point3d point(dist(gen), dist(gen), dist(gen));
            tree.updateNode(point, true);
        }
    }

    // Convert to OcTreePtr_t
    fusion_radiation::OcTreePtr_t tree_ptr = std::make_shared<octomap::OcTree>(tree);

    return tree_ptr;
}

inline Eigen::Vector3d generate_random_vector(double x_min, double x_max,
                                              double y_min, double y_max,
                                              double z_min, double z_max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x_dist(x_min, x_max);
    std::uniform_real_distribution<double> y_dist(y_min, y_max);
    std::uniform_real_distribution<double> z_dist(z_min, z_max);
    return Eigen::Vector3d(x_dist(gen), y_dist(gen), z_dist(gen));
}

inline Eigen::Vector3d generate_random_vector(double range = 10, double shift = 0) {
    return generate_random_vector(-range + shift, range + shift, -range + shift, range + shift, -range + shift, range + shift);
}

Points FusionTest::generatorPoints(const uint size) {
    Points r;
    r.reserve(size);
    for (size_t i = 0; i < size / 3; i++) {
        r.push_back({{(double)(rand() % 100), (double)(rand() % 10), 0}});
        r.push_back({{(double)(rand() % 10 + 1000), (double)(rand() % 100 + 100), 0}});
        r.push_back({{(double)(rand() % 10 - 1000), (double)(rand() % 100 + 100), 0}});
    }

    return r;
}

template <typename Func, typename... Args>
double FusionTest::measure_time(Func&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();
    std::invoke(func, std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count();
}

template <typename... Funcs>
void FusionTest::compare_time(Funcs&&... funcs) {
    std::vector<std::pair<std::string, std::function<void()>>> func_vec = {
        {"Function 1", std::forward<Funcs>(funcs)}...};

    for (const auto& func : func_vec) {
        double elapsed_time = measure_time(func.second);
        ROS_INFO_STREAM(func.first << " took " << elapsed_time << " seconds.");
    }
}

void FusionTest::timeCompareSampler() {
      const int loop = 1000;
    {
        vector<double> all_rs_size(3);
        vector<double> times(3);
        OcTreePtr_t collisions = generateRandomOctomap(1, 100);
        vector<Points> rs;
      
        for (size_t i = 0; i < loop; i++) {
            rs.resize(3);
            Cone cone(20);
            times[2] += measure_time(&SampleGenerator::generateSamplesLines, cone, collisions, rs[2]);

            times[1] += measure_time(&SampleGenerator::generateSamplesRandom, cone, collisions, rs[1]);

            times[0] += measure_time(&SampleGenerator::generateSamplesUniform, cone, collisions, rs[0]);
            // ROS_INFO_STREAM("samples:  uniform: " << rs[0].size() << " random: " << rs[1].size() << " lines: " << rs[2].size());
            all_rs_size[0] += rs[0].size();
            all_rs_size[1] += rs[1].size();
            all_rs_size[2] += rs[2].size();

            rs.clear();
        }
        ROS_INFO_STREAM("times:  uniform: " << times[0]/loop << " random: " << times[1]/loop << " lines: " << times[2]/loop);
        ROS_INFO_STREAM("rs size uniform: " << all_rs_size[0]/loop << " random: " << all_rs_size[1]/loop << " lines: " << all_rs_size[2]/loop);
    }
    {
        vector<double> all_rs_size(3);
        vector<double> times(3);
        OcTreePtr_t collisions = generateRandomOctomap(1, 100, false,300000);
        vector<Points> rs;
        for (size_t i = 0; i < loop; i++) {
            rs.resize(3);
            Cone cone(50);
            times[2] += measure_time(&SampleGenerator::generateSamplesLines, cone, collisions, rs[2]);

            times[1] += measure_time(&SampleGenerator::generateSamplesRandom, cone, collisions, rs[1]);

            times[0] += measure_time(&SampleGenerator::generateSamplesUniform, cone, collisions, rs[0]);
            // ROS_INFO_STREAM("samples:  uniform: " << rs[0].size() << " random: " << rs[1].size() << " lines: " << rs[2].size());
            all_rs_size[0] += rs[0].size();
            all_rs_size[1] += rs[1].size();
            all_rs_size[2] += rs[2].size();

            rs.clear();
        }
        ROS_INFO_STREAM("times:  uniform: " << times[0]/loop << " random: " << times[1]/loop << " lines: " << times[2]/loop);
        ROS_INFO_STREAM("rs size uniform: " << all_rs_size[0]/loop << " random: " << all_rs_size[1]/loop << " lines: " << all_rs_size[2]/loop);
    }
}

}  // namespace fusion_radiation