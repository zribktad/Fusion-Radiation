

#include <chrono>
#include <memory>
#include <vector>

#include "KDTree.hpp"
#include "fusion_radiation.hpp"
#include "point.hpp"

using namespace fusion_radiation;

Points generator(const uint size) {
    Points r;
    r.reserve(size);
    for (size_t i = 0; i < size / 3; i++) {
        r.push_back({{rand() % 100, rand() % 10, 0}});
        r.push_back({{rand() % 10 + 1000, rand() % 100 + 100, 0}});
        r.push_back({{rand() % 10 - 1000, rand() % 100 + 100, 0}});
    }

    return r;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fusion_radiation");
    fusion_radiation::FusionRadiation fr;
    SampleFilter sf;

    // std::vector<fusion_radiation::Point> points = {{{1, 0, 0}},
    //                                                {{2, 0, 0}},
    //                                                {{2.2, 0, 0}},
    //                                                {{0, 1, 0}},
    //                                                {{3, 0, 0}},
    //                                                {{4, 0, 0}},
    //                                                {{5, 0, 0}},
    //                                                {{10, 0, 0}}};

    // points = generator(1000000);
    // sf.setDataset(points);

    // std::vector<Eigen::Vector3d> r1, r2;

    // auto start = std::chrono::high_resolution_clock::now();  // Get the current time
    // sf.estimateManySources(r1);
    // auto end = std::chrono::high_resolution_clock::now();                                // Get the current time
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);  // Calculate the duration in microseconds

    // auto start2 = std::chrono::high_resolution_clock::now();  // Get the current time
    // sf.estimateManySourcesTree(r2);
    // auto end2 = std::chrono::high_resolution_clock::now();                                  // Get the current time
    // auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2);  // Calculate the duration in microseconds

    // std::cout << " first size: " << r1.size() << std::endl;
    // for (const auto& p : r1) {
    //     cout << p.transpose() << endl;
    // }
    // std::cout << " second size: " << r2.size() << std::endl;
    // for (const auto& p : r2) {
    //     cout << p.transpose() << endl;
    // }

    // std::cout << "Execution1 time: " << duration.count() << " microseconds" << std::endl;
    // std::cout << "Execution2 time: " << duration2.count() << " microseconds" << std::endl;
    // // fusion_radiation::KDTree tree(points);

    // fusion_radiation::Point center({0, 0, 0});
    // double distance = 3;
    // vector<ulong> result1, result2;
    // tree.findPointsWithinDistance(center, distance, result1);
    // cout << "distance" << endl;
    // for (const auto& p : result1) {
    //     cout << points[p].toString() << endl;
    // }

    // tree.findNearestPoints(center, 2, result2);
    // cout << "N" << endl;

    // for (const auto& p : result2) {
    //     cout << points[p].toString() << endl;
    // }

    // cout << "Finish" << endl;

    ros::spin();
    return 0;
}
