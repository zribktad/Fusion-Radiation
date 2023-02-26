

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
        r.push_back({{(double)(rand() % 100), (double)(rand() % 10), 0}});
        r.push_back({{(double)(rand() % 10 + 1000), (double)(rand() % 100 + 100), 0}});
        r.push_back({{(double)(rand() % 10 - 1000), (double)(rand() % 100 + 100), 0}});
    }

    return r;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fusion_radiation");
     fusion_radiation::FusionRadiation fr;
    // SampleFilter sf, sf2;

    // std::vector<fusion_radiation::Point> points = {{{1, 0, 0}},
    //                                                {{2, 0, 0}},
    //                                                {{2.2, 0, 0}},
    //                                                {{0, 1, 0}},
    //                                                {{3, 0, 0}},
    //                                                {{4, 0, 0}},
    //                                                {{5, 0, 0}},
    //                                                {{10, 0, 0}}};
    // std::vector<fusion_radiation::Point> points2;

    // points2 = generator(100);
    // sf.setDataset(points);
    // sf2.setDataset(points);

    // double c1 = 0, c2 = 0;

    // for (size_t i = 0; i < 150; i++) {
    //     points = generator(50);
    //     std::vector<Eigen::Vector3d> r1, r2;
    //     std::vector<fusion_radiation::Point> d1(points);
    //     std::vector<fusion_radiation::Point> d2(points);
    //     if (i % 2) {
    //         auto start = std::chrono::high_resolution_clock::now();  // Get the current time
    //         sf.CicleFilter(d1);
    //         auto end = std::chrono::high_resolution_clock::now();                              // Get the current time
    //         c1 += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();  // Calculate the duration in microseconds

    //         auto start2 = std::chrono::high_resolution_clock::now();  // Get the current time
    //         sf2.CicleFilter2(d2);
    //         auto end2 = std::chrono::high_resolution_clock::now();                               // Get the current time
    //         c2 += std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();  // Calculate the duration in microseconds
    //     } else {
    //         auto start2 = std::chrono::high_resolution_clock::now();  // Get the current time
    //         sf.CicleFilter(d2);
    //         auto end2 = std::chrono::high_resolution_clock::now();                               // Get the current time
    //         c1 += std::chrono::duration_cast<std::chrono::microseconds>(end2 - start2).count();  // Calculate the duration in microseconds
    //         auto start = std::chrono::high_resolution_clock::now();                              // Get the current time

    //         sf2.CicleFilter2(d1);
    //         auto end = std::chrono::high_resolution_clock::now();                              // Get the current time
    //         c2 += std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();  // Calculate the duration in microseconds
    //     }
    // }

    // std::cout << "Execution1 time: " << c1 << " microseconds" << std::endl;
    // std::cout << "Execution2 time: " << c2 << " microseconds" << std::endl;
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
