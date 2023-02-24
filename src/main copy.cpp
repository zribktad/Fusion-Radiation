

#include <vector>

#include "KDTree.hpp"
#include "fusion_radiation.hpp"
#include "point.hpp"

using namespace fusion_radiation;
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fusion_radiation");
    // fusion_radiation::FusionRadiation fr;

    std::vector<fusion_radiation::Point> points = {{{1, 0, 0}},
                                                   {{2, 0, 0}},
                                                   {{2.2, 0, 0}},
                                                   {{0, 1, 0}},
                                                   {{3, 0, 0}},
                                                   {{4, 0, 0}},
                                                   {{5, 0, 0}},
                                                   {{6, 0, 0}}};

    fusion_radiation::KDTree tree(points);

    fusion_radiation::Point center({0, 0, 0});
    double distance = 3;
    vector<fusion_radiation::ref_Point> result1, result2;
    points[2].weight = 1;
    tree.findPointsWithinDistance(center, distance, result1);
    cout << "distance" << endl;
    for (const auto& p : result1) {
        cout << p->toString() << endl;
    }
    result1[2]->weight = 1;

    tree.findNearestPoints(center, 2, result2);
    cout << "N" << endl;

    for (const auto& p : result2) {
        cout << p->toString() << endl;
    }

    std::vector<Point*> pointers;
    pointers.reserve(points.size());
    for (auto& p : points) {
        pointers.push_back(&p);
    }

    auto medianIter = pointers.begin() + pointers.size() / 2;
    const int axis = 0;
    nth_element(pointers.begin(), medianIter, pointers.end(), [axis](const Point* a, const Point* b) {
        return a->operator[](axis) < b->operator[](axis);
    });

    cout << "Finish" << endl;

    ros::spin();
    return 0;
}
