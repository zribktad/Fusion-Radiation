#ifndef FUSION_POINT
#define FUSION_POINT

#include <eigen3/Eigen/Core>
#include <vector>

using namespace Eigen;

namespace fusion_radiation {

struct Point {
    Vector3d coord;
    double weight;
    long cone_id;

    Point();
    Point(Vector3d coord);
    Point(Vector3d coord, long cone_id);
    Point(Vector3d coord, double weight, long cone_id);

    bool operator<(const Point &rhs) const;
    bool operator==(const Point &rhs) const;
    bool operator!=(const Point &rhs) const;
    double &operator[](int i);
    const double &operator[](int i) const;

    

    double distance(const Point &rhs) const;

    double distanceSquared(const Point &rhs) const;
    const std::string toString() const;
};
typedef std::vector<Point> Points;

}  // namespace fusion_radiation

#endif
