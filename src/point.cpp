#include <point.hpp>
#include <string>

namespace fusion_radiation {

Point::Point() : coord({0, 0, 0}), weight(INFINITY), cone_id(-1) {
}

Point::Point(Vector3d coord) : coord(coord), weight(INFINITY), cone_id(-1) {
}

Point::Point(Vector3d coord, long cone_id) : coord(coord), weight(INFINITY), cone_id(cone_id) {
}

Point::Point(Vector3d coord, double weight, long cone_id) : coord(coord), weight(weight), cone_id(cone_id) {
}

bool Point::operator<(const Point &rhs) const {
    return rhs.weight < weight;
}

bool Point::operator==(const Point &rhs) const {
    return rhs.coord == coord;
}

bool Point::operator!=(const Point &rhs) const {
    return rhs.coord != coord;
}

double &Point::operator[](int i) {
    return coord[i];
}

const double &Point::operator[](int i) const {
    return coord[i];
}

double Point::distance(const Point &rhs) const {
    return sqrt(pow(rhs.coord.x() - this->coord.x(), 2) +
                pow(rhs.coord.y() - this->coord.y(), 2) +
                pow(rhs.coord.z() - this->coord.z(), 2));
}

double Point::distanceSquared(const Point &rhs) const {
    return pow(rhs.coord.x() - this->coord.x(), 2) +
           pow(rhs.coord.y() - this->coord.y(), 2) +
           pow(rhs.coord.z() - this->coord.z(), 2);
}

const std::string Point::toString() const {
    std::ostringstream ss;
    ss << "Point coord: (" << coord.x() << ", " << coord.y() << ", " << coord.z() << ") , weight: " << weight << " , cone id: " << cone_id;
    std::string s =ss.str();
    return s;
}

}  // namespace fusion_radiation