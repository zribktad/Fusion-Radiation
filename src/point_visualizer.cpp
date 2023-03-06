#include "point_visualizer.hpp"

namespace fusion_radiation {

void PointVisualzer::init(mrs_lib::BatchVisualizer &bv) {
    PointVisualzer::bv = bv;
}

void PointVisualzer::setSourceLocation(map<int, Vector3d> &locations) {
    PointVisualzer::radiation_locations = locations;
}

void PointVisualzer::clearVisual() {
    bv.clearBuffers();
    bv.clearVisuals();
}

void PointVisualzer::drawPoints(const Points &points, const Vector4f &color, const double scale) {
    bv.setPointsScale(scale);
    for (const Point &point : points) {
        bv.addPoint(point.coord, color.x(), color.y(), color.z(), color.w());
    }
    bv.publish();
    ros::spinOnce();
}
void PointVisualzer::drawPoints(const vector<Vector3d> &points, const Vector4f &color, const double scale) {
    bv.setPointsScale(scale);
    for (const Vector3d &point : points) {
        bv.addPoint(point, color.x(), color.y(), color.z(), color.w());
    }
    bv.publish();
    ros::spinOnce();
}

void PointVisualzer::drawPoints(const Points &points, const Vector4f &color, const ulong size, const double scale) {
    bv.setPointsScale(scale);
    const ulong limit = points.size() < size ? points.size() : size;

    for (ulong i = 0; i < limit; i++) {
        const Point &p = points[i];
        bv.addPoint(p.coord, color.x(), color.y(), color.z(), color.w());
    }

    bv.publish();
    ros::spinOnce();
}

void PointVisualzer::drawSources(const double scale) {
    bv.setPointsScale(scale);
    // drawPoints(radiation_locations, {1, 0, 0, 1});
    map<int, Vector3d>::iterator itr;
    for (itr = radiation_locations.begin(); itr != radiation_locations.end(); ++itr) {
        const auto &p = itr->second;
        bv.addPoint(p, 1, 0, 0, 1);
    }

    bv.publish();
    ros::spinOnce();

}  // namespace fusion_radiation
}