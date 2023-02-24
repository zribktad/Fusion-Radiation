#ifndef FUSION_CONE_H
#define FUSION_CONE_H

#include <rad_msgs/Cone.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace fusion_radiation {

using namespace Eigen;

struct Cone {
    /*base*/
    Vector3d origin;
    Vector3d direction;
    double angle;

    /*calculated*/
    double opposite_hypotenuse; 
    double adjacent_hypotenuse;
    Vector3d direction_norm_side;
    Vector3d u_side;
    Vector3d v_side;

    Cone();
    Cone(Vector3d o, Vector3d d, double a);
    Cone(const rad_msgs::Cone::ConstPtr& msg);
    void calculate_cone();
};
}  // namespace fusion_radiation

#endif
