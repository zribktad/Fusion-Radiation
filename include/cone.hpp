/**

    @file cone.h
    @brief This file contains the Cone class and its methods.
    */

#ifndef FUSION_CONE_H
#define FUSION_CONE_H

#include <rad_msgs/Cone.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sstream>

namespace fusion_radiation {

using namespace Eigen;
/**
@class Cone
@brief This class represents a cone object with calcualtions.
 */
struct Cone {
    /*base*/
    Vector3d origin;    /**< origin vector */
    Vector3d direction; /**< direction vector */
    double angle;       /**< angle in radians */

    /*calculated*/
    double opposite_hypotenuse;   /**< opposite hypotenuse */
    double adjacent_hypotenuse;   /**< adjacent hypotenuse */
    Vector3d direction_norm_side; /**< normalized direction side vector */
    Vector3d u_side;              /**< u side vector */
    Vector3d v_side;              /**< v side vector */

    Cone();
    /**

    @brief Constructor for Cone class with random values
    @param range the range of the random values
    */
    Cone(const double range);
    /*

    @brief Constructor for Cone class
    @param o origin vector
    @param d direction vector
    @param a angle in radians
    */
    Cone(Vector3d o, Vector3d d, double a);
    /**

    @brief Constructor for Cone class with ROS message
    @param msg ROS message
    */
    Cone(const rad_msgs::Cone::ConstPtr& msg);
    /**

    @brief Returns a string representation of the cone object
    @return string representation of the cone object
    */
    std::string toString() const;
    /**
    Calculates the sides of the cone based on the angle and direction.
    */
    void calculate_cone();
};
}  // namespace fusion_radiation

#endif
