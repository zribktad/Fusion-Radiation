/**
 * @file point.hpp
 * @author Tadeas Zribko
 * @brief Header file for the Point class
 * This file contains the definition of the Point class, which represents a point in 3D space. It provides methods for calculating the distance between points, accessing and modifying  the coordinates of a point, and printing a point's information as a string.
 * @version 0.1
 * @date 2023-04-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef FUSION_POINT
#define FUSION_POINT

#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>

using namespace Eigen;

namespace fusion_radiation {

/**
 *   @brief The Point class represents a point in 3D space.
 */
struct Point {
    Vector3d coord;
    double weight;
    ulong coeff;
    long cone_id;
    /*
        @brief Default constructor.
        Initializes the coordinate of the point to (0, 0, 0) and weight to INFINITY.
        The coefficient and cone id of the point are initialized to 0 and -1 respectively.
        */
    Point();
    /**

    @brief Constructor.
    @param[in] coord The coordinate of the point.
    Initializes the weight to INFINITY and coefficient and cone id to 0 and -1 respectively.
    */
    Point(Vector3d coord);
    /**

    @brief Constructor.
    @param[in] coord The coordinate of the point.
    @param[in] cone_id The cone id of the point.
    Initializes the weight to INFINITY and coefficient to 0.
    */
    Point(Vector3d coord, ulong cone_id);
    /**

    @brief Constructor.
    @param[in] coord The coordinate of the point.
    @param[in] weight The weight of the point.
    @param[in] cone_id The cone id of the point.
    Initializes the coefficient to 0.
    */
    Point(Vector3d coord, double weight, ulong cone_id);
    /**
        @brief Less than operator.
        @param[in] rhs The right-hand side Point object.
        @return True if the weight of the current point is less than the weight of the right-hand side point.
        */
    bool operator<(const Point &rhs) const;
    /**

    @brief Equality operator.
    @param[in] rhs The right-hand side Point object.
    @return True if the coordinate of the current point is equal to the coordinate of the right-hand side point.
    */
    bool operator==(const Point &rhs) const;
    /**

    @brief Inequality operator.
    @param[in] rhs The right-hand side Point object.
    @return True if the coordinate of the current point is not equal to the coordinate of the right-hand side point.
    */
    bool operator!=(const Point &rhs) const;
    /**

    @brief Index operator.
    @param[in] i The index of the coordinate.
    @return The reference to the coordinate value at the given index.
    */
    double &operator[](int i);
    /**

    @brief Const index operator.
    @param[in] i The index of the coordinate.
    @return The const reference to the coordinate value at the given index.
    */
    const double &operator[](int i) const;
    /**

        @brief Writes a vector of points to the console using ROS_INFO_STREAM.
        @param points A constant reference to a vector of Point objects to be written.
        */
    static void writePoints(const std::vector<Point> &points);
    /**

    @brief Calculates the Euclidean distance between two points.
    @param[in] rhs The right-hand side Point object.
    @return The Euclidean distance between the current point and the right-hand side point.
    */
    double distance(const Point &rhs) const;

    /**
    @brief Calculates the squared Euclidean distance between two points.
    @param[in] rhs The right-hand side Point object.
    @return The squared Euclidean distance between the current point and the right-hand side point.
    */
    double distanceSquared(const Point &rhs) const;
    /**

    @brief Returns a string representation of the Point object.
    @return A string containing the coordinates, weight, code coefficient, and cone ID of the Point object.
    */
    const std::string toString() const;
};
typedef std::vector<Point> Points;

}  // namespace fusion_radiation

#endif
