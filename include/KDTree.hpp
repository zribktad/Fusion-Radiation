/**
 * @file KDTree.hpp
 *
 * @brief Contains the KDTree class for performing nearest neighbor searches.
 *
 * This file contains the KDTree class for performing nearest neighbor searches
 * in three-dimensional space. The KDTree is constructed from a vector of Point
 * objects, where each Point object is a tuple of three double values.
 */

#ifndef KDTREE_H
#define KDTREE_H

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <queue>
#include <vector>

#include "point.hpp"

namespace fusion_radiation {

using namespace Eigen;
using namespace std;

/**
 * @class KDTree
 *
 * @brief A KDTree for performing nearest neighbor searches.
 *
 * This class is a KDTree for performing nearest neighbor searches in
 * three-dimensional space. The KDTree is constructed from a vector of Point
 * objects, where each Point object is a tuple of three double values.
 */
class KDTree {
   public:
    /**
     * @brief Construct a KDTree from a vector of points.
     *
     * This constructor constructs a KDTree from a vector of Point objects, where
     * each Point object is a tuple of three double values.
     *
     * @param points A vector of Point objects to build the KDTree from.
     */
    explicit KDTree(std::vector<Point> &points);
    /**
     * @brief Destroy the KDTree object and free its memory.
     *
     * This destructor destroys the KDTree object and frees its memory.
     */
    ~KDTree();
    /**
     *
     * @brief Print the tree.
     */
    void printTree() const;
    /**
     *
     *  @brief Inserts a point into the KDTree.
     *  @param p The point to insert.
     *
     */
    void insertPoint(const Point &p);
    /**
     * @brief Find points within a certain distance of a target point.
     *
     * This method finds all points in the KDTree that are within a certain
     * Euclidean distance of a target point. The indices of the points are stored
     * in the result vector.
     *
     * @param p The target point.
     * @param distance The maximum Euclidean distance from the target point to
     * any of the points in the result set.
     * @param result The vector to store the indices of the points in that are
     * within the specified distance of the target point.
     */
    void findPointsWithinDistance(const Point &p, const double distance, vector<ulong> &result) const;
    /**
     * @brief Find the nearest n points to a target point.
     *
     * This method finds the nearest n points in the KDTree to a target point.
     * The indices of the points are stored in the nearestPoints vector.
     *
     * @param p The target point.
     * @param n The number of nearest points to find.
     * @param nearestPoints The vector to store the indices of the nearest
     * points in.
     */
    void findNearestPoints(const Point &p, const int n, vector<ulong> &nearestpoints) const;
    /**
     * @brief Find the nearest n points to a target point, along with their distances.
     *
     * This method finds the nearest n points in the KDTree to a target point,
     * along with their distances. The indices of the points are stored in the
     * nearestPoints vector, and the distances are stored in the distances
     * vector.
     *
     * @param p The target point.
     * @param n The number of nearest points to find.
     * @param nearestPoints The vector to store the indices of the nearest
     * points in.
     * @param distances The vector to store the distances from the target point
     * to the nearest points in.
     */
    void findNearestPoints(const Point &p, const int n, vector<ulong> &nearestpoints, vector<double> &distances) const;
    /**
     * @brief Finds all points within a certain distance of a given point.
     * @param p The point to search around.
     * @param distance The distance to search within.
     * @return True if there exists a point within the given distance of @p p.
     */
    const bool existsPointWithinDistance(const Point &p, const double distance) const;

   private:
    /**
     * @brief A node in the KDTree.
     */
    struct Node;
    Node *root;
    /**
     * @brief A comparator class for sorting nearest neighbors by distance.
     */
    class NearestNeighbor;
    /**
     * @brief Builds a KDTree from a vector of nodes.
     * @param nodes The vector of nodes to build the tree from.
     * @param depth The depth of the current node in the tree.
     * @return The root node of the built tree.
     */
    Node *buildTree(std::vector<Node *> &nodes, int depth);
    /**
     *
     *  @brief Deletes the entire KDTree.
     * @param node Pointer to the root node of the KDTree.
     */
    inline void deleteTree(Node *&node);
    /**
     * @brief Recursively inserts a point into the KDTree.
     * @param node Pointer to the current node in the KDTree.
     * @param p The point to insert.
     * @param depth The current depth of the node in the KDTree.
     */
    inline void insertPointHelp(Node *&node, const Point &p, const int depth);
    /**
     * @brief Recursively prints the points in the KDTree in a tree-like format.
     * @param node Pointer to the current node in the KDTree.
     * @param depth The current depth of the node in the KDTree.
     */
    inline void printTreeHelper(const Node *node, int depth) const;
    /**
     * @brief Finds all points within a certain distance of a given point.
     * @param node The current node being searched.
     * @param p The point to search around.
     * @param distance_point The distance to search within.
     * @param depth The depth of the current node in the tree.
     * @param result The vector to store the indices of the found points in.
     */
    inline void findPointsWithinDistanceHelper(const Node *node, const Point &p, const double distance_point, const int depth, vector<ulong> &result) const;
    /**
     * @brief Finds the nearest `n` points to point `p` in the KDTree.
     *
     * @param node The node to search from.
     * @param p The point to find the nearest neighbors to.
     * @param n The number of nearest neighbors to find.
     * @param depth The depth of the current node in the tree.
     * @param pq The priority queue of nearest neighbors.
     */
    inline void findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth, priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq) const;

    /**
     * @brief Finds the nearest `n` points to point `p` in the KDTree.
     *
     * @param node The node to search from.
     * @param p The point to find the nearest neighbors to.
     * @param n The number of nearest neighbors to find.
     * @param depth The depth of the current node in the tree.
     * @param pq The priority queue of nearest neighbors.
     * @param distances The priority queue of distances to nearest neighbors.
     */
    inline void findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth, priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq, priority_queue<double> &distances) const;
    /**
     * @brief Finds all points within a certain distance of a given point.
     * @param node The current node being searched.
     * @param target The point to search around.
     * @param depth The depth of the current node in the tree.
     * @param dist_comp The distance to search within.
     * @return True if there exists a point within the given distance of @p target.
     */
    inline const bool existsPointWithinDistanceHelper(const Node *node, const Point &target, int depth, double dist_comp) const;
};
}  // namespace fusion_radiation

#endif
