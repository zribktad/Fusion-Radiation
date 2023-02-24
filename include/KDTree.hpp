
#ifndef KDTREE_H
#define KDTREE_H

#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <queue>
#include <vector>
#include <functional>

#include "point.hpp"



namespace fusion_radiation {
    
using namespace Eigen;
using namespace std;


class KDTree {
   public:
    explicit KDTree(std::vector<Point> &points);
    ~KDTree();

    void printTree() const;
    void insertPoint(const Point &p);
    void findPointsWithinDistance(const  Point &p, const  double distance, vector<ulong> &result) const;
    void findNearestPoints(const Point &p, const int n, vector<ulong> &nearestpoints) const;
    void findNearestPoints(const Point &p, const int n, vector<ulong> &nearestpoints, vector<double> & distances) const;
    const bool existsPointWithinDistance(const Point &p, const double distance) const;

   private:
    struct Node;
    Node *root;

    class NearestNeighbor;

    Node *buildTree(std::vector<Node*> &nodes, int depth);
    inline void deleteTree(Node *&node);
    inline void insertPointHelp(Node *&node, const Point &p, const int depth);
    inline void printTreeHelper(const Node *node, int depth) const;
    inline void findPointsWithinDistanceHelper(const Node *node, const Point &p, const double distance_point, const int depth, vector<ulong> &result) const;
    inline void findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth, priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq) const;
    inline void findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth, priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq,priority_queue<double>& distances) const;
    inline const bool existsPointWithinDistanceHelper(const Node *node, const Point &target, int depth, double dist_comp) const;
};
}  // namespace fusion_radiation

#endif
