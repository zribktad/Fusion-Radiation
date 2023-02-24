#ifndef C15ADAC8_29EE_4E1B_AEAC_5CCEF5598AD8
#define C15ADAC8_29EE_4E1B_AEAC_5CCEF5598AD8

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

typedef Point *ref_Point;

class KDTree {
   public:
    explicit KDTree(std::vector<Point> &points);
    ~KDTree();

    void printTree() const;
    void insertPoint(Point& p);
    void findPointsWithinDistance(const  Point &p, const  double distance, vector<ref_Point> &result) const;
    void findNearestPoints(const Point &p, const int n, vector<ref_Point> &nearestpoints) const;
    const bool existsPointWithinDistance(const Point &p, const double distance) const;

   private:
    struct Node;
    Node *root;

    class NearestNeighbor;

    Node *buildTree(std::vector<Point> points, int depth);
    inline void deleteTree(Node *&node);
    inline void insertPointHelp(Node *&node, Point &p, const int depth);
    inline void printTreeHelper(const Node *node, int depth) const;
    inline void findPointsWithinDistanceHelper(const Node *node, const Point &p, const double distance_point, const int depth, vector<ref_Point> &result) const;
    inline void findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth, priority_queue<pair<double, ref_Point>, vector<pair<double, ref_Point>>, NearestNeighbor> &pq) const;
    inline const bool existsPointWithinDistanceHelper(const Node *node, const Point &target, int depth, double dist_comp) const;
};
}  // namespace fusion_radiation

#endif


#endif /* C15ADAC8_29EE_4E1B_AEAC_5CCEF5598AD8 */
