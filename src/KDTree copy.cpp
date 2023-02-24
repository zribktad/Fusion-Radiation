#include "KDTree.hpp"

namespace fusion_radiation {

struct KDTree::Node {
    Point *p;
    Node *left;
    Node *right;

    explicit Node(Point *p) : p(p), left(nullptr), right(nullptr) {}
};

class KDTree::NearestNeighbor {
   public:
    bool operator()(const pair<double, Point*> &a, const pair<double, Point*> &b) const {
        return a.first < b.first;
    }
};

KDTree::KDTree(std::vector<Point> &points) {
    root = buildTree(points, 0);
}

KDTree::~KDTree() {
    deleteTree(root);
}

KDTree::Node *KDTree::buildTree(vector<Point> points, int depth) {
    if (points.empty()) {
        return nullptr;
    }
    auto medianIter = points.begin() + points.size() / 2;
    const int axis = depth % 3;
    nth_element(points.begin(), medianIter, points.end(), [axis](const Point &a, const Point &b) {
        return a[axis] < b[axis];
    });

    Node *node = new Node(&(*medianIter));
    node->left = buildTree(vector<Point>(points.begin(), medianIter), depth + 1);
    node->right = buildTree(vector<Point>(medianIter + 1, points.end()), depth + 1);

    return node;
}

void KDTree::findPointsWithinDistance(const Point &p, const double distance, vector<ref_Point> &result) const {
    findPointsWithinDistanceHelper(root, p, distance, 0, result);
}

inline void KDTree::findPointsWithinDistanceHelper(const Node *node, const Point &p, const double distance_point, const int depth,
                                                   vector<ref_Point> &result) const {
    if (node == nullptr) {
        return;
    }

    const double dist = p.distanceSquared(*node->p);

    if (dist <= distance_point * distance_point) {
        result.push_back(node->p);
    }

    const int axis = depth % 3;
    const double diff = node->p->operator[](axis)- p[axis];

    if (diff >= -distance_point) {
        findPointsWithinDistanceHelper(node->left, p, distance_point, depth + 1, result);
    }
    if (diff <= distance_point) {
        findPointsWithinDistanceHelper(node->right, p, distance_point, depth + 1, result);
    }
}

void KDTree::findNearestPoints(const Point &p, const int n, vector<ref_Point> &nearestPoints) const {
    priority_queue<pair<double, Point*>, vector<pair<double, Point*>>, NearestNeighbor> pq;
    findNearestPointsHelper(root, p, n, 0, pq);
    while (!pq.empty()) {
        nearestPoints.emplace_back(pq.top().second);
        pq.pop();
    }
}

inline void KDTree::findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth,
                                            priority_queue<pair<double, Point*>, vector<pair<double, Point*>>, NearestNeighbor> &pq) const {
    if (node == nullptr) {
        return;
    }
    double dist = p.distanceSquared(*node->p);
    if (pq.size() < n) {
        pq.push({dist, node->p});
    } else if (dist < pq.top().first) {
        pq.pop();
        pq.push({dist, node->p});
    }

    const int axis = depth % 3;
    const double targetCoord = p[axis];
    const double nodeCoord = node->p->operator[](axis);

    const Node *firstChild = node->left;
    const Node *secondChild = node->right;

    if (targetCoord >= nodeCoord) {
        swap(firstChild, secondChild);
    }

    if (firstChild) {
        findNearestPointsHelper(firstChild, p, n, depth + 1, pq);
    }
    if (secondChild && (pq.size() < n || abs(nodeCoord - targetCoord) < pq.top().first)) {
        findNearestPointsHelper(secondChild, p, n, depth + 1, pq);
    }

    // if (targetCoord >= nodeCoord) {
    //      if (secondChild) {
    //         findNearestPointsHelper(secondChild, p, n, depth + 1, pq);
    //     }
    //     if (firstChild && (pq.size() < n || abs(nodeCoord - targetCoord) < pq.top().first)) {
    //         findNearestPointsHelper(firstChild, p, n, depth + 1, pq);
    //     }
    // } else {
    //     if (firstChild) {
    //         findNearestPointsHelper(firstChild, p, n, depth + 1, pq);
    //     }
    //     if (secondChild && (pq.size() < n || abs(nodeCoord - targetCoord) < pq.top().first)) {
    //         findNearestPointsHelper(secondChild, p, n, depth + 1, pq);
    //     }
    // }
}

const bool KDTree::existsPointWithinDistance(const Point &p, const double distance) const {
    return existsPointWithinDistanceHelper(root, p, 0, distance);
}

inline const bool KDTree::existsPointWithinDistanceHelper(const Node *node, const Point &target, int depth, double dist_comp) const {
    if (node == nullptr) {
        return false;
    }

    double dist = target.distance(*node->p);
    if (dist <= dist_comp) {
        return true;
    }

    const int axis = depth % 3;

    if (target[axis] < node->p->operator[](axis)) {
        if (existsPointWithinDistanceHelper(node->left, target, depth + 1, dist_comp)) {
            return true;
        }
        if (target[axis] + dist_comp >= node->p->operator[](axis)) {
            return existsPointWithinDistanceHelper(node->right, target, depth + 1, dist_comp);
        }
    } else {
        if (existsPointWithinDistanceHelper(node->right, target, depth + 1, dist_comp)) {
            return true;
        }
        if (target[axis] - dist_comp <= node->p->operator[](axis)) {
            return existsPointWithinDistanceHelper(node->left, target, depth + 1, dist_comp);
        }
    }

    return false;
}

void KDTree::printTree() const {
    printTreeHelper(root, 0);
}

inline void KDTree::printTreeHelper(const Node *node, int depth) const {
    if (node == nullptr) {
        return;
    }

    printTreeHelper(node->right, depth + 1);
    for (int i = 0; i < depth; i++) {
        cout << "  ";
    }
    cout << "(" << node->p->coord.transpose() << ")" << endl;
    printTreeHelper(node->left, depth + 1);
}

void KDTree::insertPoint(Point &p) {
    insertPointHelp(root, p, 0);
}

inline void KDTree::insertPointHelp(Node *&node, Point &p, const int depth) {
    if (!node) {
        node = new Node(&p);
        return;
    }

    const int curDim = depth % 3;

    if (p[curDim] < node->p->operator[](curDim)) {
        insertPointHelp(node->left, p, depth + 1);
    } else {
        insertPointHelp(node->right, p, depth + 1);
    }
}

inline void KDTree::deleteTree(Node *&node) {
    if (!node) {
        return;
    }

    deleteTree(node->left);
    deleteTree(node->right);
    delete node;
    node = nullptr;
}

}  // namespace fusion_radiation