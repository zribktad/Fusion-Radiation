#include "KDTree.hpp"

namespace fusion_radiation {

struct KDTree::Node {
    Point p;
    ulong id;
    Node *left;
    Node *right;

    explicit Node(const Point &p) : p(p), left(nullptr), right(nullptr) {}
    explicit Node(const Point &p, const ulong &id) : p(p), left(nullptr), right(nullptr), id(id) {}
};

class KDTree::NearestNeighbor {
   public:
    bool operator()(const pair<double, ulong> &a, const pair<double, ulong> &b) const {
        return a.first < b.first;
    }
};

KDTree::KDTree(std::vector<Point> &points) {
    vector<Node *> nodes;
    const ulong size_array = points.size();
    nodes.reserve(size_array);

    for (const auto &p : points) {
        nodes.push_back(new Node(p, nodes.size()));
    }

    root = buildTree(nodes, 0);
}

KDTree::~KDTree() {
    deleteTree(root);
}

KDTree::Node *KDTree::buildTree(vector<Node *> &nodes, int depth) {
    if (nodes.empty()) {
        return nullptr;
    }
    auto medianIter = nodes.begin() + nodes.size() / 2;
    const int axis = depth % 3;
    nth_element(nodes.begin(), medianIter, nodes.end(), [axis](const Node *a, const Node *b) {
        return a->p[axis] < b->p[axis];
    });

    Node *node = *medianIter;
    vector<Node *> nodesCopy(nodes.begin(), medianIter);
    node->left = buildTree(nodesCopy, depth + 1);

    vector<Node *> medianIterCopy(medianIter + 1, nodes.end());
    node->right = buildTree(medianIterCopy, depth + 1);

    return node;
}

void KDTree::findPointsWithinDistance(const Point &p, const double distance, vector<ulong> &result) const {
    findPointsWithinDistanceHelper(root, p, distance, 0, result);
}

inline void KDTree::findPointsWithinDistanceHelper(const Node *node, const Point &p, const double distance_point, const int depth,
                                                   vector<ulong> &result) const {
    if (node == nullptr) {
        return;
    }

    const double dist = p.distanceSquared(node->p);

    if (dist <= distance_point * distance_point) {
        result.emplace_back(node->id);
    }

    const int axis = depth % 3;
    const double diff = node->p[axis] - p[axis];

    if (diff >= -distance_point) {
        findPointsWithinDistanceHelper(node->left, p, distance_point, depth + 1, result);
    }
    if (diff <= distance_point) {
        findPointsWithinDistanceHelper(node->right, p, distance_point, depth + 1, result);
    }
}

void KDTree::findNearestPoints(const Point &p, const int n, vector<ulong> &nearestPoints) const {
    priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> pq;
    findNearestPointsHelper(root, p, n, 0, pq);
    while (!pq.empty()) {
        nearestPoints.emplace_back(pq.top().second);
        pq.pop();
    }
}

void KDTree::findNearestPoints(const Point &p, const int n, vector<ulong> &nearestPoints, vector<double> &distances) const {
    priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> pq;
    priority_queue<double> dq;

    findNearestPointsHelper(root, p, n, 0, pq, dq);
    nearestPoints.reserve(pq.size());
    distances.reserve(dq.size());
    while (!pq.empty()) {
        nearestPoints.emplace_back(pq.top().second);
        pq.pop();
        distances.emplace_back(dq.top());
        dq.pop();
    }
}

inline void KDTree::findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth,
                                            priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq, priority_queue<double> &dq) const {
    if (node == nullptr) {
        return;
    }
    double dist = p.distanceSquared(node->p);
    if (pq.size() < n) {
        pq.push({dist, node->id});
        dq.emplace(dist);
    } else if (dist < pq.top().first) {
        pq.pop();
        dq.pop();
        pq.push({dist, node->id});
        dq.emplace(dist);
    }

    const int axis = depth % 3;
    const double targetCoord = p[axis];
    const double nodeCoord = node->p[axis];

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
}

inline void KDTree::findNearestPointsHelper(const Node *node, const Point &p, const int n, const int depth,
                                            priority_queue<pair<double, ulong>, vector<pair<double, ulong>>, NearestNeighbor> &pq) const {
    if (node == nullptr) {
        return;
    }
    double dist = p.distanceSquared(node->p);
    if (pq.size() < n) {
        pq.push({dist, node->id});
    } else if (dist < pq.top().first) {
        pq.pop();
        pq.push({dist, node->id});
    }

    const int axis = depth % 3;
    const double targetCoord = p[axis];
    const double nodeCoord = node->p[axis];

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
}

const bool KDTree::existsPointWithinDistance(const Point &p, const double distance) const {
    return existsPointWithinDistanceHelper(root, p, 0, distance);
}

inline const bool KDTree::existsPointWithinDistanceHelper(const Node *node, const Point &target, int depth, double dist_comp) const {
    if (node == nullptr) {
        return false;
    }

    double dist = target.distance(node->p);
    if (dist <= dist_comp) {
        return true;
    }

    const int axis = depth % 3;

    if (target[axis] < node->p[axis]) {
        if (existsPointWithinDistanceHelper(node->left, target, depth + 1, dist_comp)) {
            return true;
        }
        if (target[axis] + dist_comp >= node->p[axis]) {
            return existsPointWithinDistanceHelper(node->right, target, depth + 1, dist_comp);
        }
    } else {
        if (existsPointWithinDistanceHelper(node->right, target, depth + 1, dist_comp)) {
            return true;
        }
        if (target[axis] - dist_comp <= node->p[axis]) {
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
    cout << "(" << node->p.coord.transpose() << ")" << endl;
    printTreeHelper(node->left, depth + 1);
}

void KDTree::insertPoint(const Point &p) {
    insertPointHelp(root, p, 0);
}

inline void KDTree::insertPointHelp(Node *&node, const Point &p, const int depth) {
    if (!node) {
        node = new Node(p);
        return;
    }

    const int curDim = depth % 3;

    if (p[curDim] < node->p[curDim]) {
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