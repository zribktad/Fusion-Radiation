#include <cone.hpp>
#include <random>

namespace fusion_radiation {

Cone::Cone(Vector3d o, Vector3d d, double a) : origin(o), direction(d), angle(a) {
    calculate_cone();
}

Cone::Cone(const double range) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static  std::uniform_real_distribution<double> coord_dist(-range, range);
    static std::uniform_real_distribution<double> dist(0.0001, M_PI_2 - 0.0001);
    angle = dist(gen);
    origin = {coord_dist(gen),coord_dist(gen),4};
    direction = {coord_dist(gen),coord_dist(gen),-abs(coord_dist(gen))};
    calculate_cone();
}

Cone::Cone(const rad_msgs::Cone::ConstPtr& msg) {
    origin = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    direction = {msg->direction.x, msg->direction.y, msg->direction.z};
    angle = msg->angle;

    calculate_cone();
}

void Cone::calculate_cone() {
    const Vector3d direction_norm = direction.normalized();
    const Vector3d v = Vector3d(direction_norm.y(), -direction_norm.x(), 0).normalized();
    const Vector3d u = v.cross(direction_norm).normalized();
    opposite_hypotenuse = sin(angle);
    adjacent_hypotenuse = cos(angle);

    u_side = opposite_hypotenuse * u;
    v_side = opposite_hypotenuse * v;
    direction_norm_side = adjacent_hypotenuse * direction_norm;
}

}  // namespace fusion_radiation