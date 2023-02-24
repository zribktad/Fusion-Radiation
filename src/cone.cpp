#include <cone.hpp>

namespace fusion_radiation {

Cone::Cone(Vector3d o, Vector3d d, double a) : origin(o), direction(d), angle(a) {
    calculate_cone();
}

Cone::Cone(const rad_msgs::Cone::ConstPtr& msg) {
    origin = {msg->pose.position.x, msg->pose.position.x, msg->pose.position.x};
    direction = {msg->direction.x, msg->direction.y, msg->direction.z};
    angle = msg->angle;

    calculate_cone();
}

void Cone::calculate_cone(){
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