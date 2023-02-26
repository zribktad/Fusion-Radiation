#include "fusion_radiation.hpp"

#include <mrs_lib/param_loader.h>

#include "image_filter.hpp"

namespace fusion_radiation {

void FusionRadiation::onInit() {
    loadParameters();
    
    initSourcesCallbacks();
    initComptonConeCallBack();
    initOctomapCallBack();
    initCameraCallBacks();

    bv = mrs_lib::BatchVisualizer(n, "markers_visualizer", _uav_name_ + string("/gps_origin"));

    PointVisualzer::init(bv);
    ImageFilter::initImageFilter(n, _uav_name_);
}
inline void FusionRadiation::loadParameters() {
    ROS_INFO("[FusionRadiation]: initializing");

    ros::Time::waitForValid();
    mrs_lib::ParamLoader param_loader(n, "fusion_radiation");
    param_loader.setPrefix("fusion_radiation/");

    param_loader.loadParam("uav_name", _uav_name_);

    FusionRun::loadTesting(param_loader);
    ImageFilter::loadParameters(param_loader);

    if (!param_loader.loadedSuccessfully()) {
        ROS_ERROR("[FusionRadiation]: Could not load all parameters!");
        ros::requestShutdown();
    }
    ROS_INFO("[FusionRadiation: initialized");
}

void FusionRadiation::comptonConeCallBack(const rad_msgs::Cone::ConstPtr& msg) {
    const Cone cone(msg);

    PointVisualzer::clearVisual();
    PointVisualzer::setSourceLocation(radiation_locations);
    PointVisualzer::drawSources();

     FusionRun::generateSample(cone, octree_out);
    ROS_INFO("New compton cone");
}

void FusionRadiation::octomapCallBack(const octomap_msgs::OctomapConstPtr& msg) {
    octomap_msgs::OctomapConstPtr octomap = msg;
    octomap::AbstractOcTree* tree_ptr = nullptr;

    // tree_ptr->clear();
    tree_ptr = octomap_msgs::msgToMap(*octomap);

    if (!tree_ptr) {
        ROS_WARN_THROTTLE(1.0, "[OctomapCeilingRemover]: octomap message is empty!");
        return;
    } else {
        octree_out = OcTreePtr_t(dynamic_cast<octomap::OcTree*>(tree_ptr));
    }
}

void FusionRadiation::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) { camera_model_.fromCameraInfo(*msg); }

void FusionRadiation::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    image_header = msg->header;

    const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);

    cv::Mat image = bridge_image_ptr->image;
    ImageFilter::loadCameraModel(camera_model_);
    ImageFilter::findObjectInImage(image, FusionRun::estimation);

}

void FusionRadiation::setSourceRadiaitonPositionCallBack0(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 0;
    Vector3d pose;

    pose.x() = msg->pose.position.x;
    pose.y() = msg->pose.position.y;
    pose.z() = msg->pose.position.z;

    if (radiation_locations.size() < 1) {
        radiation_locations.resize(1);
    }
    radiation_locations[index] = pose;
}
void FusionRadiation::setSourceRadiaitonPositionCallBack1(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 1;

    if (radiation_locations.size() < 2) {
        radiation_locations.resize(2);
    }
    radiation_locations[index].x() = msg->pose.position.x;
    radiation_locations[index].y() = msg->pose.position.y;
    radiation_locations[index].z() = msg->pose.position.z;
}
void FusionRadiation::setSourceRadiaitonPositionCallBack2(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 2;
    if (radiation_locations.size() < 3) {
        radiation_locations.resize(3);
    }
    radiation_locations[index].x() = msg->pose.position.x;
    radiation_locations[index].y() = msg->pose.position.y;
    radiation_locations[index].z() = msg->pose.position.z;
}

inline void FusionRadiation::initComptonConeCallBack() {
    const string cone_topic = string("/") + _uav_name_ + string("/compton_cone_generator/cones");
    cone_subscriber = n.subscribe(cone_topic, 1, &FusionRadiation::comptonConeCallBack, this);
}

inline void FusionRadiation::initOctomapCallBack() {
    const string octomap_topic = string("/") + _uav_name_ + string("/octomap_server/octomap_global_full");
    octomap_subscriber = n.subscribe(octomap_topic, 1, &FusionRadiation::octomapCallBack, this);
}

inline void FusionRadiation::initCameraCallBacks() {
    const string image_topic = string("/") + _uav_name_ + string("/mobius_front/image_raw");
    const string camera_info_topic = string("/") + _uav_name_ + string("/mobius_front/camera_info");
    camera_image_sub = n.subscribe(image_topic, 1, &FusionRadiation::imageCallback, this);
    camera_info_sub = n.subscribe(camera_info_topic, 1, &FusionRadiation::cameraInfoCallback, this);
}

inline void FusionRadiation::initSourcesCallbacks() {
    radiation_locations.reserve(3);
    {
        ros::Subscriber source_subcriber = n.subscribe("/gazebo/cs137_100GBq/source_gt", 10, &FusionRadiation::setSourceRadiaitonPositionCallBack0, this);
        if (source_subcriber) {
            source_subcribers.emplace_back(source_subcriber);
        }
    }
    {
        ros::Subscriber source_subcriber = n.subscribe("/gazebo/cs137_100GBq_0/source_gt", 10, &FusionRadiation::setSourceRadiaitonPositionCallBack1, this);
        if (source_subcriber) {
            source_subcribers.emplace_back(source_subcriber);
        }
    }
    {
        ros::Subscriber source_subcriber = n.subscribe("/gazebo/cs137_100GBq_1/source_gt", 10, &FusionRadiation::setSourceRadiaitonPositionCallBack2, this);
        if (source_subcriber) {
            source_subcribers.emplace_back(source_subcriber);
        }
    }
}
}  // namespace fusion_radiation