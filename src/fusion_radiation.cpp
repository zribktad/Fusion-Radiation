#include "fusion_radiation.hpp"

#include <mrs_lib/param_loader.h>

#include "image_filter.hpp"
#include "fusion_test.hpp"



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
    // FusionTest::timeCompareSampler();
}
inline void FusionRadiation::loadParameters() {
    ROS_INFO("[FusionRadiation]: initializing");
    ros::Time::waitForValid();
    mrs_lib::ParamLoader param_loader(n, "fusion_radiation");
    param_loader.setPrefix("fusion_radiation/");
    param_loader.loadParam("uav_name", _uav_name_);
    FusionRun::loadParameters(param_loader);
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
    PointVisualzer::setSourceLocation(radiation_sources);
    PointVisualzer::drawSources();
    FusionRun::processData(cone, octree_out);
    ROS_INFO("New compton cone");
}

void FusionRadiation::octomapCallBack(const octomap_msgs::OctomapConstPtr& msg) {
      std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));

    if (tree_ptr) {
        octree_out = OcTreePtr_t(dynamic_cast<octomap::OcTree*>(tree_ptr.release()));
    } else {
        ROS_WARN_THROTTLE(1.0, "[OctomapCeilingRemover]: octomap message is empty!");
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


 void FusionRadiation::initComptonConeCallBack() {
    const string cone_topic = string("/") + _uav_name_ + string("/compton_cone_generator/cones");
    cone_subscriber = n.subscribe(cone_topic, 1, &FusionRadiation::comptonConeCallBack, this);
}

 void FusionRadiation::initOctomapCallBack() {
    const string octomap_topic = string("/") + _uav_name_ + string("/octomap_server/octomap_global_full");
    octomap_subscriber = n.subscribe(octomap_topic, 1, &FusionRadiation::octomapCallBack, this);
}

 void FusionRadiation::initCameraCallBacks() {
    const string image_topic = string("/") + _uav_name_ + string("/mobius_front/image_raw");
    const string camera_info_topic = string("/") + _uav_name_ + string("/mobius_front/camera_info");
    camera_image_sub = n.subscribe(image_topic, 1, &FusionRadiation::imageCallback, this);
    camera_info_sub = n.subscribe(camera_info_topic, 1, &FusionRadiation::cameraInfoCallback, this);
}


void FusionRadiation::setSourceRadiationPositionCallback(const gazebo_rad_msgs::RadiationSourceConstPtr& msg) {
    radiation_sources[msg->id]=Vector3d{msg->world_pos.x,msg->world_pos.y,msg->world_pos.z};
}

void FusionRadiation::initSourcesCallbacks() {
    //radiation_locations.reserve(3);

    source_subscriber = n.subscribe<gazebo_rad_msgs::RadiationSource>("/radiation/sources", 10,&FusionRadiation::setSourceRadiationPositionCallback, this);
}


}  // namespace fusion_radiation
