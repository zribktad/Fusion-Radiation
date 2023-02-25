#include "fusion_radiation.hpp"

#include <mrs_lib/param_loader.h>

namespace fusion_radiation {

void FusionRadiation::onInit() {
    loadParameters();
    initSourcesCallbacks();
    initComptonConeCallBack();
    initOctomapCallBack();
    initCameraCallBacks();

    // transformer
    image_transport::ImageTransport it(n);
    transformer_ = std::make_unique<mrs_lib::Transformer>("fusion_radiation");
    transformer_->setDefaultPrefix(_uav_name_);
    transformer_->retryLookupNewest(true);

    bv = mrs_lib::BatchVisualizer(n, "markers_visualizer", _uav_name_ + string("/gps_origin"));

    PointVisualzer::init(bv);
}
inline void FusionRadiation::loadParameters() {
    ROS_INFO("[FusionRadiation]: initializing");

    ros::Time::waitForValid();
    mrs_lib::ParamLoader param_loader(n, "fusion_radiation");
    param_loader.setPrefix("fusion_radiation/");

    param_loader.loadParam("uav_name", _uav_name_);

    FusionRun::loadTesting(param_loader);

    if (!param_loader.loadedSuccessfully()) {
        ROS_ERROR("[FusionRadiation]: Could not load all parameters!");
        ros::requestShutdown();
    }
    ROS_INFO("[FusionRadiation: initialized");

    cout << _uav_name_ << endl;
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
    //? if (!estimates.empty()) {

    const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);

    cv::Mat image = bridge_image_ptr->image;
    //! auto estimates = TestingAndUsing::getEstimates();
    //! ImageWork::findObjectInImage(image, estimates, camera_model_, uav_name, transformer_);
    // ImageWork::publishImage(image,image_header, color_encoding, image_pub);
}

void FusionRadiation::setSourceRadiaitonPositionCallBack0(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 0;
    Vector3d pose;
    
    pose.x() = msg->pose.position.x;
    pose.y() = msg->pose.position.y;
    pose.z() = msg->pose.position.z;

    if(radiation_locations.size()<1){
       radiation_locations.resize(1);
    }
    radiation_locations[index] = pose;

    // cout << "Source position num " << index << " : " << radiation_locations[index].transpose() << endl;
}
void FusionRadiation::setSourceRadiaitonPositionCallBack1(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 1;

     if(radiation_locations.size()<2){
       radiation_locations.resize(2);
    }
    radiation_locations[index].x() = msg->pose.position.x;
    radiation_locations[index].y() = msg->pose.position.y;
    radiation_locations[index].z() = msg->pose.position.z;
   
    //  cout << "Source position num " << index << " : " << radiation_locations[index].transpose() << endl;
}
void FusionRadiation::setSourceRadiaitonPositionCallBack2(const geometry_msgs::PoseStampedConstPtr& msg) {
    const int index = 2;
     if(radiation_locations.size()<3){
       radiation_locations.resize(3);
    }
    radiation_locations[index].x() = msg->pose.position.x;
    radiation_locations[index].y() = msg->pose.position.y;
    radiation_locations[index].z() = msg->pose.position.z;
    // cout << "Source position num " << index << " : " << radiation_locations[index].transpose() << endl;
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
    camera_image_sub = n.subscribe(image_topic, 1, &FusionRadiation::cameraInfoCallback, this);
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
    // ros::Subscriber cone_sub = n.subscribe(cone_topic, 1, newConeCallback);

    // ros::Subscriber  source_sub = n.subscribe("/gazebo/cs137_100GBq/source_gt", 10, &FusionRadiation::setSourceRadiaitonPositionCallBack,this);

    // uint number_sources_ = 2;
    // for (int i = 0;; i++) {
    //     cout << "Sorces" << endl;
    //     radiation_locations.emplace_back(Vector3d(0));
    //     const string to_name = ((i == 0) ? "" : ("_" + std::to_string(i - 1))) + "/source_gt";
    //     ros::TransportHints transport_hints;
    //     ros::Subscriber source_subcriber = n.subscribe("/gazebo/cs137_100GBq" + to_name, 10, &FusionRadiation::setSourceRadiaitonPositionCallBack0, this);
    //     cout << "Sub "
    //          << "/gazebo/cs137_100GBq" << to_name << "  " << source_subcriber << endl;
    //     if (!source_subcriber) {
    //         source_subcriber = n.subscribe("/gazebo/cs137_5000GBq/" + to_name, 10, &FusionRadiation::setSourceRadiaitonPositionCallBack0, this);
    //     }

    //     if (!source_subcriber) {
    //         radiation_locations.pop_back();
    //         break;
    //     }

    //     source_subcribers.emplace_back(source_subcriber);

    //     if (i > 2) {
    //         break;
    //     }
    // }
}
}  // namespace fusion_radiation