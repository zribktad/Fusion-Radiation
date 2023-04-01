#include "fusion_radiation.hpp"

#include <mrs_lib/param_loader.h>

#include "fusion_test.hpp"
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
    if (is_camera_GUI_active) {
        ImageFilter::initImageFilter(n, _uav_name_);
    }
    ros::ServiceServer service_state = n.advertiseService("change_estimation_state", changeEstimationState);
    ros::ServiceServer service_filter_params = n.advertiseService("filter_params", setFilterParams);
    ros::ServiceServer service_estimation = n.advertiseService("estimation_params", setEstimationParams);

    ros::spin();
    ROS_INFO("[FusionRadiation: initialized");
    // FusionTest::timeCompareSampler();
}
inline void FusionRadiation::loadParameters() {
    ROS_INFO("[FusionRadiation]: initializing");
    // ros::Time::waitForValid();
    mrs_lib::ParamLoader param_loader(n, "fusion_radiation");
    param_loader.setPrefix("fusion_radiation/");
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("sample_filter/draw_limit_dataset", draw_limit_dataset);
    param_loader.loadParam("estimation_active", is_active);
    param_loader.loadParam("camera_process", is_camera_active);
    param_loader.loadParam("camera_GUI", is_camera_GUI_active);
    param_loader.loadParam("octomap", is_octomap_active);
    param_loader.loadParam("visualization", is_visualization);
    param_loader.loadParam("csv_writer", is_csv_writer);

    SampleGenerator::loadParameters(param_loader);
    filter.loadParameters(param_loader);

    ImageFilter::loadParameters(param_loader);

    if (!param_loader.loadedSuccessfully()) {
        ROS_ERROR("[FusionRadiation]: Could not load all parameters!");
        ros::requestShutdown();
    }
}

void FusionRadiation::comptonConeCallBack(const rad_msgs::Cone::ConstPtr& msg) {
    ROS_INFO("New compton cone");
    if (!is_active) return;
    ROS_INFO("Cone processing");
    const Cone cone(msg);
    processData(cone, octree_out);

    if (is_csv_writer) csv_radiations.writeRadiations(radiation_sources);
}

inline void FusionRadiation::processData(const Cone& cone, OcTreePtr_t collisions) {
    Points samples;
    /*Sampling*/
    SampleGenerator::generateSamplesUniform(cone, collisions, samples);
    ROS_INFO_STREAM("New generated samples size:" << samples.size());

    /*Filter part */
    switch (mode_) {
        case 0:
            filter.SurroundingModel(samples);
            break;
        case 1:
            filter.AverageModel(samples);
            break;
        case 2:
            filter.AverageTreeModel(samples);
            break;
        case 3:
            filter.WorstOfNumModel(samples);
            break;
        default:
            filter.SurroundingModel(samples);
            break;
    }

    filter.estimateManySources(estimation);  // get estimation of radiation sources
     ROS_INFO_STREAM("New generated estiamtions:"<< estimation.size());
    if (is_csv_writer) csv_estimations.writePoints(estimation);

    /*Drawing*/
    if (is_visualization) {
        const auto& dataset = filter.getDataset();
        PointVisualzer::clearVisual();
        PointVisualzer::setSourceLocation(radiation_sources);
        PointVisualzer::drawPoints(samples, {0.5, 0.5, 0.5, 0.8});
        PointVisualzer::drawPoints(estimation, {0, 1, 0, 1});
        PointVisualzer::drawPoints(dataset, {0, 0, 1, 0.8}, (const ulong)draw_limit_dataset);
        // Point::writePoints(dataset);
        PointVisualzer::drawSources();
    }
}

void FusionRadiation::octomapCallBack(const octomap_msgs::OctomapConstPtr& msg) {
    if (!is_active || !is_octomap_active) return;

    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));

    if (tree_ptr) {
        octree_out = OcTreePtr_t(dynamic_cast<octomap::OcTree*>(tree_ptr.release()));
    } else {
        ROS_WARN_THROTTLE(1.0, "[OctomapCeilingRemover]: octomap message is empty!");
    }
}

void FusionRadiation::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) { camera_model_.fromCameraInfo(*msg); }

void FusionRadiation::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!is_active) return;
    image_header = msg->header;
    const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
    if (!is_camera_active) return;
    cv::Mat image = bridge_image_ptr->image;
    ImageFilter::loadCameraModel(camera_model_);
    ImageFilter::findObjectInImage(image, estimation);
}

void FusionRadiation::initComptonConeCallBack() {
    const string cone_topic = string("/") + _uav_name_ + string("/compton_cone_generator/cones");
    cone_subscriber = n.subscribe(cone_topic, 1000, &FusionRadiation::comptonConeCallBack, this);
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
    radiation_sources[msg->id] = Vector3d{msg->world_pos.x, msg->world_pos.y, msg->world_pos.z};
}

void FusionRadiation::initSourcesCallbacks() {
    // radiation_locations.reserve(3);

    source_subscriber = n.subscribe<gazebo_rad_msgs::RadiationSource>("/radiation/sources", 10, &FusionRadiation::setSourceRadiationPositionCallback, this);
}

bool FusionRadiation::changeEstimationState(ToggleService::Request& req, ToggleService::Response& res) {
    is_active = (bool)req.state;
    ROS_INFO_STREAM("****************  " << is_active << "  ******************  " << (bool)req.state << "  **********");
    res.success = req.state == is_active;
    ROS_INFO_STREAM("Estimation state changed : " << is_active);
    return res.success;
}

bool FusionRadiation::setFilterParams(ModelService::Request& req, ModelService::Response& res) {
    // Set the parameters based on the request

    mode_ = req.mode;
    if (req.dataset_limit != 0) {
        filter.dataset_limit = req.dataset_limit;
        filter.threshold_hit = req.threshold_hit;
        filter.threshold_distance = req.threshold_distance;
        filter.hit_score = req.hit_score;
        filter.miss_score = req.miss_score;
        filter.hit_position = req.hit_position;
        filter.miss_position = req.miss_position;
        filter.nearest_sum_n = req.nearest_sum_n;
        filter.queue_sum_n = req.queue_sum_n;
        filter.input_coef_avg_best = req.input_coef_avg_best;
        filter.output_size_avg_best = req.output_size_avg_best;
        filter.random_sample_coef = req.random_sample_coef;
        filter.input_size_avg_worst = req.input_size_avg_worst;
        filter.output_coef_avg_worst = req.output_coef_avg_worst;
    }
    ROS_INFO_STREAM("Data changed, filter model is: " << model_list[mode_]);
    reset(req.message);

    res.success = true;
    res.message = "Filter parameters set successfully";
    return true;
}

bool FusionRadiation::setEstimationParams(EstimationService::Request& req, EstimationService::Response& res) {
    // Process the request
    filter.estimation_limit = req.estimation_limit;
    filter.estimation_dist = req.estimation_dist;
    filter.estimation_min_group_size = req.estimation_min_group_size;

    // Populate the response
    res.success = true;
    res.message = "EstimationService request processed successfully";

    reset("");

    return true;
}
void FusionRadiation::reset(const string& msg) {
    ROS_INFO("Filter reset");
    filter.clearDataset();
    radiation_sources = {};    

    if (!msg.empty()) {
        CSVFileWriter::DICT = msg + "_model_" + model_list[mode_] + "_";
        csv_estimations.createNewFile("estim");
        csv_radiations.createNewFile("rad_src");
        csv_radiations.radiations = {};
        stringstream ss;
        ss << "New mode =, " << model_list[mode_] << " , " << filter.get_settings_string();
        csv_estimations.writeHeaderOrLine(ss);

        is_csv_writer = true;
    }
}

}  // namespace fusion_radiation
