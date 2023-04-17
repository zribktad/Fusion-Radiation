#ifndef FUSION_RADIATION_HPP
#define FUSION_RADIATION_HPP

/*ROS and mrs*/
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/transformer.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

/* msgs */
#include <gazebo_rad_msgs/RadiationSource.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32.h>

/* octomap */
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

/* image */
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

/*Default and download*/
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <map>
#include <vector>

/*My library*/
#include <cone.hpp>

#include "csv_file_writer.hpp"
#include "fusion_radiation/EstimationService.h"
#include "fusion_radiation/FusionService.h"
#include "fusion_radiation/ModelService.h"
#include "fusion_radiation/ToggleService.h"
#include "image_filter.hpp"
#include "point_visualizer.hpp"
#include "sample_filter.hpp"
#include "sample_generator.hpp"

using namespace std;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace fusion_radiation {
class FusionRadiation : public nodelet::Nodelet {
   private:
    ros::Subscriber source_sub;
    ros::NodeHandle n;
    inline static bool is_active = false;
    inline static bool is_camera_active = false;
    inline static bool is_camera_GUI_active = false;
    inline static bool is_octomap_active = true;
    inline static bool is_visualization = false;
    inline static bool is_csv_writer = false;
    inline static bool is_pub_est_active = false;

    inline static mrs_lib::BatchVisualizer bv = {};

    /******** parameters *****************/
    string _uav_name_;

    virtual void onInit();
    inline void loadParameters();

    static void reset(const string& msg);

    /********* Sources  position  **************/
    ros::Subscriber source_subscriber;
    inline static map<int, Vector3d> radiation_sources = {};
    void setSourceRadiationPositionCallback(const gazebo_rad_msgs::RadiationSourceConstPtr& msg);
    void initSourcesCallbacks();

    /************ Compton cone  ************/
    ros::Subscriber cone_subscriber;
    void comptonConeCallBack(const rad_msgs::Cone::ConstPtr& msg);
    void initComptonConeCallBack();

    /************ Octomap  ************/
    OcTreePtr_t octree_out;
    ros::Subscriber octomap_subscriber;
    void octomapCallBack(const octomap_msgs::OctomapConstPtr& msg);
    void initOctomapCallBack();

    /******** Camera Image and Info **********/
    image_geometry::PinholeCameraModel camera_model_;
    std_msgs::Header image_header;
    ros::Subscriber camera_info_sub, camera_image_sub;
    image_transport::Publisher image_rad_source_est_pub;
    void initCameraCallBacks();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /***************** Estimation **************************/
    inline static SampleFilter filter = {};
    inline static vector<Vector3d> estimation = {};
    inline static int draw_limit_dataset = 400;
    inline static int mode_ = 0;
    inline static string model_list[] = {"SurroundingModel", "AverageModel", "AverageTreeModel", "WorstOfNumModel"};
    inline void processData(const Cone& cone, OcTreePtr_t collisions);
    ros::Publisher estimation_pub;

    /**************** Service ****************/
    static bool changeEstimationState(ToggleService::Request& req, ToggleService::Response& res);
    static bool setFusionState(FusionService::Request& req, FusionService::Response& res);
    static bool setFilterParams(ModelService::Request& req, ModelService::Response& res);
    static bool setEstimationParams(EstimationService::Request& req, EstimationService::Response& res);

    /*************** openCL ****************/
    inline static const std::string color_encoding = "bgr8";
    inline static const std::string grayscale_encoding = "mono8";

    /******** csv *****************/
    inline static CSVFileWriter csv_radiations = {};
    inline static CSVFileWriter csv_estimations = {};
    inline static CSVFileWriter csv_particles = {};
};

}  // namespace fusion_radiation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fusion_radiation::FusionRadiation, nodelet::Nodelet)
#endif
