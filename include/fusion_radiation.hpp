/**
 * @file fusion_radiation.hpp
 * @author Tadeáš Zribko
 * @brief
 * @version 0.1
 * @date 2023-04-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef FUSION_RADIATION_HPP
#define FUSION_RADIATION_HPP

/*ROS and mrs*/
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/param_loader.h>
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
#include "fusion_test.hpp"
#include "image_filter.hpp"
#include "point_visualizer.hpp"
#include "sample_filter.hpp"
#include "sample_generator.hpp"

using namespace std;
using namespace Eigen;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace fusion_radiation {
/**
 * @brief This class implements the main node logic for radiation source estimation and fusion.
 *
 */
class FusionRadiation : public nodelet::Nodelet {
   private:
    ros::Subscriber source_sub;
    ros::NodeHandle n;
    inline static bool is_active = false;
    inline static bool is_camera_active = false;

    inline static bool is_octomap_active = true;
    inline static bool is_visualization = false;
    inline static bool is_csv_writer = false;
    inline static bool is_pub_est_active = false;

    static const int MODEL_SURROUNDING = 0;
    static const int MODEL_AVERAGE = 1;
    static const int MODEL_AVERAGE_TREE = 2;
    static const int MODEL_WORST_OF_NUM = 3;

    inline static mrs_lib::BatchVisualizer bv = {};

    /******** parameters *****************/
    string _uav_name_;
    /**
     * @brief Initialization method for the node. It loads the parameters and sets up the subscribers, publishers and services.
     */
    virtual void onInit();
    /**
     * @brief Loads the required parameters from the ROS parameter server.
     */
    inline void loadParameters();
    /**
     * @brief  Reset particle filter function
     *
     * @param msg string for input in csv file name
     */
    static void reset(const string& msg);

    /********* Sources  position  **************/
    ros::Subscriber source_subscriber;
    inline static map<int, Vector3d> radiation_sources = {};
    /**
     * @brief Set the position of the radiation source.
     *
     * This function is called when a new message is received on the `/fusion_radiation/source_position` topic.
     * It sets the position of the radiation source to the position specified in the message.
     *
     * @param msg A pointer to a `geometry_msgs::PointStamped` message containing the position of the radiation source.
     */
    void setSourceRadiationPositionCallback(const gazebo_rad_msgs::RadiationSourceConstPtr& msg);
    /**
     * @brief Initialize the subscriber for the octomap topic.
     */
    void initSourcesCallbacks();

    /************ Compton cone  ************/
    ros::Subscriber cone_subscriber;
    /**
     * @brief Callback function for Compton cones received
     * @param msg - ROS message containing the cone
     */
    void comptonConeCallBack(const rad_msgs::Cone::ConstPtr& msg);
    /**
     * @brief Initialize callback for incoming Compton cone messages
     *
     */
    void initComptonConeCallBack();

    /************ Octomap  ************/
    OcTreePtr_t octree_out;
    ros::Subscriber octomap_subscriber;
    /**
     * @brief Callback function for the octomap topic. This function receives the octomap and generates an octree from it.
     *
     * @param msg - Pointer to the received Octomap message.
     */
    void octomapCallBack(const octomap_msgs::OctomapConstPtr& msg);
    /**
     * @brief Function to initialize the octomap callbacks.
     */
    void initOctomapCallBack();

    /******** Camera Image and Info **********/
    image_geometry::PinholeCameraModel camera_model_;
    std_msgs::Header image_header;
    ros::Subscriber camera_info_sub, camera_image_sub;
    image_transport::Publisher image_rad_source_est_pub;
    /**
     * @brief Function to initialize the camera callbacks.
     */
    void initCameraCallBacks();
    /**
     * @brief Callback function for the camera info message
     *
     * This function is called when a new camera info message is received.
     *
     * @param msg A pointer to the received camera info message
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    /**
     * @brief Callback function for the camera topic. This function receives an image, applies the specified filters, and estimates the radiation sources from it.
     *
     * @param msg - Pointer to the received sensor_msgs/Image message.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /***************** Estimation **************************/
    inline static SampleFilter filter = {};
    inline static vector<Vector3d> estimation = {};
    inline static int draw_limit_dataset = 400;
    inline static int model_ = 0;
    inline static string model_list[] = {"SurroundingModel", "AverageModel", "AverageTreeModel", "WorstOfNumModel"};
    /**
     * @brief Process the received data from the cone and update the estimation of radiation sources.
     *
     * @param cone - The received Cone object.
     * @param collisions - The Octree representing the environment.
     */
    inline void processData(const Cone& cone, OcTreePtr_t collisions);
    ros::Publisher estimation_pub;

    /**************** Service ****************/
    /**
     * @brief Function to change the estimation state.
     * @param req Request.
     * @param res Response.
     * @return Returns true if the service has successfully changed the estimation state, false otherwise.
     */
    static bool changeEstimationState(ToggleService::Request& req, ToggleService::Response& res);
    /**
     * @brief Function to set the fusion state.
     * @param req Request.
     * @param res Response.
     * @return Returns true if the service has successfully set the fusion state, false otherwise.
     */
    static bool setFusionState(FusionService::Request& req, FusionService::Response& res);
    /**
     * @brief Function to set the filter parameters.
     * @param req Request.
     * @param res Response.
     * @return Returns true if the service has successfully set the filter parameters, false otherwise.
     */
    static bool setFilterParams(ModelService::Request& req, ModelService::Response& res);
    /**
     * @brief Function to set the estimation parameters.
     * @param req Request.
     * @param res Response.
     * @return Returns true if the service has successfully set the estimation parameters, false otherwise.
     */
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
