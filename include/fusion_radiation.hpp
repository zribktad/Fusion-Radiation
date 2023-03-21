#ifndef FUSION_RADIATION_HPP
#define FUSION_RADIATION_HPP

/*ROS and mrs*/
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/transformer.h>

/* msgs */
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <gazebo_rad_msgs/RadiationSource.h>

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
#include <vector>
#include <map>

/*My library*/
#include <cone.hpp>
#include "point_visualizer.hpp"
#include "fusion_run.hpp"
#include "image_filter.hpp"
#include "csv_file_writer.hpp"




using namespace std;
using namespace Eigen;



namespace fusion_radiation {
class FusionRadiation: public nodelet::Nodelet {


   private:
    ros::Subscriber source_sub;
    ros::NodeHandle n;
    inline static mrs_lib::BatchVisualizer bv = {};

    /******** parameters *****************/
    string _uav_name_;

    virtual void onInit();
    inline void loadParameters();

    /********* Sources  position  **************/
    ros::Subscriber source_subscriber;
    map<int,Vector3d> radiation_sources;
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
    void initCameraCallBacks();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    /*************** openCL ****************/
    inline static const std::string color_encoding = "bgr8";
    inline static const std::string grayscale_encoding = "mono8";


     /******** csv *****************/
     inline static CSVFileWriter csv_radiations = {"rad_src"};
};

}  // namespace fusion_radiation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(fusion_radiation::FusionRadiation, nodelet::Nodelet)
#endif
