
#ifndef IMAGE_FILTER_H
#define IMAGE_FILTER_H

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"

/* builded c++ */
#include <eigen3/Eigen/Core>
#include <queue>
#include <vector>

/* ROS includes for working with OpenCV and images */
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>

/* OpenCV includes */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace Eigen;
using namespace std;
using CameraModel_t = image_geometry::PinholeCameraModel;
using Transformer_t = std::unique_ptr<mrs_lib::Transformer>;

namespace fusion_radiation {
class ImageFilter {
   public:
    static void initImageFilter(ros::NodeHandle &n, string &uav_name);
    static void loadCameraModel(CameraModel_t &camera_model);
    static void findObjectInImage(cv::Mat &image, vector<Vector3d> &estimates);
    static void loadParameters(mrs_lib::ParamLoader &param_loader);

   private:
    inline static bool transformPointTocamera(const double &x, const double &y, const double &z, cv::Point2d &out_point);
    static void publishImage(cv::InputArray image, std_msgs::Header image_header, const std::string &encoding, const image_transport::Publisher &pub);
    inline static void drawToImage(cv::Mat &image, cv::Mat &detected_edges, const vector<Vector3d> &estimates,const vector<vector<cv::Point> > &contours);

    inline static std::unique_ptr<mrs_lib::Transformer> transformer_ = {};
    inline static string origin_name = "";
    inline static geometry_msgs::PoseStamped pt3d_world = {};
    inline static CameraModel_t camera_model ={};

    inline static const int txt_font = cv::FONT_HERSHEY_PLAIN;
    inline static const int pt_thickness = 1;  // pixels, -1 means filled
    inline static const double txt_font_scale = 1.0;
    inline static const cv::Scalar circle_color = {255, 0, 0};
    inline static const cv::Scalar mark_color = {255, 0, 255};

    inline static int threshold_shift = 135;
    inline static int size_threshold = 360;
    inline static int dilation = 0;
    inline static int dilation_size = 1;
    inline static int delta_distance = 25;
    inline static int resize_image = 50;
    inline static int show_edges =1;
    
};

}  // namespace fusion_radiation

#endif
