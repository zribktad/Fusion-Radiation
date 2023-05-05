
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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

const std::string color_encoding = "bgr8";
const std::string grayscale_encoding = "mono8";

namespace fusion_radiation {
class ImageFilter {
   public:
    /**
     * @brief Initializes the image filter with the given ROS node handle and UAV name.
     *
     * @param n The ROS node handle.
     * @param uav_name The name of the UAV.
     */
    static void initImageFilter(ros::NodeHandle &n, string &uav_name);
    /**
     * @brief Loads the camera model used by the image filter.
     *
     * @param camera_model The camera model.
     */
    static void loadCameraModel(CameraModel_t &camera_model);

    /**
     * @brief Finds object in the input image and estimates its 3D position.
     *
     * @param image Input image where the object will be detected.
     * @param estimates Vector of estimated points in 3D space.
     */
    static void findObjectInImage(cv::Mat &image, vector<Vector3d> &estimates);
    /**
     * @brief Loads the parameters for the image filter from the given parameter loader.
     *
     * @param param_loader The parameter loader.
     */
    static void loadParameters(mrs_lib::ParamLoader &param_loader);
    /**
     * @brief Publishes an image message to a topic.
     *
     * @param image Input image to be published.
     * @param image_header Header information for the published image.
     * @param encoding Encoding type of the published image.
     * @param pub ROS image transport publisher object.
     */
    static void publishImage(cv::InputArray &image, std_msgs::Header &image_header, const std::string &encoding, const image_transport::Publisher &pub);

   private:
    /**
     * @brief Transforms a 3D point to the camera frame and projects it onto the image plane.
     *
     * @param x The x-coordinate of the 3D point.
     * @param y The y-coordinate of the 3D point.
     * @param z The z-coordinate of the 3D point.
     * @param out_point The resulting 2D point on the image plane.
     * @return true if the transformation and projection was successful, false otherwise.
     */
    inline static bool transformPointTocamera(const double &x, const double &y, const double &z, cv::Point2d &out_point);
        /**
     * @brief Transforms a 3D point to the camera frame and projects it onto the image plane.
     *
     * @param x The x-coordinate of the 3D point.
     * @param y The y-coordinate of the 3D point.
     * @param z The z-coordinate of the 3D point.
     * @param out_point The resulting 2D point on the image plane.
     * @return true if the transformation and projection was successful, false otherwise.
     */
    inline static bool transformPointTocamera2(const double &x, const double &y, const double &z, cv::Point2d &out_point);
    /**
     * @brief Draws detected edges and estimated points to the input image.
     *
     * @param image Input image where the edges and points will be drawn.
     * @param detected_edges Binary image containing the detected edges.
     * @param estimates Vector of estimated points in 3D space.
     * @param contours Vector of contours enclosing the detected edges.
     */
    inline static void drawToImage(cv::Mat &image, cv::Mat &detected_edges, const vector<Vector3d> &estimates, const vector<vector<cv::Point> > &contours);

    /**
     *
     * @brief Calculates the squared distance between a point and a rectangle
     * @param p Point to calculate distance to
     * @param rect Rectangle to calculate distance from
     * @return Squared distance between point and rectangle
     */
    inline static const double distanceSquared(cv::Point2d p, cv::Rect rect);

    /**
     *
     * @brief Finds all rectangles in a certain distance of a point
     * @param p Point to find rectangles around
     * @param found_quads Vector of rectangles to search
     * @param delta Distance to search for rectangles within
     * @param closed Vector of rectangles found within the specified distance
     */
    inline static void findRectanglesInDistance(cv::Point2d &p, std::vector<cv::Rect> &found_quads, const double delta, std::vector<cv::Rect> &closed);
    /**
     * @brief A transformer object for converting points between frames of reference
     */
    inline static std::unique_ptr<mrs_lib::Transformer> transformer_ = {};
    /**
     * @brief The name of the frame of reference for the robot's position
     */
    inline static string origin_name = "";
    /**
     * @brief The position of a 3D point in the world frame of reference
     */
    inline static geometry_msgs::PoseStamped pt3d_world = {};
    /**
     * @brief The camera model for the camera used to capture images
     */
    inline static CameraModel_t camera_model = {};
    /**
     * @brief The font used for text displayed on images
     */
    inline static const int txt_font = cv::FONT_HERSHEY_PLAIN;
    /**
     * @brief The thickness of points used to mark rectangles
     */
    inline static const int pt_thickness = 1;  // pixels, -1 means filled
    /**
     * @brief The font scale used for text displayed on images
     */
    inline static const double txt_font_scale = 1.0;
    /**
     * @brief The color used for circles marking points
     */
    inline static const cv::Scalar circle_color = {255, 0, 0};
    /**
     * @brief The color used for rectangles marking found quads
     */
    inline static const cv::Scalar mark_color = {255, 0, 255};
    /**
     * @brief The threshold shift used for the Canny edge detection algorithm
     */
    inline static int threshold_shift = 135;
    /**
     * @brief The size threshold used for the Canny edge detection algorithm
     */
    inline static int size_threshold = 360;
    /**
     * @brief The mode for dilation
     */
    inline static int dilation = 0;
    /**
     * @brief The size of dilation
     */
    inline static int dilation_size = 1;
    /**
     * @brief The distance between points for filtering images
     */
    inline static int delta_distance = 25;
    /**
     * @brief The percentage of image size to resize to
     */
    inline static int resize_image = 50;
    /**
     * @brief A flag indicating whether to show edges on the GUI
     */
    inline static int show_edges = 0;
    /**
     * @brief A flag indicating whether to show the GUI
     */
    inline static int show_image = 0;
    /**
     * @brief A flag indicating whether the camera GUI is active
     */
    inline static bool is_camera_GUI_active = false;
};

}  // namespace fusion_radiation

#endif
