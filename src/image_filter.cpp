#include "image_filter.hpp"

namespace fusion_radiation {

void ImageFilter::initImageFilter(ros::NodeHandle &n, string &uav_name) {
    image_transport::ImageTransport it(n);
    ImageFilter::transformer_ = std::make_unique<mrs_lib::Transformer>("fusion_radiation");
   // ImageFilter::transformer_->setDefaultPrefix(uav_name);
    ImageFilter::transformer_->retryLookupNewest(true);
    ImageFilter::origin_name = uav_name + string("/rtk_origin");
}

void ImageFilter::loadParameters(mrs_lib::ParamLoader &param_loader) {
    param_loader.loadParam("image_filter/threshold_shift", threshold_shift);
    param_loader.loadParam("image_filter/size_threshold", size_threshold);
    param_loader.loadParam("image_filter/dilation", dilation);
    param_loader.loadParam("image_filter/dilation_size", dilation_size);
    param_loader.loadParam("image_filter/delta_distance", delta_distance);
    param_loader.loadParam("image_filter/resize_image", resize_image);
    param_loader.loadParam("image_filter/show_edges", show_edges);
    param_loader.loadParam("image_filter/show_image", show_image);
    param_loader.loadParam("image_filter/camera_GUI", is_camera_GUI_active);

    // namedWindow("tested_image", WINDOW_NORMAL);

    if (!is_camera_GUI_active) return;

    ROS_INFO("OPENCL GUI ON");
    namedWindow("OpenCL Settings", WINDOW_NORMAL);
    namedWindow("Detector", WINDOW_NORMAL);

    createTrackbar("show image", "OpenCL Settings", &show_image, 1);
    createTrackbar("canny thresh shift (%) ", "OpenCL Settings", &threshold_shift, 1000);
    createTrackbar("canny size thresh", "OpenCL Settings", &size_threshold, 1000);
    createTrackbar("dilatiation mode (0 = off)", "OpenCL Settings", &dilation, 1);
    createTrackbar("dilatation size", "OpenCL Settings", &dilation_size, 10);
    createTrackbar("resize image (%)", "OpenCL Settings", &resize_image, 150);
    createTrackbar("show edge image", "OpenCL Settings", &show_edges, 1);
    createTrackbar("delta distance", "OpenCL Settings", &delta_distance, 300);
}
 
void ImageFilter::loadCameraModel(CameraModel_t &camera_model) {
    ImageFilter::camera_model = camera_model;
}

inline bool ImageFilter::transformPointTocamera(const double &x, const double &y, const double &z, cv::Point2d &out_point) {


        geometry_msgs::PoseStamped pt3d_world;
    pt3d_world.header.frame_id = origin_name;
    pt3d_world.header.stamp = ros::Time::now();
    pt3d_world.pose.position.x = x;
    pt3d_world.pose.position.y = y;
    pt3d_world.pose.position.z = z;

    auto ret = ImageFilter::transformer_->transformSingle(pt3d_world, camera_model.tfFrame());
    
    geometry_msgs::PoseStamped pt3d_cam;
    if (ret) {
        pt3d_cam = ret.value();

    } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[PointToCamera]: Failed to tranform point from world to camera frame, cannot backproject point to image header:" << pt3d_world.header.frame_id );
        return false;
    }
    const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);
    const cv::Point2d pt2d = ImageFilter::camera_model.project3dToPixel(pt3d);  // this is now in rectified image coordinates
    out_point = ImageFilter::camera_model.unrectifyPoint(pt2d);

    return true;
}

inline const double distanceSquared(cv::Point2d p, cv::Rect rect) {
    const auto delta_x = max(min((int)p.x, rect.x + rect.width), rect.x);
    const auto delta_y = max(min((int)p.y, rect.y + rect.height), rect.y);
    const auto dx = p.x - delta_x;
    const auto dy = p.y - delta_y;
    return dx * dx + dy * dy;
}

inline void findRectanglesInDistance(cv::Point2d &p, vector<cv::Rect> &found_quads, const double delta, vector<cv::Rect> &closed) {
    const double distance = delta * delta;
    for (auto &r : found_quads) {
        if (distance >= distanceSquared(p, r))
            closed.emplace_back(r);
    }
}

inline void ImageFilter::drawToImage(cv::Mat &image, cv::Mat &detected_edges, const vector<Vector3d> &estimates, const vector<vector<cv::Point>> &contours) {
    vector<cv::Rect> found_quads;
    found_quads.reserve(contours.size());
    for (const auto &contour : contours) {
        cv::Rect boundingBox = boundingRect(contour);
        found_quads.emplace_back(boundingBox);
        if (show_edges)
            cv::rectangle(detected_edges, boundingBox, cv::Scalar(255));
    }

    for (const auto &point3D : estimates) {
        cv::Point2d point2D;
        if (transformPointTocamera(point3D.x(), point3D.y(), point3D.z(), point2D)) {  // red or blue color, depending on the pixel ordering (BGR or RGB)
            cv::circle(image, point2D, delta_distance, circle_color, pt_thickness);
            cv::circle(image, point2D, delta_distance, circle_color, 5);
            const std::string coord_txt = "[" + std::to_string(point3D.x()) + "," + std::to_string(point3D.y()) + "," + std::to_string(point3D.z()) + "]";
            const cv::Point2d txt_pos(point2D.x + delta_distance, point2D.y + delta_distance);  // offset the text a bit to avoid overlap with the circle
            cv::putText(image, coord_txt, txt_pos, txt_font, txt_font_scale, circle_color);

            vector<cv::Rect> closed_rects;
            findRectanglesInDistance(point2D, found_quads, delta_distance, closed_rects);
            for (auto &r : closed_rects) {
                cv::rectangle(image, r, mark_color, 3);
            }
        }
    }
}

void ImageFilter::findObjectInImage(cv::Mat &image, vector<Vector3d> &estimates) {
    cv::Mat detected_edges, gray;
    cv::cvtColor(image, gray, CV_BGR2GRAY);  // Convert the image to grayscale
    cv::medianBlur(gray, gray, 3);           // Reduce noise with a 3x3 blur

    if (dilation != 0) {
        int size = 2 * dilation_size + 1;
        Mat element = getStructuringElement(MORPH_RECT, Size(size, size), Point(dilation_size, dilation_size));
        dilate(gray, gray, element);
    }

    cv::Mat tmp;
    const int threshold_start = cv::threshold(gray, tmp, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU) * threshold_shift / 100;
    
    cv::Canny(gray, detected_edges, threshold_start, threshold_start + size_threshold);

    vector<vector<cv::Point>> contours;
    cv::findContours(detected_edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawToImage(image, detected_edges, estimates, contours);

    double resize_value = ((double)resize_image) / 100;

    if (is_camera_GUI_active)

    {
        if (show_edges) {
            cv::resize(detected_edges, detected_edges, cv::Size(), resize_value, resize_value, cv::INTER_NEAREST);
            cv::putText(detected_edges, "Threshold min: " + std::to_string(threshold_start) + " max: " + std::to_string(threshold_start + size_threshold), {5, 10 * resize_value}, txt_font, 1 * resize_value, cv::Scalar(255));
            cv::putText(detected_edges, "Number of points: " + std::to_string(estimates.size()), {5, 25 * resize_value}, txt_font, 1 * resize_value, cv::Scalar(255));
            cv::putText(detected_edges, "Number of quads: " + std::to_string(contours.size()), {5, 35 * resize_value}, txt_font, 1 * resize_value, cv::Scalar(255));
            imshow("Detected edges info", detected_edges);
        }
        if (show_image) {
            cv::resize(image, image, cv::Size(), resize_value, resize_value, cv::INTER_NEAREST);
            imshow("Detector", image);
        }else{
               cv::Mat empty(1,1,CV_8UC3, cv::Scalar(0, 0, 0));
            imshow("Detector",empty);
        }
    }
    waitKey(20);
}

void ImageFilter::publishImage(cv::InputArray &image, std_msgs::Header &image_header, const std::string &encoding, const image_transport::Publisher &pub) {
    cv_bridge::CvImage bridge_image_out;
    bridge_image_out.header = image_header;
    bridge_image_out.image = image.getMat();
    bridge_image_out.encoding = encoding;
    sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
    pub.publish(out_msg);
}

}  // namespace fusion_radiation