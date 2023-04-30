#ifndef FUSION_VISUALIZER
#define FUSION_VISUALIZER

#include <mrs_lib/batch_visualizer.h>

#include <map>
#include <point.hpp>
#include <vector>

using namespace Eigen;
using namespace std;

namespace fusion_radiation {

class PointVisualzer {
   private:
    inline static mrs_lib::BatchVisualizer bv = {};
    inline static map<int, Vector3d> radiation_locations = {};

   public:
    /**
    @brief Initializes the PointVisualzer with a BatchVisualizer instance.
    @param bv The BatchVisualizer instance to use for visualization.
    */
    static void init(mrs_lib::BatchVisualizer &bv);
    /*
    @brief Sets the radiation source locations for visualization.
    @param locations The map of radiation source locations.
    */
    static void setSourceLocation(map<int, Vector3d> &locations);
    /**
    @brief Clears all the previously drawn visuals.
    */
    static void clearVisual();
    /**
    @brief Draws a set of points with the given color and scale.
    @param points The Points object to draw.
    @param color The color to use for drawing the points.
    @param scale The scale factor to apply to the points.
    */
    static void drawPoints(const vector<Vector3d> &points, const Vector4f &color, const double scale = 0.2);
    /**
    @brief Draws a set of points with the given color, size limit, and scale.
    @param points The Points object to draw.
    @param color The color to use for drawing the points.
    @param size The maximum number of points to draw.
    @param scale The scale factor to apply to the points.
    */
    static void drawPoints(const Points &points, const Vector4f &color, const double scale = 0.2);
    /**
    @brief Draws a set of points with the given color and scale.
    @param points The vector of points to draw.
    @param color The color to use for drawing the points.
    @param scale The scale factor to apply to the points.
    */
    static void drawPoints(const Points &points, const Vector4f &color, const ulong size, const double scale = 0.2);
    /**
    @brief Draws the radiation source locations with the given scale.
    @param scale The scale factor to apply to the points.
    */
    static void drawSources(const double scale = 0.2);
};
}  // namespace fusion_radiation

#endif
