#ifndef FUSION_VISUALIZER
#define FUSION_VISUALIZER

#include <mrs_lib/batch_visualizer.h>

#include <point.hpp>
#include <vector>

using namespace Eigen;
using namespace std;

namespace fusion_radiation {

class PointVisualzer {
   private:
    inline static mrs_lib::BatchVisualizer bv = {};
    inline static vector<Vector3d> radiation_locations = {};

   public:
    static void init(mrs_lib::BatchVisualizer& bv);
    static void setSourceLocation(vector<Vector3d>& locations);
    static void clearVisual();
    static void drawPoints(const vector<Vector3d>&points,const Vector4f &color,const double scale= 0.2) ;
    static void drawPoints(const Points &points,const Vector4f &color,const double scale= 0.2);
    static void drawPoints(const Points &points, const Vector4f &color, const ulong size, const double scale = 0.2);
    static void drawSources(const double scale = 0.2);
};
}  // namespace fusion_radiation

#endif
