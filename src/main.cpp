

#include <chrono>
#include <memory>
#include <vector>

#include "KDTree.hpp"
#include "fusion_radiation.hpp"
#include "fusion_test.hpp"
#include "point.hpp"

using namespace fusion_radiation;



int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fusion_radiation");
     fusion_radiation::FusionRadiation fr;


    //FusionTest::timeCompareSampler();


    ros::spin();
    return 0;
}
