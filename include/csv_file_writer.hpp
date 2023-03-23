#ifndef CSV_FILE_WRITER_H
#define CSV_FILE_WRITER_H

#include <ros/ros.h>

#include <chrono>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>
#include <point.hpp>

using namespace std;
using namespace Eigen;
using namespace fusion_radiation;

class CSVFileWriter {
   public:
   CSVFileWriter(const std::string &identifier = "") {
        // Generate a unique file name based on the current date and time
    
        std::stringstream ss;

        ss << "/home/tadeas/my_workspace/src/fusion_radiation/data/";
        if (identifier.empty()) {
            ss << "data_";
        }
        auto now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        ss << identifier << "_"<<std::put_time(std::localtime(&now_c), "%Y:%m:%d_%H:%M:%S")<< ".csv";

        filename_ = ss.str();

        outfile_.open(filename_);
        outfile_ << " timestamp ,x , y , z , ... " << std::endl;
        ROS_INFO_STREAM("Created new .csv file with name " << filename_);
        ROS_INFO_STREAM("Path: " << __FILE__  );
        //estim_2023:03:21_21:28:44.csv
    }

    virtual ~CSVFileWriter() {
        closeFile();
    }
    void writePoints(const Points &points) {
       double timestamp= ros::Time::now().toSec();
       outfile_ << timestamp;
        for (const auto &point : points) {
            outfile_ << "," << point.coord.x() << "," << point.coord.y() << "," << point.coord.z();
        }
        outfile_ << std::endl;
    }

    void writePoints(const vector<Vector3d> &points) {
        writePoints(points, ros::Time::now().toSec());
    }

    void writePoints(const vector<Vector3d> &points, uint64_t timestamp) {
        outfile_ << timestamp;
        for (const auto &point : points) {
            outfile_ << "," << point.x() << "," << point.y() << "," << point.z();
        }
        outfile_ << std::endl;
    }

    void writeRadiations(map<int, Vector3d> &radiation_locations) {
        map<int, Vector3d>::iterator itr;
        for (itr = radiation_locations.begin(); itr != radiation_locations.end(); ++itr) {
           if(radiations.find(itr->first)!=radiations.end()){
            writeRadiation(itr->second);
            radiation_locations[itr->first]=itr->second;
           }
        }
    }
    void writeRadiation(const Vector3d &radiation) {
        outfile_ <<radiation.x() << "," << radiation.y() << "," << radiation.z() << "," << std::endl;
    }

    void closeFile() {
        outfile_.close();
    }

    std::string getFilename() const {
        return filename_;
    }

   private:
    std::string filename_;
    std::ofstream outfile_;
    map<int, Vector3d> radiations;
};
#endif