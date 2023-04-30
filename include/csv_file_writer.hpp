/**
 * @file csv_file_writer.hpp
 * @author Tadeas Zribko
 * @brief  Header file for CSVFileWriter class
 * @version 0.1
 * @date 2023-04-30
 *
 * This header file declares the CSVFileWriter class, which is used to write data to a CSV file.
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CSV_FILE_WRITER_H
#define CSV_FILE_WRITER_H

#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <point.hpp>
#include <sstream>
#include <tuple>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace fusion_radiation;

const string PATH_D = "/home/tadeas/my_workspace/src/fusion_radiation/data/";
/**
 * @class CSVFileWriter
 * @brief Class used to write data to a CSV file
 * This class provides methods to write various types of data to a CSV file. It also provides methods to create a new file, close the current file, and get the filename of the current file.
 *
 */
class CSVFileWriter {
   public:
    /**
     * @brief Constructor for CSVFileWriter class
     */
    CSVFileWriter() {}
    /**
     * @brief Destructor for CSVFileWriter class
     */
    virtual ~CSVFileWriter() {
        closeFile();
    }
    /**
     * @brief Write header or line to the output file
     * @param[in] ss stringstream containing the header or line to write
     */
    void writeHeaderOrLine(const std::stringstream &ss) {
        if (!outfile_.is_open()) {
            ROS_ERROR("SVFileWriter:: Line:: File not open");
            return;
        }
        outfile_ << ss.str() << endl;
    }
    /**
     * @brief Create a new .csv file with a unique name
     * @param[in] identifier optional identifier to include in the file name
     */
    void createNewFile(const std::string &identifier = "") {
        // Generate a unique file name based on the current date and time
        if (outfile_.is_open())
            outfile_.close();

        std::stringstream ss;
        ss << PATH_D;
        // createRepository(ss.str());
        if (identifier.empty()) {
            ss << "data_";
        }
        auto now_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        ss << identifier << "_" << DICT << "_" << std::put_time(std::localtime(&now_c), "%Y:%m:%d_%H:%M:%S") << ".csv";

        filename_ = ss.str();

        outfile_.open(filename_);

        ROS_INFO_STREAM("Created new .csv file with name " << filename_);
        ROS_INFO_STREAM("Path: " << __FILE__);
    }
    /**
     * @brief Write a vector of points to the output file
     * @param[in] points vector of points to write
     */
    void writePoints(const Points &points) {
        if (!outfile_.is_open()) {
            createNewFile("points");
        }

        outfile_ << ros::Time().now().toSec();
        for (const auto &point : points) {
            outfile_ << "," << point.coord.x() << "," << point.coord.y() << "," << point.coord.z();
        }
        outfile_ << std::endl;
    }
    /**
     * @brief Write a vector of points to the output file
     * @param[in] points vector of points to write
     */
    void writePoints(const vector<Vector3d> &points) {
        writePoints(points, ros::Time().now().toSec());
    }
    /**
     * @brief Write a vector of points to the output file with a specified timestamp
     * @param[in] points vector of points to write
     * @param[in] timestamp timestamp to include in the output file
     */
    void writePoints(const vector<Vector3d> &points, double timestamp) {
        if (!outfile_.is_open()) {
            createNewFile("vectors");
        }

        outfile_ << timestamp;
        for (const auto &point : points) {
            outfile_ << "," << point.x() << "," << point.y() << "," << point.z();
        }
        outfile_ << std::endl;
    }
    /**
     * @brief Write a map of radiation sources to the output file
     * @param[in] radiation_locations map of radiation source IDs to their locations
     */
    void writeRadiations(map<int, Vector3d> &radiation_locations) {
        if (radiation_locations.empty()) {
            return;
        }

        if (!outfile_.is_open()) {
            createNewFile("rad_src");
        }

        map<int, Vector3d>::iterator itr;
        for (itr = radiation_locations.begin(); itr != radiation_locations.end(); ++itr) {
            if (radiations.find(itr->first) == radiations.end()) {
                writeRadiation(itr->second);
                radiations[itr->first] = itr->second;
            }
        }
    }
    /**
     * @brief Write a single radiation source location to the output file
     * @param[in] radiation location of the radiation source
     */
    void writeRadiation(const Vector3d &radiation) {
        outfile_ << radiation.x() << "," << radiation.y() << "," << radiation.z() << "," << std::endl;
    }
    /**
     * @brief Close the output file
     */
    void closeFile() {
        outfile_.close();
    }
    /**
     * @brief Get the name of the output file
     * @return name of the output file
     */
    std::string getFilename() const {
        return filename_;
    }
    /**
     * @brief Create a new directory for the output file
     * @param[in] name name of the directory to create
     * @return true if the directory was created successfully, false otherwise
     */
    bool createRepository(const std::string &name) {
        string repo_name(name);
        cout << "Get Repository: " << repo_name << endl;
        repo_name.erase(remove(repo_name.begin(), repo_name.end(), ' '), repo_name.end());
        // Set the path for the new repository
        boost::filesystem::path repo_path = boost::filesystem::current_path() / repo_name;

        // Check if the repository already exists
        if (boost::filesystem::exists(repo_path)) {
            std::cerr << "Error: repository already exists" << std::endl;
            return true;
        }

        // Create the directory for the new repository
        if (!boost::filesystem::create_directory(repo_path)) {
            std::cerr << "Error: failed to create repository directory" << std::endl;
            return false;
        }

        std::cout << "Repository " << repo_name << " created successfully" << std::endl;
        return true;
    }
    inline static string DICT = "CSVs";  ///< dictionary for file names
    map<int, Vector3d> radiations;       ///< map of radiation sources

   private:
    std::string filename_;   ///< name of the output file
    std::ofstream outfile_;  ///< output file stream
};
#endif