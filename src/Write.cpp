/*
 * Write.cpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Write.hpp"

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace point_cloud_io {

Write::Write(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), fileName_("point_cloud"), fileEnding_("ply") {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &Write::pointCloudCallback, this);
  ROS_INFO_STREAM("Subscribed to topic \"" << pointCloudTopic_ << "\".");
}

bool Write::readParameters() {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("topic", pointCloudTopic_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("folder_path", folderPath_) && allParametersRead;

  nodeHandle_.getParam("file_name", fileName_);
  nodeHandle_.getParam("file_ending", fileEnding_);

  if (!allParametersRead) {
    ROS_WARN(
        "Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io write"
        " _topic:=/my_topic"
        " _folder_path:=/home/user/my_point_clouds"
        " _file_name:=my_filename"
        " _file_ending:=my_ending");
    return false;
  }

  return true;
}

void Write::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  ROS_INFO_STREAM("Received point cloud with " << cloud->height * cloud->width << " points.");
  std::cout << folderPath_ << std::endl;
  std::stringstream filePath;
  filePath << folderPath_ << "/";
  filePath << fileName_;
  filePath << ".";
  filePath << fileEnding_;

  filePathComplete_ = filePath.str();

  if (fileEnding_ == "ply") {
    // Write .ply file.
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);

    pcl::PLYWriter writer;
    bool binary = false;
    bool use_camera = false;
    if (writer.write(filePath.str(), pclCloud, binary, use_camera) != 0) {
      ROS_ERROR("Something went wrong when trying to write the point cloud file.");
      return;
    }
  } else if (fileEnding_ == "pcd") {
    // Write pcd file
    pcl::PointCloud<pcl::PointXYZRGBNormal> pclCloud;
    pcl::fromROSMsg(*cloud, pclCloud);
    pcl::io::savePCDFile(filePath.str(), pclCloud);
  } else {
    ROS_ERROR_STREAM("Data format not supported.");
    return;
  }

  radiation_srvs::MeshInfo meshInfo;
  meshInfo.request.meshSaved = true;
  meshInfo.request.fileName = filePathComplete_;
  if(!ros::service::call("/radiation_estimator/meshWritten",
                                             meshInfo)){
      ROS_ERROR("Could not send info that mesh file is saved in radiation estimator folder.");
  }
  ROS_INFO_STREAM("Saved point cloud to " << filePath.str() << ".");
}

}  // namespace point_cloud_io
