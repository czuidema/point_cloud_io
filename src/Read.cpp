/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Read.hpp"

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

namespace point_cloud_io {

Read::Read(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), pointCloudMessage_(new sensor_msgs::PointCloud2()) {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 1, true);
  pub_srv_ = nodeHandle_.advertiseService(
          "readAndSendPly", &Read::readServiceCallback, this);
}

bool Read::readParameters() {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("file_path", filePath_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("topic", pointCloudTopic_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("frame", pointCloudFrameId_) && allParametersRead;

  /*double updateRate;
  nodeHandle_.param("rate", updateRate, 0.0);
  if (updateRate == 0.0) {
    isContinuouslyPublishing_ = false;
  } else {
    isContinuouslyPublishing_ = true;
    updateDuration_.fromSec(1.0 / updateRate);
  }*/

  if (!allParametersRead) {
    ROS_WARN(
        "Could not read all parameters. Typical command-line usage:\n"
        "rosrun point_cloud_io read"
        " _file_path:=/home/user/my_point_cloud.ply"
        " _topic:=/my_topic"
        " _frame:=sensor_frame"
        " (optional: _rate:=publishing_rate)");
    return false;
  }

  return true;
}

bool Read::readFile(const std::string& filePath, const std::string& pointCloudFrameId) {
  if (filePath.find(".ply") != std::string::npos) {
      std::cout << "File path is " << filePath << std::endl;
    // Load .ply file.
    pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud; // try PointXYZ instead of PointXYZRGBNormal
    if (pcl::io::loadPLYFile(filePath, pointCloud) != 0) {
      return false;
    }

    // Define PointCloud2 message.
    pcl::toROSMsg(pointCloud, *pointCloudMessage_);
  } else if (filePath.find(".vtk") != std::string::npos) {
    // Load .vtk file.
    pcl::PolygonMesh polygonMesh;
    pcl::io::loadPolygonFileVTK(filePath, polygonMesh);

    // Define PointCloud2 message.
    pcl_conversions::moveFromPCL(polygonMesh.cloud, *pointCloudMessage_);
  } else {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  ROS_INFO_STREAM("Loaded point cloud with " << pointCloudMessage_->height * pointCloudMessage_->width << " points.");
  return true;
}

void Read::timerCallback(const ros::TimerEvent& /*timerEvent*/) {
  if (!publish()) {
    ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
  }
}


bool Read::readServiceCallback(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res){
    if(!readFile(filePath_, pointCloudFrameId_)){
        ROS_ERROR("Something went wrong when trying to read the point cloud file.");
    }
    if (!publish()) {
      ROS_ERROR("Something went wrong when trying to publish the point cloud file.");
    }
};


bool Read::publish() {
  pointCloudMessage_->header.stamp = ros::Time::now();
  if (pointCloudPublisher_.getNumSubscribers() > 0u) {
    pointCloudPublisher_.publish(pointCloudMessage_);
    ROS_INFO_STREAM("Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}

}  // namespace point_cloud_io
