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
  polygonSubscriber_ = nodeHandle_.subscribe(polygonMeshTopic_, 1, &Write::polygonCallback, this);
  ROS_INFO_STREAM("Subscribed to polygonMeshTopic \"" << polygonMeshTopic_ << "\".");
}

bool Write::readParameters() {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("topic", polygonMeshTopic_) && allParametersRead;
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

void Write::polygonCallback(const pcl_msgs::PolygonMesh& polygon){
    ROS_INFO("Got a polygon message.");

    pcl::PolygonMesh receivedMesh;
    pcl_conversions::toPCL (polygon, receivedMesh);
    std::cout << folderPath_ << std::endl;

    std::stringstream filePath;
    filePath << folderPath_ << "/";
    filePath << fileName_;
    filePath << "_polygon.";
    filePath << fileEnding_;

    filePathComplete_ = filePath.str();
    if (fileEnding_ == "ply") {
        pcl::io::savePLYFile(filePathComplete_, receivedMesh);
    }
    else{
        ROS_ERROR_STREAM("Data format not supported.");
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
