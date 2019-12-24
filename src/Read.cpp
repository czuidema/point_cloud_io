/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Read.hpp"

// define the following in order to eliminate the deprecated headers warning
#define VTK_EXCLUDE_STRSTREAM_HEADERS
#include <pcl/io/vtk_lib_io.h>

namespace point_cloud_io {

Read::Read(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
  if (!readParameters()) {
    ros::requestShutdown();
  }
  polygonMeshPublisher_ = nodeHandle_.advertise<pcl_msgs::PolygonMesh>(polygonMeshTopic_, 1, true);
  pub_srv_ = nodeHandle_.advertiseService(
          "readAndSendPly", &Read::readServiceCallback, this);
}

bool Read::readParameters() {
  bool allParametersRead = true;
  allParametersRead = nodeHandle_.getParam("file_path", filePath_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("topic", polygonMeshTopic_) && allParametersRead;
  allParametersRead = nodeHandle_.getParam("frame", pointCloudFrameId_) && allParametersRead;

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

    pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    pcl::io::loadPolygonFilePLY(filePath, *mesh);
    pcl_msgs::PolygonMesh pcl_msg_mesh;
    pcl_conversions::fromPCL(*mesh, polygonMeshMessage_);

  } else {
    ROS_ERROR_STREAM("Data format not supported.");
    return false;
  }

  polygonMeshMessage_.header.frame_id = pointCloudFrameId;
  return true;
}


bool Read::readFileDebug(const std::string& filePath, const std::string& pointCloudFrameId) {
    std::cout << "Load cloud from: " << filePath << std::endl;
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PolygonMesh polymesh;
    std::cout << "Loading...\n";
    pcl::io::loadPolygonFilePLY(filePath, polymesh);
    std::cout << "loaded." << std::endl;

    // Load .ply file.
    //pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud; // try PointXYZ instead of PointXYZRGBNormal
    auto pointCloud2 = polymesh.cloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud;
    pcl::fromPCLPointCloud2( pointCloud2, *pointCloud);
    std::cout << "converted." << std::endl;
    return true;
}


void Read::timerCallback(const ros::TimerEvent& /*timerEvent*/) {
  if (!publish()) {
    ROS_ERROR("Something went wrong when trying to read and publish the point cloud file.");
  }
}


bool Read::readServiceCallback(std_srvs::Trigger::Request &req,
                         std_srvs::Trigger::Response &res){
    if(!readFileDebug(filePath_, pointCloudFrameId_)){
        ROS_ERROR("Something went wrong when trying to read the point cloud file.");
    }
    if (!publish()) {
      ROS_ERROR("Something went wrong when trying to publish the point cloud file.");
    }
};


bool Read::publish() {
    polygonMeshMessage_.header.stamp = ros::Time::now();
    if (polygonMeshPublisher_.getNumSubscribers() > 0u) {
        std::cout << "Message published with " << polygonMeshMessage_.polygons.size() << " polygons, "
                     << polygonMeshMessage_.cloud.height << " height of cloud, "
                     << polygonMeshMessage_.cloud.width << " width of cloud." << std::endl;

      polygonMeshPublisher_.publish(polygonMeshMessage_);
      ROS_INFO_STREAM("Polygon mesh published to topic \"" << polygonMeshTopic_ << "\".");
    }
  return true;
}
}  // namespace point_cloud_io
