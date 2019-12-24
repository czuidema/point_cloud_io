/*
 * Write.hpp
 *
 *  Created on: Nov 13, 2015
 *      Author: Remo Diethelm
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <radiation_srvs/MeshInfo.h>
#include <pcl_msgs/PolygonMesh.h>

namespace point_cloud_io {

class Write {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit Write(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~Write() = default;

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Point cloud callback function
   * @param cloud point cloud message.
   */
  void polygonCallback(const pcl_msgs::PolygonMesh& polygon);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Point cloud subscriber.
  ros::Subscriber polygonSubscriber_;

  //! Point cloud topic to subscribe to.
  std::string polygonMeshTopic_;

  //! Path to the point cloud folder.
  std::string folderPath_;

  //! Point cloud file prefix.
  std::string fileName_;

  //! Point cloud file ending.
  std::string fileEnding_;

  std::string filePathComplete_;

  //! Point cloud counter.
  unsigned int counter_ = 0;
};

}  // namespace point_cloud_io
