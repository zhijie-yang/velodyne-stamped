//
// Created by yang on 2020/4/14.
//
// Original author
/*
 * velodyne_spherical_laserscan_node.h
 *
 *  Created on: Nov 30, 2017
 *      Author: bmacallister
 */

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <ros/ros.h>

#include <velodyne_stamped_points/packet_data.h>
#include <stamped_scan_msgs/Scan.h>
#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

namespace velodyne_pointcloud
{
class Convert
{
public:

  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert()
  {
  }

private:

  void callback(velodyne_pointcloud::CloudNodeConfig &config, uint32_t level);

  void processScanPackets(const velodyne_msgs::VelodyneScan::ConstPtr &scan_packet_msg);

  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> > srv_;
  boost::shared_ptr<velodyne_rawdata::PacketData_Converter> packet_data_converter_;
  ros::Subscriber velodyne_scan_;
  ros::Publisher output_;

};

} // namespace velodyne_pointcloud

#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
