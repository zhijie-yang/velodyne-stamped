//
// Created by yang on 2020/4/14.
//
// Original author
/*
 * velodyne_spherical_laserscan_node.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: bmacallister
 *
 * NOTE:This file was adapted from velodyne_pointcloud/cloude_node
 */

#include <velodyne_stamped_points/velodyne_stamped_points_node.h>

namespace velodyne_pointcloud
{
/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :
    packet_data_converter_(new velodyne_rawdata::PacketData_Converter())
{
  //Setup the converter from ros params
  packet_data_converter_->setup(private_nh);
  // Advertise stamped scan
  output_ = node.advertise<stamped_scan_msgs::Scan>("velodyne_stamped_scan", 10);


  //dynamic reconfigure
  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  // subscribe to VelodyneScan packets
  velodyne_scan_ = node.subscribe("velodyne_packets", 10, &Convert::processScanPackets, (Convert *)this,
                                  ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config, uint32_t level)
{
  //Set converter dynamic reconfigure paramss
  packet_data_converter_->setParameters(config.min_range, config.max_range, config.view_direction, config.view_width);
}

/** @brief Callback for raw scan messages. */
void Convert::processScanPackets(const velodyne_msgs::VelodyneScan::ConstPtr &msg)
{
  //Process scan
  velodyne_msgs::VelodyneScan scan_packet = *msg;
  stamped_scan_msgs::Scan scan_msg;
  scan_msg.header = scan_packet.header;

  for (const auto & pkt : scan_packet.packets)
  {
    packet_data_converter_->unpackVPL16ToStampedMessage(pkt, scan_msg);
  }

  // Publish
  output_.publish(scan_msg);
}

}

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  //Setup node
  velodyne_pointcloud::Convert conv(node, priv_nh);
  ros::spin();

  return 0;
}
