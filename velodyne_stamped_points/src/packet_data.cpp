//
// Created by yang on 2020/4/14.
//
// Original author
/*
 * packet_data.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: bmacallister
 */

#include <velodyne_stamped_points/packet_data.h>

namespace velodyne_rawdata
{

    PacketData_Converter::PacketData_Converter()
    {
        // TODO Auto-generated constructor stub

    }

    PacketData_Converter::~PacketData_Converter()
    {
        // TODO Auto-generated destructor stub
    }

} /* namespace velodyne_rawdata */

void velodyne_rawdata::PacketData_Converter::unpackVPL16ToStampedMessage(const velodyne_msgs::VelodynePacket &pkt,
                                                                             stamped_scan_msgs::Scan & scan)
{

  float azimuth;
  float azimuth_diff;
  float last_azimuth_diff = 0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];

  long header_time = pkt.stamp.toNSec();

  for (int block = 0; block < BLOCKS_PER_PACKET; block++)
  {

    // ignore packets with mangled or otherwise different contents
    if (UPPER_BANK != raw->blocks[block].header)
    {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per minute.
      ROS_WARN_STREAM_THROTTLE(
          60, "skipping invalid VLP-16 packet: block " << block << " header value is " << raw->blocks[block].header);
      return;                         // bad packet: skip the rest
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float)(raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - 1))
    {
      azimuth_diff = (float)((36000 + raw->blocks[block + 1].rotation - raw->blocks[block].rotation) % 36000);
      last_azimuth_diff = azimuth_diff;
    }
    else
    {
      azimuth_diff = last_azimuth_diff;
    }

    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
    {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE)
      {
        velodyne_pointcloud::LaserCorrection &corrections = calibration_.laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f = azimuth
            + (azimuth_diff * ((dsr * VLP16_DSR_TOFFSET) + (firing * VLP16_FIRING_TOFFSET)) / VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;

        /*condition added to avoid calculating points which are not
         in the interesting defined area (min_angle < area < max_angle)*/
        if ((azimuth_corrected >= config_.min_angle && azimuth_corrected <= config_.max_angle
            && config_.min_angle < config_.max_angle)
            || (config_.min_angle > config_.max_angle
                && (azimuth_corrected <= config_.max_angle || azimuth_corrected >= config_.min_angle)))
        {

          // convert polar coordinates to Euclidean XYZ
          float distance = tmp.uint * DISTANCE_RESOLUTION;
          distance += corrections.dist_correction;

          float cos_vert_angle = corrections.cos_vert_correction;
          float sin_vert_angle = corrections.sin_vert_correction;
          float cos_rot_correction = corrections.cos_rot_correction;
          float sin_rot_correction = corrections.sin_rot_correction;

          // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
          // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
          float cos_rot_angle = cos_rot_table_[azimuth_corrected] * cos_rot_correction
              + sin_rot_table_[azimuth_corrected] * sin_rot_correction;
          float sin_rot_angle = sin_rot_table_[azimuth_corrected] * cos_rot_correction
              - cos_rot_table_[azimuth_corrected] * sin_rot_correction;

          float horiz_offset = corrections.horiz_offset_correction;
          float vert_offset = corrections.vert_offset_correction;

          // Compute the distance in the xy plane (w/o accounting for rotation)
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

          // Calculate temporal X, use absolute value.
          float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
          // Calculate temporal Y, use absolute value
          float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
          if (xx < 0)
            xx = -xx;
          if (yy < 0)
            yy = -yy;

          // Get 2points calibration values,Linear interpolation to get distance
          // correction for X and Y, that means distance correction use
          // different value at different distance
          float distance_corr_x = 0;
          float distance_corr_y = 0;
          if (corrections.two_pt_correction_available)
          {
            distance_corr_x = (corrections.dist_correction - corrections.dist_correction_x) * (xx - 2.4) / (25.04 - 2.4)
                + corrections.dist_correction_x;
            distance_corr_x -= corrections.dist_correction;
            distance_corr_y = (corrections.dist_correction - corrections.dist_correction_y) * (yy - 1.93)
                / (25.04 - 1.93) + corrections.dist_correction_y;
            distance_corr_y -= corrections.dist_correction;
          }

          float distance_x = distance + distance_corr_x;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
          x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

          float distance_y = distance + distance_corr_y;
          /**the new term of 'vert_offset * sin_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
          y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

          // Using distance_y is not symmetric, but the velodyne manual
          // does this.
          /**the new term of 'vert_offset * cos_vert_angle'
           * was added to the expression due to the mathemathical
           * model we used.
           */
          z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

          /** Use standard ROS coordinate system (right-hand rule) */
          float x_coord = y;
          float y_coord = -x;
          float z_coord = z;

//          /** Retrieve calibrated polar coordinates  Keeps corrections made from calibration **/
//          double calibrated_radius = sqrt((x_coord * x_coord) + (y_coord * y_coord) +(z_coord * z_coord));
//          double calibrated_azimuth = atan2(y_coord, x_coord);
//          double calibrated_polar = 0;
//          if (calibrated_radius != 0.0)
//          {
//            calibrated_polar = acos(z_coord / calibrated_radius);
//          }

          /** Calculate time of laser */
          ros::Duration d;
          size_t seq_index = (block * BLOCKS_PER_PACKET) + firing;
          size_t data_index = dsr;
          long time_offset = (55296 * seq_index) + (2304 * data_index);
          d.fromNSec(time_offset);
          ros::Time t = pkt.stamp + d;

          /** Intensity Calculation */
          float min_intensity = corrections.min_intensity;
          float max_intensity = corrections.max_intensity;

          intensity = raw->blocks[block].data[k + 2];

          float focal_offset = 256 * (1 - corrections.focal_distance / 13100)
              * (1 - corrections.focal_distance / 13100);
          float focal_slope = corrections.focal_slope;
          intensity += focal_slope * (abs(focal_offset - 256 * (1 - tmp.uint / 65535) * (1 - tmp.uint / 65535)));
          intensity = (intensity < min_intensity) ? min_intensity : intensity;
          intensity = (intensity > max_intensity) ? max_intensity : intensity;

          if (pointInRange(distance))
          {

            // append this point to the scan
            stamped_scan_msgs::Point point;
            point.position.x = x_coord;
            point.position.y = y_coord;
            point.position.z = z_coord;
            point.intensity = intensity;
            point.time_stamp = t;
            scan.points.push_back(point);
          }
        }
      }
    }
  }
}
