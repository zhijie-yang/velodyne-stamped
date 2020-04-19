//
// Created by yang on 2020/4/14.
//
// Original author
/*
 * packet_data.h
 *
 *  Created on: Nov 15, 2017
 *      Author: bmacallister
 *
 *  Note:  This only works for a specific model
 */


#ifndef _SRC_PACKET_DATA_H
#define _SRC_PACKET_DATA_H

#include <velodyne_stamped_points/rawdata.h>
#include <stamped_scan_msgs/Scan.h>

namespace velodyne_rawdata
{

    class PacketData_Converter : public RawData
    {
    public:
        PacketData_Converter();

        virtual ~PacketData_Converter();

        /**
         * \brief Converts raw velodyne packets to stamped scan message;  Comes from unpack_vlp16 function
         * @param pkt
         * @param scan
         */
        void unpackVPL16ToStampedMessage(const velodyne_msgs::VelodynePacket &pkt, stamped_scan_msgs::Scan & scan);
    };

} /* namespace velodyne_rawdata */

#endif //_SRC_PACKET_DATA_H
