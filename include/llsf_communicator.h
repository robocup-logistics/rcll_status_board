/*
The MIT License (MIT)

Copyright (c) 2017-2018 Florian Eith <florian.eith@web.de>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __LLSF_COMMUNICATOR_H_
#define __LLSF_COMMUNICATOR_H_

#include <boost/asio.hpp>
#include <boost/date_time.hpp>
#include <google/protobuf/message.h>

#include <map>
#include <vector>
#include <string>
#include <mutex>

#include <rcll_vis_msgs/GameInfo.h>
#include <rcll_vis_msgs/Machines.h>
#include <rcll_vis_msgs/Robots.h>
#include <rcll_vis_msgs/Products.h>
#include <rcll_vis_msgs/SetGameField.h>

#include <ros/ros.h>

namespace protobuf_comm {
    class ProtobufStreamClient;
}

namespace llsf_msgs {
    class MachineInfo;
    class GameInfo;
    class RobotInfo;
    class OrderInfo;
    class GameState;
    class BeaconSignal;
}

class LLSFRefBoxCommunicator{
    public:
        LLSFRefBoxCommunicator(std::string host, int recv_port);
        ~LLSFRefBoxCommunicator();

        int run();

    private: // methods
        bool initMachine(const std::string name, const std::string type, const int team_color, rcll_vis_msgs::Machine &machine);

        void handle_reconnect_timer(const boost::system::error_code& error);
        void handle_signal(const boost::system::error_code& error, int signum);

        void dispatch_client_connected();
        void dispatch_client_disconnected(const boost::system::error_code &error);
        void dispatch_client_msg(uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);

        void client_connected();
        void client_disconnected(const boost::system::error_code &error);
        void client_msg(uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);

    private: // members
          bool        quit_;

          boost::asio::io_service      io_service_;
          boost::asio::deadline_timer  reconnect_timer_;
          bool                         try_reconnect_;

          protobuf_comm::ProtobufStreamClient *client;

          std::string  cfg_refbox_host_;
          unsigned int cfg_refbox_port_;

          rcll_vis_msgs::Machines machines_msg;
          rcll_vis_msgs::GameInfo gameinfo_msg;
          rcll_vis_msgs::Products products_msg;
          rcll_vis_msgs::Robots robots_msg;

          ros::Publisher pub_machines;
          ros::Publisher pub_gameinfo;
          ros::Publisher pub_products;
          ros::Publisher pub_robots;
};

#endif
