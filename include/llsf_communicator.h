
/***************************************************************************
 *  shell.h - LLSF RefBox shell
 *
 *  Created: Fri Feb 15 10:12:50 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <rcll_msgs/GameInfo.h>
#include <rcll_msgs/MachinesStatus.h>
#include <rcll_msgs/Robots.h>
#include <rcll_msgs/Products.h>
#include <rcll_msgs/SetMachines.h>
#include <rcll_msgs/SetRobot.h>
#include <rcll_msgs/SetGameField.h>

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
}

class LLSFRefBoxCommunicator{
    public:
        LLSFRefBoxCommunicator();
        ~LLSFRefBoxCommunicator();

        const char * error() const;

        int run();

    private: // methods
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
          const char *error_;

          boost::asio::io_service      io_service_;
          boost::asio::deadline_timer  reconnect_timer_;
          bool                         try_reconnect_;

          protobuf_comm::ProtobufStreamClient *client;

          std::string  cfg_refbox_host_;
          unsigned int cfg_refbox_port_;

          std::vector<rcll_msgs::SetRobot> robot_init_msgs;
          rcll_msgs::SetMachines machines_init_msg;
          rcll_msgs::MachinesStatus machines_update_msg;
          rcll_msgs::GameInfo gameinfo_msg;
          rcll_msgs::Products products_msg;
          rcll_msgs::Robots robots_msg;

          ros::Publisher pub_setmachines;
          ros::Publisher pub_machinesstatus;
          ros::Publisher pub_gameinfo;
          ros::Publisher pub_products;
          ros::Publisher pub_robots;
          ros::Publisher pub_setrobot;
};

#endif
