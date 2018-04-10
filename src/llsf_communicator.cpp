
/***************************************************************************
 *  shell.h - LLSF RefBox shell
 *
 *  Created: Fri Feb 15 10:22:41 2013
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

#define BOOST_DATE_TIME_POSIX_TIME_STD_CONFIG

#include <llsf_communicator.h>

#include <protobuf_comm/client.h>

#include <llsf_msgs/GameState.pb.h>
#include <llsf_msgs/RobotInfo.pb.h>
#include <llsf_msgs/MachineInfo.pb.h>
#include <llsf_msgs/MachineCommands.pb.h>
#include <llsf_msgs/AttentionMessage.pb.h>
#include <llsf_msgs/OrderInfo.pb.h>
#include <llsf_msgs/VersionInfo.pb.h>
#include <llsf_msgs/GameInfo.pb.h>
#include <llsf_msgs/RobotCommands.pb.h>

#include <cursesp.h>
#include <cursesf.h>
#include <cursesm.h>

#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

#include <cstring>
#include <unistd.h>
#include <modbus/modbus.h>

//#include <libs/llsf_sps/mps_band.h>

// defined in miliseconds
#define TIMER_INTERVAL 500
#define RECONNECT_TIMER_INTERVAL 1000
#define BLINK_TIMER_INTERVAL 250
#define ATTMSG_TIMER_INTERVAL 1000
#define MIN_NUM_ROBOTS 6

using namespace protobuf_comm;


LLSFRefBoxCommunicator::LLSFRefBoxCommunicator()
  : quit_(false), error_(NULL), reconnect_timer_(io_service_), try_reconnect_(true){
    cfg_refbox_host_ = "localhost";
    cfg_refbox_port_ = 4444;
    client = new ProtobufStreamClient();

    ros::NodeHandle nh;

    pub_setmachines = nh.advertise<rcll_msgs::SetMachines>("refbox/set_machines", 10, true);
    pub_gameinfo = nh.advertise<rcll_msgs::GameInfo>("refbox/gameinfo", 10);
    pub_products = nh.advertise<rcll_msgs::Products>("refbox/update_products", 10);
    pub_machinesstatus = nh.advertise<rcll_msgs::MachinesStatus>("refbox/update_machines", 10);
    pub_robots = nh.advertise<rcll_msgs::Robots>("refbox/update_robots", 10);
    pub_setrobot = nh.advertise<rcll_msgs::SetRobot>("refbox/set_robot", 10);

    robot_init_msgs.resize(6);
}


LLSFRefBoxCommunicator::~LLSFRefBoxCommunicator() {
    quit_ = true;
    io_service_.stop();
    try_reconnect_ = false;

    reconnect_timer_.cancel();

    delete client;
    client = 0;
}


const char * LLSFRefBoxCommunicator::error() const {
  return error_;
}


/** Handle operating system signal.
 * @param error error code
 * @param signum signal number
 */
void LLSFRefBoxCommunicator::handle_signal(const boost::system::error_code& error, int signum) {
    reconnect_timer_.cancel();
    io_service_.stop();
}


/** Handle reconnect timer event.
 * @param error error code
 */
void LLSFRefBoxCommunicator::handle_reconnect_timer(const boost::system::error_code& error) {
    if (! error && try_reconnect_ && ! quit_) {
        client->async_connect(cfg_refbox_host_.c_str(), cfg_refbox_port_);
    }
}

void LLSFRefBoxCommunicator::client_connected() {
    //io_service_.dispatch(boost::bind(&LLSFRefBoxCommunicator::refresh, this));
}

void LLSFRefBoxCommunicator::dispatch_client_connected() {
    io_service_.dispatch(boost::bind(&LLSFRefBoxCommunicator::client_connected, this));
}

void LLSFRefBoxCommunicator::client_disconnected(const boost::system::error_code &error) {
    if (! quit_) {

        if (try_reconnect_) {
          reconnect_timer_.expires_from_now(boost::posix_time::milliseconds(RECONNECT_TIMER_INTERVAL));
          reconnect_timer_.async_wait(boost::bind(&LLSFRefBoxCommunicator::handle_reconnect_timer, this, boost::asio::placeholders::error));
        }

    }
    //io_service_.dispatch(boost::bind(&LLSFRefBoxCommunicator::refresh, this));
}

void LLSFRefBoxCommunicator::dispatch_client_disconnected(const boost::system::error_code &error) {
    io_service_.dispatch(boost::bind(&LLSFRefBoxCommunicator::client_disconnected, this, error));
}

void LLSFRefBoxCommunicator::dispatch_client_msg(uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
    io_service_.dispatch(boost::bind(&LLSFRefBoxCommunicator::client_msg, this, comp_id, msg_type, msg));
}

void LLSFRefBoxCommunicator::client_msg(uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
    std::shared_ptr<llsf_msgs::GameState> gstate;
    if ((gstate = std::dynamic_pointer_cast<llsf_msgs::GameState>(msg))) {
        gameinfo_msg.team_name_cyan = gstate->team_cyan();
        gameinfo_msg.team_name_magenta = gstate->team_magenta();
        gameinfo_msg.team_points_cyan = gstate->points_cyan();
        gameinfo_msg.team_points_magenta = gstate->points_magenta();
        gameinfo_msg.game_state = (int)gstate->state();
        gameinfo_msg.game_phase = (int)gstate->phase();
        gameinfo_msg.phase_time = gstate->game_time().sec();
        pub_gameinfo.publish(gameinfo_msg);
    }

    std::shared_ptr<llsf_msgs::RobotInfo> r;
    if ((r = std::dynamic_pointer_cast<llsf_msgs::RobotInfo>(msg))) {
        robots_msg.robots.clear();
        for (int i = 0; i < r->robots_size(); i++){
            llsf_msgs::Robot rob = r->robots(i);
            if (robot_init_msgs[i].robot_name == ""){
                robot_init_msgs[i].index = i;
                robot_init_msgs[i].robot_name = rob.name();
                robot_init_msgs[i].robot_id = rob.number();
                robot_init_msgs[i].team = rob.team_color();
                robot_init_msgs[i].active = true;
                pub_setrobot.publish(robot_init_msgs[i]);
            } else {
                rcll_msgs::Robot robot_update;
                robot_update.index = i;
                robot_update.active = true;
                robot_update.status = ""; //TODO
                robot_update.active_time = 0.0; //TODO
                robot_update.maintenance_count = rob.maintenance_cycles();
                robot_update.x = rob.pose().x();
                robot_update.y = rob.pose().y();
                robot_update.yaw = rob.pose().ori();
                robots_msg.robots.push_back(robot_update);
            }
        }
        pub_robots.publish(robots_msg);
    }

    std::shared_ptr<llsf_msgs::MachineInfo> minfo;
    if ((minfo = std::dynamic_pointer_cast<llsf_msgs::MachineInfo>(msg))) {
        if (machines_init_msg.machines.size() == 0){
            machines_init_msg.machines.resize(minfo->machines_size());
            for (int i = 0; i < minfo->machines_size(); i++){
                llsf_msgs::Machine m = minfo->machines(i);
                rcll_msgs::MachineInit m_init;
                if (m.type() == "BS"){
                    m_init.name_short = "BS";
                    m_init.name_long = "Base Station";
                    m_init.index = 0;
                } else if (m.type() == "DS"){
                    m_init.name_short = "DS";
                    m_init.name_long = "Delivery Station";
                    m_init.index = 1;
                } else if (m.type() == "SS"){
                    m_init.name_short = "SS";
                    m_init.name_long = "Storage Station";
                    m_init.index = 2;
                } else if (m.type() == "RS"){
                    if (m.name().find("1") != std::string::npos){
                        m_init.name_short = "RS1";
                        m_init.name_long = "Ring Station 1";
                        m_init.index = 3;
                    } else if (m.name().find("2") != std::string::npos){
                        m_init.name_short = "RS2";
                        m_init.name_long = "Ring Station 2";
                        m_init.index = 4;
                    } else {
                        continue;
                    }
                } else if (m.type() == "CS"){
                    if (m.name().find("1") != std::string::npos){
                        m_init.name_short = "CS1";
                        m_init.name_long = "Cap Station 1";
                        m_init.index = 5;
                    } else if (m.name().find("2") != std::string::npos){
                        m_init.name_short = "CS2";
                        m_init.name_long = "Cap Station 2";
                        m_init.index = 6;
                    } else {
                        continue;
                    }
                } else {
                    continue;
                }

                if (m.team_color() == llsf_msgs::CYAN){
                    m_init.team = llsf_msgs::CYAN;
                } else if (m.team_color() == llsf_msgs::MAGENTA){
                    m_init.team = llsf_msgs::MAGENTA;
                    m_init.index += 7;
                } else {
                    continue;
                }

                if ((int)m.zone() > 1000){
                    // magenta
                    m_init.x = -((double)(((int)m.zone() - 1000) / 10)) + 0.5;
                    m_init.y = (double)(((int)m.zone() - 1000) % 10) - 0.5;
                } else {
                    // cyan
                    m_init.x = (double)((int)m.zone() / 10) - 0.5;
                    m_init.y = (double)((int)m.zone() % 10) - 0.5;
                }
                m_init.yaw = (int)m.rotation() / 180.0 * M_PI;
                machines_init_msg.machines[m_init.index] = m_init;
            }

            pub_setmachines.publish(machines_init_msg);
        }

        if (machines_init_msg.machines.size() == 14){
            machines_update_msg.machines.clear();
            for (int i = 0; i < minfo->machines_size(); i++){
                llsf_msgs::Machine m = minfo->machines(i);
                rcll_msgs::MachineStatus m_update;
                if (m.type() == "BS"){
                    m_update.index = 0;
                } else if (m.type() == "DS"){
                    m_update.index = 1;
                } else if (m.type() == "SS"){
                    m_update.index = 2;
                } else if (m.type() == "RS"){
                    if (m.name().find("1") != std::string::npos){
                        m_update.index = 3;
                    } else if (m.name().find("2") != std::string::npos){
                        m_update.index = 4;
                    } else {
                        continue;
                    }
                } else if (m.type() == "CS"){
                    if (m.name().find("1") != std::string::npos){
                        m_update.index = 5;
                    } else if (m.name().find("2") != std::string::npos){
                        m_update.index = 6;
                    } else {
                        continue;
                    }
                } else {
                    continue;
                }

                if (m.team_color() == llsf_msgs::CYAN){

                } else if (m.team_color() == llsf_msgs::MAGENTA){

                    m_update.index += 7;
                } else {
                    continue;
                }

                m_update.machine_status_exploration1 = (int)m.exploration_zone_state();
                m_update.machine_status_exploration2 = (int)m.exploration_rotation_state();
                m_update.machine_status_production = m.state();
                machines_update_msg.machines.push_back(m_update);
            }

            pub_machinesstatus.publish(machines_update_msg);
        }
    }

    std::shared_ptr<llsf_msgs::OrderInfo> ordins;
    if ((ordins = std::dynamic_pointer_cast<llsf_msgs::OrderInfo>(msg))) {
        products_msg.orders.clear();
        for (int i = 0; i < ordins->orders_size(); i++){
            llsf_msgs::Order order = ordins->orders(i);
            for (size_t j = 1; j <= order.quantity_requested(); j++){
                rcll_msgs::Product product;
                product.product_id = (int)order.id();
                product.quantity_id = j;
                product.complexity = (int)order.complexity();
                product.structure.push_back((int)order.base_color());
                for (int k = 0; k < 3; k++){
                    if (k < order.ring_colors_size()){
                        product.structure.push_back(order.ring_colors(k));
                    } else {
                        product.structure.push_back(0);
                    }
                }
                product.structure.push_back((int)order.cap_color());
                for (int j = 0; j < 5; j++){
                    product.step_stati_cyan.push_back(0); //TODO
                    product.step_stati_magenta.push_back(0); //TODO
                }
                product.progress_cyan = 0.0; //TODO
                product.progress_magenta = 0.0; //TODO
                product.end_delivery_time = (int)order.delivery_period_end();
                product.points_cyan = 0; //TODO
                product.points_magenta = 0; //TODO
                product.points_max = 0; //TODO
                products_msg.orders.push_back(product);
            }
        }
        pub_products.publish(products_msg);
    }

    std::shared_ptr<llsf_msgs::VersionInfo> vi;
    if ((vi = std::dynamic_pointer_cast<llsf_msgs::VersionInfo>(msg))) {
        ROS_INFO("Received VersionInfo: %i %i %i", (int)vi->version_major(), (int)vi->version_minor(), (int)vi->version_micro());
    }
}

int LLSFRefBoxCommunicator::run() {
    //cfg_refbox_host_ = config_->get_string("/llsfrb/shell/refbox-host");
    //cfg_refbox_port_ = config_->get_uint("/llsfrb/shell/refbox-port");

    MessageRegister & message_register = client->message_register();
    message_register.add_message_type<llsf_msgs::GameState>();
    message_register.add_message_type<llsf_msgs::RobotInfo>();
    message_register.add_message_type<llsf_msgs::MachineInfo>();
    message_register.add_message_type<llsf_msgs::AttentionMessage>();
    message_register.add_message_type<llsf_msgs::OrderInfo>();
    message_register.add_message_type<llsf_msgs::VersionInfo>();
    message_register.add_message_type<llsf_msgs::GameInfo>();

    client->signal_connected().connect(boost::bind(&LLSFRefBoxCommunicator::dispatch_client_connected, this));
    client->signal_disconnected().connect(boost::bind(&LLSFRefBoxCommunicator::dispatch_client_disconnected, this, boost::asio::placeholders::error));
    client->signal_received().connect(boost::bind(&LLSFRefBoxCommunicator::dispatch_client_msg, this, _1, _2, _3));

    client->async_connect(cfg_refbox_host_.c_str(), cfg_refbox_port_);
    ROS_INFO("connected to %s %i", cfg_refbox_host_.c_str(), cfg_refbox_port_);

#if BOOST_ASIO_VERSION >= 100601
  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service_, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(boost::bind(&LLSFRefBoxCommunicator::handle_signal, this, boost::asio::placeholders::error, boost::asio::placeholders::signal_number));
#endif
  io_service_.run();

  return 0;
}
