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

// defined in miliseconds
#define RECONNECT_TIMER_INTERVAL 1000

using namespace protobuf_comm;

LLSFRefBoxCommunicator::LLSFRefBoxCommunicator(std::string host, int recv_port)
  : quit_(false), reconnect_timer_(io_service_), try_reconnect_(true){
    cfg_refbox_host_ = host;
    cfg_refbox_port_ = recv_port;
    client = new ProtobufStreamClient();

    ros::NodeHandle nh;

    pub_gameinfo = nh.advertise<rcll_vis_msgs::GameInfo>("refbox/gameinfo", 10);
    pub_products = nh.advertise<rcll_vis_msgs::Products>("refbox/products", 10);
    pub_machines = nh.advertise<rcll_vis_msgs::Machines>("refbox/machines", 10);
    pub_robots = nh.advertise<rcll_vis_msgs::Robots>("refbox/robots", 10);
}


LLSFRefBoxCommunicator::~LLSFRefBoxCommunicator() {
    quit_ = true;
    io_service_.stop();
    try_reconnect_ = false;

    reconnect_timer_.cancel();

    delete client;
    client = 0;
}

bool LLSFRefBoxCommunicator::initMachine(const std::string name, const std::string type, int const team_color, rcll_vis_msgs::Machine &machine){
    if (type == "BS"){
        machine.name_short = "BS";
        machine.name_long = "Base Station";
    } else if (type == "DS"){
        machine.name_short = "DS";
        machine.name_long = "Delivery Station";
    } else if (type == "SS"){
        machine.name_short = "SS";
        machine.name_long = "Storage Station";
    } else if (type == "RS"){
        if (name.find("1") != std::string::npos){
            machine.name_short = "RS1";
            machine.name_long = "Ring Station 1";
        } else if (name.find("2") != std::string::npos){
            machine.name_short = "RS2";
            machine.name_long = "Ring Station 2";
        } else {
            return false;
        }
    } else if (type == "CS"){
        if (name.find("1") != std::string::npos){
            machine.name_short = "CS1";
            machine.name_long = "Cap Station 1";
        } else if (name.find("2") != std::string::npos){
            machine.name_short = "CS2";
            machine.name_long = "Cap Station 2";
        } else {
            return false;
        }
    } else {
        return false;
    }

    if (team_color == llsf_msgs::CYAN){
        machine.team = llsf_msgs::CYAN;
    } else if (team_color == llsf_msgs::MAGENTA){
        machine.team = llsf_msgs::MAGENTA;
    } else {
        return false;
    }

    machine.key = machine.name_short;
    return true;
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
            int last_seen_delta = (int)ros::Time::now().toSec() - rob.last_seen().sec();
            if (rob.state() == llsf_msgs::ACTIVE && last_seen_delta < 5.0){
                rcll_vis_msgs::Robot robot;
                robot.key = "R" + std::to_string(robot.robot_id);
                robot.robot_name = rob.name();
                robot.robot_id = rob.number();
                robot.team = rob.team_color();
                robot.active = true;
                robot.status = ""; //TODO
                robot.active_time = 0.0; //TODO
                robot.maintenance_count = rob.maintenance_cycles();
                robot.x = rob.pose().x();
                robot.y = rob.pose().y();
                robot.yaw = rob.pose().ori();
                robot.stamp.data = ros::Time(rob.last_seen().sec(), rob.last_seen().nsec());
                robots_msg.robots.push_back(robot);
            } else {
                ROS_WARN_THROTTLE(5, "Robot '%s' of team %s last seen@%i delta=%i is INACTIVE or timed out", rob.name().c_str(), rob.team().c_str(), (int)rob.last_seen().sec(), last_seen_delta);
            }
        }
        pub_robots.publish(robots_msg);
    }

    std::shared_ptr<llsf_msgs::MachineInfo> minfo;
    if ((minfo = std::dynamic_pointer_cast<llsf_msgs::MachineInfo>(msg))) {
        machines_msg.machines.clear();
        for (int i = 0; i < minfo->machines_size(); i++){
            llsf_msgs::Machine m = minfo->machines(i);
            rcll_vis_msgs::Machine machine;
            if (!initMachine(m.name(), m.type(), m.team_color(), machine)){
                continue;
            }

            if ((int)m.zone() > 1000){
                // magenta
                machine.x = -((double)(((int)m.zone() - 1000) / 10)) + 0.5;
                machine.y = (double)(((int)m.zone() - 1000) % 10) - 0.5;
            } else {
                // cyan
                machine.x = (double)((int)m.zone() / 10) - 0.5;
                machine.y = (double)((int)m.zone() % 10) - 0.5;
            }
            machine.yaw = (int)m.rotation() / 180.0 * M_PI;
            machine.machine_status_exploration1 = (int)m.exploration_zone_state();
            machine.machine_status_exploration2 = (int)m.exploration_rotation_state();
            machine.machine_status_production = m.state();
            machines_msg.machines.push_back(machine);
        }

        pub_machines.publish(machines_msg);
    }

    std::shared_ptr<llsf_msgs::OrderInfo> ordins;
    if ((ordins = std::dynamic_pointer_cast<llsf_msgs::OrderInfo>(msg))) {
        products_msg.orders.clear();
        for (int i = 0; i < ordins->orders_size(); i++){
            llsf_msgs::Order order = ordins->orders(i);
            for (size_t j = 1; j <= order.quantity_requested(); j++){
                rcll_vis_msgs::Product product;
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
    ROS_INFO("Connected comunicator to %s:%i", cfg_refbox_host_.c_str(), cfg_refbox_port_);

#if BOOST_ASIO_VERSION >= 100601
  // Construct a signal set registered for process termination.
  boost::asio::signal_set signals(io_service_, SIGINT, SIGTERM);

  // Start an asynchronous wait for one of the signals to occur.
  signals.async_wait(boost::bind(&LLSFRefBoxCommunicator::handle_signal, this, boost::asio::placeholders::error, boost::asio::placeholders::signal_number));
#endif
  io_service_.run();

  return 0;
}
