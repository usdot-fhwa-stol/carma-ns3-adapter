#pragma once

/*
 * Copyright (C) 2019-2024 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <boost/asio.hpp>
#include <boost/signals2/signal.hpp>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <queue>
#include <vector>
#include <rapidjson/document.h>
#include <rapidjson/rapidjson.h>
#include <gtest/gtest.h>

#include "udp_listener.h"

class NS3Client
{

public:
    /**
     * @brief Initializes the NS-3 client, sets up i/o services
     */
    NS3Client();
    ~NS3Client();

    /**
    * @brief Connects the driver to the OBU at the provided IPv4 address and Port
    * @param address IPv4 address of OBU
    * @param remote_v2x_broadcasting_port of server's v2x service
    * @param remote_registration_port of server's registration service
    * @param local_v2x_port of client v2x service
    * @param local_time_port of client time service
    * @param ec error code set during connect
    * @return true on sucessful connect, false otherwise
    */
    bool connect_registration_and_broadcasting(const std::string &remote_address,unsigned short remote_v2x_broadcasting_port,unsigned short remote_registration_port,
                        unsigned short local_v2x_port, unsigned short local_time_port, boost::system::error_code &ec);

    /**
    * @brief Closes connection
    *
    * closes all sockets, and stops all threads used for io and processing, blocks until threads are closed
    */
    void close();

    /**
    * @brief returns connected state
    * @return returns true if connected, false otherwise
    */
    inline bool connected() { return running_; }

    /**
    * @brief Signaled when client is connected to the NS-3 adapter
    */
    boost::signals2::signal<void()> onConnect;

    /**
    * @brief Signaled when client is disconnected from hardware
    */
    boost::signals2::signal<void()> onDisconnect;

    /**
     * @brief Signaled when client experiences a fatal error. Client should be closed and reconnected
     */
    boost::signals2::signal<void(const boost::system::error_code&)> onError;

    /**
    * @brief Signaled when message received
    */
    boost::signals2::signal<void(std::vector<uint8_t> const &, uint16_t id)> onMessageReceived;

    /**
    * @brief Signaled when time received
    */
    boost::signals2::signal<void(unsigned long)> onTimeReceived;

    /**
     * @brief sends a udp v2x message
     */
    bool send_ns3_message(const std::shared_ptr<std::vector<uint8_t>>&message);

    /**
     * @brief sends a udp registration message
     */
    bool send_registration_message(const std::shared_ptr<std::vector<uint8_t>>&message);

    FRIEND_TEST(NS3Adapter, testNS3ClientSSM);

    FRIEND_TEST(NS3Adapter, testNS3ClientSRM);


private:
    std::unique_ptr<boost::asio::io_service> io_;
    std::unique_ptr<boost::asio::io_service::strand> output_strand_;
    std::shared_ptr<boost::asio::io_service::work> work_;

    std::shared_ptr<std::thread> io_thread_;
    volatile bool running_;

    //udp
    std::unique_ptr<boost::asio::ip::udp::socket> udp_out_registration_socket_;
    std::unique_ptr<boost::asio::ip::udp::socket> udp_out_v2x_msg_broadcasting_socket_;
    boost::asio::ip::udp::endpoint remote_udp_ep_registration_;
    boost::asio::ip::udp::endpoint remote_udp_ep_v2x_msg_broadcasting_;


    std::unique_ptr<cav::UDPListener> udp_v2x_listener_;
    std::unique_ptr<cav::UDPListener> udp_time_listener_;

    /**
    * @brief maintains the process thread
    *
    * This will parse through the incoming UDP packets to find what looks like a
    * valid J2735 message. The UPER scheme makes it hard to definitively know if
    * a message is valid or just noise, so all possible messages from a packet are
    * sent. This will create some false positives, but will ensure that a falsely
    * identified message doesn't keep a real one from getting through.
    */
    void process(const std::shared_ptr<const std::vector<uint8_t>> &data);

    /**
    * @brief maintains the process_time thread
    *
    * This will parse the UDP packet to get json document about the current simulation time
    * coming from mosaic
    */
    void process_time(const std::shared_ptr<const std::vector<uint8_t>> &data);
};