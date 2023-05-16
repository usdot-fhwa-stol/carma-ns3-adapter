#include <iostream>
#include <functional>
#include "ns3_reg_client.h"

NS3RegClient::NS3RegClient() :
    running_(false)
{}


NS3RegClient::~NS3RegClient() {
    try{
        close();
    }catch(...){}
}

bool NS3RegClient::registermsg(const std::shared_ptr<std::vector<uint8_t>>&message, const std::string &remote_address, unsigned short remote_port,
                            unsigned short local_port)
{
    bool success = false;
    
    //if (connect(remote_address, remote_port, local_port))
    //{
        bool send_success = sendNS3Message(message);
        if (send_success)
        { 
            // ROS_DEBUG_STREAM("Handshake Message sent successfully");
            success = true;
        }
        // else ROS_DEBUG_STREAM("Handshake Message send failed");
    //}
    // else ROS_DEBUG_STREAM( "Connection failed" );
    //else std::cerr << "Connection failed" << std::endl;


    // close();
    
    return success;
}


bool NS3RegClient::connect(const std::string &remote_address, unsigned short remote_port) {
    boost::system::error_code ignored_ec;
    return connect(remote_address, remote_port, ignored_ec);
}

bool NS3RegClient::connect(const std::string &remote_address,
                                unsigned short remote_port,
                                 boost::system::error_code &ec)
{
    //If we are already connected return false
    if(running_) return false;
    //Get remote endpoint
    try
    {
        remote_udp_ep_ = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(remote_address),remote_port);
    }catch(std::exception e)
    {
        ec = boost::asio::error::invalid_argument;
        throw e;
    }

    ec.clear(); 

    io_.reset(new boost::asio::io_service());
    output_strand_.reset(new boost::asio::io_service::strand(*io_));

    udp_out_socket_.reset(new boost::asio::ip::udp::socket(*io_,remote_udp_ep_.protocol()));
    work_.reset(new boost::asio::io_service::work(*io_));
    io_thread_.reset(new std::thread([this]()
                                     {
                                         boost::system::error_code err;
                                         io_->run(err);
                                         if(err)
                                         {
                                             onError(err);
                                         }
                                     }));
    running_ = true;
    onConnect();
    return true;
}

void NS3RegClient::close() {
    if(!running_) return;
    running_ = false;
    work_.reset();
    io_->stop();
    io_thread_->join();
    udp_out_socket_.reset();
    onDisconnect();
}


bool NS3RegClient::sendNS3Message(const std::shared_ptr<std::vector<uint8_t>>&message) {
    if(!running_) return false;
    try {
        output_strand_->post([this,message]()
                             {
                                 try
                                 {
                                     udp_out_socket_->send_to(boost::asio::buffer(*message), remote_udp_ep_);
                                 }
                                 catch(boost::system::system_error error_code)
                                 {
                                     onError(error_code.code());
                                 }
                                 catch(...)
                                 {
                                     onError(boost::asio::error::fault);
                                 }
                             });
        return true;
    }
    catch (std::exception& e) {
        return false;
    }
}
