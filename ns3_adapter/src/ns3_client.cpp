#include <iostream>
#include <functional>
#include "ns3_client.h"
#include <ros/ros.h>

NS3Client::NS3Client() :
    running_(false)
{}


NS3Client::~NS3Client() {
    try{
        close();
    }catch(...){}
}

bool NS3Client::registermsg(const std::shared_ptr<std::vector<uint8_t>>&message)
{
    return sendNS3Message(message);
}


bool NS3Client::connect(const std::string &remote_address, unsigned short remote_port) {
    boost::system::error_code ignored_ec;
     //If we are already connected return false
    if(running_) return false;
    //Get remote endpoint
    try
    {
        remote_udp_ep_ = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(remote_address),remote_port);
    }catch(std::exception e)
    {
        ignored_ec = boost::asio::error::invalid_argument;
        throw e;
    }

    ignored_ec.clear();

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


bool NS3Client::connect(const std::string &remote_address,
                                unsigned short remote_port,
                                 unsigned short local_v2x_port,
                                 unsigned short local_time_port,
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

    //V2X RECEIVER: build the udp listener, this class listens on address::port and sends packets through onReceive
    try
    {
        if(udp_v2x_listener_){
            udp_v2x_listener_.reset(nullptr);

        }
        udp_v2x_listener_.reset(new cav::UDPListener(*io_,local_v2x_port));
    }catch(boost::system::system_error e)
    {
        ec = e.code();
        ROS_ERROR_STREAM("NS3Client::connect on v2x port threw system_error : " << e.what());
        return false;
    }
    catch(std::exception e)
    {
        ROS_ERROR_STREAM("NS3Client::connect on v2x port threw exception : " << e.what());
        return false;
    };

    //connect signals
    udp_v2x_listener_->onReceive.connect([this](const std::shared_ptr<const std::vector<uint8_t>>& data){process(data);});

    //TIME RECEIVER: build the udp listener, this class listens on address::port and sends packets through onReceive
    try
    {
        if(udp_time_listener_){
            udp_time_listener_.reset(nullptr);

        }
        udp_time_listener_.reset(new cav::UDPListener(*io_,local_time_port));
    }catch(boost::system::system_error e)
    {
        ec = e.code();
        ROS_ERROR_STREAM("NS3Client::connect on time port threw system_error : " << e.what());
        return false;
    }
    catch(std::exception e)
    {
        ROS_ERROR_STREAM("NS3Client::connect on time port threw exception : " << e.what());
        return false;
    };

    //connect signals
    udp_time_listener_->onReceive.connect([this](const std::shared_ptr<const std::vector<uint8_t>>& data){process_time(data);});

    udp_out_socket_.reset(new boost::asio::ip::udp::socket(*io_,remote_udp_ep_.protocol()));
    work_.reset(new boost::asio::io_service::work(*io_));
    // run the io service
    udp_v2x_listener_->start();
    udp_time_listener_->start();

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

void NS3Client::close() {
    if(!running_) return;
    running_ = false;
    work_.reset();
    io_->stop();
    io_thread_->join();
    udp_v2x_listener_->stop();
    udp_time_listener_->stop();
    udp_out_socket_.reset();
    onDisconnect();
}

void NS3Client::process_time(const std::shared_ptr<const std::vector<uint8_t>>& data)
{
    const std::vector<uint8_t> vec = *data;
    std::string json_string(vec.begin(), vec.end());
    // JSON
    rapidjson::Document obj;
    std::string timestep_member_name = "timestep";
    obj.Parse(json_string);
    if (obj.HasParseError())
    {
        // TODO: Change to json_document_parse_exception. Requires changes to services and unit tests
        throw std::runtime_error("Message JSON is misformatted. JSON parsing failed!");
    }

    std::optional<unsigned long> result;
    if (obj.HasMember(timestep_member_name.c_str()) && obj.FindMember(timestep_member_name.c_str())->value.IsUint64())
    {
        result = obj[timestep_member_name.c_str()].GetUint64();
    }

    onTimeReceived(result.value());
}

void NS3Client::process(const std::shared_ptr<const std::vector<uint8_t>>& data)
{
    auto & entry = *data;
    // Valid message should begin with 2 bytes message ID and 1 byte length.
    for (size_t i = 0; i < entry.size() - 3; i++) { // leave 3 bytes after (for lsb of id, length byte 1, and either message body or length byte 2)
        // Generate the 16-bit message id from two bytes, skip if it isn't a valid one
        uint16_t msg_id = (static_cast<uint16_t>(entry[i]) << 8) | static_cast<uint16_t>(entry[i + 1]);

        // Parse the length, check it doesn't run over
        size_t len = 0;
        size_t len_byte_1 = entry[i + 2];
        int len_bytes = 0;
        // length < 128 encoded by single byte with msb set to 0
        if ((len_byte_1 & 0x80 ) == 0x00) {
            len = static_cast<size_t>(len_byte_1);
            len_bytes = 1;
            // check for 0 length
            if (len_byte_1 == 0x00) { continue; }
        }
            // length < 16384 encoded by 14 bits in 2 bytes (10xxxxxx xxxxxxxx)
        else if ((len_byte_1 & 0x40) == 0x00) { //we know msb = 1, check that next bit is 0
            if (i + 3 < entry.size()) {
                size_t len_byte_2 = entry[i + 3];
                len = ((len_byte_1 & 0x3f) << 8) | len_byte_2;
                len_bytes = 2;
            }
        }
        else {
            // TODO lengths greater than 16383 (0x3FFF) are encoded by splitting up the message into discrete chunks, each with its own length
            // marker. It doesn't look like we'll be receiving anything that long
            std::cerr << "NS3Client::process() : received a message with length field longer than 16383." << std::endl;
            continue;
        }
        if (len == -1) { continue; }
        // If the length makes sense bsmPub(fits in the buffer), copy out the message bytes and pass to the Application class
        if ((i + 1 + len + len_bytes) < entry.size()) {
            // bool found_valid_msg = true;
            size_t start_index = i;
            size_t end_index = i + 2 + len + len_bytes; // includes 2 msgID bytes before message body
            //this constructor has range [first, last) hence the + 1
            std::vector<uint8_t> msg_vec(entry.begin() + start_index, entry.begin() + end_index);
            onMessageReceived(msg_vec, msg_id);
            break;
        }
    }
}

bool NS3Client::sendNS3Message(const std::shared_ptr<std::vector<uint8_t>>&message) {
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
