#include "ns3_adapter.h"

#include <rapidjson/document.h>
#include <rapidjson/istreamwrapper.h>
#include <rapidjson/schema.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <cav_msgs/ByteArray.h>
#include <fstream>
#include <chrono>

std::string NS3Adapter::uint8_vector_to_hex_string(const std::vector<uint8_t>& v) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    std::vector<uint8_t>::const_iterator it;
    for (it = v.begin(); it != v.end(); it++) {
        ss << std::setw(2) << static_cast<unsigned>(*it);
    }

    return ss.str();
}

NS3Adapter::NS3Adapter(int argc, char **argv) : cav::DriverApplication(argc, argv, "ns3")
{
    queue_size_ = 1000;
    cav_msgs::DriverStatus status;
    status.status = cav_msgs::DriverStatus::OFF;
    status.comms = true;
    setStatus(status);
}


void NS3Adapter::initialize() {

    std::string wave_cfg_file;
    ros::NodeHandle pnh("~");
    comms_api_nh_.reset(new ros::NodeHandle("comms"));

    pnh_->param<std::string>("wave_cfg_file",wave_cfg_file,"/opt/carma/install/ns3_adapter/share/ns3_adapter/config/wave.json");
    load_wave_config(wave_cfg_file);

    // Start the handshake

    pnh_->getParam("/vehicle_id", vehicle_id_);
    pnh.param<std::string>("role_id", role_id_, "carma_1");
    pnh.param<std::string>("ns3_address", ns3_address_, "172.2.0.2");
    pnh.param<int>("ns3_registration_port", ns3_registration_port_, 1515);
    pnh.param<int>("ns3_broadcasting_port", ns3_broadcasting_port_, 1516);
    pnh.param<int>("ns3_v2x_listening_port", ns3_v2x_listening_port_, 2500);
    pnh.param<int>("ns3_time_listening_port", ns3_time_listening_port_, 2501);
    pnh.param<std::string>("host_ip", host_ip_, "172.2.0.3");

    //Setup connection handlers
    ns3_client_error_.clear();
    ns3_client_.onConnect.connect([this]() { on_connect_handler(); });
    ns3_client_.onDisconnect.connect([this]() { on_disconnect_handler(); });
    ns3_client_.onError.connect([this](const boost::system::error_code& err){ns3_client_error_ = err;});

    //Setup the ROS API
    std::string node_name = ros::this_node::getName();
    api_.clear();

    //Comms Subscriber
    comms_sub_ = comms_api_nh_->subscribe("outbound_binary_msg", queue_size_, &NS3Adapter::on_outbound_message, this);
    api_.push_back(comms_sub_.getTopic());

    //Comms Publisher
    comms_pub_ = comms_api_nh_->advertise<cav_msgs::ByteArray>("inbound_binary_msg", queue_size_);
    api_.push_back(comms_pub_.getTopic());

    //Time publisher
    time_pub_ = comms_api_nh_->advertise<rosgraph_msgs::Clock>("/sim_clock", queue_size_);
    api_.push_back(time_pub_.getTopic());

    //Comms Service
    comms_srv_ = comms_api_nh_->advertiseService("send", &NS3Adapter::send_message_srv, this);
    api_.push_back(comms_srv_.getService());

    pose_sub_ = pnh_->subscribe("current_pose", 1, &NS3Adapter::pose_cb, this);

    ns3_client_.onMessageReceived.connect([this](std::vector<uint8_t> const &msg, uint16_t id) {on_message_received_handler(msg, id); });
    ns3_client_.onTimeReceived.connect([this](unsigned long timestamp) {on_time_received_handler(timestamp); });

    spin_rate = 10;
}

void NS3Adapter::on_connect_handler() {
    ROS_WARN_STREAM("NS-3 Adapter Connected");
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);
}

void NS3Adapter::on_disconnect_handler() {
    cav_msgs::DriverStatus status = getStatus();
    status.status = cav_msgs::DriverStatus::OFF;
    setStatus(status);
    ROS_WARN_STREAM("NS-3 Adapter Disconnected");
}

void NS3Adapter::on_time_received_handler(unsigned long timestamp)
{
    rosgraph_msgs::Clock time_now;
    // A script to validate time synchronization of tools in CDASim currently relies on the following
    // log line. TODO: This line is meant to be removed in the future upon completion of this work:
    // https://github.com/usdot-fhwa-stol/carma-analytics-fotda/pull/43
    auto chrono_time = std::chrono::system_clock::now();
    auto epoch = chrono_time.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
    auto time_to_milli = static_cast<int>(timestamp / 1e6);
    ROS_DEBUG_STREAM("Simulation Time: " << std::to_string(time_to_milli) << " where current system time is: "
        << std::to_string(milliseconds.count()));
    time_now.clock.sec = static_cast<int>(timestamp / 1e9);
    time_now.clock.nsec = timestamp - time_now.clock.sec * 1e9;

    time_pub_.publish(time_now);
}

/**
* @brief Handles messages received from the NS-3 Client
*
* Populates a ROS message with the contents of the incoming OBU message, and
* publishes to the ROS 'recv' topic.
*/
void NS3Adapter::on_message_received_handler(const std::vector<uint8_t> &data, uint16_t id) {
    // Create and populate the message
    ROS_WARN_STREAM("in on_message_received_handler");

    cav_msgs::ByteArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";
    msg.message_type = get_message_name_from_id(id);
    msg.content = data;
    // Publish it
    comms_pub_.publish(msg);

    ROS_WARN_STREAM("Application received Data: " << data.size() << " bytes, message: " << uint8_vector_to_hex_string(data));
}

std::string NS3Adapter::get_message_name_from_id(uint16_t id)
{
    auto it = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[id](const WaveConfigStruct& entry)
                                                                            {
                                                                               return entry.ns3_id == std::to_string(id);
                                                                            });
    return (it != wave_cfg_items_.end() ? it->name : "Unknown");
}

/**
 * @brief Packs an outgoing message into J2375 standard.
 * @param message
 *
 * This processes an incoming ByteArray message, and packs it according to the
 * J2735 standard.
 *
 * TODO: Right now it doesn't do anything except return the message data (ignoring message type).
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 * Depending on what the input is, that might be all that's necessary, but possibly more.
 */
std::vector<uint8_t> NS3Adapter::pack_message(const cav_msgs::ByteArray& message) {
    std::stringstream ss;
    auto wave_item = std::find_if(wave_cfg_items_.begin(),wave_cfg_items_.end(),[&message](const WaveConfigStruct& entry)
    {
        return entry.name == message.message_type;
    });

    WaveConfigStruct cfg;
    if(wave_item == wave_cfg_items_.end())
    {
        ROS_WARN_STREAM("No wave config entry for type: " << message.message_type << ", using defaults");
        cfg.name = message.message_type;
        cfg.channel = "CCH";  //Assuming the Default channel is not the safety related info that would be in a BSM message
        cfg.priority = "1";
        cfg.ns3_id = std::to_string((message.content[0] << 8 ) | message.content[1]);
        cfg.psid = cfg.ns3_id;
    }
    else
    {
        cfg = *wave_item;
    }

    ss << "Version=0.7" << std::endl;
    ss << "Type=" << cfg.name << std::endl;
    ss << "PSID=" << cfg.psid << std::endl;
    // ss << "VehicleID=" << vehicle_id_ << std::endl;
    ss << "Priority=" << cfg.priority << std::endl;
    ss << "TxMode=ALT" << std::endl;
    ss << "TxChannel=" << cfg.channel << std::endl;
    ss << "TxInterval=0" << std::endl;
    ss << "DeliveryStart=" << std::endl;
    ss << "DeliveryStop=" << std::endl;
    ss << "Signature=False" << std::endl;
    ss << "Encryption=False" << std::endl;
    // ss << "VehiclePosX=" << pose_msg_.pose.position.x << std::endl;
    // ss << "VehiclePosY=" << pose_msg_.pose.position.y << std::endl;


    ss << "Payload=" <<  uint8_vector_to_hex_string(message.content) << std::endl;

    std::string str = ss.str();

    return std::vector<uint8_t>(str.begin(), str.end());
}

/**
* @brief Handles outbound messages from the ROS network
* @param message
*
* This method receives a message from the ROS network, and adds it to the send queue.
*/
void NS3Adapter::on_outbound_message(const cav_msgs::ByteArrayPtr& message) {
    ROS_WARN_STREAM("in on_outbound_message");
    ROS_WARN_STREAM("message received: " << message->message_type);
    ROS_WARN_STREAM("connected: " << ns3_client_.connected());
    if(!ns3_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to NS-3");
        return;
    }

    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(pack_message(*message)));
    send_msg_queue_.push_back(std::move(message_content));
    ROS_WARN_STREAM("queue size: " << send_msg_queue_.size());
}

/**
* @brief Sends a message from the queue of outbound messages
*/
void NS3Adapter::send_message_from_queue() {
    if (!send_msg_queue_.empty()) {
        ROS_DEBUG_STREAM("Sending message: " << std::string(send_msg_queue_.front()->begin(),send_msg_queue_.front()->end()));
        bool success = ns3_client_.send_ns3_message(send_msg_queue_.front());
        send_msg_queue_.pop_front();
        if (!success) {
            ROS_WARN_STREAM("Message send failed");
        }
        else {
            ROS_DEBUG("Message successfully sent from queue");
        }
    }
}

/**
* @brief Message sending service
* @param req
* @param res
*/
bool NS3Adapter::send_message_srv(cav_srvs::SendMessage::Request& req, cav_srvs::SendMessage::Response& res) {
    if(!ns3_client_.connected())
    {
        ROS_WARN_STREAM("Outbound message received but node is not connected to NS-3 Radio");
        res.errorStatus = 1;
        return true;
    }

    // Package data into a message shared pointer; this lets pack_message have
    // the same interface for outgoing messages from the topic and service.
    const cav_msgs::ByteArray::ConstPtr message = cav_msgs::ByteArray::ConstPtr(new cav_msgs::ByteArray(req.message_to_send));
    std::shared_ptr<std::vector<uint8_t>> message_data = std::make_shared<std::vector<uint8_t>>(std::move(pack_message(*message)));
    bool success = ns3_client_.send_ns3_message(message_data);
    if (success) {
        ROS_DEBUG("SendMessage service returned success");
        res.errorStatus = 0;
    }
    else {
        ROS_WARN_STREAM("SendMessage service returned failure");
        res.errorStatus = 1;
    }

    return true;
}

void NS3Adapter::pre_spin()
{
    // Adjust output queue size if config changed.
    if(ns3_client_error_)
    {
        ROS_WARN_STREAM("NS-3 Client Error detected: " << ns3_client_error_.message() << ", resetting the client before attempting to connect again...");
        ns3_client_.close();
        ns3_client_error_.clear();
    }
    //If we are not connected
    //TODO: Set up functionality for disconnected NS-3
    if (!connecting_ && !ns3_client_.connected())
    {
        connecting_ = true;
        if (connect_thread_)
            connect_thread_->join();

        //We don't want to block the spin thread because the driver
        //application maintains driver status topic
        connect_thread_.reset(new std::thread([this]()
        {
            {
                std::lock_guard<std::mutex> lock(cfg_mutex_);
            }
            ROS_DEBUG("Attempting to connect to NS3");
            boost::system::error_code ec;
            ROS_DEBUG("Remote broadcasting port: %u", ns3_broadcasting_port_);
            ROS_DEBUG("Remote registration port: %u", ns3_registration_port_);

            try {
                if (!ns3_client_.connect_registration_and_broadcasting(ns3_address_, ns3_broadcasting_port_, ns3_registration_port_,
                                          ns3_v2x_listening_port_, ns3_time_listening_port_, ec))
                {
                    ROS_WARN_STREAM("Failed to connect, err: " << ec.message());
                }
            } catch (const std::exception &e)
            {
                ROS_ERROR_STREAM("Exception connecting to CARMA Ambassador: " << e.what() << " error_code: " << ec.message());
            }
            connecting_ = false;
        }));
    }

    std::string handshake_msg = compose_handshake_msg(vehicle_id_, role_id_, ns3_v2x_listening_port_, ns3_time_listening_port_, host_ip_);
    broadcast_handshake_msg(handshake_msg);
}


void NS3Adapter::post_spin() {
    send_message_from_queue();
}

void NS3Adapter::load_wave_config(const std::string &fileName)
{
    ROS_DEBUG_STREAM("Loading wave config");

    const char* schema = "{\n"
                        " \"$schema\":\"http://json-schema.org/draft-06/schema\",\n"
                        " \"title\":\"Wave Config Schema\",\n"
                        " \"description\":\"A simple schema to describe DSRC/Wave messages\",\n"
                        "  \"type\": \"array\",\n"
                        "  \"items\": {\n"
                        "    \"type\": \"object\",\n"
                        "    \"properties\": {\n"
                        "      \"name\": {\n"
                        "        \"description\": \"message type - abbreviated name\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"psid\": {\n"
                        "        \"description\": \"psid assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"dsrc_id\": {\n"
                        "        \"description\": \"J2735 DSRC id assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"channel\": {\n"
                        "        \"description\": \"DSRC radio channel assigned to message type in decimal\",\n"
                        "        \"type\": \"string\"\n"
                        "      },\n"
                        "      \"priority\": {\n"
                        "        \"description\": \"WSM Priotiy to use assigned to message type in decimal\",\n"
                        "        \"type\":\"string\"\n"
                        "      }\n"
                        "    },\n"
                        "    \"required\":[\"name\",\"psid\",\"dsrc_id\",\"channel\",\"priority\"]"
                        "  }\n"
                        "}\n";

    std::ifstream file;
    try
    {
        file.open(fileName);
        ROS_INFO_STREAM("fileName : " << fileName);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open file : " << fileName << ", exception: " << e.what());
        return;
    }
    rapidjson::Document sd;
    if(sd.Parse(schema).HasParseError())
    {
        ROS_ERROR_STREAM("Invalid Wave Config Schema");
        return;
    }
    rapidjson::SchemaDocument schemaDocument(sd);
    rapidjson::Document doc;
    rapidjson::IStreamWrapper isw(file);
    if(doc.ParseStream(isw).HasParseError())
    {
        ROS_ERROR_STREAM("Error Parsing Wave Config");
        return;
    }
    rapidjson::SchemaValidator validator(schemaDocument);
    if(!doc.Accept(validator))
    {
        ROS_ERROR_STREAM("Wave Config improperly formatted");
        return;
    }
    for(auto& it : doc.GetArray())
    {
        auto entry = it.GetObject();
        wave_cfg_items_.emplace_back(entry["name"].GetString(),
                                     entry["psid"].GetString(),
                                     entry["dsrc_id"].GetString(),
                                     entry["channel"].GetString(),
                                     entry["priority"].GetString());

    }
}

void NS3Adapter::shutdown()
{
    ROS_INFO("Shutdown signal received from DriverApplication");
    if (connect_thread_)
    {
        ROS_INFO("Cleaning up connection thread");
        connect_thread_->join();
        connect_thread_.reset();
    }

    ROS_INFO("Closing connection to radio");
    ns3_client_.close();

}

void NS3Adapter::pose_cb(geometry_msgs::PoseStamped pose_msg)
{
    pose_msg_ = pose_msg;

    /*TODO: Add Pose Functionality*/
}

std::string NS3Adapter::compose_handshake_msg(const std::string& veh_id, const std::string& role_id, int message_port, int time_port, const std::string& ip)
{
    // document is the root of a json message
	rapidjson::Document document;

	// define the document as an object rather than an array
	document.SetObject();

	// create a rapidjson array type with similar syntax to std::vector
	rapidjson::Value array(rapidjson::kArrayType);

	// must pass an allocator when the object may need to allocate memory
	rapidjson::Document::AllocatorType& allocator = document.GetAllocator();

    rapidjson::Value idtextPart;
	idtextPart.SetString(veh_id.c_str(), allocator);
    document.AddMember("carmaVehicleId", idtextPart, allocator);

    rapidjson::Value roletextPart;
	roletextPart.SetString(role_id.c_str(), allocator);
    document.AddMember("carlaVehicleRole", roletextPart, allocator);

    rapidjson::Value iptextPart;
	iptextPart.SetString(ip.c_str(), allocator);
    document.AddMember("rxMessageIpAddress", iptextPart, allocator);

    rapidjson::Value porttextPart;
	porttextPart.SetInt(message_port);
    document.AddMember("rxMessagePort", porttextPart, allocator);

    rapidjson::Value portTimePart;
	portTimePart.SetInt(time_port);
    document.AddMember("rxTimeSyncPort", portTimePart, allocator);

    rapidjson::StringBuffer strbuf;
	rapidjson::Writer<rapidjson::StringBuffer> writer(strbuf);
	document.Accept(writer);

    std::string strbufstring = strbuf.GetString();

    return strbufstring;
}

void NS3Adapter::broadcast_handshake_msg(const std::string& msg_string)
{
    ROS_DEBUG_STREAM("Attempting to broadcast_handshake_msg: " << msg_string);
    auto msg_vector = std::vector<uint8_t>(msg_string.begin(), msg_string.end());
    std::shared_ptr<std::vector<uint8_t>> message_content = std::make_shared<std::vector<uint8_t>>(std::move(msg_vector));

    bool success = ns3_client_.send_registration_message(message_content);
    ROS_DEBUG_STREAM("ns3_address_: " << ns3_address_);
    ROS_DEBUG_STREAM("ns3_registration_port_: " << ns3_registration_port_);
    ROS_DEBUG_STREAM("ns3_v2x_listening_port_: " << ns3_v2x_listening_port_);
    ROS_DEBUG_STREAM("ns3_time_listening_port_: " << ns3_time_listening_port_);

    if (!success) {
        ROS_WARN_STREAM("Handshake Message send failed");
    }
    else {
        ROS_DEBUG_STREAM("Handshake Message successfully");
    }
}

cav_msgs::DriverStatus NS3Adapter::get_driver_status()
{
    return getStatus();
}

std::deque<std::shared_ptr<std::vector<uint8_t>>> NS3Adapter::get_msg_queue()
{
    return send_msg_queue_;
}
