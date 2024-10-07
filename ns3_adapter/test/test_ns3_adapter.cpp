#include <ns3_adapter.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(NS3AdapterTest, DISABLED_testOnConnectHandler)
{
    int argc = 1;
    //char* argv[] {c[0], c[1]};
    char **argv;
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
    worker.onConnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OPERATIONAL);
}

TEST(NS3AdapterTest, DISABLED_testOnDisconnectHandler)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
    worker.onDisconnectHandler();
    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);

}

TEST(NS3AdapterTest, DISABLED_testOnMsgReceivedHandler)
{
   int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);
    uint16_t id = 123;
    std::vector<uint8_t> content;
    content.push_back(1);

    ROS_ERROR_STREAM("Pre-Connection NS-3 Status: " << static_cast<int>(worker.getDriverStatus().status));
    EXPECT_THROW(worker.onMessageReceivedHandler(content, id), ros::TimeNotInitializedException); //Since the onMessageReceivedHandler requires ros::Time initialized, this should throw an exception

    EXPECT_EQ(worker.getDriverStatus().status, cav_msgs::DriverStatus::OFF);


}

TEST(NS3AdapterTest, DISABLED_testpackMessage)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray array1;

    uint8_t msg = 4;

    array1.content.push_back(msg);


    auto pm = worker.packMessage(array1);

    ASSERT_GT(pm.size(), 0);
    ASSERT_EQ(pm.size(), 150); //not 189 because VehicleID, VehiclePosX and VehiclePosY is turned off
}

TEST(NS3AdapterTest, testonOutboundMessage)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};

    NS3Adapter worker(argc,argv);
    cav_msgs::ByteArray byte_array;
    uint8_t msg = 1;
    byte_array.message_type = "Unknown";
    byte_array.content.push_back(msg);
    cav_msgs::ByteArrayPtr message;
    message = boost::make_shared<cav_msgs::ByteArray>(byte_array);

    worker.onOutboundMessage(message);
    auto msg_q = worker.getMsgQueue();
    EXPECT_EQ(msg_q.size(), 0);

}

TEST(NS3AdapterTest, testpackSRMMessage)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};

    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray byte_array;
    std::vector<uint8_t> content = {0,29,43,112,191,41,206,32,1,3,132,0,1,156,44,0,128,209,126,83,186,40,0,4,96,35,181,201,99,0,65,169,27,118,237,69,36,242,169,101,157,70,253,56,221,192};
    byte_array.message_type = "SRM";
    byte_array.content = content;
    cav_msgs::ByteArrayPtr message;
    message = boost::make_shared<cav_msgs::ByteArray>(byte_array);

    auto vector_msg = worker.packMessage(byte_array);
    //Compare the payload with the input byte array converted to hex string
    auto content_to_hex = worker.uint8_vector_to_hex_string(content);

    std::vector<uint8_t> payload_vec;
    payload_vec.assign(vector_msg.end() - content_to_hex.size() - 1, vector_msg.end()-1);

    std::string payload_str;
    for (int i = 0;i < payload_vec.size(); i++){
        payload_str.push_back(char(payload_vec[i]));
    }

    EXPECT_EQ(payload_str, content_to_hex);
}

TEST(NS3AdapterTest, testpackSSMMessage)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};

    NS3Adapter worker(argc,argv);

    cav_msgs::ByteArray byte_array;
    std::vector<uint8_t> content = {0,30,62,101,68,151,210,240,8,0,36,0,0,15,172,75,144,0,0,9,100,20,18,0,32,0,0,64,5,1,244,17,114,0,0,1,46,130,134,64,6,0,0,8,0,176,62,130,46,64,0,0,38,32,80,196,128,192,0,1,0,22,7,208,0};
    byte_array.message_type = "SSM";
    byte_array.content = content;
    cav_msgs::ByteArrayPtr message;
    message = boost::make_shared<cav_msgs::ByteArray>(byte_array);

    auto vector_msg = worker.packMessage(byte_array);
    //Compare the payload with the input byte array converted to hex string
    auto content_to_hex = worker.uint8_vector_to_hex_string(content);

    std::vector<uint8_t> payload_vec;
    payload_vec.assign(vector_msg.end() - content_to_hex.size() - 1, vector_msg.end()-1);

    std::string payload_str;
    for (int i = 0;i < payload_vec.size(); i++){
        payload_str.push_back(char(payload_vec[i]));
    }

    EXPECT_EQ(payload_str, content_to_hex);
}


TEST(NS3AdapterTest, DISABLED_testSendMessageSrv)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);

    cav_srvs::SendMessage::Request req;
    cav_srvs::SendMessage::Response resp;

    uint8_t msg = 8;
    req.message_to_send.content.push_back(msg);

   bool result = worker.sendMessageSrv(req, resp);

   EXPECT_EQ(result, true);
   EXPECT_EQ(resp.errorStatus, 1);

}

TEST(NS3AdapterTest, DISABLED_testcompose_handshake_msg)
{
    int argc = 1;
    char c[2][2] = {{'a','b'}, {'c','d'}};
    char* argv[] {c[0], c[1]};
    NS3Adapter worker(argc,argv);
    std::string result = worker.compose_handshake_msg("default_id", "ego1", 2000, 2001, "127.0.0.1");
    EXPECT_EQ(result, "{\"carmaVehicleId\":\"default_id\",\"carlaVehicleRole\":\"ego1\",\"rxMessageIpAddress\":\"127.0.0.1\",\"rxMessagePort\":2000,\"rxTimeSyncPort\":2001}");
}
