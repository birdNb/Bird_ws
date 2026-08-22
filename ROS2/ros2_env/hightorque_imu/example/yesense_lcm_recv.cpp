#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "imu_msg.hpp"


class Handler 
{
    public:
        ~Handler() {}
// Callback function to handle incoming imu_msg_t messages
    void imu_msg_handler(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const imu_msg::imu_msg* msg) 
    {
        std::cout << "Received imu_msg_t message on channel: " << chan << std::endl;
        std::cout << "Timestamp: " << msg->timestamp << " ms" << std::endl;
        std::cout << "RPY: [" << msg->rpy[0] << ", " << msg->rpy[1] << ", " << msg->rpy[2] << "]" << std::endl;
        std::cout << "Quaternion: [" << msg->quat[0] << ", " << msg->quat[1] << ", " << msg->quat[2] << ", " << msg->quat[3] << "]" << std::endl;
        std::cout << "Accel: [" << msg->accel[0] << ", " << msg->accel[1] << ", " << msg->accel[2] << "]" << std::endl;
        std::cout << "Gyro: [" << msg->gyro[0] << ", " << msg->gyro[1] << ", " << msg->gyro[2] << "]" << std::endl;
        std::cout << "Magnet: [" << msg->magnet[0] << ", " << msg->magnet[1] << ", " << msg->magnet[2] << "]" << std::endl;
        std::cout << "Description: " << msg->description << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    }
};

int main(int argc, char** argv) {
    // Initialize LCM
    lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm.good()) {
        std::cerr << "Failed to initialize LCM" << std::endl;
        return 1;
    }
    Handler handler;
    // Subscribe to the imu_msg_t channel
    lcm.subscribe("imu_msg", &Handler::imu_msg_handler, &handler);

    std::cout << "Listening for imu_msg_t messages on channel 'imu_msg'..." << std::endl;

    // Run the LCM event loop
    while (true) {
        lcm.handle();
    }

    return 0;
}