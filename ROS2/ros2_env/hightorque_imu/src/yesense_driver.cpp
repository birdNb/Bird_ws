#include "yesense_driver.h"
#include <map>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <libserialport.h>
#include <dirent.h>
#include <algorithm>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include "imu_msg.hpp"
#include <iostream>

namespace yesense{

YesenseDriver::YesenseDriver()
    : port_("/dev/ttyACM")
    , baudrate_(460800)
    , buffer_size_(4096)
    , wait_response_flag_(false)
    , check_respose_flag_(false)
    , error_respose_cnt_(0)
    , mode_(0)
    , configured_(false)
    , lcm_enable(false)
{

    // 数据缓冲区
    data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(buffer_size_));

    // 读取串口数据所需的变量
    index_    = 0;
    mode_     = 0;
    bytes_    = 0;
    checksum_ = 0;
    if(this->lcm_enable)
    {
        this->lcm_ptr_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=0");
        if(!this->lcm_ptr_->good())
        {
            std::cerr << "\033[31mFailed to create LCM instance.\033[0m" << std::endl;
        }
    }

    this->load_config();
    this->initSerial();
    this->serial_thread_ = boost::thread(boost::bind(&YesenseDriver::run,this));
    this->deseralize_thread_ = boost::thread(boost::bind(&YesenseDriver::_parse,this));
}

YesenseDriver::YesenseDriver(const std::string& config_file)
    : port_("/dev/ttyACM")
    , baudrate_(460800)
    , buffer_size_(4096)
    , wait_response_flag_(false)
    , check_respose_flag_(false)
    , error_respose_cnt_(0)
    , mode_(0)
    , configured_(false)
    , lcm_enable(false)
{

    // 数据缓冲区
    data_buffer_ptr_ = boost::shared_ptr<boost::circular_buffer<char> >(new boost::circular_buffer<char>(buffer_size_));

    // 读取串口数据所需的变量
    index_    = 0;
    mode_     = 0;
    bytes_    = 0;
    checksum_ = 0;
    if(this->lcm_enable)
    {
        this->lcm_ptr_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=0");
        if(!this->lcm_ptr_->good())
        {
            std::cerr << "\033[31mFailed to create LCM instance.\033[0m" << std::endl;
        }
    }

    this->load_config(config_file);  // 使用指定的配置文件
    this->initSerial();
    this->serial_thread_ = boost::thread(boost::bind(&YesenseDriver::run,this));
    this->deseralize_thread_ = boost::thread(boost::bind(&YesenseDriver::_parse,this));
}

YesenseDriver::~YesenseDriver()
{
    std::cout << "Close yesense device." << std::endl;
    if(serial_.isOpen())
    {
        serial_.close();
    }
    data_buffer_ptr_.reset();

    configured_ = false;
    deseralize_thread_.join();
}

void YesenseDriver::load_config()
{
    // 尝试多个配置文件路径
    std::vector<std::string> config_paths = {
#ifdef CONFIG_INSTALL_PATH
        CONFIG_INSTALL_PATH,  // 安装路径（由 CMake 定义）
#endif
        "../cfg/config.yaml", // 开发路径（从 build 目录运行）
        "cfg/config.yaml",    // 当前目录
        "./config.yaml"       // 备用路径
    };

    YAML::Node config;
    bool config_loaded = false;
    std::string loaded_path;

    // 尝试从各个路径加载配置文件
    for (const auto& path : config_paths) {
        try {
            config = YAML::LoadFile(path);
            config_loaded = true;
            loaded_path = path;
            std::cout << "成功加载配置文件: " << path << std::endl;
            break;
        } catch (const YAML::Exception& e) {
            // 继续尝试下一个路径
            continue;
        }
    }

    if (!config_loaded) {
        std::cerr << "\033[31m错误: 无法找到配置文件。尝试过以下路径:\033[0m" << std::endl;
        for (const auto& path : config_paths) {
            std::cerr << "  - " << path << std::endl;
        }
        return;
    }

    try {
        // Access the IMU configuration
        if (config["imu"]) {

            this->port_ = config["imu"]["device"].as<std::string>();
            this->baudrate_ = config["imu"]["baud_rate"].as<int>();
            this->dev_pid = config["imu"]["pid"].as<int>();
            this->dev_vid = config["imu"]["vid"].as<int>();

            std::cout << "IMU Configuration:" << std::endl;
            // Access individual fields
            std::cout << "    Product ID (pid): " << this->dev_pid << std::endl;
            std::cout << "    Vendor ID (vid): " << this->dev_vid << std::endl;
            std::cout << "    Device: " << this->port_ << std::endl;
            std::cout << "    Baud Rate: " << this->baudrate_  << "\n---------------------- " << std::endl;
        } else {
            std::cerr << "\033[31mIMU configuration not found in the YAML file.\033[0m" << std::endl;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "\033[31mError parsing YAML file: " << e.what() << "\033[0m" << std::endl;
    }
}

void YesenseDriver::load_config(const std::string& config_file)
{
    YAML::Node config;
    bool config_loaded = false;

    try {
        config = YAML::LoadFile(config_file);
        config_loaded = true;
        std::cout << "成功加载配置文件: " << config_file << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "\033[31m错误: 无法加载配置文件 " << config_file << "\033[0m" << std::endl;
        std::cerr << "\033[31m异常信息: " << e.what() << "\033[0m" << std::endl;
        return;
    }

    if (!config_loaded) {
        return;
    }

    try {
        // Access the IMU configuration
        if (config["imu"]) {

            this->port_ = config["imu"]["device"].as<std::string>();
            this->baudrate_ = config["imu"]["baud_rate"].as<int>();
            this->dev_pid = config["imu"]["pid"].as<int>();
            this->dev_vid = config["imu"]["vid"].as<int>();

            std::cout << "IMU Configuration:" << std::endl;
            // Access individual fields
            std::cout << "    Product ID (pid): " << this->dev_pid << std::endl;
            std::cout << "    Vendor ID (vid): " << this->dev_vid << std::endl;
            std::cout << "    Device: " << this->port_ << std::endl;
            std::cout << "    Baud Rate: " << this->baudrate_  << "\n---------------------- " << std::endl;
        } else {
            std::cerr << "\033[31mIMU configuration not found in the YAML file.\033[0m" << std::endl;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "\033[31mError parsing YAML file: " << e.what() << "\033[0m" << std::endl;
    }
}

void YesenseDriver::run()
{   
    try 
    {
        while(true)
        {
            //read data from serial
            if (serial_.available())
            {
                data_ = serial_.read(serial_.available());                
                {
                    boost::mutex::scoped_lock lock(m_mutex_);  
                
                    for(int i=0;i<data_.length();i++)
                    {
                        data_buffer_ptr_->push_back(data_[i]);
                    }
                }
            
            }

            std::this_thread::sleep_for(std::chrono::microseconds(10));

        }

        std::cerr << "\033[31mROS Exited !\033[0m" << std::endl;
    } 
    catch (std::exception &err) 
    {
        std::cerr << "\033[31merror in 'run' function, msg: " << err.what() << "\033[0m" << std::endl;
    }    
}

void YesenseDriver::enable_lcm()
{
    if(this->lcm_enable == false)
    {
        this->lcm_enable = true;
        this->lcm_ptr_ = std::make_shared<lcm::LCM>("udpm://239.255.76.67:7667?ttl=0");
        if(!this->lcm_ptr_->good())
        {
            std::cerr << "\033[31mFailed to create LCM instance.\033[0m" << std::endl;
        }
    }
}

void YesenseDriver::disable_lcm()
{
    if(this->lcm_enable)
    {
        this->lcm_enable = false;
        
    }
}

int YesenseDriver::serial_pid_vid(const char *name)
{
    int pid, vid;
    int r = 0;
    struct sp_port *port;
    
    sp_get_port_by_name(name, &port);
    sp_open(port, SP_MODE_READ);
    if (sp_get_port_usb_vid_pid(port, &vid, &pid) != SP_OK) 
    {
        r = -1;
    } 
    else 
    {
        if (pid == 0x5543 && vid == 0x5953)
        {
            r = 1;
        }
    }
    // std::cout << "Port: " << name << ", PID: 0x" << std::hex << pid << ", VID: 0x" << vid << std::dec << std::endl;

    // 关闭端口
    sp_close(port);
    sp_free_port(port);

    return r;
}

std::vector<std::string> list_serial_ports(const std::string& full_prefix) 
{
    std::string base_path = full_prefix.substr(0, full_prefix.rfind('/') + 1);
    std::string prefix = full_prefix.substr(full_prefix.rfind('/') + 1);
    std::vector<std::string> serial_ports;
    DIR *directory;
    struct dirent *entry;

    directory = opendir(base_path.c_str());
    if (!directory)
    {
        std::cerr << "Could not open the directory " << base_path << std::endl;
        return serial_ports; // Return an empty vector if cannot open directory
    }

    while ((entry = readdir(directory)) != NULL)
    {
        std::string entryName = entry->d_name;
        if (entryName.find(prefix) == 0)
        { // Check if the entry name starts with the given prefix
            serial_ports.push_back(base_path + entryName);
        }
    }

    closedir(directory);

    // Sort the vector in ascending order
    std::sort(serial_ports.begin(), serial_ports.end());

    return serial_ports;
}

void YesenseDriver::initSerial()
{
    while (serial_.isOpen() == false)
    {
        try
        {
            // Only ttyACM need to find PID-VID
            if(this->port_ == "/dev/ttyACM")
            {
                bool flag = false;

                std::vector<std::string> ports = list_serial_ports(port_);
                for (const std::string& port : ports) 
                {
                    if (serial_pid_vid(port.c_str()) > 0)
                    {
                        std::cout << "IMU serial port:" << port.c_str() << ", rate:" << baudrate_ << std::endl;
                        port_ = port;
                        flag = true;
                        break;
                    }
                }
                if (flag == false)
                {
                    std::cerr << "Cannot find the IMU serial port number, please check if the USB connection is normal" << std::endl;
                    exit(-1);
                }
            }

            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(to);
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            std::cout << "Unable to open serial port: " << serial_.getPort().c_str() << " ,Trying again in 5 seconds." << std::endl;
            
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
    
    if (serial_.isOpen())
    {
        std::cout << "Serial port: " << serial_.getPort().c_str() << " initialized and opened." << std::endl;

        configured_ = true;
    }
}

void YesenseDriver::_parse() 
{
    try 
    {
        this->parse();
    } 
    catch (std::exception &err)
    {
        std::cerr << "\033[31merror in 'spin', msg: " << err.what() << "\033[0m" << std::endl;
    }    
}

void YesenseDriver::parse()
{
    uint8_t data = 0x00;
    uint8_t prev_data = 0x00;

    uint16_t tid = 0x00;
    uint16_t prev_tid = 0x00;

    uint32_t gps_header_sum;

    while(configured_)
    {
        while(!data_buffer_ptr_->empty())
        {            
            {
                boost::mutex::scoped_lock lock(m_mutex_);  
                data = uint8_t(data_buffer_ptr_->front());
                data_buffer_ptr_->pop_front();
            }
            
            if (mode_ == MODE_MESSAGE)            /* message data being recieved */
            {
                ck1_ += data;
                ck2_ += ck1_;

                prev_data = data; // save prev data
                
                message_in_[index_++] = data;
                bytes_--;
                if (bytes_ == 0)                  /* is message complete? if so, checksum */
                    mode_ = MODE_CHECKSUM_L;
            }
            else if (mode_ == MODE_HEADER1)
            {
                if (data == 0x59)
                {
                    mode_++;
                    // last_msg_timeout_time = c_time + SERIAL_MSG_TIMEOUT;
                }
                else if(data == '$') /* is GPS msg ? */ 
                {
                    mode_ = MODE_GPS_RAW;
                    gps_buf_index = 0;
                    gps_header_sum = 0;
                    gps_buf[gps_buf_index++] = data;
                }
            }
            else if(mode_ == MODE_GPS_RAW) 
            {                
                gps_buf[gps_buf_index++] = data;

                if (isgraph(data) || data == '\r' || data == '\n')
                {
                    if (gps_buf_index <= 6)
                    {
                        if (isalpha(data))
                            gps_header_sum += data;
                        else
                            mode_ = MODE_HEADER1;
                    }

                    if (data == '\r') /* frame end, exit */
                    {
                        mode_ = MODE_HEADER1;
                        gps_buf[gps_buf_index - 1] = '\0';
                        gsp_raw[gps_header_sum] = std::string((char *)gps_buf);
                    }
                }
                else
                {
                    mode_ = MODE_HEADER1;
                }
            }
            else if(mode_ == MODE_HEADER2)
            {
                if(data == 0x53)
                {
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;
                    
                    mode_++;
                }
                else
                {
                    mode_ = 0;
                }
            }
            else if(mode_ == MODE_TID_L)
            {
                ck1_ += data;
                ck2_ += ck1_;

                // set tid low
                tid = data;

                //判断是否为参数的返回值
                {
                    boost::mutex::scoped_lock lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        if(param_class_ == data)
                        {
                            std::cout << "Almost param response" << std::endl;
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            std::cout << "Not param response" << std::endl;

                            check_respose_flag_ = false;
                            error_respose_cnt_++;

                            if (error_respose_cnt_ > 10) 
                            {
                                wait_response_flag_ = false;
                                error_respose_cnt_  = 0;
                            }
                        }
                    }
                }

                mode_++;
            }
            else if(mode_ == MODE_TID_H)
            {
                ck1_ += data;
                ck2_ += ck1_;

                tid |= ((uint16_t)data) << 8;

                if(prev_tid != 0 && tid > prev_tid && prev_tid != tid - 1) 
                {
                    std::cerr << "\033[31mFrame losed: prev_TID: " << prev_tid << ", cur_TID: " << tid << "\033[0m" << std::endl;
                }

                prev_tid = tid;

                //判断是否为参数的返回值
                if(check_respose_flag_)
                {
                    boost::mutex::scoped_lock lock(m_response_mutex_);
                    if(wait_response_flag_)
                    {
                        //maybe this byte is class id
                        uint8_t id = data & 0x07;
                        length_low_ = data;
                        if(param_id_ == id)
                        {
                            std::cout << "Double check param response" << std::endl;
                            check_respose_flag_ = true;
                        }
                        else
                        {
                            std::cout << "Double not param response" << std::endl;
                            check_respose_flag_ = false;
                        }
                    }
                }
                mode_++;
            }
            else if(mode_ == MODE_LENGTH)
            {
                ck1_ += data;
                ck2_ += ck1_;

                if(check_respose_flag_)
                {
                    //长度为13-bit
                    bytes_ = (length_low_ | data << 8) >> 3;
                    std::cout << "package length: " << bytes_ << std::endl;
                }
                else
                {
                    bytes_ = data;
                }
                
                if(bytes_ == 0) // package length is 0, reset all and exit loop
                {
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;
                    mode_ = 0;
                    bytes_ = 0;
                    break;
                }

                mode_++;
            }
            else if(mode_ == MODE_CHECKSUM_L)
            {
                if(ck1_ == data)
                {
                    mode_++;
                }
                else
                {
                    //crc check error
                    ck1_ = 0;
                    ck2_ = 0;
                    index_ = 0;
                    mode_ = 0;
                    bytes_ = 0;
                    break;
                }
            }
            else if(mode_ == MODE_CHECKSUM_H)
            {
                //检查是否为参数设置的返回值
                if(wait_response_flag_ && check_respose_flag_)
                {
                    // log Response
                    {
                        std::cout<<"Response: "<<std::endl;

                        for (int i = 0; i < index_; i++)
                        {
                            printf("%02X ", message_in_[i]);
                        }
                        
                        std::cout<<std::endl;
                    }

                    {
                        boost::mutex::scoped_lock lock(m_response_mutex_);
                        wait_response_flag_ = false;
                        check_respose_flag_ = false;
                        error_respose_cnt_  = 0;
                    }
                    
                    continue;
                }


                if(ck2_ == data)
                {
                    // std::cerr << "We Received A Vaild Data Pack !"<< std::endl;

                    // 解析数据
                    unsigned short pos = 0;
                    int payload_len = index_;
                    payload_data_t *payload = NULL;
                    unsigned char ret = 0xff;

                    while(payload_len > 0)
                    {
                        payload = (payload_data_t *)(message_in_ + pos);
                                                
                        ret = parse_data_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + 2);

                        if((unsigned char)0x01 == ret) // check done !
                        {
                            pos += payload->data_len + sizeof(payload_data_t);
                            payload_len -= payload->data_len + sizeof(payload_data_t);
                        }
                        else // check failed
                        {
                            pos++;
                            payload_len--;
                        }
                    }

                    if(this->lcm_enable)
                    {
                        this->publish_imu(g_output_info);
                    }
                    
                }
                else
                {
                    //crc check error
                    std::cerr << "\033[31mError checksum H !, TID: "<< tid << "\033[0m" << std::endl;
                }

                ck1_ = 0;
                ck2_ = 0;
                index_ = 0;
                mode_ = 0;
                bytes_ = 0;
            }   
        }
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

void YesenseDriver::get_rpy(double *rpt)
{
    rpt[0] = g_output_info.roll;
    rpt[1] = g_output_info.pitch;
    rpt[2] = g_output_info.yaw;
}

void YesenseDriver::get_accel(double *accel)
{
    accel[0] = g_output_info.accel_x;
    accel[1] = g_output_info.accel_y;
    accel[2] = g_output_info.accel_z;
}

void YesenseDriver::get_gyro(double *gyro)
{
    gyro[0] = g_output_info.angle_x;
    gyro[1] = g_output_info.angle_y;
    gyro[2] = g_output_info.angle_z;
}

void YesenseDriver::get_quat(double *quat)
{
    quat[0] = g_output_info.quaternion_data0;
    quat[1] = g_output_info.quaternion_data1;
    quat[2] = g_output_info.quaternion_data2;
    quat[3] = g_output_info.quaternion_data3;
}

#define EARTH_RADIUS 6378.137 //地球半径
#define Angle_To_Rad(x) (((x) * 3.141592653589793) / 180.0)


void YesenseDriver::publish_imu(const protocol_info_t &imu_data)
{
    imu_msg::imu_msg imu_msg;
    auto now = std::chrono::system_clock::now();
    imu_msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    imu_msg.rpy[0] = imu_data.roll;
    imu_msg.rpy[1] = imu_data.pitch;
    imu_msg.rpy[2] = imu_data.yaw;
    imu_msg.gyro[0] = imu_data.angle_x;
    imu_msg.gyro[1] = imu_data.angle_y;
    imu_msg.gyro[2] = imu_data.angle_z;
    imu_msg.accel[0] = imu_data.accel_x;
    imu_msg.accel[1] = imu_data.accel_y;
    imu_msg.accel[2] = imu_data.accel_z;
    // // TODO
    // imu_msg.quat[0] = imu_data.quat_x;
    // imu_msg.quat[1] = imu_data.quat_y;
    // imu_msg.quat[2] = imu_data.quat_z;
    // imu_msg.quat[3] = imu_data.quat_w;
    imu_msg.magnet[0] = imu_data.mag_x;
    imu_msg.magnet[1] = imu_data.mag_y;
    imu_msg.magnet[2] = imu_data.mag_z;
    imu_msg.description = "yesense_imu";
    
    lcm_ptr_->publish("imu_msg", &imu_msg);
}
}
