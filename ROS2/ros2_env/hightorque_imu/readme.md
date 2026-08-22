# 高擎机器人IMU接口使用说明

## 一、简介

IMU提供了两种接口类型：
1. 函数接口；
2. LCM话题接口。

## 二、 编译、安装与卸载

### 1. 编译
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

   如果 yesense_imu 安装在自定义路径，需要指定：
   ```bash
   cmake -Dyesense_imu_DIR=/your/custom/path/lib/cmake/yesense_imu ..
   make
   ```

### 2. 安装

   ```
   sudo make install 
   ```

### 3. 卸载
   手动删除已安装的文件：
   ```bash
   sudo rm -rf /usr/local/lib/libyesense_imu.so
   sudo rm -rf /usr/local/include/yesense_imu
   sudo rm -rf /usr/local/lib/cmake/yesense_imu
   sudo rm -rf /usr/local/share/yesense_imu
   sudo rm -f /usr/local/bin/yesense_node
   sudo rm -f /usr/local/bin/yesense_recv
   ```


## 三、使用方法

### 函数接口的使用方法
   1. 创建imu对象:
   ```
   yesense::YesenseDriver yesense_driver;
   ```
   2. 调用函数接口：
   ```
   double rpy[3];
   double acc[3];
   double gyro[3];
   yesense_driver.get_rpy(rpy);     // 获取姿态角
   yesense_driver.get_accel(acc);   // 获取加速度
   yesense_driver.get_gyro(gyro);   // 获取角速度
   ```     

### LCM话题接口的使用方法
   1. 创建imu对象:
   ```
   yesense::YesenseDriver yesense_driver;
   ```
   2. 使能LCM话题接口：
   ```
   yesense_driver.enable_lcm();
   ```  
   3. 订阅LCM话题;
   * 略，详见[例程](example/yesense_lcm_recv.cpp)说明。

## 四、例程

1. 使用函数接口和LCM话题接口的[示例程序](example/yesense_node.cpp)说明：
   
2. 接收LCM话题的[示例程序](example/yesense_lcm_recv.cpp)说明;

## 五、注意事项

1. 如果硬件上有改动，需要及时修改[硬件配置文件](cfg/config.yaml)；
2. 虚拟串口需要额外指定USB的PID和VID，硬件串口则不需要；
