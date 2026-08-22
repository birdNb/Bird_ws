#include "yesense_driver.h"
#include <iostream>
#include <thread>
#include <chrono>
int main(int argc, char** argv)
{
    double rpy[3];
    double acc[3];
    double gyro[3];
    yesense::YesenseDriver yesense_driver;
    yesense_driver.enable_lcm();
    while(1)
    {
        yesense_driver.get_rpy(rpy);
        yesense_driver.get_accel(acc);
        yesense_driver.get_gyro(gyro);
        printf("rpy: %f %f %f\n", rpy[0], rpy[1], rpy[2]);
        printf("acc: %f %f %f\n", acc[0], acc[1], acc[2]);
        printf("gyro: %f %f %f\n", gyro[0], gyro[1], gyro[2]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    return 0;
}
