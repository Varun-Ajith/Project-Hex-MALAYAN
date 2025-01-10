#ifndef IMU_INTERFACE_HPP
#define IMU_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

class IMUInterface : public rclcpp::Node {
public:
    IMUInterface();
    ~IMUInterface();

private:
    static constexpr uint8_t MPU6886_ADDRESS = 0x68;
    static constexpr uint8_t MPU6886_WHOAMI = 0x75;
    static constexpr uint8_t MPU6886_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t MPU6886_GYRO_XOUT_H = 0x43;
    static constexpr uint8_t MPU6886_PWR_MGMT_1 = 0x6B;
    static constexpr uint8_t MPU6886_CONFIG = 0x1A;
    static constexpr uint8_t MPU6886_GYRO_CONFIG = 0x1B;
    static constexpr uint8_t MPU6886_ACCEL_CONFIG = 0x1C;

    static constexpr float ACCEL_SCALE = 16384.0;
    static constexpr float GYRO_SCALE = 131.0;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int i2c_fd_;
    bool initializeIMU();
    void readIMUData();
    int16_t read16(int reg);
    void write8(int reg, int value);
    
    float convertRawAccel(int16_t raw_value);
    float convertRawGyro(int16_t raw_value);
};

#endif