#include "hexapod_hardware/imu_interface.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

IMUInterface::IMUInterface() : Node("imu_interface") {
    this->declare_parameter("update_rate", 100.0);
    this->declare_parameter("frame_id", "imu_link");
    this->declare_parameter("i2c_device", "/dev/i2c-1");
    
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    if (!initializeIMU()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize IMU!");
        return;
    }

    double update_rate = this->get_parameter("update_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
        std::bind(&IMUInterface::readIMUData, this));
    
    RCLCPP_INFO(this->get_logger(), "IMU Interface initialized successfully");
}

IMUInterface::~IMUInterface() {
    if (i2c_fd_ >= 0) {
        write8(MPU6886_PWR_MGMT_1, 0x40);
        close(i2c_fd_);
    }
}

bool IMUInterface::initializeIMU() {
    std::string i2c_device = this->get_parameter("i2c_device").as_string();
    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    
    if (i2c_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open I2C device");
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, MPU6886_ADDRESS) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access/talk to slave");
        return false;
    }

    uint8_t buffer[1];
    buffer[0] = MPU6886_WHOAMI;
    if (write(i2c_fd_, buffer, 1) != 1 || read(i2c_fd_, buffer, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read WHO_AM_I register");
        return false;
    }

    if (buffer[0] != 0x19) {
        RCLCPP_ERROR(this->get_logger(), "Wrong device ID: %02X", buffer[0]);
        return false;
    }

    write8(MPU6886_PWR_MGMT_1, 0x00);
    rclcpp::sleep_for(100ms);
    write8(MPU6886_PWR_MGMT_1, 0x01);
    write8(MPU6886_CONFIG, 0x01);
    write8(MPU6886_GYRO_CONFIG, 0x00);
    write8(MPU6886_ACCEL_CONFIG, 0x00);

    return true;

void IMUInterface::readIMUData() {
    auto message = sensor_msgs::msg::Imu();
    message.header.stamp = this->now();
    message.header.frame_id = this->get_parameter("frame_id").as_string();

    // Read accelerometer data
    int16_t ax = read16(MPU6886_ACCEL_XOUT_H);
    int16_t ay = read16(MPU6886_ACCEL_XOUT_H + 2);
    int16_t az = read16(MPU6886_ACCEL_XOUT_H + 4);

    // Read gyroscope data
    int16_t gx = read16(MPU6886_GYRO_XOUT_H);
    int16_t gy = read16(MPU6886_GYRO_XOUT_H + 2);
    int16_t gz = read16(MPU6886_GYRO_XOUT_H + 4);

    // Convert and assign accelerometer data (m/s^2)
    message.linear_acceleration.x = convertRawAccel(ax) * 9.81;
    message.linear_acceleration.y = convertRawAccel(ay) * 9.81;
    message.linear_acceleration.z = convertRawAccel(az) * 9.81;

    // Convert and assign gyroscope data (rad/s)
    message.angular_velocity.x = convertRawGyro(gx) * M_PI / 180.0;
    message.angular_velocity.y = convertRawGyro(gy) * M_PI / 180.0;
    message.angular_velocity.z = convertRawGyro(gz) * M_PI / 180.0;

    // Set covariance matrices (you might want to adjust these values)
    for (int i = 0; i < 9; i++) {
        message.linear_acceleration_covariance[i] = (i == 0 || i == 4 || i == 8) ? 0.01 : 0.0;
        message.angular_velocity_covariance[i] = (i == 0 || i == 4 || i == 8) ? 0.01 : 0.0;
    }

    imu_pub_->publish(message);
}

int16_t IMUInterface::read16(int reg) {
    int16_t msb = wiringPiI2CReadReg8(i2c_fd_, reg);
    int16_t lsb = wiringPiI2CReadReg8(i2c_fd_, reg + 1);
    return (msb << 8) | lsb;
}

void IMUInterface::write8(int reg, int value) {
    wiringPiI2CWriteReg8(i2c_fd_, reg, value);
}

float IMUInterface::convertRawAccel(int16_t raw_value) {
    return static_cast<float>(raw_value) / ACCEL_SCALE;
}

float IMUInterface::convertRawGyro(int16_t raw_value) {
    return static_cast<float>(raw_value) / GYRO_SCALE;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}