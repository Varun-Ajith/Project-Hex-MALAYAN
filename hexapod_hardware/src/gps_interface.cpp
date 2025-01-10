#include "hexapod_hardware/gps_interface.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <cmath>

GPSInterface::GPSInterface() : Node("gps_interface") {
    // Initialize parameters
    this->declare_parameter("port", "/dev/ttyAMA0");
    this->declare_parameter("baud_rate", 9600);
    this->declare_parameter("update_rate", 1.0);  // Hz

    port_name_ = this->get_parameter("port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();

    // Create publishers
    nav_sat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
        "gps/fix", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "gps/pose", 10);

    if (!initializeSerial()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize GPS serial port");
        return;
    }

    if (!configureGPS()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure GPS");
        return;
    }

    // Create timer for GPS reading
    double update_rate = this->get_parameter("update_rate").as_double();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
        std::bind(&GPSInterface::readGPSData, this));

    RCLCPP_INFO(this->get_logger(), "GPS Interface initialized on port %s", 
                port_name_.c_str());
}

GPSInterface::~GPSInterface() {
    closeSerial();
}

bool GPSInterface::initializeSerial() {
    serial_port_ = open(port_name_.c_str(), O_RDWR);
    if (serial_port_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error opening serial port");
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting serial port attributes");
        return false;
    }

    tty.c_cflag &= ~PARENB;  // No parity
    tty.c_cflag &= ~CSTOPB;  // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait up to 1s (10 deciseconds)
    tty.c_cc[VMIN] = 0;

    // Set baud rate
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting serial port attributes");
        return false;
    }

    return true;
}

void GPSInterface::closeSerial() {
    if (serial_port_ >= 0) {
        close(serial_port_);
    }
}

bool GPSInterface::configureGPS() {
    // Configure NEO-6M GPS module
    // Disable all NMEA sentences except GGA
    const char* config_cmds[] = {
        "$PUBX,40,GLL,0,0,0,0*5C\r\n",
        "$PUBX,40,GSA,0,0,0,0*4E\r\n",
        "$PUBX,40,GSV,0,0,0,0*59\r\n",
        "$PUBX,40,RMC,0,0,0,0*47\r\n",
        "$PUBX,40,VTG,0,0,0,0*5E\r\n",
        "$PUBX,40,GGA,0,1,0,0*5A\r\n"
    };

    for (const char* cmd : config_cmds) {
        write(serial_port_, cmd, strlen(cmd));
        usleep(100000);  // Wait 100ms between commands
    }

    return true;
}

void GPSInterface::readGPSData() {
    ssize_t num_bytes = read(serial_port_, &read_buffer_, BUFFER_SIZE - 1);
    
    if (num_bytes > 0) {
        read_buffer_[num_bytes] = '\0';
        std::string data(read_buffer_);
        std::istringstream iss(data);
        std::string line;

        while (std::getline(iss, line)) {
            if (line.find("$GPGGA") != std::string::npos) {
                if (parseGGA(line)) {
                    break;
                }
            }
        }
    }
}

bool GPSInterface::parseGGA(const std::string& sentence) {
    if (!checksum(sentence)) {
        return false;
    }

    std::vector<std::string> tokens;
    std::istringstream iss(sentence);
    std::string token;

    while (std::getline(iss, token, ',')) {
        tokens.push_back(token);
    }

    if (tokens.size() < 15) return false;

    // Parse latitude
    double lat = 0.0;
    if (!tokens[2].empty()) {
        lat = std::stod(tokens[2].substr(0, 2)) + 
              std::stod(tokens[2].substr(2)) / 60.0;
        if (tokens[3] == "S") lat = -lat;
    }

    // Parse longitude
    double lon = 0.0;
    if (!tokens[4].empty()) {
        lon = std::stod(tokens[4].substr(0, 3)) + 
              std::stod(tokens[4].substr(3)) / 60.0;
        if (tokens[5] == "W") lon = -lon;
    }

    // Parse altitude
    double alt = 0.0;
    if (!tokens[9].empty()) {
        alt = std::stod(tokens[9]);
    }

    // Parse fix quality
    int fix_quality = 0;
    if (!tokens[6].empty()) {
        fix_quality = std::stoi(tokens[6]);
    }

    publishGPSData(lat, lon, alt, fix_quality);
    return true;
}

bool GPSInterface::checksum(const std::string& sentence) {
    size_t asterisk_pos = sentence.find('*');
    if (asterisk_pos == std::string::npos || asterisk_pos + 3 >= sentence.length()) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 1; i < asterisk_pos; i++) {
        checksum ^= sentence[i];
    }

    std::string checksum_str = sentence.substr(asterisk_pos + 1, 2);
    uint8_t received_checksum;
    std::stringstream ss;
    ss << std::hex << checksum_str;
    ss >> received_checksum;

    return checksum == received_checksum;
}

void GPSInterface::publishGPSData(double lat, double lon, double alt, int fix_quality) {
    auto nav_msg = sensor_msgs::msg::NavSatFix();
    nav_msg.header.stamp = this->now();
    nav_msg.header.frame_id = "gps_frame";
    
    nav_msg.latitude = lat;
    nav_msg.longitude = lon;
    nav_msg.altitude = alt;

    nav_msg.status.status = (fix_quality > 0) ? 
        sensor_msgs::msg::NavSatStatus::STATUS_FIX : 
        sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;

    nav_msg.position_covariance_type = 
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    nav_sat_pub_->publish(nav_msg);

    // Also publish as PoseStamped
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header = nav_msg.header;
    
    // Convert to local coordinates (simplified - assumes flat earth)
    const double EARTH_RADIUS = 6371000.0;  // meters
    pose_msg.pose.position.x = EARTH_RADIUS * lon * M_PI / 180.0;
    pose_msg.pose.position.y = EARTH_RADIUS * lat * M_PI / 180.0;
    pose_msg.pose.position.z = alt;

    pose_pub_->publish(pose_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPSInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}