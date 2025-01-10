#include "hexapod_hardware/camera_interface.hpp"

CameraInterface::CameraInterface() : Node("camera_interface") {
    // Declare parameters
    this->declare_parameter("frame_rate", 30);
    this->declare_parameter("width", 640);
    this->declare_parameter("height", 480);
    this->declare_parameter("night_mode", false);
    this->declare_parameter("camera_name", "raspicam");
    this->declare_parameter("camera_info_url", "package://hexapod_hardware/config/camera_info.yaml");

    // Get parameters
    frame_rate_ = this->get_parameter("frame_rate").as_int();
    capture_width_ = this->get_parameter("width").as_int();
    capture_height_ = this->get_parameter("height").as_int();
    night_mode_ = this->get_parameter("night_mode").as_bool();

    // Initialize publishers
    image_transport::ImageTransport it(this);
    image_pub_ = it.advertise("camera/image_raw", 1);
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 1);

    // Initialize camera info manager
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    std::string camera_name = this->get_parameter("camera_name").as_string();
    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();
    camera_info_manager_->setCameraName(camera_name);
    camera_info_manager_->loadCameraInfo(camera_info_url);

    initializeCamera();

    // Create timer for image capture and publishing
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frame_rate_),
        std::bind(&CameraInterface::captureAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "Camera interface initialized");
}

CameraInterface::~CameraInterface() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void CameraInterface::initializeCamera() {
    // Initialize Raspberry Pi Camera using OpenCV
    cap_.open(0); // Use first camera device

    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
        return;
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
    cap_.set(cv::CAP_PROP_FPS, frame_rate_);

    // Configure camera for night vision if enabled
    setNightMode(night_mode_);
}

void CameraInterface::setNightMode(bool enabled) {
    if (enabled) {
        // Commands to enable IR-CUT for night vision
        system("v4l2-ctl --set-ctrl wide_dynamic_range=1");
        system("v4l2-ctl --set-ctrl exposure_dynamic_framerate=1");
        system("v4l2-ctl --set-ctrl auto_exposure=1");
    } else {
        // Commands to disable IR-CUT for normal vision
        system("v4l2-ctl --set-ctrl wide_dynamic_range=0");
        system("v4l2-ctl --set-ctrl exposure_dynamic_framerate=0");
        system("v4l2-ctl --set-ctrl auto_exposure=0");
    }
}

void CameraInterface::captureAndPublish() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture frame!");
        return;
    }

    // Convert OpenCV image to ROS message
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_optical_frame";

    sensor_msgs::msg::Image::SharedPtr image_msg;
    try {
        image_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Publish the image
    image_pub_.publish(image_msg);

    // Publish camera info
    auto camera_info_msg = camera_info_manager_->getCameraInfo();
    camera_info_msg.header = header;
    camera_info_pub_->publish(camera_info_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraInterface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}