// ============================================
// dual_camera_publisher.cpp
// Raspberry Pi 4 - Dual camera publisher
// V4L2 direct access: MJPG -> grayscale decode (skip BGR conversion)
// ROS2 Foxy, C++
// ============================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>

// V4L2 headers
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

using namespace std::chrono_literals;

#define NUM_BUFFERS 2

struct V4L2Buffer {
    void* start;
    size_t length;
};

class V4L2Camera
{
public:
    V4L2Camera() : fd_(-1) {}

    bool open(const std::string& device, int width, int height, int fps)
    {
        fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) return false;

        // Set MJPG format
        v4l2_format fmt{};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = width;
        fmt.fmt.pix.height      = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) return false;

        // Set FPS
        v4l2_streamparm parm{};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator = fps;
        ioctl(fd_, VIDIOC_S_PARM, &parm);

        // Request buffers
        v4l2_requestbuffers req{};
        req.count  = NUM_BUFFERS;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) return false;

        // Map buffers
        buffers_.resize(NUM_BUFFERS);
        for (int i = 0; i < NUM_BUFFERS; i++) {
            v4l2_buffer buf{};
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = i;
            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) return false;

            buffers_[i].length = buf.length;
            buffers_[i].start  = mmap(nullptr, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) return false;

            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) return false;
        }

        // Start streaming
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        return ioctl(fd_, VIDIOC_STREAMON, &type) >= 0;
    }

    // Capture MJPG and decode directly as grayscale
    bool read(cv::Mat& gray)
    {
        // Wait for frame (100ms timeout)
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval tv{0, 100000};
        if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;

        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) return false;

        // Decode MJPG directly as grayscale (skip BGR entirely)
        std::vector<uchar> mjpg_data(
            static_cast<uchar*>(buffers_[buf.index].start),
            static_cast<uchar*>(buffers_[buf.index].start) + buf.bytesused);

        gray = cv::imdecode(mjpg_data, cv::IMREAD_GRAYSCALE);

        // Re-queue buffer
        ioctl(fd_, VIDIOC_QBUF, &buf);

        return !gray.empty();
    }

    void close()
    {
        if (fd_ < 0) return;
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        for (auto& buf : buffers_)
            munmap(buf.start, buf.length);
        ::close(fd_);
        fd_ = -1;
    }

    ~V4L2Camera() { close(); }

private:
    int fd_;
    std::vector<V4L2Buffer> buffers_;
};

// ============================================
// ROS2 Node
// ============================================
class DualCameraPublisher : public rclcpp::Node
{
public:
    DualCameraPublisher() : Node("dual_camera_publisher"), running_(true)
    {
        // --- Parameters ---
        this->declare_parameter("fixed_device",    std::string("/dev/video0"));
        this->declare_parameter("rotating_device", std::string("/dev/video2"));
        this->declare_parameter("width",  640);
        this->declare_parameter("height", 480);
        this->declare_parameter("fps",    30);

        auto fixed_dev    = this->get_parameter("fixed_device").as_string();
        auto rotating_dev = this->get_parameter("rotating_device").as_string();
        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_    = this->get_parameter("fps").as_int();

        // --- Open cameras ---
        if (!fixed_cam_.open(fixed_dev, width_, height_, fps_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open fixed camera: %s", fixed_dev.c_str());
            rclcpp::shutdown(); return;
        }
        if (!rotating_cam_.open(rotating_dev, width_, height_, fps_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open rotating camera: %s", rotating_dev.c_str());
            rclcpp::shutdown(); return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Dual cameras initialized | Resolution: %dx%d | FPS: %d", width_, height_, fps_);

        // --- Publishers ---
        fixed_pub_    = image_transport::create_publisher(this, "/camera/fixed/image_raw");
        rotating_pub_ = image_transport::create_publisher(this, "/camera/rotating/image_raw");

        // --- Start capture threads ---
        fixed_thread_    = std::thread(&DualCameraPublisher::fixed_capture_loop, this);
        rotating_thread_ = std::thread(&DualCameraPublisher::rotating_capture_loop, this);
    }

    ~DualCameraPublisher()
    {
        running_ = false;
        if (fixed_thread_.joinable())    fixed_thread_.join();
        if (rotating_thread_.joinable()) rotating_thread_.join();
    }

private:
    void fixed_capture_loop()
    {
        int interval_ms = 1000 / fps_;
        while (running_ && rclcpp::ok()) {
            auto start = std::chrono::steady_clock::now();

            cv::Mat gray;
            if (fixed_cam_.read(gray)) {
                publish_frame(fixed_pub_, gray, "fixed_camera_link");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Failed to read fixed camera frame");
            }

            auto elapsed    = std::chrono::steady_clock::now() - start;
            auto sleep_time = std::chrono::milliseconds(interval_ms) - elapsed;
            if (sleep_time > std::chrono::milliseconds(0))
                std::this_thread::sleep_for(sleep_time);
        }
    }

    void rotating_capture_loop()
    {
        int interval_ms = 1000 / fps_;
        while (running_ && rclcpp::ok()) {
            auto start = std::chrono::steady_clock::now();

            cv::Mat gray;
            if (rotating_cam_.read(gray)) {
                publish_frame(rotating_pub_, gray, "rotating_camera_link");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Failed to read rotating camera frame");
            }

            auto elapsed    = std::chrono::steady_clock::now() - start;
            auto sleep_time = std::chrono::milliseconds(interval_ms) - elapsed;
            if (sleep_time > std::chrono::milliseconds(0))
                std::this_thread::sleep_for(sleep_time);
        }
    }

    void publish_frame(
        image_transport::Publisher& pub,
        const cv::Mat& gray,
        const std::string& frame_id)
    {
        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "mono8", gray).toImageMsg();
        msg->header.stamp    = this->now();
        msg->header.frame_id = frame_id;
        pub.publish(*msg);
    }

    V4L2Camera fixed_cam_;
    V4L2Camera rotating_cam_;
    image_transport::Publisher fixed_pub_;
    image_transport::Publisher rotating_pub_;
    std::thread fixed_thread_;
    std::thread rotating_thread_;
    std::atomic<bool> running_;
    int width_;
    int height_;
    int fps_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualCameraPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
