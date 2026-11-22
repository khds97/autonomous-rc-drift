// ============================================
// dual_camera_publisher.cpp
// Raspberry Pi 4 - Dual camera publisher
// V4L2 direct access: MJPG -> grayscale decode (skip BGR conversion)
// Each frame is stamped with the V4L2 buffer time (when the frame arrived),
// converted to the ROS clock, so the stamps line up with the IMU stamps.
// ROS2 Foxy, C++
// ============================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include "sensor_publisher/time_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <string>
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

enum class ReadResult { kFrame, kSkipped, kFailed };

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
        if (ioctl(fd_, VIDIOC_S_PARM, &parm) == 0 &&
            parm.parm.capture.timeperframe.numerator != 0) {
            // The driver writes back the frame interval it actually accepted
            actual_fps_ = static_cast<double>(parm.parm.capture.timeperframe.denominator) /
                          parm.parm.capture.timeperframe.numerator;
        }

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

    // Limits the frame rate to max_fps by dropping the extra frames before they are
    // decoded. 0 disables the limit.
    void set_max_fps(double max_fps)
    {
        min_interval_ns_ = max_fps > 0.0 ? static_cast<int64_t>(1e9 / max_fps) : 0;
    }

    // Capture MJPG and decode directly as grayscale.
    // frame_ns is the V4L2 buffer time of the frame in nanoseconds (kernel clock),
    // or 0 if the driver gives no time.
    // Returns kSkipped when the frame was dropped by the frame rate limit.
    ReadResult read(cv::Mat& gray, int64_t* frame_ns)
    {
        // Wait for frame (100ms timeout)
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        timeval tv{0, 100000};
        if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return ReadResult::kFailed;

        v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) return ReadResult::kFailed;

        if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN) {
            *frame_ns = 0;
        } else {
            *frame_ns = static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
                        static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
        }

        if (*frame_ns != 0 && !frame_due(*frame_ns)) {
            ioctl(fd_, VIDIOC_QBUF, &buf);
            return ReadResult::kSkipped;
        }

        // Decode MJPG directly as grayscale (skip BGR entirely)
        std::vector<uchar> mjpg_data(
            static_cast<uchar*>(buffers_[buf.index].start),
            static_cast<uchar*>(buffers_[buf.index].start) + buf.bytesused);

        gray = cv::imdecode(mjpg_data, cv::IMREAD_GRAYSCALE);

        // Re-queue buffer
        ioctl(fd_, VIDIOC_QBUF, &buf);

        return gray.empty() ? ReadResult::kFailed : ReadResult::kFrame;
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

    // Frame rate the driver accepted, 0 if unknown
    double actual_fps() const { return actual_fps_; }

private:
    // Frame rate limiter. A frame is kept when it arrives no earlier than a quarter
    // period before the next due time, so a camera that delivers exactly max_fps
    // loses no frames. After a stall the schedule restarts from the current frame.
    bool frame_due(int64_t frame_ns)
    {
        if (min_interval_ns_ <= 0) return true;
        if (next_due_ns_ == 0 || frame_ns - next_due_ns_ > min_interval_ns_) {
            next_due_ns_ = frame_ns + min_interval_ns_;
            return true;
        }
        if (frame_ns >= next_due_ns_ - min_interval_ns_ / 4) {
            next_due_ns_ += min_interval_ns_;
            return true;
        }
        return false;
    }

    int fd_;
    double actual_fps_ = 0.0;
    int64_t min_interval_ns_ = 0;
    int64_t next_due_ns_ = 0;
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
        // Frame rate limit for the published images [Hz], 0 = no limit. Some cameras
        // deliver more than the requested fps; the extra frames are dropped before
        // decoding.
        this->declare_parameter("max_fps", 30.0);
        // Constant shift added to each camera's stamps [s]. A positive value moves
        // the stamps later. Use it to correct a measured camera-to-IMU time offset.
        this->declare_parameter("fixed_time_offset_s",    0.0);
        this->declare_parameter("rotating_time_offset_s", 0.0);

        auto fixed_dev    = this->get_parameter("fixed_device").as_string();
        auto rotating_dev = this->get_parameter("rotating_device").as_string();
        width_  = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        fps_    = this->get_parameter("fps").as_int();
        const double max_fps = this->get_parameter("max_fps").as_double();
        const double fixed_offset_s    = this->get_parameter("fixed_time_offset_s").as_double();
        const double rotating_offset_s = this->get_parameter("rotating_time_offset_s").as_double();

        // --- Open cameras ---
        if (!fixed_cam_.open(fixed_dev, width_, height_, fps_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open fixed camera: %s", fixed_dev.c_str());
            rclcpp::shutdown(); return;
        }
        if (!rotating_cam_.open(rotating_dev, width_, height_, fps_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open rotating camera: %s", rotating_dev.c_str());
            rclcpp::shutdown(); return;
        }

        fixed_cam_.set_max_fps(max_fps);
        rotating_cam_.set_max_fps(max_fps);

        RCLCPP_INFO(this->get_logger(),
            "Dual cameras initialized | Resolution: %dx%d | FPS: %d | max published FPS: %.1f",
            width_, height_, fps_, max_fps);
        warn_if_fps_differs("fixed", fixed_cam_.actual_fps());
        warn_if_fps_differs("rotating", rotating_cam_.actual_fps());

        // --- Publishers ---
        fixed_pub_    = image_transport::create_publisher(this, "/camera/fixed/image_raw");
        rotating_pub_ = image_transport::create_publisher(this, "/camera/rotating/image_raw");

        // --- Start capture threads ---
        // The loops are paced by the cameras: each iteration waits for the next frame.
        fixed_thread_ = std::thread(
            &DualCameraPublisher::capture_loop, this, std::ref(fixed_cam_),
            std::ref(fixed_pub_), std::string("fixed"),
            std::string("fixed_camera_link"), fixed_offset_s);
        rotating_thread_ = std::thread(
            &DualCameraPublisher::capture_loop, this, std::ref(rotating_cam_),
            std::ref(rotating_pub_), std::string("rotating"),
            std::string("rotating_camera_link"), rotating_offset_s);
    }

    ~DualCameraPublisher()
    {
        running_ = false;
        if (fixed_thread_.joinable())    fixed_thread_.join();
        if (rotating_thread_.joinable()) rotating_thread_.join();
    }

private:
    void warn_if_fps_differs(const char* name, double actual_fps)
    {
        if (actual_fps > 0.0 && std::abs(actual_fps - fps_) > 0.5) {
            RCLCPP_WARN(this->get_logger(),
                "%s camera: requested %d FPS, driver accepted %.1f FPS",
                name, fps_, actual_fps);
        }
    }

    void capture_loop(
        V4L2Camera& cam,
        image_transport::Publisher& pub,
        const std::string& name,
        const std::string& frame_id,
        double time_offset_s)
    {
        auto rate_start = std::chrono::steady_clock::now();
        int published_count = 0;
        int captured_count = 0;

        while (running_ && rclcpp::ok()) {
            cv::Mat gray;
            int64_t frame_ns = 0;
            const ReadResult result = cam.read(gray, &frame_ns);
            if (result == ReadResult::kFrame) {
                if (publish_frame(pub, gray, frame_id, frame_ns, time_offset_s)) {
                    ++published_count;
                }
                ++captured_count;
            } else if (result == ReadResult::kSkipped) {
                ++captured_count;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "Failed to read %s camera frame", name.c_str());
                std::this_thread::sleep_for(1ms);
            }

            // Report the frame rates every 5 s
            const auto now = std::chrono::steady_clock::now();
            const double elapsed = std::chrono::duration<double>(now - rate_start).count();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(this->get_logger(),
                    "%s camera: %.1f FPS published (%.1f FPS from the camera)",
                    name.c_str(), published_count / elapsed, captured_count / elapsed);
                published_count = 0;
                captured_count = 0;
                rate_start = now;
            }
        }
    }

    bool publish_frame(
        image_transport::Publisher& pub,
        const cv::Mat& gray,
        const std::string& frame_id,
        int64_t frame_ns,
        double time_offset_s)
    {
        // Frames can wait in the V4L2 queue for up to a couple of frame periods
        bool from_kernel = true;
        rclcpp::Time stamp = sensor_publisher::kernelEventToRosTime(
            *this->get_clock(), frame_ns, 200000000LL, &from_kernel);
        if (!from_kernel) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "%s: invalid kernel timestamp, dropping frame",
                frame_id.c_str());
            return false;
        }
        stamp = rclcpp::Time(
            stamp.nanoseconds() + static_cast<int64_t>(time_offset_s * 1e9),
            stamp.get_clock_type());

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "mono8", gray).toImageMsg();
        msg->header.stamp    = stamp;
        msg->header.frame_id = frame_id;
        pub.publish(*msg);
        return true;
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
