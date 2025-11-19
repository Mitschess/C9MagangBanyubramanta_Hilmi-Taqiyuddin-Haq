#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

class OpenCVRosNode : public rclcpp::Node {
public:
    OpenCVRosNode() : Node("opencv_ros_node") {

        publisher_raw_  = image_transport::create_publisher(this, "/raw_image");
        publisher_mask_ = image_transport::create_publisher(this, "/mask_image");

        cap_.open(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Webcam Error");
        }

        cap_.set(cv::CAP_PROP_CONVERT_RGB, true);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

        hueMin1 = 0;   hueMax1 = 32;
        hueMin2 = 160; hueMax2 = 180;
        satMin = 100;  satMax = 255;
        valMin = 100;  valMax = 255;

        namedWindow("HSV Adjustments", WINDOW_AUTOSIZE);

        Mat bg = Mat(400, 400, CV_8UC3, Scalar(255,255,255));
        imshow("HSV Adjustments", bg);

        createTrackbar("Hue Min 1", "HSV Adjustments", &hueMin1, 180);
        createTrackbar("Hue Max 1", "HSV Adjustments", &hueMax1, 180);
        createTrackbar("Hue Min 2", "HSV Adjustments", &hueMin2, 180);
        createTrackbar("Hue Max 2", "HSV Adjustments", &hueMax2, 180);
        createTrackbar("Sat Min",  "HSV Adjustments", &satMin, 255);
        createTrackbar("Sat Max",  "HSV Adjustments", &satMax, 255);
        createTrackbar("Val Min",  "HSV Adjustments", &valMin, 255);
        createTrackbar("Val Max",  "HSV Adjustments", &valMax, 255);

        timer_ = this->create_wall_timer(
            33ms, std::bind(&OpenCVRosNode::timerCallback, this)
        );
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher publisher_raw_;
    image_transport::Publisher publisher_mask_;

    cv::VideoCapture cap_;

    int hueMin1, hueMin2, hueMax1, hueMax2;
    int satMin, satMax, valMin, valMax;

    void timerCallback() {
        Mat frame, hsv, mask1, mask2, mask;

        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "⚠ No frame captured");
            return;
        }

        // Convert ke HSV
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        // Range 1
        inRange(hsv,
            Scalar(hueMin1, satMin, valMin),
            Scalar(hueMax1, satMax, valMax),
            mask1
        );

        // Range 2
        inRange(hsv,
            Scalar(hueMin2, satMin, valMin),
            Scalar(hueMax2, satMax, valMax),
            mask2
        );

        bitwise_or(mask1, mask2, mask);

        // Publish raw image
        std_msgs::msg::Header header;
        header.stamp = this->now();
        auto raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        publisher_raw_.publish(*raw_msg);

        // Publish mask image
        auto mask_msg = cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
        publisher_mask_.publish(*mask_msg);

        // Tidak ada imshow() debug
        waitKey(1);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpenCVRosNode>());
    rclcpp::shutdown();
    return 0;
}
