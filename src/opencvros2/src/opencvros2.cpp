#include <chrono>
#include <iostream>

#include <opencv2/opencv.hpp> 
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"


using namespace std::chrono_literals;
using namespace cv;
using namespace std;

class opencvros2Node : public rclcpp::Node{
    public:
        opencvros2Node() : Node("opencvros2Node"){
            publisher_raw = image_transport::create_publisher(this, "/raw_image");
            publisher_mask = image_transport::create_publisher(this, "/mask_image");

            hueMin1 = 0; hueMax1 = 10;
            hueMin2 = 170; hueMax2 = 180;
            satMin = 100; satMax = 255;
            valMin = 100; valMax = 255;

            namedWindow("trackbar", WINDOW_AUTOSIZE);
            createTrackbar("Hue Min1", "trackbar", &hueMin1, 180);
            createTrackbar("Hue Max1", "trackbar", &hueMax1, 180);
            createTrackbar("Hue Min2", "trackbar", &hueMin2, 180);
            createTrackbar("Hue Max2", "trackbar", &hueMax2, 180);
            createTrackbar("Sat Min", "trackbar", &satMin, 255);
            createTrackbar("Sat Max", "trackbar", &satMax, 255);
            createTrackbar("Val Min", "trackbar", &valMin, 255);
            createTrackbar("Val Max", "trackbar", &valMax, 255);
    
            cap_.open("src/opencvros2/resource/second.mp4"); // Ganti dengan 0 untuk webcam
            if(!cap_.isOpened()){
                RCLCPP_ERROR(this->get_logger(), "Error opening video stream or file");
                //rclcpp ::shutdown(); 
            }

            timer_= this->create_wall_timer(30ms, std::bind(&opencvros2Node::timerCallBack, this));
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        image_transport::Publisher publisher_raw;
        image_transport::Publisher publisher_mask;
        cv::VideoCapture cap_;
        
        int hueMin1, hueMax1, hueMin2, hueMax2, satMin, satMax, valMin, valMax; 

        void timerCallBack(){
            Mat frame, framehsv, maskHSV1, maskHSV2, maskHSV, resultHSV;

            if (!cap_.read(frame)){
                RCLCPP_ERROR(this->get_logger(), "No frame captured from video source");
                return;
            }

            cvtColor(frame, framehsv, COLOR_BGR2HSV);

            Scalar minHSV1 = Scalar(hueMin1, satMin, valMin);
            Scalar maxHSV1 = Scalar(hueMax1, satMax, valMax);
            inRange(framehsv, minHSV1, maxHSV1, maskHSV1);

            Scalar minHSV2 = Scalar(hueMin2, satMin, valMin);
            Scalar maxHSV2 = Scalar(hueMax2, satMax, valMax);
            inRange(framehsv, minHSV2, maxHSV2, maskHSV2);

            bitwise_or(maskHSV1, maskHSV2, maskHSV);
            bitwise_and(frame, frame, resultHSV, maskHSV);

            //imshow("hasil", resultHSV);
            //imshow("video", frame);

            if(waitKey(1) == 'a'){
                RCLCPP_INFO(this->get_logger(), "Tombol 'a' ditekan, menutup node.");
                rclcpp::shutdown();
            }

            if (!frame.empty()) {
                std_msgs::msg::Header header;
                header.stamp = this->now(); // Set timestamp

                sensor_msgs::msg::Image::SharedPtr raw_msg;
                raw_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                publisher_raw.publish(*raw_msg);

                sensor_msgs::msg::Image::SharedPtr mask_msg;
                mask_msg = cv_bridge::CvImage(header, "mono8", resultHSV).toImageMsg();
                publisher_mask.publish(*mask_msg);
            }
        }
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<opencvros2Node>());
  rclcpp::shutdown();
  destroyAllWindows();
  return 0;
}