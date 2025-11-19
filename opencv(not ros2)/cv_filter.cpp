#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int hueMin1 = 0, hueMin2 = 0, hueMax1 = 32, hueMax2 = 180;
int satMin = 100, satMax = 255;
int valMin = 100, valMax = 255;

const int HueMax = 180;
const int satValMax = 255;

int main(){
    VideoCapture cap("second.mp4");
    
    if(!cap.isOpened()){
        cout << "video error" << endl;
        return -1;
    }
    
    namedWindow("Original", WINDOW_AUTOSIZE);
    namedWindow("Mask", WINDOW_AUTOSIZE);
    namedWindow("HSV Adjustments", WINDOW_AUTOSIZE);
    
    Mat whiteBg = Mat(1, 400, CV_8UC3, Scalar(255, 255, 255));
    imshow("HSV Adjustments", whiteBg);
    
    createTrackbar("Hue Min 1", "HSV Adjustments", &hueMin1, HueMax);
    createTrackbar("Hue Max 1", "HSV Adjustments", &hueMax1, HueMax);
    createTrackbar("Hue Min 2", "HSV Adjustments", &hueMin2, HueMax);
    createTrackbar("Hue Max 2", "HSV Adjustments", &hueMax2, HueMax);
    createTrackbar("Sat Min", "HSV Adjustments", &satMin, satValMax);
    createTrackbar("Sat Max", "HSV Adjustments", &satMax, satValMax);
    createTrackbar("Val Min", "HSV Adjustments", &valMin, satValMax);
    createTrackbar("Val Max", "HSV Adjustments", &valMax, satValMax);
    
    while (true){
        Mat frame, framehsv, maskHSV1, maskHSV2, maskHSV;
        
        if(!cap.read(frame)){
            cout << "Video selesai" << endl;
            break;
        }
        
        cvtColor(frame, framehsv, COLOR_BGR2HSV);
        
        Scalar minHSV1 = Scalar(hueMin1, satMin, valMin);
        Scalar maxHSV1 = Scalar(hueMax1, satMax, valMax);
        inRange(framehsv, minHSV1, maxHSV1, maskHSV1);

        Scalar minHSV2 = Scalar(hueMin2, satMin, valMin);
        Scalar maxHSV2 = Scalar(hueMax2, satMax, valMax);
        inRange(framehsv, minHSV2, maxHSV2, maskHSV2);
        
        bitwise_or(maskHSV1, maskHSV2, maskHSV);
        
        imshow("Original", frame);
        imshow("Mask", maskHSV);
        
        char key = (char)waitKey(30);
        
        if(key == 'q' || key == 'Q'){
            cout << "Keluar..." << endl;
            break;
        }
    }
    
    cap.release();
    destroyAllWindows();
    
    return 0;
}
