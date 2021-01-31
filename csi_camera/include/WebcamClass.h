
#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstdlib>
#include <stdio.h>


class VideoHandler{
private:
    cv::Mat m_frame;
    cv::VideoCapture m_cap;

    int m_deviceID;             // 0 = open default camera
    int m_apiID;                // 0 = autodetect default API

public:
    VideoHandler(const int &device_ID_in = 0): m_deviceID(device_ID_in){
        m_apiID = cv::CAP_ANY;
        grab_cam();
    }

    ~VideoHandler(){
        
    }

    void grab_cam(){
        // open selected camera using selected API
        m_cap.open(m_deviceID, m_apiID);
        // check if we succeeded
        if (!m_cap.isOpened()) {
            std::cerr << "ERROR! Unable to open camera\n";
            exit(EXIT_FAILURE);
        }
        //--- GRAB AND WRITE LOOP
        std::cout << "Start grabbing" << std::endl
            << "Press any key to terminate" << std::endl;
    }

    bool get_frame(cv::Mat &src){
        // wait for a new frame from camera and store it into 'frame'
        m_cap.read(src);
        // check if we succeeded
        if (src.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            return false;
        }
        return true;
    }
};


// Usage
// int main(){
//     VideoHandler vh(0);
//     cv::Mat my_img;

//     // Usage
//     while(vh.get_frame(my_img)){
//         // show live and wait for a key with timeout long enough to show images
//         cv::imshow("Live", my_img);
//         if (cv::waitKey(5) >= 0)
//             break;
//     }

//     return 0;
// }