#include "../include/WebcamClass.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Webcam Image Publisher Class
class WebcamPub {
private:
    VideoHandler *m_vh;

    cv::Mat m_image;
    ros::NodeHandle m_nh;
    ros::Publisher m_pub;
    sensor_msgs::ImagePtr m_msg;
public:
    WebcamPub(const int& device_ID_in = 0 ){
        m_pub = m_nh.advertise<sensor_msgs::Image>("webcam_image", 1);
        m_vh = new VideoHandler(device_ID_in);
    }

    ~WebcamPub(){
        delete m_vh;
    }

    void get_img(){
        m_vh->get_frame(m_image);

        // // show live and wait for a key with timeout long enough to show images
        // cv::imshow("Live", m_image);
        // if (cv::waitKey(5) >= 0)
        //     break;
    }

    void pub_msg(){
        get_img();
        m_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_image).toImageMsg();
        m_pub.publish(m_msg);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "webcam_pub");
    ros::Rate r(30);
    WebcamPub my_pub(0);

    while( ros::ok() ){
        my_pub.pub_msg();
        r.sleep();
    }


    return 0;
}