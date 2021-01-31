// #include "csi_camera/VideoHandler.h"


// // Usage
// int main(){
//     VideoHandler vh(0);
//     cv::Mat my_img;

//     while(vh.get_frame(my_img)){
//         // show live and wait for a key with timeout long enough to show images
//         cv::imshow("Live", my_img);
//         if (cv::waitKey(5) >= 0)
//             break;
//     }

//     return 0;
// }

#include <opencv2/highgui.hpp>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

int main (int argc, char* argv[])
{

    VideoCapture cap(0);
    if(!cap.isOpened()) return -1;
    namedWindow("video", CV_MINOR_VERSION);

    clock_t start = clock();

    for (int i = 0; i < 101; ++i)
    {
        Mat frame;
        cap >> frame;
        imshow("video", frame);
        waitKey(1);
    }

    clock_t finish = clock();

    double time_elapsed = (finish - start) / 1000.0;
    double fps = 100 / time_elapsed;

    cout << "\n\nTIME: " << time_elapsed << "\n\nFPS: " << fps << "\n\n";

    return 0;
}