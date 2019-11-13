#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char **argv)
{
    VideoCapture cap(0);
    Mat img;

    while(1){
        cap >> img;
        imshow("video",img);
        waitKey(20);
    }

    cap.release();

    return 0;
}
