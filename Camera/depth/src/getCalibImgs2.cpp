#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "OpenNI.h"
#include <sys/time.h>

using namespace std;
using namespace cv;
using namespace openni;
static VideoCapture capture;
static Device device;
static VideoStream depth;
static VideoStream color;
static VideoStream ir;
static VideoFrameRef depthFrame;
static VideoFrameRef colorFrame;
static VideoFrameRef irFrame;

const static int MAX_DEPTH_VALUE = 0xffff;
static float* pDepthHist = NULL;
const static int OPENNI_READ_WAIT_TIMEOUT = 500;

static int mirrorColor = 0;
Mat depthImg(480, 640, CV_8UC3);
Mat depthRaw(480, 640, CV_16UC1, Scalar(0));
Mat irImg(480, 640, CV_8UC3, Scalar(0));
Mat colorImg(480, 640, CV_8UC3);

static bool quit = false;
static int openAllStream = 0;
static bool isUvcCamera = true;


cv::Size pattern_size = Size(9, 6);
vector<Point2f> corner_points_buf;
bool check(const Mat& color){
    if (findChessboardCorners(color, pattern_size, corner_points_buf) == 0)
        return false;
    return true;
}

void saveImage(const Mat &color, int index);

string getPath(int index){
    stringstream ss;
    ss<<"/home/yuan/Xstudio/Opencv/opencv-4.2.0/samples/data/left"<<setw(2)<<setfill('0')<<index<<".jpg";
    
    return ss.str();
}

void debug(int i){
    cout << "=============== " << i << " ===============" << endl;
}

int main(int argc, char*argv[])
{
    VideoCapture cap(4);
    Mat frame, stdImage;
    cap.read(frame);
    cout << frame.type() << endl;
    Mat twoImages(frame.rows, frame.cols*2, CV_8UC3, Scalar(0));
    int index = 11;
    
    Rect rectFrame(0, 0, frame.cols, frame.rows);
    Rect rectStd(frame.cols, 0, frame.cols, frame.rows);
    while (!quit)
    {
        cap.read(frame);
//         imshow("RGB", frame);
        
        stdImage = imread(getPath(index));
        if(stdImage.data==nullptr){
            cout << "no image" << endl;
            return -1;
        }
        resize(stdImage, stdImage, Size(frame.cols, frame.rows));
        frame.copyTo(twoImages(rectFrame));
        stdImage.copyTo(twoImages(rectStd));
        
        imshow("Common", twoImages);
        int key = waitKey(10);
                
        if(key==110 && check(frame)){ // key = n
            saveImage(frame, index);
            index++;
            if(index==10) index++;  // skip left10.jpg
        }
        
        if(key==27) break;
    }
        
    cap.release();
    return 0;
}

// depth: 1280 x 480        color: 640 x 480
void saveImage(const Mat &color, int index){
    stringstream ssD, ssC;
    string data_dir = "/home/yuan/XSpace/Ending/Camera/depth/data/";
    ssD << data_dir << setw(5) << setfill('0') << index << "_depth.png";
    ssC << data_dir << setw(5) << setfill('0') << index << "_color.png";
    
    imwrite(ssC.str(), color);
    
    cout << "save " << ssC.str() << endl; 
}
