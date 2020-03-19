#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int frameNumber = 10000;

void save(const Mat &Left, const Mat &Right, const int index){
    stringstream ssL, ssR;
    ssL<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_left.png";
    ssR<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_right.png";
    
    cout << "save image: " << ssL.str() << endl; 
    imwrite(ssL.str(), Left);
    imwrite(ssR.str(), Right);
}

int main(){
    VideoCapture Camera(4);
    if (!Camera.isOpened())
    {
        cout << "Could not open the Camera " << endl;
        return -1;
    }
    Mat Fream;
    Camera >> Fream;
    
    namedWindow("Right", 1);
    namedWindow("Left", 1);
    
    Mat DoubleImage;
    
    system("sudo sh /home/yuan/XSpace/Ending/Camera/stereo/camera.sh");  
    
    int index = 0;
    
    while (index < frameNumber)
    {
        Camera >> Fream;
        if (Fream.empty()) break;
        resize(Fream, DoubleImage, Size(640, 240), (0, 0), (0, 0), INTER_AREA);
        Mat LeftImage = DoubleImage(Rect(0, 0, 320, 240));
        Mat RightImage = DoubleImage(Rect(320, 0, 320, 240));
        moveWindow("Left", 10, 10);
        moveWindow("Right", 10, 320);
        
        imshow("Left", LeftImage);
        imshow("Right", RightImage);
        
        int c = waitKey(10);
        
        if(c == 27) break;
        
        save(LeftImage, RightImage, index++);
    }
}
