#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


// 横向的角点数目
const int boardWidth = 9;  
// 纵向的角点数据  
const int boardHeight = 6;
const int boardCorner = boardWidth * boardHeight;  
const int frameNumber = 15;                             //相机标定时需要采用的图像帧数  
const int squareSize = 25;                              //标定板黑白格子的大小 单位mm  
const Size boardSize = Size(boardWidth, boardHeight);   //总的内角点

// 各个图像找到的角点的集合 和objRealPoint 一一对应  
vector<Point2f> corner;                                   //某一副图像找到的角点  

bool mono_detect(Mat &rgbImage){
    Mat gray;
    cvtColor(rgbImage, gray, COLOR_BGR2GRAY);  //  CV_BGR2GRAY  COLOR_BGR2GRAY
    bool isFind = findChessboardCorners(rgbImage, boardSize, corner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
    return isFind;
}

void save(const Mat &Left, const Mat &Right, const int index){
    stringstream ssL, ssR;
    ssL<<"/home/yuan/XSpace/Ending/Camera/stereo/data/"<<setw(5)<<setfill('0')<<index << "_left.png";
    ssR<<"/home/yuan/XSpace/Ending/Camera/stereo/data/"<<setw(5)<<setfill('0')<<index << "_right.png";
    
    cout << "save image: " << ssL.str() << endl; 
    imwrite(ssL.str(), Left);
    imwrite(ssR.str(), Right);
}

int main()
{
    VideoCapture Camera(6);
    if (!Camera.isOpened())
    {
        cout << "Could not open the Camera " << endl;
        return -1;
    }
    Mat Fream;
    Camera >> Fream;
//     namedWindow("Stereo", 1);
    namedWindow("Right", 1);
    namedWindow("Left", 1);
    
//     imshow("Stereo",Fream);
    Mat DoubleImage;
    //此处改成你的脚本存放绝对路径
    system("sudo sh /home/yuan/XSpace/Ending/Camera/stereo/camera.sh");  
    //imshow("【双目视图】",Fream);
    
    int index = 0;
    bool flag = false;
    
    int good_pts = 0;
    int img_id = 1;
    
    while (index < frameNumber)
    {
        Camera >> Fream;
        if (Fream.empty()) break;
        resize(Fream, DoubleImage, Size(640, 240), (0, 0), (0, 0), INTER_AREA);
//         imshow("Stereo", DoubleImage);
        Mat LeftImage = DoubleImage(Rect(0, 0, 320, 240));
        Mat RightImage = DoubleImage(Rect(320, 0, 320, 240));
        moveWindow("Left", 10, 10);
        moveWindow("Right", 10, 320);
        
        imshow("Left", LeftImage);
        imshow("Right", RightImage);
        
        int c = waitKey(10);
        
        if(c == 27) break;
        Mat intrinsic_L, distortion_coeff_L, intrinsic_R, distortion_coeff_R;        
        
        bool goodLeft = mono_detect(LeftImage);
        bool goodRight = mono_detect(RightImage);
        
        if(goodLeft && goodRight && c == 110){  // 110 == n
            cout << "<== A good image. Next, please ==>" << endl;
            index++;
        }
        else{
//             cout << "<== A bad image. Again, please ==>" << endl;
            continue;
        }
        save(LeftImage, RightImage, index);
    }
    return 0;
}

