
#include <iostream>
#include <stdlib.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include<opencv2/opencv.hpp>

using namespace cv;
using namespace std;


// 横向的角点数目
const int boardWidth = 9;
// 纵向的角点数据  
const int boardHeight = 6;
const int boardCorner = boardWidth * boardHeight;  
const int frameNumber = 15;                             //相机标定时需要采用的图像帧数  
const int squareSize = 20; // 25                        //标定板黑白格子的大小 单位mm  
const Size boardSize = Size(boardWidth, boardHeight);   //总的内角点

// Mat intrinsic;                                       //相机内参数  
// Mat distortion_coeff;                                //相机畸变参数  
vector<Mat> rvecs;                                      //旋转向量  
vector<Mat> tvecs;                                      //平移向量  
//各个图像找到的角点的集合 和objRealPoint 一一对应  
vector<vector<Point2f>> corners;
vector<vector<Point3f>> objRealPoint;                   //各副图像的角点的实际物理坐标集合  
vector<Point2f> corner;                                 //某一副图像找到的角点  

/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize)
{
    vector<Point3f> imgpoint;
    for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)
    {
        for (int colIndex = 0; colIndex < boardwidth; colIndex++)
        {
            imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
        }
    }
    for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
    {
        obj.push_back(imgpoint);
    }
}

/*设置相机的初始参数 也可以不估计*/
void guessCameraParam(Mat &intrinsic, Mat &distortion_coeff)
{
    /*分配内存*/
    intrinsic.create(3, 3, CV_64FC1);    //相机内参数
    distortion_coeff.create(5, 1, CV_64FC1);  //畸变参数

    /*
    fx 0 cx
    0 fy cy
    0 0  1     内参数
    */
    intrinsic.at<double>(0, 0) = 256.8093262;   //fx         
    intrinsic.at<double>(0, 2) = 160.2826538;   //cx  
    intrinsic.at<double>(1, 1) = 254.7511139;   //fy  
    intrinsic.at<double>(1, 2) = 127.6264572;   //cy  

    intrinsic.at<double>(0, 1) = 0;
    intrinsic.at<double>(1, 0) = 0;
    intrinsic.at<double>(2, 0) = 0;
    intrinsic.at<double>(2, 1) = 0;
    intrinsic.at<double>(2, 2) = 1;

    /*
    k1 k2 p1 p2 p3    畸变参数
    */
    distortion_coeff.at<double>(0, 0) = -0.193740;  //k1  
    distortion_coeff.at<double>(1, 0) = -0.378588;  //k2  
    distortion_coeff.at<double>(2, 0) = 0.028980;   //p1  
    distortion_coeff.at<double>(3, 0) = 0.008136;   //p2  
    distortion_coeff.at<double>(4, 0) = 0;          //p3  
}

void outputCameraParam(const Mat &intrinsic, const Mat &distortion_coeff)
{
    /*保存数据*/
    //cvSave("cameraMatrix.xml", &intrinsic);  
    //cvSave("cameraDistoration.xml", &distortion_coeff);  
    //cvSave("rotatoVector.xml", &rvecs);  
    //cvSave("translationVector.xml", &tvecs);  
    /*输出数据*/
    //cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
    //cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;//内参数
    printf("fx:%lf...\n", intrinsic.at<double>(0, 0));
    printf("fy:%lf...\n", intrinsic.at<double>(1, 1));
    printf("cx:%lf...\n", intrinsic.at<double>(0, 2));
    printf("cy:%lf...\n", intrinsic.at<double>(1, 2));                  //我学的是printf,就试着改了一下，都能用


    //cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
    //cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
    //cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
    //cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
    //cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;   //畸变参数
    printf("k1:%lf...\n", distortion_coeff.at<double>(0, 0));
    printf("k2:%lf...\n", distortion_coeff.at<double>(1, 0));
    printf("p1:%lf...\n", distortion_coeff.at<double>(2, 0));
    printf("p2:%lf...\n", distortion_coeff.at<double>(3, 0));
    printf("p3:%lf...\n", distortion_coeff.at<double>(4, 0));
}

bool mono_calib(const Mat &rgbImage, const string &name, Mat &intrinsic,
                    Mat &distortion_coeff, vector<Point2f> &corner){
    int imageHeight = rgbImage.rows;
    int imageWidth = rgbImage.cols;
    int goodFrameCount = 0;
    Mat gray;
    cvtColor(rgbImage, gray, COLOR_BGR2GRAY);
    bool isFind = findChessboardCorners(rgbImage, boardSize, corner, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
    //bool isFind = findChessboardCorners( rgbImage, boardSize, corner,  
    //CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);  
    
    if(isFind==true){
        //精确角点位置，亚像素角点检测
        cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
        
        //绘制角点
        drawChessboardCorners(rgbImage, boardSize, corner, isFind);
        imshow("chessboard", rgbImage);
        corners.push_back(corner);
        goodFrameCount++;
        /*cout << "The image" << goodFrameCount << " is good" << endl;*/
        printf("The image %d is good...\n", goodFrameCount);
    }else{
        printf("The image is bad please try again...\n");
    }
    
    /*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
    guessCameraParam(intrinsic, distortion_coeff);
    printf("guess successful...\n");
    /*计算实际的校正点的三维坐标*/
    calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
    printf("calculate real successful...\n");
    /*标定摄像头*/
    calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, 0);
    printf("calibration successful...\n");
    /*保存并输出参数*/
    outputCameraParam(intrinsic, distortion_coeff);
    printf("output successful...\n");

    /*显示畸变校正效果*/
    Mat cImage;
    undistort(rgbImage, cImage, intrinsic, distortion_coeff);  //矫正相机镜头变形
    imshow("Corret Image " + name, cImage);
    printf("Corret Image....\n");
    printf("Wait for Key....\n");

    waitKey(0);
    
    return isFind;
}

vector<string> getImgPath(bool is_left){
    string flag;
    if(is_left)  flag = "left";
    else flag = "right";
    
    vector<string> paths;
    string dir = "/home/yuan/XSpace/Ending/Camera/stereo/data/";
    for(int i=0; i<frameNumber; i++){
        stringstream ss;
        ss << dir << setw(3) << setfill('0') << i+2 << "_" << flag << ".png";
        paths.push_back(ss.str());
        cout << ss.str() << endl;
    }
    return paths;
}

int main()
{   
    vector<string> imgsL = getImgPath(true);
    vector<string> imgsR = getImgPath(false);
    
    int index = 0;
    bool flag = false;
    vector<Mat> intrinsics_L, intrinsics_R, distortion_coeffs_L, distortion_coeffs_R;
    namedWindow("Left", 1);
    namedWindow("Right", 1);
    
    while (index < frameNumber)
    {
        Mat LeftImage = imread(imgsL[index]);
        Mat RightImage = imread(imgsR[index]);
        imshow("Left", LeftImage);
        imshow("Right", RightImage);
        waitKey(1);
        
        Mat intrinsic_L, distortion_coeff_L, intrinsic_R, distortion_coeff_R;
        vector<Point2f> cornerL, cornerR;
        mono_calib(LeftImage, "Left", intrinsic_L, distortion_coeff_L, cornerL);
        mono_calib(RightImage, "Right", intrinsic_R, distortion_coeff_R, cornerR);
        
        cout << "intrinsic_L: \n" << intrinsic_L << endl;
        cout << "intrinsic_R: \n" << intrinsic_R << endl;
        cout << "distortion_coeff_L: \n" << distortion_coeff_L << endl;
        cout << "distortion_coeff_R: \n" << distortion_coeff_R << endl;
        
        if(intrinsic_L.data==nullptr ||
            intrinsic_R.data==nullptr ||
            distortion_coeff_L.data==nullptr ||
            distortion_coeff_R.data==nullptr){
            
            cout << "mono_calib is error. " << imgsL[index] << endl;
            continue;
        }
        
        // stereo_calib 
        Mat R, T, E, F;
        double rms = stereoCalibrate(objRealPoint,
            cornerL, cornerR,
            intrinsic_L, distortion_coeff_L, 
            intrinsic_R, distortion_coeff_R, 
            Size(LeftImage.cols, LeftImage.rows),
            R, T, E, F,
            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5)
        );
        rvecs.push_back(R);
        tvecs.push_back(T);
        
//         Mat Rl, Rr, Pl, Pr, Q;
//         stereoRectify(intrinsic_L, distortion_coeff_L,
//                       intrinsic_R, distortion_coeff_R,
//                       Size(LeftImage.cols, LeftImage.rows),
//                       Rl, Rr, Pl, Pr, Q, 
//                       CALIB_ZERO_DISPARITY, -1, imageSize, &validROIL, &validROIR);
        
        if (waitKey(30) == 27)//Esc键退出
        {
            break;
        }
        
        // stereo calib        
        
        index++;
    }
    return 0;
}

