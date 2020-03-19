#if 1
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#define w 9;  //棋盘格宽的黑白交叉点个数   
#define h 6;  //棋盘格高的黑白交叉点个数

const float chessboardSquareSize = 20.0f;

using namespace std;
using namespace cv;


//从 xml 文件中读取图片存储路径 
static bool readStringList(const string& filename, vector<string>& list){
    list.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) return false;
    
    FileNode n = fs.getFirstTopLevelNode();
    if(n.type() != FileNode::SEQ) return false;
    
    FileNodeIterator it = n.begin(), it_end = n.end();
    for(; it!=it_end; ++it)
        list.push_back(string(*it));
    return true;
}


//记录棋盘格角点个数
static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners){
    corners.resize(0);
    for(int i=0; i<boardSize.height; i++)  //height和width位置不能颠倒
        for(int j=0; j<boardSize.width; j++){
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
        }
}


bool calibrate(Mat& intrMat, Mat& distCoeffs, vector<vector<Point2f>>& imagePoints, 
    vector<vector<Point3f>>& objectPoints, Size& imageSize, const int cameraId, 
    vector<string> imageList
){
    double rms = 0; //重投影误差
    
    Size boardSize;
    boardSize.height = h;
    boardSize.width = w;
    
    vector<Point2f> pointBuf;
    float squareSize = chessboardSquareSize;
    
    vector<Point2f> pointBuf;
    float squareSize = chessboardSquareSize;
    
    vector<Mat> rvecs, tvecs;
    
    bool ok = false;
    
    int nImages = (int)imageList.size() // 2;
    cout << "image nums: " << nImages << endl; 
    
    int nums_good = 0;   // 有效棋盘格图片张数
    
    for(int i=0; i<nImages; i++){
        Mat view, viewGray;
        view = imread(imageList[i*2+cameraId], 1);
        imageSize = view.size();
        cvtColor(view, viewGray, COLOR_BGR2GRAY);
        
        bool found = findChessboardCorners(view, boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        
        if(found){
            nums_good++;
            cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1), 
                TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.1));
            drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
            bitwise_not(view, view);
            imagePoints.push_back(pointBuf, objectPoints, );
            cout  << ". ";
        }
        
        imshow("view", view);
        waitKey(50);
    }
    
    cout << "good images: " << nums_good << endl;
    
    calcChessboardCorners(boardSize, squareSize, objectPoints[0]);
    objectPoints.resize(imagePoints.size(), objectPoints[0]);
    
    
    rms = calibrateCamera(objectPoints, imagePoints, imageSize, intrMat, distCoeffs, rvecs, tvecs);
    
    
    
}


int main(){
    
    
    
}
