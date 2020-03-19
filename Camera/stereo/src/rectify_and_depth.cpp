#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

Mat Q;
Mat xyz;
Ptr<StereoBM> bm = StereoBM::create(16, 9);
int blockSize=0, uniquenessRatio=0, numDisparities=0;

int width = 320, height = 240;
Size image_size(width, height);
Rect validROIL, validROIR; //图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域    
Mat rectifyL, rectifyR;
Mat disp8;

void debug(int i){
    cout << "========= " << i << " =========" << endl;
}

inline bool getPath(int index, string& left, string& right){
    stringstream ssL, ssR;
    ssL<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_left.png";
    ssR<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_right.png";
    left = ssL.str();
    right = ssR.str();
    ifstream fl(left.c_str()), fr(right.c_str());
    
    return fl.good() && fr.good();
}

inline bool getParam(Mat &M1, Mat &M2, Mat &D1, Mat &D2,
    Mat &R, Mat &T, Mat &R1, Mat &R2, Mat &P1, Mat &P2, Mat &Q)
{
    FileStorage fs("../config/intrinsics.yml", FileStorage::READ);
    if(fs.isOpened()){
        cout << "reading ..." << endl;
        fs["M1"] >> M1; fs["M2"] >> M2;
        fs["D1"] >> D1; fs["D2"] >> D2;
        
        fs["R"] >> R; fs["T"] >> T;
        fs["R1"] >> R1; fs["P1"] >> P1;
        fs["R2"] >> R2; fs["P2"] >> P2;
        
        fs["Q"] >> Q;
        
        fs.release();
        return true;
    }else{
        cerr << "read failure for stereo camera configure file, please." << endl; 
        return false;
    }
}

void stereo_match_sgbm(const Mat &rectifyImageL, const Mat &rectifyImageR){
    bm->setBlockSize(2 * blockSize + 5);     //SAD窗口大小，5~21之间为宜
    bm->setROI1(validROIL);
    bm->setROI2(validROIR);
    bm->setPreFilterCap(31);
    bm->setMinDisparity(0);  //最小视差，默认值为0, 可以是负值，int型
    bm->setNumDisparities(numDisparities * 16 + 16);//视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(uniquenessRatio);//uniquenessRatio主要可以防止误匹配
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(-1);
    Mat disp;
    bm->compute(rectifyImageL, rectifyImageR, disp);//输入图像必须为灰度图
    reprojectImageTo3D(disp, xyz, Q, true); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16;
    imshow("disparity", disp8);
}

void saveResult(int index){
    stringstream ssL, ssR, ssD;
    string data_dir = "/home/yuan/XSpace/Ending/Camera/stereo/dataset_retified/";
    ssL<<data_dir<<setw(5)<<setfill('0')<<index<<"_rectifyL.png";
    ssR<<data_dir<<setw(5)<<setfill('0')<<index<<"_rectifyR.png";
    ssD<<data_dir<<setw(5)<<setfill('0')<<index<<"_disp.png";
    
//     rectifyL = rectifyL(validROIL);
//     rectifyR = rectifyR(validROIR);
//     xyz = xyz(validROIR);
//     resize(rectifyL, rectifyL, image_size);
//     resize(rectifyR, rectifyR, image_size);
//     resize(xyz, xyz, image_size);
//     Mat depth(width, height, CV_8UC1, Scalar(0));
//     for(int i=0; i<height; i++){
//         for(int j=0; j<width; j++){
//             depth[] = xyz.data[i*xyz.step + j*xyz.channels() + 2];
//         }
//     }
    
    imwrite(ssL.str(), rectifyL);
    imwrite(ssR.str(), rectifyR);
//     imwrite(ssD.str(), xyz);
    imwrite(ssD.str(), disp8);
}

int main(int argc, char** argv){
    // camera intrinsic: (3d -> 2d)
    Mat M1, M2, D1, D2;
    // R/T: word -> camera   R1/R2/P1/P2: projection   E:essential matrix
    // F: functional matrix   Q: back projection (2d->3d)
    Mat R, T, R1, R2, P1, P2, E, F;
    
    Rect validRoi[2];
    Mat viewLeft, viewRight;
    
    int index = 4;
    string leftPath, rightPath;
    
    if(!getParam(M1, M2, D1, D2, R, T, R1, R2, P1, P2, Q))  return 1;
    
    cout << "start rectify ..." << endl;
    Mat viewleft_retified, right_retified, disparity;
    stereoRectify(M1, D1, M2, D2, image_size, R, T, R1,
 R2, P1, P2, Q,
        CALIB_ZERO_DISPARITY, -1, image_size, &validROIL, &validROIR);

    
    cout << validROIR.tl() << " " << validROIR.br() << endl;
    cout << validROIR.width << " " << validROIR.height << endl;
    
    namedWindow("left", 1); namedWindow("right", 1);
    namedWindow("remapL", 1); namedWindow("remapR", 1); namedWindow("disparity", 1);
    while(getPath(index, leftPath, rightPath)){
        cout << "rectify: " << leftPath << endl;
        
        Mat viewL = imread(leftPath);
        Mat viewR = imread(rightPath);
        imshow("left", viewL);
        imshow("right", viewR);
        
        // 1. rectify
        stereoRectify(M1, D1, M2, D2, image_size, R, T, R1, R2, P1, P2, Q,
            CALIB_ZERO_DISPARITY, -1, image_size, &validROIL, &validROIR);
        
        Mat rmapL[2], rmapR[2], rviewL, rviewR;
        initUndistortRectifyMap(M1, D1, R1, P1, image_size, CV_16SC2, rmapL[0], rmapL[1]);
        initUndistortRectifyMap(M2, D2, R2, P2, image_size, CV_16SC2, rmapR[0], rmapR[1]);
        
        remap(viewL, rectifyL, rmapL[0], rmapL[1], INTER_LINEAR);
        remap(viewR, rectifyR, rmapR[0], rmapR[1], INTER_LINEAR);
        
        cvtColor(rectifyL, rectifyL, COLOR_BGR2GRAY);
        cvtColor(rectifyR, rectifyR, COLOR_BGR2GRAY);
        
        imshow("remapL", rectifyL);
        imshow("remapR", rectifyR);
        
        // 2. compute disparity
        stereo_match_sgbm(rectifyL, rectifyR);
        if(waitKey(10)==27) break;
        
        saveResult(index);
        
        index++;
    }
}
