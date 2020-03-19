#include <iomanip>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

/** 双目图像获取深度图
 *   (0. 相机标定获取内参)
 *   1. 获取内参
 *   2. 校正双目图像
 *   3. 计算视差图
 *   4. 视察图转深度图
     5. 视差图孔洞填充
*/

void debug(int i){
    cout<<"=========== "<<i<<" ==========="<<endl;
}

// ########## config path ########## 

string root = "/home/yuan/XSpace/Ending/Camera/stereo/";
string dir = root + "data/dataset/";
string intrinsicPath = root + "config/intrinsics.yml";
int start = 4;  // valid stereo image id for start

// ########## config path ########## 


// ########## Camera intrinsic ########## 

Size s1(320, 240);
Size s2(320*2, 240*2);

float bl = 40;  // baseline 单位: mm
Mat ML, DL;  // 内参:fx, fy, cx, cy / 畸变系数：k1 k2 p1 p2 p3
Mat MR, DR;  // 内参:fx, fy, cx, cy / 畸变系数：k1 k2 p1 p2 p3
Mat RL, TL;  // 左相机平面变换到共平面
Mat RR, TR;  // 右相机平面变换到共平面
Mat R, T;    // R&T：word -> camera
Mat F, E;    // E: essential matrix / F: functional matrix 
Mat PL, PR;  // P1/P2: projection 3D -> 2D
Mat Q;       // Q: reprojection (2D->3D) 4x4

// ########## Camera intrinsic ########## 


// 0. 获取内参 
bool getIntrinsics(string path){
    FileStorage fs(path, FileStorage::READ);
    if(fs.isOpened()){
        cout << "reading camera parameters ..." << endl;
        
        fs["M1"] >> ML; fs["M2"] >> MR;
        fs["D1"] >> DL; fs["D2"] >> DR;
        
        fs["R"] >> R; fs["T"] >> T;
        fs["R1"] >> RL; fs["P1"] >> PL;
        fs["R2"] >> RR; fs["P2"] >> PR;
        
        fs["Q"] >> Q;
        
//         cout << "left image: " << endl << ML << endl;
//         cout << "right image: " << endl << MR << endl;
        fs.release();
        return true;
    }else{
        cerr << "read failure for stereo camera configure file, please." << endl; 
        return false;
    }    
}


inline bool getPath(int index, string& left, string& right){
    stringstream ssL, ssR;
    
    ssL<<dir<<setw(5)<<setfill('0')<<index<< "_left.png";
    ssR<<dir<<setw(5)<<setfill('0')<<index<< "_right.png";
    left = ssL.str();
    right = ssR.str();
    ifstream fl(left.c_str()), fr(right.c_str());
    
    return fl.good() && fr.good();
}


// 1.校正图片
void image_retify(const Mat& imgL, const Mat& imgR,  Mat& resL, Mat& resR){
    Rect roiL, roiR;
    stereoRectify(ML, DL, ML, DR, s1, R, T, RL, RR, PL, PR, Q,
        CALIB_ZERO_DISPARITY, -1, s1, &roiL, &roiR);
    
    // 去畸变
    Mat rmapL[2], rmapR[2];
    initUndistortRectifyMap(ML, DL, RL, PL, s1, CV_32FC1, rmapL[0], rmapL[1]);
    initUndistortRectifyMap(MR, DR, RR, PR, s1, CV_32FC1, rmapR[0], rmapR[1]);
    
    // 校正图片
    remap(imgL, resL, rmapL[0], rmapL[1], INTER_LINEAR);
    remap(imgR, resR, rmapR[0], rmapR[1], INTER_LINEAR);
    
    // 图片裁减
    resize(resL(roiL), resL, s1);
    resize(resL(roiR), resR, s1);    
}


// 2.计算视差图 - bm
bool disp_bm(const Mat& imgL, const Mat& imgR, Mat& dispL, Mat& dispR){   // BUG
    Mat disp, disp8U, left, right;
    cvtColor(imgL, left, COLOR_BGR2GRAY);
    cvtColor(imgR, right, COLOR_BGR2GRAY);
 
	int mindisparity = 0;
	int ndisparities = 64;  
	int SADWindowSize = 11; 
 
	cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(ndisparities, SADWindowSize);
	// setter
	bm->setBlockSize(SADWindowSize);
	bm->setMinDisparity(mindisparity);
	bm->setNumDisparities(ndisparities);
	bm->setPreFilterSize(15);
	bm->setPreFilterCap(31);
	bm->setTextureThreshold(10);
	bm->setUniquenessRatio(10);
	bm->setDisp12MaxDiff(1);
 
	copyMakeBorder(left, left, 0, 0, 80, 0, IPL_BORDER_REPLICATE);  //防止黑边
	copyMakeBorder(right, right, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
	bm->compute(left, right, dispL);
	bm->compute(right, left, dispR);
    
	dispL.convertTo(disp, CV_32F, 1.0 / 16); //除以16得到真实视差值
	disp = disp.colRange(80, disp.cols);
	disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
	normalize(disp, dispL, 0, 255, NORM_MINMAX, CV_8UC1);
	dispR.convertTo(disp, CV_32F, 1.0 / 16); //除以16得到真实视差值
	disp = disp.colRange(80, disp.cols);
	disp8U = Mat(disp.rows, disp.cols, CV_8UC1);
	normalize(disp, dispR, 0, 255, NORM_MINMAX, CV_8UC1);
    
    return true;
}

// 2.计算视差图 - sgbm
bool disp_sgbm(const Mat& imgL, const Mat& imgR, Mat& dispL, Mat& dispR){
    // parameters
    int numberOfDisparities=48, SADWindowSize=11;
    int uniquenessRatio = 15, speckleWindowSize = 50, speckleRange = 32;
    int cn = imgL.channels();
    
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, numberOfDisparities,
                                                    SADWindowSize);
    
    SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setP1(8 * cn * SADWindowSize * SADWindowSize);
    sgbm->setP2(32 * cn * SADWindowSize * SADWindowSize);
    sgbm->setPreFilterCap(63);
    sgbm->setUniquenessRatio(uniquenessRatio);
    sgbm->setSpeckleRange(speckleRange);
    sgbm->setDisp12MaxDiff(1);
//     sgbm->setMode(cv::StereoSGBM::MODE_HH);// MODE_HH STEREO_SGBM MODE_SGBM_3WAY
    sgbm->compute(imgL, imgR, dispL);
    sgbm->compute(imgR, imgL, dispR);
    
    // normalize
    dispL.convertTo(dispL, CV_8U, 255/(numberOfDisparities*16.));
    dispR.convertTo(dispR, CV_8U, 255/(numberOfDisparities*16.));
    
    return true;
}

// 2.计算视差图 - gc
// bool disp_gc(const Mat& imgL, const Mat& imgR, Mat& dispL, Mat& dispR){
//     CvStereoGCState* state = cvCreateStereoGCState( 16, 2 );
//     left_disp_  =cvCreateMat( left->height,left->width, CV_32F ) 
//     right_disp_ =cvCreateMat( right->height,right->width,CV_32F );
//     cvFindStereoCorrespondenceGC( left, right, dispL, dispR, state, 0 );
//     cvReleaseStereoGCState( &state );
// }

// 2.计算视差图
bool computeDisp(const Mat& imgL, const Mat& imgR, const string& method,
        Mat& dispL, Mat& dispR){
/*
    视差效果：BM < SGBM < GC；
    处理速度：BM > SGBM > GC ；
*/
    bool ok = false;
    if(method == "SGBM")
        ok = disp_sgbm(imgL, imgR, dispL, dispR);
    else if(method == "BM")
        ok = disp_bm(imgL, imgR, dispL, dispR);
//     else if(method == "GC")
//         return disp_gc(imgL, imgR, dispL, dispR);
    else{
        cerr << method << " is not implement." << endl;
        return false;
    }
    if(ok){
        dispL = abs(dispL);
        dispR = abs(dispR);
    }
    return ok;
}


// 3. 视察图转深度图: depth = (f*baseline) / disp
bool disp2Depth(const Mat& disp, float fx, Mat& depth){
    depth = Mat(s1.height, s1.width, CV_16UC1, Scalar(0));
    
    int type = disp.type();
    if(type == CV_8U){
        uchar* dispData = (uchar*)disp.data;
        ushort* depthData = (ushort*)depth.data;
        
        for(int i=0; i<s1.height; i++)
            for(int j=0; j<s1.width; j++){
                int id = i*s1.width + j;
                if(!dispData[id]) continue;
                depthData[id] = ushort((float)fx*bl / (float)dispData[id]);
            }
        
        return true;
    }else{
        cerr << "please confirm image type..." << endl;
        return false;
    }
}


// 4.视差图孔洞填充
void depthFill(const Mat& depth){
    debug(1);
    
    const int width = depth.cols;
    const int height = depth.rows;
    
    cout << "width: " << width << ' ' << "height: " << height << endl;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    debug(2);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    debug(3);
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    debug(4);
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    debug(5);
    
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
    cout << "width: " << depth.cols << ' ' << "height: " << depth.rows << endl;
    debug(6);
}


/** 双目图像获取深度图
 *   (0. 相机标定获取内参)
 *   1. 获取相机内参
 *   2. 校正双目图像
 *   3. 计算视差图
 *   4. 视察图转深度图
 *   5. 深度图孔洞填充
 */
int main()
{
    // 1. get Intrinsics
    if(!getIntrinsics(intrinsicPath))
        return 1;
    
    string pathL, pathR;
    Mat imgL, imgR, rectifyL, rectifyR, dispL, dispR;
    Mat depthL, depthR, visDepthL, visDepthR;
    Mat visDepthLNew, visDepthRNew;
    
    namedWindow("left"), namedWindow("right");
    namedWindow("rectify_left"), namedWindow("rectify_right");
    namedWindow("disp_left"), namedWindow("disp_right");
    namedWindow("depth_left"), namedWindow("depth_right");
//     namedWindow("depth_left_new"); namedWindow("depth_right_new");
    moveWindow("left", 0, 0); // resizeWindow("left", s1.width, s1.height);
    moveWindow("right", 0, s1.height+s1.height/3);
    moveWindow("rectify_left", s1.width+s1.width/4, 0);
    moveWindow("rectify_right", s1.width+s1.width/4, s1.height+s1.height/2);
    moveWindow("disp_left", s1.width*2+s1.width/3, 0);
    moveWindow("disp_right", s1.width*2+s1.width/3, s1.height+s1.height/2);
    moveWindow("depth_left", s1.width*3+s1.width/2, 0);
    moveWindow("depth_right", s1.width*3+s1.width/2, s1.height+s1.height/2);
    moveWindow("depth_left_new", s1.width+s1.width/4, 0);
//     moveWindow("depth_right_new", s1.width+s1.width/4, s1.height+s1.height/2);
    
    while(getPath(start, pathL, pathR)){
        cout << pathL << endl;
        
        imgL = imread(pathL);  // resize(imgL, imgL, s2);
        imgR = imread(pathR);
        imshow("left", imgL), imshow("right", imgR);
        
        // 2. rectify image  (problem： 右相机内参出问题)
        image_retify(imgL, imgR, rectifyL, rectifyR);
        imshow("rectify_left", rectifyL), imshow("rectify_right", rectifyR);
        
        // 3. 计算视差图
        string method = "SGBM";  // BM SGBM GC
        if(!computeDisp(rectifyL, rectifyR, method, dispL, dispR)) return 3;
        imshow("disp_left", dispL), imshow("disp_right", dispR);
        
        // 4.视差图转换成深度图
        float fxL=4.2375831664303877e+02, fxR=4.0573696859338202e+02;
        if(!disp2Depth(dispL,fxL,depthL)||!disp2Depth(dispR,fxR,depthR))
            return 4;
        
        // 5.视差图孔洞填充
        
        
        normalize(depthL, visDepthL, 0, 256, NORM_MINMAX);
        normalize(depthR, visDepthR, 0, 256, NORM_MINMAX);
        visDepthL.convertTo(visDepthL, CV_8U, 1);
        visDepthR.convertTo(visDepthR, CV_8U, 1);
        imshow("depth_left", visDepthL);
        imshow("depth_right", visDepthR);
        
//         visDepthL.copyTo(visDepthLNew);
//         visDepthR.copyTo(visDepthRNew);
//         debug(11111111);
//         depthFill(visDepthLNew);
//         debug(22222222);
//         depthFill(visDepthRNew);
//         debug(33333333);
//         imshow("depth_left_new", visDepthLNew);
//         debug(44444444);
//         imshow("depth_right_new", visDepthRNew);
//         debug(55555555);
        
        if(waitKey(3000) == 27){
            destroyAllWindows();
            break;
        }
//         debug(14);
        
        start++;
    }
    
    return 0;
}

