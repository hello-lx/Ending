#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>
#include <iomanip>

#include <cstring>  // strcpy

#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

string dir = "/home/yuan/XSpace/Ending/Camera/stereo/data/dataset_retified/";
int id = 4;

void debug(int i){
    cout<<"=========== "<<i<<" ==========="<<endl;
}

inline bool getPath(int index, string& path){
    stringstream ss;
    
    ss<<dir<<setw(5)<<setfill('0')<<index<<"_depth.png";
    ifstream f(ss.str());
    path = ss.str();
    return f.good();
}


Mat depthFill(const Mat& depth0){
// void depthFill(Mat& depth){
    debug(1);
    
    Mat depth = depth0.clone();
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
//     cout << "integral: " << (float)integral.cols << ' ' << (float)integral.rows << endl;
//     cout << "ptsIntegral: " << (float)ptsIntegral.cols << ' ' << (float)ptsIntegral.rows << endl;
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
            cout << "x: " << j << "  y:" << i << endl;
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
        cout << "test0..." << endl;
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
//                 cout << "x: " << j << " y:" << i << endl;
                
                if (ptsCnt <= 0)
                {
                    continue;
                }
                if(j>=320 || i>=240) continue;
                float temp = float(sumGray / ptsCnt);
                data[id2] = temp;
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cout << "test1..." << s << endl;
        cout << depth.type() << endl;
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
        cout << "test2..." << endl;
    }
    cout << "width: " << depth.cols << ' ' << "height: " << depth.rows << endl;
    debug(6);
    
    return depth;
}


void test2(const Mat& depth){
    for(int i=0; i<3; i++)
        cv::GaussianBlur(depth, depth, cv::Size(3, 3), 3, 3);
}

int main(int argc, char** argv)
{
    string path = "";
    Mat depth;
    while(getPath(id++, path)){
        cout << path << endl;
        depth = imread(path);
        cvtColor(depth, depth, CV_BGR2GRAY);
        imshow("depth", depth);
        
        test2(depth);
        
//         cout << "depth type: "<< depth.type() << endl;
        
        imshow("depth_fill", depth);
        
        if(waitKey(5000) == 27){
            break;
        }
    }
    return 0;
}
