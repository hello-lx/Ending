#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

inline bool getPath(int index, string& left, string& right){
    stringstream ssL, ssR;
    ssL<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_left.png";
    ssR<<"/home/yuan/XSpace/Ending/Camera/stereo/dataset/"<<setw(5)<<setfill('0')<<index << "_right.png";
    left = ssL.str();
    right = ssR.str();
    ifstream fl(left.c_str()), fr(right.c_str());
    
    return fl.good() && fr.good();
}

int main(){
    int index = 0;
    
    string leftPath, rightPath;
    
    namedWindow("Left", 1);
    namedWindow("Right", 1);
    while(getPath(index++, leftPath, rightPath)){
        cout << leftPath << endl;
        
        Mat left = imread(leftPath);
        Mat right = imread(rightPath);
        
        cout << "width: " << left.cols << "  height: " << left.rows << endl;
        break;
        
//         imshow("Left", left);
//         imshow("Right", right);
//         
//         waitKey(10);
    }
    
    return 0;
}
