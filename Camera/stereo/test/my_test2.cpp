#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/highgui.hpp>
using namespace std;
using namespace cv;

/**
 *   study memset
 */

int main()
{
    Mat img(3, 3, CV_32FC1, Scalar(0));
    int rows = img.rows;
    int cols = img.cols;
    float* data = (float*)img.data;
    
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            cout << data[i*cols+j] << ' ';
        }
        cout << endl;
    }
    
    memset(data, 10.0, sizeof(float)*cols*rows);
    data = (float*)img.data;
    
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            cout << data[i*cols+j] << ' ';
        }
        cout << endl;
    }
        
//     char str[10];
//     char *p = str;
//     memset(str, 0, sizeof(str));
//     for(int i=0; i<10; i++)
//         printf("%d\x20", str[i]);
//     printf("\n");
//     
//     memset(str, 1, sizeof(str));
//     for(int i=0; i<10; i++)
//         printf("%d\x20", str[i]);
//     printf("\n");
    
    
    return 0;
}
