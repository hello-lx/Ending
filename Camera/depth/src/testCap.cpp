#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main(int argc, char** argv){
    
    VideoCapture cap(4);  // cap.open()
    if(!cap.isOpened()) return -1;
    
    Mat frame;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    cout << "FPS: " << cap.get(CAP_PROP_FPS) << endl;
    
    while(true){
        cap.read(frame); // == cap >> frame
        imshow("frame", frame);        
        if(waitKey(0) == 27) break;
    }
    
    cap.release();
    return 0;
}
