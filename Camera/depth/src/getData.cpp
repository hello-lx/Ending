#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "OpenNI.h"
 
using namespace std;
using namespace openni;
using namespace cv;

int debug(int i){
    cout<<"=========== "<<i<<" ==========="<<endl;
}

void showdevice(){
    // 获取设备信息  
    Array<DeviceInfo> aDeviceList;
    OpenNI::enumerateDevices(&aDeviceList);
    
    cout << "==================================================" << endl;
    cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << endl;
    cout << "==================================================" << endl;

    for (int i = 0; i < aDeviceList.getSize(); ++i)
    {
        cout << "设备 " << i << endl;
        const DeviceInfo& rDevInfo = aDeviceList[i];
        cout << "设备名： " << rDevInfo.getName() << endl;
        cout << "设备Id： " << rDevInfo.getUsbProductId() << endl;
        cout << "供应商名： " << rDevInfo.getVendor() << endl;
        cout << "供应商Id: " << rDevInfo.getUsbVendorId() << endl;
        cout << "设备URI: " << rDevInfo.getUri() << endl;

    }
}

void hMirrorTrans(const Mat &src, Mat &dst)
{
    dst.create(src.rows, src.cols, src.type());

    int rows = src.rows;
    int cols = src.cols;

    switch (src.channels())
    {
    case 1:   //1通道比如深度图像
        const uchar *origal;
        uchar *p;
        for (int i = 0; i < rows; i++){
            origal = src.ptr<uchar>(i);
            p = dst.ptr<uchar>(i);
            for (int j = 0; j < cols; j++){
                p[j] = origal[cols - 1 - j];
            }
        }
        break;
    case 3:   //3通道比如彩色图像
        const Vec3b *origal3;
        Vec3b *p3;
        for (int i = 0; i < rows; i++) {
            origal3 = src.ptr<Vec3b>(i);
            p3 = dst.ptr<Vec3b>(i);
            for (int j = 0; j < cols; j++){
                p3[j] = origal3[cols - 1 - j];
            }
        }
        break;
    default:
        break;
    }
 
}

int main()
{
    showdevice();
    Status rc = STATUS_OK;

    // 1. 初始化OpenNI环境
    if(OpenNI::initialize() != STATUS_OK) {
        printf("init failed:%s\n", OpenNI::getExtendedError());
        return 1;
    }
    
    // 2. open device
    Device device;
    if(rc != device.open(ANY_DEVICE)){
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;        
    }
    
    // 3. start streamColor  && streamDepth
    VideoStream streamColor, streamDepth;
    
    const SensorInfo* colorSensorInfo = device.getSensorInfo(openni::SENSOR_COLOR); 
    if(colorSensorInfo != NULL){
        if(rc != streamColor.create(device, SENSOR_COLOR)){
            printf("Couldn't create ir stream\n%s\n", OpenNI::getExtendedError());
            return 3;
        }
        // set mode
//         VideoMode mModeColor;
//         mModeColor.setResolution(320, 240);
//         mModeColor.setFps(30);
//         mModeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
//         streamColor.setVideoMode(mModeColor);
        
        if(rc != streamColor.start()){
            printf("Couldn't start the ir stream\n%s\n", OpenNI::getExtendedError());
            return 4;
        }
    }else{
        printf("There are not information for color sensor. \n");
        return 3;
    }
    
    if(streamDepth.create(device, SENSOR_DEPTH) == rc){
        // 设置深度图像视频模式
//         VideoMode mModeDepth;
//         // 分辨率大小
//         mModeDepth.setResolution(640, 480);
//         // 每秒30帧
//         mModeDepth.setFps(30);
//         // 像素格式
//         mModeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
// 
//         streamDepth.setVideoMode(mModeDepth);

        // 打开深度数据流
        if (streamDepth.start() != STATUS_OK)
        {
            cerr << "无法打开深度数据流：" << OpenNI::getExtendedError() << endl;
            streamDepth.destroy();
        }
    }
    else
    {
        cerr << "无法创建深度数据流：" << OpenNI::getExtendedError() << endl;
    }
    
    if(!streamColor.isValid() || !streamDepth.isValid()){
        printf("彩色或深度数据流不合法");
        return 4;
        OpenNI::shutdown();
    }
    
    if (device.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }

    // 4. read videoFrameRef
    namedWindow("depth", 1);
    namedWindow("color", 1);
    
    VideoFrameRef frameColor, frameDepth;
    int iMaxDepth = streamDepth.getMaxPixelValue();
    
    while(true){
        if (streamDepth.readFrame(&frameDepth) == rc)
        {
            // 将深度数据转换成OpenCV格式
            const Mat mImageDepth(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
            // 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
            Mat mScaledDepth, hScaledDepth;
            mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);

            //水平镜像深度图
            hMirrorTrans(mScaledDepth, hScaledDepth);
            // 显示出深度图像
            imshow("depth", hScaledDepth);
//             imshow("depth", mScaledDepth);
        }

        if (streamColor.readFrame(&frameColor) == STATUS_OK)
        {
            // 同样的将彩色图像数据转化成OpenCV格式
            const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
            // 首先将RGB格式转换为BGR格式
            Mat cImageBGR,bImageBGR,hImageBGR;
            cvtColor(mImageRGB, cImageBGR, COLOR_RGB2BGR);

            //水平镜像深度图
            hMirrorTrans(cImageBGR, hImageBGR);
            resize(hImageBGR, hImageBGR, Size(640, 480));
            // 然后显示彩色图像
            imshow("Color Image", hImageBGR);            
//             resize(cImageBGR, cImageBGR, Size(640, 480));
//             imshow("color", cImageBGR);            
        }

        // 终止快捷键
        if (waitKey(1) == 27)
            break;
    }
    
    streamColor.destroy();
    device.close();
    OpenNI::shutdown();
    
    return 0;
}
