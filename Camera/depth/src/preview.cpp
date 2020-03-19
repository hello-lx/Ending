#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include "OpenNI.h"
#include <sys/time.h>

using namespace std;
using namespace cv;
using namespace openni;
static VideoCapture capture;
static Device device;
static VideoStream depth;
static VideoStream color;
static VideoStream ir;
static VideoFrameRef depthFrame;
static VideoFrameRef colorFrame;
static VideoFrameRef irFrame;

const static int MAX_DEPTH_VALUE = 0xffff;
static float* pDepthHist = NULL;
const static int OPENNI_READ_WAIT_TIMEOUT = 500;

static int mirrorColor = 0;
Mat depthImg(480, 640, CV_8UC3);
Mat depthRaw(480, 640, CV_16UC1, Scalar(0));
Mat irImg(480, 640, CV_8UC3, Scalar(0));
Mat colorImg(480, 640, CV_8UC3);

static bool quit = false;
static int openAllStream = 0;
static bool isUvcCamera = true;

int waitForColorFrame()
{
    capture >> colorImg;
    if (colorImg.empty()){
        printf("Capture UVC color failed.\n");
        return -1;
    }
    return 0;
}

int waitForOpenNIFrame()
{
	int streamIndex;
	VideoFrameRef frame;

	VideoStream* pStream[] = {&depth, &color, &ir};
	Status rc = OpenNI::waitForAnyStream(pStream, 3, &streamIndex, OPENNI_READ_WAIT_TIMEOUT);
	if (rc != STATUS_OK)
	{
		printf("Wait failed! (timeout is %d ms)\n%s\n", OPENNI_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
		return 1;
	}

	switch(streamIndex) {
		case 0:
			rc = depth.readFrame(&depthFrame);
			break;
		case 1:
			rc = color.readFrame(&colorFrame);
			break;
		case 2:
			rc = ir.readFrame(&irFrame);
			break;
		default:
			printf("Wait frame error. Stream index:%d\n", streamIndex);
			return 1;
	}

	if (rc != STATUS_OK)
	{
		printf("Read failed!\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	if(depthFrame.isValid()) {
		if (depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && depthFrame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			printf("Unexpected depthFrame format\n");
			return 3;
		}

		DepthPixel* pDepth = (DepthPixel*)depthFrame.getData();
		depthRaw = Mat(depthFrame.getVideoMode().getResolutionY(), depthFrame.getVideoMode().getResolutionX(), CV_16UC1, (unsigned char*)pDepth);


		if (pDepthHist == NULL) {
			pDepthHist = new float[MAX_DEPTH_VALUE];
		}
		memset(pDepthHist, 0, MAX_DEPTH_VALUE * sizeof(float));


		int numberOfPoints = 0;
		openni::DepthPixel nValue;

		int totalPixels = depthFrame.getVideoMode().getResolutionY() * depthFrame.getVideoMode().getResolutionX();

		for (int i = 0; i < totalPixels; i ++) {
			nValue = pDepth[i];
			if (nValue != 0) {
				pDepthHist[nValue] ++;
				numberOfPoints ++;
			}
		}

		for (int i = 1; i < MAX_DEPTH_VALUE; i ++) {
			pDepthHist[i] += pDepthHist[i - 1];
		}

		for (int i = 1; i < MAX_DEPTH_VALUE; i ++) {
			if (pDepthHist[i] != 0) {
				pDepthHist[i] = (numberOfPoints - pDepthHist[i]) / (float)numberOfPoints;
			}
		}

		int height = depthFrame.getVideoMode().getResolutionY();
		int width = depthFrame.getVideoMode().getResolutionX();
		for (int row = 0; row < height; row++) {
			DepthPixel* depthCell = pDepth + row * width;
			uchar * showcell = (uchar *)depthImg.ptr<uchar>(row);
			for (int col = 0; col < width; col++)
			{
				char depthValue = pDepthHist[*depthCell] * 255;
				*showcell++ = 0;
				*showcell++ = depthValue;
				*showcell++ = depthValue;
				depthCell++;
			}
		}
	}

	if(irFrame.isValid()) {
		int height = irFrame.getVideoMode().getResolutionY();
		int width = irFrame.getVideoMode().getResolutionX();
		const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)irFrame.getData();
		memcpy(irImg.data, pImageRow, height * width * 3);
	}

	if(colorFrame.isValid()) {
		int height = colorFrame.getVideoMode().getResolutionY();
		int width = colorFrame.getVideoMode().getResolutionX();
		const openni::RGB888Pixel* pImageRow = (const openni::RGB888Pixel*)colorFrame.getData();
		memcpy(colorImg.data, pImageRow, height * width * 3);
		cv::cvtColor(colorImg, colorImg, COLOR_RGB2BGR);
	}

	return 0;
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}

void captureImage(int flag, Mat& img, int num)
{
// 	std::vector<int> png_params;
// 	png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
// 	png_params.push_back(0);    // 无损
	char csName[512];
	char fileName[64] = {0};
	if (flag == 0) {
		strcpy(fileName, "Depth");
	}
	else if (flag == 1){
		strcpy(fileName, "Color");
	}

	else {
		strcpy(fileName, "IR");
	}

	sprintf(csName, "./Capture/%s_%d.png", fileName, num);
	cv::imwrite(csName, img);  //
}

int waitForFrame(Mat& previewImg)
{
	if(isUvcCamera) {
		waitForColorFrame();
	}
	waitForOpenNIFrame();

	static long long lastTimeStamp = 0;
	long long currentTimeStamp = current_timestamp();
	long timepast = currentTimeStamp - lastTimeStamp;
	static int num = 0;
	if (mirrorColor != 0) {
		cv::flip(colorImg, colorImg, 1);
	}

	if (timepast > 1000.0) {
		captureImage(0, depthRaw, num);
		captureImage(1, colorImg, num);
		if(irFrame.isValid()) {
			captureImage(2, irImg, num);
		}
		lastTimeStamp = currentTimeStamp;
		num ++;
	}

	previewImg = colorImg.clone();
	hconcat(previewImg, depthImg, previewImg);

	if(irFrame.isValid()) {
		hconcat(previewImg, irImg, previewImg);
	}
	return 0;
}

int keyboardEvent(int key)
{
	if(key == 27) {
		quit = true;
	}
	return 0;
}

int main(int argc, char*argv[])
{

	int camNum = 1;

	if(argc >= 2) {
		sscanf(argv[1], "%d", &openAllStream);
	}

	if(argc >= 3) {
		sscanf(argv[2], "%d", &camNum);
	}

	if(argc >= 4) {
		sscanf(argv[3], "%d", &mirrorColor); 
	}

	Status rc;
	rc = openni::OpenNI::initialize();
	if(rc != STATUS_OK) {
		printf("init failed:%s\n", OpenNI::getExtendedError());
		return 1;
	}

	rc = device.open(ANY_DEVICE);
	if(rc != STATUS_OK) {
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}
	}
  
	rc = depth.start();
	if (rc != STATUS_OK)
	{
		printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 4;
	}

	bool hasOpenNIColor = device.hasSensor(openni::SENSOR_COLOR);
    if(hasOpenNIColor) cout << "hasOpenNIColor = true" <<endl;
    else cout << "hasOpenNIColor = false" << endl;
//     exit(1);
    
	const SensorInfo* colorSensorInfo = device.getSensorInfo(openni::SENSOR_COLOR);
	if (colorSensorInfo != NULL){
		isUvcCamera = false;
		rc = color.create(device, SENSOR_COLOR);
		if (rc != STATUS_OK)
		{
			printf("Couldn't create ir stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}

		rc = color.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the ir stream\n%s\n", OpenNI::getExtendedError());
			return 4;
		}
	}
    
    if(isUvcCamera == false)
        cout << "==> isUvcCamera: false " << endl;
    else
        cout << "==> isUvcCamera: true " << endl;

    if(openAllStream == false)
        cout << "==> openAllStream: false " << endl;
    else
        cout << "==> openAllStream: true " << endl;
    
    if(isUvcCamera && openAllStream)
        cout << "isUvcCamera && openAllStream: true" << endl;
    else
        cout << "isUvcCamera && openAllStream: false" << endl;

//     exit(1);
    
	if (isUvcCamera && openAllStream) {

		if (device.getSensorInfo(SENSOR_IR) != NULL)
		{
			rc = ir.create(device, SENSOR_IR);
			if (rc != STATUS_OK)
			{
				printf("Couldn't create ir stream\n%s\n", OpenNI::getExtendedError());
				return 3;
			}
		}

		rc = ir.start();
		if (rc != STATUS_OK)
		{
			printf("Couldn't start the ir stream\n%s\n", OpenNI::getExtendedError());
			return 4;
		}

	}

	if (isUvcCamera) {
		capture.set(6, CAP_OPENCV_MJPEG);   // CV_FOURCC('M', 'J', 'P', 'G') -> CAP_OPENCV_MJPEG  
		
		if (!capture.open(1))
		{
			capture.open(0);
		}

		if (!capture.isOpened())
		{
			return -1;
		}
	}
  

	const char* title = "UVC Color";
	namedWindow(title, 1);

	char fpsStr[64] = "30.0";
	while (!quit)
	{
		Mat previewImg(480, 640, CV_8UC3);
		waitForFrame(previewImg);
// 		IplImage image = previewImg;
		imshow(title, previewImg);
		int key = waitKey(10);
		if(key >= 0) {
			keyboardEvent(key);
		}
	}

	if (isUvcCamera) {
		capture.release();
	}
	device.close();
	openni::OpenNI::shutdown();

	return 0;
}
