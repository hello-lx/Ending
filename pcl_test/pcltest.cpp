#define  _SCL_SECURE_NO_WARNINGS

// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/cloud_viewer.h>
#include<iostream>//��׼C++���е�������������ͷ�ļ���
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
int user_data;
using std::cout;


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0, 0.5, 1.0);   //���ñ�����ɫ
}

int main() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	char strfilepath[256] = "../rabbit.pcd";
	if (-1 == pcl::io::loadPCDFile(strfilepath, *cloud)) {
		cout << "error input!" << endl;
		return -1;
	}

	cout << cloud->points.size() << endl;
	pcl::visualization::CloudViewer viewer("Cloud Viewer");     //����viewer����

    viewer.showCloud(cloud);
    viewer.runOnVisualizationThreadOnce(viewerOneOff);
//     system("pause");
    
    while(1){}
    
	return 0;
}