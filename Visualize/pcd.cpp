#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl;
using namespace std;

int main(int argc, char** argv){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string pcd_path;    
    if(argc < 2){
        pcd_path = "/home/yuan/XSpace/Ending/DenseSurfelMapping/surfel_fusion/results/kitti/kitti00_loop.PCD";
    }else{
        pcd_path = argv[1];
    }
    
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1){
        PCL_ERROR("Could't read file.");
        return -1;
    }
    
//     cout << "size: " << cloud.size() << endl;
    pcl::visualization::CloudViewer viewer("Cloud viewer");
    viewer.showCloud(cloud);
    
    while(!viewer.wasStopped()){}
    
    system("pause");
    
    return 0;
}
