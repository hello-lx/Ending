#include <iostream> 
#include <string>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    
    std::string ply_path = "/home/yuan/XSpace/Ending/DenseSurfelMapping/surfel_fusion/results/kitti/kitti00_loop_mesh.PLY";
    
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path, *cloud) == -1) //* load the file 
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        system("PAUSE");
        return (-1);
    }
    //pcl::StatisticalOutlierRemoval::applyFileter()
    pcl::visualization::CloudViewer viewer("Viewer");
    viewer.showCloud(cloud);
 
    system("PAUSE");
    return (0);
}
