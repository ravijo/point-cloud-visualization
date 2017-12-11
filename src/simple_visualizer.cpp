#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main (int argc, char** argv)
{
    // initialize PCL visualizer class
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    // our point cloud file contains color information too, hence use  PointXYZRGBA
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcd_file (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // load the file and provide the pointer to file object
    pcl::io::loadPCDFile ("../pcd/target.pcd", *pcd_file);

    // add the pcd file to visualizer
    viewer.addPointCloud(pcd_file, "my_pcd_file");

    // open the visualizer
    viewer.spin();
    return 0;
}
