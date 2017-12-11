#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

float x_min = -0.8, y_min = -0.5, z_min = +0.7;
float x_max = +0.5, y_max = +1.0, z_max = +2.5;

pcl::PointXYZ left_wrist_center(0.065,  0.138, 0.746);
pcl::PointXYZ right_wrist_center(-0.203, 0.101, 0.819);

void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
  std::cout << "[INOF] Point picking event occurred." << std::endl;
  if (event.getPointIndex () == -1)
  {
     return;
  }

  float x, y, z;
  event.getPoint(x, y, z);
  std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer)
{
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    std::cout << "[INFO] Key 's' was pressed " << std::endl;
    pcl::visualization::PCLVisualizer* v = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
    v->saveScreenshot("Image.png");
  }
  else if (event.getKeySym () == "q" && event.keyDown ())
  {
    std::cout << "[INFO] Key 'q' was pressed " << std::endl;
    pcl::visualization::PCLVisualizer* v = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
    v->close();
  }
}

int main (int argc, char** argv)
{
    std::cout << "###############################################################" << std::endl;
    std::cout << "[INFO] Hold down SHIFT key while left-clicking to pick a point." << std::endl;
    std::cout << "###############################################################" << std::endl;

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr body (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("../pcd/target.pcd", *body);
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_wrist (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../pcd/left.pcd", *left_wrist);

    pcl::PointCloud<pcl::PointXYZ>::Ptr right_wrist (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../pcd/right.pcd", *right_wrist);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_body (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::CropBox<pcl::PointXYZRGBA> box_filter;
    box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    box_filter.setInputCloud(body);
    box_filter.filter(*filtered_body);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> left_green (left_wrist, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> right_green (right_wrist, 0, 255, 0);

    viewer.addPointCloud(filtered_body,"body");
    viewer.addPointCloud(left_wrist, left_green, "left_wrist");
    viewer.addPointCloud(right_wrist, right_green, "right_wrist");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "left_wrist");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "right_wrist");
    //to set the min and max bound, below can be turned on
    //viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, 1, 0, 0, "cube");

    viewer.addSphere(left_wrist_center, 0.02, 255, 0, 0, "left_wrist_center");
    viewer.addSphere(right_wrist_center, 0.02, 255, 0, 0, "right_wrist_center");

    viewer.setBackgroundColor (0.5, 0.5, 0.5, 0);
    viewer.initCameraParameters ();
    //viewer.addCoordinateSystem (1.0);

    //0.026123,26.123/-0.00599269,0.00252074,1.00064/0.160405,0.457474,-1.54746/0.0138908,-0.984493,-0.17487/0.8575/1863,1176/57,24
    bool result = viewer.getCameraParameters(argc, argv);
    std::cout << "[INFO] Set camera parameter returns " << result << std::endl;

    viewer.registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer.registerPointPickingCallback (pointPickingEventOccurred, (void*)&viewer);

    viewer.spin();
    return 0;
}
