#include <pcl/features/normal_3d.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char** argv) {
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::io::loadPCDFile("../pcd/milk_cartoon_all_small_clorox.pcd", *cloud);

  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation
  // object. Its content will be filled inside the object, based on the given
  // input dataset (as no other search surface is given).
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch(0.03);

  // Compute the features
  ne.compute(*normals);

  std::cout << normals->size() << " same size as " << cloud->size()
            << std::endl;

  pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
  viewer.addPointCloudNormals<PointT, pcl::Normal>(cloud, normals);
  viewer.addPointCloud<PointT>(cloud, "cloud_only");

  viewer.addCoordinateSystem(0.2);
  viewer.spin();

  return 0;
}
