#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::console;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudXYZRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

void pointPickingEventOccurred(
    const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    std::cout << "[INOF] Point picking event occurred." << std::endl;

    int index = event.getPointIndex();
    if (index == -1)
    {
        return;
    }

    pcl::PointXYZRGB p = pointCloudXYZRGB->points[index];
    std::cout << "x: " << p.x << ", y: " << p.y << ", z: " << p.z << ", r: " << (int)p.r
              << ", g: " << (int)p.g << ", b: " << (int)p.b << std::endl;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer)
{
    if (event.getKeySym() == "s" && event.keyDown())
    {
        std::cout << "[INFO] Key 's' was pressed " << std::endl;
        pcl::visualization::PCLVisualizer* v
            = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
        v->saveScreenshot("Image.png");
    }
    else if (event.getKeySym() == "q" && event.keyDown())
    {
        std::cout << "[INFO] Key 'q' was pressed " << std::endl;
        pcl::visualization::PCLVisualizer* v
            = static_cast<pcl::visualization::PCLVisualizer*>(viewer);
        v->close();
    }
}

int main(int argc, char** argv)
{
    std::cout << "###############################################################" << std::endl;
    std::cout << "[INFO] Hold down SHIFT key while left-clicking to pick a point." << std::endl;
    std::cout << "###############################################################" << std::endl;

    std::vector<int> pcd_file_indices
        = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (pcd_file_indices.size() != 1)
    {
        std::cerr << "[ERROR] Need one input PCD file" << '\n';
        return (-1);
    }

    std::string pcd_filename = argv[pcd_file_indices[0]];
    std::cout << "[INFO] Input file name " << pcd_filename << '\n';

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_filename, *pointCloudXYZRGB) == -1)
    {
        std::cerr << "[ERROR] Couldn't read file " << pcd_filename << '\n';
        return (-1);
    }

    viewer.addPointCloud(pointCloudXYZRGB, "pointCloudXYZRGB");
    viewer.initCameraParameters();
    bool result = viewer.getCameraParameters(argc, argv);

    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer.registerPointPickingCallback(pointPickingEventOccurred, (void*)&viewer);

    viewer.spin();
    return 0;
}
