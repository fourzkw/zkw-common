#include <iostream>
#include <boost/thread/thread.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

void visualization_PCLVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    /* 创建窗口对象，并设置名称为“3D Viewer” 。boost::shared_ptr为智能共享指针，这样可以保证指针在程序中全局使用，而不引起内存错误*/
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));  

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 255, 0, 0);    // 设置点云颜色
    viewer->setBackgroundColor (255, 255, 255); //设置视窗的背景色，可以任意设置RGB的颜色，这里是设置为白色
    viewer->addPointCloud<pcl::PointXYZ> (cloud, color, "cloud"); // 核心代码，添加点云和颜色，并且定一个字符串作为ID号，多次调用addPointCloud可实现多个添加。
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");    // 设置点的大小，这里注意ID号与添加的点云一致
    viewer->addCoordinateSystem (3);  // 显示一个坐标轴，以防没有方向感。X（红色）Y（绿色 ）Z （蓝色），30代表轴的大小
    viewer->initCameraParameters(); //通过设置照相机参数使得从默认的角度和方向观察点云
    
    while (!viewer->wasStopped()) // 直到窗口关闭才结束循环
    { 
		viewer->spinOnce(100000);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}  
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("../resource/thing_three.pcd", *cloud);   // 读取pcd文件

    std::cout << "原始点云数: " <<cloud->points.size() <<std::endl;

    visualization_PCLVisualizer(cloud);


    return 0;
}
