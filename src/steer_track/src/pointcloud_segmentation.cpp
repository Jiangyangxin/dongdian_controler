#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <iostream>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>


//111111111111111111111111111111111111
// typedef pcl::PointXYZ PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// ros::Publisher pub;

// // 计算点的角度（相对于x轴）
// double calculateAngle(const PointT& point)
// {
//     return std::atan2(point.y, point.x);
// }

// void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
// {
//     // 将LaserScan转换为PointCloud，并过滤无效点
//     PointCloudT::Ptr cloud(new PointCloudT);
//     for (size_t i = 0; i < scan->ranges.size(); ++i)
//     {
//         float range = scan->ranges[i];
//         // 过滤无效点（NaN或Inf）
//         if (!std::isfinite(range))
//         {
//             continue;
//         }

//         float angle = scan->angle_min + i * scan->angle_increment;
//         PointT point;
//         point.x = range * cos(angle);
//         point.y = range * sin(angle);
//         point.z = 0.0;
//         cloud->points.push_back(point);
//     }

//     // 检查点云是否为空
//     if (cloud->points.empty())
//     {
//         ROS_WARN("No valid points in the point cloud!");
//         return;
//     }

//     // 半径滤波，去除噪声点
//     pcl::RadiusOutlierRemoval<PointT> radius_filter;
//     radius_filter.setInputCloud(cloud);
//     radius_filter.setRadiusSearch(0.5); // 搜索半径
//     radius_filter.setMinNeighborsInRadius(10); // 最小邻居数
//     radius_filter.filter(*cloud);

//     // 创建KdTree对象
//     pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
//     tree->setInputCloud(cloud);

//     // 欧几里得聚类
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<PointT> ec;
//     ec.setClusterTolerance(0.05); // 聚类容忍度（单位：米）
//     ec.setMinClusterSize(100);     // 最小聚类点数
//     ec.setMaxClusterSize(25000);  // 最大聚类点数
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_indices);

//     // 找到最大的半圆形物体
//     double max_semicircle_radius = 0;
//     PointCloudT::Ptr largest_semicircle(new PointCloudT);
//     pcl::ModelCoefficients::Ptr largest_coefficients(new pcl::ModelCoefficients);

//     for (const auto& indices : cluster_indices)
//     {
//         PointCloudT::Ptr cluster(new PointCloudT);
//         for (const auto& idx : indices.indices)
//         {
//             cluster->points.push_back(cloud->points[idx]);
//         }

//         // 使用RANSAC进行半圆形检测
//         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//         pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//         pcl::SACSegmentation<PointT> seg;
//         seg.setOptimizeCoefficients(true);
//         seg.setModelType(pcl::SACMODEL_CIRCLE2D);
//         seg.setMethodType(pcl::SAC_RANSAC);
//         seg.setDistanceThreshold(0.1); // 距离阈值（单位：米）
//         seg.setMaxIterations(1000);     // 最大迭代次数
//         seg.setInputCloud(cluster);
//         seg.segment(*inliers, *coefficients);

//         if (inliers->indices.size() > 0)
//         {
//             double radius = coefficients->values[2];
//             if (radius > max_semicircle_radius)
//             {
//                 max_semicircle_radius = radius;
//                 largest_semicircle = cluster;
//                 largest_coefficients = coefficients;
//             }
//         }
//     }

//     // 输出最大半圆形物体的首尾两端点
//     if (largest_semicircle->points.size() > 0)
//     {
//         std::cout << "Detected largest semicircle with radius: " << max_semicircle_radius << std::endl;

//         // 找到首尾两端点
//         PointT start_point, end_point;
//         double min_angle = M_PI; // 初始化为最大值
//         double max_angle = -M_PI; // 初始化为最小值

//         for (const auto& point : largest_semicircle->points)
//         {
//             double angle = calculateAngle(point);
//             if (angle < min_angle)
//             {
//                 min_angle = angle;
//                 start_point = point;
//             }
//             if (angle > max_angle)
//             {
//                 max_angle = angle;
//                 end_point = point;
//             }
//         }

//         // 输出首尾两端点坐标
//         std::cout << "Start point (x, y): (" << start_point.x << ", " << start_point.y << ")" << std::endl;
//         std::cout << "End point (x, y): (" << end_point.x << ", " << end_point.y << ")" << std::endl;

//         // 发布最大的半圆形物体
//         sensor_msgs::PointCloud2 output;
//         pcl::toROSMsg(*largest_semicircle, output);
//         output.header = scan->header;
//         pub.publish(output);
//     }
// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "laser_scan_processor");
//     ros::NodeHandle nh;

//     ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/RightLidar/scan_right", 1, laserScanCallback);
//     pub = nh.advertise<sensor_msgs::PointCloud2>("largest_semicircle", 1);

//     ros::spin();

//     return 0;
// }


//222222222222222222222222222222222

laser_geometry::LaserProjection projector_;
ros::Publisher pc_pub_;
ros::Publisher largest_obj_pub_;
ros::Publisher lidar_info_pub;
// 计算点云的平均距离
double calculateAverageDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->points.empty()) return 0.0;
    
    double total_distance = 0.0;
    const pcl::PointXYZ origin(0, 0, 0);  // 传感器原点
    
    for (const auto& point : cloud->points) {
        total_distance += pcl::euclideanDistance(point, origin);
    }
    
    return total_distance / cloud->points.size();
}

// 计算点的角度（相对于x轴）
double calculateAngle(const pcl::PointXYZ& point)
{
    return std::atan2(point.y, point.x);
}
double calculateCloudLength(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->points.empty()) return 0.0;
    
    // 方法1：简单计算边界框尺寸
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    // 计算三个维度的长度
    double x_len = max_pt.x - min_pt.x;
    // double y_len = max_pt.y - min_pt.y;
    // double z_len = max_pt.z - min_pt.z;
    
    // 返回最大的长度（如果是2D激光数据，z_len可能很小）
    // return std::max({x_len, y_len, z_len});
    return x_len;
    
    /*
    // 方法2：使用PCA计算主成分长度（更精确但计算量更大）
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector3f eigenvalues = pca.getEigenValues();
    return sqrt(eigenvalues[0]); // 返回最大特征值的平方根
    */
}

std::vector<pcl::PointIndices> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 创建KD树对象用于搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    
    // 设置聚类参数
    ec.setClusterTolerance(0.1);  // 1m
    ec.setMinClusterSize(10);     // 最小点数
    ec.setMaxClusterSize(25000);  // 最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr findLargestCluster(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
    std::vector<pcl::PointIndices> cluster_indices) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    size_t max_size = 0;
    int max_index = -1;
    
    // 找到点数最多的聚类
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        if (cluster_indices[i].indices.size() > max_size) {
            max_size = cluster_indices[i].indices.size();
            max_index = i;
        }
    }
    
    // 提取最大聚类
    if (max_index >= 0) {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices(cluster_indices[max_index]));
        extract.setInputCloud(cloud);
        extract.setIndices(indices_ptr);
        extract.setNegative(false);
        extract.filter(*largest_cluster);
    }
    
    return largest_cluster;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // 将LaserScan转换为PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    
    // 转换为PCL点云格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud, *cloud_ptr);
    
    // 移除NaN点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud_ptr, indices);
    
    // 发布原始点云（可选）
    // pc_pub_.publish(cloud);
    
    // 进行欧式聚类分割
    std::vector<pcl::PointIndices> cluster_indices = euclideanCluster(cloud_ptr);
    
    // 找到最大的聚类
    pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster = findLargestCluster(cloud_ptr, cluster_indices);
    
    // 发布最大的聚类
    if (largest_cluster->points.size() > 0) {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*largest_cluster, output);

        // 找到首尾两端点
        pcl::PointXYZ start_point, end_point;
        double min_angle = M_PI; // 初始化为最大值
        double max_angle = -M_PI; // 初始化为最小值

        for (const auto& point : largest_cluster->points)
        {
            double angle = calculateAngle(point);
            if (angle < min_angle)
            {
                min_angle = angle;
                start_point = point;
            }
            if (angle > max_angle)
            {
                max_angle = angle;
                end_point = point;
            }
        }
        std_msgs::Float32MultiArray array_msg;

        // 计算并输出最大物体的长度
        double length = calculateCloudLength(largest_cluster);
        ROS_INFO("Largest object length: %.2f meters", length);
        // 计算并输出平均距离
        double avg_distance = calculateAverageDistance(largest_cluster);
        // 输出首尾两端点坐标
        // std::cout << "Start point (x, y): (" << start_point.x << ", " << start_point.y << ")" << std::endl;
        // std::cout << "End point (x, y): (" << end_point.x << ", " << end_point.y << ")" << std::endl;
        array_msg.data.clear();
        array_msg.data.push_back(start_point.x);//x1
        array_msg.data.push_back(start_point.y);//y1
        array_msg.data.push_back(end_point.x);//x2
        array_msg.data.push_back(end_point.y);//y2
        array_msg.data.push_back(avg_distance);//nowBdis 保存着最大物体扫描出的每个点的距离数据
        array_msg.data.push_back(length);//当前扫描出的最大物体的曲线长度
        array_msg.data.push_back(abs(start_point.x-end_point.x));//当前扫描出的最大物体的直线长度
        lidar_info_pub.publish(array_msg);

        output.header.frame_id = scan->header.frame_id;
        output.header.stamp = scan->header.stamp;
        largest_obj_pub_.publish(output);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_clustering");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    // 订阅Lasercan话题
    std::string lidar_base;
    nh_private.param<std::string>("lidar_base",lidar_base,"left");
    ROS_INFO("lidar_base: %s",lidar_base.c_str());
    if(lidar_base=="left"){
        ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/LeftLidar/scan", 1, scanCallback);
        lidar_info_pub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_info_left", 10);
    }

    else{
        ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/RightLidar/scan", 1, scanCallback);
        lidar_info_pub = nh.advertise<std_msgs::Float32MultiArray>("/lidar_info_right", 10);
    }
    // 发布原始点云和最大聚类
    // pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1); //原始点云
    // largest_obj_pub_ = nh.advertise<sensor_msgs::PointCloud2>("largest_cluster", 1);//最大物体的点云
    
    ros::spin();
    return 0;
}


//33333333333333333333333333333333333333
// class YAxisHighestPointsDetector {
// public:
//     YAxisHighestPointsDetector() : nh_("~") {
//         // 参数设置
//         nh_.param("eps", eps_, 0.1f);
//         nh_.param("min_samples", min_samples_, 10);
//         nh_.param("y_bin_size", y_bin_size_, 0.1f);

//         // 订阅和发布
//         cloud_sub_ = nh_.subscribe("/velodyne_points", 1, &YAxisHighestPointsDetector::cloudCallback, this);
//         largest_obj_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/largest_object", 1);
//         highest_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/y_axis_highest_points", 1);

//         ROS_INFO("Y-Axis Highest Points Detector initialized");
//     }

//     void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
//         // 转换为PCL点云
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::fromROSMsg(*cloud_msg, *cloud);

//         if (cloud->empty()) {
//             ROS_WARN("Received empty point cloud");
//             return;
//         }

//         // 1. 欧式聚类提取最大物体
//         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//         tree->setInputCloud(cloud);

//         std::vector<pcl::PointIndices> cluster_indices;
//         pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//         ec.setClusterTolerance(eps_);
//         ec.setMinClusterSize(min_samples_);
//         ec.setSearchMethod(tree);
//         ec.setInputCloud(cloud);
//         ec.extract(cluster_indices);

//         if (cluster_indices.empty()) {
//             ROS_WARN("No clusters found in the point cloud");
//             return;
//         }

//         // 找到最大的聚类
//         auto largest_cluster = std::max_element(
//             cluster_indices.begin(), cluster_indices.end(),
//             [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
//                 return a.indices.size() < b.indices.size();
//             });

//         // 提取最大聚类的点云
//         pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud(cloud);
//         pcl::IndicesPtr indices_ptr(new std::vector<int>(largest_cluster->indices));
//         extract.setIndices(indices_ptr);
//         extract.filter(*largest_cloud);

//         // 发布最大物体点云
//         sensor_msgs::PointCloud2 largest_cloud_msg;
//         pcl::toROSMsg(*largest_cloud, largest_cloud_msg);
//         largest_cloud_msg.header = cloud_msg->header;
//         largest_obj_pub_.publish(largest_cloud_msg);

//         // 2. 按Y轴分段并找出每段的最高点
//         if (largest_cloud->empty()) {
//             ROS_WARN("Largest cluster has no points");
//             return;
//         }

//         // 计算Y轴范围
//         float y_min = std::numeric_limits<float>::max();
//         float y_max = std::numeric_limits<float>::lowest();
//         for (const auto& point : *largest_cloud) {
//             if (point.y < y_min) y_min = point.y;
//             if (point.y > y_max) y_max = point.y;
//         }

//         // 创建Y轴分段
//         std::map<int, pcl::PointXYZ> highest_points_map; // key: y_bin_index

//         for (const auto& point : *largest_cloud) {
//             int y_bin_index = static_cast<int>(std::floor((point.y - y_min) / y_bin_size_));
            
//             // 如果当前分段没有点，或者当前点更高，则更新
//             if (highest_points_map.find(y_bin_index) == highest_points_map.end() || 
//                 point.z > highest_points_map[y_bin_index].z) {
//                 highest_points_map[y_bin_index] = point;
//             }
//         }

//         // 提取并打印最高点
//         pcl::PointCloud<pcl::PointXYZ>::Ptr highest_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//         // 按Y坐标排序
//         std::vector<std::pair<int, pcl::PointXYZ>> sorted_points(
//             highest_points_map.begin(), highest_points_map.end());
//         std::sort(sorted_points.begin(), sorted_points.end(),
//             [](const auto& a, const auto& b) { return a.first < b.first; });

//         for (const auto& entry : sorted_points) {
//             const auto& point = entry.second;
//             highest_cloud->push_back(point);

//             // 打印点信息
//             ROS_INFO_STREAM("Y segment [" << (y_min + entry.first * y_bin_size_) << ", " 
//                             << (y_min + (entry.first + 1) * y_bin_size_) << "): Highest point (x,y,z) = ("
//                             << point.x << ", " << point.y << ", " << point.z << ")");

//             // 添加点到标记
//             geometry_msgs::Point p;
//             p.x = point.x;
//             p.y = point.y;
//             p.z = point.z;

//         }

//         // 发布最高点点云
//         if (!highest_cloud->empty()) {
//             sensor_msgs::PointCloud2 highest_cloud_msg;
//             pcl::toROSMsg(*highest_cloud, highest_cloud_msg);
//             highest_cloud_msg.header = cloud_msg->header;
//             highest_points_pub_.publish(highest_cloud_msg);

//         } else {
//             ROS_WARN("No highest points found in any Y segments");
//         }
//     }
//     ros::NodeHandle nh_;
//     ros::Subscriber cloud_sub_;
//     ros::Publisher largest_obj_pub_;
//     ros::Publisher highest_points_pub_;

//     float eps_;
//     int min_samples_;
//     float y_bin_size_;
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "y_axis_highest_points_detector");
//     YAxisHighestPointsDetector detector;
//     ros::spin();
//     return 0;
// }