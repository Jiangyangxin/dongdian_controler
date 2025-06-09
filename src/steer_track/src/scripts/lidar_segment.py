#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
# from pcl_helper import *
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

def create_cloud_xyz32(header, points):
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1)]
    return pc2.create_cloud(header, fields, points)


class YAxisHighestPointsDetector:
    def __init__(self):
        rospy.init_node('y_axis_highest_points_detector', anonymous=True)
        
        # 订阅PointCloud2话题
        self.cloud_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.cloud_callback)
        
        # 发布分割后的最大物体点云
        self.largest_obj_pub = rospy.Publisher("/largest_object", PointCloud2, queue_size=1)
        
        # 发布按Y轴分段的最高点
        self.highest_points_pub = rospy.Publisher("/y_axis_highest_points", PointCloud2, queue_size=1)
        
        # 发布标记显示
        # self.marker_pub = rospy.Publisher("/highest_points_markers", MarkerArray, queue_size=1)
        
        # 参数设置
        self.eps = rospy.get_param('~eps', 0.1)          # DBSCAN聚类参数
        self.min_samples = rospy.get_param('~min_samples', 10)
        self.y_bin_size = rospy.get_param('~y_bin_size', 0.1)  # Y轴分段大小(米)
        
        rospy.loginfo("Y-Axis Highest Points Detector initialized")

    def cloud_callback(self, cloud_msg):
        try:
            # 1. 将PointCloud2转换为numpy数组
            points = pc2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z"))
            points_list = np.array(list(points), dtype=np.float32)
            
            if len(points_list) == 0:
                rospy.logwarn("Received empty point cloud")
                return
            
            # 2. 使用DBSCAN进行聚类
            clustering = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit(points_list)
            labels = clustering.labels_
            
            # 3. 找出最大的聚类(忽略噪声点label=-1)
            unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
            
            if len(unique_labels) == 0:
                rospy.logwarn("No clusters found in the point cloud")
                return
            
            largest_cluster_label = unique_labels[np.argmax(counts)]
            largest_cluster_points = points_list[labels == largest_cluster_label]
            
            # 4. 发布最大的聚类点云
            largest_cluster_cloud = create_cloud_xyz32(cloud_msg.header, largest_cluster_points)
            self.largest_obj_pub.publish(largest_cluster_cloud)
            
            # 5. 按Y轴分段并找出每段的最高点
            if len(largest_cluster_points) > 0:
                # 计算Y轴范围并创建分段
                y_min = np.min(largest_cluster_points[:, 1])
                y_max = np.max(largest_cluster_points[:, 1])
                y_bins = np.arange(y_min, y_max + self.y_bin_size, self.y_bin_size)
                
                # 存储每段的最高点
                highest_points = []
                
                # 遍历每个Y轴分段
                for i in range(len(y_bins)-1):
                    y_start = y_bins[i]
                    y_end = y_bins[i+1]
                    
                    # 获取当前Y轴段的点
                    mask = (largest_cluster_points[:, 1] >= y_start) & (largest_cluster_points[:, 1] < y_end)
                    segment_points = largest_cluster_points[mask]
                    
                    # 如果分段中有点，找出Z坐标最大的点
                    if len(segment_points) > 0:
                        highest_idx = np.argmax(segment_points[:, 2])
                        highest_point = segment_points[highest_idx]
                        highest_points.append(highest_point)
                        
                        # # 打印点信息
                        # rospy.loginfo(f"Y segment [{y_start:.3f}, {y_end:.3f}): Highest point (x,y,z) = "
                        #               f"({highest_point[0]:.3f}, {highest_point[1]:.3f}, {highest_point[2]:.3f})")
                
                # 发布最高点点云
                if len(highest_points) > 0:
                    highest_points_array = np.array(highest_points)
                    highest_cloud = create_cloud_xyz32(cloud_msg.header, highest_points_array)
                    self.highest_points_pub.publish(highest_cloud)
                    
                    # # 发布标记显示
                    # self.publish_highest_points_markers(cloud_msg.header, highest_points_array)
                else:
                    rospy.logwarn("No highest points found in any Y segments")
            else:
                rospy.logwarn("Largest cluster has no points")
                
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {str(e)}")

    # def publish_highest_points_markers(self, header, points):
    #     marker_array = MarkerArray()
        
    #     # 创建点标记
    #     points_marker = Marker()
    #     points_marker.header = header
    #     points_marker.ns = "highest_points"
    #     points_marker.id = 0
    #     points_marker.type = Marker.POINTS
    #     points_marker.action = Marker.ADD
    #     points_marker.scale.x = 0.05  # 点大小
    #     points_marker.scale.y = 0.05
    #     points_marker.color.a = 1.0
    #     points_marker.color.r = 1.0
    #     points_marker.color.g = 0.0
    #     points_marker.color.b = 0.0
    #     points_marker.lifetime = rospy.Duration(1.0)
        
    #     # 创建线标记(连接各点)
    #     line_marker = Marker()
    #     line_marker.header = header
    #     line_marker.ns = "highest_line"
    #     line_marker.id = 1
    #     line_marker.type = Marker.LINE_STRIP
    #     line_marker.action = Marker.ADD
    #     line_marker.scale.x = 0.02  # 线宽
    #     line_marker.color.a = 1.0
    #     line_marker.color.r = 0.0
    #     line_marker.color.g = 1.0
    #     line_marker.color.b = 0.0
    #     line_marker.lifetime = rospy.Duration(1.0)
        
    #     # 按Y坐标排序点
    #     sorted_points = points[np.argsort(points[:, 1])]
        
    #     # 添加点到标记
    #     for point in sorted_points:
    #         p = Point()
    #         p.x = point[0]
    #         p.y = point[1]
    #         p.z = point[2]
            
    #         points_marker.points.append(p)
    #         line_marker.points.append(p)
        
    #     marker_array.markers.append(points_marker)
    #     marker_array.markers.append(line_marker)
        
    #     self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        detector = YAxisHighestPointsDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass