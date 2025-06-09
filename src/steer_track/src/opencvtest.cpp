#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
int main(){
    //jyx添加，用于剔除不必要的特征点
    //测试用代码，平时不调用
    int ROW=1024;
    int COL=1440;
    int width = 1000;
    int height = 700;
    int x = COL/2 - width/2;
    int y = ROW/2 - height/2;
    cv::Rect roiRect(x, y, width, height);
    cv::Mat ROI_black= cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(0));//先建立一个跟原始图像相同分辨率的纯黑色的矩形
    // cv::Mat ROI_white= cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));//建立一个跟原始图像相同分辨率的白黑色的矩形
    ROI_black(roiRect)=255; //再建立一个纯白色的矩形，这个矩形代表了真正的ROI区域
        // 展示结果
    cv::imshow("White Region", ROI_black);
    cv::waitKey(0);
    
    return 0;
}
