# include<ros/ros.h>    // ROS头文件
# include<cv_bridge/cv_bridge.h>    // cv_bridge头文件，实现类型转换
# include<sensor_msgs/Image.h>  // ROS相机传感器数据格式
# include<opencv-3.3.1-dev/opencv2/opencv.hpp>  //OpenCV核心头文件

// 定义的全局变量
ros::Subscriber sub;

cv::Mat cvtImgMsg2Mat(const sensor_msgs::ImageConstPtr& msg){
    // 利用cv_bridge::toCvCopy函数将消息格式的图片转成CV格式
    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    return cvImgPtr->image;
}

void callback(const sensor_msgs::ImageConstPtr& msg){
    // 调用函数，将msg格式img转成mat
    cv::Mat img = cvtImgMsg2Mat(msg);
    
    // 展示图像
    cv::imshow("processed_img",img);
    cv::waitKey(3);
}

int main(int argc, char *argv[])
{
    // 初始化节点并指定名称
    ros::init(argc, argv, "show_img");
    // 新建节点管理器
    ros::NodeHandle nh;

    // 初始化pub和sub
    sub = nh.subscribe("processed_img",1,callback);
    
    // 循环执行
    ros::spin();
    return 0;
}