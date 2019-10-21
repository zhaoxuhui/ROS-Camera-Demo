# include<ros/ros.h>    // ROS头文件
# include<cv_bridge/cv_bridge.h>    // cv_bridge头文件，实现类型转换
# include<sensor_msgs/Image.h>  // ROS相机传感器数据格式
# include<opencv-3.3.1-dev/opencv2/opencv.hpp>  //OpenCV核心头文件

// 定义的全局变量
ros::Subscriber sub;
ros::Publisher pub;

// 对彩色图像进行灰度直方图均衡化，增强对比度
cv::Mat imgStretch(cv::Mat cv_img){
    cv::Mat img_hsv;
    cv::cvtColor(cv_img,img_hsv,cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_bands;
    cv::split(img_hsv,hsv_bands);
    cv::equalizeHist(hsv_bands[2],hsv_bands[2]);
    cv::Mat equal_img;
    cv::merge(hsv_bands,equal_img);
    cv::cvtColor(equal_img,equal_img,cv::COLOR_HSV2BGR);
    return equal_img;
}

cv::Mat cvtImgMsg2Mat(const sensor_msgs::ImageConstPtr& msg){
    // 利用cv_bridge::toCvCopy函数将消息格式的图片转成CV格式
    cv_bridge::CvImagePtr cvImgPtr;
    cvImgPtr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    return cvImgPtr->image;
}

sensor_msgs::ImagePtr cvtMat2ImgMsg(cv::Mat img){
    // 利用cv_bridge::CvImage函数将CV图像再次转成消息格式
    sensor_msgs::ImagePtr img_msg;
    img_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg();
    return img_msg;
}

void callback(const sensor_msgs::ImageConstPtr& msg){
    // 调用函数，将msg格式img转成mat
    cv::Mat img = cvtImgMsg2Mat(msg);

    // 对转换后的数据进行处理
    cv::Mat equal_img = imgStretch(img);
    
    // 将处理后的数据重新转成msg格式
    sensor_msgs::ImagePtr img_msg = cvtMat2ImgMsg(equal_img);

    // 发送处理后的数据
    pub.publish(img_msg);
}

int main(int argc, char *argv[])
{
    // 初始化节点并指定名称
    ros::init(argc, argv, "receive_process");
    // 新建节点管理器
    ros::NodeHandle nh;

    // 初始化pub和sub
    sub = nh.subscribe("image_raw",1,callback);
    pub = nh.advertise<sensor_msgs::Image>("processed_img",1);
    
    // 循环执行
    ros::spin();
    return 0;
}