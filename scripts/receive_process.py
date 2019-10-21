# coding=utf-8
import rospy  # ROS库
from sensor_msgs.msg import Image   # ROS中接收传感器的图片格式，要注意的是别漏了.msg，否则导入失败
import cv_bridge    # 用于方便实现OpenCV与ROS的类型转换
import cv2  # OpenCV库


# 相关内容在之前OpenCV笔记中已经说过，此处不再赘述
def imgStretch(cv_img):
    img_hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
    img_h = img_hsv[:, :, 0]
    img_s = img_hsv[:, :, 1]
    img_v = img_hsv[:, :, 2]
    equal_img_v = cv2.equalizeHist(img_v)
    img_hsv[:, :, 2] = equal_img_v
    equal_img = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
    return equal_img


# 接收到数据后的回调函数
def callback(img):
    # 新建一个cv_bridge对象
    bridge = cv_bridge.CvBridge()

    # 调用bridge对象的成员函数imgmsg_to_cv2将ROS的Image转成OpenCV的NDArray
    # 第一个参数是待转换数据
    # 第二个参数是编码格式字符串，在sensor_msgs/image_encodings.h中有定义
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")

    # 调用直方图均衡化函数对彩色图像进行拉伸
    equal_img = imgStretch(cv_img)

    # 再次调用bridge的成员函数cv2_to_imgmsg将NDArray类型转成ROS的Image类型
    # 第一个参数是待转换数据
    # 第二个参数是待编码格式字符串，这里因为还是8位彩色图像，所以不变
    img_msg = bridge.cv2_to_imgmsg(equal_img, "bgr8")

    # 调用发布函数发布数据
    pub.publish(img_msg)

    # 调用OpenCV展示直接接收到的原始数据
    cv2.imshow("data from camera", cv_img)
    cv2.waitKey(3)


if __name__ == "__main__":
    # 初始化节点并指定名称
    rospy.init_node("receive_process")

    # 新建Subscriber用于接收原始数据
    # 第一个参数是接收Topic的名称
    # 第二个参数是接收数据的类型
    # 第三个参数是接收到数据后的回调函数
    rospy.Subscriber("image_raw", Image, callback)

    # 新建Publisher用于发布处理后的数据
    # 第一个参数是发布的Topic名称
    # 第二个参数是发布的数据类型
    # 第三个参数是队列长度，一般是一个较小的整数即可
    pub = rospy.Publisher("processed_img", Image, queue_size=1)

    # 循环执行
    rospy.spin()
