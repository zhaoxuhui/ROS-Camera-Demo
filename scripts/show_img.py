# coding=utf-8
import rospy  # ROS库
from sensor_msgs.msg import Image   # ROS中接收传感器的图片格式，要注意的是别漏了.msg，否则导入失败
import cv_bridge    # 用于方便实现OpenCV与ROS的类型转换
import cv2  # OpenCV库


# 接收到数据后的回调函数
def callback(img):
    # 新建一个cv_bridge对象
    bridge = cv_bridge.CvBridge()

    # 调用bridge对象的成员函数imgmsg_to_cv2将ROS的Image转成OpenCV的NDArray
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
    cv2.imshow("data processed", cv_img)
    cv2.waitKey(3)


if __name__ == "__main__":
    # 初始化节点并指定名称
    rospy.init_node("show_img")

    # 新建Subscriber用于接收发布的处理后的数据
    rospy.Subscriber("processed_img", Image, callback)

    # 循环执行
    rospy.spin()
