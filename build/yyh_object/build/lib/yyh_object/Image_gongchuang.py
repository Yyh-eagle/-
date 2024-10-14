import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
from rcl_interfaces.msg import ParameterDescriptor



import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库
import math

lower_red = np.array([0, 90, 128])      # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])   # 红色的HSV阈值上限


#创建一个订阅者节点

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                # ROS2节点父类初始化
        
    
        self.task_state=-1# the state machine
    

        self.usb = cv2.VideoCapture(0)#机械臂摄像头
        self.usb2 = cv2.VideoCapture(2)#定位规划摄像头
        if not self.usb.isOpened() or not self.usb2.isOpened():
            print("无法打开摄像头")
            return
        self.cv_bridge = CvBridge()                           # 创建一个图像转换对象，用于OpenCV图像与ROS的图像消息的互相转换
        self.ProcessImage()#图像处理函数，进去了就不出来了
        
    
    def ProcessImage(self):
        while True:
            # 捕获视频流中的一帧
            ret, frame = self.usb.read()
            ret2, frame2 = self.usb2.read() 
            # 检查是否成功读取帧
            if not ret :
                print("机械臂摄像头无法接收帧，退出...")
                break
            if not ret2:
                print("定位摄像头无法接收帧，退出...")
                break
            # 在窗口中显示视频流
            cv2.imshow('arm camera', frame)
            cv2.imshow('nav camera', frame2)
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 释放摄像头和关闭窗口
        self.usb.release()
        self.usb2.release()
        cv2.destroyAllWindows()

   
    #二维码
    def decodeDisplay(self):
        img=cv2.cvtColor(self.usb,cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(self.usb)
        rects_list = []
        QR_info = []
        results=[]
        # 这里循环，因为画面中可能有多个二维码
        for barcode in barcodes:
            # 提取条形码的边界框的位置
            # 画出图像中条形码的边界框
            (x, y, w, h) = barcode.rect
            cx=x+w//2
            cy=y+h//2
            rects_list.append((x, y, w, h))
            cv2.rectangle(self.usb, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # 条形码数据为字节对象，所以如果我们想在输出图像上画出来，就需要先将它转换成字符串
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            location=[cx,cy]
            # 绘出图像上条形码的数据和条形码类型
            text = str(barcodeData)
            result=[text,location]
            results.append(result)
            cv2.putText(self.usb, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0, 0, 125), 2)
            # 向终端打印条形码数据和条形码类型
            # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
        return results

    #颜色识别
    def color_detect(self):
        hsv_img = cv2.cvtColor(self.colord435i, cv2.COLOR_BGR2HSV)      # 图像从BGR颜色模型转换为HSV模型
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red) # 图像二值化
        contours, hierarchy = cv2.findContours(
            mask_red, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)   # 图像中轮廓检测
 
        for cnt in contours:                                  # 去除一些轮廓面积太小的噪声
            if cnt.shape[0] < 150:
                continue
 
            (x, y, w, h) = cv2.boundingRect(cnt)              # 得到苹果所在轮廓的左上角xy像素坐标及轮廓范围的宽和高
            cv2.drawContours(self.colord435i, [cnt], -1, (0, 255, 0), 2)# 将苹果的轮廓勾勒出来
            cv2.circle(self.colord435i, (int(x+w/2), int(y+h/2)), 5,
                       (0, 255, 0), -1)                       # 将苹果的图像中心点画出来
 
        cv2.imshow("object", self.colord435i)                           # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)
        
    #circle detect
    def houf_circle(self):
        
        #time =0.02s
        minRadius=25
        maxRadius=600
        circles = []
        e1=cv2.getTickCount()
        gray_image = cv2.cvtColor(self.colord435i,cv2.COLOR_BGR2GRAY)
        blur_image = cv2.GaussianBlur(gray_image,(5,5),0)
        ret,thresh = cv2.threshold(gray_image,125,255,cv2.THRESH_BINARY)
        
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5), (-1, -1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, (-1, -1))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, (-1, -1))

        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if len(cnt) <5:
                continue
                
            area = cv2.contourArea(cnt)
            if area < (minRadius**2) * math.pi or area > (maxRadius**2) * math.pi:
                continue
            
            arc_length = cv2.arcLength(cnt, True)
            radius = arc_length / (2 * math.pi)
            
            if not (minRadius < radius and radius < maxRadius):
                continue
            
            ellipse = cv2.fitEllipse(cnt)
            ratio = float(ellipse[1][0]) / float(ellipse[1][1])
            
            if ratio > 0.8 and ratio < 1.2:#e
                corner = cv2.approxPolyDP(cnt, 0.02 * arc_length, True)
                cornerNum = len(corner)
                if cornerNum > 4: # 当cornerNum=4时，识别矩形；而cornerNum>4时，识别圆
                    circles.append(ellipse)
                    
            for circle in circles:
                print(circle)
                cv2.circle(self.colord435i, (int(circle[0][0]), int(circle[0][1])), int(0.25*(circle[1][0]+circle[1][1])), (0, 255, 0), thickness=5)
        
        e2 = cv2.getTickCount()
        time = (e2 - e1)/ cv2.getTickFrequency()
        #print(circles)
        cv2.imshow("hourf", self.colord435i)                           # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)  
        #print(time)
                    

 
 
def main(args=None):                            # ROS2节点主入口main函数()
    rclpy.init(args=args)                       # ROS2 Python接口初始化
    node = ImageSubscriber("Image")  # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                            # 循环等待ROS2退出
    node.destroy_node()                         # 销毁节点对象
    rclpy.shutdown()                            # 关闭ROS2 Python接口
