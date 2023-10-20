import cv2
import numpy as np
import math
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import serial
import re
import threading

# 初始化全局变量，用于存储最后检测到的圆心坐标
last_detected_circle_center = (0, 0)

# 雷达坐标和yaw
position_x, position_y, position_yaw = 0, 0, 0

# 串口初始化
try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',  # 根据你的串口设备更改
        baudrate=9600,        # 波特率，需要与接收端一致
        timeout=5             # 读取串口的超时时间，单位秒
    )
except Exception as e:
    print(f"An error occurred: {e}")

try:
    ser_2 = serial.Serial(
        port='/dev/ttyUSB1',  # 根据你的串口设备更改
        baudrate=115200,        # 波特率，需要与接收端一致
        timeout=5             # 读取串口的超时时间，单位秒
    )
except Exception as e:
    print(f"An error occurred: {e}")

# 颜色编码和颜色范围
# color_ranges_map = {
#     1: ([147, 69,  61], [190, 255, 218]),    # 红色
#     3: ([91, 72, 54], [142, 115, 147]),    # 蓝色
#     2: ([58, 24, 137], [83, 78, 242])  # 绿色
# }

# color_ranges_obj = {
#     1: ([160, 145, 90], [190, 145, 170]),    # 红色
#     3: ([102, 130, 84], [130, 209, 128]),    # 蓝色
#     2: ([67, 165, 133], [85, 255, 209])  # 绿色
# }#亮光

# color_ranges_obj = {
#     1: ([163, 70, 130], [190, 169, 222]),    # 红色
#     3: ([100, 120, 140], [120, 214, 210]),    # 蓝色
#     2: ([70, 90, 143], [95, 190, 210])  # 绿色
# }  # 晚上暗光

# color_ranges_obj = {
#     1: ([162, 155, 124], [190, 234, 174]),    # 红色
#     3: ([94, 90, 100], [123, 255, 150]),    # 蓝色
#     2: ([70, 111, 119], [100, 255, 166])  # 绿色
# }  # 下午

color_ranges_obj = {
    1: ([166, 90, 140], [255, 246, 255]),    # 红色
    2: ([26, 72, 141], [75, 204, 241]),  # 绿色
    3: ([106, 32, 90], [146, 222, 238])    # 蓝色
}  # 补光灯

color_ranges_map = {
    1: ([159, 55, 67], [255, 255, 180]),    # 红色
    2: ([31, 42, 96], [76, 158, 223]),  # 绿色
    3: ([109, 75, 32], [144, 255, 165])    # 蓝色
} #补光灯

color = 1  # red blue green
model = 1  # map obj
param1 = 20  # 霍夫圆参数
param2 = 5
qrcode = [1,2,3, 3, 2, 1]  # 二维码信息
flag = 1
flag_qrcode = 0 #二维码信息接收标志位
color_num = 0
switch_flag=0
# 串口发送二进制流


def send_msgs(data_list):
    try:
        # 打开串口
        if ser.is_open:
            for data in data_list:
                # 将整数转换为二进制字符串
                binary_data = format(data, '08b')  # 将整数转换为8位的二进制字符串
                # 发送二进制数据
                ser.write(bytes([int(binary_data, 2)]))  # 将二进制字符串转换为字节并发送

        else:
            print(f"Could not open serial port {ser.port}.")
    except Exception as e:
        print(f"An error occurred: {e}")

# 串口发送字符串


def send_string(data_string):
    try:
        if ser_2.is_open:
            string = "t0.txt=\""+data_string+"\""
            ser_2.write(string.encode("GB2312"))
            ser_2.write(bytes.fromhex('ff ff ff'))
            print("successful")
            # ser.close()
        else:
            print(f"Could not open serial port {ser_2.port}.")
    except Exception as e:
        print(f"An error occurred: {e}")


def send_string2(data_string):
    try:
        if ser.is_open:
            string= "$"+data_string+"\r\n"
            ser.write(string.encode("GB2312"))
            print("successful")
        else:
            print(f"Could not open serial port {ser.port}.")
    except Exception as e:
        print(f"An error occurred: {e}")
# 封装坐标信息


def lidar_Send_msgs(x, y, yaw):
    datalist = []
    datalist.append(0xAA)
    bin_x = bin(x & 0xffff)[2:].zfill(16)
    bin_y = bin(y & 0xffff)[2:].zfill(16)
    bin_yaw = bin(yaw & 0xffff)[2:].zfill(16)
    datalist.append(int(bin_x[8:16], 2))
    datalist.append(int(bin_x[0:8], 2))
    datalist.append(int(bin_y[8:16], 2))
    datalist.append(int(bin_y[0:8], 2))
    datalist.append(int(bin_yaw[8:16], 2))
    datalist.append(int(bin_yaw[0:8], 2))
    datalist.append(0x0A)
    datalist.append(0x0D)

    return datalist

# 封装偏差坐标信息


def erro_Send_msgs(x, y):
    datalist = []
    datalist.append(0x55)
    bin_x = bin(x & 0xffff)[2:].zfill(16)
    bin_y = bin(y & 0xffff)[2:].zfill(16)
    datalist.append(int(bin_x[8:16], 2))
    datalist.append(int(bin_x[0:8], 2))
    datalist.append(int(bin_y[8:16], 2))
    datalist.append(int(bin_y[0:8], 2))
    datalist.append(0x0A)
    datalist.append(0x0D)

    return datalist

# 获取定位坐标


def lidar_callback(data):
    global position_x, position_y, position_yaw
    position_x = int(data.pose.pose.position.x*100)
    position_y = int(data.pose.pose.position.y*100)
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    position_yaw = int(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))*100)
    # print("x: ", position_x)
    # print("y: ", position_y)
    # print("yaw:", position_yaw)

# 获取二维码信息


def qrcode_callback(data):
    global qrcode,flag_qrcode
    msg = data.data
    # 遍历字符串中的每个字符
    if flag_qrcode==1:
        for char in msg:
            if char.isdigit():
                # 如果字符是数字，则将其转换为整数并添加到数组中
                qrcode.append(int(char))
        # send_string(msg)
        send_string2(msg)
        flag_qrcode=0

# 识别颜色并返回对应颜色编码


def detect_color(frame):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    color_code = 0

    for code, (lower, upper) in color_ranges_obj.items():
        mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        if cv2.countNonZero(mask) > 10000:  # 如果颜色区域像素数量大于100，认为检测到颜色
            color_code = code
            break

    return color_code

# 检测最靠近图像下方的圆的坐标


def find_obj_pos(frame, radius):
    global last_detected_circle_center, param1, param2
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_frame=hsv_frame[100:480,:]
    mask_red = cv2.inRange(hsv_frame, np.array(
        color_ranges_obj[1][0]), np.array(color_ranges_obj[1][1]))
    mask_green = cv2.inRange(hsv_frame, np.array(
        color_ranges_obj[2][0]), np.array(color_ranges_obj[2][1]))
    mask_blue = cv2.inRange(hsv_frame, np.array(
        color_ranges_obj[3][0]), np.array(color_ranges_obj[3][1]))
    mask = mask_red + mask_blue + mask_green
    # cv2.imshow("mask", mask)
    # cv2.imshow("mask_blue", mask_blue)

    circles = cv2.HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=10000,
        param1=20,
        param2=5,
        minRadius=70,
        maxRadius=100
    )
    max_y = -1
    max_circle = None
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            # cv2.circle(src, (x, y), r, (0, 0, 255), 2)
            if y > max_y:
                max_y = y
                max_circle = circle
    if max_circle is not None:
        x, y, r = max_circle
        cv2.circle(src,(x,y+100),r,(0,0,255),2)
        x=int(x)
        y=int(y)
        r=int(r)
        # print("x:{}".format(x))
        # print("y:{}".format(y))
        # print("erro_x:{}".format(x-last_detected_circle_center[0]))
        # print("erro_y:{}".format(y-last_detected_circle_center[1]))
        # print("last_x{}".format(last_detected_circle_center[0]))
        # print("last_y{}".format(last_detected_circle_center[1]))
        if abs(x-last_detected_circle_center[0])+abs(y-last_detected_circle_center[1]) < radius:
            last_detected_circle_center = (x, y)
            return x, y+100, r
        # elif last_detected_circle_center[0]<680 and last_detected_circle_center[1]<480:
        else:
            last_detected_circle_center = (x, y)
            return math.nan, math.nan, math.nan
    else:
        return math.nan, math.nan, math.nan


# 函数3: 检测颜色并返回颜色编码

def get_obj(frame, radius):

    x, y, r = find_obj_pos(frame, radius)
    # print("r={}".format(r))
    cv2.line(src, (int(640*0.3), 0),(int(640*0.3),480),255,2)
    cv2.line(src, (int(640*0.7), 0),(int(640*0.7),480),255,2)
    if not math.isnan(x) and x>frame.shape[1]*0.3 and x<frame.shape[1]*0.7:
        roi = frame[max(y-r,0):min(y+r,480), max(x-r,0):min(x+r,640)]
        # cv2.imshow("roi",roi)
        color_code = detect_color(roi)
        return color_code
    else:
        return math.nan


# 函数4: 根据颜色编码识别圆圈，返回坐标

def put_obj(frame, color_code, model):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if model == 1:
        mask = cv2.inRange(hsv_frame, np.array(
            color_ranges_map[color_code][0]), np.array(color_ranges_map[color_code][1]))
    elif model == 2:
        mask = cv2.inRange(hsv_frame, np.array(
            color_ranges_obj[color_code][0]), np.array(color_ranges_obj[color_code][1]))
    kernel = np.ones((11, 11), np.uint8)
    close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("close in put_obj",close)

    circles = cv2.HoughCircles(
        close,
        cv2.HOUGH_GRADIENT,
        dp=1,
        minDist=1000,
        param1=20,
        param2=5,
        minRadius=20,
        maxRadius=200
    )
    max_y = -1
    max_circle = None
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for circle in circles[0, :]:
            x, y, r = circle
            cv2.circle(src, (x, y), r, (0, 0, 255), 2)
            if y > max_y:
                max_y = y
                max_circle = circle
    if max_circle is not None:
        x,y,r=max_circle
        erro_x = 0
        erro_y = 0
        height, width = frame.shape[:2]
        erro_x = x-width*0.5
        erro_y = y-height*0.5
        return erro_x, erro_y
        # cv2.circle(src,(x,y),r,(0,0,255),2)
    else:
        return math.nan, math.nan


# 状态切换

def status_change(sta, frame, color):
    # global flag, qrcode, status

    flag = 0  # 颜色轮换标志位
    # print("sta={}".format(sta))

    if sta == 0:
        pass

    elif sta == 1:  # 第一次取料

        # x, y, _ = find_obj_pos(frame, radius=10)
        # while abs(x-frame.shape[1]*0.5) > 20 or abs(y-frame.shape[0]*0.5) > 20:
        #     x, y, _ = find_obj_pos(frame, radius=10)
        #     data=erro_Send_msgs(int(x),int(y))
        #     send_msgs(data)
        #     pass  # 串口发送偏差
        print("取料:{}\n".format(color))
        if color == get_obj(frame, radius=2):
            send_msgs([0xfc])  # 抓取
            flag = 1
        return flag

    elif sta == 2:  # 第一次放置识别

        print("第一次放置识别内:{}\n".format(color))
        erro_x, erro_y = put_obj(frame, color, 1)
        if not math.isnan(erro_x) and not math.isnan(erro_y):
            data = erro_Send_msgs(int(erro_x), int(erro_y))
            send_msgs(data)
            pass  # 串口发送偏差

    # elif sta == 3:  # 第一次重复取料

    #     print("重复取料第一次:{}\n".format(color))
    #     erro_x, erro_y = put_obj(frame, color, 2)
    #     if not math.isnan(erro_x) and not math.isnan(erro_y):
    #         data = erro_Send_msgs(int(erro_x), int(erro_y))
    #         send_msgs(data)
    #         pass  # 串口发送偏差

    elif sta == 3:  # 第二次放置识别

        print("第一次放置识别外:{}\n".format(color))
        erro_x, erro_y = put_obj(frame, color, 1)
        if not math.isnan(erro_x) and not math.isnan(erro_y):
            data = erro_Send_msgs(int(erro_x), int(erro_y))
            send_msgs(data)
            pass  # 串口发送偏差

    elif sta == 4:  # 第二次取料

        # x, y, _ = find_obj_pos(frame, radius=10)
        # while abs(x-frame.shape[1]*0.5) > 10 or abs(y-frame.shape[0]*0.5) > 10:
        #     x, y, _ = find_obj_pos(frame, radius=10)
        #     data = erro_Send_msgs(int(x), int(y))
        #     send_msgs(data)
        #     pass  # 串口发送偏差

        print("第二次取料:{}\n".format(color))
        if color == get_obj(frame, radius=2):
            send_msgs([0xfc])
            flag = 1
        return flag

    elif sta == 5:

        print("第二次放置识别内:{}\n".format(color))
        erro_x, erro_y = put_obj(frame, color, 1)
        if not math.isnan(erro_x) and not math.isnan(erro_y):
            data = erro_Send_msgs(int(erro_x), int(erro_y))
            send_msgs(data)
            pass  # 串口发送偏差

    # elif sta == 7:

    #     print("重复取料第二次:{}\n".format(color))
    #     erro_x, erro_y = put_obj(frame, color, 2)
    #     if not math.isnan(erro_x) and not math.isnan(erro_y):
    #         data = erro_Send_msgs(int(erro_x), int(erro_y))
    #         send_msgs(data)
    #         pass  # 串口发送偏差

    elif sta == 6:

        print("码垛:{}\n".format(color))
        erro_x, erro_y = put_obj(frame, color, 2)
        if not math.isnan(erro_x) and not math.isnan(erro_y):
            data = erro_Send_msgs(int(erro_x), int(erro_y))
            send_msgs(data)
            pass  # 串口发送偏差


# 串口接收

def receive_msgs(num):
    global flag_qrcode
    try:
        # 从串口读取数据
        data = ser.read(1)  # 读取1字节数据
        print(data)
        print(data)
        print(data)
        if len(data) == 0:
            return 0
        # 解析数据并输出相应的值
        first_byte = data[0]
        if first_byte == 0xfe:  # 开始识别
            status = 1
        elif first_byte == 0xfd:  # 扫码
            flag_qrcode = 1
            status=0
        elif first_byte == 0xfb:  # 结束识别
            status = 3
        elif first_byte == 0xfc:  # 放置完成
            status = 4
        else:
            status=0

    except serial.SerialException as e:
        print("Serial Exception:", e)

    return status


# def test(frame):
#     global color,model
#     hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
#     # print("color={}".format(color))
#     if model==1:
#         if color==1:
#             mask=cv2.inRange(hsv,np.array(color_ranges_map[1][0]),np.array(color_ranges_map[1][1]))
#         elif color==2:
#             mask=cv2.inRange(hsv,np.array(color_ranges_map[2][0]),np.array(color_ranges_map[2][1]))
#         elif color==3:
#             mask=cv2.inRange(hsv,np.array(color_ranges_map[3][0]),np.array(color_ranges_map[3][1]))
#     elif model==2:
#         if color==1:
#             mask=cv2.inRange(hsv,np.array(color_ranges_obj[1][0]),np.array(color_ranges_obj[1][1]))
#         elif color==2:
#             mask=cv2.inRange(hsv,np.array(color_ranges_obj[2][0]),np.array(color_ranges_obj[2][1]))
#         elif color==3:
#             mask=cv2.inRange(hsv,np.array(color_ranges_obj[3][0]),np.array(color_ranges_obj[3][1]))
#     cv2.imshow("test_color",mask)
#     cv2.imshow("frame",frame)

# def get_trackbar(_):
#     global color_ranges_map,color_ranges_obj,color,model,param1,param2
#     h_min=cv2.getTrackbarPos('h_min','color space test')
#     h_max=cv2.getTrackbarPos('h_max','color space test')
#     s_min=cv2.getTrackbarPos('s_min','color space test')
#     s_max=cv2.getTrackbarPos('s_max','color space test')
#     v_min=cv2.getTrackbarPos('v_min','color space test')
#     v_max=cv2.getTrackbarPos('v_max','color space test')
#     color=cv2.getTrackbarPos('color','color space test')
#     model=cv2.getTrackbarPos('model','color space test')
#     param1=cv2.getTrackbarPos('param1','color space test')
#     param2=cv2.getTrackbarPos('param2','color space test')
#     if model==1:
#         if color==1:
#             color_ranges_map[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])
#         elif color==2:
#             color_ranges_map[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])
#         elif color==3:
#             color_ranges_map[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])
#     elif model==2:
#         if color==1:
#             color_ranges_obj[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])
#         elif color==2:
#             color_ranges_obj[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])
#         elif color==3:
#             color_ranges_obj[color]=([h_min,s_min,v_min],[h_max,s_max,v_max])


# frame = cv2.imread(
#     'C:\\Users\\25102\\Desktop\\openCV_python\\data_gongxun\\2.jpg')
# frame = cv2.resize(frame, (640, 360))

cap = cv2.VideoCapture(211)


# 订阅雷达消息
rospy.init_node("lidar_position")
rospy.Subscriber("/Odometry", Odometry, lidar_callback, queue_size=1)
rospy.Subscriber("qrcode", String, qrcode_callback, queue_size=1)
rate = rospy.Rate(100)

# cv2.namedWindow('color space test')
# cv2.resizeWindow('color space test',800,600)
# cv2.createTrackbar('h_min','color space test',0,255,get_trackbar)
# cv2.createTrackbar('h_max','color space test',0,255,get_trackbar)
# cv2.createTrackbar('s_min','color space test',0,255,get_trackbar)
# cv2.createTrackbar('s_max','color space test',0,255,get_trackbar)
# cv2.createTrackbar('v_min','color space test',0,255,get_trackbar)
# cv2.createTrackbar('v_max','color space test',0,255,get_trackbar)
# cv2.createTrackbar('color','color space test',1,3,get_trackbar)
# cv2.createTrackbar('model','color space test',1,2,get_trackbar)
# cv2.createTrackbar('param1','color space test',10,255,get_trackbar)
# cv2.createTrackbar('param2','color space test',10,255,get_trackbar)

# send_string("123+321")

while not rospy.is_shutdown():

    # 发布坐标定位信息
    send_data = lidar_Send_msgs(position_x, position_y, position_yaw)
    send_msgs(send_data)

    # 图像部分
    _, frame = cap.read()
    if frame is not None:
        src = frame.copy()
    else:
        continue

    # 串口接收部分
    num = ser.in_waiting
    status = 0
    if num != 0:
        status = receive_msgs(num)

    # 状态机部分
    if status == 3:
        switch_flag=0
        flag += 1
        if flag == 4:
            color_num = 3
    elif status==1:
        switch_flag=1
    # switch_flag=1
    if switch_flag==1:
        color = qrcode[color_num]
        result = status_change(flag, frame, color)
        print("color_num={}".format(color_num))
        if result == 1 or status == 4:  # 抓完以及放置完颜色轮换
            color_num += 1
        if color_num > 2 and flag < 4:  # 第一次任务
            color_num = 0
        elif color_num > 5 and 4 <= flag < 7:  # 第二次任务
            color_num = 3
    # test(frame)
    # cv2.imshow("src", src)
    if cv2.waitKey(10) == ord('q'):
        break
    rate.sleep()

cv2.destroyAllWindows()
