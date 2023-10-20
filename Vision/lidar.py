import rospy
from nav_msgs.msg import Odometry
import serial
import re
import numpy as np

position_x,position_y,position_yaw=0 , 0 , 0

try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',  # 根据你的串口设备更改
        baudrate=9600,        # 波特率，需要与接收端一致
        timeout=5             # 读取串口的超时时间，单位秒
    )
except Exception as e:
    print(f"An error occurred: {e}")

def send_msgs(data_list):
    try:
        # 打开串口
        if ser.is_open:
            # print(f'Serial port {ser.port} is open.')
            # ser.write(bytes([0x00]))
            for data in data_list:
                # 将整数转换为二进制字符串
                binary_data = format(data, '08b')  # 将整数转换为8位的二进制字符串

                # 发送二进制数据
                ser.write(bytes([int(binary_data, 2)]))  # 将二进制字符串转换为字节并发送

                # print(f'Sent data: {data} ({binary_data})')

            # 关闭串口
            # ser.write("\r\n".encode("utf-8"))
            # ser.close()
            # print(f'Serial port {ser.port} is closed.')
        else:
            print(f"Could not open serial port {ser.port}.")
    except Exception as e:
        print(f"An error occurred: {e}")

def send_string(data_string):
    try:
        if ser.is_open:
            string="t0.txt=\""+data_string+"\""
            ser.write(string.encode("GB2312"))
            ser.write(bytes.fromhex('ff ff ff'))
            print("successful")
            # ser.close()
        else :
            print(f"Could not open serial port {ser.port}.")
    except Exception as e:
        print(f"An error occurred: {e}")

def get_data(ser,over_time=10):
    pass

def lidar_Send_msgs(x,y,yaw):
    datalist=[]
    datalist.append(0xAA)
    bin_x=bin(x&0xffff)[2:].zfill(16)
    bin_y=bin(y&0xffff)[2:].zfill(16)
    bin_yaw=bin(yaw&0xffff)[2:].zfill(16)
    datalist.append(int(bin_x[8:16],2))
    datalist.append(int(bin_x[0:8],2))
    datalist.append(int(bin_y[8:16],2))
    datalist.append(int(bin_y[0:8],2))
    datalist.append(int(bin_yaw[8:16],2))
    datalist.append(int(bin_yaw[0:8],2))
    datalist.append(0x0A)
    datalist.append(0x0D)
    
    #for unknown reason,this line can not be deleted
#    print(len(datalist),datalist) 
    return datalist
    #return [0xff,0x01,0x02,0x03,0x04,0x05,0x06,0x0a,0x0d]


def lidar_callback(data):
    global position_x,position_y,position_yaw
    position_x=int(data.pose.pose.position.x*100)
    position_y=int(data.pose.pose.position.y*100)
    x=data.pose.pose.orientation.x
    y=data.pose.pose.orientation.y
    z=data.pose.pose.orientation.z
    w=data.pose.pose.orientation.w
    position_yaw= int(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))*100)
    print("x: ",position_x)
    print("y: ",position_y)
    print("yaw:",position_yaw)
    
rospy.init_node("lidar_position")
rospy.Subscriber("/Odometry",Odometry,lidar_callback,queue_size=1) 
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    # com_input = ser.read()
    # if com_input:
    #     print(com_input)
    # send_msgs([int(abs(position_x)),int(abs(position_y))])
    send_data=lidar_Send_msgs(position_x,position_y,position_yaw)
    #print(send_data)
    # send_msgs()
    send_msgs(send_data)
    # send_string("123+321")
    rate.sleep()

ser.close()
