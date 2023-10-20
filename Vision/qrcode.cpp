#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>
#include <vector>
#include <zbar.h>
#include <sstream>
#include <std_msgs/String.h>

using namespace std;
using namespace cv;
using namespace zbar;

int detect_barcode(Mat src, char *buffer);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "code_car");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("qrcode", 1);
    ros::Rate loop_rate(10);
    VideoCapture cap(213);
    char QR_code_buffer[10];
    std_msgs::String msg;

    while (ros::ok())
    {
        Mat frame;
        cap >> frame;
        if (frame.empty())
        {   
            ROS_ERROR("frame is empty");
            continue;
        }

        int flag;
        flag = detect_barcode(frame, QR_code_buffer);
        if (flag == 1)
        {
            std::stringstream ss;
            ss << QR_code_buffer;
            msg.data = ss.str();
            ROS_INFO("%s", msg.data.c_str());
            pub.publish(msg);
            cout << "QR_code_buffer: " << QR_code_buffer << endl;
        }

        imshow("frame", frame);

        if (waitKey(30) == 'q')
            break;
        loop_rate.sleep();
    }
    return 0;
}

// 检测条形码
int detect_barcode(Mat src, char *buffer)
{
    int result = 0;
    string code_str;
    Mat src_gray;
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    ImageScanner scanner;
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
    int width = src_gray.cols;
    int height = src_gray.rows;
    uchar *raw = (uchar *)src_gray.data;
    Image imageZbar(width, height, "Y800", raw, width * height);
    scanner.scan(imageZbar); // 扫描条码
    Image::SymbolIterator symbol = imageZbar.symbol_begin();

    if (imageZbar.symbol_begin() == imageZbar.symbol_end())
    {
        cout << "查询条码失败，请检查图片！" << endl;
    }

    for (; symbol != imageZbar.symbol_end(); ++symbol)
    {
        result = 1;
        cout << "类型：" << endl
             << symbol->get_type_name() << endl
             << endl;
        cout << "条码：" << endl
             << symbol->get_data() << endl
             << endl;
        strcpy(buffer, symbol->get_data().c_str());
        // code_str = symbol->get_data();
    }
    // int result = 0;
    // result = code_str[3] - '0';
    return result;
}