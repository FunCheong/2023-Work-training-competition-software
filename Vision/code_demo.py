import cv2
import numpy as np
import time


def white_balance(img, parameter=1.55):

    b, g, r = cv2.split(img)

    def con_num(x):
        if x > 0:
            return 1
        if x < 0:
            return -1
        if x == 0:
            return 0
    yuv_img = cv2.cvtColor(img, cv2.COLOR_BGR2YCrCb)
    (y, u, v) = cv2.split(yuv_img)

    max_y = np.max(y.flatten())

    avl_u = np.average(u)
    avl_v = np.average(v)

    avl_du = np.average(np.abs(u-avl_u))
    avl_dv = np.average(np.abs(v-avl_v))

    radio = 0.5  # 如果该值过大过小，色温向两极端发展
    # 计算差值矩阵
    du_diff = np.abs(u - (avl_u + avl_du * con_num(avl_u)))
    dv_diff = np.abs(v - (avl_v + avl_dv * con_num(avl_v)))

    # 创建二值掩码矩阵
    value = (du_diff < radio * avl_du) | (dv_diff < radio * avl_dv)

    # 将满足条件的像素点赋值给num_y，并计算yhistogram和ysum
    num_y = np.where(value, y, 0)
    yhistogram, _ = np.histogram(num_y, bins=range(257))
    ysum = np.sum(value)

    cumulative_histogram = np.cumsum(yhistogram)
    key = np.argmax(cumulative_histogram >= parameter * ysum)
    print("light key={}".format(key))

    avl_r = np.average(r[num_y > key])
    avl_g = np.average(g[num_y > key])
    avl_b = np.average(b[num_y > key])

    # 计算缩放因子
    scale_factor = max_y / np.array([avl_b, avl_g, avl_r])

    # 通过广播机制对通道进行缩放
    b = np.round(b * scale_factor[0]).clip(0, 255).astype(np.uint8)
    g = np.round(g * scale_factor[1]).clip(0, 255).astype(np.uint8)
    r = np.round(r * scale_factor[2]).clip(0, 255).astype(np.uint8)

    return cv2.merge([b, g, r]), key


# frame = cv2.imread(
#     'C:\\Users\\25102\\Desktop\\openCV_python\\data_gongxun\\9.jpg')
# frame = cv2.resize(frame, (640, 360))

w_b = 2.5
cap = cv2.VideoCapture(211)
# cap.set(cv2.CAP_PROP_FPS, 15)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,12)
while True:
    # wb_b, wb_g, wb_r = calculate_white_balance(frame)
    # print(wb_b, wb_g, wb_r)
    _, frame = cap.read()
    # src = adjust_color_thresholds(frame, white_balance)
    src, key = white_balance(frame, w_b)
    if key > 200:
        w_b = w_b-0.02
    elif key < 190:
        w_b = w_b+0.02
    if w_b > 3.0:
        w_b = 1.4

    cv2.imshow('src', src)
    cv2.imshow('frame', frame)
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break
