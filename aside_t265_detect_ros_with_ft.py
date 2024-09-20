from calendar import c
from cmath import nan
from pyexpat.errors import XML_ERROR_UNKNOWN_ENCODING
from time import time
import pyrealsense2 as rs
import sys
import cv2
from cv2 import imshow
import numpy as np
import math
import time
import cv2.aruco as aruco
import csv
from sklearn import linear_model, datasets
import matplotlib.pyplot as plt
from numba import njit
import xlsxwriter
import rospy
import message_filters
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2

##############################################################
# (update 20220803) t265 detect method used now
##############################################################

detect_count = 0
is_print = False # 若为true则打印所有信息


@njit
def EdgeLine_seg(vet1, threhold1, i1, i2, j1, j2, edges1):
    x1 = []
    y1 = []
    # edgesPro=np.zeros_like(shape)
    # and img_gray1[j,i] <= 70
    for i in range(i1, i2):
        for j in range(j1, j2):
            if ((i - vet1[0]) ** 2 + (j - 0) ** 2) <= threhold1 and edges1[j, i] > 10:
                edges1[j, i] = 250
                x1.append(i)
                y1.append(j)

    xe = []
    ye = []
    stat = np.bincount(np.array(y1)) 
    #bincount() 函数计算了每个值在数组中的出现频数，然后返回一个数组。索引位置 i 的值表示值为 i 的元素在 arr 中出现的次数。
    
    # print(stat)
    # print(np.mean(np.where(stat>4)[0]))
    # height=np.mean(stat[stat>4])
    if len(np.where(stat > 4)[0]) > 0:
        # height=np.mean(np.where(stat>4)[0])
        height = np.argmax(stat)
    else:
        height = -1
    for s in range(len(x1)):
        if abs(y1[s] - height) < 5 and stat[y1[s]] > 4:
            xe.append(x1[s])
            ye.append(y1[s])
        else:
            edges1[y1[s], x1[s]] = 0
    return xe, ye, edges1


# 求两点间的距离
def distanceofpint(point0, point1):
    if point0.shape[0] == 3:# 3D
        return (
            (point0[0] - point1[0]) ** 2
            + (point0[1] - point1[1]) ** 2
            + (point0[2] - point1[2]) ** 2
        ) ** 0.5
    if point0.shape[0] == 2:# 2D
        return ((point0[0] - point1[0]) ** 2 + (point0[1] - point1[1]) ** 2) ** 0.5


# 点到直线的距离，参考网址：https://blog.csdn.net/sinat_29957455/article/details/107490561'''
def get_distance_from_point_to_line(point, line_point1, line_point2):
    # 对于两点坐标为同一点时,返回点与点的距离
    if line_point1 == line_point2:
        point_array = np.array(point)
        point1_array = np.array(line_point1)
        return np.linalg.norm(point_array - point1_array)
    # 计算直线的三个参数
    A = line_point2[1] - line_point1[1]
    B = line_point1[0] - line_point2[0]
    C = (line_point1[1] - line_point2[1]) * line_point1[0] + (
        line_point2[0] - line_point1[0]
    ) * line_point1[1]
    # 根据点到直线的距离公式计算距离
    distance = np.abs(A * point[0] + B * point[1] + C) / (np.sqrt(A**2 + B**2))
    return distance


# 最小二乘法拟合曲线，参考网址：https://blog.csdn.net/m0_38128647/article/details/75689228'''
def Least_squares(x, y):# x,y为列表
    x_ = x.mean()
    y_ = y.mean()
    m = 0
    n = 0
    k = 0
    p = 0
    for i in np.arange(0, len(x)):
        k = (x[i] - x_) * (y[i] - y_)
        m += k
        p = np.square(x[i] - x_)
        n = n + p
    a = m / n
    b = y_ - a * x_
    return a, b #系数


def getPoint(cx, cy, r, stx, sty, edx, edy):
    k = (edy - sty) / (edx - stx)
    b = edy - k * edx
    c = cx * cx + (b - cy) * (b - cy) - r * r
    a = 1 + k * k
    b1 = 2 * cx - 2 * k * (b - cy)
    if b1 * b1 - 4 * a * c < 0:
        return -1, -1
    tmp = math.sqrt(b1 * b1 - 4 * a * c) # 求根公式 delta
    x1 = (b1 + tmp) / (2 * a)
    y1 = k * x1 + b
    x2 = (b1 - tmp) / (2 * a)
    y2 = k * x2 + b
    #得到两点(x1,y1),(x2,y2)

    # 判断求出的点是否在圆上
    # (cx,cy)为圆心坐标；r:圆的半径
    res1 = (x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy) #距离平方
    res2 = (x2 - cx) * (x2 - cx) + (y2 - cy) * (y2 - cy)
    p_cirleline1 = np.array(([0, 0])) # 一个列表，储存坐标类型的数据
    '''
    np.array() 函数用于将输入数据转换为 NumPy 数组。
    ([0, 0]) 表示一个包含两个元素的列表，元素分别为 0 和 0。这是一个一维的数组。
    np.array(([0, 0])) 将这个一维数组转换为一个 NumPy 数组，即创建一个二维数组。在这个例子中，由于只有一个元素，因此创建的数组的形状为 (1, 2)一行两列
    '''
    p_cirleline2 = np.array(([0, 0]))
    if r * r - 0.01 < res1 < r * r + 0.01: #满足条件则认为点在圆周上
        p_cirleline1[0] = x1
        p_cirleline1[1] = y1
    if r * r - 0.01 < res2 < r * r + 0.01:
        p_cirleline2[0] = x2
        p_cirleline2[1] = y2
    return p_cirleline1, p_cirleline2


def angle(vec1, vec2):# 传入两个向量
    _angle = np.arctan2(np.abs(np.cross(vec1, vec2)), np.dot(vec1, vec2))
    deg_angle = np.rad2deg(_angle)
    return deg_angle


def angle_rot(vec1, vec2):
    if vec2[1] < 0:
        _angle = np.arctan2(np.abs(np.cross(vec1, vec2)), np.dot(vec1, vec2))
    else:
        _angle = 2 * np.pi - np.arctan2(
            np.abs(np.cross(vec1, vec2)), np.dot(vec1, vec2)
        )
    deg_angle = np.rad2deg(_angle)
    return deg_angle


def isRotationMatrix(R): # 判断是否为（近似的）旋转矩阵
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# function to calculate rotation matrix to euler angles
# 将旋转矩阵转换为欧拉角（绕各轴旋转的角度）
def rotationMatrixToEulerAngles(R):
    assert isRotationMatrix(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6 # 是奇异矩阵则返回true

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def detect(data1, data2, data3, data4):  # 回调函数，data1～4对应Full_Data中同步时间戳的四个对象
    Pos_Now = np.array([data2.pose.pose.position.x, data2.pose.pose.position.y])
    timestamp = data1.header.stamp.secs + data1.header.stamp.nsecs / 1000000000
    force = data3.wrench.force
    torque = data3.wrench.torque
    points = pc2.read_points(data4, skip_nans=True)
    
    if is_print:  # 测试用
        print("force: ", force)
        print("wrench: ", torque)
    # return

    # global vecHorizon
    # global vecLast
    # global angle_diff
    # optical flow params
    # global feature_params
    # global lk_params
    global veloAvg
    global angle_last
    global wheel_theta_vec0
    global body_dist_vec0
    global stereo_height_px
    global image_last
    global slippage
    global workbook
    global worksheet
    global row
    global lm1
    global lm2
    global LastTime
    global tracks
    global detect_count

    if timestamp < LastTime: # ？？
        print("1111---omit...")
        return

    start = time.time()
    
    # 点云处理
    p = np.array([point[:3] for point in points])
    matrix_tilt = np.transpose(p)
    angle = np.pi/3
    rotation_matrix = np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])
    matrix_upright = np.matmul(rotation_matrix, matrix_tilt) # 包含真实的(x,y,z)
    condition = (matrix_upright[0] < 0) & (matrix_upright[0] > -0.058) & (matrix_upright[1] > -0.35) & (matrix_upright[1] < -0.263) & (matrix_upright[2] > 0.13) & (matrix_upright[2] < 0.22)
    a = matrix_upright[:,condition][2]
    depth = np.mean(a)
    if(force.x<1 and force.x>-1):
        depth0 = depth
    sinkage = depth0 - depth
    
    # 将 ROS（Robot Operating System）消息中的图像数据转换为 OpenCV 中的图像格式
    # 第二个参数 "8UC1" 指定转换后的图像格式为 8 位无符号单通道图像（即灰度图像）
    raw_img = CvBridge().imgmsg_to_cv2(data1, "8UC1")
    
    if is_print:
        print("########## image detect come... #############")
        print("start time: ", start)
        print("POS_NOW", np.linalg.norm(Pos_Now))
    src = np.asarray(raw_img)
    color_image = cv2.remap(src=src, map1=lm1, map2=lm2, interpolation=cv2.INTER_LINEAR) #重映射
    # lim=cv2.cvtColor(undistort[:,max_disp:], cv2.COLOR_GRAY2RGB)
    # cv2.imshow("left", color_image)
    # cv2.imshow("right", rim)

    color_image = cv2.transpose(color_image) # 图像转置
    color_image = cv2.flip(color_image, 0) # 图像上下翻转
    vecHorizon = np.array([1, 0])
    # cv2.imshow("color_image", color_image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # input_pose = pose
    # print("Position: {}".format(input_pose.translation))
    # print("Velocity: {}".format(input_pose.velocity))
    # # print("Acceleration: {}\n".format(input_pose.acceleration))
    # w = input_pose.rotation.w
    # x = -input_pose.rotation.z
    # y = input_pose.rotation.x
    # z = -input_pose.rotation.y
    # pitch =  -math.asin(2.0 * (x*z - w*y)) * 180.0 / math.pi;
    # roll  =  math.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / math.pi;
    # yaw   =  math.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / math.pi;
    # print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))

    """0.-exposure"""
    # # TODO
    ######################################

    """0.-Area split"""
    # t1=time.time()
    
    # 根据 stereo_height_px（可能是分辨率？）变量的值，设置不同的相机内参、畸变系数和图像裁剪区域，以便后续处理
    if stereo_height_px == 1024:
        mtx = np.array(
            [[513.716595, 0, 509.761300], [0, 513.716595, 510.650140], [0, 0, 1.0]]
        )  # 相机内参
        dist = np.array(
            [0.004410, -0.001790, -0.000712, -0.000941, 0.000000], dtype=np.double
        )  # Distortion coefficient: k1, k2, p1, p2, k3
        ConfinedSinkImg = color_image[550:850, 0 : color_image.shape[1]]
        # ConfinedSlipImg=color_image[780:860,280:440]              #Q 40 pixel for square
        # cv2.imwrite('/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/gen/'+str(row)+'in.png',ConfinedSlipImg)
        # ConfinedSlipImg=color_image[780:820,280:440]           #B
        # ConfinedSlipImg=color_image[780:830,240:320]           #Ori
        ConfinedSlipImg = color_image[840:880, 300:460]
    elif stereo_height_px == 800:
        mtx = np.array(
            [[402.443415, 0, 398.135578], [0, 402.503472, 399.215228], [0, 0, 1.0]]
        )  # 相机内参
        dist = np.array(
            [0.003403, -0.000676, -0.001002, -0.000081, 0.000000], dtype=np.double
        )  # Distortion coefficient: k1, k2, p1, p2, k3
        ConfinedSinkImg = color_image[400:620, 0 : color_image.shape[1]]
        ConfinedSlipImg = color_image[600:620, 200:400]
    elif stereo_height_px == 300:
        mtx = np.array(
            [[150.755767, 0, 149.255243], [0, 150.691861, 149.582182], [0, 0, 1.0]]
        )  # 相机内参
        dist = np.array(
            [0.003074, -0.000409, 0.000168, 0.000093, 0.000000], dtype=np.double
        )  # Distortion coefficient: k1, k2, p1, p2, k3
        ConfinedSinkImg = color_image[110:210, 0 : color_image.shape[1]]
        ConfinedSlipImg = color_image[
            210 : color_image.shape[0], 0 : color_image.shape[1]
        ]
    # # print("ConfinedSinkImg: ",ConfinedSinkImg.shape[0], ConfinedSinkImg.shape[1])
    # # print("ConfinedSlipImg: ",ConfinedSlipImg.shape[0], ConfinedSlipImg.shape[1])
    # cv2.imwrite('/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/gen/'+str(row)+'in.png',ConfinedSinkImg);
    # cv2.imshow("ConfinedSlipImg", ConfinedSlipImg)
    # cv2.waitKey(0)
    # t2=time.time()
    # print('ConfinedImg_time',t2-t1)
    
    # 下面代码使用 ArUco 库来检测给定图像 ConfinedSinkImg 中的 ArUco 标记。通过选择特定的 ArUco 字典和指定检测器参数
    # 函数返回检测到的标记的角点坐标、ID 和未被检测为标记的图像点。（检测Aprril_tag的四角，用于计算轮子转速？）
    aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
    parameters = aruco.DetectorParameters_create()
    corner, ids, rejectedImgPoints = aruco.detectMarkers(
        ConfinedSinkImg, aruco_dict, parameters=parameters
    )
    
    if ids is not None and corner is not None:
        if is_print:
            detect_count += 1
            print("detect: ", detect_count)
        # print(ids)
        # print(corner)
        # return
        ret = aruco.estimatePoseSingleMarkers(corner[0], 0.03, mtx, dist)
        rvecs, tvecs = ret[0][0, 0, :], ret[1][0, 0, :] #得到旋转向量 和 平移向量
        # rvecs, tvecs, _= aruco.estimatePoseSingleMarkers(corner[0], 0.03, mtx, dist)
        # rvecs = rvecs.reshape(3,1)
        # tvecs = tvecs.reshape(3,1)
        # print("tvecs: ", tvecs)
        # print("rvecs: ", rvecs)
        # aruco.drawAxis(color_image, mtx, dist, rvecs, tvecs, 0.1) #绘制轴
        
        aruco.drawDetectedMarkers(ConfinedSinkImg, corner, ids)  # 在标记周围画一个正方形
        '''
        函数将在图像上绘制一个正方形框，围绕检测到的每个 ArUco 标记。这样可以可视化标记的位置和边界，以便在后续处理中进行参考和分析
        注意：此操作是可选的，仅用于可视化和调试目的，并不影响后续的姿态估计等任务的计算和结果。
        Was this response better or worse?

        '''
        
        # cv2.imshow('detect_marker',ConfinedSinkImg)
        # cv2.waitKey(0)
        # (roll_angle, pitch_angle, yaw_angle) =  rvecs[0]*180/math.pi, rvecs[1]*180/math.pi, rvecs[2]*180/math.pi
        # print("roll_angle: ", roll_angle)
        # print("pitch_angle: ", pitch_angle)
        # print("yaw_angle: ", yaw_angle)

        # xm = tvecs[0]
        # ym = tvecs[1]
        # zm = tvecs[2]
        # str_position = "Marker pos [cm] x=%f  y=%f  z=%f" % (xm, ym, zm)
        R_flip = np.zeros((3, 3), dtype=np.float32) #3*3数组，装float32类型
        R_flip[0, 0] = 1.0
        R_flip[1, 1] = -1.0
        R_flip[2, 2] = -1.0
        R_ct = np.matrix(cv2.Rodrigues(rvecs)[0])
        R_tc = R_ct.T
        #将翻转矩阵 R_flip 与转置矩阵 R_tc 相乘，得到合并的旋转矩阵。然后使用 rotationMatrixToEulerAngles() 函数将合并的旋转矩阵转换为欧拉角表示
        x, y, z = rotationMatrixToEulerAngles(R_flip * R_tc)
        #得到滚转角、俯仰角和偏航角
        roll = math.degrees(x)
        pitch = math.degrees(y)
        yaw = math.degrees(z)
        # str_position += " |  Marker angle roll=%f  pitch=%f  yaw=%f" % (roll, pitch, yaw)
        # print(str_position)

        ################################
        # 处理 ArUco 标记的角点坐标，并将其转换为齐次坐标形式（在x，y坐标后统一添加1，转化为三维坐标）
        corners = np.array(corner).squeeze()
        if corners[1].shape == (2,): # 确保仅包含x，y坐标
            pc1 = np.append(corners[1], 1).reshape(3, 1)
            pc2 = np.append(corners[2], 1).reshape(3, 1)
            pc3 = np.append(corners[0], 1).reshape(3, 1)
            pc4 = np.append(corners[3], 1).reshape(3, 1)

        # '''0.-Perspective Transform'''
        # px1,px2,px3,px4=pc1[0:2],pc2[0:2],pc3[0:2],pc4[0:2]
        # tf1=[px1[0],px1[1]]
        # tf2=[px2[0],px1[1]+px4[1]-px3[1]]
        # tf3=[px1[0]+px4[0]-px2[0],px3[1]]
        # tf4=[px4[0],px4[1]]

        # src=np.array([px1,px2,px3,px4],dtype='float32')
        # dst=np.array([tf1,tf2,tf3,tf4],dtype='float32')
        # M=cv2.getPerspectiveTransform(src,dst)
        # transImg=cv2.warpPerspective(ConfinedSinkImg,M,(ConfinedSinkImg.shape[1],ConfinedSinkImg.shape[0]))
        # cv2.imshow('trans',transImg)

        img_gray = ConfinedSinkImg.copy()
        img_flow_gray = ConfinedSlipImg.copy()
        # print("img_flow_gray: ", img_flow_gray.shape)

        """1.1-求解圆心，使用opencv arcuo 算法"""
        diameter = (
            (distanceofpint(pc1, pc2) + distanceofpint(pc3, pc4)) / 2 / 0.032 * 0.15
        )  # 图像中的轮子？0.15/0.032像素转真实，那么应该是真实的tag边长？？
        radius = diameter / 2  # 图像中的轮子半径？应该是真实的1/2tag边长？？
        center = np.array( # 图像中的圆心坐标
            [
                (corners[0][0] + corners[2][0] + corners[1][0] + corners[3][0]) / 4,
                (corners[0][1] + corners[2][1] + corners[1][1] + corners[3][1]) / 4,
            ]
        )
        # 图像中的圆心到角点距离？
        DST1 = (
            distanceofpint(corners[0], center)
            + distanceofpint(corners[1], center)
            + distanceofpint(corners[2], center)
            + distanceofpint(corners[3], center)
        ) / 4
        DST = (distanceofpint(pc1, pc2) + distanceofpint(pc3, pc4)) / 2 #图像中的tag边长
        # print("diameter: ", diameter)
        if is_print:
            print("radius: ", radius)
            print("center: ", center[0], center[1])
            print("DST1: ", DST1)
        cv2.circle(
            ConfinedSinkImg, tuple(center.astype(int)), int(radius), (0, 255, 0), 2
        )
        cv2.circle(ConfinedSinkImg, tuple(center.astype(int)), 2, (0, 0, 255), 3)
        # img_flow_gray = img_flow_gray[int(center[1]+80):int(center[1]+80)+30,:]
        # cv2.imshow('circle',ConfinedSinkImg)
        # cv2.waitKey(0)

        bool_sink = True
        if bool_sink:
            #######################################
            """2.使用稳健回归算法来拟合沉陷曲线"""
            # [int(center[1]):int(center[1]+radius), int(center[0]-radius*2):int(center[0]+radius*2)]
            # confinedImg=color_image[int(center[1]-radius*2):int(center[1]+radius*2),int(center[0]-radius*2):int(center[0]+radius*2)]
            # out_img = img_I[int(center[1]):int(center[1]+radius*1.1), int(center[0]-radius*1.1):int(center[0]+radius*1.1)]
            # img_test = cv2.cvtColor(ConfinedSinkImg,cv2.COLOR_RGB2GRAY)
            out_img = img_gray[
                int(center[1]) : int(center[1] + radius * 1.1),
                int(center[0] - radius * 1.1) : int(center[0] + radius * 1.1),
            ]
            confinedImg = img_gray[
                int(center[1]) : int(center[1] + radius * 1.1),
                int(center[0] - radius * 1.1) : int(center[0] + radius * 1.1),
            ].copy()
            # print("confinedImgSize",img_gray.shape)
            # cv2.imshow('confinedImg',confinedImg)
            # cv2.imshow('confinedImg1',out_img)

            # (thresh2, im_bw2) = cv2.threshold(img_S,200, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            # # cv2.imshow('otsu2', im_bw2)
            img_blur = cv2.GaussianBlur(
                confinedImg, (9, 9), 0
            )  # Gaussian Filter process
            # cv2.imshow('img_blur',img_blur)
            edges = cv2.Canny(img_blur, 30, 45)
            # edges=cv2.Canny(img_blur,10,70)
            edges[edges < 10] = 0
            edges[edges >= 10] = 120
            # cv2.imshow('edges_init',edges)
            # kernel = np.ones((5,5), np.uint8)
            # img_Erode = cv2.erode(edges, kernel, iterations=3)
            # cv2.imshow('img_Erode',img_Erode)
            # img_Dilation = cv2.dilate(edges, kernel, iterations=2)
            # cv2.imshow('img_Dilation',img_Dilation)

            # startx2=time.time()
            centerx = [edges.shape[0], edges.shape[1]]
            vet = np.array([centerx[1] / 2, 0])
            if is_print:
                print("centerx", centerx)
            i11 = int(centerx[1] * 0.5 - radius)
            i22 = int(centerx[1] * 0.5 + radius)
            j11 = int(DST1 * 1.5)
            j22 = int(radius)
            threhold = (radius * 0.99) ** 2
            x, y, edges = EdgeLine_seg(vet, threhold, i11, i22, j11, j22, edges)
            edges[edges < 130] = 0
            # cv2.imshow('edges_final',edges)
            # cv2.waitKey(0)
            mask_mat = cv2.addWeighted(out_img, 0.5, edges, 0.5, 0.0)
            # cv2.imshow('add_img', mask_mat)
            # endx2=time.time()
            # print('time for edge detect and serach: ',endx2-startx2)

            #######################################
            x = np.array(x)
            y = np.array(y)
            # if x.shape[0]==0:
            #     return
            # ///////////////////// 此处使用了最小二乘法，但发现偏差过大
            # aa,bb = Least_squares(x,y)
            # yy = aa * x + bb
            # # print('Least Square:',aa,bb,min(x))
            # if math.isnan(aa) or math.isnan(bb):
            #     return
            # p111 = np.array([0,0])
            # p222 = np.array([0,0])
            # p111[0] = min(x)
            # p111[1] = aa*min(x)+bb
            # p222[0] = max(x)
            # p222[1] = aa*max(x)+bb
            # cv2.line(mask_mat, (p111[0], p111[1]),(p222[0],p222[1]), (0,0,255), 5)
            # # imshow("line1", mask_mat)

            # /////////////////////  RANSAC
            model_ransac = linear_model.RANSACRegressor(
                linear_model.LinearRegression(),
                max_trials=100,  # 最大迭代次数
                min_samples=5,  # 最小抽取的内点样本数量
            )
            if x.shape[0] > 5:
                try:
                    model_ransac.fit(x.reshape(-1, 1), y)
                except:
                    return
            if hasattr(model_ransac, "estimator_") == False:
                print("exit1")
                return
            a = model_ransac.estimator_.coef_  # 直线斜率
            b = model_ransac.estimator_.intercept_  # 截距
            y2 = a * x + b
            p10 = np.array((0, 0))
            p20 = np.array((0, 0))
            p10[0] = min(x)
            p10[1] = a * min(x) + b
            p20[0] = max(x)
            p20[1] = a * max(x) + b
            cv2.line(mask_mat, (p10[0], p10[1]), (p20[0], p20[1]), (255, 0, 0), 5)
            # imshow("line_ransac", mask_mat)

            """3. 求出沉陷量"""
            # distance = get_distance_from_point_to_line(vet, tuple(p10), tuple(p20))
            # sink = 0.0165 * 2**0.5 / DST1 * (radius - distance) #0.0165 * 2**0.5 / DST1：像素转真实
            # sinkage = (radius - distance) / radius * (0.15 / 2) #/ radius * (0.15 / 2)：像素转真实
            # sinkage = depth0 - depth1
            # #两种计算沉陷的方法：参考的转换系不同，一个依赖轮子半径，一个依赖tag边长
            # if is_print:
            #     print("沉陷量", sink)
            #     print("沉陷量2", sinkage)

            """3.求进入角和退出角"""
            cross_point1 = np.array([0.0, 0.0])
            cross_point2 = np.array([0.0, 0.0])
            cross_point = np.array([0.0, 0.0])
            cross_point[0] = vet[0]
            # cross_point1,cross_point2 = getPoint(vet[0], vet[1], radius, p111[0], p111[1], p222[0], p222[1])
            # cross_point[1] = aa*vet[0] + bb
            cross_point1, cross_point2 = getPoint(
                vet[0], vet[1], radius, p10[0], p10[1], p20[0], p20[1]
            )
            cross_point[1] = a * vet[0] + b
            vec1 = vet - cross_point1
            vec2 = vet - cross_point2
            vec3 = vet - cross_point
            angle1 = angle(vec1, vec3)
            angle2 = angle(vec2, vec3)
            if is_print:
                print("left_angle", angle1)
                print("right_angle", angle2)

            # show in limited iamge
            cv2.line(
                mask_mat,
                tuple(cross_point1.astype(int)),
                tuple(vet.astype(int)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                mask_mat,
                tuple(cross_point2.astype(int)),
                tuple(vet.astype(int)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                mask_mat,
                tuple(cross_point1.astype(int)),
                tuple(cross_point2.astype(int)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                mask_mat,
                tuple(vet.astype(int)),
                (int(vet[0]), int(radius)),
                (255, 0, 0),
                1,
            )
            # imshow("sinkage_final", mask_mat)
            # cv2.waitKey(0)

            # show in only single image
            cp1 = cross_point1 + np.array(
                [int(center[0] - radius * 1.1), int(center[1])]
            )
            cp2 = cross_point2 + np.array(
                [int(center[0] - radius * 1.1), int(center[1])]
            )
            vetx = center
            cv2.line(
                ConfinedSinkImg,
                tuple(cp1.astype(int)),
                tuple(vetx.astype(int)),
                (255, 0, 0),
                2,
            )
            cv2.line(
                ConfinedSinkImg,
                tuple(cp2.astype(int)),
                tuple(vetx.astype(int)),
                (255, 0, 0),
                2,
            )
            # cv2.line(ConfinedSinkImg, tuple(cp1.astype(int)), tuple(cp2.astype(int)), (255, 0, 0), 2)
            cv2.line(
                ConfinedSinkImg,
                tuple(vetx.astype(int)),
                (int(vetx[0]), 2 * int(radius)),
                (255, 0, 0),
                1,
            )
            # imshow("sinkage_final_X",ConfinedSinkImg)
            cv2.imwrite("reslult.png", ConfinedSinkImg)
            # print(cp1,cp2,cross_point1,cross_point2,vet,center)

        bool_slippage = True
        if bool_slippage:
            if is_print:
                print("calculate slippage")
            bool_slip_real_data = False
            if bool_slip_real_data:
                if is_print:
                    print("slip from real sensor data...")
                body_dist_vec1 = Pos_Now  # read apriltag's pose

                # read point to get angle directly form aruco centre yaw
                wheel_theta_vec1 = np.deg2rad(yaw)
                radius_data = 0.15 / 2
                if abs(wheel_theta_vec1 - wheel_theta_vec0) > 1:
                    if wheel_theta_vec0 < 0:
                        wheel_theta_vec0 += np.pi * 2
                    else:
                        wheel_theta_vec0 -= np.pi * 2

                # read point to get angle from four corners
                vecCor = (corners - center).reshape(4, 2)
                yaw_angle = np.array(
                    [
                        angle_rot(vecHorizon, vecCor[0]),
                        angle_rot(vecHorizon, vecCor[1]),
                        angle_rot(vecHorizon, vecCor[2]),
                        angle_rot(vecHorizon, vecCor[3]),
                    ]
                )
                angle_now = np.deg2rad(yaw_angle)
                if is_print:
                    print("Angle", yaw_angle)

            else:
                if is_print:
                    print("slip from pixel data...")
                # - improve angle calculate with more line and angle
                # The vector of corner0 to horizon now
                radius_data = radius
                vecCor = (corners - center).reshape(4, 2)
                # print(vecCor)
                # print(vecCor.reshape(4,2))
                yaw_angle = np.array(
                    [
                        angle_rot(vecHorizon, vecCor[0]),
                        angle_rot(vecHorizon, vecCor[1]),
                        angle_rot(vecHorizon, vecCor[2]),
                        angle_rot(vecHorizon, vecCor[3]),
                    ]
                )
                if is_print:
                    print("Angle", yaw_angle)
                angle_now = np.deg2rad(yaw_angle)

                # veloAvg=0
                if image_last.shape[0] == 2:
                    image_last = img_flow_gray.copy()
                    angle_last = np.deg2rad(yaw_angle)
                    return

                # /////////////////////////
                # sparse optical flow
                # optical flow params
                # feature_params=dict(maxCorners=80,qualityLevel=0.15,minDistance=10,blockSize=3)
                # lk_params=dict(winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,10,0.03))
                # feature_params=dict(maxCorners=100,qualityLevel=0.15,minDistance=7,blockSize=7)
                # lk_params=dict(winSize=(9,9),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,10,0.03))#Speed up, Win Size up
                # roi=np.zeros_like(img_flow_gray)+255
                # pLast=cv2.goodFeaturesToTrack(image_last,mask=roi,**feature_params)
                # color = np.random.randint(0, 255, (100, 3))
                # mask = np.zeros_like(image_last) #创建一个mask, 用于进行横线的绘制
                # try:
                #     pNow,st,err=cv2.calcOpticalFlowPyrLK(image_last,img_flow_gray,pLast,None,**lk_params)
                # except:
                #     return
                # goodNew=pNow[st==1] #get good points to track
                # goodOld=pLast[st==1]
                # veloAvg=np.mean(np.linalg.norm(goodNew-goodOld,axis=1))
                # print('velocity',veloAvg)
                # pLast = goodNew.reshape(-1, 1, 2)

                # Opti_LK:Lucas-Kanade 方法计算稀疏光流 + track：追踪
                feature_params=dict(maxCorners=100,qualityLevel=0.3,minDistance=7,blockSize=7)
                lk_params=dict(winSize=(15,15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,10,0.03))
                if len(tracks) > 0:
                    img0, img1 = image_last, img_flow_gray
                    p0 = np.float32([tr[-1] for tr in tracks]).reshape(-1, 1, 2) #-1表示根据其他维度自动计算该维度的长度
                    #提取特征点坐标
                    p1, _st, _err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params) #得到img1中的特征点坐标
                    p0r, _st, _err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params) #又由img1倒推img0中的特征点坐标？？
                    d = abs(p0 - p0r).reshape(-1, 2).max(-1) #在img0中的误差
                    good = d < 1
                    new_tracks = []
                    for tr, (x, y), good_flag in zip(tracks, p1.reshape(-1, 2), good):
                        if not good_flag:
                            continue
                        tr.append((x, y)) #tr是一个装有坐标的列表
                        if len(tr) > 10:
                            del tr[0]
                        new_tracks.append(tr) #new_tracks容纳tr
                        cv2.circle(ConfinedSlipImg, (int(x), int(y)), 2, (0, 255, 0), -1)
                    tracks = new_tracks
                    veloAvg=np.mean(np.linalg.norm(p0 - p0r,axis=1)) #why？

                #???### if frame_idx % 30 == 0:
                mask = np.zeros_like(img_flow_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(img_flow_gray, mask=mask, **feature_params)
                if p is not None:
                    for x, y in np.float32(p).reshape(-1, 2):
                        tracks.append([(x, y)])

                # # DENSE 稠密光流
                # flow = cv2.calcOpticalFlowFarneback(
                #     image_last,
                #     img_flow_gray,
                #     None,
                #     0.5,
                #     4,
                #     60,
                #     3,
                #     7,
                #     0.8,
                #     cv2.OPTFLOW_FARNEBACK_GAUSSIAN,
                # )  # Win Size increase with speed 0.4 for positive 0.4 and 0.5 using 25
                # flowVal = np.linalg.norm(flow, axis=2)  # 求二范数:平方求和再开根号，指定轴向为2 为何取范数？
                # # flowVal=flowVal[:,30:flowVal.shape[1]-30]
                # veloAvg = np.round(np.mean(flowVal), 2)
                # tm = np.around(flowVal, decimals=3)
                # np.savetxt('/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/gen/'+str(row)+'in.txt',tm)
                # cv2.imwrite('/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/gen/'+str(row)+'in.png',img_flow_gray)

                # #DualTVl
                # dual_proc = cv2.optflow.DualTVL1OpticalFlow_create()
                # est_flow = np.zeros(shape=(image_last.shape[0], image_last.shape[1],2), dtype=np.float32)
                # dual_proc.calc(image_last,img_flow_gray, est_flow)
                # flowVal=np.linalg.norm(est_flow,axis=2)
                # veloAvg=np.round(np.mean(flowVal),2)

            if bool_slip_real_data:
                # s_body = np.round(abs(np.linalg.norm(body_dist_vec1) - np.linalg.norm(body_dist_vec0)),5)
                s_body = np.round(
                    np.linalg.norm(body_dist_vec1 - body_dist_vec0), 5
                )  # 为何是5-范数？？

                # s_wheel = np.round(abs(wheel_theta_vec1 - wheel_theta_vec0),3)*radius_data
                angle_diff = angle_now - angle_last
                for i in range(4):
                    if abs(angle_diff[i]) > 1:
                        angle_diff[i] = (-abs(angle_diff[i])) % (np.pi * 2)
                # ???
                angle_diff = abs(angle_diff)
                if is_print:
                    print("diff", abs(angle_diff))
                # angle_diff=sorted(abs(angle_diff))
                s_wheel = np.round(np.mean(angle_diff[0:4]), 3) * radius_data
            else:
                s_body = veloAvg

                angle_diff = angle_now - angle_last
                for i in range(4):
                    if abs(angle_diff[i]) > 1:
                        angle_diff[i] = (-abs(angle_diff[i])) % (np.pi * 2)
                s_wheel = np.round(abs(np.mean(angle_diff)), 3) * radius_data

            if is_print:
                print("s_body: ", s_body)
                print("s_wheel: ", s_wheel)
            if s_wheel == 0 or s_body == 0:
                slippage = 0
            elif s_wheel >= s_body:
                slippage = (float)(s_wheel - s_body) / (s_wheel)
            else:
                slippage = (float)(s_wheel - s_body) / (s_body)

            if slippage > 1:
                slippage = 1
            elif slippage < -1:
                slippage = -1
            if is_print:
                print("slippage: ", slippage)
            # print("wheel_theta_vec0: ", wheel_theta_vec0)
            # print("wheel_theta_vec1: ", wheel_theta_vec1)
            # print("body_dist_vec0: ", body_dist_vec0)
            # print("body_dist_vec1: ", body_dist_vec1)
            # cv2.imwrite('/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/gen/'+str(row)+'in.png',img_flow_gray)
        end = time.time()
        if s_wheel != 0:
            if row >= 200:  # 200组数据
                workbook.close()
            else:
                row += 1
                worksheet.write(row, 0, timestamp)
                worksheet.write(row, 1, radius)
                worksheet.write(row, 2, center[0])
                worksheet.write(row, 3, center[1])
                worksheet.write(row, 4, depth0)
                worksheet.write(row, 5, sinkage * 1000)
                worksheet.write(row, 6, angle1)
                worksheet.write(row, 7, angle2)
                if np.isnan(s_body) == False:
                    worksheet.write(row, 8, s_body)
                worksheet.write(row, 9, np.linalg.norm(Pos_Now))
                worksheet.write(row, 10, s_wheel / radius_data)
                
                
                # worksheet.write(row, 11,yaw)
                # worksheet.write(row, 12,tvecs[1])
                # worksheet.write(row, 13,angle_now[0])
                # worksheet.write(row, 14,angle_now[1])
                # worksheet.write(row, 15,angle_now[2])
                # worksheet.write(row, 16,angle_now[3])
                # if np.isnan(slippage)==False:
                worksheet.write(row, 17, 1 / (end - start))
                worksheet.write(row, 18, force.x)
                worksheet.write(row, 19, force.y)
                worksheet.write(row, 20, force.z)
                worksheet.write(row, 21, torque.x)
                worksheet.write(row, 22, torque.y)
                worksheet.write(row, 23, torque.z)

            print("row: ", row)
            if bool_slip_real_data:
                wheel_theta_vec0 = np.deg2rad(yaw)
                angle_last = np.deg2rad(yaw_angle)
                body_dist_vec0 = Pos_Now
            else:
                angle_last = np.deg2rad(yaw_angle)
                body_dist_vec0 = veloAvg
                image_last = img_flow_gray.copy()
            LastTime = timestamp

        print("all_detect_time", 1 / (end - start))
        print("-------------------------------")


# def rostest():
if __name__ == "__main__":
    rospy.init_node("Measurement", anonymous=True)

    # vecHorizon=np.array([1,0])
    vecLast = np.array([0, 0])
    angle_last = np.array([0, 0, 0, 0])
    wheel_theta_vec0 = 0
    body_dist_vec0 = np.array([0, 0])
    image_last = np.array([0, 0])
    veloAvg = -10
    angle_diff = np.array([-10, -10, -10, -10])
    slippage = -10
    LastTime = 0
    tracks = []

    # 写入表格，把第四个订阅方的数据写入workbook需要在这里实现
    workbook = xlsxwriter.Workbook(
        "/home/roma/rover/rover_ws/src/rover_camera/src/rover_camera/test_t265.xlsx",
        {"constant_memory": True},
    )
    worksheet = workbook.add_worksheet("sheet1")
    row = 0
    worksheet.write(row, 0, "time_label")
    worksheet.write(row, 1, "radius/pixel")
    worksheet.write(row, 2, "center_X")
    worksheet.write(row, 3, "center_Y")
    worksheet.write(row, 4, "depth0/mm")
    worksheet.write(row, 5, "sinkage/mm")
    worksheet.write(row, 6, "Left_Angle/deg")
    worksheet.write(row, 7, "Right_Angle/deg")
    worksheet.write(row, 8, "VeloAvg/rad")
    worksheet.write(row, 9, "t265_trans")
    worksheet.write(row, 10, "angle_diff/rad")
    
    
    # worksheet.write(row, 11, 'tag_rot')
    # worksheet.write(row, 12, 'tag_trans')
    worksheet.write(row, 17, "freq")
    worksheet.write(row, 18, "fx")
    worksheet.write(row, 19, "fy")
    worksheet.write(row, 20, "fz")
    worksheet.write(row, 21, "mx")
    worksheet.write(row, 22, "my")
    worksheet.write(row, 23, "mz")

    # Undistort Params 畸变修正矩阵
    K = np.array(
        [
            286.1329040527344,
            0.0,
            427.32501220703125,
            0.0,
            286.0943908691406,
            402.7410888671875,
            0.0,
            0.0,
            1.0,
        ]
    ).reshape(3, 3)
    D = np.array(
        [
            -0.008217182010412216,
            0.0459030382335186,
            -0.04332152009010315,
            0.008243677206337452,
        ]
    )
    R = np.eye(3)
    P = np.array(
        [
            286.1329040527344,
            0.0,
            427.32501220703125,
            0.0,
            0.0,
            286.0943908691406,
            402.7410888671875,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
    ).reshape(3, 4)

    min_disp = 0
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo_fov_rad = 90 * (math.pi / 180)  # 90 degree desired fov
    stereo_height_px = 1024  # 300-800-1024
    stereo_focal_px = stereo_height_px / 2 / math.tan(stereo_fov_rad / 2)
    if is_print:
        print("stereo_focal_px: ", stereo_focal_px)
    stereo_cx = (stereo_height_px - 1) / 2 + max_disp
    stereo_cy = (stereo_height_px - 1) / 2

    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, P, stereo_size, cv2.CV_32FC1
    )

    # 订阅
    color = message_filters.Subscriber(
        "/camera/fisheye1/image_raw", Image, queue_size=1
    )
    Pose = message_filters.Subscriber(
        "/camera_odom/odom/sample", Odometry, queue_size=1
    )
    ft = message_filters.Subscriber("/force_sensor/data", WrenchStamped, queue_size=1)
    
    pointcloud = message_filters.Subscriber("/camera/depth/color/points", PointCloud2 , queue_size=1)
    # # 添加第四个订阅方
    # present_velocity_sub = message_filters.Subscriber("present_velocity", Int32, queue_size=1)
    
    Full_Data = message_filters.ApproximateTimeSynchronizer([color, Pose, ft, pointcloud], 1, 0.035) #可能需要更改时间同步的阈值
    # 同步多个订阅方的时间戳
    # 参数1：要同步的订阅方：color、Pose、ft；参数2：queue长度；参数3：slop：允许的时间偏差范围
    Full_Data.registerCallback(detect)  # 指定回调函数
    rospy.spin()

    # try:
    #     rostest()
    # # except rospy.ROSInitException:
    # except rospy.ROSInterruptException:
    #     pass
