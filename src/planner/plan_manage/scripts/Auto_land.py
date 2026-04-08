#!/usr/bin/env python3

#这段Python脚本是一个ROS节点，用于实现视觉伺服控制（Visual Servoing）来控制UR5机械臂的关节速度。以下是对代码的解释和注释：

#添加深度跟随效果的视觉伺服测试：改为对矩形特征或圆特征进行视觉伺服

import rospy
import numpy as np
from math import *
import ctypes
from std_msgs.msg import Float64MultiArray
import time
import cv2
import pyrealsense2 as rs

pub = rospy.Publisher("/auto_land",Float64MultiArray,queue_size=10)

#aruco配置
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
arucoParams = cv2.aruco.DetectorParameters_create()

#realsense配置
resolution = [640,480]
frame_rate = 15  # fps
pipeline = rs.pipeline()  #定义流程pipeline
config = rs.config()   #定义配置config
config.enable_stream(rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

connect_device = []
for d in rs.context().devices:
    print('Found device: ',
            d.get_info(rs.camera_info.name), ' ',
            d.get_info(rs.camera_info.serial_number))
    if d.get_info(rs.camera_info.name).lower() != 'platform camera':
        connect_device.append(d.get_info(rs.camera_info.serial_number))

if len(connect_device) < 2:
    print('Registrition needs two camera connected.But got one.')
    exit()

config.enable_device(connect_device[1])


profile = pipeline.start(config)  #流程开始
align_to = rs.stream.color  #与color流对齐
align = rs.align(align_to)
def get_aligned_rgb_imgs():
    frames = pipeline.wait_for_frames()  #等待获取图像帧
    aligned_frames = align.process(frames)  #获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  #获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()   #获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics   #获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  #获取深度参数（像素坐标系转相机坐标系会用到）
    camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }
    
    depth_rgb_img = np.asanyarray(aligned_depth_frame.get_data())  #深度图（默认16位）
    depth_rgb_img_8bit = cv2.convertScaleAbs(depth_rgb_img, alpha=0.03)  #深度图（8位）
    depth_rgb_img_3d = np.dstack((depth_rgb_img_8bit,depth_rgb_img_8bit,depth_rgb_img_8bit))  #3通道深度图
    color_rgb_img = np.asanyarray(color_frame.get_data())  # RGB图
    
    #返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_rgb_img, depth_rgb_img, aligned_depth_frame

global desired_points


mid_point = [resolution[0]/2,resolution[1]/2]
delta = 40
desired_points =[mid_point[0], mid_point[1], 0.2]
p0_gain = 0.4

##功能:根据画面检测情况计算无人机需要运动的x、y速度，并以话题ros2serial_msg_to_send发送
#参数：points：u、v、depth（u、v用于计算图像误差，depth更新图像雅可比）

def joint_velocities(points):
    if points != None: 
        global desired_points
        fl = 605.9
        # Put your desired points as an array here, from the output of the object_detect.py node.
        # desired_points =[96.40369045377518, -88.57089060440595, 768]
        VisualServoResults = Float64MultiArray()

        desired_points_arr = np.asarray(desired_points) #list to numpy array
        
        points = np.asarray(points) #real-time points
        
        error = []
    
        for index in range(len(points)):
            if (index+1)%3 == 0:#此处仅计算u、v方向误差
                index += 1
            else:
                error.append(points[index] - desired_points_arr[index])
        
        avg_x=0
        avg_y=0
        error = np.asarray(error) #1v6 array
        # print("error is ", error)
        #切片：起始、终点、步长。
        for err_x in error[0:len(error):2]:
            avg_x+=err_x**2
        for err_y in error[1:len(error):2]:
            avg_y+=err_y**2

        mean_error=(avg_x+avg_y)**0.5
        if mean_error<=10:                              ########误差死区阈值########
            error=np.array([0,0])
        
        
        error = np.matrix(error)
        error = np.ndarray.transpose(error)
 
        u1 = points[0]
        v1 = points[1]
        z1 = points[2]
        z1 = round(z1,2)

#####################深度无关控制
        z1 = 1.5

###############################
        
 ############使用基于图像的视觉伺服
        int_matrix = np.array([   [ -fl/z1,  0,    u1/z1],
                                    [    0,  -fl/z1, v1/z1,]
                                    ])

        int_matrix = np.matrix(int_matrix) #6v6 matrix
        J_inverse = np.linalg.pinv(int_matrix)
        cam_vel_np = 0.5*J_inverse*error                ########比例控制器系数########
        cam_vel_np[1] =-cam_vel_np[1]
        for i in range(cam_vel_np.shape[0]):
            # print(cam_vel_np[i-1,0])
            if cam_vel_np[i-1,0] > 0.2:                 ########控制器输出限幅########
                cam_vel_np[i-1,0] = 0.2
            elif cam_vel_np[i-1,0] < -0.2:
                cam_vel_np[i-1,0] = -0.2

        VisualServoResults.data = list(cam_vel_np)
###############################


        pub.publish(VisualServoResults)

        print ("Joint Velocity: ")
        print (cam_vel_np)
        print("\n")
        return cam_vel_np

    
def AruCo_detect(rgb_img):
    (corners, ids, rejected) = cv2.aruco.detectMarkers(rgb_img, arucoDict, parameters=arucoParams)
    # 验证*至少*一个 ArUco 标记被检测到
    cv2.circle(rgb_img, (int(desired_points[0]), int(desired_points[1])), 3, (0, 0, 255), -1)
    if len(corners) > 0:
        # 展平 ArUco ID 列表
        ids = ids.flatten()
        # 循环检测到的 ArUCo 角
        for (markerCorner, markerID) in zip(corners, ids):
            # 提取标记角（始终按左上角、右上角、右下角和左下角顺序返回）
            corners = markerCorner.reshape((4, 2))
            (bottomRight, bottomLeft, topLeft, topRight) = corners
            # 将每个 (x, y) 坐标对转换为整数
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))


            # 绘制ArUCo检测的边界框
            cv2.line(rgb_img, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(rgb_img, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(rgb_img, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(rgb_img, bottomLeft, topLeft, (0, 255, 0), 2)
            mid_x = int((topLeft[0]+bottomRight[0])/2)
            mid_y = int((topLeft[1] + bottomRight[1])/2)

            # cv2.circle(rgb_img, (int(topLeft[0]), int(topLeft[1])), 8, (0, 0, 255), -1)
            cv2.circle(rgb_img, (mid_x, mid_y), 8, (0, 0, 255), -1)

            # 计算并绘制 ArUco 标记的中心 (x, y) 坐标
            cv2.putText(rgb_img, str(5), (mid_x, mid_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # cv2.putText(rgb_img, str(1), (topLeft[0], topLeft[1] - 15),
            #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # corner = [topLeft[0],topLeft[1]]
        corner = [mid_x,mid_y]


        # print("aruco is",corner)

    else:
        corner = None
    cv2.imshow('rgb_img', rgb_img)
    cv2.waitKey(3)
    return corner
    


def vs_ur5():
    ZerosRes = Float64MultiArray()
    rospy.init_node('vs_ur5')
    rate = rospy.Rate(10)  # 设置循环频率为1Hz，可以根据需要调整
    pix_coordinates = []
    res_to_pub = [0, 0, 0, 0, 0, 0, 0]
    j_velocity = []
    t1 = 0
    t2 = 0
    flag_arm1 = False
    flag_arm2 = False
    uav_arm   = False


    while not rospy.is_shutdown():
        intr, depth_intrin, rgb_img, depth_img, aligned_depth_frame = get_aligned_rgb_imgs() #获取对齐的图像与相机内参
        corner = AruCo_detect(rgb_img)
        if corner is not None:
            for i in range(0,2,2):
                corner_dis = aligned_depth_frame.get_distance(int(corner[i]), int(corner[i+1]))#角点深度值
                orin_dis = aligned_depth_frame.get_distance(int(mid_point[0]), int(mid_point[1]))#中心深度值

                x = corner[0]
                y = corner[1]
                camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], corner_dis)#角点真实坐标
                orin_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, mid_point, orin_dis)#中心真实坐标


                #数据保留小数点后两位
                camera_coordinate[0] = round(camera_coordinate[0], 2)  
                camera_coordinate[1] = round(camera_coordinate[1], 2) 
                camera_coordinate[2] = round(camera_coordinate[2], 2) 

                orin_coordinate[0] = round(orin_coordinate[0], 2)
                orin_coordinate[1] = round(orin_coordinate[1], 2)
                orin_coordinate[2] = round(orin_coordinate[2], 2)

                #u、 v、 dis
                pix_coordinates.append(corner[i] )
                pix_coordinates.append(corner[i+1] )
                pix_coordinates.append(corner_dis)


            j_velocity = joint_velocities(pix_coordinates)#1.当前关节角位置，2.u、v、depth
            mean_err = (camera_coordinate[0] - orin_coordinate[0])**2 + (camera_coordinate[1] - orin_coordinate[1])**2 
            mean_err = mean_err**0.5

            res_to_pub[0] = round(float(j_velocity[0]), 2)
            res_to_pub[1] = round(float(j_velocity[1]), 2)
            res_to_pub[2] = round(float(j_velocity[2]), 2)
            res_to_pub[3] = round(float(camera_coordinate[0] - orin_coordinate[0]), 2)
            res_to_pub[4] = round(float(camera_coordinate[1] - orin_coordinate[1]), 2)
            res_to_pub[5] = round(float(orin_coordinate[2]), 2)
            res_to_pub[6] = 0

            #状态机判断是否机身与二维码欧式距离误差小于20cm超过2s
            if mean_err < 0.2 and flag_arm1 == False:
                flag_arm1 = True
                t1 = time.time()
            if flag_arm1 == True:
                if mean_err < 0.2:
                    if time.time() - t1 > 2.5:
                        flag_arm1 = False
                        uav_arm = True
                        print("Allow land")
                        res_to_pub[6] = 1
                else:
                    flag_arm1 = False
            print(res_to_pub)
            print("\n")
            pix_coordinates = []
            res_to_pub = [0, 0, 0, 0, 0, 0, 0]
        else:
            Zeros = [0, 0, 0, 0, 0, 0, 0]
            ZerosRes.data = list(Zeros)
            pub.publish(ZerosRes)


        # rate.sleep()  # 控制循环频率




if __name__ == '__main__':
    try:
        vs_ur5()
    except rospy.ROSInterruptException:
        pass

    
