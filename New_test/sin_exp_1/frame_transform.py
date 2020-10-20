import numpy as np
import os
import re

def PreProcess():
    #joint_path = './selected_joint'
    position_path = './selected_position'
    #joint_file = os.listdir(joint_path)
    position_file = os.listdir(position_path)
    #joint_file.sort(key= lambda x:int(x[:-4]))
    position_file.sort(key= lambda x:int(x[:-4]))
    # if(joint_file == position_file):
    #     print("Read all the files!")
    # else:
    #     print("Files missed!")
    # joints = []
    # for file in joint_file: #遍历文件夹
    #     if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
    #         joint = []
    #         f = open(joint_path+"/"+file); #打开文件
    #         iter_f = iter(f); #创建迭代器
    #         for line in iter_f: #遍历文件，一行行遍历，读取文本
    #             digit = re.findall(r"\d+\.?\d*",line)
    #             for i in digit:
    #                 index = line.index(i)
    #                 if line[index - 1] == '-':
    #                     joint.append(0-float(i))
    #                 else:
    #                     joint.append(float(i))
    #     joints.append(joint)
    positions = []
    orientations = []
    for file in position_file: #遍历文件夹
        if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
            tmp = []
            f = open(position_path+"/"+file); #打开文件
            iter_f = iter(f); #创建迭代器
            position = []
            orientation = []
            for line in iter_f: #遍历文件，一行行遍历，读取文本
                digit = re.findall(r"\d+\.?\d*",line)
                if len(digit) > 0:
                    if line[line.index(digit[0]) + len(digit[0])] == 'e':
                        print("Scientific notation!")
                        print(file)
                    index = line.index(digit[0])
                    if line[index - 1] == '-':
                        if len(position) < 3:
                            position.append(0-float(digit[0]))
                        else:
                            orientation.append(0-float(digit[0]))
                    else:
                        if len(position) < 3:
                            position.append(float(digit[0]))
                        else:
                            orientation.append(float(digit[0]))
        positions.append(position)
        orientations.append(orientation)
    return positions, orientations,position_file

def isRotationMatrix(R):
    # square matrix test
    if R.ndim != 2 or R.shape[0] != R.shape[1]:
        return False
    should_be_identity = np.allclose(R.dot(R.T), np.identity(R.shape[0], np.float))
    should_be_one = np.allclose(np.linalg.det(R), 1)
    return should_be_identity and should_be_one

def quaternion_to_rotation_matrix(quat, pos):
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    rot_matrix = np.array(
        [[1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0], pos[0]],
         [q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0], pos[1]],
         [q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2], pos[2]],
         [0, 0, 0, 1]],
        dtype=q.dtype)
    return rot_matrix

def update_rotaton(privious_rotation, alpha, theta):
    tmp1 = np.array([[1, 0, 0],[0, np.cos(alpha), -np.sin(alpha)],[0, np.sin(alpha), np.cos(alpha)]])
    tmp2 = np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    return np.dot(np.dot(privious_rotation, tmp1), tmp2)

def Camera2Base(position, orientaion, file_name): #Frame Transform from camera to base
    #E -> End_effector
    #C -> Camera
    #B -> Base
    Transform_E2C = quaternion_to_rotation_matrix(np.array(orientaion), np.array(position))
    # print("Transform from E to C:\n", Transform_E2C)
    # print("Is the rotation matrix(End-effector to camera) valid or not:\n", isRotationMatrix(Transform_E2C[0:3,0:3]))
    Transform_C2E = np.linalg.inv(Transform_E2C)
    # print("Transform from C to E:\n", Transform_C2E)
    # print("Is the rotation matrix(Camera to End_effector) valid or not:\n", isRotationMatrix(Transform_C2E[0:3,0:3]))
    #print(Transform_E2C[0:3,0:3])
    # #joint
    # J_1 = joint[0]
    # J_2 = joint[1]
    # J_3 = joint[2]
    # J_4 = joint[3]
    # J_5 = joint[4]
    # J_6 = joint[5]
    # Rotation_base = np.identity(3)
    # position_base = np.zeros(3)
    # #print(Rotation_base)
    # #print(position_base) #base frame
    # #0.0091, 0.4318, 0.4162 are lengths described in the munual book
    # a = np.array([0,0,0,0,0,0.0091])
    # alpha = np.array([np.pi/2,-np.pi/2,np.pi/2,0,-np.pi/2,-np.pi/2])
    # D = np.array([0,0,J_3-0.4318,0.4162,0,0])
    # theta = np.array([J_1+np.pi/2, J_2-np.pi/2,0,J_4,J_5-np.pi/2,J_6-np.pi/2])
    # rotation_ = Rotation_base
    # position_ = position_base
    # for i in range(a.shape[0]):
    #     previous_x = rotation_[0:3,0]
    #     previous_x_dir = np.dot(np.identity(3), previous_x)
    #     rotation_ = update_rotaton(rotation_,alpha[i],theta[i])
    #     next_z = rotation_[0:3,2]
    #     next_z_dir = np.dot(np.identity(3), next_z)
    #     position_ = position_ + a[i] * previous_x_dir + D[i] * next_z_dir
    # Transfrom_E2B = np.ones([4,4])
    # Transfrom_E2B[0:3,0:3] = rotation_
    # Transfrom_E2B[0:3,3] = position_
    # Transfrom_E2B[3,:] = np.array([0,0,0,1])
    clamp_E = np.array([0,0,0,1]) #along the z-axis(instead of y-axis in the guidence book)
    clamp_C = np.dot(Transform_E2C, clamp_E)
    # print("Position of clamp in the camera frame:\n", clamp_C)
    # print("Transform from E to B:\n", Transfrom_E2B)
    # print("Is the rotation matrix(End_effector to base) valid or not:\n", isRotationMatrix(Transfrom_E2B[0:3,0:3]))
    # Transfrom_C2B = np.dot(Transfrom_E2B, Transform_C2E)
    # print("Transform from C to B:\n", Transfrom_C2B)
    # print("Is the rotation matrix(End_effector to base) valid or not:\n", isRotationMatrix(Transfrom_C2B[0:3,0:3]))
    # np.savetxt('./selected_frame_transform/' + file_name + '_Matrix.txt',Transfrom_C2B, fmt='%.07f')
    np.savetxt('./tool_trajectory/' + file_name + '_position.txt', clamp_C, fmt='%.07f')

def main():
    positions, orientations, file_name = PreProcess()
    for index in range(len(positions)):
        # joint = joints[index][:-1]
        orientation = orientations[index]
        orientation.insert(0, orientation.pop())
        camera2base = Camera2Base(positions[index], orientation, file_name[index][0:-4])

if __name__ == '__main__':
    main()
