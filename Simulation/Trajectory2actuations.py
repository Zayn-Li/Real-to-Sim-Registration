import numpy as np
import os

def Read_files():
    position_path = './tool_trajectory'
    #joint_file = os.listdir(joint_path)
    position_file = os.listdir(position_path)
    #joint_file.sort(key= lambda x:int(x[:-4]))
    position_file.sort(key= lambda x:int(x[:x.index('_')]))
    positions = []
    for file in position_file: #遍历文件夹
        if not os.path.isdir(file): #判断是否是文件夹，不是文件夹才打开
            f = open(position_path+"/"+file); #打开文件
            iter_f = iter(f); #创建迭代器
            position = []
            for line in iter_f: #遍历文件，一行行遍历，读取文本
                position.append(float(line))
        positions.append(position[0:-1])
    return positions, position_file

def Generate_actuations(positions, file_names):
    for index in range(len(positions) - 1):
        p_1 = positions[index]
        p_2 = positions[index + 1]
        actuation = []
        for i in range(len(p_1)):
            actuation.append(p_2[i] - p_1[i])
        file_name = file_names[index][:file_names[index].index('_')] + 'to' + file_names[index + 1][:file_names[index + 1].index('_')] + '.txt'
        save_dir = './Control_actions/'
        if not os.path.exists(save_dir):
            os.mkdir(save_dir)
        save_file = save_dir + file_name
        filename = open(save_file, 'w')
        for xyz in actuation:
            filename.write(str(xyz) + '\n')
        filename.close()

def main():
    positions, file_names = Read_files()
    print(positions[0])
    print(positions[-1])
    Generate_actuations(positions, file_names)

if __name__ == '__main__':
    main()
