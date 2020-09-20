# -*- coding: UTF-8 -*-
import os
import cv2
import time
import sys
# 图片合成视频
def picvideo(path,size,total_images):
    # path = r'C:\Users\Administrator\Desktop\1\huaixiao\\'#文件路径
    filelist = []
    for i in range(total_images):
        filelist.append('untitled' + str(i) + '.png')
    fps = 20
    # size = (591,705) #图片的分辨率片
    file_path = path + '/demo' + ".mp4"#导出路径
    fourcc = cv2.VideoWriter_fourcc('I', '4', '2', '0')#不同视频编码对应不同视频格式（例：'I','4','2','0' 对应avi格式）

    video = cv2.VideoWriter( file_path, fourcc, fps, size )

    for item in filelist:
        if item.endswith('.png'):   #判断图片后缀是否是.png
            item = path + '/' + item
            img = cv2.imread(item)  #使用opencv读取图像，直接返回numpy.ndarray 对象，通道顺序为BGR ，注意是BGR，通道值默认范围0-255。
            video.write(img)        #把图片写进视频

    video.release() #释放

dir = sys.argv[1]
picvideo(dir,(1280, 720),88)
