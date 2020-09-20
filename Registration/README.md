## 观测图像插值
差值后的观测点保存在 Registration/Registration/reg_data/obs_interp_ply/ 中，文件名对应tetgen生成的surface mesh文件（interior_0000XX.ply）的后两位数字
因为现在每一帧需要修正的量偏小，可以减少每一帧的循环量，我感觉可以减到10或更少。
registration的系数你也可以适当调整

## 梯度平均
梯度平均我提供了两种滤波器在 registration.cpp 中，你可以分别理解为高斯滤波（注释掉的）和均值滤波（现在的代码）。你在cpp文件中搜索alternative应该可以找到这两段代码。
单纯看生成的梯度文件我觉得均值滤波好一些。
现在文件编译默认加上了均值滤波，如果不需要它的话（比如你在测试图像插值的时候），记得把cpp文件中 savePLYfiles 这个函数输入的变量从 err_dev_avg 改成 err_dev
