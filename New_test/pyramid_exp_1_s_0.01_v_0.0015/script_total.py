import os
import numpy as np

path = os.getcwd()
csv_path = path + '/fusion_pose/' 
off_path = path + '/OFF/'
ply_path = path + '/PLY/'
tool_path = path + '/selected_position/'
tl_output_path = path + '/tool/'

# csv to off
files = os.listdir(csv_path) # all files
files.sort()
file_id = 0
file_names = []
os.mkdir('./OFF')
for file in files:
    data = []
    timestamp = file.split(".")
    file_names.append(timestamp[0])
    with open(csv_path+file,'r') as f:# open a file
        point_flag = 0
        point_num = 0
        flag_pos = 0
        for line in f.readlines():
            # if line.find('position'): 
            #     point_flag = 1
            #     continue
            # if line.startswith('channels'):
            #     break
            # if point_flag:
            if line.find('position') > 0:
                point_num += 1
                flag_pos = 3
            elif flag_pos > 0:
                oneline = line.split()
                data.append(float(oneline[1]))
                flag_pos = flag_pos - 1

    points = np.array(data)
    pos = np.reshape(points,(-1,3))
    output_name = "./OFF/" + str(file_id)+".off"
    np.savetxt(output_name,pos)
    with open(output_name, 'r+') as fw:
        content = fw.read()        
        fw.seek(0, 0)
        fw.write('OFF\n'+str(point_num)+' 0 0\n'+content)
    file_id += 1
with open('stamps.txt', 'w') as fw2:
    for i in range(len(file_names)):
        fw2.write(file_names[i]+"\n")

# off to ply
os.mkdir('./PLY')
off_files = os.listdir(off_path)
for file in range(len(off_files)):
    
    command = "python ~/MeshReconstruction/converter/model-converter-python/convert.py -i " +\
         off_path + str(file) + ".off" + " -o " + ply_path + str(file) + ".ply"
    os.system(command)

# tool trajectory
tool_files = os.listdir(tool_path) # all files
tool_files.sort()
tool_file_id = 0
tool_file_names = []
os.mkdir('./tool')

for file in tool_files:
    data = []
    timestamp = file.split(".")
    tool_file_names.append(timestamp[0])
    with open(tool_path+file,'r') as f:# open a file
        point_flag = 0
        flag_pos = 0
        for line in f.readlines():
            if line.find('position') > 0:
                flag_pos = 3
            elif flag_pos > 0:
                oneline = line.split()
                data.append(float(oneline[1]))
                flag_pos = flag_pos - 1

    points = np.array(data)
    pos = np.reshape(points,(-1,1))
    output_name = tl_output_path + str(file_id)+"_position.txt"
    np.set_printoptions(suppress=True)
    np.set_printoptions(precision=4)  
    np.savetxt(output_name,pos,fmt='%.04f')
    tool_file_id += 1
with open(tl_output_path+'stamps.txt', 'w') as fw2:
    for i in range(len(tool_file_names)):
        fw2.write(tool_file_names[i]+"\n")

os.mkdir('./test_pc')
