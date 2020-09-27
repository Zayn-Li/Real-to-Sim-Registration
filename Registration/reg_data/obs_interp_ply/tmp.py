import sys

def Read_ActuatedPoints_FromRegistration(regis_dir, actuated_points):
    f=open(regis_dir)
    iter_f=iter(f)
    tmp=[]
    for line in iter_f:
        line = line.split(" ")
        for index in range(len(line)):
            if len(line[index]) > 3 and line[index][3].isdigit():
                line[index] = float(line[index])
        tmp.append(line)
    f.close()
    tmp=tmp[10:]
    return tmp[actuated_points]

Read_ActuatedPoints_FromRegistration('./0.ply')

