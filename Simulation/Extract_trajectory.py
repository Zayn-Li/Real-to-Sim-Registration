import math as math
import numpy as np
import os
import sys

def Read_ControlIndex(thin_or_thick, Experiment_set):
    f = open(Experiment_set + '/Particle_index/control_' + thin_or_thick + '.txt')
    iter_f = iter(f)
    ControlParticles = []
    for line in iter_f:
        ControlParticles.append(int(line))
    f.close()
    return ControlParticles

# Regis_pos=Read_ActuatedPoints_FromRegistration(Registraion_source, ControlParticleIndex) #for control points
# Registraion_source = Experiment_set + "/mesh_with_deform/obs_interp_ply/" + str(iteration + 1) + ".ply" #observed points(ground-truth)

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
    tmp=tmp[tmp.index(['end_header\n'])+1:]
    actuated_points_positons=[]
    for i in actuated_points:
        actuated_points_positons.append(tmp[i])
    return actuated_points_positons

def Read_files(Experiment_set):
    position_path = Experiment_set + '/tool_trajectory'
    position_file = os.listdir(position_path)
    position_file.sort(key= lambda x:int(x[:x.index('_')]))
    positions = []
    for file in position_file:
        if not os.path.isdir(file):
            f = open(position_path+"/"+file);
            iter_f = iter(f);
            position = []
            for line in iter_f:
                position.append(float(line))
        positions.append(position[0:-1])
    return positions

if __name__ == '__main__':
    #Control input during each iteration
    Experiment_set = sys.argv[1]
    Thin_or_Thick = 'thin'
    ControlParticleIndex = Read_ControlIndex(Thin_or_Thick, Experiment_set)
    Tool_positions = Read_files(Experiment_set)
    np.savetxt(Experiment_set + '/Tool.txt', np.array(Tool_positions))
    total = len(Tool_positions)
    actuated_points_positons = []
    for iteration in range(total):
        actuated_points_positons.append(Read_ActuatedPoints_FromRegistration(Experiment_set + "/mesh_with_deform/obs_interp_ply/" + str(iteration) + ".ply", ControlParticleIndex)[0])
    np.savetxt(Experiment_set + '/pointcloud.txt', np.array(actuated_points_positons))