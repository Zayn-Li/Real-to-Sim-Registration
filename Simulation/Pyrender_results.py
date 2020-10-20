import taichi as ti
import math as math
import pptk
import time
import numpy as np
import trimesh
import pyrender
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import random
import scipy.linalg
import time
import os
import copy
import re
import registration
import sys
import os

class Render():
    def __init__(self, offset, wire_frame, Experiment_set):
        self.Experiment_set = Experiment_set
        self.offset = offset
        self.wire_frame = wire_frame
        self.iteration = 0
        self.scene = pyrender.Scene()
        self.camera = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1)
        self.cam_pose_cube = np.array([
            [0.0,  -np.sqrt(2)/2, np.sqrt(2)/2, 0.4],
            [1.0, 0.0,           0.0,           0.1],
            [0.0,  np.sqrt(2)/2,  np.sqrt(2)/2, 1.2],
            [0.0,  0.0,           0.0,          1.0]
        ])
        self.cam_pose_lung = np.array([
            [1,   0, 0, 0],
            [0.0, 1, 0, -5],
            [0,  0,  1, 45],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.light_pose = np.array([
            [0.0,  -np.sqrt(2)/2, np.sqrt(2)/2, 15],
            [1.0, 0.0,           0.0,           20],
            [0.0,  np.sqrt(2)/2,  np.sqrt(2)/2, 14],
            [0.0,  0.0,           0.0,          1.0]
        ])
        self.direct_l = pyrender.DirectionalLight(color=np.ones(3), intensity=10.0)
        self.spot_l = pyrender.SpotLight(color=np.ones(3), intensity=10.0,
                                innerConeAngle=np.pi/16, outerConeAngle=np.pi/6)
        self.point_l = pyrender.PointLight(color=np.ones(3), intensity=10.0)
        self.v = pyrender.Viewer(self.scene, run_in_thread = True, use_raymond_lighting=True)
        self.node_current = pyrender.Node()
        self.mesh_current = 2 #define a class member
        self.mesh_pose_current = np.eye(4)
        self.node_map = {}
        self.i = 0
        self.fk = 0 #define a class member

    def configuration(self):
        if self.offset == 0:
            cam_node = self.scene.add(self.camera, pose=self.cam_pose_lung)
            self.light_pose = self.cam_pose_lung
        else:
            cam_node = self.scene.add(self.camera, pose=self.cam_pose_cube)
            self.light_pose = self.cam_pose_cube
        if(self.wire_frame == False):
            direc_l_node = self.scene.add(self.direct_l, pose=self.light_pose)
            spot_l_node = self.scene.add(self.spot_l, pose=self.light_pose)

    def update_mesh(self, index):
        ##deformable object
        if self.iteration >= 0 and self.iteration <= 9:
            fuze_trimesh_current = trimesh.load(self.Experiment_set + '/Results/Interior/interior_00000' + str(self.iteration) +'.ply')
            self.mesh_current = pyrender.Mesh.from_trimesh(fuze_trimesh_current, wireframe = self.wire_frame)
            self.mesh_pose_current[0,3] = -np.min(fuze_trimesh_current.vertices[:,0])
            self.mesh_pose_current[1,3] = -np.min(fuze_trimesh_current.vertices[:,1])
            self.mesh_pose_current[2,3] = -np.min(fuze_trimesh_current.vertices[:,2])
            if self.iteration == 0 and index == 0:
                self.node_current = self.scene.add(self.mesh_current, pose = self.mesh_pose_current)
        elif self.iteration <= 99:
            fuze_trimesh_current = trimesh.load(self.Experiment_set + '/Results/Interior/interior_0000' + str(self.iteration) +'.ply')
            self.mesh_current = pyrender.Mesh.from_trimesh(fuze_trimesh_current, wireframe = self.wire_frame)
            self.mesh_pose_current[0,3] = -np.min(fuze_trimesh_current.vertices[:,0])
            self.mesh_pose_current[1,3] = -np.min(fuze_trimesh_current.vertices[:,1])
            self.mesh_pose_current[2,3] = -np.min(fuze_trimesh_current.vertices[:,2])
        else:
            fuze_trimesh_current = trimesh.load(self.Experiment_set + '/Results/Interior/interior_000' + str(self.iteration) +'.ply')
            self.mesh_current = pyrender.Mesh.from_trimesh(fuze_trimesh_current, wireframe = self.wire_frame)
            self.mesh_pose_current[0,3] = -np.min(fuze_trimesh_current.vertices[:,0])
            self.mesh_pose_current[1,3] = -np.min(fuze_trimesh_current.vertices[:,1])
            self.mesh_pose_current[2,3] = -np.min(fuze_trimesh_current.vertices[:,2])
    
    def render(self):
        self.v.render_lock.acquire()
        print("This is", self.iteration, "iteration")
        #deformable object
        self.node_current.mesh = self.mesh_current
        self.node_current.matrix = self.mesh_pose_current
        #deformable object
        self.v.render_lock.release()
        self.iteration += 1


def main(Experient_set):
    Iteration_times = len(os.listdir(Experient_set + 'Results/Interior/'))
    wire_frame = False
    offset = 0
    off_screen = Render(offset, wire_frame, Experient_set)
    off_screen.configuration()
    index = 0
    while True:
        for i in range(Iteration_times):
            off_screen.update_mesh(index)
            off_screen.render()
            time.sleep(0.05)
        off_screen.iteration = 0
        index+=1


if __name__ == '__main__':
    ##python3 Pyrender.py ./cube_exp_2/ ##
    dir = sys.argv[1]
    main(dir)

