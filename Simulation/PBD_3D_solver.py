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


ti.init(debug=False,arch=ti.cpu) #cpu or cuda
real = ti.f32 #data type f32 -> float in C

max_num_particles = 10000
maximum_tetras = 40000

lambda_epsilon = 0.0 #user specified relaxation parameter(important) -> adjustable
dt = 1e-2#simulation time step(important) -> adjustable
dt_inv = 1 / dt
dx = 0.02
dim = 3
pbd_num_iters = 30#Iteration number(important) -> adjustable

scalar = lambda: ti.var(dt=real) #2D dense tensor
vec = lambda: ti.Vector(dim, dt=real) #2*1 vector(each element in a tensor)
mat = lambda: ti.Matrix(dim, dim, dt=real) #2*2 matrix(each element in a tensor)

num_particles = ti.var(ti.i32, shape=())
num_tetra = ti.var(ti.i32, shape=())
damping = ti.var(ti.f32, shape=())
control_one_particle = ti.var(ti.i32, shape=())

particle_mass = 1 #mi
particle_mass_inv = 1 / particle_mass # 1 / mi
particle_mass_invv = 1 / (particle_mass_inv + particle_mass_inv + lambda_epsilon)
maximum_constraints = 50
epsolon = 1e-8 #digit accurary(important) -> adjustable
volumn_epsolon = 1e-11

x, v, old_x = vec(), vec(), vec()
actuation_type = scalar()
# if rest_length[i, j] = 0, it means i and j are not connected
rest_length = scalar()
tetra_volumn = scalar()
mass = scalar()
position_delta_tmp = vec()
constraint_neighbors = ti.var(ti.i32)
constraint_num_neighbors = ti.var(ti.i32)
volumn_constraint_num = ti.var(ti.i32)
volumn_constraint_list = scalar()
gravity = [0, 0, 0] #direction (x,y,z) accelaration

@ti.layout  #Environment layout(placed in ti.layout) initializatioxn of the dimensiond of each tensor variables(global)
def place():
    ti.root.dense(ti.i, maximum_tetras).place(volumn_constraint_list)
    ti.root.dense(ti.ij, (maximum_tetras, 5)).place(tetra_volumn)
    ti.root.dense(ti.ij, (max_num_particles, max_num_particles)).place(rest_length)
    ti.root.dense(ti.i, max_num_particles).place(x, v, old_x, actuation_type, position_delta_tmp, mass) #initialzation to zero
    nb_node = ti.root.dense(ti.i, max_num_particles)
    nb_node.place(constraint_num_neighbors)
    nb_node.place(volumn_constraint_num)
    nb_node.dense(ti.j, maximum_constraints).place(constraint_neighbors)

@ti.kernel
def old_posi(n: ti.i32):
    for i in range(n):
        old_x[i] = x[i]

@ti.kernel
def find_spring_constraint(n: ti.i32):
    for i in range(n):
        nb_i = 0
        for j in range(n):
            if rest_length[i, j] != 0: #spring-constraint
                x_ij = x[i] - x[j]
                dist_diff = abs(x_ij.norm() - rest_length[i, j])
                if dist_diff > epsolon:
                    constraint_neighbors[i, nb_i] = j
                    nb_i += 1
        constraint_num_neighbors[i] = nb_i

@ti.kernel
def find_volumn_constraint(n: ti.i32):
    for i in range(n):
        p1_index = tetra_volumn[i, 0]
        p2_index = tetra_volumn[i, 1]
        p3_index = tetra_volumn[i, 2]
        p4_index = tetra_volumn[i, 3]
        p1 = x[int(p1_index)]
        p2 = x[int(p2_index)]
        p3 = x[int(p3_index)]
        p4 = x[int(p4_index)]
        volumn = ((p2[1]-p1[1])*(p3[2]-p1[2])-(p2[2]-p1[2])*(p3[1]-p1[1]))*(p4[0]-p1[0]) + \
                 ((p3[0]-p1[0])*(p2[2]-p1[2])-(p2[0]-p1[0])*(p3[2]-p1[2]))*(p4[1]-p1[1]) + \
                 ((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]))*(p4[2]-p1[2])
        volumn /= 6
        if(abs(volumn - tetra_volumn[i, 4]) > volumn_epsolon):
            volumn_constraint_list[i] = volumn - tetra_volumn[i, 4]
            volumn_constraint_num[int(p1_index)] += 1
            volumn_constraint_num[int(p2_index)] += 1
            volumn_constraint_num[int(p3_index)] += 1
            volumn_constraint_num[int(p4_index)] += 1
        else:
            volumn_constraint_list[i] = 0.0


@ti.kernel
def substep(n: ti.i32, x_: ti.f32, y_: ti.f32, z_: ti.f32): # Compute force and new velocity
    for i in range(n):
        if actuation_type[i] == 1:  #control points fixed on the robot arm
            x[i][0] += x_
            x[i][1] += y_
            x[i][2] += z_
            # x[i] += ti.Vector([x, y, z])

@ti.kernel
def Position_update(n: ti.i32):# Compute new position
    for i in range(n):
        x[i] += v[i] * dt

@ti.kernel
def stretch_constraint(n: ti.i32):
    for i in range(n):
        pos_i = x[i]
        posi_tmp = ti.Vector([0.0, 0.0, 0.0])
        mass_i_inv = 1 / mass[i]
        for j in range(constraint_num_neighbors[i]):
            p_j = constraint_neighbors[i, j]
            pos_j = x[p_j]
            x_ij = pos_i - pos_j
            dist_diff = x_ij.norm() - rest_length[i, p_j]
            grad = x_ij.normalized()
            mass_j_inv = 1 / mass[p_j]
            mass_ij_inv = 1 / (mass_i_inv + mass_j_inv)
            position_delta = -mass_i_inv * mass_ij_inv * dist_diff * grad / constraint_num_neighbors[i]
            posi_tmp += position_delta
        position_delta_tmp[i] = posi_tmp

@ti.kernel
def volumn_constraint(n: ti.i32):
    for i in range(n):
        if(volumn_constraint_list[i] != 0.0):
            diff_volumn = volumn_constraint_list[i]
            p1_index = tetra_volumn[i, 0]
            p2_index = tetra_volumn[i, 1]
            p3_index = tetra_volumn[i, 2]
            p4_index = tetra_volumn[i, 3]
            #position of each particle
            p1 = x[int(p1_index)]
            p2 = x[int(p2_index)]
            p3 = x[int(p3_index)]
            p4 = x[int(p4_index)]
            grad_x1 = ((p1[1] - p3[1])*(p1[2] - p2[2]))/6 - ((p1[1] - p2[1])*(p1[2] - p3[2]))/6 + ((p1[1] - p4[1])*(p2[2] - p3[2]))/6 - ((p2[1] - p3[1])*(p1[2] - p4[2]))/6
            grad_y1 = ((p1[0] - p2[0])*(p1[2] - p3[2]))/6 - ((p1[0] - p3[0])*(p1[2] - p2[2]))/6 - ((p1[0] - p4[0])*(p2[2] - p3[2]))/6 + ((p2[0] - p3[0])*(p1[2] - p4[2]))/6
            grad_z1 = ((p1[0] - p3[0])*(p1[1] - p2[1]))/6 - ((p1[0] - p2[0])*(p1[1] - p3[1]))/6 + ((p1[0] - p4[0])*(p2[1] - p3[1]))/6 - ((p2[0] - p3[0])*(p1[1] - p4[1]))/6
            grad_x2 = ((p1[1] - p3[1])*(p1[2] - p4[2]))/6 - ((p1[1] - p4[1])*(p1[2] - p3[2]))/6
            grad_y2 = ((p1[0] - p4[0])*(p1[2] - p3[2]))/6 - ((p1[0] - p3[0])*(p1[2] - p4[2]))/6
            grad_z2 = ((p1[0] - p3[0])*(p1[1] - p4[1]))/6 - ((p1[0] - p4[0])*(p1[1] - p3[1]))/6
            grad_x3 = ((p1[1] - p4[1])*(p1[2] - p2[2]))/6 - ((p1[1] - p2[1])*(p1[2] - p4[2]))/6
            grad_y3 = ((p1[0] - p2[0])*(p1[2] - p4[2]))/6 - ((p1[0] - p4[0])*(p1[2] - p2[2]))/6
            grad_z3 = ((p1[0] - p4[0])*(p1[1] - p2[1]))/6 - ((p1[0] - p2[0])*(p1[1] - p4[1]))/6
            grad_x4 = ((p1[1] - p2[1])*(p1[2] - p3[2]))/6 - ((p1[1] - p3[1])*(p1[2] - p2[2]))/6
            grad_y4 = ((p1[0] - p3[0])*(p1[2] - p2[2]))/6 - ((p1[0] - p2[0])*(p1[2] - p3[2]))/6
            grad_z4 = ((p1[0] - p2[0])*(p1[1] - p3[1]))/6 - ((p1[0] - p3[0])*(p1[1] - p2[1]))/6
            grad_p1 = ti.Vector([grad_x1, grad_y1, grad_z1])
            tmp_p1 = grad_p1.norm() * grad_p1.norm()
            w_p1 = 1 / mass[int(p1_index)]
            grad_p2 = ti.Vector([grad_x2, grad_y2, grad_z2])
            tmp_p2 = grad_p2.norm() * grad_p2.norm()
            w_p2 = 1 / mass[int(p2_index)]
            grad_p3 = ti.Vector([grad_x3, grad_y3, grad_z3])
            tmp_p3 = grad_p3.norm() * grad_p3.norm()
            w_p3 = 1 / mass[int(p3_index)]
            grad_p4 = ti.Vector([grad_x4, grad_y4, grad_z4])
            tmp_p4 = grad_p4.norm() * grad_p4.norm()
            w_p4 = 1 / mass[int(p4_index)]
            denominator = w_p1 * tmp_p1 + w_p2 * tmp_p2 + w_p3 * tmp_p3 + w_p4 * tmp_p4
            constraint_lambda = volumn_constraint_list[i] / denominator
            delta_p1 = -constraint_lambda * w_p1 * grad_p1 / volumn_constraint_num[int(p1_index)]
            delta_p2 = -constraint_lambda * w_p2 * grad_p2 / volumn_constraint_num[int(p2_index)]
            delta_p3 = -constraint_lambda * w_p3 * grad_p3 / volumn_constraint_num[int(p3_index)]
            delta_p4 = -constraint_lambda * w_p4 * grad_p4 / volumn_constraint_num[int(p4_index)]
            position_delta_tmp[int(p1_index)] += delta_p1
            position_delta_tmp[int(p2_index)] += delta_p2
            position_delta_tmp[int(p3_index)] += delta_p3
            position_delta_tmp[int(p4_index)] += delta_p4

@ti.kernel
def apply_position_deltas(n: ti.i32):
    for i in range(n):
        if actuation_type[i] != 1: #or if actuation_type[i] == 0
            x[i] += position_delta_tmp[i]

@ti.kernel
def updata_velosity(n: ti.i32): #updata velosity after combining constraints
    for i in range(n):
        v[i] = (x[i] - old_x[i]) * dt_inv

@ti.kernel
def new_particle(pos_x: ti.f32, pos_y: ti.f32, pos_z: ti.f32, control_type: ti.i32): # Taichi doesn't support using Matrices as kernel arguments yet
    new_particle_id = num_particles[None]
    x[new_particle_id] = [pos_x, pos_y, pos_z]
    v[new_particle_id] = [0, 0, 0]
    #control_type:
    # 0 -> fixed particles
    # 1 -> actuated particles
    # else -> other particles
    #assign control label to each particle(-1,1,0)
    if control_type == 0:
        actuation_type[new_particle_id] = -1
        mass[new_particle_id] = 10000000
    elif control_type == 1:
        actuation_type[new_particle_id] = 1
        mass[new_particle_id] = 1
        control_one_particle[None] = new_particle_id
    else:
        actuation_type[new_particle_id] = 0
        mass[new_particle_id] = 1
    num_particles[None] += 1

@ti.kernel
def new_costraint(p1_index: ti.i32, p2_index: ti.i32, dist: ti.f32):
    # Connect with existing particles
    rest_length[p1_index, p2_index] = dist
    rest_length[p2_index, p1_index] = dist

@ti.kernel
def new_tetra(p1_index: ti.i32, p2_index: ti.i32, p3_index: ti.i32, p4_index: ti.i32):
    new_tetra_id = num_tetra[None]
    tetra_volumn[new_tetra_id, 0] = float(p1_index)
    tetra_volumn[new_tetra_id, 1] = float(p2_index)
    tetra_volumn[new_tetra_id, 2] = float(p3_index)
    tetra_volumn[new_tetra_id, 3] = float(p4_index)
    p1 = x[p1_index]
    p2 = x[p2_index]
    p3 = x[p3_index]
    p4 = x[p4_index]
    volumn = ((p2[1]-p1[1])*(p3[2]-p1[2])-(p2[2]-p1[2])*(p3[1]-p1[1]))*(p4[0]-p1[0]) + \
             ((p3[0]-p1[0])*(p2[2]-p1[2])-(p2[0]-p1[0])*(p3[2]-p1[2]))*(p4[1]-p1[1]) + \
             ((p2[0]-p1[0])*(p3[1]-p1[1])-(p3[0]-p1[0])*(p2[1]-p1[1]))*(p4[2]-p1[2])
    volumn /= 6
    tetra_volumn[new_tetra_id, 4] = volumn
    num_tetra[None] += 1

def check_single_particle():
    cons = rest_length.to_numpy()
    invalid_particle = []
    for i in range(num_particles[None]):
        sum = 0
        for j in range(num_particles[None]):
            sum += cons[i ,j]
        if sum == 0:
            invalid_particle.append(i)
    return invalid_particle

def tetealedon2constraint(constraints, X):
    new_constraints = []
    for i in constraints:
        for j in range(len(i)):
            pos_j = X[i[j]]
            for z in range(j + 1, len(i)):
                new_constraint = []
                pos_z = X[i[z]]
                pos_jz = pos_j - pos_z
                new_constraint.append(i[j])
                new_constraint.append(i[z])
                new_constraint.append(np.linalg.norm(pos_jz))
                new_constraints.append(new_constraint)
    return new_constraints

def forward(number_particles, number_tetra, x_, y_, z_):
    #the first three steps -> only consider external force
    old_posi(number_particles)
    substep(number_particles, x_, y_, z_)
    Position_update(number_particles)
    for i in range(pbd_num_iters):
        constraint_neighbors.fill(-1)
        find_spring_constraint(number_particles)
        volumn_constraint_num.fill(0)
        find_volumn_constraint(number_tetra)
        #print("This is ", i , "th iteration.")
        stretch_constraint(number_particles)
        volumn_constraint(number_tetra)
        apply_position_deltas(number_particles)
    updata_velosity(number_particles)

#gui = ti.GUI('Mass Spring System', res=(640, 640), background_color=0xdddddd)
class Render():
    def __init__(self, offset, wire_frame):
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

    def update_mesh(self):
        ##deformable object
        if self.iteration >= 0 and self.iteration <= 9:
            fuze_trimesh_current = trimesh.load('./Results/Interior/interior_00000' + str(self.iteration) +'.ply')
            self.mesh_current = pyrender.Mesh.from_trimesh(fuze_trimesh_current, wireframe = self.wire_frame)
            self.mesh_pose_current[0,3] = -np.min(fuze_trimesh_current.vertices[:,0])
            self.mesh_pose_current[1,3] = -np.min(fuze_trimesh_current.vertices[:,1])
            self.mesh_pose_current[2,3] = -np.min(fuze_trimesh_current.vertices[:,2])
            if self.iteration == 0:
                self.node_current = self.scene.add(self.mesh_current, pose = self.mesh_pose_current)
        else:
            fuze_trimesh_current = trimesh.load('./Results/Interior/interior_0000' + str(self.iteration) +'.ply')  
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

control_one_particle[None] = max_num_particles
damping[None] = 30

def L2_distance(p1, p2):
    return (p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2

def solver_and_render(total_images, wire_frame, controlTrajectory, offset, dir_path, scalar):
    #Read all mesh points from txt.file
    points = []
    with open(dir_path + 'node', 'r') as f:
        data = f.readlines()
    for line in data[1: len(data) - 1]:
        odom = line.split()
        points.append([float(odom[1]), float(odom[2]), float(odom[3])])
    # Read all constraints(tetrahedron)
    constraints = []
    volumn = []
    with open(dir_path + 'ele', 'r') as f:
        data = f.readlines()
    for line in data[1: len(data) - 1]:
        odom = line.split()
        constraints.append([int(odom[1]) - offset, int(odom[2])- offset, int(odom[3]) - offset, int(odom[4]) - offset])
    # Read all faces(interior tetrahedron)
    surface_tri = []
    with open(dir_path + 'face', 'r') as f:
        data = f.readlines()
    for line in data[1: len(data) - 1]:
        odom = line.split()
        if (int(odom[1]) - offset) not in surface_tri:
            surface_tri.append((int(odom[1]) - offset))
        if (int(odom[2]) - offset) not in surface_tri:
            surface_tri.append((int(odom[2]) - offset))
        if (int(odom[3]) - offset) not in surface_tri:
            surface_tri.append((int(odom[3]) - offset))
    surface_tri.sort()
    surface_points = []
    y_surface = 0
    x_max = 0
    x_min = 10000
    z_max = 0
    z_min = 10000
    for i in range(len(points)):
        if i in surface_tri:
            surface_points.append(points[i])
    p=np.array(surface_points)
    v=pptk.viewer(p)
    print("Number of surface vertices:", len(surface_points))
    original_index = surface_tri
    new_index = list(range(len(surface_tri)))
    surface_only_tri = [] #facet information for only surface vertices
    for line in data[1: len(data) - 1]:
        odom = line.split()
        a1 = original_index.index(int(odom[1]) - offset)
        a2 = original_index.index(int(odom[2]) - offset)
        a3 = original_index.index(int(odom[3]) - offset)
        surface_only_tri.append([new_index[a2], new_index[a1], new_index[a3]])

    num_particles[None] = 0
    num_tetra[None] = 0
    for i in range(len(points)):
        control_type = -1
        new_particle(points[i][0], points[i][1], points[i][2], control_type)
    n = num_particles[None]
    X = x.to_numpy()[:n]
    new_constraints = tetealedon2constraint(constraints, X)
    for i in new_constraints:
        new_costraint(i[0], i[1], float(i[2]))

    for tetra_vertices in constraints:
        new_tetra(tetra_vertices[0], tetra_vertices[1], tetra_vertices[2], tetra_vertices[3])
    number_tetra = num_tetra[None]
    volumn_start = tetra_volumn.to_numpy()[:number_tetra]
    volumn_sum = 0
    for i in range(number_tetra):
        volumn_sum += volumn_start[i, 4]
    print("Total volumn is: ", volumn_sum)
    single_particle_list = check_single_particle()
    ##begin: setup PLY
    tmp = []
    for i in surface_only_tri:
        for j in i:
            tmp.append(j)
    interior_mesh = np.array(tmp)
    interior_prefix_ascii = "./Results/Interior/interior.ply"
    ##end: setup PLY
    #begin:setup rendering offscreen
    off_screen = Render(offset, wire_frame)
    off_screen.configuration()
    #end:setup rendering offscreen
    iteration = 0
    frame = 0
    while iteration <= total_images - 1: #total simulation steps
        if iteration % 1 == 0: #output every n iterations(n = 1 here)
            X = x.to_numpy()[original_index] #extract surface points
            iterior_x = X[:,0] / scalar
            iterior_y = X[:,1] / scalar
            iterior_z = X[:,2] / scalar
            writer_interior = ti.PLYWriter(num_vertices=len(surface_tri), num_faces=len(surface_only_tri), face_type="tri")
            writer_interior.add_vertex_pos(iterior_x, iterior_y, iterior_z)
            writer_interior.add_faces(interior_mesh)
            writer_interior.export_frame_ascii(frame, interior_prefix_ascii)
            off_screen.update_mesh()
            off_screen.render()
            frame += 1
        #user specify
        for step in range(1):
            x_ = controlTrajectory[iteration][0]
            y_ = controlTrajectory[iteration][1]
            z_ = controlTrajectory[iteration][2]
            forward(n, number_tetra, x_, y_, z_) #x,y,z control input
        print("Next step!")
        iteration += 1
    num_tetra[None] = 0
    for tetra_vertices in constraints:
        new_tetra(tetra_vertices[0], tetra_vertices[1], tetra_vertices[2], tetra_vertices[3])
    volumn_final = tetra_volumn.to_numpy()[:num_tetra[None]]
    volumn_sum = 0
    for i in range(num_tetra[None]):
        volumn_sum += volumn_final[i, 4]
    print("After simulation, volumn is:", volumn_sum)
    # print("Video has been generated!")

def Read_control():
    position_path = './Control_actions'
    position_file = os.listdir(position_path)
    position_file.sort(key= lambda x:int(x[:x.index('to')]))

if __name__ == '__main__':
    #Control input during each iteration
    ControlTrajectory = Read_control()
    scalar = 1
    offset = 0
    dir = './volume_mesh/tetgenq1.4/vol_mesh_thin/vol_mesh_thin.1.'
    wire_frame = False #Render option: True -> wire frame; False -> surface
    total_images = 51 #Total number of steps
    ControlTrajectory = [[x_, y_, z_]] * total_images #User specify

    #Control input for each step. Currently, they are the same control inputs.
    #TODO: User specify which particle(particles) to control; Current, I pick up a surface particle randomly.
    solver_and_render(total_images, wire_frame, ControlTrajectory, offset, dir, scalar)
