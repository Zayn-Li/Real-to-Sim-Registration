#!/usr/bin/env bash


python3 ComputeError.py ./cube_exp_2/mesh_with_deform/obs_interp_ply/ ./cube_exp_2/NoRegis/Interior/ ./cube_exp_2/Regis0.1/Interior/ False

python3 ComputeError.py ./cube_exp_2/mesh_with_deform/obs_interp_ply/ ./cube_exp_2/NoRegis/Interior/ ./cube_exp_2/Regis0.1/Interior/ False

python3 ComputeError.py ./cube_exp_2/mesh_with_deform/obs_interp_ply/ ./cube_exp_2/NoRegis/Interior/ ./cube_exp_2/Regis0.2/Interior/ False

python3 ComputeError.py ./cube_exp_2/mesh_with_deform/obs_interp_ply/ ./cube_exp_2/NoRegis/Interior/ ./cube_exp_2/Regis0.2/Interior/ False
