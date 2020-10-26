import registration

# reg_init() calculate some initial values for the algorithm, the values will be written into two .txt files under the same folder of the this script
# call it before your iteration
# parameters: reg_init(string   path_of_the_initial_file_"initial.ply", 
#                      string   path_of_the_initial_tetgen_surface_"vol_mesh_thin_tetgen.ply")

registration.reg_init("./reg_data/initial.ply","./reg_data/vol_mesh.ply")

# reg_err() calculate one error and derivatives of all points 
# you need to include it in your iteration
# parameters: reg_init(string   path_of_the_initial_file_"initial.ply", 
#                      string   path_of_the_simulated_surface_"xxxxxxxx.ply",
#                      string   path_of_the_observed_surface_"xxxxxxxx.ply",
#                      string   path_and_filename_you_would_like_to_save_error_derivatives_"xxxxx.txt",
#                      string   path_and_filename_you_would_like_to_save_total_error_"xxxxx.txt")
registration.reg_err("./reg_data/initial.ply","./reg_data/interior_000075.ply","./reg_data/75.ply","./eg_der.txt","./eg_err.txt")

#registration.reg_err("./reg_data/initial.ply","./reg_data/tmp_000001.ply","./reg_data/30.ply","./eg_der.txt","./eg_err.txt")

