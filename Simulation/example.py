import registration

# reg_init() calculate some initial values for the algorithm, the values will be written into two .txt files under the same folder of the this script
# call it before your iteration
# parameters: reg_init(string   path_of_the_initial_file_"initial.ply", 
#                      string   path_of_the_initial_tetgen_surface_"vol_mesh_thin_tetgen.ply")


def Read_registration(file_error, file_deriv):
    errors = []
    f = open(file_error)
    iter_f = iter(f)
    tmp = []
    for line in iter_f:
        line = line.split(" ")
        #line = re.findall(r"\d+\.?\d*",line)
        for index in range(len(line)):
            if len(line[index]) > 2 and line[index][2].isdigit():
                line[index] = float(line[index])
        tmp.append(line)
    f.close()
    errors = tmp[1:]
    tmp = []
    Deviat = []
    f = open(file_deriv)
    iter_f = iter(f)
    for line in iter_f:
        line = line.split(" ")
        for index in range(len(line)):
            if len(line[index]) > 3 and line[index][3].isdigit():
                line[index] = float(line[index])
        tmp.append(line)
    f.close()
    tmp = tmp[9:]  #remove the header information
    Deviat.append(tmp)
    #764 surface vertices that will be corrected
    return Deviat, errors


registration.reg_init("./surface_mesh/tetgenq1.4/initial.ply","./surface_mesh/tetgenq1.4/vol_mesh_" + "thin" + "_tetgen.ply")

# reg_err() calculate one error and derivatives of all points 
# you need to include it in your iteration
# parameters: reg_init(string   path_of_the_initial_file_"initial.ply", 
#                      string   path_of_the_simulated_surface_"xxxxxxxx.ply",
#                      string   path_of_the_observed_surface_"xxxxxxxx.ply",
#                      string   path_and_filename_you_would_like_to_save_error_derivatives_"xxxxx.txt",
#                      string   path_and_filename_you_would_like_to_save_total_error_"xxxxx.txt")
tmp_ply = "./Registration/tmp_00000" + str(0) +".ply"
Registraion_source = "./mesh_with_deform/" + str(30) + ".ply"
#registration.reg_err("./surface_mesh/tetgenq1.4/initial.ply",tmp_ply,Registraion_source,"./eg_der.txt","./eg_err.txt")
#Devir, error = Read_registration("./eg_err.txt", "./eg_der.txt")
#print(error)
tmp_ply = "./Registration/tmp_00000" + str(1) +".ply"
registration.reg_err("./surface_mesh/tetgenq1.4/initial.ply",tmp_ply,Registraion_source,"./eg_der.txt","./eg_err.txt")
Devir, error = Read_registration("./eg_err.txt", "./eg_der.txt")
print(error)
