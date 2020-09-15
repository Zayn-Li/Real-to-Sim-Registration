import registration

registration.reg_init("./reg_data/initial.ply","./reg_data/vol_mesh_thin_tetgen.ply")
registration.reg_err("./reg_data/initial.ply","./reg_data/interior000017.ply","./reg_data/32.ply","./eg_der.txt","./eg_err.txt")