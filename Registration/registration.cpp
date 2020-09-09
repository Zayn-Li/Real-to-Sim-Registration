#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

#include <pcl/surface/mls.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/point_types.h>

using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ> surf_sim;
    pcl::PCLPointCloud2 surf_pt_2;
    pcl::PointCloud<pcl::PointXYZ> surf_init;
    pcl::PointCloud<pcl::PointXYZ> surf_track;
    pcl::PointCloud<pcl::PointXYZ> surf_obs;
    pcl::PolygonMesh temp;
    string init_path,sim_path,track_path,obs_path,out_path;
    string file[17] = {"00","05","11","17","23","29","35","41","47","53","61","65","71","76","81","84","88"};
    double err_total[17] = {};

    for(int i_file = 0;i_file<17;i_file++)
    {
        // int i_file = 0;
        init_path = "./reg_data/initial.ply";
        sim_path = "./reg_data/interior_0000" + file[i_file] + ".ply";
        track_path = "./reg_data/vol_mesh_thin_tetgen.ply";
        obs_path = "./reg_data/"+ to_string(i_file+29)+".ply";
        out_path = "./results/deriv_0000" + file[i_file] + ".ply";

        pcl::io::loadPLYFile (init_path, temp);  
        pcl::fromPCLPointCloud2 (temp.cloud, surf_init);
        pcl::io::loadPLYFile (sim_path, temp);  
        pcl::fromPCLPointCloud2 (temp.cloud, surf_sim); 
        pcl::io::loadPLYFile (track_path, temp);  
        pcl::fromPCLPointCloud2 (temp.cloud, surf_track);     
        pcl::io::loadPLYFile (obs_path, temp);  
        pcl::fromPCLPointCloud2 (temp.cloud, surf_obs);   

        double dist_new;
        double dist = 1;
        int index_new;

        // construct initial SDF
        double offset_x = -0.045; //0.045
        double offset_y = -0.025; //0.023
        double offset_z = 0.075; //0.119
        // the SDF grid of initial frame
        static double SDF[100][60][55];
        static int id_close[100][60][55];
        for (int i =0; i<100 ;i++)
        {
            for (int j =0; j<60 ;j++)
            {
                for (int k =0; k<55 ;k++)
                {
                    dist = 1;

                    for (int i_surf_pt = 0; i_surf_pt<surf_init.width; i_surf_pt++)
                    {
                        dist_new = pow(surf_init.points[i_surf_pt].x - offset_x - i*0.001,2) + pow(surf_init.points[i_surf_pt].y - offset_y - j*0.001,2)
                        + pow(surf_init.points[i_surf_pt].z - offset_z - k*0.001,2);
                        if (dist_new < dist)
                        {
                            dist = dist_new;
                            index_new = i_surf_pt;
                        }
                    }
                    SDF[i][j][k] = dist;
                    id_close[i][j][k] = index_new;
                }        
            }       
        }

        //point matching(tracking)
        uint32_t tracking[surf_init.width];
        for (int i_track=0;i_track<surf_init.width;i_track++)
        {
            for (int i_search = 0;i_search<surf_track.width;i_search++)
            {
                if (fabs(surf_init.points[i_track].x - surf_track.points[i_search].x)<0.0002 &&
                    fabs(surf_init.points[i_track].y - surf_track.points[i_search].y)<0.0002 &&
                    fabs(surf_init.points[i_track].z - surf_track.points[i_search].z)<0.0002 )
                {
                    tracking[i_track] = i_search;
                    break;
                }
            }

        }

        // static pcl::PointXYZ eul_deform[90][50][45];
        static pcl::PointXYZ eul_after_def[100][60][55];
        
        // deformation of surface point
        pcl::PointXYZ pt_deform[surf_init.width];
        for (int i_pt=0;i_pt<surf_init.width;i_pt++)
        {
            pt_deform[i_pt].x = surf_sim.points[tracking[i_pt]].x-surf_init.points[i_pt].x;
            pt_deform[i_pt].y = surf_sim.points[tracking[i_pt]].y-surf_init.points[i_pt].y;
            pt_deform[i_pt].z = surf_sim.points[tracking[i_pt]].z-surf_init.points[i_pt].z;
        }
        
        for (int i =0; i<100 ;i++)
        {
            for (int j =0; j<60 ;j++)
            {
                for (int k =0; k<55 ;k++)
                {
                    dist = 1;

                    for (int i_surf_pt = 0; i_surf_pt<surf_init.width; i_surf_pt++)
                    {
                        dist_new = pow(surf_sim.points[tracking[i_surf_pt]].x - offset_x - i*0.001,2) + pow(surf_sim.points[tracking[i_surf_pt]].y - offset_y - j*0.001,2)
                        + pow(surf_sim.points[tracking[i_surf_pt]].z - offset_z - k*0.001,2);
                        if (dist_new < dist)
                        {
                            dist = dist_new;
                            index_new = i_surf_pt;
                        }
                    }
                    
                    // id_close[i][j][k] = index_new;
                }        
            }       
        }

        // the eular grid of simulated deformation
        
        for (int i =0; i<100 ;i++)
        {
            for (int j =0; j<60 ;j++)
            {
                for (int k =0; k<55 ;k++)
                {
                    eul_after_def[i][j][k] = pt_deform[id_close[i][j][k]];
                }
            }
        }

        // double dist_grid;
        // double dist_grid_cl = 1;
        
        // for (int i =0; i<90 ;i++)
        // {
        //     for (int j =0; j<50 ;j++)
        //     {
        //         for (int k =0; k<45 ;k++)
        //         {
        //             dist_grid_cl = 1;

        //             for (int i_grid =0; i_grid<90 ;i_grid++)
        //             {
        //                 for (int j_grid =0; j_grid<50 ;j_grid++)
        //                 {
        //                     for (int k_grid =0; k_grid<45 ;k_grid++)
        //                     {                
                                
        //                         dist_grid = pow((i_grid - i)*0.001 + eul_deform[i_grid][j_grid][k_grid].x,2)
        //                         + pow((j_grid - j)*0.001 + eul_deform[i_grid][j_grid][k_grid].y,2) + pow((k_grid - k)*0.001 + eul_deform[i_grid][j_grid][k_grid].z,2);

        //                         if (dist_grid < dist_grid_cl)
        //                         {
        //                             dist_grid_cl = dist_grid;
        //                             // use the displacement of the nearest transformed grid to represent the value of grid points (not optimal)
        //                             eul_after_def[i][j][k] = eul_deform[i_grid][j_grid][k_grid];
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }


        // find the interpolation coefficient

        double alpha,beta,gamma,alpha_g,beta_g,gamma_g,coeff_x,coeff_y,coeff_z;
        int i_grid,j_grid,k_grid,i_grid_g,j_grid_g,k_grid_g; 
        pcl::PointXYZ pt[8];  
        double err_grid[8];
        double err_pt[surf_obs.width];

        for (int i_pt=0;i_pt<surf_obs.width;i_pt++)
        {
            // find the interpolation coefficient
            i_grid = floor((surf_obs.points[i_pt].x - offset_x)*1000);
            alpha = (surf_obs.points[i_pt].x - offset_x)*1000 - i_grid;
            j_grid = floor((surf_obs.points[i_pt].y - offset_y)*1000);
            beta = (surf_obs.points[i_pt].y - offset_y)*1000 - j_grid;
            k_grid = floor((surf_obs.points[i_pt].z - offset_z)*1000);
            gamma = (surf_obs.points[i_pt].z - offset_z)*1000 - k_grid;             

            // inverse-transformation  
            for(int add_x=0; add_x<2;add_x++)
            {
                for(int add_y=0; add_y<2;add_y++)
                {
                    for(int add_z=0; add_z<2;add_z++)
                    {
                        pt[add_x*4+add_y*2+add_z].x = (i_grid+add_x) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].x;
                        pt[add_x*4+add_y*2+add_z].y = (j_grid+add_y) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].y;
                        pt[add_x*4+add_y*2+add_z].z = (k_grid+add_z) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].z;
                    }            
                }            
            }
            // compute error
            for(int i_grid_pt=0; i_grid_pt<8;i_grid_pt++)
            {        
                i_grid_g = floor(pt[i_grid_pt].x*1000);
                alpha_g = pt[i_grid_pt].x*1000 - i_grid_g;
                j_grid_g = floor(pt[i_grid_pt].y*1000);
                beta_g = pt[i_grid_pt].y*1000 - j_grid_g;
                k_grid_g = floor(pt[i_grid_pt].z*1000);
                gamma_g = pt[i_grid_pt].z*1000 - k_grid_g;  
                err_grid[i_grid_pt] = 0;
                
                for(int add_x=0; add_x<2;add_x++)
                {
                    if(add_x==0) coeff_x=1-alpha_g;
                    else coeff_x=alpha_g;

                    for(int add_y=0; add_y<2;add_y++)
                    {
                        if(add_y==0) coeff_y=1-beta_g;
                        else coeff_y=beta_g;                    
                        
                        for(int add_z=0; add_z<2;add_z++)
                        {
                            if(add_z==0) coeff_z=1-gamma_g;
                            else coeff_z=gamma_g;                         
                            // error of each grid vertice
                            err_grid[i_grid_pt] += coeff_x * coeff_y * coeff_z * SDF[i_grid_g+add_x][j_grid_g+add_y][k_grid_g+add_z];

                        }            
                    }            
                }   

            }
            // error of each point
            err_pt[i_pt] = alpha * beta * gamma * err_grid[7] + alpha * beta * (1-gamma) * err_grid[6] + alpha * (1-beta) * gamma * err_grid[5] + alpha * (1-beta) * (1-gamma) * err_grid[4]
                                + (1-alpha) * beta * gamma * err_grid[3] + (1-alpha) * beta * (1-gamma) * err_grid[2] + (1-alpha) * (1-beta) * gamma * err_grid[1] + (1-alpha) * (1-beta) * (1-gamma) * err_grid[0];
            err_total[i_file] += err_pt[i_pt];
        }

        double err_temp;
        pcl::PointCloud<pcl::PointXYZ> pt_dev;
        pt_dev = surf_init;
        for (int i_dev_pt=0;i_dev_pt<surf_init.width;i_dev_pt++)
        {
            for (int dev_ax =0;dev_ax<3;dev_ax++)
            {    
                for (int i_pt=0;i_pt<surf_init.width;i_pt++)
                {
                    pt_deform[i_pt].x = surf_sim.points[tracking[i_pt]].x-surf_init.points[i_pt].x;
                    pt_deform[i_pt].y = surf_sim.points[tracking[i_pt]].y-surf_init.points[i_pt].y;
                    pt_deform[i_pt].z = surf_sim.points[tracking[i_pt]].z-surf_init.points[i_pt].z;        
                    if (i_pt == i_dev_pt)
                    {
                        if(dev_ax==0) pt_deform[i_pt].x += 0.0001;            
                        else if(dev_ax==1) pt_deform[i_pt].y += 0.0001;
                        else if(dev_ax==2) pt_deform[i_pt].z += 0.0001;
                    }

                }
                    
                // the eular grid of simulated deformation

                
                for (int i =0; i<100 ;i++)
                {
                    for (int j =0; j<60 ;j++)
                    {
                        for (int k =0; k<55 ;k++)
                        {
                            eul_after_def[i][j][k] = pt_deform[id_close[i][j][k]];
                        }
                    }
                }

                // double dist_grid;
                // double dist_grid_cl = 1;
                
                // for (int i =0; i<90 ;i++)
                // {
                //     for (int j =0; j<50 ;j++)
                //     {
                //         for (int k =0; k<45 ;k++)
                //         {
                //             dist_grid_cl = 1;

                //             for (int i_grid =0; i_grid<90 ;i_grid++)
                //             {
                //                 for (int j_grid =0; j_grid<50 ;j_grid++)
                //                 {
                //                     for (int k_grid =0; k_grid<45 ;k_grid++)
                //                     {                               
                                        
                //                         dist_grid = pow((i_grid - i)*0.001 + eul_deform[i_grid][j_grid][k_grid].x,2)
                //                         + pow((j_grid - j)*0.001 + eul_deform[i_grid][j_grid][k_grid].y,2) + pow((k_grid - k)*0.001 + eul_deform[i_grid][j_grid][k_grid].z,2);

                //                         if (dist_grid < dist_grid_cl)
                //                         {
                //                             dist_grid_cl = dist_grid;
                //                             // use the displacement of the nearest transformed grid to represent the value of grid points (not optimal)
                //                             eul_after_def[i][j][k] = eul_deform[i_grid][j_grid][k_grid];
                //                         }
                //                     }
                //                 }
                //             }
                //         }
                //     }
                // }


                // find the interpolation coefficient
                
                err_temp = 0;

                for (int i_pt=0;i_pt<surf_obs.width;i_pt++)
                {
                    // find the interpolation coefficient
                    i_grid = floor((surf_obs.points[i_pt].x - offset_x)*1000);
                    alpha = (surf_obs.points[i_pt].x - offset_x)*1000 - i_grid;
                    j_grid = floor((surf_obs.points[i_pt].y - offset_y)*1000);
                    beta = (surf_obs.points[i_pt].y - offset_y)*1000 - j_grid;
                    k_grid = floor((surf_obs.points[i_pt].z - offset_z)*1000);
                    gamma = (surf_obs.points[i_pt].z - offset_z)*1000 - k_grid;             

                    // inverse-transformation  
                    for(int add_x=0; add_x<2;add_x++)
                    {
                        for(int add_y=0; add_y<2;add_y++)
                        {
                            for(int add_z=0; add_z<2;add_z++)
                            {
                                pt[add_x*4+add_y*2+add_z].x = (i_grid+add_x) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].x;
                                pt[add_x*4+add_y*2+add_z].y = (j_grid+add_y) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].y;
                                pt[add_x*4+add_y*2+add_z].z = (k_grid+add_z) *0.001 - eul_after_def[i_grid+add_x][j_grid+add_y][k_grid+add_z].z;
                            }            
                        }            
                    }
                    // compute error
                    for(int i_grid_pt=0; i_grid_pt<8;i_grid_pt++)
                    {        
                        i_grid_g = floor(pt[i_grid_pt].x*1000);
                        alpha_g = pt[i_grid_pt].x*1000 - i_grid_g;
                        j_grid_g = floor(pt[i_grid_pt].y*1000);
                        beta_g = pt[i_grid_pt].y*1000 - j_grid_g;
                        k_grid_g = floor(pt[i_grid_pt].z*1000);
                        gamma_g = pt[i_grid_pt].z*1000 - k_grid_g;  
                        err_grid[i_grid_pt] = 0;
                        
                        for(int add_x=0; add_x<2;add_x++)
                        {
                            if(add_x==0) coeff_x=1-alpha_g;
                            else coeff_x=alpha_g;

                            for(int add_y=0; add_y<2;add_y++)
                            {
                                if(add_y==0) coeff_y=1-beta_g;
                                else coeff_y=beta_g;                    
                                
                                for(int add_z=0; add_z<2;add_z++)
                                {
                                    if(add_z==0) coeff_z=1-gamma_g;
                                    else coeff_z=gamma_g;                         
                                    // error of each grid vertice
                                    err_grid[i_grid_pt] += coeff_x * coeff_y * coeff_z * SDF[i_grid_g+add_x][j_grid_g+add_y][k_grid_g+add_z];

                                }            
                            }            
                        }   

                    }
                    // error of each point
                    err_pt[i_pt] = alpha * beta * gamma * err_grid[7] + alpha * beta * (1-gamma) * err_grid[6] + alpha * (1-beta) * gamma * err_grid[5] + alpha * (1-beta) * (1-gamma) * err_grid[4]
                                + (1-alpha) * beta * gamma * err_grid[3] + (1-alpha) * beta * (1-gamma) * err_grid[2] + (1-alpha) * (1-beta) * gamma * err_grid[1] + (1-alpha) * (1-beta) * (1-gamma) * err_grid[0];

                    err_temp += err_pt[i_pt];
                    
                }
                
                if(dev_ax==0) pt_dev.points[i_dev_pt].x = (err_temp-err_total[i_file]) * 10000;            
                else if(dev_ax==1) pt_dev.points[i_dev_pt].y = (err_temp-err_total[i_file]) * 10000;
                else if(dev_ax==2) pt_dev.points[i_dev_pt].z = (err_temp-err_total[i_file]) * 10000;            
                
            }
        }
        pcl::io::savePLYFile (out_path, pt_dev);  
        // cout << err_total << endl;
    }
	std::ofstream wfile("./results/error.txt");

    if (!wfile)
        return 1;
    wfile << "ERROR" << std::endl;
	for (uint32_t i=0; i < 17; i++)
	{
		
        wfile << err_total[i] << std::endl;
	}    

    return 0;
}