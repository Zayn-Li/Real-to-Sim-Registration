#include <math.h>
#include<iostream>
#include<string>
#include<fstream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

#include <pcl/surface/mls.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
double str2num(string s)
{
	
    bool temp=false;
    bool sign=false;
	int ndata=0;		
	double fdata=0;	
    double pdata;
	int n=0;			

	for(int i=0;i<s.length();i++)
	{
		while(((s[i]>='0')&&(s[i]<='9'))||(s[i]=='.')||(s[i]=='-'))	
		{
			temp=true;
			if(s[i]=='-')
            {
                sign = true;
                i++;
            }
            else
            {
                if(s[i]=='.')		
                {
                
                    i++;	
                    while((s[i]>='0')&&(s[i]<='9'))
                    {
                        n--;
                        fdata += (pow(10, n)*(s[i]-'0'));
                        i++;
                    }
                }

                else
                {
                    ndata*=10;
                    ndata+=(s[i]-'0');
                    i++;
                }                
            }	
	
		}

		if(temp)
		{
			pdata=ndata+fdata;
            if(sign)
                pdata = -pdata;
            temp = false;
		}
	}
    return pdata;
}


int main (int argc, char** argv)
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr surf_pt (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB> surf_pt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surf_pt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 surf_pt_2;
    // pcl::PointCloud<pcl::PointXYZRGB> surf_init;
    pcl::PointCloud<pcl::PointXYZ> surf_init;
    pcl::PolygonMesh surface;
    pcl::PolygonMesh surface_init;
    // std::cout<<"good"<<std::endl;

    // load surface file
    string tool;		
	ifstream infile;
    
    pcl::io::loadPLYFile ("../../PLY/0.ply", surface_init);

    pcl::fromPCLPointCloud2 (surface_init.cloud, surf_init);    
     
    // set number of files
    for(int file=0; file<95; file++){
        string in_path;
        string pt_path;
        string out_path;
        string t_path;
        
        in_path = "../../PLY/" + std::to_string(file) + ".ply";
        out_path = "../../test_pc/" + std::to_string(file) + ".ply";
        // t_path = "./selected_frame_transform/" + std::to_string(file) + "_Matrix.txt";

        pcl::io::loadPLYFile (in_path, surface);

        pcl::fromPCLPointCloud2 (surface.cloud, *surf_pt);

        
        // double array[4][4]={0.0};        
        // ifstream t_file(t_path);         
        // for(int i =0; i<4; i++)
        //     for(int j =0; j<4;j++)
        //     {
        //         t_file>>array[i][j];
        //     }  
        // // cout<< array[0][0]<<array[0][1]<<endl;      
        // infile.close();
        // double x,y,z;        
        // for(uint32_t i = 0; i < surf_init.width; i++)
        // {                        
        //     x = surf_pt.points[i].x;
        //     y = surf_pt.points[i].y;
        //     z = surf_pt.points[i].z;
            
        //     surf_pt.points[i].x = x * array[0][0] + y * array[0][1] + z * array[0][2] + array[0][3];
        //     surf_pt.points[i].y = x * array[1][0] + y * array[1][1] + z * array[1][2] + array[1][3];
        //     surf_pt.points[i].z = x * array[2][0] + y * array[2][1] + z * array[2][2] + array[2][3];      
        // }        
        
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        
         
        if (file < 82)
        {
            inliers->indices.clear();
            inliers->indices.push_back(160); 
            // inliers->indices.push_back(149);            
        
            extract.setInputCloud(surf_pt);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*surf_pt);
        }
//////////////////////////////////////////////////////

//         if (file < 56)
//         {
//             inliers->indices.clear();
//             inliers->indices.push_back(238); 
                  
//             extract.setInputCloud(surf_pt);
//             extract.setIndices(inliers);
//             extract.setNegative(true);
//             extract.filter(*surf_pt);
//         }
// /////////////////////////////////////////////////////

//         if (file < 148)
//         {
//             inliers->indices.clear();
//             inliers->indices.push_back(252);    
        
//             extract.setInputCloud(surf_pt);
//             extract.setIndices(inliers);
//             extract.setNegative(true);
//             extract.filter(*surf_pt);  
//         }
/////////////////////////////////////////////////////

        // if (file < 95)
        // {
        //     inliers->indices.clear();
        //     inliers->indices.push_back(221);    
        
        //     extract.setInputCloud(surf_pt);
        //     extract.setIndices(inliers);
        //     extract.setNegative(true);
        //     extract.filter(*surf_pt);  
        // }        
    
        // if(file ==0)
        // {
        //     pcl::toPCLPointCloud2 (surf_pt, surf_pt_2);
        //     surface_init.cloud = surf_pt_2;
        //     pcl::io::savePLYFile ("./test2_pc/initial.ply", surface_init);             
        // }        

        pcl::io::savePLYFile (out_path, *surf_pt);      

    }
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // Mesh visualization
    //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // pcl::io::loadPLYFile ("./PLY/mesh.ply", surface_init);
    // pcl::fromPCLPointCloud2 (surface_init.cloud, surf_init);

    // for(int file=0; file<203; file++){
    //     string in_path;
    //     string pt_path;
    //     string out_path;
    //     string t_path;
        
    //     in_path = "./PLY/" + std::to_string(file) + ".ply";
    //     pt_path = "./tool_pos/" + std::to_string(file) + "_position.txt";
    //     out_path = "./test2_pc/" + std::to_string(file) + ".ply";
    //     // t_path = "./selected_frame_transform/" + std::to_string(file) + "_Matrix.txt";

    //     pcl::io::loadPLYFile (in_path, surface);

    //     pcl::fromPCLPointCloud2 (surface.cloud, *surf_pt);

    //     // for(uint32_t i = 0; i < surf_init.width; i++)
    //     // {                        
    //     //     surf_init.points[i].r = 250;
    //     //     surf_init.points[i].g = 0;
    //     //     surf_init.points[i].b = 0;
 
    //     // }

    //     if(file ==0)
    //     {
    //         pcl::toPCLPointCloud2 (*surf_pt, surf_pt_2);
    //         surface_init.cloud = surf_pt_2;
    //         pcl::io::savePLYFile ("./test2_pc/initial.ply", surface_init);             
    //     }        
    //     // std::cout<<"good"<<std::endl;
        
    //     // pcl::PointXYZRGB cut_pt;
    //     // pcl::PointXYZRGB surr_pt[27];
    //     // double pt_data;
    //     // infile.open(pt_path);

    //     // // getline(infile, tool);

    //     // getline(infile, tool);
    //     // cut_pt.x = str2num(tool);
        
    //     // getline(infile, tool);
    //     // cut_pt.y = str2num(tool);
        
    //     // getline(infile, tool);
    //     // cut_pt.z = str2num(tool);

    //     // cut_pt.r = 0;
    //     // cut_pt.g = 250;
    //     // cut_pt.b = 0;
        
    //     // infile.close(); 


    //     // int i =0;
    //     // for(int x=-1; x<2; x++)
    //     //     for(int y=-1; y<2; y++)
    //     //         for(int z=-1; z<2; z++){
    //     //             surr_pt[i] = cut_pt;
    //     //             surr_pt[i].x += x*0.0004;
    //     //             surr_pt[i].y += y*0.0004;
    //     //             surr_pt[i].z += z*0.0004;
    //     //             *surf_pt.push_back(surr_pt[i]);
    //     //             i++;
                    
    //     //         }

    //     pcl::toPCLPointCloud2 (*surf_pt, surf_pt_2);
    //     surface_init.cloud = surf_pt_2;
    //     pcl::io::savePLYFile (out_path, surf_init);      

    // }
    return 0;
}
