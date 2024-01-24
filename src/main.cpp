#include <vector>
#include <Eigen/Core>
//#include <pcl/memory.h>
#include<fstream>
#include <io.h>
#include"algo/point_cloud_registration.h"
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
//#include<pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
#include"algo/point_cloud_registration.h"
#include"algo/registration/plane_simplify.h"
#include<string>

/*
int main(int argc, char** argv)
{
    //system("pause");
    scan3d::PointCloudRegistration a;
    std::vector<std::string> data_name = { "08","09", "10" };
    std::vector <MatrixXi> point_pair;
    point_pair.resize(8);

   // for (int v = 1; v < 10; v++)
    {
        //double value = 0.5 + 0.5 * v;
        //for (int i = 0; i < data_name.size(); i++)
        {
           
            a.Registration_PPF_KITTI(data_name[0], "fpfh_4000_ppfa-" + std::to_string(int(0)), point_pair[0], 0);
             //a.cal_FPFH(data_name[i], 8000);
             //system("pause");
        }

    }


    system("pause");
    return EXIT_SUCCESS;
}*/

/*
int main(int argc, char** argv)
{
    system("pause");
    scan3d::PointCloudRegistration a;
    std::vector<std::string> data_name = {"redkitchen",
        "home_at", "home_md", "hotel_uc", "hotel_1", "hotel3", "study","lab" };
    std::vector <MatrixXi> point_pair;
    point_pair.resize(8);

    std::vector<int> data_value = { 250,500,1000,2500,5000 };

    for (int v = 0; v < 1; v++)
    {
        double value = 500;
        for (int i = 0; i < data_name.size() ; i++)
        {
            
            //a.Registration_PPF_3DLoMatch(data_name[i], "fpfh_4000_ppfa-" + std::to_string(int(value)), point_pair[i], value);
            a.Registration_PPF(data_name[i], "fpfh_ransac_10", point_pair[i], value);
            // a.cal_FPFH(data_name[i], 8000);
             //system("pause");
        }

    }


    system("pause");
    return EXIT_SUCCESS;
}*/
/*
int main(int argc, char** argv)
{
   // system("pause");
    scan3d::PointCloudRegistration a;
    std::vector<std::string> data_name = { "redkitchen",
        "home_at", "home_md", "hotel_uc", "hotel_1", "hotel3", "study","lab" };
    std::vector <MatrixXi> point_pair;
    point_pair.resize(8);

    std::vector<int> data_value = { 250,500,1000,2500,5000 };

    for (int v = 0; v < 1; v++)
    {
       // double value = 500;
        std::vector<double> value;
        for (int i = 0; i < 1; i++)
        {

            //a.Registration_PPF_3DLoMatch(data_name[i], "fpfh_4000_ppfa-" + std::to_string(int(value)), point_pair[i], value);
            a.Registration_PPF(data_name[i], "fpfh_ransac_10",  value);
            // a.cal_FPFH(data_name[i], 8000);
             //system("pause");
        }

    }


    system("pause");
    return EXIT_SUCCESS;
}*/

int main(int argc, char** argv)
{
    //system("pause");
    scan3d::PointCloudRegistration a;

    {
            std::vector<double> value;
            //a.Registration_PPF_3DLoMatch(data_name[i], "fpfh_4000_ppfa-" + std::to_string(int(value)), point_pair[i], value);
            //a.Registration_PPF_noise(data_name[0], value);
           a.Registration_DP();
            // a.Registration_DP_KITTI();
           // re.push_back(value);
            // a.cal_FPFH(data_name[i], 8000);
             //system("pause");
            /**/
     }
    

    system("pause");
    return EXIT_SUCCESS;
}

/*int main(int argc, char** argv)
{
   // system("pause");
   // scan3d::PointCloudRegistration a;
   // a.GeometricPrimitiveExtraction();
    scan3d::PlaneSimplify a;
   // a.PlanePrimitiveSimplify();
    a.PlaneLine_3DMatch();
    //a.PlaneLine_KITTI();
    //a.cal_pose();
    //a.PlaneLine_WuSTL();
    system("pause");
    return EXIT_SUCCESS;

}*/

/*
int main(int argc, char** argv)
{
    system("pause");
    scan3d::PointCloudRegistration a;
    std::vector<std::string> data_name = {"1-SubwayStation/","2-HighSpeedRailway/","3-Mountain/","5-park/","6-Campus/", "7-Residence/","8-RiverBanK/","9-HeritageBuilding/"};
    std::vector <MatrixXi> point_pair;
    point_pair.resize(8);

    std::vector<int> data_value = { 250,500,1000,2500,5000 };

   // a.cal_pose();
   // system("pause");

    for (int v = 0; v < 5; v++)
    {
        double value = data_value[v];// data_value[v];
        for (int i =3; i < 4; i++)//data_name.size()
        {
           // a.cal_trans(data_name[i]);
            //a.cal_reT(data_name[i]);
            //a.cal_FPFH(data_name[i], 8000,true);
            a.Registration_PPF_WUSTL(data_name[i], "fpfh_" + std::to_string(int(value )), point_pair[i], value);
        
             //system("pause");
        }

    }
    system("pause");
    return EXIT_SUCCESS;
}*/

/*
int main(int argc, char** argv)
{
    system("pause");
   
   
    scan3d::PointCloudRegistration a;
    std::string source = "E:/pointclouddata/SCVC_PIC/reslute2/zagreb_cathedral/scan000.ply";
    std::string target = "E:/pointclouddata/SCVC_PIC/reslute2/zagreb_cathedral/scan004.ply";
    a.cal_point_cloud_FPFH(source, 8000, true);
    a.cal_point_cloud_FPFH(target, 8000, true);
    a.Registration_PPF_Dome(source, target);
     
    system("pause");
    return EXIT_SUCCESS;
}


int main(int argc, char** argv)
{
    system("pause");
    scan3d::PointCloudRegistration a;
    std::vector<std::string> data_name={"lab","redkitchen", 
        "home_at", "home_md", "hotel_uc", "hotel_1", "hotel3", "study"};
    data_name = { "lab" };
    std::vector <MatrixXi> point_pair;
    point_pair.resize(8);
    
     for (int v = 1; v <2; v++)
    {
        double value =  v*250;
        for (int i = 0; i < data_name.size(); i++) 
        {
            if (v== 100 && i < 2)
            {
                i = 2;
            }
            a.Registration_PPF(data_name[i], "fpfh_4000_ppfa-" + std::to_string(int(value)), point_pair[i], value);
            //a.Registration_PPF_inlier(data_name[i], "fpfh_4000_ppfa-" + std::to_string(int(value)), point_pair[i], value);
           // a.cal_FPFH(data_name[i], 8000);
            //system("pause");
        }
           
    }

    
    system("pause");
    return EXIT_SUCCESS;
}*/
