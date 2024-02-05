#include <vector>
//#include <Eigen/Core>
#include<fstream>
#include <io.h>
#include"algo/point_cloud_registration.h"
#include<string>
//#include"Eigen/Core"
#include"../3rdParty/eigen-3.4.0/Eigen/Core"

int main(int argc, char** argv)
{
    std::string datasetname = "KITTI";

    registration::PointCloudRegistration a;

    if (datasetname == "3DMatch" || datasetname == "3DLoMatch")
    {
        std::string featuretype = "fcgf";//fcgf fpfh
        std::string posepath = "../data/3DMatch3DLoMatch/benchmarks/";
        posepath = posepath + datasetname + "/";
        std::string featruepath = "../data/3DMatch3DLoMatch/"+ featuretype +"/";
        std::string pointcloudpath = "../data/3DMatch3DLoMatch/pointcloud/";
        a.Registration_3DMatch3DLoMatch(datasetname, featuretype, posepath, featruepath, pointcloudpath);

    }
    else if(datasetname == "KITTI")
    {
        
        std::string posepath = "../data/KITTI/icp";

        std::string pointcloudpath = "../data/KITTI/sequences/";
        std::string featuretype = "fpfh";//fcgf fpfh
        std::string featurepath = "../data/KITTI/feat_"+ featuretype +"_txt/";
        //std::string featurepath = "E:/pointclouddata/data_odometry_velodyne/feat_fpfh_txt/";
         a.Registration_KITTI(featuretype, posepath, featurepath, pointcloudpath);

    }

   
    system("pause");
    return EXIT_SUCCESS;
}






