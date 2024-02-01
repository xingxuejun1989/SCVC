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
        std::string featuretype = "fpfh";//fcgf fpfh
        std::string posepath = "E:/pointclouddata/3Dmatch/benchmarks/";
        posepath = posepath + datasetname + "/";
        std::string featruepath = "E:/pointclouddata/3Dmatch/noise/"+ featuretype +"/";
        std::string pointcloudpath = "E:/pointclouddata/3Dmatch/pointcloud/";
        a.Registration_3DMatch3DLoMatch(datasetname, featuretype, posepath, featruepath, pointcloudpath);

    }
    else if(datasetname == "KITTI")
    {
        
        std::string posepath = "E:/pointclouddata/data_odometry_velodyne/icp";

        std::string pointcloudpath = "E:/pointclouddata/data_odometry_velodyne/sequences/";
        std::string featuretype = "fpfh";//fcgf fpfh
        std::string featurepath = "E:/pointclouddata/data_odometry_velodyne/feat_"+ featuretype +"_txt/";
        //std::string featurepath = "E:/pointclouddata/data_odometry_velodyne/feat_fpfh_txt/";
         a.Registration_KITTI(featuretype, posepath, featurepath, pointcloudpath);

    }

   
    system("pause");
    return EXIT_SUCCESS;
}






