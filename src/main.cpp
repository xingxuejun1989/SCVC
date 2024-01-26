#include <vector>
#include <Eigen/Core>
#include<fstream>
#include <io.h>
#include"algo/point_cloud_registration.h"
#include<string>


int main(int argc, char** argv)
{
    std::string datasetname = "3DMatch";

    registration::PointCloudRegistration a;

    if (datasetname == "3DMatch" || datasetname == "3DMatch")
    {
        a.Registration_3DMatch3DLoMatch();

    }
    else if(datasetname == "KITTI")
    {
        std::string featuretype = "fcgf";//fcgf fpfh
        std::string posepath = "E:/pointclouddata/data_odometry_velodyne/icp";
         a.Registration_KITTI(featuretype, posepath);

    }

   
    system("pause");
    return EXIT_SUCCESS;
}






