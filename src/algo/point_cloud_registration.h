#ifndef SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H
#define SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include "registration/registration_SCVC.h"

namespace registration
{
	

	class PointCloudRegistration 
	{
	public:
		
	
		static void Registration_3DMatch3DLoMatch(std::string dataset,std::string featuretype, std::string posepath, std::string featruepath, std::string pointcloudpath);
		static void Registration_KITTI(std::string featuretype, std::string posepath, std::string featruepath, std::string pointcloudpath);
	
	};
}
#endif //SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H