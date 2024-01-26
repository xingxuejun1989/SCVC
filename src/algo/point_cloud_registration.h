#ifndef SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H
#define SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include "registration/registration_SCVC.h"

namespace registration
{
	struct registration_parameter{
		double down_sampling_rate;//指点云体素网格化过程中体素的边长大小，是点云包围盒的边长的相对比。推荐范围0.02-0.08。
		double SCVC_distance;// SCVC离散的距离尺度，推荐值0.01-0.04
		double SCVC_angle;//SCVC离散的角度尺度，推荐值60-90。
		double referencepoint_rate;//参考点选择比例，推荐值0.05-0.2。	
		double votingangle;//投票的角度，计算累积矩阵
		double clusterdistance;
		int ICPiterations;
		double DenoisingConfidence;
		int denoisingneighborhood;
		int votepeaks_number;
		double votepeakseffective;
		double SCVC_weight;
		std::string gorundturth_path;

	};

	class PointCloudRegistration 
	{
	public:
		
	
		static void Registration_3DMatch3DLoMatch();
		static void Registration_KITTI(std::string featuretype, std::string posepath);
	
	};
}
#endif //SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H