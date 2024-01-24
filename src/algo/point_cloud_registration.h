#ifndef SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H
#define SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H

#include <vector>
#include "registration/registration_PPF_Match.h"

namespace scan3d
{
	struct registration_parameter{
		double down_sampling_rate;//指点云体素网格化过程中体素的边长大小，是点云包围盒的边长的相对比。推荐范围0.02-0.08。
		double PPF_distance;// PPF离散的距离尺度，推荐值0.01-0.04
		double PPF_angle;//PPF离散的角度尺度，推荐值60-90。
		double referencepoint_rate;//参考点选择比例，推荐值0.05-0.2。	
		double votingangle;//投票的角度，计算累积矩阵
		double clusterdistance;
		int ICPiterations;
		double DenoisingConfidence;
		int denoisingneighborhood;
		int votepeaks_number;
		double votepeakseffective;
		double PPF_weight;
		std::string gorundturth_path;

	};
	class PointCloud;
	class PointCloudRegistration 
	{
	public:
		/// 功能：实现将第一帧点云与其他帧点云配准，主要用于求取模型与点云之间的变换矩阵，也可用在模型上添加补充点云
		/// @参数 cloud: 输入所要配准的点云列表，要求点云与第一帧有重叠。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration_fine(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		/// 功能：实现将序列点云之间的配准，相邻点云配准，主要利用了粗配准和icp精配准，并进行了全局优化
		/// @参数 cloud: 输入所要配准的点云列表，要求点云是有重叠的序列。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		/// 功能：实现将序列点云之间的配准，相邻点云配准，只icp精配准，同意进行进行了全局优化
		/// @参数 cloud: 输入所要配准的点云列表，要求点云是有重叠的序列。 
		/// @参数 newcloud: 配准后的点云列表。
		/// @参数 para: 参数结构体。
		/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
		static void Registration_ICP_Princeton(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);
		
		static void EstimationLineMODO(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose);

		static void Registration_PPF(std::string dataname, std::string savename, std::vector<double>& value);
		static void Registration_PPF_noise(std::string dataname, std::vector<double> &value2);
		static void Registration_PPF_KITTI(std::string dataname, std::string savename, MatrixXi r_pair, double value);
		static void Registration_PPF_3DLoMatch(std::string dataname, std::string savename, MatrixXi r_pair, double value);
		static void Registration_PPF_WUSTL(std::string dataname, std::string savename, MatrixXi r_pair, double value);
		static void Registration_PPF_inlier(std::string dataname, std::string savename, MatrixXi r_pair, double value);
		static void Registration_PPF_Dome(std::string source, std::string target);

		static void Registration_DP();
		static void Registration_DP_KITTI();

		void cal_FPFH(std::string dataname, int pointnum,bool withnormal);
		void cal_point_cloud_FPFH(std::string filename, int pointnum, bool withnormal);
		void cal_trans(std::string dataname);
		void cal_reT(std::string dataname);
		void cal_pose();
		
		static bool savePoses(const std::vector<std::string> &poseName, const std::vector<Matrix4d> &pose, const std::string filepath);
	};
}
#endif //SCAN3D_ALGO_POINT_CLOUD_REGISTRATION_H