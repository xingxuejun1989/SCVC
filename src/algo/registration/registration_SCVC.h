
#ifndef __SCVC_MATCH_3D_H__
#define __SCVC_MATCH_3D_H__


#include <vector>
//#include "registration_ObjectOpse.h"
#include "Eigen/Dense"
#include "registration_stdafx.h"
#include "registration_helpers.hpp"
#include "FLANN/flann.hpp"
#include<omp.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>

using namespace Eigen;

static omp_lock_t lock;

	
namespace scvc
{
	typedef unsigned long  uint;
	typedef unsigned long long KeyType;
	typedef Eigen::Matrix<double, Eigen::Dynamic, 3> PointCloud;
	typedef Eigen::Matrix<double, 3, 3> RotMatrix;               
	typedef Eigen::Matrix<double, 3, 1> TransMatrix;
		/**
		* @brief Struct, holding a node in the hashtable
			*/

		typedef struct THash_R
		{
			int is;
			int it;
			//int ppfInd;
			unsigned long long ppfindex;
		} THash_R;


		typedef struct SCVC_R_SCENE
		{
			MatrixXf  original_scene;
			MatrixXf  downsample_scene;
			MatrixXf  downsample_scene_f;
			MatrixXf  feature_point;
			double scene_samplingdistance;
			double scene_diameter;
			KDTree* down_kdt;
		};

		struct hashtable_list{
			std::vector<int> index;
			unsigned int No;
		};

		typedef struct SCVC_R_MODEL
		{
			std::string path;
			MatrixXf  original_model;
			MatrixXf  downsample_model;
			MatrixXf  downsample_model_f;
			MatrixXf  feature_point;
			Matrix4d  ferencepoint;

			double model_diameter;
			Vector3d center;

			std::vector<hashtable_list> hash_table;
			THash_R* hash_nodes;

			double model_samplingdistance;
			double model_feat_samplingdistance;
			double model_ppf_distance_step;

		};

		//配准数据结构
		typedef struct SCVC_MODEL_R
		{
			SCVC_R_MODEL model;
			SCVC_R_SCENE scene;

		};
		

		typedef struct ppf_model_parameter
		{
			double relativeSamplingnum;
			double SamplingNumber_f;
			//double SCVC_distance;
			double SCVC_anglenumber;
			double Sampling_threshold_Angles;
			double line_minlenght;
			int voting_top;
			int feature_num;
			int clutter_veri;
			double overlap_angle;
			double overlap_dis;
			double overlap_vl_angle;
			double overlap_n;
		};
		typedef struct ppf_scenematching_parameter
		{
			double relativeSamplingStep;	
			int voteAnglenum;
			double  samplingrate;
			double ICP_nbIterations;
			double  Clusteringdistance;
			int votepeaks_number;
			double votepeakseffective;
			double SCVC_weight;

		};
	
		typedef struct Correspondence
		{
			int model;
			int scene;
			float  value;
		};

		
		struct SPose3D_R
		{
			Matrix4d  Pose;
			Vector3d aulerangle;
			Vector3d translation;
			double overlap;
			double occlude;
			double numVotes;
			std::vector<int> modelpair;
			std::vector<int> scenepair;
			std::vector<double> corresweight;
			int corresindex;
		};

		class  SCVC_PSO_Match
		{
		public:
			SCVC_PSO_Match();
			SCVC_PSO_Match(ppf_model_parameter model_para);
			~SCVC_PSO_Match();
			
			void trainModel_SCVC_R();

			void Modeldown(const MatrixXf& original_model,int const feature_rows);
			void Scenedown(const MatrixXf& original_scene, int const feature_rows);
			void Model_Scene_down(const MatrixXf& original_model,const MatrixXf& original_scene);
			void FeatureMatch();
			void FeatureMatch(const MatrixXf& feature_model, const MatrixXf& feature_scene);
			void FeatureMatch_cos(const MatrixXf& feature_model, const MatrixXf& feature_scene);
			void FeatureMatch_match(const VectorXf& corr_scores);
			Matrix4d  ICP_Optimizing(MatrixXf source, MatrixXf target, int max_iter);
			void set_groundturth_pose(Matrix4d pose)
			{
				groundtruth_pose= pose;
				groundtruth_enable = true;
			}
			
			void match_SCVC_registration2(std::vector<SPose3D_R>& resultsS, const ppf_scenematching_parameter parameter);
			
		

		private:
			void  Optimizing_registration(SPose3D_R& resultsS, int max_iter, bool down);
			void  Pose_Verfication2(SPose3D_R& resultsS);
			void  Pose_Verfication3(SPose3D_R& resultsS);
			void  Pose_Verfication(SPose3D_R& resultsS);
			
			void PoseMatrixToAngle(Matrix4d pose, Vector3d& angle, Vector3d& translation);
			static void  computeSCVCFeatures(const Vector3f& p1, const Vector3f& n1,const Vector3f& p2, const Vector3f& n2,Vector4d& f);
			double feature_cos(pcl::FPFHSignature33 &a, pcl::FPFHSignature33 &b);
			double feature_cos(RowVectorXf& a, RowVectorXf& b);
			Matrix4d   SparceICP_Optimizing(MatrixXf source, MatrixXf target, int nbIterations);

		
			
		public:
			SCVC_MODEL_R _R_model_para;
		private:
			ppf_model_parameter model_parameter;
			Matrix4d groundtruth_pose;
			bool groundtruth_enable;
		
			std::vector <Correspondence>  Correspondencepoint;
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr model_features;
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features;
		};

		
		//! @}

	} // namespace ppf_match_3d


#endif
