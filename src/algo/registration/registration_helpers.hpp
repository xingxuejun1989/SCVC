#ifndef __OPENCV_SURFACE_MATCHING_HELPERS_HPP__
#define __OPENCV_SURFACE_MATCHING_HELPERS_HPP__



#include "../3rdParty/kd_tree/FLANN/algorithms/dist.h"
#include"../3rdParty/kd_tree/FLANN/flann.hpp"
//#include "Eigen/Dense"

#include <sstream>  // flann dependency, needed in precomp now
#include"../3rdParty/eigen-3.4.0/Eigen/Dense"


using namespace Eigen;


namespace scvc
	{
		
		typedef flann::Index<flann::L2<float> > KDTree;

		KDTree* BuildKDTree(const MatrixXf& data);



		void SearchKDTree(KDTree* tree,
			const MatrixXf & input,
			std::vector<std::vector<int>>& indices,
			std::vector<std::vector<float>>& dists,
			int nn);
		void  radiusSearchhKDTree(KDTree* tree,
			const MatrixXf & input,
			std::vector<std::vector<int>>& indices,
			std::vector<std::vector<float>>& dists, float radius);
		
		const float EPS = 1.192092896e-07;

	
		static inline void poseToR(const Matrix4d& Pose, Matrix3d& R)
		{
			R = Pose.block(0, 0, 3, 3);

			//Mat(Pose).rowRange(0, 3).colRange(0, 3).copyTo(R);
		}

		static inline void poseToRT(const Matrix4d& Pose, Matrix3d& R, Vector3d& t)
		{
			R = Pose.block(0, 0, 3, 3);
			t = Pose.block(0, 3, 3, 1);
			// Mat(Pose).rowRange(0, 3).colRange(3, 4).copyTo(t);
		}

		
		MatrixXf loadPLYSimple(const char* fileName, int withNormals = 0);
		MatrixXf loadPLYSimple_bin(const char* fileName, int withNormals=0);
		
		
		std::vector<std::string> split(const std::string &text, char sep);
		void writePLY(MatrixXf PC, const char* fileName);

	
		MatrixXf  normal(MatrixXf pc, Vector3f centerpoint, int nei, double o);
	

		MatrixXf samplePCByQuantization(MatrixXf pc, Vector2f & xrange, Vector2f& yrange, Vector2f& zrange, float sample_step_relative, int weightByCenter = 0);
		
		

		void computeBboxStd(MatrixXf pc, Vector2f& xRange, Vector2f& yRange, Vector2f& zRange);


		MatrixXf transformPCPose(MatrixXf & pc, const Matrix4d& Pose);
		MatrixXf transformPCPosecood(MatrixXf pc, const Matrix4d& Pose);


		MatrixXf StatisticsDenoise(MatrixXf& pc_s, int NumNeighbors, double rate);
	

		//double GaussianNormallyDistributed_Fast(double x);
		//double GaussianNormallyDistributed_Slow(double x);
		

	} // namespace 


#endif
