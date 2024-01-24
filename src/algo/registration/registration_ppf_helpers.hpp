

#ifndef __OPENCV_SURFACE_MATCHING_HELPERS_HPP__
#define __OPENCV_SURFACE_MATCHING_HELPERS_HPP__



#include "FLANN/algorithms/dist.h"
#include "Eigen/Dense"
#include"flann/flann.hpp"
//#include "3rd_party/eigen/Eigen/Sparse"
#include <sstream>  // flann dependency, needed in precomp now


using namespace Eigen;

namespace play3d {
	namespace ppf_match
	{
		
		typedef flann::Index<flann::L2<float> > KDTree;
		/**
		*  @构建kd树
		*  @param [in]
		*  @param [in]
		*  @return Returns
		*/

		KDTree* BuildKDTree(const MatrixXf& data);

		/**
		*  @查询
		*  @param [in]
		*  @param [in]
		*  @return Returns
		*/

		void SearchKDTree(KDTree* tree,
			const MatrixXf & input,
			std::vector<std::vector<int>>& indices,
			std::vector<std::vector<float>>& dists,
			int nn);
		void  radiusSearchhKDTree(KDTree* tree,
			const MatrixXf & input,
			std::vector<std::vector<int>>& indices,
			std::vector<std::vector<float>>& dists, float radius);
		//void  cuvrefeatrue(KDTree * tree, MatrixXf & Model, int nn, float radius, std::vector<std::vector<float>>& model_curve_feature);
		//void  cuvrefeatrue2(KDTree * tree, MatrixXf & Model, int nn, float radius, std::vector<std::vector<float>>& model_curve_feature);
		//void  cuvrefeatrue_8v(KDTree* tree, MatrixXf& Model, int nn, float radius, MatrixXf& model_curve_feature);
		//void  normal(KDTree* tree, MatrixXf& Model, int nn, MatrixXf& normal, double o);
		//void kmeans(MatrixXf feature, int k, VectorXi& classid, MatrixXf& center, double error);
		//最小二乘法拟合平面
		//void  leastsquarePlane(const MatrixXf & Model, Vector4d & coff, int rownum);
		bool  CreatePlane(const Matrix3f & Model,Vector3f direction, Vector4d & coff);
		//void  Ransac_Plane(MatrixXf & Model, Vector3f direction,double threshold, double gridstepleng, int itermum, std::vector<SPlane> & Plane, VectorXi  &planeindex);
		
		Vector3f world2screen(MatrixXf cam_K, Vector3f v, int w, int h);
		MatrixXf transformPose3f(MatrixXf pc, const Matrix4d& Pose);
		Vector3f barycentric(Vector3f A, Vector3f B, Vector3f C, Vector3f P);
		void triangle(Matrix3f p, MatrixXf& zbuffer, MatrixXf cam_K);
		/**
		罗德里格旋转公式
		*/
		void  Rodriguesrotationformula(const Vector3d  v1, const Vector3d v2, Matrix3d& rotation);
		void  Rodriguesrotationformula(const Vector3d  v1, const Vector3d v2, const Vector3d axle, Matrix3d& rotation);
		//std::vector<int> random_permun(int n);
		
		void WriteBMP(char* img, int w, int h, const char* filename);
		///MatrixXf loadPLYSimple_face(const char* fileName, MatrixXi& face);
		MatrixXf loadPLYSimple(const char* fileName, int withNormals = 0);
		MatrixXf loadPLYSimple_bin(const char* fileName, int withNormals=0);
		MatrixXf loadXYZSimple(const char* fileName);
		/**
		 *  @brief Write a point cloud to PLY file
		 *  @param [in] PC Input point cloud
		 *  @param [in] fileName The PLY model file to write
		*/
		std::string remove(const std::string& text, char sep);
		std::vector<std::string> split(const std::string &text, char sep);
		void writePLY(MatrixXf PC, const char* fileName);
		void writePLYRGB(MatrixXf PC, const char* FileName);
		//MatrixXf loadRGBD(const char* fileName, MatrixXf cam_K, MatrixXf& depthimage, MatrixXf& depthimagen1, MatrixXf& depthimagen2,
		//	MatrixXf& depthimagen3, double maxlength);
		//void writePLYRGBnor(MatrixXf PC, const char* FileName);
		
		//MatrixXf sampleBin(MatrixXf PC, int sampleStep);
		/**
		*  @brief Used for debbuging pruposes, writes a point cloud to a PLY file with the tip
		*  of the normal vectors as visible red points
		*  @param [in] PC Input point cloud
		*  @param [in] fileName The PLY model file to write
		*/
		//void writePLYVisibleNormals(MatrixXf PC, const char* fileName);
		double calvoerlap(MatrixXf Model1, MatrixXf Model2,float distance);
		void  normal(MatrixXf& Model, Vector3f centerpoint);
		MatrixXf normal(MatrixXf pc, Vector3f centerpoint, int nei, double o);
		MatrixXf  normal(std::vector<RowVector3f> data, Vector3f centerpoint,int nei,double o);
		//MatrixXf samplePCUniformInd(MatrixXf PC, int sampleStep, std::vector<int>& indices);

		/**
		 *  Sample a point cloud using uniform steps
		 *  @param [in] pc Input point cloud
		 *  @param [in] xrange X components (min and max) of the bounding box of the model
		 *  @param [in] yrange Y components (min and max) of the bounding box of the model
		 *  @param [in] zrange Z components (min and max) of the bounding box of the model
		 *  @param [in] sample_step_relative The point cloud is sampled such that all points
		 *  have a certain minimum distance. This minimum distance is determined relatively using
		 *  the parameter sample_step_relative.
		 *  @param [in] weightByCenter The contribution of the quantized data points can be weighted
		 *  by the distance to the origin. This parameter enables/disables the use of weighting.
		 *  @return Sampled point cloud
		*/
		MatrixXf samplePCByQuantization(MatrixXf pc, Vector2f & xrange, Vector2f& yrange, Vector2f& zrange, float sample_step_relative, int weightByCenter = 0);
		MatrixXf sample_FSP(MatrixXf pc, int sample_pn);
		MatrixXf samplePCByQuantization_normal(MatrixXf pc, Vector2f& xrange, Vector2f& yrange, Vector2f& zrange, float sampleStep, float anglethreshold, int level, VectorXi& sumnum);
		//MatrixXf BinningDenormalnoise(MatrixXf &pc, float sampleStep);

		void computeBboxStd(MatrixXf pc, Vector2f& xRange, Vector2f& yRange, Vector2f& zRange);

		//void* indexPCFlann(MatrixXf pc);
		//void destroyFlann(void* flannIndex);
		////void queryPCFlann(void* flannIndex, Mat& pc, Mat& indices, Mat& distances);
		//void queryPCFlann(void* flannIndex, Mat& pc, Mat& indices, Mat& distances, const int numNeighbors);

		//MatrixXf normalizePCCoeff(MatrixXf pc, float scale, float* Cx, float* Cy, float* Cz, float* MinVal, float* MaxVal);
		//MatrixXf transPCCoeff(MatrixXf pc, float scale, float Cx, float Cy, float Cz, float MinVal, float MaxVal);

		/**
		 *  Transforms the point cloud with a given a homogeneous 4x4 pose matrix (in double precision)
		 *  @param [in] pc Input point cloud (CV_32F family). Point clouds with 3 or 6 elements per
		 *  row are expected. In the case where the normals are provided, they are also rotated to be
		 *  compatible with the entire transformation
		 *  @param [in] Pose 4x4 pose matrix, but linearized in row-major form.
		 *  @return Transformed point cloud
		*/
		MatrixXf transformPCPose(MatrixXf & pc, const Matrix4d& Pose);
		MatrixXf transformPCPosecood(MatrixXf pc, const Matrix4d& Pose);

		//void addNoisePC(MatrixXf &pc, double scale);

		MatrixXf StatisticsDenoise(MatrixXf& pc_s, int NumNeighbors, double rate);
		//MatrixXf StatisticsDenoiseNormal(MatrixXf &pc_s, int NumNeighbors, double rate, double angle);
		//MatrixXf BinningDenoise(MatrixXf &pc_s, float sampleStep,double rate);
		//MatrixXf BinningDenoiseNormal(MatrixXf &pc, float sampleStep, double rate,double angle);

		double GaussianNormallyDistributed_Fast(double x);
		double GaussianNormallyDistributed_Slow(double x);
		/**
		 *  Generate a random 4x4 pose matrix
		 *  @param [out] Pose The random pose
		*/
		// void getRandomPose(Matx44d& Pose);

		/**
		 *  Adds a uniform noise in the given scale to the input point cloud
		 *  @param [in] pc Input point cloud (CV_32F family).
		 *  @param [in] scale Input scale of the noise. The larger the scale, the more noisy the output
		*/
		// Mat addNoisePC(Mat pc, double scale);

		/**
		 *  @brief Compute the normals of an arbitrary point cloud
		 *  computeNormalsPC3d uses a plane fitting approach to smoothly compute
		 *  local normals. Normals are obtained through the eigenvector of the covariance
		 *  matrix, corresponding to the smallest eigen value.
		 *  If PCNormals is provided to be an Nx6 matrix, then no new allocation
		 *  is made, instead the existing memory is overwritten.
		 *  @param [in] PC Input point cloud to compute the normals for.
		 *  @param [out] PCNormals Output point cloud
		 *  @param [in] NumNeighbors Number of neighbors to take into account in a local region
		 *  @param [in] FlipViewpoint Should normals be flipped to a viewing direction?
		 *  @param [in] viewpoint
		 *  @return Returns 0 on success
		 */
		 //int computeNormalsPC3d(const Mat& PC, CV_OUT Mat& PCNormals, const int NumNeighbors, const bool FlipViewpoint, const Vec3f& viewpoint);

		//! @}

		/**  计算RGBD法向  Function that multiplies K_inv by a vector. It is just meant to speed up the product as we know
		* that K_inv is upper triangular and K_inv(2,2)=1
		* @param K_inv
		* @param a
		* @param b
		* @param c
		* @param res
		*/
		//void  multiply_by_K_inv(Matrix3f & K_inv, double a, double b, double c, Vector3d & res);
		/** Compute the normals
		* @param r
		* @param normals the output normals
		*/
		//MatrixXf computeImpl(const int rows, const int cols, const VectorXf  &depth, const MatrixXf  &cam_K, MatrixXf & normals);

		

	} // namespace ppf_match_3d

}
#endif
