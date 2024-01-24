
#ifndef __OBJECTOPSE_H__
#define __OBJECTOPSE_H__

#include <vector>
#include <string>
#include<memory>
//#include"Eigen/Core"
#include<Eigen/Core>
using namespace Eigen;
namespace play3d {
	namespace ppf_match
	{

		template<typename T>
		class Deleter
		{
		public:
			void operator () (T* x) const
			{
				if (x != NULL)
				{
					std::cout << __LINE__ << std::endl;
					delete x;
					x = NULL;
				}
			}
		};
		//! @addtogroup surface_matching
		//! @{
		struct Partigrouppara
		{
			std::vector<int>  sceneindex;
			std::vector<Vector3f>  Particlevel;
			std::vector<int> maxscoreindex;
			std::vector<double>  weight;
			double velweight;
			int groupscoreindex;//群的最高得索引
			double groupscoreweight;//群的最高得分权值
			double c1;
			double c2;
		};
		struct SPose3D
		{
			Matrix4d  Pose;
			Vector3d aulerangle;
			Vector3d translation;
			double overlap;
			Matrix4d  oldPose;
			double  oldoverlap;
			float planterate;
			size_t numVotes;
			int   overnum;
			std::vector<Vector3f> modelpair;
			std::vector<Vector3f> scenepair;
			std::vector<int> overlapindex;
			Matrix3d referenceopint;
			Matrix3i referenceopinti;
			int model_id;
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

		struct SPlane
		{
			int index;
			Vector4d  coff;
			Matrix3f  point;
			Matrix3d  rotation;
			double area;
			double planeratio;
		};

		class Pose3D;
		typedef std::shared_ptr<Pose3D> Pose3DPtr;//由opencv的智能指针ptr改来，可能会出问题 需要验证

		class PoseCluster3D;
		typedef std::shared_ptr<PoseCluster3D> PoseCluster3DPtr;//由opencv的智能指针ptr改来，可能会出问题 需要验证

		/**
		* @brief Class, allowing the storage of a pose. The data structure stores both
		* the quaternions and the matrix forms. It supports IO functionality together with
		* various helper methods to work with poses
		*
		*/
		class  Pose3D
		{
		public:
			Pose3D()
			{
				alpha = 0;
				modelIndex = 0;
				numVotes = 0;
				residual = 0;

				pose.setZero();
			}

			Pose3D(double Alpha, size_t ModelIndex = 0, size_t NumVotes = 0)
			{
				alpha = Alpha;
				modelIndex = ModelIndex;
				numVotes = NumVotes;
				residual = 0;

				pose.setZero();
			}

			/**
			*  \brief Updates the pose with the new one
			*  \param [in] NewPose New pose to overwrite
			*/
			void updatePose(Matrix4d& NewPose);

			/**
			*  \brief Updates the pose with the new one
			*/
			void updatePose(Matrix3d& NewR, Vector3d& NewT);

			/**
			*  \brief Updates the pose with the new one, but this time using quaternions to represent rotation
			*/
			void updatePoseQuat(Vector4d& Q, Vector3d& NewT);

			/**
			*  \brief Left multiplies the existing pose in order to update the transformation
			*  \param [in] IncrementalPose New pose to apply
			*/
			void appendPose(Matrix4d& IncrementalPose);
			void printPose();

			Pose3DPtr clone();

			int writePose(FILE* f);
			int readPose(FILE* f);
			int writePose(const std::string& FileName);
			int readPose(const std::string& FileName);

			virtual ~Pose3D() {}

			double alpha, residual;
			size_t modelIndex, numVotes;
			Matrix4d pose;
			float overlap;
			double angle;
			Vector3d t;
			Vector4d q;
			std::vector<Vector3f> modelpair;
			std::vector<Vector3f> scenepair;
		};

		/**
		* @brief When multiple poses (see Pose3D) are grouped together (contribute to the same transformation)
		* pose clusters occur. This class is a general container for such groups of poses. It is possible to store,
		* load and perform IO on these poses.
		*/
		class  PoseCluster3D
		{
		public:
			PoseCluster3D()
			{
				numVotes = 0;
				id = 0;
			}

			PoseCluster3D(Pose3DPtr newPose)
			{
				poseList.clear();
				poseList.push_back(newPose);
				numVotes = newPose->numVotes;
				id = 0;
			}

			PoseCluster3D(Pose3DPtr newPose, int newId)
			{
				poseList.push_back(newPose);
				this->numVotes = newPose->numVotes;
				this->id = newId;
			}

			virtual ~PoseCluster3D()
			{}

			/**
			*  \brief Adds a new pose to the cluster. The pose should be "close" to the mean poses
			*  in order to preserve the consistency
			*  \param [in] newPose Pose to add to the cluster
			*/
			void addPose(Pose3DPtr newPose);

			int writePoseCluster(FILE* f);
			int readPoseCluster(FILE* f);
			int writePoseCluster(const std::string& FileName);
			int readPoseCluster(const std::string& FileName);

			std::vector<Pose3DPtr> poseList;
			size_t numVotes;
			int id;
		};

		//! @}

	} // namespace ppf_match_3d
}
#endif
