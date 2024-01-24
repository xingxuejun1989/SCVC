#include "registration_stdafx.h"
#include "registration_ObjectOpse.h"

namespace play3d {
	namespace ppf_match
	{


		void Pose3D::updatePose(Matrix4d& NewPose)
		{
			Matrix3d R;

			pose = NewPose;
			poseToRT(pose, R, t);

			// compute the angle
			const double trace = R.trace();

			if (fabs(trace - 3) <= EPS)
			{
				angle = 0;
			}
			else
				if (fabs(trace + 1) <= EPS)
				{
					angle = M_PI;
				}
				else
				{
					angle = (acos((trace - 1) / 2));
				}

			// compute the quaternion
			dcmToQuat(R, q);
		}

		void Pose3D::updatePose(Matrix3d& NewR, Vector3d& NewT)
		{
			rtToPose(NewR, NewT, pose);

			// compute the angle
			const double trace = NewR.trace();

			if (fabs(trace - 3) <= EPS)
			{
				angle = 0;
			}
			else
				if (fabs(trace + 1) <= EPS)
				{
					angle = M_PI;
				}
				else
				{
					angle = (acos((trace - 1) / 2));
				}

			// compute the quaternion
			dcmToQuat(NewR, q);
		}

		void Pose3D::updatePoseQuat(Vector4d& Q, Vector3d& NewT)
		{
			Matrix3d NewR;

			quatToDCM(Q, NewR);
			q = Q;

			rtToPose(NewR, NewT, pose);

			// compute the angle
			const double trace = NewR.trace();

			if (fabs(trace - 3) <= EPS)
			{
				angle = 0;
			}
			else
			{
				if (fabs(trace + 1) <= EPS)
				{
					angle = M_PI;
				}
				else
				{
					angle = (acos((trace - 1) / 2));
				}
			}
		}


		void Pose3D::appendPose(Matrix4d& IncrementalPose)
		{
			Matrix3d R;
			Matrix4d PoseFull = IncrementalPose * this->pose;

			poseToRT(PoseFull, R, t);

			// compute the angle
			const double trace = R.trace();

			if (fabs(trace - 3) <= EPS)
			{
				angle = 0;
			}
			else
				if (fabs(trace + 1) <= EPS)
				{
					angle = M_PI;
				}
				else
				{
					angle = (acos((trace - 1) / 2));
				}

			// compute the quaternion
			dcmToQuat(R, q);

			pose = PoseFull;
		}

		Pose3DPtr Pose3D::clone()
		{
			//Ptr<Pose3D> new_pose(new Pose3D(alpha, modelIndex, numVotes));//opencv的智能指针
			std::shared_ptr<Pose3D> new_pose(new Pose3D(alpha, modelIndex, numVotes));
			new_pose->pose = this->pose;
			new_pose->q = q;
			new_pose->t = t;
			new_pose->angle = angle;

			return new_pose;
		}

		void Pose3D::printPose()
		{
			printf("\n-- Pose to Model Index %d: NumVotes = %d, Residual = %f, overlap = %f\n", (uint)this->modelIndex, (uint)this->numVotes, this->residual, this->overlap);
			std::cout << this->pose << std::endl;
		}

		int Pose3D::writePose(FILE* f)
		{
			int POSE_MAGIC = 7673;
			fwrite(&POSE_MAGIC, sizeof(int), 1, f);
			fwrite(&angle, sizeof(double), 1, f);
			fwrite(&numVotes, sizeof(int), 1, f);
			fwrite(&modelIndex, sizeof(int), 1, f);
			fwrite(&pose(0, 0), sizeof(double) * 16, 1, f);//初始位置可能有问题的
			fwrite(&t(0), sizeof(double) * 3, 1, f);
			fwrite(&q(0), sizeof(double) * 4, 1, f);
			fwrite(&residual, sizeof(double), 1, f);
			return 0;
		}

		int Pose3D::readPose(FILE* f)
		{
			int POSE_MAGIC = 7673, magic;

			size_t status = fread(&magic, sizeof(int), 1, f);
			if (status && magic == POSE_MAGIC)
			{
				status = fread(&angle, sizeof(double), 1, f);
				status = fread(&numVotes, sizeof(int), 1, f);
				status = fread(&modelIndex, sizeof(int), 1, f);
				status = fread(&pose(0, 0), sizeof(double) * 16, 1, f);
				status = fread(&t(0), sizeof(double) * 3, 1, f);
				status = fread(&q(0), sizeof(double) * 4, 1, f);
				status = fread(&residual, sizeof(double), 1, f);
				return 0;
			}

			return -1;
		}

		int Pose3D::writePose(const std::string& FileName)
		{
			FILE* f = fopen(FileName.c_str(), "wb");

			if (!f)
				return -1;

			int status = writePose(f);

			fclose(f);
			return status;
		}

		int Pose3D::readPose(const std::string& FileName)
		{
			FILE* f = fopen(FileName.c_str(), "rb");

			if (!f)
				return -1;

			int status = readPose(f);

			fclose(f);
			return status;
		}


		void PoseCluster3D::addPose(Pose3DPtr newPose)
		{
			poseList.push_back(newPose);
			this->numVotes += newPose->numVotes;
		};

		int PoseCluster3D::writePoseCluster(FILE* f)
		{
			int POSE_CLUSTER_MAGIC_IO = 8462597;
			fwrite(&POSE_CLUSTER_MAGIC_IO, sizeof(int), 1, f);
			fwrite(&id, sizeof(int), 1, f);
			fwrite(&numVotes, sizeof(int), 1, f);

			int numPoses = (int)poseList.size();
			fwrite(&numPoses, sizeof(int), 1, f);

			for (int i = 0; i < numPoses; i++)
				poseList[i]->writePose(f);

			return 0;
		}

		int PoseCluster3D::readPoseCluster(FILE* f)
		{
			// The magic values are only used to check the files
			int POSE_CLUSTER_MAGIC_IO = 8462597;
			int magic = 0, numPoses = 0;
			size_t status;
			status = fread(&magic, sizeof(int), 1, f);

			if (!status || magic != POSE_CLUSTER_MAGIC_IO)
				return -1;

			status = fread(&id, sizeof(int), 1, f);
			status = fread(&numVotes, sizeof(int), 1, f);
			status = fread(&numPoses, sizeof(int), 1, f);
			fclose(f);

			poseList.clear();
			poseList.resize(numPoses);
			for (size_t i = 0; i < poseList.size(); i++)
			{
				poseList[i] = Pose3DPtr(new Pose3D());
				poseList[i]->readPose(f);
			}

			return 0;
		}

		int PoseCluster3D::writePoseCluster(const std::string& FileName)
		{
			FILE* f = fopen(FileName.c_str(), "wb");

			if (!f)
				return -1;

			int status = writePoseCluster(f);

			fclose(f);
			return status;
		}

		int PoseCluster3D::readPoseCluster(const std::string& FileName)
		{
			FILE* f = fopen(FileName.c_str(), "rb");

			if (!f)
				return -1;

			int status = readPoseCluster(f);

			fclose(f);
			return status;
		}

	} // namespace ppf_match_3d
}
