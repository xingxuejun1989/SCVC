//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Author: Tolga Birdal <tbirdal AT gmail.com>

#ifndef __MATCHING_UTILS_HPP_
#define __MATCHING_UTILS_HPP_


#include "Eigen/Dense"
#include <cmath>
#include <cstdio>

using namespace Eigen;
namespace play3d {
	namespace ppf_match
	{
		typedef unsigned long uint;
		const float EPS = 1.192092896e-07F;        /* smallest such that 1.0+FLT_EPSILON != 1.0 */

#ifndef M_PI
#define M_PI  3.1415926535897932384626433832795
#endif

		static inline void TNormalize3(Vector3d& v)
		{
			double norm = v.norm();
			if (norm > EPS)
			{
				v *= 1.0 / norm;
			}
		}

		/**
		 *  \brief Calculate angle between two normalized vectors
		 *
		 *  \param [in] a normalized vector
		 *  \param [in] b normalized vector
		 *  \return angle between a and b vectors in radians
		 */
		static inline double TAngle3Normalized(const Vector3f& a, const Vector3f& b)
		{
			/*
			 angle = atan2(a dot b, |a x b|) # Bertram (accidental mistake)
			 angle = atan2(|a x b|, a dot b) # Tolga Birdal (correction)
			 angle = acos(a dot b)           # Hamdi Sahloul (simplification, a & b are normalized)
			*/
			double an= acos(a.dot(b));
			//if (an > M_PI / 2.0)
			//	an = M_PI - an;
			return an;
		}
		/*
		*rtToPose 是将R,t和并未4*4的转换矩阵
		*/
		static inline void rtToPose(const Matrix3d& R, const Vector3d& t, Matrix4d& Pose)
		{
			//Matx34d P; OPENCV
			MatrixXd P(3, 4);

			P << R, t;//横行向合并
			Pose << P,
				RowVector4d(0, 0, 0, 1);
		}
		/*
		*poseToR 转换矩阵中提取旋转矩阵
		*/
		static inline void poseToR(const Matrix4d& Pose, Matrix3d& R)
		{
			R = Pose.block(0, 0, 3, 3);

			//Mat(Pose).rowRange(0, 3).colRange(0, 3).copyTo(R);
		}
		/*
		*poseToR 转换矩阵中提取旋转矩阵和平移矩阵
		*/
		static inline void poseToRT(const Matrix4d& Pose, Matrix3d& R, Vector3d& t)
		{
			poseToR(Pose, R);
			t = Pose.block(0, 3, 3, 1);
			// Mat(Pose).rowRange(0, 3).colRange(3, 4).copyTo(t);
		}

		/**
		 *  \brief Axis angle to rotation 绕轴旋转求R
		 */
		static inline void aaToR(const Vector3d& axis, double angle, Matrix3d& R)
		{
			const double sinA = sin(angle);
			const double cosA = cos(angle);
			const double cos1A = (1 - cosA);
			uint i, j;

			Matrix3d eye3d;
			eye3d.setIdentity();
			R = eye3d*cosA;

			for (i = 0; i < 3; i++)
				for (j = 0; j < 3; j++)
				{
					if (i != j)
					{
						// Symmetry skew matrix
						R(i, j) += (((i + 1) % 3 == j) ? -1 : 1) * sinA * axis[3 - i - j];
					}
					R(i, j) += cos1A * axis[i] * axis[j];
				}
		}

		/**
		 *  \brief Compute a rotation in order to rotate around X direction
		 *  计算绕x周方向旋转角度angle的旋转矩阵
		 */
		static inline void getUnitXRotation(double angle, Matrix3d& Rx)
		{
			const double sx = sin(angle);
			const double cx = cos(angle);
			Rx.setIdentity();
			// Mat(Rx.eye()).copyTo(Rx);
			Rx(1, 1) = cx;
			Rx(1, 2) = -sx;
			Rx(2, 1) = sx;
			Rx(2, 2) = cx;
		}

		/**
		*  \brief Compute a rotation in order to rotate around Y direction
		*  计算绕y周方向旋转角度angle的旋转矩阵
		*/
		static inline void getUnitYRotation(double angle, Matrix3d& Ry)
		{
			const double sy = sin(angle);
			const double cy = cos(angle);
			Ry.setIdentity();
			Ry(0, 0) = cy;
			Ry(0, 2) = sy;
			Ry(2, 0) = -sy;
			Ry(2, 2) = cy;
		}

		/**
		*  \brief Compute a rotation in order to rotate around Z direction
		   计算绕z周方向旋转角度angle的旋转矩阵
		*/
		static inline void getUnitZRotation(double angle, Matrix3d& Rz)
		{
			const double sz = sin(angle);
			const double cz = cos(angle);

			Rz.setIdentity();
			Rz(0, 0) = cz;
			Rz(0, 1) = -sz;
			Rz(1, 0) = sz;
			Rz(1, 1) = cz;
		}

		/**
		*  \brief Convert euler representation to rotation matrix
		*  将欧拉角转换为旋转矩阵
		*  \param [in] euler RPY angles
		*  \param [out] R 3x3 Rotation matrix
		*/
		static inline void eulerToDCM(const Vector3d& euler, Matrix3d& R)
		{
			Matrix3d Rx, Ry, Rz;

			getUnitXRotation(euler[0], Rx);
			getUnitYRotation(euler[1], Ry);
			getUnitZRotation(euler[2], Rz);
			R = Rx * (Ry * Rz);
		}


		/**
		 *  \brief Compute the transformation needed to rotate n1 onto x axis and p1 to origin 把法向旋转到x轴正向 点移动到原点
		 * \p1=n1 该算法错误
		 */
		static inline void computeTransformRT(const Vector3d& p1, const Vector3d& n1, Matrix3d& R, Vector3d& t)
		{
			//罗德里格斯公式Rodrigues rotation formula
			// dot product with x axis
			double angle = acos(n1(0));

			// cross product with x axis
			Vector3d axis(0, n1(2), -n1(1));

			// we try to project on the ground plane but it's already parallel
			if (n1(1) == 0 && n1(2) == 0)
			{
				axis(1) = 1;
				axis(2) = 0;
			}
			else
			{
				TNormalize3(axis);
			}

			aaToR(axis, angle, R);
			t = -R * p1;
		}

		/**
		 *  \brief Flip a normal to the viewing direction
		 *  \将法向翻转到视觉观测方向
		 *  \param [in] point Scene point
		 *  \param [in] vp view direction
		 *  \param [in] n normal
		 */
		static inline void flipNormalViewpoint(const Vector3f& point, const Vector3f& vp, Vector3f& n)
		{
			float cos_theta;

			// See if we need to flip any plane normals
			Vector3f diff = vp - point;

			// Dot product between the (viewpoint - point) and the plane normal 计算点积
			cos_theta = diff.dot(n);

			// Flip the plane normal
			if (cos_theta < 0)
			{
				n *= -1;
			}
		}

		/**
		 *  \brief Convert a rotation matrix to axis angle representation
		 *
		 *  \param [in] R Rotation matrix
		 *  \param [out] axis Axis vector
		 *  \param [out] angle Angle in radians
		 */
		static inline void dcmToAA(Matrix3d& R, Vector3d& axis, double *angle)
		{
			Vector3d a(R(2, 1) - R(2, 1),
				R(0, 2) - R(2, 0),
				R(1, 0) - R(0, 1));
			axis = a;
			TNormalize3(axis);
			*angle = acos(0.5 * (R.trace() - 1.0));
		}

		/**
		 *  \brief Convert axis angle representation to rotation matrix
		 *
		 *  \param [in] axis Axis Vector
		 *  \param [in] angle Angle (In radians)
		 *  \param [out] R 3x3 Rotation matrix
		 */
		static inline void aaToDCM(const Vector3d& axis, double angle, Matrix3d& R)
		{
			uint i, j;
			Matrix3d n;
			n.setZero();
			for (i = 0; i < 3; i++)
				for (j = 0; j < 3; j++)
					if (i != j)
						n(i, j) = (((i + 1) % 3 == j) ? -1 : 1) * axis[3 - i - j];
			R = Matrix3d::Identity() + sin(angle) * n + cos(angle) * n * n;
		}

		/**
		 *  \brief Convert a discrete cosine matrix to quaternion
		 *
		 *  \param [in] R Rotation Matrix
		 *  \param [in] q Quaternion
		 */
		static inline void dcmToQuat(Matrix3d& R, Vector4d& q)
		{
			double tr = R.trace();
			// Vector3d v(R(0, 0), R(1, 1), R(2, 2));
			// int idx = tr > 0.0 ? 3 : (int)(std::max_element(v.val, v.val + 3) - v.val);
			std::vector<double> v = { R(0, 0), R(1, 1), R(2, 2) };
			int idx = tr > 0.0 ? 3 : (int)(std::max_element(v.begin(), v.end()) - v.begin());
			double norm4 = q[(idx + 1) % 4] = 1.0 + (tr > 0.0 ? tr : 2 * R(idx, idx) - tr);
			int i, prev, next, step = idx % 2 ? 1 : -1, curr = 3;
			for (i = 0; i < 3; i++)
			{
				curr = (curr + step) % 4;
				next = (curr + 1) % 3, prev = (curr + 2) % 3;
				q[(idx + i + 2) % 4] = R(next, prev) + (tr > 0.0 || idx == curr ? -1 : 1) * R(prev, next);
			}
			q *= 0.5 / sqrt(norm4);
		}

		/**
		 *  \brief Convert quaternion to a discrete cosine matrix
		 *
		 *  \param [in] q Quaternion (w is at first element)
		 *  \param [in] R Rotation Matrix
		 *
		 */
		static inline void quatToDCM(Vector4d& q, Matrix3d& R)
		{
			Vector4d sq = q.array()*q.array();//q.mul(q);

			double tmp1, tmp2;

			R(0, 0) = sq[0] + sq[1] - sq[2] - sq[3]; // since norm(q) = 1
			R(1, 1) = sq[0] - sq[1] + sq[2] - sq[3];
			R(2, 2) = sq[0] - sq[1] - sq[2] + sq[3];

			tmp1 = q[1] * q[2];
			tmp2 = q[3] * q[0];

			R(0, 1) = 2.0 * (tmp1 + tmp2);
			R(1, 0) = 2.0 * (tmp1 - tmp2);

			tmp1 = q[1] * q[3];
			tmp2 = q[2] * q[0];

			R(0, 2) = 2.0 * (tmp1 - tmp2);
			R(2, 0) = 2.0 * (tmp1 + tmp2);

			tmp1 = q[2] * q[3];
			tmp2 = q[1] * q[0];

			R(1, 2) = 2.0 * (tmp1 + tmp2);
			R(2, 1) = 2.0 * (tmp1 - tmp2);
		}

	} // namespace ppf_match_3d

}

#endif
