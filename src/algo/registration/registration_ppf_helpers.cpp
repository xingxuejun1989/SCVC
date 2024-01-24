#include"registration_stdafx.h"
#include <random>
#include <ctime>
#include  <direct.h>  

namespace play3d {
namespace ppf_match
{

//typedef flann::L2<float> Distance_32F;
//typedef flann::Index< Distance_32F > FlannIndex;

void shuffle(int *array, size_t n);


  std::vector<std::string> split(const std::string &text, char sep) {
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

 std::string remove(const std::string& text, char sep) {
	  std::string str="";
	  std::size_t start = 0, end = text.length();
	  while ((end = text.find(sep, start)) != std::string::npos) {
		  str +=text.substr(start, end - start);
		  start = end + 1;
	  }
	  str += text.substr(start, end - start);
	  return str;
  }


KDTree* BuildKDTree(const  MatrixXf & data )
{
	int rows, dim;
	rows = (int)data.rows();
	dim = (int)data.cols();
	//std::cout << rows << " " << std::endl;
	flann::Matrix<float> dataset_mat(new float[rows*dim], rows, dim);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < dim; j++)
		{
			dataset_mat[i][j] = data(i, j);
			//std::cout << data(i, j) << "  ";
		}
		//std::cout << std::endl;
	}
		
	//KDTreeSingleIndexParams 为搜索最大叶子数
	KDTree* tree = new KDTree(dataset_mat, flann::KDTreeSingleIndexParams(64));
	tree->buildIndex();
	//std::cout << "test..." << tree->size() << std::endl;
	//tree = &temp_tree;
	delete[] dataset_mat.ptr();

	return tree;
}
void  radiusSearchhKDTree(KDTree* tree, const MatrixXf & input, std::vector<std::vector<int>>& indices,
	std::vector<std::vector<float>>& dists,float radius)
{
	int rows_t = input.rows();
	int dim = input.cols();


	flann::Matrix<float> query_mat(new float[rows_t*dim], rows_t, dim);
	for (int i = 0; i < rows_t; i++)
	{
		for (int j = 0; j <dim; j++)
		{
			query_mat[i][j] = input(i, j);
		}
	}

	indices.resize(rows_t);
	dists.resize(rows_t);

	tree->radiusSearch(query_mat, indices, dists, radius, flann::SearchParams(128));
	delete[] query_mat.ptr();
}

void SearchKDTree(KDTree* tree, const MatrixXf & input,
	std::vector<std::vector<int>>& indices,
	std::vector<std::vector<float>>& dists, int nn)
{
	int rows_t = input.rows();
	int dim = input.cols();
	//rows_t = 100;
	
	flann::Matrix<float> query_mat(new float[dim], 1,dim);
	flann::Matrix<int> indices_m(new int[nn], 1, nn);
	flann::Matrix<float> dists_m(new float[ nn],1, nn);

	indices.resize(rows_t);
	dists.resize(rows_t);

	for (int i = 0; i < rows_t; i++)
	{
		indices[i].resize(nn);
		dists[i].resize(nn);
		for (int j = 0; j <dim; j++)
		{
			query_mat[0][j] = input(i, j);	
			//std::cout << query_mat[0][j] << "  ";
		}
		//std::cout << i<<"  "<<std::endl;
		tree->knnSearch(query_mat, indices_m, dists_m, nn, flann::SearchParams(32));
		for (int j = 0; j < nn; j++)
		{
			indices[i][j] = indices_m[0][j];
			dists[i][j] = dists_m[0][j];
		}
	}
	//std::cout << rows_t * dim << std::endl;

	//std::cout << rows_t *nn<< std::endl;
	
	//std::cout << rows_t * nn << std::endl;

	delete[] query_mat.ptr();
	delete[] indices_m.ptr();
	delete[] dists_m.ptr();
}
/*
Eigen::Matrix4d GetTransformationOriginalScale(
	const Eigen::Matrix4d& transformation,
	const std::vector<Eigen::Vector3d>& pcd_mean_vec,
	const double scale_global) {
	Eigen::Matrix3d R = transformation.block<3, 3>(0, 0);
	Eigen::Vector3d t = transformation.block<3, 1>(0, 3);
	Eigen::Matrix4d transtemp = Eigen::Matrix4d::Zero();
	transtemp.block<3, 3>(0, 0) = R;
	transtemp.block<3, 1>(0, 3) =
		-R * pcd_mean_vec[1] + t * scale_global + pcd_mean_vec[0];
	transtemp(3, 3) = 1;
	return transtemp;
}
*/

std::tuple<std::vector<Eigen::Vector3d>, double, double> NormalizePointCloud(
	std::vector<Matrix<double, Dynamic, 6>>& point_cloud_vec) {
	int num = 2;
	double scale = 0;
	std::vector<Eigen::Vector3d> pcd_mean_vec;
	double scale_global, scale_start;
	bool use_absolute_scale_ = false;
	for (int i = 0; i < num; ++i) {
		double max_scale = 0.0;
		Eigen::Vector3d mean;
		mean.setZero();

		int npti = static_cast<int>(point_cloud_vec[i].rows());
		for (int ii = 0; ii < npti; ++ii)
			mean = mean + point_cloud_vec[i].row(ii).head(3).transpose();
		mean = mean / npti;
		pcd_mean_vec.push_back(mean);

		// utility::LogDebug("normalize points :: mean = [{:f} {:f} {:f}]",
		//                   mean(0), mean(1), mean(2));
		for (int ii = 0; ii < npti; ++ii)
			point_cloud_vec[i].row(ii).head(3) -= mean.transpose();

		for (int ii = 0; ii < npti; ++ii) {
			Eigen::Vector3d p(point_cloud_vec[i].row(ii).head(3).transpose());
			double temp = p.norm();
			if (temp > max_scale) max_scale = temp;
		}
		if (max_scale > scale) scale = max_scale;
	}

	if (use_absolute_scale_) {
		scale_global = 1.0;
		scale_start = scale;
	}
	else {
		scale_global = scale;
		scale_start = 1.0;
	}
	//utility::LogDebug("normalize points :: global scale : {:f}", scale_global);

	for (int i = 0; i < num; ++i) {
		int npti = static_cast<int>(point_cloud_vec[i].rows());
		for (int ii = 0; ii < npti; ++ii) {
			point_cloud_vec[i].row(ii).head(3) /= scale_global;
		}
	}
	return std::make_tuple(pcd_mean_vec, scale_global, scale_start);
}
//Eigen::Matrix4d OptimizePairwiseRegistration(
//	const std::vector<Matrix<double, Dynamic, 6>>& point_cloud_vec,
//	const std::vector<std::pair<int, int>>& corres,
//	double scale_start) 
//{
//	//utility::LogDebug("Pairwise rigid pose optimization");
//	double par = scale_start;
//	int iteration_number_ = 64;
//	bool decrease_mu_ = true;
//	double maximum_correspondence_distance_ = 0.025;
//	double division_factor_ = 1.4;
//
//	int numIter = iteration_number_;
//
//	int i = 0, j = 1;
//	Matrix<double, Dynamic, 3> point_cloud_copy_j = point_cloud_vec[j].block(0, 0, point_cloud_vec[j].rows(), 3);
//
//	if (corres.size() < 10) return Eigen::Matrix4d::Identity();
//
//	std::vector<double> s(corres.size(), 1.0);
//	Eigen::Matrix4d trans;
//	trans.setIdentity();
//
//	for (int itr = 0; itr < numIter; itr++) {
//		const int nvariable = 6;
//		Eigen::MatrixXd JTJ(nvariable, nvariable);
//		Eigen::MatrixXd JTr(nvariable, 1);
//		Eigen::MatrixXd J(nvariable, 1);
//		JTJ.setZero();
//		JTr.setZero();
//		double r = 0.0, r2 = 0.0;
//
//		for (size_t c = 0; c < corres.size(); c++) {
//			int ii = corres[c].first;
//			int jj = corres[c].second;
//			Eigen::Vector3d p, q;
//			p = point_cloud_vec[i].row(ii).head(3).transpose();
//			q = point_cloud_copy_j.row(jj).head(3).transpose();
//			Eigen::Vector3d rpq = p - q;
//
//			size_t c2 = c;
//			double temp = par / (rpq.dot(rpq) + par);
//			s[c2] = temp * temp;
//
//			J.setZero();
//			J(1) = -q(2);
//			J(2) = q(1);
//			J(3) = -1;
//			r = rpq(0);
//			JTJ += J * J.transpose() * s[c2];
//			JTr += J * r * s[c2];
//			r2 += r * r * s[c2];
//
//			J.setZero();
//			J(2) = -q(0);
//			J(0) = q(2);
//			J(4) = -1;
//			r = rpq(1);
//			JTJ += J * J.transpose() * s[c2];
//			JTr += J * r * s[c2];
//			r2 += r * r * s[c2];
//
//			J.setZero();
//			J(0) = -q(1);
//			J(1) = q(0);
//			J(5) = -1;
//			r = rpq(2);
//			JTJ += J * J.transpose() * s[c2];
//			JTr += J * r * s[c2];
//			r2 += r * r * s[c2];
//			r2 += (par * (1.0 - sqrt(s[c2])) * (1.0 - sqrt(s[c2])));
//		}
//		bool success;
//		Eigen::VectorXd result;
//		std::tie(success, result) = SolveLinearSystemPSD(-JTJ, JTr, false, false, false, false);
//		Eigen::Matrix4d delta = TransformVector6dToMatrix4d(result);
//		trans = delta * trans;
//		//point_cloud_copy_j.Transform(delta);
//		for (int row = 0; row < point_cloud_copy_j.rows(); row++) {
//			Eigen::Vector4d new_point =
//				delta *
//				Eigen::Vector4d(point_cloud_copy_j(row, 0), point_cloud_copy_j(row, 1), point_cloud_copy_j(row, 2), 1.0);
//			point_cloud_copy_j.row(row) = new_point.head<3>() / new_point(3);
//		}
//		// graduated non-convexity.
//		if (decrease_mu_) {
//			if (itr % 4 == 0 && par >maximum_correspondence_distance_) {
//				par /= division_factor_;
//			}
//		}
//	}
//	return trans;
//}
//
//std::tuple<bool, Eigen::VectorXd> SolveLinearSystemPSD(
//	const Eigen::MatrixXd &A,
//	const Eigen::VectorXd &b,
//	bool prefer_sparse /* = false */,
//	bool check_symmetric /* = false */,
//	bool check_det /* = false */,
//	bool check_psd /* = false */) {
//	// PSD implies symmetric
//	check_symmetric = check_symmetric || check_psd;
//	if (check_symmetric && !A.isApprox(A.transpose())) {
//		//LogWarning("check_symmetric failed, empty vector will be returned");
//		std::cout << "check_symmetric failed, empty vector will be returned" << std::endl;
//		return std::make_tuple(false, Eigen::VectorXd::Zero(b.rows()));
//	}
//
//	if (check_det) {
//		double det = A.determinant();
//		if (fabs(det) < 1e-6 || std::isnan(det) || std::isinf(det)) {
//			//LogWarning("check_det failed, empty vector will be returned");
//			std::cout << "check_det failed, empty vector will be returned" << std::endl;
//			return std::make_tuple(false, Eigen::VectorXd::Zero(b.rows()));
//		}
//	}
//
//	// Check PSD: https://stackoverflow.com/a/54569657/1255535
//	if (check_psd) {
//		Eigen::LLT<Eigen::MatrixXd> A_llt(A);
//		if (A_llt.info() == Eigen::NumericalIssue) {
//			//LogWarning("check_psd failed, empty vector will be returned");
//			std::cout << "check_psd failed, empty vector will be returned" << std::endl;
//			return std::make_tuple(false, Eigen::VectorXd::Zero(b.rows()));
//		}
//	}
//
//	Eigen::VectorXd x(b.size());
//
//	if (prefer_sparse) {
//		Eigen::SparseMatrix<double> A_sparse = A.sparseView();
//		// TODO: avoid deprecated API SimplicialCholesky
//		Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> A_chol;
//		A_chol.compute(A_sparse);
//		if (A_chol.info() == Eigen::Success) {
//			x = A_chol.solve(b);
//			if (A_chol.info() == Eigen::Success) {
//				// Both decompose and solve are successful
//				return std::make_tuple(true, std::move(x));
//			}
//			else {
//				//LogWarning("Cholesky solve failed, switched to dense solver");
//				std::cout << "Cholesky solve failed, switched to dense solver" << std::endl;
//
//			}
//		}
//		else {
//			//LogWarning("Cholesky decompose failed, switched to dense solver");
//			std::cout << "Cholesky decompose failed, switched to dense solver" << std::endl;
//		}
//	}
//
//	x = A.ldlt().solve(b);
//	return std::make_tuple(true, std::move(x));
//}
/*Eigen::Matrix4d TransformVector6dToMatrix4d(const Eigen::VectorXd &input) {
	Eigen::Matrix4d output;
	output.setIdentity();
	output.block<3, 3>(0, 0) =
		(Eigen::AngleAxisd(input(2), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(input(1), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(input(0), Eigen::Vector3d::UnitX()))
		.matrix();
	output.block<3, 1>(0, 3) = input.block<3, 1>(3, 0);
	return output;
}*/

void  Ransac_Plane( MatrixXf & Model, Vector3f direction,double threshold, double gridstepleng ,int itermum, std::vector<SPlane> & Plane, VectorXi  &planeindex)
{

	std::default_random_engine random(0);
	std::uniform_int_distribution<long> rand(0, Model.rows()-1);
	
	VectorXi tempindex = VectorXi::Zero(Model.rows());

	MatrixXf  modeltemp = Model.block(0, 0, Model.rows(), 3);
	MatrixXf  modeltempn = Model.block(0, 3, Model.rows(), 3);

	for (int nn = 0; nn < Plane.size(); nn++)
	{
		int tieration = itermum;
		Plane[nn].planeratio = 0;
		//
		std::vector<Matrix3f> tieration_point;
		std::vector<Vector4d> tieration_coff;
		std::vector<double> tieration_planeratio;
		tieration_point.resize(tieration);
		tieration_coff.resize(tieration);
		tieration_planeratio.resize(tieration);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic,8) 
#endif
		for (int i = 0; i < tieration; i++)
		{
			Matrix3f point;
			Vector4d coff_temp;
			point.row(0) = modeltemp.row(rand(random));
			point.row(1) = modeltemp.row(rand(random));
			point.row(2) = modeltemp.row(rand(random));
			if (!CreatePlane(point, direction, coff_temp))
			{
				tieration_planeratio[i] = -1;
				continue;
			}
			Vector3d  abc;
			abc = coff_temp.head(3);
			
			double g = abc.dot(direction.cast<double>());
			if (g<0.7)
			{
				tieration_planeratio[i] = -1;
				continue;
			}
			VectorXd diff(Model.rows());
			diff = modeltemp.cast<double>()*abc;
			diff = diff.array() + coff_temp(3);
			diff = diff.array().abs();
			int k = 0;
			for (int j = 0; j < Model.rows(); j++)
			{
				Vector3d  n1 = modeltempn.row(j).cast<double>().transpose();
				double an = n1.dot(abc);
				if (diff(j) < threshold && planeindex[j] == 0 && fabs(an) > 0.8)
				{
					k++;
				}

			}
			tieration_planeratio[i] = k * 1.0 / Model.rows();
			tieration_point[i] = point;
			tieration_coff[i] = coff_temp;
		}
		//查找最大占比
		int index = 0;
		double maxratio = -1;
		for (int i = 0; i < tieration; i++)
		{
			if (tieration_planeratio[i] > maxratio)
			{
				 maxratio= tieration_planeratio[i];
				index = i;
			}
		}
		//找索引
		{
			
			Vector3d  abc;
			abc = tieration_coff[index].head(3);
			VectorXd diff(Model.rows());
			diff = modeltemp.cast<double>()*abc;
			diff = diff.array() + tieration_coff[index](3);
			diff = diff.array().abs();

			tempindex.setZero();
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic,8) 
#endif
			for (int i = 0; i < Model.rows(); i++)
			{
				if (diff(i) < threshold && planeindex[i] == 0)
				{
					tempindex[i] = 1;
				}

			}
			Plane[nn].point = tieration_point[index];
			Plane[nn].coff = tieration_coff[index];
			Plane[nn].planeratio = tieration_planeratio[index];
		}
		
		
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic,8) 
#endif
		for (int i = 0; i < Model.rows(); i++)
		{
			if (tempindex[i] != 0)
			{
				planeindex[i] = nn+1;
			}
		}
		/*
		//计算面积
		if (Plane[nn].coff(2) < 0)
			Plane[nn].coff = Plane[nn].coff*(-1);
		Vector3d  v1(0, 0, 1);
		Vector3d  v2(Plane[nn].coff(0), Plane[nn].coff(1), Plane[nn].coff(2));
		Matrix3d rotation;
		Rodriguesrotationformula(v1, v2, rotation);
		MatrixXd model1(modeltemp.rows(),3);
		model1 = modeltemp.cast<double>()*rotation.transpose();
		double xmin, xmax, ymin, ymax;
		xmin = model1(0, 0);
		xmax = model1(0, 0);
		ymin = model1(0, 1);
		ymax = model1(0, 1);

		for (int i = 0; i < modeltemp.rows(); i++)
		{
			if (tempindex[i] != 0)
			{
				if (model1(i, 0) > xmax) xmax = model1(i, 0);
				if (model1(i, 0) < xmin) xmin = model1(i, 0);
				if (model1(i, 1) > ymax) ymax = model1(i, 1);
				if (model1(i, 1) < ymin) ymin = model1(i, 1);
			}
		}
		int in = (xmax - xmin) / gridstepleng+1;
		int jn = (ymax - ymin) / gridstepleng + 1;
		MatrixXi area = MatrixXi::Zero(in, jn);

		for (int i = 0; i < modeltemp.rows(); i++)
		{
			if (tempindex[i] != 0)
			{
				in = (model1(i, 0) - xmin) / gridstepleng;
				jn = (model1(i, 1) - ymin) / gridstepleng;
				area(in, jn) = 1;
			}
		
		}
		Plane[nn].rotation = rotation;
		Plane[nn].area = area.sum()*gridstepleng*gridstepleng;*/

	}
	
}
void  Rodriguesrotationformula(const Vector3d  v1, const Vector3d v2, Matrix3d& rotation)
{
	double cosvalue,sinvalue;
	cosvalue= v1.dot(v2)/v1.norm() / v1.norm();
	sinvalue = sqrt(1- cosvalue*cosvalue);
	Vector3d  v3 = v2.cross(v1);
	if (v3.norm() != 0)
		v3 = v3 / v3.norm();
	else
		v3<<0, 0, 1;
	Array33d rot;
	rot<<1,0,0,
		  0,1,0,
		  0,0,1;
	rot = rot*cosvalue;
	rotation= rot;
	rot = v3*v3.transpose();
	rot= rot*(1-cosvalue);
	rotation = rotation.array() + rot;
	rot << 0, -v3(2), v3(1),
		v3(2), 0, -v3(0),
		-v3(1), v3(0), 0;
	rot = sinvalue*rot;
	rotation = rotation.array() + rot;

}

void  Rodriguesrotationformula(const Vector3d  v1, const Vector3d v2, const Vector3d axle, Matrix3d& rotation)
{
	//计算角
	Vector3d n1 = axle.cross(v1);
	Vector3d n2 = axle.cross(v2);
	n1 = n1 / n1.norm();
	n2 = n2 / n2.norm();
	double angle = acos(n1.dot(n2));
	Vector3d n2ton1=n2.cross(n1);
	n2ton1 = n2ton1 / n2ton1.norm();
	double  a = n2ton1.dot(axle);
	if (a < 0)
		angle = 0-angle;

	double cosvalue, sinvalue;
	cosvalue = cos(angle);
	sinvalue = sin(angle);
	Vector3d  v3 = axle;
	
	Array33d rot;
	rot << 1, 0, 0,
		0, 1, 0,
		0, 0, 1;
	rot = rot*cosvalue;
	rotation = rot;
	rot = v3*v3.transpose();
	rot = rot*(1 - cosvalue);
	rotation = rotation.array() + rot;
	rot << 0, -v3(2), v3(1),
		v3(2), 0, -v3(0),
		-v3(1), v3(0), 0;
	rot = sinvalue*rot;
	rotation = rotation.array() + rot;

}

// 计算高斯累积正态分布值——快速
double GaussianNormallyDistributed_Fast(double x)
{
	// constants
	double a1 = 0.254829592;
	double a2 = -0.284496736;
	double a3 = 1.421413741;
	double a4 = -1.453152027;
	double a5 = 1.061405429;
	double p = 0.3275911;

	// Save the sign of x
	int sign = 1;
	if (x < 0)
		sign = -1;
	x = fabs(x) / sqrt(2.0);

	// A&S formula 7.1.26
	double t = 1.0 / (1.0 + p * x);
	double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x * x);

	return 0.5*(1.0 + sign * y);
}

#include <cmath>

double GaussianNormallyDistributed_Slow(double x)
{
	static const double RT2PI = sqrt(4.0*acos(0.0));

	static const double SPLIT = 7.07106781186547;

	static const double N0 = 220.206867912376;
	static const double N1 = 221.213596169931;
	static const double N2 = 112.079291497871;
	static const double N3 = 33.912866078383;
	static const double N4 = 6.37396220353165;
	static const double N5 = 0.700383064443688;
	static const double N6 = 3.52624965998911e-02;
	static const double M0 = 440.413735824752;
	static const double M1 = 793.826512519948;
	static const double M2 = 637.333633378831;
	static const double M3 = 296.564248779674;
	static const double M4 = 86.7807322029461;
	static const double M5 = 16.064177579207;
	static const double M6 = 1.75566716318264;
	static const double M7 = 8.83883476483184e-02;

	const double z = fabs(x);
	double c = 0.0;

	if (z <= 37.0)
	{
		const double e = exp(-z * z / 2.0);
		if (z < SPLIT)
		{
			const double n = (((((N6*z + N5)*z + N4)*z + N3)*z + N2)*z + N1)*z + N0;
			const double d = ((((((M7*z + M6)*z + M5)*z + M4)*z + M3)*z + M2)*z + M1)*z + M0;
			c = e * n / d;
		}
		else
		{
			const double f = z + 1.0 / (z + 2.0 / (z + 3.0 / (z + 4.0 / (z + 13.0 / 20.0))));
			c = e / (RT2PI*f);
		}
	}
	return x <= 0.0 ? c : 1 - c;
}
//最小二乘法拟合平面
bool  CreatePlane(const Matrix3f & Model, Vector3f direction, Vector4d & coff)
{
	Vector3d n1, n2,n;
	n1 = (Model.row(1) - Model.row(0)).transpose().cast<double>();
	n2 = (Model.row(2) - Model.row(0)).transpose().cast<double>();

	if (n1.norm()<0.1 || n2.norm()<0.1)
		return false;
	
	double a = acos(n1.dot(n2) / n1.norm() / n2.norm())*180/3.1415926;
	if(a<20 || a>160)
		 return false;
	n = n1.cross(n2);
	double d= n.norm();
	if (d < 0.01)
	{
		return false;
	}
	else
	{
		n = n / d;
	}
	
	
	double d1 = n.dot(direction.cast<double>());
	if (d1 < 0)
		n = -1 * n;

	d = 0-(n.dot(Model.row(1).cast<double>()));
	coff << n, d;
	return true;
}
//最小二乘法拟合平面
void  leastsquarePlane(const MatrixXf & Model,Vector4d & coff,int rownum)
{
	double xx, yy, xy, xz, yz, x,y,z;
	xx = 0;
	yy = 0;
	xy = 0;
	xz = 0;
	xy = 0;
	yz = 0;
	x = 0;
	y = 0;
	z = 0;

	int n = Model.rows();
	for (int i = 0; i < rownum; i++)
	{
		xx += Model(i, 0)*Model(i, 0);
		xy += Model(i, 0)*Model(i, 1);
		xz += Model(i, 0)*Model(i, 2);
		yy += Model(i, 1)*Model(i, 1);
		yz += Model(i, 1)*Model(i, 2);
		x += Model(i, 0);
		y += Model(i, 1);
		z += Model(i, 2);
	}

	Matrix3d A;
	A(0, 0) = xx;
	A(0, 1) = xy;
	A(0, 2) = x;
	A(1, 0) = xy;
	A(1, 1) = yy;
	A(1, 2) = y;
	A(2, 0) = x;
	A(2, 1) = y;
	A(2, 2) = n;

	Vector3d B(xz,yz,z);
	Vector3d XX;
	XX =A.ldlt().solve(B);
	coff(0) = XX(0);
	coff(1) = XX(1);
	coff(2) = -1;
	coff(3) = XX(2);
}
void kmeans(MatrixXf feature, int k, VectorXi& classid, MatrixXf& center,double error)
{
	//std::default_random_engine random(time(NULL));
	//std::uniform_int_distribution<long> rand(0, feature.rows() - 1);
	
	
	classid.resize(feature.rows());
	center.resize(k, feature.cols());

	VectorXi num(k);
	num.setZero();

	/*for (int i = 0; i < k; i++)
	{
		center.row(i) = feature.row(rand(random));
	}*/
	center.row(0).setZero();
	center.row(1).setOnes();
	int nn = feature.rows();
	int iteration = 10000;
	while (iteration--)
	{
		for (int i = 0; i < nn; i++)
		{
			double maxd = 100000000;
			double index = -1;
			
			for (int j = 0; j < k; j++)
			{   
				VectorXf v = feature.row(i) - center.row(j);
				/*v(0) = v(0) * 1;
				v(1) = v(1) * 1;
				v(2) = v(2) * 0;
				v(3) = v(3) * 0;
				v(4) = v(4) * 0;*/
				double d = v.norm(); // fabs(v(0))+ fabs(v(1))+ fabs(v(2))+ fabs(v(3))+ fabs(v(4));
				if (d <maxd)
				{
					 maxd=d;
					index = j;
				}
			}
			classid[i] = index;
		}

		//计算中心
		MatrixXf centern(k, feature.cols());
		centern.setZero();
		num.setZero();
		for (int i = 0; i < nn; i++)
		{
			double index = classid[i];
			centern.row(index) += feature.row(i);
			num[index]++;
		}
		for (int i = 0; i < k; i++)
		{
			centern.row(i) /= num[i];
		}

		VectorXi e(k);
		e.setZero();
		double dmax = 0;
		for (int i = 0; i < k; i++)
		{
			double d=(centern.row(i)- center.row(i)).norm();
			e(i) = d;
			if (dmax < d)
			{
				dmax = d;
			}
		}
		//std::cout<< iteration << std::endl << center << std::endl << centern << std::endl;

		center = centern;
		if (dmax < error)
		{
			break;
		}
	}

	
}
void  normal(KDTree* tree, MatrixXf& Model, int nn,MatrixXf& normal,double o)
{
	std::vector<std::vector<int>>model_indices;
	std::vector<std::vector<float>>model_dists;
	o = o * o;
	SearchKDTree(tree, Model, model_indices, model_dists, nn);

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < Model.rows(); i++)//
	{
		Vector3f center(0, 0, 0);
		/*center = Model.row(i).head(3).transpose();*/
		for (int k = 0; k < nn; k++)
		{
			center += Model.row(model_indices[i][k]).head(3).transpose();
		}
		center /= nn;
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 0; k < nn; k++)
		{
			Vector3d a = (Model.row(model_indices[i][k]).head(3).transpose() - center).cast<double>();
			double d = a(0) * a(0) + a(1) * a(1) + a(2) * a(2);
			double w = exp(-d / o);
			cov += a * a.transpose() * w;
		}


		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));
		double a= D(0, 0);
		int index = 0;
		for (int j = 1; j < 3; j++)
		{
			if (Eigenvalue(j) < a)
			{
				a = Eigenvalue(j);
				index = j;
			}
			
		}
	

		normal.row(i) = V.col(index).transpose().cast<float>();
		if (normal(i,2) > 0)
			normal.row(i) *= -1;
	}
}
double calvoerlap(MatrixXf Model1, MatrixXf Model2, float distance)
{
	KDTree* kdt1 = BuildKDTree(Model1);
	std::vector<std::vector<int>> indices;
	std::vector<std::vector<float>> dists;
	SearchKDTree(kdt1, Model2, indices, dists, 1);
	delete kdt1;

	int row = Model2.rows();
	int count = 0;
	distance = distance * distance;
	for (int i = 0; i < row; i++)//
	{
		//Vector3f dn= Model1.row(indices[i][0]).head(3).transpose()
		//	  - Model2.row(i).head(3).transpose();
		//std::cout << dn.norm() << "  " << dists[i][0] << "  " << sqrt(dists[i][0])<< "  " << distance << std::endl;
		if (dists[i][0] < distance)
		{
			count++;
		}
	}

	float total = row;
	if (Model1.rows() > total)
		total = Model1.rows();
	return count / total;
}

MatrixXf normal(MatrixXf pc, Vector3f centerpoint, int nei, double o)
{
	MatrixXf  normal;


	int row = pc.rows();
	normal.resize(row, 3);

	KDTree* kdt1 = BuildKDTree(pc);
	std::vector<std::vector<int>> indices;
	std::vector<std::vector<float>> dists;
	SearchKDTree(kdt1, pc, indices, dists, nei);
	delete kdt1;
#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < row; i++)//
	{
		Vector3f center(0, 0, 0);
		/*center = Model.row(i).head(3).transpose();*/
		for (int k = 0; k < nei; k++)
		{
			center += pc.row(indices[i][k]).transpose();
		}
		center /= nei;
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 0; k < nei; k++)
		{
			Vector3d a = (pc.row(indices[i][k]).transpose() - center).cast<double>();
			double d = a(0) * a(0) + a(1) * a(1) + a(2) * a(2);
			double w = exp(-d / o);
			cov += a * a.transpose() * w;
		}


		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));
		double a = D(0, 0);
		int index = 0;
		for (int j = 1; j < 3; j++)
		{
			if (Eigenvalue(j) < a)
			{
				a = Eigenvalue(j);
				index = j;
			}

		}


		normal.row(i) = V.col(index).transpose().cast<float>();
		//std::cout << normal.row(i).norm() << std::endl;
		Vector3f np = centerpoint - pc.row(i).transpose();
		if (np.dot(normal.row(i).transpose()) < 0)
		{
			normal(i, 0) = -1 * normal(i, 0);
			normal(i, 1) = -1 * normal(i, 1);
			normal(i, 2) = -1 * normal(i, 2);
		}
	}
	MatrixXf p(pc.rows(), 6);
	p << pc, normal;
	return p;
}


MatrixXf normal(std::vector<RowVector3f> data, Vector3f centerpoint, int nei, double o)
{
	MatrixXf pc, normal;
	

	int row = data.size();
	pc.resize(row, 3);
	normal.resize(row, 3);

	for (int i = 0; i < row; i++)
	{
		pc.row(i) = data[i];
	}

	KDTree* kdt1 = BuildKDTree(pc);
	std::vector<std::vector<int>> indices;
	std::vector<std::vector<float>> dists;
	SearchKDTree(kdt1, pc, indices, dists, nei);

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < row; i++)//
	{
		Vector3f center(0, 0, 0);
		/*center = Model.row(i).head(3).transpose();*/
		for (int k = 0; k < nei; k++)
		{
			center += pc.row(indices[i][k]).transpose();
		}
		center /= nei;
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 0; k < nei; k++)
		{
			Vector3d a = (pc.row(indices[i][k]).transpose() - center).cast<double>();
			double d = a(0) * a(0) + a(1) * a(1) + a(2) * a(2);
			double w = exp(-d / o);
			cov += a * a.transpose() * w;
		}


		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));
		double a = D(0, 0);
		int index = 0;
		for (int j = 1; j < 3; j++)
		{
			if (Eigenvalue(j) < a)
			{
				a = Eigenvalue(j);
				index = j;
			}

		}


		normal.row(i) = V.col(index).transpose().cast<float>();
		//std::cout << normal.row(i).norm() << std::endl;
		Vector3f np = centerpoint - pc.row(i).transpose();
		if (np.dot(normal.row(i).transpose()) < 0)
		{
			normal(i, 0) = -1 * normal(i, 0);
			normal(i, 1) = -1 * normal(i, 1);
			normal(i, 2) = -1 * normal(i, 2);
		}
	}
	MatrixXf p(pc.rows(), 6);
	p << pc, normal;
	return p;
}

void  normal(MatrixXf& Model, Vector3f centerpoint)
{
	

#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < Model.rows(); i++)//
	{
		double d = Model.row(i).tail(3).norm();
		if (d < 0.00001)
		{
			Model(i, 3) = 0;
			Model(i, 4) = 0;
			Model(i, 5) = 1;
			continue;
		}
		Vector3f np= centerpoint-Model.row(i).head(3).transpose() ;
		if (np.dot(Model.row(i).tail(3).transpose())<0)
		{
			Model(i, 3) = -1* Model(i, 3);
			Model(i, 4) = -1 * Model(i, 4);
			Model(i, 5) = -1 * Model(i, 5);
		}
	}
}

void  cuvrefeatrue_8v(KDTree * tree, MatrixXf & Model, int nn, float radius, MatrixXf& model_curve_feature)
{
	std::vector<std::vector<int>>model_indices;
	std::vector<std::vector<float>>model_dists;
	
	radiusSearchhKDTree(tree, Model.block(0, 0, Model.rows(), 3), model_indices, model_dists, radius* radius);
	model_curve_feature.resize(Model.rows(),8);
	double th = 0;
	double o = radius / 3.0;
	o = o*o;
	VectorXi vec_index(Model.rows());
	vec_index.setOnes();
	MatrixXd minmax(2, 5);
	minmax.row(0) << 100000.0, 100000.0, 100000.0, 100000.0, 100000.0;
	minmax.row(1) << -100000.0, -100000.0, -100000.0, -100000.0, -100000.0;
#ifdef _OPENMP
  // #pragma omp parallel for 
#endif
	//计算协方差矩
	for (int i = 0; i <Model.rows(); i++)//
	{
		nn = model_indices[i].size();
		if (nn < 5)
		{
			model_curve_feature.row(i).setZero();
			vec_index[i] = 0;
			continue;
		}
		//计算切平面，用用户提供的法向
		Vector3d p = Model.row(i).head(3).transpose().cast<double>();
		Vector3d n = Model.row(i).tail(3).transpose().cast<double>();
		
		double PlaneD =0-p.dot(n);

		Vector4d tangentplane;
		tangentplane << n, PlaneD;
		//计算再切平面的投影点
		MatrixXd subpoint(nn-1,3);
		for (int k = 1; k < nn; k++)
		{
			Vector3d a = (Model.row(model_indices[i][k]).head(3).cast<double>()).transpose();
			double t = a.dot(n) + PlaneD;
			subpoint.row(k - 1) = (a - n * t).transpose();
		}
		//计算角度，k=1的点为轴方向 两个点必须不同
		Vector3d nx = (subpoint.row(0).head(3) 
			- Model.row(model_indices[i][0]).head(3).cast<double>()).transpose();
		double nxd = nx.norm();
		if (nxd < 0.00001)
		{
			std::cout<<"点云中有重复的点"<<std::endl;
			model_curve_feature.row(i).setZero();
			vec_index[i] = 0;
			continue;
		}
		nx = nx / nxd;
		Vector3d ny = n.cross(nx);

			std::vector<double> angle;
		//计算极值
		for (int k = 1; k < nn; k++)
		{
			Vector3d nxk = (subpoint.row(k).head(3) - Model.row(model_indices[i][0]).head(3).cast<double>()).transpose();
			double nxdk = nxk.norm();
			if (nxdk > 0.000001)
			{
				nxk = nxk / nxdk;
				double x = nx.dot(nxk);
				if (x > 1)
					x = 1;
				else if (x < -1)
					x = -1;
				double y = ny.dot(nxk);
				if (y > 1)
					y = 1;
				else if (y < -1)
					y = -1;
				//std::cout <<x<<" "<<y << "  " << nx.norm() << "  " << ny.norm() <<"  "<< nxk.norm()<< std::endl;
				double a = 0;
				if (x == 1 )
				{
					a = 0;
				}
				else if(x==0 && y==1)
				{
					a = M_PI / 2;
				}
				else if (x == -1)
				{
					a = M_PI;
				}
				else if (x > 0 && y > 0)
				{
					a = acos(x);
				}
				else if (x <0 && y > 0)
				{
					a = acos(x);
				}
				else if (x < 0 && y < 0)
				{
					a = M_PI*2-acos(x);
				}
				else if (x > 0 && y < 0)
				{
					a = M_PI * 2 - acos(x);
				}
				
				angle.push_back(a);
			}
		}
		//对点排序
		std::sort(angle.begin(), angle.end());
		//求最大夹角
		double max = angle[0];
		for (int k = 1; k < angle.size(); k++)
		{
			double a = angle[k] - angle[k - 1];
			if (a > max)
				max = a;
		}
		if (M_PI * 2 - angle[angle.size() - 1] > max)
		{
			max = M_PI * 2 - angle[angle.size() - 1];
		}
		double r1 = 2.0 * M_PI / (nn - 1.0);
		r1 = (max-r1) / (M_PI-r1);
		//std::cout << r1 << " " << nn << " " << max << std::endl;
		if (r1 > 1) r1 = 1;
		//计算球面
		//计算中心点
		Vector3f ucenter= Model.row(i).head(3).transpose();
		double ud = 1;
		for (int k = 1; k < nn; k++)
		{
			Vector3f v = Model.row(model_indices[i][k]).head(3).transpose()- Model.row(i).head(3).transpose();
			double d = v(0) * v(0) + v(1) * v(1) + v(2) * v(2);
			double w = exp(-d / o);
			ucenter += Model.row(model_indices[i][k]).head(3).transpose()* w;
			ud += w;
		}
		ucenter /= ud;
		double t = ucenter.cast<double>().dot(n) + PlaneD;
		ucenter = (ucenter - n.cast<float>() * t).transpose();
		double d = (ucenter - Model.row(i).head(3).transpose()).norm();
		double r2 = d * 3.0 * M_PI / (4.0 * radius);
		//std::cout <<r2<< std::endl;
		if (r2 > 1) r2 = 1;
		//std::cout << ucenter.transpose() << " " << 
		//	Model.row(i).head(3) << " " <<  std::endl;
		//计算中心点
		Vector3f center(0,0,0);
		
		for (int k = 0; k < nn; k++)
		{
			center += Model.row(model_indices[i][k]).head(3).transpose();
		}
		center /= nn;
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 0; k < nn; k++)
		{
			Vector3d a = (Model.row(model_indices[i][k]).head(3).transpose() - center).cast<double>();
			double d = a(0)*a(0) + a(1)*a(1) + a(2)*a(2);
			double w = exp(-d/ o);
			cov += a*a.transpose()*w;
		}
		

		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));

		for (int j = 0; j < 2; j++)
		{
			double a;
			Vector3d  ed;
			for (int k = j + 1; k < 3; k++)
			{
				if (Eigenvalue(j) < Eigenvalue(k))
				{
					a = Eigenvalue(j);
					ed = V.col(j);
					Eigenvalue(j) = Eigenvalue(k);
					V.col(j) = V.col(k);
					Eigenvalue(k) = a;
					V.col(k) = ed;
				}
			}
		}
		//std::cout << Eigenvalue.transpose() << std::endl;

		model_curve_feature(i,0)=r1;
		model_curve_feature(i, 1) = r2;
		model_curve_feature(i, 2) =  1 - (Eigenvalue(1) - Eigenvalue(2)) / Eigenvalue(0);
		model_curve_feature(i, 3) =  Eigenvalue(2) / Eigenvalue(0);
		model_curve_feature(i, 4) =  Eigenvalue(2) / (Eigenvalue(0) + Eigenvalue(1) + Eigenvalue(2));
		model_curve_feature(i, 5) = V(0, 0);
		model_curve_feature(i, 6)= V(1, 0);
		model_curve_feature(i, 7)= V(2, 0);

		for (int k = 0; k < 5; k++)
		{
			if (minmax(0, k) > model_curve_feature(i, k))
			{
				minmax(0, k) = model_curve_feature(i, k);
			}
			if (minmax(1, k) < model_curve_feature(i, k))
			{
				minmax(1, k) = model_curve_feature(i, k);
			}
		}
	}
	int sum = vec_index.sum();
	//std::cout << vec_index.sum() << std::endl;
	if (sum != Model.rows())
	{
		MatrixXf M = Model;
		MatrixXf feature = model_curve_feature;
		Model.resize(sum, 6);
		model_curve_feature.resize(sum, 8);
		int nn = 0;
		for (int i = 0; i < M.rows(); i++)
		{
			if (vec_index(i))
			{
				Model.row(nn) = M.row(i);
				model_curve_feature.row(nn) = feature.row(i);
				nn++;
			}
		}
	}
	for (int k = 0; k < 5; k++)
	{
		double d = (minmax(1, k) - minmax(0, k)) * 0.05;
		minmax(0, k) = minmax(0, k)+d;
		minmax(1, k) = minmax(1, k)-d;
	}
	//std::cout << minmax << std::endl;
	//system("pause");
	for (int i = 0; i < model_curve_feature.rows(); i++)
	{
		for (int k = 0; k < 5; k++)
		{
			model_curve_feature(i, k) = (model_curve_feature(i, k)-minmax(0, k)) / minmax(1, k);
		
			if (model_curve_feature(i, k) < 0)
				model_curve_feature(i, k) = 0;
			else if (model_curve_feature(i, k) > 1)
				model_curve_feature(i, k) = 1;
		}
	    //std::cout << model_curve_feature.row(i) <<" "<< model_curve_feature.row(i).tail(3).norm()<< std::endl;
	}
}


void  cuvrefeatrue(KDTree * tree, MatrixXf & Model, int nn, float radius, std::vector<std::vector<float>>& model_curve_feature)
{
	std::vector<std::vector<int>>model_indices;
	std::vector<std::vector<float>>model_dists;

	SearchKDTree(tree, Model.block(0, 0, Model.rows(), 3), model_indices, model_dists, nn);
	model_curve_feature.resize(Model.rows());
	double th = 0;
#ifdef _OPENMP
   #pragma omp parallel for 
#endif
	//计算协方差矩
	for (int i = 0; i <Model.rows(); i++)//
	{
		//std::cout << i << std::endl;
		//计算变形度
		MatrixXf modelknn(nn, 3);//记录knn坐标
		for (int k = 0; k < nn; k++)
		{
			modelknn.row(k) = Model.row(model_indices[i][k]).head(3);
		}
		//modelknn.row(nn) = Model.row(i).head(3);
		
		//cout << mean(0) <<" "<< mean(1) << " " << mean(2) << " " << endl;
		modelknn.col(0) = modelknn.col(0).array() - modelknn(0,0);
		modelknn.col(1) = modelknn.col(1).array() - modelknn(0, 1);
		modelknn.col(2) = modelknn.col(2).array() - modelknn(0, 2);
		double  len=0;
		//计算方差矩阵
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 1; k < nn; k++)
		{
			if (radius > model_dists[i][k])
			{
				cov += (modelknn.row(k).cast<double>().transpose() *modelknn.row(k).cast<double>()*1.0 / model_dists[i][k]);
				len += 1.0 / model_dists[i][k];
			}
			//std::cout << model_dists[i][k] << " ";
		}
		cov = cov / len;
		//cov = cov ;
		//std::cout << cov << std::endl;
		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));

		for (int j = 0; j < 2; j++)
		{
			double a;
			for (int k = j+1; k < 3; k++)
			{
				if (Eigenvalue(j) > Eigenvalue(k))
				{
					a = Eigenvalue(j);
					Eigenvalue(j) = Eigenvalue(k);
					Eigenvalue(k) = a;
				}
			}
		}
		//if (th < Eigenvalue(0)) th = Eigenvalue(0);
		model_curve_feature[i].push_back(Eigenvalue(0));
		//model_curve_feature[i].push_back(Eigenvalue(1));
		//model_curve_feature[i].push_back(Eigenvalue(2));
		
		//std::cout << Eigenvalue(0)<<"  " << Eigenvalue(1) << "  " << Eigenvalue(2) << "  " << std::endl;


	}
	//滤波
	int k =3;
	th = 0.0001;
	while (0)
	{
		for (int i = 0; i < Model.rows(); i++)//
		{
			double a;
			a = 0;
			for (int k = 1; k < nn; k++)
			{
				a += model_curve_feature[model_indices[i][k]][0];
			}
			a = a / (nn - 1);
			if (a < th) a = 0;
			model_curve_feature[i][0] = a;
		//	Model(i, 3) = a * 200;
		//	Model(i, 4) = 0;
		//	Model(i, 5) = 0;
		}
	}
	/*	k = 3;
	while (k--)
	{
		for (int i = 0; i < Model.rows(); i++)//
		{
			int  a;
			a = 0;
			for (int k = 1; k < nn; k++)
			{
				if (model_curve_feature[model_indices[i][k]][0]>th)
					a++;
			}
			if (a < nn / 2)
			{
				model_curve_feature[i][0] = 0;
				Model(i, 3) = 0;
				Model(i, 4) = 0;
				Model(i, 5) = 0;
			}
		}
	}*/
	

}

void  cuvrefeatrue2(KDTree * tree, MatrixXf & Model, int nn, float radius, std::vector<std::vector<float>>& model_curve_feature)
{
	std::vector<std::vector<int>>model_indices;
	std::vector<std::vector<float>>model_dists;
	SearchKDTree(tree, Model.block(0, 0, Model.rows(), 3), model_indices, model_dists, nn);
	model_curve_feature.resize(Model.rows());
	//std::cout << Model.rows() << std::endl;
#ifdef _OPENMP
#pragma omp parallel for 
#endif
	//计算协方差矩阵
	for (int i = 0; i <Model.rows(); i++)//
	{
		//std::cout << i << std::endl;
		//计算变形度
		MatrixXf modelknn(nn + 1, 3);//记录knn坐标
		for (int k = 0; k < nn; k++)
		{
			modelknn.row(k) = Model.row(model_indices[i][k]).head(3);
		}
		modelknn.row(nn) = Model.row(i).head(3);
		Array3f mean;//计算中值
		mean(0) = modelknn.col(0).mean();
		mean(1) = modelknn.col(1).mean();
		mean(2) = modelknn.col(2).mean();
		//cout << mean(0) <<" "<< mean(1) << " " << mean(2) << " " << endl;
		modelknn.col(0) = modelknn.col(0).array() - mean(0);
		modelknn.col(1) = modelknn.col(1).array() - mean(1);
		modelknn.col(2) = modelknn.col(2).array() - mean(2);
		//计算方差矩阵
		Matrix3d cov = Matrix3d::Zero();
		for (int k = 0; k < nn; k++)
		{
			cov += modelknn.row(k).cast<double>().transpose() *modelknn.row(k).cast<double>();
		}
		cov = cov / nn;

		EigenSolver<Matrix3d> es(cov);

		Matrix3d D = es.pseudoEigenvalueMatrix();
		Matrix3d V = es.pseudoEigenvectors();
		Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));
		//if(Eigenvalue.minCoeff()>0.1)
		//cout << Eigenvalue.minCoeff() << endl;
		//Eigenvalue.minCoeff();
		//cout << V << endl;
		//double lam0 = Eigenvalue.minCoeff();

		for (int j = 0; j < 2; j++)
		{
			double a;
			for (int k = j + 1; k < 3; k++)
			{
				if (Eigenvalue(j) > Eigenvalue(k))
				{
					a = Eigenvalue(j);
					Eigenvalue(j) = Eigenvalue(k);
					Eigenvalue(k) = a;
				}
			}
		}
		model_curve_feature[i].push_back(Eigenvalue(0));
		//std::cout << Eigenvalue(0)<<"  " << Eigenvalue(1) << "  " << Eigenvalue(2) << "  " << std::endl;


		//Model(i, 3) = Eigenvalue(0) / (D(0, 0) + D(1, 1) + D(2, 2)) * 100;
///
		//Model(i, 4) = 0;
		//Model(i, 5) = 0;
	}

}
std::vector<int> random_permun(int n)
{
	std::random_device rd;
	std::mt19937 g(rd());
	std::vector<int> temp;
	for (int i = 0; i < n; i++)
		temp.push_back(i);
	std::shuffle(temp.begin(), temp.end(), g);
	return temp;
}
MatrixXf computeImpl(const int rows, const int cols, const VectorXf  &depth, const MatrixXf  &cam_K, MatrixXf & normals)
{
	const int r = 5; // used to be 7
	const int sample_step = r;
	const int square_size = ((2 * r / sample_step) + 1);
	long offsets[square_size * square_size];
	long offsets_x[square_size * square_size];
	long offsets_y[square_size * square_size];
	long offsets_x_x[square_size * square_size];
	long offsets_x_y[square_size * square_size];
	long offsets_y_y[square_size * square_size];
	
	for (int j = -r, index = 0; j <= r; j += sample_step)
		for (int i = -r; i <= r; i += sample_step, ++index)
		{
			offsets_x[index] = i;
			offsets_y[index] = j;
			offsets_x_x[index] = i*i;
			offsets_x_y[index] = i*j;
			offsets_y_y[index] = j*j;
			offsets[index] = j * cols + i;
		}

	// Define K_inv by hand, just for higher accuracy
	Matrix3f  K_inv, K;
	K = cam_K;
	K_inv.setIdentity();
	//Mat33T K_inv = Matx<T, 3, 3>::eye(), K;
	//K_.copyTo(K);
	K_inv(0, 0) = 1.0f / K(0, 0);
	K_inv(0, 1) = -K(0, 1) / (K(0, 0) * K(1, 1));
	K_inv(0, 2) = (K(0, 1) * K(1, 2) - K(0, 2) * K(1, 1)) / (K(0, 0) * K(1, 1));
	K_inv(1, 1) = 1 / K(1, 1);
	K_inv(1, 2) = -K(1, 2) / K(1, 1);
	std::cout << K_inv << std::endl;
	std::cout << K_inv*K << std::endl;
	Vector3d X1_minus_X, X2_minus_X;
	double  difference_threshold = 500;
	//Vec3T X1_minus_X, X2_minus_X;

	
	normals.resize(cols*rows,3);
	normals.setZero();
	//normals.setTo(std::numeric_limits<DepthDepth>::quiet_NaN());

	/*for (int y = r; y < rows - r - 1; ++y)
	{

		//const DepthDepth * p_line = reinterpret_cast<const DepthDepth*>(depth.ptr(y, r));
		//Vec3T *normal = normals.ptr<Vec3T>(y, r);

		
		for (int x = r; x < cols - r - 1; ++x)
		{
			int index = y*cols + x;
			double d = depth(index);// p_line[0];
			if(d<0.0000001) 
				continue;
			// accum
			long A[4];
			A[0] = A[1] = A[2] = A[3] = 0;
			double  b[2];
			b[0] = b[1] = 0;
			for (unsigned int i = 0; i < square_size * square_size; ++i) {
				// We need to cast to ContainerDepth in case we have unsigned DepthDepth
				//ContainerDepth delta = ContainerDepth(p_line[offsets[i]]) - ContainerDepth(d);
				double delta = depth(index+offsets[i]) - d;
				
				if (std::abs(delta) > difference_threshold)
					continue;
                //std::cout << delta<< std::endl;
				A[0] += offsets_x_x[i];
				A[1] += offsets_x_y[i];
				A[3] += offsets_y_y[i];
				b[0] += offsets_x[i] * delta;
				b[1] += offsets_y[i] * delta;
			}

			// solve for the optimal gradient D of equation (8)
			long det = A[0] * A[3] - A[1] * A[1];
			// We should divide the following two by det, but instead, we multiply
			// X1_minus_X and X2_minus_X by det (which does not matter as we normalize the normals)
			// Therefore, no division is done: this is only for speedup

			double dx = (A[3] * b[0] - A[1] * b[1]);
			double dy = (-A[1] * b[0] + A[0] * b[1]);

			// Compute the dot product
			//Vec3T X = K_inv * Vec3T(x, y, 1) * depth(y, x);
			//Vec3T X1 = K_inv * Vec3T(x + 1, y, 1) * (depth(y, x) + dx);
			//Vec3T X2 = K_inv * Vec3T(x, y + 1, 1) * (depth(y, x) + dy);
			//Vec3T nor = (X1 - X).cross(X2 - X);
			multiply_by_K_inv(K_inv, d * det + (x + 1) * dx, y * dx, dx, X1_minus_X);
			multiply_by_K_inv(K_inv, x * dy, d * det + (y + 1) * dy, dy, X2_minus_X);

			//std::cout << X1_minus_X.transpose()<<" "<< X2_minus_X.transpose() << std::endl;

			Vector3d nor = X1_minus_X.cross(X2_minus_X);
	
			double  norm = 1.0 / nor.norm();
			if (nor[2] > 0)
				nor = -nor* norm;
			else
				nor = nor*norm;

			normals(index,0) = nor[0];
			normals(index, 1) = nor[1];
			normals(index, 2) = nor[2];

		}
	}*/

	return normals;
}
/** Function that multiplies K_inv by a vector. It is just meant to speed up the product as we know
* that K_inv is upper triangular and K_inv(2,2)=1
* @param K_inv
* @param a
* @param b
* @param c
* @param res
*/

void  multiply_by_K_inv(Matrix3f & K_inv, double a, double b, double c, Vector3d & res)
{
	res[0] = (K_inv(0, 0) * a + K_inv(0, 1) * b + K_inv(0, 2) * c);
	res[1] = (K_inv(1, 1) * b + K_inv(1, 2) * c);
	res[2] = c;
}

MatrixXf loadRGBD_enhance(const char* fileName, MatrixXf  cam_K, double maxlength)
{
	//读入时前面6列为坐标和法向量

	std::ifstream ifs1(fileName);
	std::string str1;
	str1 = fileName;
	MatrixXf points3d;
	if (!ifs1.is_open())
	{
		std::cout << "Error opening input file: " + str1 + "\n";
		return  points3d;
	}

	VectorXf point;
	int rows, cols;
	rows = 0;
	cols = 0;
	ifs1 >> rows >> cols;
	point.resize(rows * cols);
	if (!ifs1.eof())
	{
		for (int ii = 0; ii < rows; ii++)
			for (int jj = 0; jj < cols; jj++)
			{
				ifs1 >> point(ii * cols + jj);
			}
	}
	std::cout << "hang=" << rows << " " << cols << std::endl;
	MatrixXf normals;
	computeImpl(rows, cols, point, cam_K, normals);
	int num = rows * cols;

	//对近似垂直点加密
	MatrixXf points3d_temp_x;
	MatrixXf points3d_temp_y;
	MatrixXf points3d_temp_z;
    std::vector<Vector3f> pointadd;
	points3d_temp_x.resize(rows, cols);
	points3d_temp_y.resize(rows, cols);
	points3d_temp_z.resize(rows, cols);
	
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			int index = i * cols + j;
			points3d_temp_x(i,j) = point(index) * (index % cols - cam_K(0, 2)) / cam_K(0, 0) / cam_K(2, 2);
			points3d_temp_y(i, j) = point(index) * (int(index / cols) - cam_K(1, 2)) / cam_K(1, 1) / cam_K(2, 2);
			points3d_temp_z(i, j) = point(index) / cam_K(2, 2);
		}
	}



	std::vector<int> indexlist;



	double w = cols / 2;
	double h = rows / 2;
	double r = h * h * 0.8;
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if ((w - j) * (w - j) + (h - i) * (h - i) < r)
			{
				int index = i * cols + j;

				double d = normals(index, 0) * normals(index, 0)
					+ normals(index, 1) * normals(index, 1) + normals(index, 2) * normals(index, 2);
				//std::cout << d << " ";
				//d = 1;
				if (d > 0.1 && point(index) / cam_K(2, 2) < maxlength)
				{
					indexlist.push_back(index);
				}
			}
		}
	}
	num = indexlist.size();
	points3d.resize(num, 6);
	std::cout << num << " " << cols << std::endl;
	for (int i = 0; i < num; i++)
	{
		int index = indexlist[i];
		double d = normals(index, 0) * normals(index, 0)
			+ normals(index, 1) * normals(index, 1) + normals(index, 2) * normals(index, 2);
		d = sqrt(d);
		points3d(i, 0) = point(index) * (index % cols - cam_K(0, 2)) / cam_K(0, 0) / cam_K(2, 2);
		points3d(i, 1) = point(index) * (int(index / cols) - cam_K(1, 2)) / cam_K(1, 1) / cam_K(2, 2);
		points3d(i, 2) = point(index) / cam_K(2, 2);
		points3d(i, 3) = normals(index, 0) / d;
		points3d(i, 4) = normals(index, 1) / d;
		points3d(i, 5) = normals(index, 2) / d;
	}
	return points3d;
	//cam_K={fx,0,cx;
	//       0  fy,cy;0  0 1}
	//x=z*(x-cx)/fx  y=z*(y-cy)/fy z=y 


	//cloud *= 5.0f;

}

void WriteBMP(char* img, int w, int h, const char* filename)
{
	int l = (w * 3 + 3) / 4 * 4 ;
	int bmi[] = { l * h + 54,0,54,40,w,h,1 | 3 * 8 << 16,0,l * h,0,0,100,0 };
	FILE* fp = fopen(filename, "wb");
	fprintf(fp, "BM");
	fwrite(&bmi, 52, 1, fp);
	fwrite(img, 1, l * h, fp);
	fclose(fp);
}
Vector3f world2screen(MatrixXf cam_K, Vector3f v,int w,int h) {

	float dx = v(0) * cam_K(0, 0) /v(2) + cam_K(0, 2);
	float dy = v(1) * cam_K(1, 1) /v(2) + cam_K(1, 2);
	Vector3f screen(-1, -1, 0);
	if (dx < 0 || dy < 0 || dx >= w || dy >= h)
		return screen;
	screen(0) = int(dx);
	screen(1) = int(dy);
	screen(2) = v(2);
	return screen;
}
Vector3f barycentric(Vector3f A, Vector3f B, Vector3f C, Vector3f P) {
	Vector3f s[3];
	for (int i = 2; i--; ) {
		s[i][0] = C[i] - A[i];
		s[i][1] = B[i] - A[i];
		s[i][2] = A[i] - P[i];
	}
	Vector3f u = s[0].cross(s[1]);
	
	if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
		return Vector3f(1.0 - (u(0) + u(1)) / u(2), u(1) / u(2), u(0) / u(2));
	return Vector3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}
void triangle(Matrix3f p, MatrixXf& zbuffer, MatrixXf cam_K)
{
	//https://zhuanlan.zhihu.com/p/56502233  中心法求解
	Vector2i bboxmin(zbuffer.rows(), zbuffer.cols());
	Vector2i bboxmax(0, 0);
	Matrix3f ps;
	for (int i = 0; i < 3; i++)
	{
		if (p(2, i) < 0.00001) return;
		ps.col(i) = world2screen(cam_K, p.col(i), zbuffer.cols(), zbuffer.rows());
		if (ps(2, i) <0.00001) return;
	}
	

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 3; j++) {
			bboxmin(i)= std::min(bboxmin(i), int(ps(i,j)));
			bboxmax(i) = std::max(bboxmax(i), int(ps(i, j)));
		}
	}
	Vector3f P;
	for (P(0) = bboxmin(0); P(0) <= bboxmax(0); P(0)++) {
		for (P(1) = bboxmin(1); P(1) <= bboxmax(1); P(1)++) {
			Vector3f bc_screen = barycentric(ps.col(0), ps.col(1), ps.col(2), P);
			if (bc_screen(0) < 0 || bc_screen(1) < 0 || bc_screen(2) < 0) continue;
			P(2) = 0;
			for (int i = 0; i < 3; i++) 
				P(2) += ps(2,i) * bc_screen[i];//计算深度 P=A+u*(B-A)+v*(C-A);
			if (zbuffer(int(P(1)), int(P(0)))> P(2)&& zbuffer(int(P(1)), int(P(0))) != 0) {
				zbuffer(int(P(1)), int(P(0))) = P(2);
			}
			else if (zbuffer(int(P(1)), int(P(0))) == 0)
			{
				zbuffer(int(P(1)), int(P(0))) = P(2);
			}
		}
	}
}

MatrixXf loadRGBD(const char* fileName,  MatrixXf cam_K, 
	MatrixXf& depthimage, MatrixXf& depthimagen1, MatrixXf& depthimagen2, 
	MatrixXf& depthimagen3, double maxlength)
{
	//读入时前面6列为坐标和法向量

	std::ifstream ifs1(fileName);
	std::string str1;
	str1 = fileName;
	MatrixXf points3d;
	if (!ifs1.is_open())
	{
		std::cout << "Error opening input file: " + str1 + "\n";
		return  points3d;
	}

	VectorXf point;
	MatrixXf point_temp;
	int rows, cols;
	rows = 0;
	cols = 0;
	ifs1 >> rows >> cols;
	point.resize(rows* cols);
	point_temp.resize(rows, cols);
	depthimage.resize(rows, cols);
	depthimagen1.resize(rows, cols);
	depthimagen2.resize(rows, cols);
	depthimagen3.resize(rows, cols);
	depthimagen1.setZero();
	depthimagen2.setZero();
	depthimagen2.setZero();
	
	if(!ifs1.eof())
	{
		for (int ii = 0; ii < rows; ii++)
			for (int jj = 0; jj < cols; jj++)
			{
				int i = ii * cols + jj;
				ifs1 >> point(i);
				point(i) = point(i) / cam_K(2, 2);
				depthimage(ii, jj) = point(i);
				point_temp(ii, jj) = point(i);
			}
	}
	
	MatrixXf normals;
	computeImpl(rows, cols, point, cam_K, normals);
	for (int i = 2; i < rows - 2; i++)
	{
		for (int j = 2; j < cols - 2; j++)
		{

			
			int index = i * cols + j;
			double d = normals(index, 0) * normals(index, 0)
				+ normals(index, 1) * normals(index, 1)
				+ normals(index, 2) * normals(index, 2);
			if (d > 0.000001 && point(index) < maxlength)
			{
				d = sqrt(d);
				depthimagen1(i, j) = normals(index, 0) / d;
				depthimagen2(i, j) = normals(index, 1) / d;
				depthimagen3(i, j) = normals(index, 2) / d;
			}

		}
	}
	//滤波 
	/*
	for (int ii = 1; ii < rows - 1; ii++)
		for (int jj = 1; jj < cols - 1; jj++)
		{
			if(point_temp(ii, jj)!=0)
			for (int i = ii - 1; i < ii + 2; i++)
			{
				for (int j = jj - 1; j < jj + 2; j++)
				{
					if (point_temp(i, j) == 0)
					{
						point_temp(i, j) = point_temp(ii, jj);
					}
				}
			}
		}
	for (int c = 0; c < 1; c++)
	{
		for (int ii = 2; ii < rows - 2; ii++)
			for (int jj = 2; jj < cols - 2; jj++)
			{
				int n = 0;
				double d = 0;
				for (int i = ii - 2; i < ii + 3; i++)
				{
					for (int j = jj -2; j < jj + 3; j++)
					{
						if (point_temp(i, j) != 0)
						{
							d += point_temp(i, j);
							n++;
						}

					}
				}
				int m = ii * cols + jj;
				point(m) = d / n;
				//std::cout << point(m)<<" ";
			}
		for (int ii = 0; ii < rows; ii++)
			for (int jj = 0; jj < cols; jj++)
			{
				int m = ii * cols + jj;
				point_temp(ii, jj)=point(m);
			}

	}*/
	
	

	std::cout<<"hang=" << rows << " " << cols << std::endl;
	
	computeImpl(rows, cols, point, cam_K, normals);
	std::vector<int> indexlist;
	int num = rows*cols;
	
	MatrixXf normals_temp(normals.rows(),3);
	normals_temp.setZero();
	for (int i = 2; i < rows-2; i++)
	{
		for (int j = 2; j < cols-2; j++)
		{
			int ind= i * cols + j;
			double n = 0;
			for (int ii = i-0; ii < i +1; ii++)
			{
				for (int jj = j-0; jj < j + 1; jj++)
				{
					n++;
					int index = ii * cols + jj;
					double d = normals(index, 0) * normals(index, 0)
						+ normals(index, 1) * normals(index, 1)
						+ normals(index, 2) * normals(index, 2);
					if (d > 0.00001)
					{
						normals_temp.row(ind) += normals.row(index);
						n += 1;
					}

					
				}
			}
			normals_temp.row(ind) = normals_temp.row(ind) / n;
			//normals_temp.row(ind) = normals.row(ind);
			double d = normals_temp(ind, 0) * normals_temp(ind, 0)
				+ normals_temp(ind, 1) * normals_temp(ind, 1)
				+ normals_temp(ind, 2) * normals_temp(ind, 2);
			int di = (i - rows / 2) * (i - rows / 2) + (j - cols / 2) * (j - cols / 2);
			if (d > 0.000001 && point(ind) < maxlength && di<(rows*rows+cols*cols)/1.0)
			{
				d = sqrt(d);
				normals_temp(ind, 0) /= d;
				normals_temp(ind, 1) /= d;
				normals_temp(ind, 2) /= d;
				indexlist.push_back(ind);
			}
		}
	}
	num = indexlist.size();
	points3d.resize(num, 6);
	std::cout << num << " " << cols << std::endl;
	for (int i = 0; i < num; i++)
	{
		int index = indexlist[i];
		

		points3d(i, 0) = point(index)*(index%cols- cam_K(0, 2)) / cam_K(0, 0) ;
		points3d(i, 1) = point(index)*(int(index/cols)- cam_K(1, 2)) / cam_K(1, 1) ;
		points3d(i, 2) = point(index) ;
		points3d(i, 3) = normals_temp(index, 0) ;
		points3d(i, 4) = normals_temp(index, 1) ;
		points3d(i, 5) = normals_temp(index, 2) ;
	}
	return points3d;
}
MatrixXf loadPLYSimple_bin(const char* fileName, int withNormals)
{
	//读入时前面6列为坐标和法向量

	int numVertices = 0;
	int numCols = 3;
	int has_normals = 0;


	std::ifstream ifs(fileName, std::ios::in | std::ios::binary);
	std::string str1;
	str1 = fileName;
	if (!ifs.is_open())
		std::cout << "Error opening input file: " + str1 + "\n";

	/* char   buffer[80];
	getcwd(buffer, 80);
	printf("The   current   directory   is:   %s ", buffer);*/

	std::string str;
	while (str.substr(0, 10) != "end_header")
	{
		std::vector<std::string> tokens = split(str, ' ');
		if (tokens.size() == 3)
		{
			if (tokens[0] == "element" && tokens[1] == "vertex")
			{
				numVertices = atoi(tokens[2].c_str());
			}
			else if (tokens[0] == "property")
			{
				if (tokens[2] == "nx" || tokens[2] == "normal_x")
				{
					has_normals = -1;
					numCols += 3;
				}
				else if (tokens[2] == "r" || tokens[2] == "red")
				{
					//has_color = true;
					numCols += 3;
				}
				else if (tokens[2] == "a" || tokens[2] == "alpha")
				{
					//has_alpha = true;
					numCols += 1;
				}
			}
		}
		else if (tokens.size() > 1 && tokens[0] == "format" && tokens[1] != "ascii")
			std::cout << "Cannot read file, only ascii ply format is currently supported...\n";
		std::getline(ifs, str);
	}
	withNormals &= has_normals;

	MatrixXf cloud(numVertices, withNormals ? 6 : 3);
	//Eigen::Matrix<float,numVertices, withNormals ? 6 : 3,RowMajor> eigMatRow;

	std::cout << "模型点数：" << numVertices << "  法向量状态：" << withNormals << std::endl;
	
	int n = 0;
	if (withNormals==1)
	{
		for (int i = 0; i < numVertices; )
		{
			float fea[6];
			if (ifs.read((char*)&fea[0], 6 * sizeof(float)))
			{

				for (int col = 0; col < 6; ++col)
				{
					cloud(i, col) = fea[col];
				}

				if (withNormals)//模型归一化
				{
					// normalize to unit norm
					double norm = sqrt(cloud(i, 3) * cloud(i, 3) + cloud(i, 4) * cloud(i, 4) + cloud(i, 5) * cloud(i, 5));
					if (norm > 0.00001)
					{
						cloud(i, 3) /= static_cast<float>(norm);
						cloud(i, 4) /= static_cast<float>(norm);
						cloud(i, 5) /= static_cast<float>(norm);
					}
				}
				i++;
			}


		}
	}
	else 
	{
		for (int i = 0; i < numVertices; )
		{
			double fea[3];
			if (ifs.read((char*)&fea[0], 3 * sizeof(double)))
			{

				for (int col = 0; col < 3; ++col)
				{
					cloud(i, col) = fea[col];
				}
				i++;
			}
		}
	}
	

	//cloud *= 5.0f;
	return cloud;
}

MatrixXf loadPLYSimple_face(const char* fileName, MatrixXi& face)
{
	//读入时前面6列为坐标和法向量

	int numVertices = 0;
	int facenum = 0;
	int numCols = 3;
	int has_normals = 0;

	int norindex[3];
	int pointindex[3];

	std::ifstream ifs(fileName);
	std::string str1;
	str1 = fileName;
	if (!ifs.is_open())
		std::cout << "Error opening input file: " + str1 + "\n";

	


	std::string str;
	int n = 0;
	while (str.substr(0, 10) != "end_header")
	{
		std::vector<std::string> tokens = split(str, ' ');
		if (tokens.size() == 3)
		{
			if (tokens[0] == "element" && tokens[1] == "vertex")
			{
				numVertices = atoi(tokens[2].c_str());
			}
			else if (tokens[0] == "element" && tokens[1] == "face")
			{
				facenum = atoi(tokens[2].c_str());
			}
			else if (tokens[0] == "property")
			{
				if (tokens[2] == "x" )
				{
					pointindex[0] = n;
					n++;
				}
				else if (tokens[2] == "y" )
				{
					pointindex[1] = n;
					n++;
				}
				else if (tokens[2] == "z")
				{
					pointindex[2] = n;
					n++;
				}
				else if (tokens[2] == "nx" || tokens[2] == "normal_x")
				{
					norindex[0] = n;
					has_normals=1;
					n++;
				}
				else if (tokens[2] == "ny" || tokens[2] == "normal_y")
				{
					norindex[1] = n;
					n++;
				}
				else if (tokens[2] == "nz" || tokens[2] == "normal_z")
				{
					norindex[2] = n;
					n++;
				}
				else 
				{
					n++;
				}
				
			}
		}
		else if (tokens.size() > 1 && tokens[0] == "format" && tokens[1] != "ascii")
			std::cout << "Cannot read file, only ascii ply format is currently supported...\n";
		std::getline(ifs, str);
	}


	MatrixXf cloud(numVertices, 6);
	face.resize(facenum,3);
	//Eigen::Matrix<float,numVertices, withNormals ? 6 : 3,RowMajor> eigMatRow;

	std::cout << "模型点数：" << numVertices << "  面数：" << facenum << std::endl;
	//std::cout << pointindex[0] << "  " << pointindex[1] << "  " << pointindex[2] << "  " << std::endl;
	//std::cout << norindex[0] << "  " << norindex[1] << "  " << norindex[2] << "  " << std::endl;

	for (int i = 0; i < numVertices; i++)
	{
		std::getline(ifs, str);
		//std::cout << str << std::endl;
		std::vector<std::string> tokens = split(str, ' ');
		cloud(i, 0) = atof(tokens[pointindex[0]].c_str());
		cloud(i, 1) = atof(tokens[pointindex[1]].c_str());
		cloud(i, 2) = atof(tokens[pointindex[2]].c_str());

		cloud(i, 3) = atof(tokens[norindex[0]].c_str());
		cloud(i, 4) = atof(tokens[norindex[1]].c_str());
		cloud(i, 5) = atof(tokens[norindex[2]].c_str());

		// normalize to unit norm
		double norm = sqrt(cloud(i, 3) * cloud(i, 3) + cloud(i, 4) * cloud(i, 4) + cloud(i, 5) * cloud(i, 5));
		if (norm > 0.00001)
		{
			cloud(i, 3) /= static_cast<float>(norm);
			cloud(i, 4) /= static_cast<float>(norm);
			cloud(i, 5) /= static_cast<float>(norm);
		}
		
	}

	for (int i = 0; i < facenum; i++)
	{
		std::getline(ifs, str);
		//std::cout << str << std::endl;
		std::vector<std::string> tokens = split(str, ' ');
		face(i, 0) = atoi(tokens[1].c_str());
		face(i, 1) = atoi(tokens[2].c_str());
		face(i, 2) = atoi(tokens[3].c_str());
		//std::cout << face(i, 0) << " " << face(i, 1) << " " << face(i, 2) << std::endl;
	}

	//cloud *= 5.0f;
	return cloud;
}
void addNoisePC(MatrixXf &pc, double scale)
{
	double mean = 0.0;//均值
	double stddev = scale;//标准差
	std::random_device rd;
	std::default_random_engine generator{ rd() };
	std::normal_distribution<double> dist(mean, stddev);

	int rows = pc.rows();
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			pc(i, j) += dist(generator);
		}
	}
}
MatrixXf loadPLYSimple(const char* fileName, int withNormals)
{
   //读入时前面6列为坐标和法向量
 
   int numVertices = 0;
   int numCols = 3;
   int has_normals = 0;
  

   std::ifstream ifs(fileName);
   std::string str1;
   str1 = fileName ;
   if (!ifs.is_open())
      std::cout<<"Error opening input file: " + str1 + "\n";
  
  /* char   buffer[80];
   getcwd(buffer, 80);
   printf("The   current   directory   is:   %s ", buffer);*/

  std::string str;
  while (str.substr(0, 10) != "end_header")
  {
    std::vector<std::string> tokens = split(str,' ');
    if (tokens.size() == 3)
    {
      if (tokens[0] == "element" && tokens[1] == "vertex")
      {
        numVertices = atoi(tokens[2].c_str());
      }
      else if (tokens[0] == "property")
      {
        if (tokens[2] == "nx" || tokens[2] == "normal_x")
        {
          has_normals = -1;
          numCols += 3;
        }
        else if (tokens[2] == "r" || tokens[2] == "red")
        {
          //has_color = true;
          numCols += 3;
        }
        else if (tokens[2] == "a" || tokens[2] == "alpha")
        {
          //has_alpha = true;
          numCols += 1;
        }
      }
    }
    else if (tokens.size() > 1 && tokens[0] == "format" && tokens[1] != "ascii")
		std::cout << "Cannot read file, only ascii ply format is currently supported...\n";
     std::getline(ifs, str);
  }
  withNormals &= has_normals;

  MatrixXf cloud(numVertices, withNormals ? 6 : 3);
  //Eigen::Matrix<float,numVertices, withNormals ? 6 : 3,RowMajor> eigMatRow;

  std::cout<<"模型点数：" << numVertices<<"  法向量状态：" << withNormals<<std::endl;
  for (int i = 0; i < numVertices; i++)
  {
   
    int col = 0;
    for (; col < (withNormals ? 6 : 3); ++col)
    {
      ifs >> cloud(i, col);
    }
    for (; col < numCols; ++col)
    {
      float tmp;
      ifs >> tmp;
    }
    if (withNormals)//模型归一化
    {
      // normalize to unit norm
      double norm = sqrt(cloud(i, 3)* cloud(i, 3) + cloud(i, 4)* cloud(i, 4) + cloud(i, 5)* cloud(i, 5));
      if (norm>0.00001)
      {
		  cloud(i, 3)/=static_cast<float>(norm);
		  cloud(i, 4)/=static_cast<float>(norm);
		  cloud(i, 5)/=static_cast<float>(norm);
      }
    }
  }

  //cloud *= 5.0f;
  return cloud;
}

MatrixXf loadXYZSimple(const char* fileName)
{
	//读入时前面6列为坐标和法向量

	int numVertices = 0;
	int numCols = 3;
	int has_normals = 0;


	std::ifstream ifs(fileName);
	std::string str1;
	str1 = fileName;
	if (!ifs.is_open())
		std::cout << "Error opening input file: " + str1 + "\n";
	std::vector<double> data;
	while (!ifs.eof())
	{

		double a = 0;
		ifs >> a;
		data.push_back(a);
	}
	int row = data.size() / 3;
	MatrixXf cloud(row, 3);
	for (int i = 0; i < row; i++)
	{
		cloud(i, 0) = data[i * 3];
		cloud(i, 1) = data[i * 3+1];
		cloud(i, 2) = data[i * 3+2];
	}
	return cloud;
}


void writePLY(MatrixXf PC, const char* FileName)
{
  std::ofstream outFile( FileName );

  std::string str;
  str = FileName;
  if (!outFile.is_open())
	  std::cout << "Error opening output file:" + str + "\n";

  

  ////
  // Header
  ////

  const int pointNum = ( int ) PC.rows();
  const int vertNum  = ( int ) PC.cols();

  outFile << "ply" << std::endl;
  outFile << "format ascii 1.0" << std::endl;
  outFile << "element vertex " << pointNum << std::endl;
  outFile << "property float x" << std::endl;
  outFile << "property float y" << std::endl;
  outFile << "property float z" << std::endl;
  if (vertNum==6)
  {
    outFile << "property float nx" << std::endl;
    outFile << "property float ny" << std::endl;
    outFile << "property float nz" << std::endl;
  }
  outFile << "end_header" << std::endl;

  ////
  // Points
  ////

  for ( int pi = 0; pi < pointNum; ++pi )
  {
    //const float* point = &PC(pi,0);

    outFile << PC(pi, 0) << " " << PC(pi, 1) << " " << PC(pi, 2);

    if (vertNum==6)
    {
      outFile<<" " << PC(pi, 3) << " "<< PC(pi, 4) <<" "<< PC(pi, 5);
    }

    outFile << std::endl;
  }

  return;
}
void writePLYRGBnor(MatrixXf PC, const char* FileName)
{
	std::ofstream outFile(FileName);

	std::string str;
	str = FileName;
	if (!outFile.is_open())
		std::cout << "Error opening output file:" + str + "\n";



	////
	// Header
	////

	const int pointNum = (int)PC.rows();
	const int vertNum = (int)PC.cols();

	outFile << "ply" << std::endl;
	outFile << "format ascii 1.0" << std::endl;
	outFile << "element vertex " << pointNum << std::endl;
	outFile << "property float x" << std::endl;
	outFile << "property float y" << std::endl;
	outFile << "property float z" << std::endl;
	outFile << "property float red" << std::endl;
	outFile << "property float green" << std::endl;
	outFile << "property float blue" << std::endl;
	outFile << "property float nx" << std::endl;
	outFile << "property float ny" << std::endl;
	outFile << "property float nz" << std::endl;
	outFile << "end_header" << std::endl;

	////
	// Points
	////

	for (int pi = 0; pi < pointNum; ++pi)
	{
		//const float* point = &PC(pi,0);

		outFile << PC(pi, 0) << " " << PC(pi, 1) << " " << PC(pi, 2);
		outFile << " " << PC(pi, 3) << " " << PC(pi, 4) << " " << PC(pi, 5);
		outFile << " " << PC(pi, 6) << " " << PC(pi, 7) << " " << PC(pi, 8);
		outFile << std::endl;
	}

}
void writePLYRGB(MatrixXf PC, const char* FileName)
{
	std::ofstream outFile(FileName);

	std::string str;
	str = FileName;
	if (!outFile.is_open())
		std::cout << "Error opening output file:" + str + "\n";



	////
	// Header
	////

	const int pointNum = (int)PC.rows();
	const int vertNum = (int)PC.cols();

	outFile << "ply" << std::endl;
	outFile << "format ascii 1.0" << std::endl;
	outFile << "element vertex " << pointNum << std::endl;
	outFile << "property float x" << std::endl;
	outFile << "property float y" << std::endl;
	outFile << "property float z" << std::endl;
	outFile << "property float red" << std::endl;
	outFile << "property float green" << std::endl;
	outFile << "property float blue" << std::endl;
	outFile << "end_header" << std::endl;

	////
	// Points
	////

	for (int pi = 0; pi < pointNum; ++pi)
	{
		//const float* point = &PC(pi,0);

		outFile << PC(pi, 0) << " " << PC(pi, 1) << " " << PC(pi, 2);
		outFile << " " << PC(pi, 3) << " " << PC(pi, 4) << " " << PC(pi, 5);
		outFile << std::endl;
	}

	return;
}

/*

void writePLYVisibleNormals(Mat PC, const char* FileName)
{
  std::ofstream outFile(FileName);

  if (!outFile.is_open())
    CV_Error(Error::StsError, String("Error opening output file: ") + String(FileName) + "\n");

  ////
  // Header
  ////

  const int pointNum = (int)PC.rows;
  const int vertNum = (int)PC.cols;
  const bool hasNormals = vertNum == 6;

  outFile << "ply" << std::endl;
  outFile << "format ascii 1.0" << std::endl;
  outFile << "element vertex " << (hasNormals? 2*pointNum:pointNum) << std::endl;
  outFile << "property float x" << std::endl;
  outFile << "property float y" << std::endl;
  outFile << "property float z" << std::endl;
  if (hasNormals)
  {
    outFile << "property uchar red" << std::endl;
    outFile << "property uchar green" << std::endl;
    outFile << "property uchar blue" << std::endl;
  }
  outFile << "end_header" << std::endl;

  ////
  // Points
  ////

  for (int pi = 0; pi < pointNum; ++pi)
  {
    const float* point = PC.ptr<float>(pi);

    outFile << point[0] << " " << point[1] << " " << point[2];

    if (hasNormals)
    {
      outFile << " 127 127 127" << std::endl;
      outFile << point[0] + point[3] << " " << point[1] + point[4] << " " << point[2] + point[5];
      outFile << " 255 0 0";
    }

    outFile << std::endl;
  }

  return;
}*/


MatrixXf samplePCUniform(MatrixXf PC, int sampleStep)
{
  int numRows = PC.rows()/sampleStep;
  MatrixXf sampledPC (numRows, PC.cols());

  int c=0;
  for (int i=0; i<PC.rows() && c<numRows; i+=sampleStep)
  {
	  sampledPC.row(c++)=PC.row(i);
  }

  return sampledPC;
}

MatrixXf samplePCUniformInd(MatrixXf PC, int sampleStep, std::vector<int> &indices)
{
  int numRows = std::round((double)PC.rows()/(double)sampleStep);//四舍五入函数
  indices.resize(numRows);
  MatrixXf sampledPC(numRows, PC.cols());

  int c=0;
  for (int i=0; i<PC.rows() && c<numRows; i+=sampleStep)
  {
    indices[c] = i;
	sampledPC.row(c++)=PC.row(i);
  }

  return sampledPC;
}

MatrixXf StatisticsDenoiseNormal(MatrixXf &pc_s, int NumNeighbors, double rate, double angle)
{
	//标准正态分布
	double sta_z = 1.0;
	if (rate < 0.99999)
	{
		MatrixXf m(31, 10);

		m << 0, 0.004, 0.008, 0.012, 0.016, 0.0199, 0.0239, 0.0279, 0.0319, 0.0359,
			0.0398, 0.0438, 0.0478, 0.0517, 0.0557, 0.0596, 0.0636, 0.0675, 0.0714, 0.0753,
			0.0793, 0.0832, 0.0871, 0.091, 0.0948, 0.0987, 0.1026, 0.1064, 0.1103, 0.1141,
			0.1179, 0.1217, 0.1255, 0.1293, 0.1331, 0.1368, 0.1406, 0.1443, 0.148, 0.1517,
			0.1554, 0.1591, 0.1628, 0.1664, 0.17, 0.1736, 0.1772, 0.1808, 0.1844, 0.1879,
			0.1915, 0.195, 0.1985, 0.2019, 0.2054, 0.2088, 0.2123, 0.2157, 0.219, 0.2224,
			0.2257, 0.2291, 0.2324, 0.2357, 0.2389, 0.2422, 0.2454, 0.2486, 0.2517, 0.2549,
			0.258, 0.2611, 0.2642, 0.2673, 0.2704, 0.2734, 0.2764, 0.2794, 0.2823, 0.2852,
			0.2881, 0.291, 0.2939, 0.2967, 0.2995, 0.3023, 0.3051, 0.3078, 0.3106, 0.3133,
			0.3159, 0.3186, 0.3212, 0.3238, 0.3264, 0.3289, 0.3315, 0.334, 0.3365, 0.3389,
			0.3413, 0.3438, 0.3461, 0.3485, 0.3508, 0.3531, 0.3554, 0.3577, 0.3599, 0.3621,
			0.3643, 0.3665, 0.3686, 0.3708, 0.3729, 0.3749, 0.377, 0.379, 0.381, 0.383,
			0.3849, 0.3869, 0.3888, 0.3907, 0.3925, 0.3944, 0.3962, 0.398, 0.3997, 0.4015,
			0.4032, 0.4049, 0.4066, 0.4082, 0.4099, 0.4115, 0.4131, 0.4147, 0.4162, 0.4177,
			0.4192, 0.4207, 0.4222, 0.4236, 0.4251, 0.4265, 0.4279, 0.4292, 0.4306, 0.4319,
			0.4332, 0.4345, 0.4357, 0.437, 0.4382, 0.4394, 0.4406, 0.4418, 0.4429, 0.4441,
			0.4452, 0.4463, 0.4474, 0.4484, 0.4495, 0.4505, 0.4515, 0.4525, 0.4535, 0.4545,
			0.4554, 0.4564, 0.4573, 0.4582, 0.4591, 0.4599, 0.4608, 0.4616, 0.4625, 0.4633,
			0.4641, 0.4649, 0.4656, 0.4664, 0.4671, 0.4678, 0.4686, 0.4693, 0.4699, 0.4706,
			0.4713, 0.4719, 0.4726, 0.4732, 0.4738, 0.4744, 0.475, 0.4756, 0.4761, 0.4767,
			0.4772, 0.4778, 0.4783, 0.4788, 0.4793, 0.4798, 0.4803, 0.4808, 0.4812, 0.4817,
			0.4821, 0.4826, 0.483, 0.4834, 0.4838, 0.4842, 0.4846, 0.485, 0.4854, 0.4857,
			0.4861, 0.4864, 0.4868, 0.4871, 0.4875, 0.4878, 0.4881, 0.4884, 0.4887, 0.489,
			0.4893, 0.4896, 0.4898, 0.4901, 0.4904, 0.4906, 0.4909, 0.4911, 0.4913, 0.4916,
			0.4918, 0.492, 0.4922, 0.4925, 0.4927, 0.4929, 0.4931, 0.4932, 0.4934, 0.4936,
			0.4938, 0.494, 0.4941, 0.4943, 0.4945, 0.4946, 0.4948, 0.4949, 0.4951, 0.4952,
			0.4953, 0.4955, 0.4956, 0.4957, 0.4959, 0.496, 0.4961, 0.4962, 0.4963, 0.4964,
			0.4965, 0.4966, 0.4967, 0.4968, 0.4969, 0.497, 0.4971, 0.4972, 0.4973, 0.4974,
			0.4974, 0.4975, 0.4976, 0.4977, 0.4977, 0.4978, 0.4979, 0.4979, 0.498, 0.4981,
			0.4981, 0.4982, 0.4982, 0.4983, 0.4984, 0.4984, 0.4985, 0.4985, 0.4986, 0.4986,
			0.4987, 0.4987, 0.4987, 0.4988, 0.4988, 0.4989, 0.4989, 0.4989, 0.499, 0.499;
		double rate1 = rate - 0.5;
		double ratemin = rate1;
		for (int i = 0; i < 31; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (ratemin > fabs(m(i, j) - rate1))
				{
					sta_z = i * 0.1 + j * 0.01;
					ratemin = fabs(m(i, j) - rate1);
				}

			}
		}
	}
	else
	{
		sta_z = 100000000.0;
	}
	//
	MatrixXf pc;
	{
		int n = pc_s.rows();
		pc.resize(n, 3);
		int k = 0;

		for (int i = 0; i < n; i++)
		{
			////if (pc_s(i, 0) < 800 && pc_s(i, 0) > -800
				//&& pc_s(i, 1) < 800 && pc_s(i, 1) > -800
				//&& pc_s(i, 2) < 800 && pc_s(i, 2) > -10)
			{
				pc.row(k) = pc_s.row(i);
				k++;
			}
		}
		pc.conservativeResize(k, 3);
		if (k < NumNeighbors)
		{
			pc.conservativeResize(k, 6);
			return pc;
		}
	}

	KDTree*  kdt1 = BuildKDTree(pc);
	std::vector<std::vector<int>> indices;
	std::vector<std::vector<float>> dists;
	SearchKDTree(kdt1, pc, indices, dists, NumNeighbors);

	int const n = pc.rows();
	MatrixXf p_nor(n, 3);
	VectorXf distance(n);
#ifdef _OPENMP
#pragma omp parallel for
#endif	
	for (int i = 0; i < n; i++)
	{
		MatrixXf M(NumNeighbors, 4);
		//M.setOnes();
		double d = 0.0;
		for (int j = 0; j < NumNeighbors; j++)
		{
			M.row(j) << pc.row(indices[i][j]), 1.0;
			d += sqrt(dists[i][j]);
		}
		distance(i) = d / NumNeighbors;

		//求法向
		JacobiSVD<MatrixXf> svd(M, ComputeThinU | ComputeThinV);
		MatrixXf V = svd.matrixV();
		//MatrixXf A = svd.singularValues();
		p_nor.row(i) = V.col(3).head(3).transpose();
	}

	//去噪
	double const mean = distance.mean();
	double const std_dev = sqrt((distance.array() - mean).square().sum() / (n - 1));
	/*double dt = 0;
	for (int i = 0; i < n; i++)
	{
		dt += (distance(i) - mean)*(distance(i) - mean);

	}*/
	std::cout << mean << " " << std_dev << " " << sta_z << std::endl;
	//置信区间设置为85
	double max_distance = mean + sta_z * std_dev / sqrt(n);
	double th = 0 - cos(angle / 180.0*3.1415926);
#ifdef _OPENMP
#pragma omp parallel for
#endif	
	for (int i = 0; i < n; i++)
	{
		if (distance(i) > max_distance)
		{
			distance(i) = 0;
			continue;
		}
		p_nor.row(i) = p_nor.row(i) / p_nor.row(i).norm();
		if (p_nor(i, 2) > 0)
			p_nor.row(i) *= -1;

		if (p_nor(i, 2) > th)
		{
			distance(i) = 0;
			continue;
		}
		distance(i) = 1;
	}

	MatrixXf pcn(n, 6);
	int k = 0;
	for (int i = 0; i < n; i++)
	{
		if (distance(i) > 0.5)
		{
			pcn.row(k) << pc.row(i), p_nor.row(i);
			k++;
		}
	}
	pcn.conservativeResize(k, 6);
	return pcn;
}

MatrixXf StatisticsDenoise(MatrixXf& pc_s, int NumNeighbors, double rate)
{
	//标准正态分布
	double sta_z = 1.0;
	if (rate < 0.99999)
	{
		MatrixXf m(31, 10);

		m << 0, 0.004, 0.008, 0.012, 0.016, 0.0199, 0.0239, 0.0279, 0.0319, 0.0359,
			0.0398, 0.0438, 0.0478, 0.0517, 0.0557, 0.0596, 0.0636, 0.0675, 0.0714, 0.0753,
			0.0793, 0.0832, 0.0871, 0.091, 0.0948, 0.0987, 0.1026, 0.1064, 0.1103, 0.1141,
			0.1179, 0.1217, 0.1255, 0.1293, 0.1331, 0.1368, 0.1406, 0.1443, 0.148, 0.1517,
			0.1554, 0.1591, 0.1628, 0.1664, 0.17, 0.1736, 0.1772, 0.1808, 0.1844, 0.1879,
			0.1915, 0.195, 0.1985, 0.2019, 0.2054, 0.2088, 0.2123, 0.2157, 0.219, 0.2224,
			0.2257, 0.2291, 0.2324, 0.2357, 0.2389, 0.2422, 0.2454, 0.2486, 0.2517, 0.2549,
			0.258, 0.2611, 0.2642, 0.2673, 0.2704, 0.2734, 0.2764, 0.2794, 0.2823, 0.2852,
			0.2881, 0.291, 0.2939, 0.2967, 0.2995, 0.3023, 0.3051, 0.3078, 0.3106, 0.3133,
			0.3159, 0.3186, 0.3212, 0.3238, 0.3264, 0.3289, 0.3315, 0.334, 0.3365, 0.3389,
			0.3413, 0.3438, 0.3461, 0.3485, 0.3508, 0.3531, 0.3554, 0.3577, 0.3599, 0.3621,
			0.3643, 0.3665, 0.3686, 0.3708, 0.3729, 0.3749, 0.377, 0.379, 0.381, 0.383,
			0.3849, 0.3869, 0.3888, 0.3907, 0.3925, 0.3944, 0.3962, 0.398, 0.3997, 0.4015,
			0.4032, 0.4049, 0.4066, 0.4082, 0.4099, 0.4115, 0.4131, 0.4147, 0.4162, 0.4177,
			0.4192, 0.4207, 0.4222, 0.4236, 0.4251, 0.4265, 0.4279, 0.4292, 0.4306, 0.4319,
			0.4332, 0.4345, 0.4357, 0.437, 0.4382, 0.4394, 0.4406, 0.4418, 0.4429, 0.4441,
			0.4452, 0.4463, 0.4474, 0.4484, 0.4495, 0.4505, 0.4515, 0.4525, 0.4535, 0.4545,
			0.4554, 0.4564, 0.4573, 0.4582, 0.4591, 0.4599, 0.4608, 0.4616, 0.4625, 0.4633,
			0.4641, 0.4649, 0.4656, 0.4664, 0.4671, 0.4678, 0.4686, 0.4693, 0.4699, 0.4706,
			0.4713, 0.4719, 0.4726, 0.4732, 0.4738, 0.4744, 0.475, 0.4756, 0.4761, 0.4767,
			0.4772, 0.4778, 0.4783, 0.4788, 0.4793, 0.4798, 0.4803, 0.4808, 0.4812, 0.4817,
			0.4821, 0.4826, 0.483, 0.4834, 0.4838, 0.4842, 0.4846, 0.485, 0.4854, 0.4857,
			0.4861, 0.4864, 0.4868, 0.4871, 0.4875, 0.4878, 0.4881, 0.4884, 0.4887, 0.489,
			0.4893, 0.4896, 0.4898, 0.4901, 0.4904, 0.4906, 0.4909, 0.4911, 0.4913, 0.4916,
			0.4918, 0.492, 0.4922, 0.4925, 0.4927, 0.4929, 0.4931, 0.4932, 0.4934, 0.4936,
			0.4938, 0.494, 0.4941, 0.4943, 0.4945, 0.4946, 0.4948, 0.4949, 0.4951, 0.4952,
			0.4953, 0.4955, 0.4956, 0.4957, 0.4959, 0.496, 0.4961, 0.4962, 0.4963, 0.4964,
			0.4965, 0.4966, 0.4967, 0.4968, 0.4969, 0.497, 0.4971, 0.4972, 0.4973, 0.4974,
			0.4974, 0.4975, 0.4976, 0.4977, 0.4977, 0.4978, 0.4979, 0.4979, 0.498, 0.4981,
			0.4981, 0.4982, 0.4982, 0.4983, 0.4984, 0.4984, 0.4985, 0.4985, 0.4986, 0.4986,
			0.4987, 0.4987, 0.4987, 0.4988, 0.4988, 0.4989, 0.4989, 0.4989, 0.499, 0.499;
		double rate1 = rate - 0.5;
		double ratemin = rate1;
		for (int i = 0; i < 31; i++)
		{
			for (int j = 0; j < 10; j++)
			{
				if (ratemin > fabs(m(i, j) - rate1))
				{
					sta_z = i * 0.1 + j * 0.01;
					ratemin = fabs(m(i, j) - rate1);
				}

			}
		}
	}
	else
	{
		sta_z = 10.0;
	}
	//
	MatrixXf pc= pc_s.block(0,0, pc_s.rows(),3);
	

	KDTree* kdt1 = BuildKDTree(pc);
	std::cout << NumNeighbors << std::endl;

	std::vector<std::vector<int>> indices;
	std::vector<std::vector<float>> dists;
	SearchKDTree(kdt1, pc, indices, dists, NumNeighbors);

	std::cout << NumNeighbors << std::endl;
	int const n = pc.rows();
	VectorXf distance(n);
#ifdef _OPENMP
  #pragma omp parallel for
#endif	
	for (int i = 0; i < n; i++)
	{
		double d = 0.0;
		for (int j = 1; j < dists[i].size(); j++)
		{
			d += sqrt(dists[i][j]);
		}
		distance(i) = d /(NumNeighbors-1);
	}
	
	//去噪
	double const mean = distance.mean();
	double const std_dev = sqrt((distance.array() - mean).square().sum() / (n - 1));
	
	
	//置信区间设置为85
	double max_distance = mean + sta_z * std_dev;

	std::cout << mean << " " << std_dev << " " << sta_z<<" "<< max_distance << std::endl;

#ifdef _OPENMP
  #pragma omp parallel for
#endif	
	for (int i = 0; i < n; i++)
	{
		if (distance(i) > max_distance)
		{
			distance(i) = 0;
		}
		else
		    distance(i) = 1;
	}

	MatrixXf pcn(n, pc_s.cols());
	int k = 0;
	for (int i = 0; i < n; i++)
	{
		if (distance(i) > 0.5)
		{
			pcn.row(k) = pc_s.row(i);
			k++;
		}
	}
	pcn.conservativeResize(k, pc_s.cols());
	delete kdt1;
	return pcn;
}

MatrixXf BinningDenoise(MatrixXf &pc_s, float sampleStep, double rate)
{

	MatrixXf pc;
	{
		
		int n = pc_s.rows();
		pc.resize(n, 3);
		int k = 0;
		
		for (int i = 0; i < n; i++)
		{
		////	if (pc_s(i,0)<800 && pc_s(i, 0) >-800
		//		&& pc_s(i, 1) < 800 && pc_s(i, 1) > -800
		//		&& pc_s(i, 2) < 800 && pc_s(i, 2) > -10)
			{
				pc.row(k) = pc_s.row(i);
				k++;
			}
		}
		pc.conservativeResize(k, 3);
	}
	RowVector3f pc_max, pc_min, stepsize;
	
	int size = int(1 / sampleStep);
	for (int i = 0; i < 3; i++)
	{
		pc_max(i) = pc.col(i).maxCoeff();
		pc_min(i) = pc.col(i).minCoeff();
		stepsize(i) =1.0/( (pc_max(i) - pc_min(i)) / size + 0.001);
		std::cout << pc_max(i) << "  " << pc_min(i) << std::endl;
		
	}
	//stepsize.setConstant(stepsize.mean());
	
	std::vector<MatrixXi> indexsum, indexsum2;
	indexsum.resize(size);
	indexsum2.resize(size);
	for (int i = 0; i < size; i++)
	{
		indexsum[i].resize(size, size);
	
		indexsum[i].setZero();
		indexsum2[i].resize(size, size);
		indexsum2[i].setZero();
	}
	int n = pc.rows();
	for (int i = 0; i < n; i++)
	{
		RowVector3f a=(pc.row(i) - pc_min).array()*stepsize.array();
		(indexsum[int(a(2))](int(a(0)), int(a(1)))) += 1;
	}

	int maxcount = 0;
	for (int i = 0; i < size; i++)
	{
		int max = indexsum[i].maxCoeff();
		if (max > maxcount)
		{
			maxcount = max;
		}
	}
	//std::cout << maxcount << "  " << maxcount << std::endl;
	maxcount *= rate;//3
	int size1 = size - 1;
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			for (int k = 0; k < size; k++)
			{
				if (indexsum[k](i, j) > maxcount)
				{
					int i1 = i-1;
					int i2 = 3;

					int j1 = j - 1;
					int j2 = 3;

					if (i == 0)
					{
						i1 = i;
						i2 = 2;
					}
					else if(i == size1)
					{
						i1 = i-1;
						i2 = 2;
					}

					if (j == 0)
					{
						j1 = j;
						j2 = 2;
					}
					else if (j == size1)
					{
						j1 = j-1;
						j2 = 2;
					}

					if(k==0)
					{
						
						indexsum2[k].block(i1, j1, i2, j2).setConstant(1);
						indexsum2[k + 1].block(i1, j1, i2, j2).setConstant(1);
					}
					else if(k == size1)
					{
						indexsum2[k - 1].block(i1, j1, i2, j2).setConstant(1);
						indexsum2[k].block(i1, j1, i2, j2).setConstant(1);
					}
					else
					{
						indexsum2[k - 1].block(i1, j1, i2, j2).setConstant(1);
						indexsum2[k].block(i1, j1, i2, j2).setConstant(1);
						indexsum2[k + 1].block(i1, j1, i2, j2).setConstant(1);
					}
					

					
				}
			}
		}
	}
	int m;

	VectorXi denoise(n);
	denoise.setZero();
	for (int i = 0; i < n; i++)
	{
		RowVector3f a = (pc.row(i) - pc_min).array()*stepsize.array();
		if(indexsum2[int(a(2))](int(a(0)), int(a(1)))==1)
		    denoise(i) = 1;
	}

	int count = denoise.sum();
	//std::cout << n << "  " << count << std::endl;
	MatrixXf pc1(count,3);
	int c = 0;
	for (int i = 0; i < n; i++)
	{
		if (denoise(i) == 1)
		{
			pc1.row(c) = pc.row(i);
			c++;
		}
	}
	//std::cout << c << "  " << count << std::endl;
	//std::cin >> m;
	return pc1;
}
MatrixXf BinningDenoiseNormal(MatrixXf &pc, float sampleStep, double rate, double angle)
{
	RowVector3f pc_max, pc_min, stepsize;
	int m;
	int size = int(1 / sampleStep);
	//std::cout << pc.cols()<<"  " << pc.rows() << std::endl;
	//std::cin >> m;
	for (int i = 0; i < 3; i++)
	{
		pc_max(i) = pc.col(i).maxCoeff();
		pc_min(i) = pc.col(i).minCoeff();
		stepsize(i) = 1.0 / ((pc_max(i) - pc_min(i)) / size + 0.001);
	}
	double step = (stepsize(0)+ stepsize(1)+ stepsize(2))/3.0;
	std::cout << pc_max << "  " << pc_min << "  " << std::endl;
	//std::cout<< pc.rows() << "  " << stepsize(0) << "  " << stepsize(1)<<"  " << stepsize(2) << std::endl;

	//std::cin >> m;
	//if (step < stepsize(1)) step = stepsize(1);
	//if (step < stepsize(2)) step = stepsize(2);

	Vector3i stepcount;
	stepcount = ((pc_max - pc_min)*step).cast<int>().array()+1;
	int totalcount = stepcount(0)*stepcount(1)*stepcount(2);
	std::vector< std::vector<int>> index;

	index.resize(totalcount);
	std::cout << totalcount << std::endl;
	int n = pc.rows();
	for (int i = 0; i < n; i++)
	{
		RowVector3f a = (pc.row(i) - pc_min)*step;
		int k = int(a(0))*stepcount(1)*stepcount(2) + int(a(1))*stepcount(2) + int(a(2));
		index[k].push_back(i);
	}

	std::vector<MatrixXi> indexsum;
	indexsum.resize(stepcount(0));
	for (int i = 0; i < stepcount(0); i++)
	{
		indexsum[i].resize(stepcount(1), stepcount(2));
		indexsum[i].setZero();
	}

	int maxcount = 0;
	for (int i = 0; i < totalcount; i++)
	{
		int max = index[i].size();
		if (max > maxcount)
		{
			maxcount = max;
		}
	}

	int x_count = stepcount(0) - 1;
	int y_count = stepcount(1) - 1;
	int z_count = stepcount(2) - 1;
	std::cout << n << "   " << maxcount << std::endl;
	 maxcount = maxcount*rate;//2.5
	 if (maxcount < 1) maxcount = 1;
	for (int i = 0; i <= x_count; i++)
	{
		for (int j = 0; j <= y_count; j++)
		{
			for (int k = 0; k <= z_count; k++)
			{

				int kk = i*stepcount(1)*stepcount(2) + j*stepcount(2) + k;
				if (index[kk].size() >= maxcount)
				{

					int k1 = k - 1;
					int k2 = 3;

					int j1 = j - 1;
					int j2 = 3;

					if (k == 0)
					{
						k1 = k;
						k2 = 2;
					}
					else if (k == z_count)
					{
						k1 = k - 1;
						k2 = 2;
					}

					if (j == 0)
					{
						j1 = j;
						j2 = 2;
					}
					else if (j == y_count)
					{
						j1 = j - 1;
						j2 = 2;
					}


					if (i == 0)
					{
						indexsum[i].block(j1, k1, j2, k2).setConstant(1);
						indexsum[i + 1].block(j1, k1, j2, k2).setConstant(1);
					
					}
					else if (i == x_count)
					{
						indexsum[i - 1].block(j1, k1, j2, k2).setConstant(1);
						indexsum[i].block(j1, k1, j2, k2).setConstant(1);
					}
					else
					{
						indexsum[i - 1].block(j1, k1, j2, k2).setConstant(1);
						indexsum[i].block(j1, k1, j2, k2).setConstant(1);
						indexsum[i + 1].block(j1, k1, j2, k2).setConstant(1);
					}

					
				}
			}
		}
	}
	std::cout << n<<"   "<< maxcount << std::endl;
	MatrixXf pc1(n, 6);
	int  indexcount = 0;
	double o = 1 / step;
	o = o * o;
	double th = 0-cos(angle/ 180.0*3.1415926);
	for (int i = 0; i <= x_count; i++)
	{
		for (int j = 0; j <= y_count; j++)
		{
			for (int k = 0; k <= z_count; k++)
			{
				if (indexsum[i](j, k) == 1)
				{
					int kk = i * stepcount(1)*stepcount(2) + j * stepcount(2) + k;
					int nn = index[kk].size();
				
					if ( nn> 0)
					{
						RowVector3f center(0, 0, 0);
						for (int i1 = 0; i1 < nn; i1++)
						{
							center += pc.row(index[kk][i1]);
						}
						center = center / nn;

						Matrix3d cov = Matrix3d::Zero();
		
						for (int i1 = i - 1; i1 < i + 2; i1++)
						{
							if (i1 < 0 || i1>x_count)
								continue;
							for (int j1 = j - 1; j1 < j + 2; j1++)
							{
								if (j1 < 0 || j1>y_count)
									continue;
								for (int k1 = k - 1; k1 < k + 2; k1++)
								{
									if (k1 < 0 || k1>z_count)
										continue;

									int kk = i1 * stepcount(1)*stepcount(2) + j1 * stepcount(2) + k1;
									int nn = index[kk].size();
									for (int ii = 0; ii < nn; ii++)
									{
										RowVector3f a = pc.row(index[kk][ii]) - center;
										double d = a.squaredNorm();
										double w = exp(-d / o);
								
										cov += (a.transpose()*a*w).cast<double>();
									}

								}
							}
						}
						//cov /= tw;
						EigenSolver<Matrix3d> es(cov);
						Matrix3d D = es.pseudoEigenvalueMatrix();
						Matrix3d V = es.pseudoEigenvectors();
						//Vector3d Eigenvalue(D(0, 0), D(1, 1), D(2, 2));
						//std::cout << std::endl << D(0, 0)<<" " << D(1, 1) << " " << D(2, 2) << std::endl;
						int minindex = 0;
						double mind = D(0, 0);
						for (int i1 = 1; i1 < 3; i1++)
						{
							if (mind > D(i1, i1))
							{
								minindex = i1;
								mind = D(i1, i1);
							}
						}
						RowVector3f nor = V.col(minindex).transpose().cast<float>();
						if (nor(2) > 0)
							nor *= -1;

						if (nor(2) < th)
						{
							pc1.row(indexcount).head(3) = center;
							pc1.row(indexcount).tail(3) = nor;
							indexcount++;
						}
					//	std::cout << nor.norm()<< std::endl;
					
					}


				}
			}
		}
	}
	//std::cout <<" tt "<< indexcount << std::endl;
	//std::cout << " tt1 " << pc1.rows()<<" "<< pc1.cols()<< std::endl;
	
	//std::cout << pc1.block(0, 0, 100, 6) << std::endl;
	//{
	//	int mm;
	//	std::cin >> mm;
	//}
	pc1.conservativeResize(indexcount, 6);
	return pc1;
}
MatrixXf BinningDenormalnoise(MatrixXf &pc, float sampleStep)
{
	RowVector3f pc_max, pc_min, stepsize;

	int size = int(1 / sampleStep);
	for (int i = 0; i < 3; i++)
	{
		pc_max(i) = pc.col(i).maxCoeff();
		pc_min(i) = pc.col(i).minCoeff();
		stepsize(i) = 1.0 / ((pc_max(i) - pc_min(i)) / size + 0.001);
	}


	std::vector<MatrixXi> indexsum, indexsum2;
	indexsum.resize(size);
	indexsum2.resize(size);
	for (int i = 0; i < size; i++)
	{
		indexsum[i].resize(size, size);

		indexsum[i].setZero();
		indexsum2[i].resize(size, size);
		indexsum2[i].setZero();
	}
	int n = pc.rows();
	for (int i = 0; i < n; i++)
	{
		RowVector3f a = (pc.row(i).head(3) - pc_min).array()*stepsize.array();
		(indexsum[a(2)](int(a(0)), int(a(1)))) += 1;
	}

	int maxcount = 0;
	for (int i = 0; i < size; i++)
	{
		int max = indexsum[i].maxCoeff();
		if (max > maxcount)
		{
			maxcount = max;
		}
	}

	maxcount /= 10.0;
	int size1 = size - 1;
	for (int i = 1; i < size1; i++)
	{
		for (int j = 1; j < size1; j++)
		{
			for (int k = 1; k < size1; k++)
			{
				if (indexsum[k](i, j) > maxcount)
				{
					indexsum2[k - 1].block(i - 1, j - 1, 3, 3).setConstant(1);
					indexsum2[k].block(i - 1, j - 1, 3, 3).setConstant(1);
					indexsum2[k + 1].block(i - 1, j - 1, 3, 3).setConstant(1);
				}
			}
		}
	}

	VectorXi denoise(n);
	denoise.setZero();
	double d=0.0-cos(85.0 * 3.1415926 / 180);
	for (int i = 0; i < n; i++)
	{
		RowVector3f a = (pc.row(i).head(3) - pc_min).array()*stepsize.array();
	
		if (indexsum2[a(2)](int(a(0)), int(a(1))) == 1 && pc(i,5)< d)
			denoise(i) = 1;
	}

	int count = denoise.sum();

	MatrixXf pc1(count, 6);
	int c = 0;
	for (int i = 0; i < n; i++)
	{
		if (denoise(i) == 1)
		{
			pc1.row(c) = pc.row(i);
			c++;
		}
	}
	return pc1;
}

MatrixXf samplePCByQuantization_normal(MatrixXf pc, Vector2f& xrange, Vector2f& yrange, Vector2f& zrange, float sampleStep, float anglethreshold, int level, VectorXi& sumnum)
{

	//设置网格参数
	float xr = xrange[1] - xrange[0] + 0.001;//x的跨度
	float yr = yrange[1] - yrange[0] + 0.001;
	float zr = zrange[1] - zrange[0] + 0.001;

	//std::cout << xr << " " << xr << " " << xr << " " << sampleStep << std::endl;

	int numPoints = 0;


	int xnumSamplesDim = (int)(xr / sampleStep) + 1;//采样宽度数
	int ynumSamplesDim = (int)(yr / sampleStep) + 1;//采样宽度数
	int znumSamplesDim = (int)(zr / sampleStep) + 1;//采样宽度数
	std::vector< std::vector<int> > map;

	map.resize((xnumSamplesDim + 1)*(ynumSamplesDim + 1)*(znumSamplesDim + 1));//设置行数


	//std::cout << xnumSamplesDim << "  vvvv " << ynumSamplesDim << "  vvvv " << znumSamplesDim << "  vvvv " << map.size() << std::endl;



	for (int i = 0; i < pc.rows(); i++)
	{

		const int xCell = (int)((float)xnumSamplesDim*(pc(i, 0) - xrange[0]) / xr);//计算x轴索引下标
		const int yCell = (int)((float)ynumSamplesDim*(pc(i, 1) - yrange[0]) / yr);//计算y轴索引下标
		const int zCell = (int)((float)znumSamplesDim*(pc(i, 2) - zrange[0]) / zr);//计算z轴索引下标
		const int index = xCell * ynumSamplesDim*znumSamplesDim + yCell * znumSamplesDim + zCell;//计算在二维向量组中的下标
//#ifdef _OPENMP
//		omp_set_lock(&mylock1);
//#endif
		map[index].push_back(i);//把下标压入二维向量组		
//#ifdef _OPENMP
//		omp_unset_lock(&mylock1);
//#endif			
	}	


	//下采样


	//sumnum.resize(pc.rows());
	//sumnum.setOnes();
	//VectorXi dsumnum;
	//dsumnum.resize(pc.rows());
	//dsumnum.setZero();

	MatrixXf ypc=pc;
	MatrixXf dpc(pc.rows(),6);
	//sumnum.head
	int row = 0;
	int mapsize = map.size();
	double cosanglethreshold=cos(anglethreshold);
	
	for (int lev = 0; lev < level; lev++)
	{
		row = 0;
		//int nn = 0;
		int span = pow(2, lev);
		//std::cout << lev << " " << span << std::endl;
#ifdef _OPENMP
	//omp_lock_t    mylock1;
	//omp_init_lock(&mylock1);
   // #pragma omp parallel for schedule(dynamic,2) 
#endif		
		for (int i = 0; i <xnumSamplesDim; i+= span)
		{
			for (int j = 0; j <ynumSamplesDim; j+= span)
			{
				for (int k = 0; k <znumSamplesDim; k+= span)
				{
					//
					std::vector<Vector3f> normal;
					std::vector<Vector3f> clustnormal;
					std::vector<Vector3f> clustcoord;
					std::vector<int> count;
					std::vector<int> num;
					std::vector<int> total_num;
					//聚类
					for (int i1 = 0; i1 < span && i + i1<xnumSamplesDim; i1++)
					{
						for (int j1 = 0; j1 < span && j + j1<ynumSamplesDim; j1++)
						{
							for (int k1 = 0; k1 < span && k + k1<znumSamplesDim; k1++)
							{
								int index = (i + i1)*ynumSamplesDim*znumSamplesDim + (j + j1)*znumSamplesDim + (k + k1);
								//nn += map[index].size();
								for (int n = 0; n < map[index].size(); n++)
								{
									int yn = true;
									int m = map[index][n];

									Vector3f a(ypc(m, 3), ypc(m, 4), ypc(m, 5));
									Vector3f c(ypc(m, 0), ypc(m, 1), ypc(m, 2));
									//int num1= sumnum(m);
									for (int m = 0; m < normal.size(); m++)
									{
										float acosz = (a.dot(normal[m]));
										if (acosz > cosanglethreshold)
										{
											yn = false;
											clustnormal[m] = clustnormal[m] + a;
											clustcoord[m] = clustcoord[m] + c;
											count[m]++;
											//total_num[m] += num1;
											break;
										}
									}

									if (yn)
									{
										normal.emplace_back(a);
										clustnormal.emplace_back(a);
										clustcoord.emplace_back(c);
										count.emplace_back(1);
										//total_num.push_back(num1);
									}
								}



							}

						}
					}
						//清空
					for (int i1 = 0; i1 < span && i + i1<xnumSamplesDim; i1++)
					{
						for (int j1 = 0; j1 < span && j + j1<ynumSamplesDim; j1++)
						{
							for (int k1 = 0; k1 < span && k + k1<znumSamplesDim; k1++)
								{
									int index = (i + i1)*ynumSamplesDim*znumSamplesDim + (j + j1)*znumSamplesDim + (k + k1);
									map[index].clear();
									//std::cout << map[index].size()<<std::endl;
								}
							}

						}
						
						
						
						//插入
					int index = (i)*ynumSamplesDim*znumSamplesDim + (j)*znumSamplesDim + (k);
					for (int i1 = 0; i1 < clustnormal.size(); i1++)
					{
						double norm = clustnormal[i1].norm();
						if (norm > 0.000001) {
							Vector3f a = clustnormal[i1] / norm;

#ifdef _OPENMP
	//	omp_set_lock(&mylock1);
#endif
							
	
							dpc(row, 0) = clustcoord[i1](0) / count[i1];
							dpc(row, 1) = clustcoord[i1](1) / count[i1];
							dpc(row, 2) = clustcoord[i1](2) / count[i1];
							dpc(row, 3) = a(0);
							dpc(row, 4) = a(1);
							dpc(row, 5) = a(2);
							//dsumnum(row) = total_num[i1];
							/*if (count[i1] != total_num[i1])
							{
								std::cout << count[i1] << "  " << total_num[i1] << std::endl;
								system("pause");
							}*/
							map[index].emplace_back(row);
							row++;
	#ifdef _OPENMP
		//	omp_unset_lock(&mylock1);
	#endif
						}
						/*else
						{
							std::cout << total_num[i1] << std::endl;
						}*/
						
					}
				}

			}
		}
			
		
		//改写数据
		ypc = dpc.block(0,0,row,6);
	   // std::cout<< sumnum.sum() <<"xxx="<< dsumnum.sum()<<" " <<nn<< std::endl;
		//sumnum = dsumnum.head(row);
	}

	return ypc;
}
// uses a volume instead of an octree
// TODO: Right now normals are required.
 // This is much faster than (sample_pc_octree 八查树快速采样) 这个单元格方法快

MatrixXf sample_FSP(MatrixXf pc, int sample_pn)
{
	int row = pc.rows();

	VectorXf dismax(row);
	VectorXi maxindex = VectorXi::Constant(row, -1);

	dismax(0) = 0;
	maxindex(0) = 0;
	MatrixXf pcwn = pc.block(0, 0, row, 3);
	RowVector3f maxrow = pcwn.row(0);
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 1; i < row; i++)
	{
		dismax(i) = (maxrow - pcwn.row(i)).squaredNorm();
	}
		
	sample_pn = sample_pn - 2;
	
	for (int i = 0; i < sample_pn; i++)
	{
		int index= 0;
		dismax.maxCoeff(&index);
		maxrow = pcwn.row(index);
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int j = 1; j < row; j++)
		{
			
			double d1 = (maxrow - pcwn.row(j)).squaredNorm();
			
			//dismax[j] = d1 < dismax[j]?d1: dismax[j];
			
			
		}
	}

	int index = 0;
	for (int j = 0; j < row; j++)
	{
		if (dismax(j)==0)
		{
			pc.row(index) = pc.row(j);
			index++;
		}
	}
	pc.conservativeResize(index, pc.cols()); 
		
   return pc;


	/*int rowindex, colindex;
	double d=dismap.maxCoeff(&rowindex, &colindex);
	dismap.col(rowindex).setConstant(-1);
	dismap.col(colindex).setConstant(-1);
	dismax(rowindex) = dismap.row(rowindex).maxCoeff(&dismaxindex(rowindex));
	dismax(colindex) = dismap.row(rowindex).maxCoeff(&dismaxindex(colindex));

	sample_pn = sample_pn - 2;
	int index,k;

	for (int i = 0; i < sample_pn; i++)
	{
		d=dismax.maxCoeff(&index);
		k = dismaxindex(index);
		std::cout << k << std::endl;
		dismax(k) = dismap.row(k).maxCoeff(&dismaxindex(k));
		dismap.col(k).setConstant(-1);
#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int j = 0; j < row; j++)
		{
			if (dismaxindex(j)== k)
			{
				dismax(dismaxindex(j)) = dismap.row(dismaxindex(j)).maxCoeff(&dismaxindex(dismaxindex(j)));
			}
		}
	}
	index = 0;
	for (int j = 0; j < row; j++)
	{
		if (dismaxindex(j) > -1)
		{
			pc.row(index) = pc.row(j);
			index++;
		}
	}
	pc.conservativeResize(index, pc.cols());*/
	return pc;
}
MatrixXf samplePCByQuantization(MatrixXf pc, Vector2f& xrange, Vector2f& yrange, Vector2f& zrange, float sampleStep,  int weightByCenter)
{


	
	float xr = xrange[1] - xrange[0];//x的跨度
	float yr = yrange[1] - yrange[0];
	float zr = zrange[1] - zrange[0];

	int numPoints = 0;

	int xnumSamplesDim = (int)(xr / sampleStep);//采样宽度数
	int ynumSamplesDim = (int)(yr / sampleStep);//采样宽度数
	int znumSamplesDim = (int)(zr / sampleStep);//采样宽度数

	

	std::vector< std::vector<int> > scene_map;
	scene_map.resize((xnumSamplesDim + 1)*(ynumSamplesDim + 1)*(znumSamplesDim + 1));//设置行数
	

																				  //std::cout << numSamplesDim  << "  vvvv " << map.size()<< std::endl;


	for (int i = 0; i<pc.rows(); i++)
	{
		const int xCell = (int)((float)xnumSamplesDim*(pc(i, 0) - xrange[0]) / xr);//计算x轴索引下标
		const int yCell = (int)((float)ynumSamplesDim*(pc(i, 1) - yrange[0]) / yr);//计算y轴索引下标
		const int zCell = (int)((float)znumSamplesDim*(pc(i, 2) - zrange[0]) / zr);//计算z轴索引下标
		const int index = xCell*ynumSamplesDim*znumSamplesDim + yCell*znumSamplesDim + zCell;//计算在二维向量组中的下标

		scene_map[index].push_back(i);//把下标压入二维向量组
									  //  }
	}
	//std::cout << numSamplesDim << "  vvvv " << map.size() << std::endl;
	//std::cin >> m;
	for (unsigned int i = 0; i<scene_map.size(); i++)
	{
		numPoints += (scene_map[i].size()>0);//计算二维向量组行非零的行数
	}
	//std::cout << numPoints;
	//int ii;
	//std::cin >>ii;
	MatrixXf pcSampled(numPoints, pc.cols());
	int c = 0;

	for (unsigned int i = 0; i<scene_map.size(); i++)
	{
		double px = 0, py = 0, pz = 0;
		double nx = 0, ny = 0, nz = 0;

		std::vector<int> curCell = scene_map[i];//获得每个单元格内点的索引列表
		int cn = (int)curCell.size();//列表大小
		if (cn>0)
		{
			if (weightByCenter)//有加权计算
			{
				int xCell, yCell, zCell;
				double xc, yc, zc;
				double weightSum = 0;
		
				zCell = i % znumSamplesDim;//计算点云点的下标索引
				yCell = ((i - zCell) / znumSamplesDim) % ynumSamplesDim;
				xCell = ((i - zCell - yCell*ynumSamplesDim) / (ynumSamplesDim*znumSamplesDim));
				xc = ((double)xCell + 0.5) * (double)xr / xnumSamplesDim + (double)xrange[0];//计算单元格中心坐标
				yc = ((double)yCell + 0.5) * (double)yr / ynumSamplesDim + (double)yrange[0];
				zc = ((double)zCell + 0.5) * (double)zr / znumSamplesDim + (double)zrange[0];

				for (int j = 0; j<cn; j++)
				{
					const int ptInd = curCell[j];
					//float* point = &pc(ptInd,0);
					const double dx = pc(ptInd, 0) - xc;
					const double dy = pc(ptInd, 1) - yc;
					const double dz = pc(ptInd, 2) - zc;
					const double d = sqrt(dx*dx + dy*dy + dz*dz);
					double w = 0;

					if (d>EPS)//采用反距离加权
					{
						// it is possible to use different weighting schemes.
						// inverse weigthing was just good for me
						// exp( - (distance/h)**2 )
						//const double w = exp(-d*d);
						w = 1.0 / d;
					}

					//float weights[3]={1,1,1};
					px += w*(double)pc(ptInd, 0);
					py += w*(double)pc(ptInd, 1);
					pz += w*(double)pc(ptInd, 2);
					nx += w*(double)pc(ptInd, 3);
					ny += w*(double)pc(ptInd, 4);
					nz += w*(double)pc(ptInd, 5);

					weightSum += w;
				}
				px /= (double)weightSum;
				py /= (double)weightSum;
				pz /= (double)weightSum;
				nx /= (double)weightSum;
				ny /= (double)weightSum;
				nz /= (double)weightSum;
			}
			else
			{
				for (int j = 0; j<cn; j++)
				{
					int ptInd = curCell[j];//获得点云点的索引
					px += (double)pc(ptInd, 0);
					py += (double)pc(ptInd, 1);
					pz += (double)pc(ptInd, 2);
					nx += (double)pc(ptInd, 3);
					ny += (double)pc(ptInd, 4);
					nz += (double)pc(ptInd, 5);
				}

				px /= (double)cn;
				py /= (double)cn;
				pz /= (double)cn;
				//int ptInd = curCell[0];
				//px = pc(ptInd, 0);
				//py = pc(ptInd, 1);
				//pz = pc(ptInd, 2);
				nx /= (double)cn;
				ny /= (double)cn;
				nz /= (double)cn;

				//nx = pc(ptInd, 3);
				//ny = pc(ptInd, 4);
				//nz = pc(ptInd, 5);

			}

			//float *pcData = &pcSampled(c,0);
			pcSampled(c, 0) = (float)px;
			pcSampled(c, 1) = (float)py;
			pcSampled(c, 2) = (float)pz;

			// normalize the normals 法向归一化  法向为零处理
			double norm = sqrt(nx*nx + ny*ny + nz*nz);

			if (norm>EPS)
			{
				pcSampled(c, 3) = (float)(nx / norm);
				pcSampled(c, 4) = (float)(ny / norm);
				pcSampled(c, 5) = (float)(nz / norm);
				c++;
			}
			
			//#pragma omp atomic
			
			curCell.clear();
		}
	}
	//std::cout << c << "  下采样去掉法向为0  " << numPoints << std::endl;
	pcSampled.conservativeResize(c, 6);
	return pcSampled;
}


void shuffle(int *array, size_t n)
{
  size_t i;
  for (i = 0; i < n - 1; i++)
  {
    size_t j = i + rand() / (RAND_MAX / (n - i) + 1);
    int t = array[j];
    array[j] = array[i];
    array[i] = t;
  }
}

// compute the standard bounding box
void  computeBboxStd(MatrixXf pc, Vector2f& xRange, Vector2f& yRange, Vector2f& zRange)
{
 
  xRange[0] = pc.col(0).minCoeff();
  xRange[1] = pc.col(0).maxCoeff();
  yRange[0] = pc.col(1).minCoeff();
  yRange[1] = pc.col(1).maxCoeff();
  zRange[0] = pc.col(2).minCoeff();
  zRange[1] = pc.col(2).maxCoeff();
}
/*
* \矩阵放缩，均值为0 scale放缩倍数
*/
MatrixXf normalizePCCoeff(MatrixXf pc, float scale, float* Cx, float* Cy, float* Cz, float* MinVal, float* MaxVal)
{
  double minVal=0, maxVal=0;

  MatrixXf x,y,z, pcn;
  //pc.col(0).copyTo(x);
  //pc.col(1).copyTo(y);
  //pc.col(2).copyTo(z);
  x = pc.block(0, 0, pc.rows(), 1);
  y=  pc.block(0, 1, pc.rows(), 1);
  z = pc.block(0, 1, pc.rows(), 1);

  //float cx = (float) cv::mean(x)[0];
  //float cy = (float) cv::mean(y)[0];
  //float cz = (float) cv::mean(z)[0];

  float cx = x.mean();
  float cy = y.mean();
  float cz = z.mean();

  //cv::minMaxIdx(pc, &minVal, &maxVal);

  x=x.array()-cx;
  y=y.array() -cy;
  z=z.array() -cz;
  //pcn.create(pc.rows, 3, CV_32FC1);

  pcn.resize(pc.rows(), 3);
  pc.col(0) = x;
  pc.col(1) = y;
  pc.col(2) = z;

  //x.copyTo(pcn.col(0));
  //y.copyTo(pcn.col(1));
  //z.copyTo(pcn.col(2));

  //cv::minMaxIdx(pcn, &minVal, &maxVal);
  minVal = pcn.minCoeff();
  maxVal = pcn.maxCoeff();
  pcn=(float)scale*(pcn)/((float)maxVal-(float)minVal);

  *MinVal=(float)minVal;
  *MaxVal=(float)maxVal;
  *Cx=(float)cx;
  *Cy=(float)cy;
  *Cz=(float)cz;

  return pcn;
}
/*
* \给相关参数下的矩阵放缩，均值为0 scale放缩倍数
*/
MatrixXf transPCCoeff(MatrixXf pc, float scale, float Cx, float Cy, float Cz, float MinVal, float MaxVal)
{
	MatrixXf x,y,z, pcn;
    x=pc.col(0);
    y=pc.col(1);
    z=pc.col(2);

	x=x.array()-Cx;
	y=y.array() -Cy;
	z=z.array() -Cz;
    pcn.resize(pc.rows(), 3);
    pcn.col(0)=x;
    pcn.col(1)=y;
    pcn.col(2)=z;

    pcn=(float)scale*(pcn)/((float)MaxVal-(float)MinVal);

    return pcn;
}
MatrixXf transformPose3f(MatrixXf pc, const Matrix4d& Pose)
{
	MatrixXf pct(pc.rows(), 3);


	for (int i = 0; i<pc.rows(); i++)
	{
	
		
		Vector4d p = Pose * Vector4d(pc(i, 0), pc(i, 1), pc(i, 2), 1);
	

			pct(i, 0) = p(0);
			pct(i, 1) = p(1);
			pct(i, 2) = p(2);



	}

	return pct;
}

MatrixXf transformPCPosecood(MatrixXf pc, const Matrix4d& Pose)
{
	MatrixXf pct(pc.rows(), 3);

	Matrix3d R;
	Vector3d t;
	poseToRT(Pose, R, t);


	for (int i = 0; i<pc.rows(); i++)
	{
		//const float *pcData = &pc(i,0);
		const Vector3f n1(pc(i, 3), pc(i, 4), pc(i, 5));

		Vector4d p = Pose * Vector4d(pc(i, 0), pc(i, 1), pc(i, 2), 1);
		Vector3d p2(p.head(3));

		// p2[3] should normally be 1
		if (fabs(p[3]) > EPS)
		{
			//Mat((1.0 / p[3]) * p2).reshape(1, 1).convertTo(pct.row(i).colRange(0, 3), CV_32F);
			//Mat((1.0 / p[3]) * p2).reshape(1, 1) 变成行 并复制给 pct.row(i).colRange(0, 3)
			p2 = 1.0 / p[3] * p2;
			pct(i, 0) = p2(0);
			pct(i, 1) = p2(1);
			pct(i, 2) = p2(2);

		}

		
	}

	return pct;
}

/*
* \
*/
MatrixXf transformPCPose(MatrixXf & pc, const Matrix4d& Pose)
{
	MatrixXf pct(pc.rows(), pc.cols());
	MatrixXf pctp(pc.rows(),4);
	pctp.setOnes();
	pctp.block(0, 0, pc.rows(), 3) = pc.block(0, 0, pc.rows(), 3);

	MatrixXf pctp1 = Pose.block(0, 0, 3, 4).cast<float>()*pctp.transpose();
	MatrixXf pctp2 = Pose.block(0, 0, 3, 3).cast<float>()*pc.block(0, 3, pc.rows(), 3).transpose();
	
	pct.block(0, 0, pc.rows(), 3)= pctp1.transpose();
	pct.block(0, 3, pc.rows(), 3) = pctp2.transpose();
	
  return pct;
}


} // namespace ppf_match_3d

}