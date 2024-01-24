#include "point_cloud_registration.h"

#include "registration/registration_ppf_helpers.hpp"
#include "registration/dual_point.h"

#include "ICP_Princeton/TriMesh.h"
#include "ICP_Princeton/TriMesh_algo.h"
#include "ICP_Princeton/ICP.h"
#include "ICP_Princeton/KDtree.h"
#include<vector>
#include<fstream>
#include <io.h>
#include <random>
#include <chrono> 

using namespace std::chrono;
using namespace play3d;
using namespace ppf_match;
using namespace std;

namespace scan3d
{

	// 字符串分割函数
	std::vector<std::string> split(std::string str, std::string pattern)
	{
		std::string::size_type pos;
		std::vector<std::string> result;
		str += pattern;//扩展字符串以方便操作
		int size = str.size();
		for (int i = 0; i < size; i++)
		{
			pos = str.find(pattern, i);
			if (pos < size)
			{
				std::string s = str.substr(i, pos - i);
				result.push_back(s);
				i = pos + pattern.size() - 1;
			}
		}
		return result;
	}

void PointCloudRegistration::Registration_ICP_Princeton(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
{	
}

std::vector<Matrix4d> readpose(string gtpath, std::vector<int>& firstpointcloudname,
	std::vector<int> & secondpointcloudname)
{
	std::vector<Matrix4d> pointpose;
	ifstream fin;
	fin.open(gtpath, ios::in);
	if (!fin.is_open())
	{
		std::cout << gtpath << "无法找到这个文件！" << endl;
	}
	else
	{

		while (!fin.eof())
		{
			int i, j, k;
			fin >> i;
			if (fin.eof())
				break;
			fin >> j;
			fin >> k;

			firstpointcloudname.push_back(i);
			secondpointcloudname.push_back(j);

			Matrix4d pose;
			pose.setIdentity();

			fin >> pose(0, 0);
			fin >> pose(0, 1);
			fin >> pose(0, 2);
			fin >> pose(0, 3);

			fin >> pose(1, 0);
			fin >> pose(1, 1);
			fin >> pose(1, 2);
			fin >> pose(1, 3);

			fin >> pose(2, 0);
			fin >> pose(2, 1);
			fin >> pose(2, 2);
			fin >> pose(2, 3);

			fin >> pose(3, 0);
			fin >> pose(3, 1);
			fin >> pose(3, 2);
			fin >> pose(3, 3);

			pointpose.push_back(pose);
			//std::cout << i << endl;
			//std::cout << pose << endl;
		}

	}
	fin.close();
	return pointpose;
}

bool readfeat_match(std::string filename, MatrixXf& fristpoint, MatrixXf& scendpoint,VectorXf& corr_scores)
{
	ifstream fin;
	fin.open(filename, ios::in);
	if (!fin.is_open())
	{
		std::cout << filename << "无法找到这个文件！" << endl;
		return false;
	}
	else
	{
		
		fristpoint.resize(100000, 3);
		scendpoint.resize(100000, 3);
		corr_scores.resize(100000);
		int row = 0;
		while (1)
		{
			float a;
			fin >> a;
			if (fin.eof())
				break;
			fristpoint(row, 0) = a;
			fin >> fristpoint(row, 1);
			fin >> fristpoint(row, 2);

			fin >> scendpoint(row, 0);
			fin >> scendpoint(row, 1);
			fin >> scendpoint(row, 2);

			fin >> corr_scores(row);
			row++;
		}
		fristpoint.conservativeResize(row, 3);
		scendpoint.conservativeResize(row, 3);
		corr_scores.conservativeResize(row);
	}
	fin.close();

	//cout << keypoint.block(0, 0, 3, 3);
	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}

bool readfeat_stc_tgt(std::string filename, MatrixXf& fristpoint, MatrixXf& scendpoint, MatrixXf& fristfeats, MatrixXf& scendfeats)
{
	ifstream fin;
	fin.open(filename, ios::in);
	if (!fin.is_open())
	{
		std::cout << filename << "无法找到这个文件！" << endl;
		return false;
	}
	else
	{
		int src_n, tgt_n, featdim;
		double a;
		fin >> a;
		src_n = a + 0.5;
		fin >> a;
		tgt_n = a + 0.5;
		fin >> a;
		featdim = a + 0.5;
		fristpoint.resize(src_n, 3);
		scendpoint.resize(tgt_n, 3);
		fristfeats.resize(src_n, featdim);
		scendfeats.resize(tgt_n, featdim);
		int row = 0;

		for(int i=0;i< src_n;i++)
		{
			fin >> fristpoint(i, 0) ;
			fin >> fristpoint(i, 1);
			fin >> fristpoint(i, 2);
		}

		for (int i = 0; i < tgt_n; i++)
		{
			fin >> scendpoint(i, 0);
			fin >> scendpoint(i, 1);
			fin >> scendpoint(i, 2);

		}

		for (int i = 0; i < src_n; i++)
		{
			for (int j = 0; j < featdim; j++)
			{
				fin >> fristfeats(i, j);
			}
			
		}

		for (int i = 0; i < tgt_n; i++)
		{
			for (int j = 0; j < featdim; j++)
			{
				fin >> scendfeats(i, j);
			}

		}
		
	}
	fin.close();

	//cout << keypoint.block(0, 0, 3, 3);
	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}


bool readfeat_stc_tgt_kitti(std::string filename, MatrixXf& fristpoint, MatrixXf& scendpoint, MatrixXf& fristfeats, MatrixXf& scendfeats)
{
	ifstream fin;
	fin.open(filename, ios::in);
	if (!fin.is_open())
	{
		std::cout << filename << "无法找到这个文件！" << endl;
		return false;
	}
	else
	{
		int src_n, tgt_n, featdim;
		double a;
		fin >> a;
		src_n = a + 0.5;
		fin >> a;
		tgt_n = a + 0.5;
		fin >> a;
		featdim = a-3.0 + 0.5;
		std::cout << src_n << " "<< tgt_n<<" "<< featdim << endl;
		fristpoint.resize(src_n, 3);
		scendpoint.resize(tgt_n, 3);
		fristfeats.resize(src_n, featdim);
		scendfeats.resize(tgt_n, featdim);
		
		for (int i = 0; i < 16; i++)
		{
			fin >> a;
		}
		for (int i = 0; i < src_n; i++)
		{
			fin >> fristpoint(i, 0);
			fin >> fristpoint(i, 1);
			fin >> fristpoint(i, 2);
			for (int j = 0; j < featdim; j++)
			{
				fin >> fristfeats(i, j);
			}
		}

		for (int i = 0; i < tgt_n; i++)
		{
			fin >> scendpoint(i, 0);
			fin >> scendpoint(i, 1);
			fin >> scendpoint(i, 2);

			for (int j = 0; j < featdim; j++)
			{
				fin >> scendfeats(i, j);
			}

		}

		

	}
	fin.close();

	//cout << keypoint.block(0, 0, 3, 3);
	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}


bool readfeature(std::string filename, MatrixXf& fristkeypoint, MatrixXf& fristfeature,
	MatrixXf& secondkeypoint, MatrixXf& secondfeature)
{
	ifstream fin;
	fin.open(filename + "_size.txt", ios::in);
	if (!fin.is_open())
	{
		std::cout << filename + "_size.txt" << "无法找到这个文件！" << endl;
		return false;
	}
	else
	{
		float frows, fcols, srows, scols;
		fin >> frows;
		fin >> fcols;
		fin >> srows;
		fin >> scols;
		int rows, cols, s_rows, s_cols;
		rows = frows + 0.00001;
		cols = fcols + 0.00001;
		s_rows = srows + 0.00001;
		s_cols = scols + 0.00001;

		fristkeypoint.resize(rows, 3);
		fristfeature.resize(rows, cols - 3);
		secondkeypoint.resize(s_rows, 3);
		secondfeature.resize(s_rows, s_cols - 3);

		ifstream datafin;
		datafin.open(filename + ".txt", ios::in);
		if (!datafin.is_open())
		{
			std::cout << filename + ".txt" << "无法找到这个文件！" << endl;
			return false;
		}
		else
		{
			for (int i = 0; i < rows; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					datafin >> fristkeypoint(i, j);
				}
				for (int j = 0; j < cols - 3; j++)
				{
					datafin >> fristfeature(i, j);
				}
			}
			for (int i = 0; i < s_rows; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					datafin >> secondkeypoint(i, j);
				}
				for (int j = 0; j < s_cols - 3; j++)
				{
					datafin >> secondfeature(i, j);
				}
			}
		}
		datafin.close();
		cout << "特征点数" << rows << " " << s_rows << endl;;
	}
	fin.close();


	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}

bool readfeature(std::string filename, MatrixXf& keypoint, MatrixXf& feature, bool withnormal)
{
	ifstream fin;
	fin.open(filename + "_size.txt", ios::in);
	if (!fin.is_open())
	{
		std::cout << filename + "_size.txt" << "无法找到这个文件！" << endl;
		return false;
	}
	else
	{
		float frows, fcols;
		fin >> frows;
		fin >> fcols;
		int rows, cols;
		rows = frows + 0.00001;
		cols = fcols + 0.00001;
		if (withnormal)
		{
			keypoint.resize(rows, 6);
			feature.resize(rows, cols - 6);
		}
		else
		{
			keypoint.resize(rows, 3);
			feature.resize(rows, cols - 3);
		}

		ifstream datafin;
		datafin.open(filename + ".txt", ios::in);
		if (!datafin.is_open())
		{
			std::cout << filename + ".txt" << "无法找到这个文件！" << endl;
			return false;
		}
		else
		{
			for (int i = 0; i < rows; i++)
			{
				if (withnormal)
				{
					for (int j = 0; j < 6; j++)
					{
						datafin >> keypoint(i, j);
					}
					for (int j = 0; j < cols - 6; j++)
					{
						datafin >> feature(i, j);
					}
				}
				else
				{
					for (int j = 0; j < 3; j++)
					{
						datafin >> keypoint(i, j);
					}
					for (int j = 0; j < cols - 3; j++)
					{
						datafin >> feature(i, j);
					}
				}

			}

		}
		datafin.close();
	}
	fin.close();

	//cout << keypoint.block(0, 0, 3, 3);
	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}

bool readfeature2(std::string filename, MatrixXf &keypoint,MatrixXf &feature, bool withnormal)
{
	
	{
		
		
		ifstream datafin;
		datafin.open(filename, ios::in);
		if (!datafin.is_open())
		{
			std::cout << filename << "无法找到这个文件！" << endl;
			return false;
		}
		else
		{ 
			float frows, fcols;
			datafin >> frows;
			datafin >> fcols;
			int rows, cols;
			rows = frows + 0.00001;
			cols = fcols + 0.00001;
			if (withnormal)
			{
				keypoint.resize(rows, 6);
				feature.resize(rows, cols);
			}
			else
			{
				keypoint.resize(rows, 3);
				feature.resize(rows, cols);
			}

			for (int i = 0; i < rows; i++)
			{
				if (withnormal)
				{
					for (int j = 0; j < 6; j++)
					{
						datafin >> keypoint(i, j);
					}
					for (int j = 0; j < cols; j++)
					{
						datafin >> feature(i, j);
					}
				}
				else
				{
					for (int j = 0; j < 3; j++)
					{
						datafin >> keypoint(i, j);
					}
					for (int j = 0; j < cols; j++)
					{
						datafin >> feature(i, j);
					}
				}
				
			}

		}
		datafin.close();
	}
	

	//cout << keypoint.block(0, 0, 3, 3);
	//cout << feature.block(0, 0, 3, feature.cols());
	//cout << keypoint.block(keypoint.rows()-3, 0, 3, 3);
	//cout << feature.block(feature.rows() - 3, 0, 3, feature.cols());
	return true;
}
Vector4d dcm2quat(Matrix3d trans)
{
	
    Vector4d qout(4);
	qout(0) = 0.5 * sqrt(1 + trans(0, 0) + trans(1, 1) + trans(2, 2));
	qout(1) = -(trans(2, 1) - trans(1, 2)) / (4 * qout(0));
	qout(2) = -(trans(0, 2) - trans(2, 0)) / (4 * qout(0));
	qout(3) = -(trans(1, 0) - trans(0, 1)) / (4 * qout(0));

	//cout << qout << endl << endl;
	return qout;
}
double mrComputeTransformationError(MatrixXd trans, MatrixXd info)
{
	Vector3d  te = trans.block(0,3,3,1);
	Vector4d  qt = dcm2quat(trans.block(0, 0, 3, 3));
	VectorXd er(6);
	qt = qt * -1;
	er << te, qt.tail(3);
	RowVectorXd ert = er.transpose();
	MatrixXd p= ert * info* er/info(0, 0);
	return p(0,0);
}



void samplepoint(MatrixXf &pc, MatrixXf &feat,int pointnum)
{
	std::mt19937 gen((unsigned int)time(NULL) + (unsigned int)clock()); //10000 (unsigned int)time(NULL)+ (unsigned int)clock()
	int row = pc.rows();
	uniform_int_distribution<unsigned>  u(0, row - 1);//随机数分布对象，控制随机数范围5-15
	
	VectorXi index(row);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < row; i++)
		index(i) = i;

	for (int i = 0; i < pointnum; i++)
	{
		int r = u(gen);
		int a = index(i);
		index(i) = index(r);
		index(r) = a;
	}
	MatrixXf pc1(pointnum,pc.cols());
	MatrixXf feat1(pointnum, feat.cols());
#ifdef _OPENMP
#pragma omp parallel for 
#endif
	for (int i = 0; i < pointnum; i++)
	{
		pc1.row(i) = pc.row(index(i));
		feat1.row(i) = feat.row(index(i));
	}
	pc = pc1;
	feat = feat1;

}
void PointCloudRegistration::cal_pose()
{
	std::string ppath;

	ppath = "E:/pointclouddata/reslute/02.ply";
	MatrixXf fristpoint;

	fristpoint = loadPLYSimple_bin(ppath.c_str(), 1);

	//std::string  ransacpath = "E:/pointclouddata/wuhan/pic_data/9-HeritageBuilding/ransac.txt";
	//std::string  fastpath = "E:/pointclouddata/wuhan/pic_data/9-HeritageBuilding/fast.txt";
	std::string  ourpath = "E:/pointclouddata/reslute/02-03.txt";
	//std::string  gtpath = "E:/pointclouddata/wuhan/pic_data/9-HeritageBuilding/gt.txt";
	Matrix4d ransacpath_pose;
	Matrix4d fastpath_pose;
	Matrix4d ourpath_pose;
	Matrix4d gtpath_pose;
	ifstream fin;
	/*fin.open(ransacpath, ios::in);
	if (!fin.is_open())
	{
		std::cout << ransacpath << "无法找到这个文件！" << endl;
	}
	else
	{
		for (int v = 0; v < 4; v++)
		{
			for (int c = 0; c < 4; c++)
			{
				fin >> ransacpath_pose(v, c);
			}
		}

	}
	fin.close();

	fin.open(fastpath, ios::in);
	if (!fin.is_open())
	{
		std::cout << fastpath << "无法找到这个文件！" << endl;
	}
	else
	{
		for (int v = 0; v < 4; v++)
		{
			for (int c = 0; c < 4; c++)
			{
				fin >> fastpath_pose(v, c);
			}
		}

	}
	fin.close();*/


	fin.open(ourpath, ios::in);
	if (!fin.is_open())
	{
		std::cout << ourpath << "无法找到这个文件！" << endl;
	}
	else
	{
		for (int v = 0; v < 4; v++)
		{
			for (int c = 0; c < 4; c++)
			{
				fin >> ourpath_pose(v, c);
			}
		}

	}
	fin.close();

	/*fin.open(gtpath, ios::in);
	if (!fin.is_open())
	{
		std::cout << gtpath << "无法找到这个文件！" << endl;
	}
	else
	{
		for (int v = 0; v < 4; v++)
		{
			for (int c = 0; c < 4; c++)
			{
				fin >> gtpath_pose(v, c);
			}
		}

	}
	fin.close();*/

	MatrixXf model;
	/*std::cout << ransacpath_pose << endl << "ransacpath_pose！" << endl;
	MatrixXf model = transformPCPose(fristpoint, ransacpath_pose);;
	writePLY(model, (ransacpath + ".ply").c_str());

	model = transformPCPose(fristpoint, fastpath_pose);;
	std::cout << fastpath_pose << endl << "fastpath_pose！" << endl;
	writePLY(model, (fastpath + ".ply").c_str());

	model = transformPCPose(fristpoint, gtpath_pose);;
	std::cout << gtpath_pose << endl << " gtpath_pose！" << endl;
	writePLY(model, (gtpath + ".ply").c_str());*/

	model = transformPCPose(fristpoint, ourpath_pose);;
	std::cout << ourpath_pose << endl << "ourpath_pose！" << endl;
	writePLY(model, (ourpath + ".ply").c_str());

	
}
void PointCloudRegistration::cal_trans(std::string dataname)
{
	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> pointcloudname;
	std::vector<std::string> transname;

	std::string pointcloudpath = "E:/pointclouddata/wuhan/" + dataname + "pointcloudnormal";
	std::string transpath = "E:/pointclouddata/wuhan/" + dataname + "1-RawPointCloud";

	if ((hFile = _findfirst(p.assign(pointcloudpath).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			cout << fileinfo.name << " " << fileinfo.attrib << " " << _A_SUBDIR << endl;
			if ((fileinfo.attrib & _A_ARCH))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					std::string name = fileinfo.name;
					std::vector<std::string> filename = split(name, ".");
					//cout << name << endl;
					if (filename.size() > 1 && filename[filename.size() - 1] == "ply")
					{
						pointcloudname.push_back(name);
						transname.push_back(filename[0] + ".txt");

						//firstpointcloudname.push_back(atoi(filename[0].c_str()));
						//secondpointcloudname.push_back(atoi(filename[1].c_str()));
					}
					else
					{
						//std::cout << " file error " << std::endl;
					}
				}
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);*/

			//cout << fileinfo.name << endl;
			}

		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);

	for (int i = 0; i < pointcloudname.size(); i++)
	{

		std::string ppath;
		ppath = pointcloudpath + "/" + pointcloudname[i];
		std::cout << ppath << std::endl;

		std::string tpath;
		tpath = transpath + "/" + transname[i];
		std::cout << tpath << std::endl;

		MatrixXf fristpoint;
	
		fristpoint = loadPLYSimple_bin(ppath.c_str(), 1);

		ifstream fin;
		Vector3f trans(3);
		fin.open(tpath);
		if (!fin.is_open())
		{
			std::cout << tpath << "无法找到这个文件！" << endl;
		}
		else
		{
			for (int v = 0; v < 3; v++)
			{
				fin >> trans(v);
			}

		}


		fin.close();
		for (int v = 0; v < 3; v++)
		  fristpoint.col(v) = (fristpoint.col(v).array()- trans(v));
		
		writePLY(fristpoint, (ppath+".ply").c_str());
	}
}

void PointCloudRegistration::cal_point_cloud_FPFH(std::string filename, int pointnum, bool withnormal)
{
	
	
	{

		std::string ppath;
		ppath = filename;
		std::cout << ppath << std::endl;


		MatrixXf fristpoint;
		///无方向
		if (withnormal)
		{
			fristpoint = loadPLYSimple_bin(ppath.c_str(), 1);
		}
		else
		{
			fristpoint = loadPLYSimple_bin(ppath.c_str(), 0);
			std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
			fristpoint = StatisticsDenoise(fristpoint, 10, 0.99);
			std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;


			std::cout << "计算法向" << std::endl;
			Vector3f centerpoint(0, 0, 0);
			fristpoint = normal(fristpoint, centerpoint, 25, 0.08);//30
			std::cout << "计算法向结束" << std::endl;
		}



		//下采样

		int n = 100;
		Vector2f xRange, yRange, zRange;
		computeBboxStd(fristpoint, xRange, yRange, zRange);
		// compute sampling step from diameter of bbox 计算包围盒直径和 采样半径
		float dx = xRange[1] - xRange[0];
		float dy = yRange[1] - yRange[0];
		float dz = zRange[1] - zRange[0];

		//模型直径
		double model_diameter = sqrt(dx * dx + dy * dy + dz * dz);

		MatrixXf downpoint;
		double samplingdistance = 0.05 * model_diameter;
		double min = 0.5 * model_diameter;
		double max = 0.0001 * model_diameter;
		while (n--)
		{
			downpoint = samplePCByQuantization(fristpoint, xRange, yRange, zRange, samplingdistance, 0);
			//_R_model_para.model.downsample_model = samplePCByQuantization_normal(_R_model_para.model.original_model,
			//	xRangelist[0], yRangelist[0], zRangelist[0],
			//	(float)_R_model_para.model.model_samplingdistance,
			//	model_parameter.Sampling_threshold_Angles / 180 * 3.1415, 1, sumnum_model);//无加权点云下采样
			std::cout << "下采样模型点数：" << downpoint.rows() << std::endl;
			if (fabs(downpoint.rows() - pointnum) < 500)
			{
				break;
			}
			if (downpoint.rows() > pointnum)
			{
				max = samplingdistance;
				samplingdistance = (max + min) / 2.0;
			}
			else
			{
				min = samplingdistance;
				samplingdistance = (max + min) / 2.0;
			}

		}

		//计算特征

		std::cout << "计算特征匹配 " << std::endl;

		//calculate features;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);



		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimation;

		pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMothodPtr(new pcl::search::KdTree<pcl::PointXYZ>);


		pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);


		//Establishes the search radius for the feature.

		double feature_radius = samplingdistance * 6;
		int model_row = downpoint.rows();



		source_xyz->points.resize(model_row);
		source_normal->points.resize(model_row);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < model_row; i++)
		{
			source_xyz->points[i].x = downpoint(i, 0);
			source_xyz->points[i].y = downpoint(i, 1);
			source_xyz->points[i].z = downpoint(i, 2);

			source_normal->points[i].normal_x = downpoint(i, 3);
			source_normal->points[i].normal_y = downpoint(i, 4);
			source_normal->points[i].normal_z = downpoint(i, 5);
		}

		std::cout << "计算特征匹配 " << std::endl;
		featureEstimation.setInputCloud(source_xyz);
		featureEstimation.setInputNormals(source_normal);
		featureEstimation.setSearchMethod(searchMothodPtr);
		featureEstimation.setRadiusSearch(feature_radius);
		featureEstimation.compute(*source_features);
		std::cout << "计算特征匹配结束 " << std::endl;

		//保存

		
		std::vector<std::string> fname = split(filename, ".");
		std::string fppath =  fname[0] + "_fpfh.npz.txt";
		std::string sizeppath = fname[0] + "_fpfh.npz_size.txt";
		ofstream fgfhout;
		fgfhout.open(fppath, ios::out);
		for (int i = 0; i < model_row; i++)
		{
			fgfhout << downpoint(i, 0) << " ";
			fgfhout << downpoint(i, 1) << " ";
			fgfhout << downpoint(i, 2) << " ";
			if (withnormal)
			{
				fgfhout << downpoint(i, 3) << " ";
				fgfhout << downpoint(i, 4) << " ";
				fgfhout << downpoint(i, 5) << " ";
			}
			for (int j = 0; j < 32; j++)
			{
				fgfhout << source_features->points[i].histogram[j] << " ";
			}
			fgfhout << source_features->points[i].histogram[32] << std::endl;
		}
		fgfhout.close();
		ofstream fgfhsizeout;
		fgfhsizeout.open(sizeppath, ios::out);
		fgfhsizeout << model_row << std::endl;
		if (withnormal)
			fgfhsizeout << 39 << std::endl;
		else
			fgfhsizeout << 36 << std::endl;
		fgfhsizeout.close();
	}
}


void PointCloudRegistration::cal_FPFH(std::string dataname, int pointnum,bool withnormal)
{
	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> pointcloudname;


	std::string pointcloudpath = "E:/pointclouddata/wuhan/"+ dataname +"pointcloudnormal"  ;

	if ((hFile = _findfirst(p.assign(pointcloudpath).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			cout<< fileinfo.name<<" " << fileinfo.attrib<<" " << _A_SUBDIR << endl;
			if ((fileinfo.attrib & _A_ARCH))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					std::string name = fileinfo.name;
					std::vector<std::string> filename = split(name, ".");
					//cout << name << endl;
					if (filename.size()>1 && filename[filename.size()-1] == "ply")
					{
						pointcloudname.push_back(name);
						
						//firstpointcloudname.push_back(atoi(filename[0].c_str()));
						//secondpointcloudname.push_back(atoi(filename[1].c_str()));
					}
					else
					{
						//std::cout << " file error " << std::endl;
					}
				}
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);*/

			//cout << fileinfo.name << endl;
			}

		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);

	for (int i = 0; i < pointcloudname.size(); i++)
	{

		std::string ppath;
		ppath = pointcloudpath+"/" + pointcloudname[i];
		std::cout << ppath << std::endl;
	

		MatrixXf fristpoint;
		///无方向
		if (withnormal)
		{
			fristpoint = loadPLYSimple_bin(ppath.c_str(), 1);
		}
		else
		{
			fristpoint = loadPLYSimple_bin(ppath.c_str(), 0);
			std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
			fristpoint = StatisticsDenoise(fristpoint, 10, 0.99);
			std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;


			std::cout << "计算法向" << std::endl;
			Vector3f centerpoint(0, 0, 0);
			fristpoint = normal(fristpoint, centerpoint, 25, 0.08);//30
			std::cout << "计算法向结束" << std::endl; 
		}
		

		
		//下采样

		int n = 100;
		Vector2f xRange, yRange, zRange;
		computeBboxStd(fristpoint, xRange, yRange, zRange);
		// compute sampling step from diameter of bbox 计算包围盒直径和 采样半径
		float dx = xRange[1] - xRange[0];
		float dy = yRange[1] - yRange[0];
		float dz = zRange[1] - zRange[0];

		//模型直径
		double model_diameter = sqrt(dx * dx + dy * dy + dz * dz);

		MatrixXf downpoint;
		double samplingdistance = 0.05* model_diameter;
		double min = 0.5 * model_diameter;
		double max = 0.0001 * model_diameter;
		while (n--)
		{
			downpoint = samplePCByQuantization(fristpoint, xRange, yRange, zRange, samplingdistance, 0);
			//_R_model_para.model.downsample_model = samplePCByQuantization_normal(_R_model_para.model.original_model,
			//	xRangelist[0], yRangelist[0], zRangelist[0],
			//	(float)_R_model_para.model.model_samplingdistance,
			//	model_parameter.Sampling_threshold_Angles / 180 * 3.1415, 1, sumnum_model);//无加权点云下采样
			std::cout << "下采样模型点数：" << downpoint.rows() << std::endl;
			if (fabs(downpoint.rows() - pointnum) < 500)
			{
				break;
			}
			if (downpoint.rows() > pointnum)
			{
				max =samplingdistance;
				samplingdistance =  (max + min) / 2.0;
			}
			else
			{
				min =samplingdistance;
				samplingdistance = (max + min) / 2.0;
			}
			
		}
		
		//计算特征

		std::cout << "计算特征匹配 " << std::endl;

		//calculate features;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

		

		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimation;

		pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMothodPtr(new pcl::search::KdTree<pcl::PointXYZ>);


		pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);

		
		//Establishes the search radius for the feature.

		double feature_radius = samplingdistance *6;
		int model_row = downpoint.rows();



		source_xyz->points.resize(model_row);
		source_normal->points.resize(model_row);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < model_row; i++)
		{
			source_xyz->points[i].x = downpoint(i, 0);
			source_xyz->points[i].y = downpoint(i, 1);
			source_xyz->points[i].z = downpoint(i, 2);

			source_normal->points[i].normal_x = downpoint(i, 3);
			source_normal->points[i].normal_y = downpoint(i, 4);
			source_normal->points[i].normal_z = downpoint(i, 5);
		}

		std::cout << "计算特征匹配 " << std::endl;
		featureEstimation.setInputCloud(source_xyz);
		featureEstimation.setInputNormals(source_normal);
		featureEstimation.setSearchMethod(searchMothodPtr);
		featureEstimation.setRadiusSearch(feature_radius);
		featureEstimation.compute(*source_features);
		std::cout << "计算特征匹配结束 " << std::endl;

		//保存
		
		std::string sppath = pointcloudpath + "/" ;
		std::vector<std::string> filename = split(pointcloudname[i], ".");
		std::string fppath = sppath + filename[0] + "_fpfh.npz.txt";
		std::string sizeppath = sppath + filename[0] + "_fpfh.npz_size.txt";
		ofstream fgfhout;
		fgfhout.open(fppath, ios::out);
		for (int i = 0; i < model_row; i++)
		{
			fgfhout << downpoint(i, 0) << " ";
			fgfhout << downpoint(i, 1) << " ";
			fgfhout << downpoint(i, 2) << " ";
			if (withnormal)
			{
				fgfhout << downpoint(i, 3) << " ";
				fgfhout << downpoint(i, 4) << " ";
				fgfhout << downpoint(i, 5) << " ";
			}
			for (int j = 0; j < 32; j++)
			{
				fgfhout << source_features->points[i].histogram[j] << " ";
			}
			fgfhout << source_features->points[i].histogram[32] << std::endl;
		}
		fgfhout.close();
		ofstream fgfhsizeout;
		fgfhsizeout.open(sizeppath, ios::out);
		fgfhsizeout << model_row << std::endl;
		if (withnormal)
			fgfhsizeout << 39 << std::endl;
		else
			fgfhsizeout << 36<< std::endl;
		fgfhsizeout.close();
	}
}


void PointCloudRegistration::cal_reT(std::string dataname)
{

	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;

	std::string pointcloudpath = "E:/pointclouddata/wuhan/" + dataname + "";
	std::string transpath = "E:/pointclouddata/wuhan/" + dataname + "1-RawPointCloud/";

	
	std::vector<std::string> pointcloudpairname;
	string p;
	if ((hFile = _findfirst(p.assign(pointcloudpath).append("3-GroundTruth/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			//cout<< fileinfo.name<<" " << fileinfo.attrib<<" " << _A_SUBDIR << endl;
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					std::string name = fileinfo.name;
					cout << name << endl;
					pointcloudpairname.push_back(name);

				}

			}
		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);


	std::vector<std::string> gtdirname;
	MatrixXf reslutvalue(pointcloudpairname.size(), 4);


	for (int i = 0; i < pointcloudpairname.size(); i++)//
	{



		string gtpath = pointcloudpath + "3-GroundTruth/" + pointcloudpairname[i] + "/transformation_gt.txt";

		std::cout << gtpath << std::endl;
		Matrix4f gt_pose;
		ifstream fin;
		fin.open(gtpath, ios::in);
		if (!fin.is_open())
		{
			std::cout << gtpath << "无法找到这个文件！" << endl;
		}
		else
		{
			for (int v = 0; v < 4; v++)
			{
				for (int c = 0; c < 4; c++)
				{
					fin >> gt_pose(v, c);
				}
			}

		}
		fin.close();

		std::vector<std::string> pointfilename = split(pointcloudpairname[i], "-");
		if (pointfilename.size() != 2)
		{
			std::cout << pointcloudpairname[i] << std::endl;
			continue;
		}
		std::cout << pointcloudpairname[i] << std::endl;
		



		std::string first_pc_name = std::string(2 - pointfilename[0].length(), '0') + pointfilename[0];
		std::string second_pc_name = std::string(2 - pointfilename[1].length(), '0') + pointfilename[1];

		std::string fcpath, scpath;
		fcpath = transpath + first_pc_name + ".txt";
		scpath = transpath + second_pc_name + ".txt";

	
		Vector3f ftrans(3);
		fin.open(fcpath);
		if (!fin.is_open())
		{
			std::cout << fcpath << "无法找到这个文件！" << endl;
		}
		else
		{
			for (int v = 0; v < 3; v++)
			{
				fin >> ftrans(v);
			}

		}


		fin.close();
		Vector3f strans(3);
		fin.open(scpath);
		if (!fin.is_open())
		{
			std::cout << fcpath << "无法找到这个文件！" << endl;
		}
		else
		{
			for (int v = 0; v < 3; v++)
			{
				fin >> strans(v);
			}
		}
		fin.close();

		cout << gt_pose << endl;
		cout << ftrans << endl;
		cout << strans << endl;

		Matrix4f trans;
		trans.setIdentity();
		trans.block(0, 3, 3, 1) = ftrans;
		cout << trans << endl;
		gt_pose=gt_pose* trans;
		trans.setIdentity();
		trans.block(0, 3, 3, 1) = -1*strans;
		cout << trans << endl;
		gt_pose =trans*  gt_pose ;
		cout << gt_pose << endl;

		gtpath = pointcloudpath + "3-GroundTruth/" + pointcloudpairname[i] + "/transformation.txt";
		ofstream posefout;
		posefout.open(gtpath, ios::out);
		for (int v = 0; v < 4; v++)
		{
			for (int c = 0; c < 4; c++)
			{
				posefout << gt_pose(v, c)<<"  ";
			}
			posefout << endl;
		}
		posefout.close();

	}
}

void PointCloudRegistration::Registration_PPF_KITTI(std::string dataname, std::string savename, MatrixXi r_pair, double value)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num =500;// 500;
	model_parameter.clutter_veri = 10 + 0.1;//8
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis =2.5;// 0.5;
	model_parameter.overlap_vl_angle = 3.0;//2.0
	model_parameter.overlap_n = 0.3;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 3.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	clock_t tick1 = clock();


	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> pointcloudpairname;


	std::string pointcloudpath = "E:/pointclouddata/data_odometry_velodyne/icp";

	if ((hFile = _findfirst(p.assign(pointcloudpath).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			//cout<< fileinfo.name<<" " << fileinfo.attrib<<" " << _A_SUBDIR << endl;
			if ((fileinfo.attrib & _A_ARCH))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					std::string name = fileinfo.name;
					std::vector<std::string> filename = split(name, ".");
					//cout << name << endl;
					if (filename.size() > 1 && filename[filename.size() - 1] == "npy")
					{
						pointcloudpairname.push_back(name);
						//cout << name << endl;
						//firstpointcloudname.push_back(atoi(filename[0].c_str()));
						//secondpointcloudname.push_back(atoi(filename[1].c_str()));
					}
					else
					{
						//std::cout << " file error " << std::endl;
					}
				}
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);*/
			}
		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);
	


	MatrixXf analysis_re = MatrixXf::Zero(10, 8);
	/*
	*/
	for (int ana = 0; ana < 10; ana++)
	{
		scenematching_parameter.Clusteringdistance = 3.0;
		analysis_re(ana, 0) = scenematching_parameter.Clusteringdistance;
	


		bool yn = true;
		

		std::vector<std::vector<string>> re_08;
		std::vector<std::vector<string>> re_09;
		std::vector<std::vector<string>> re_10;

		std::vector<std::vector<double>> res;
		res.resize(3);
		for (int i = 0; i < 3; i++)
		{
			res[i].resize(5);
			for (int j = 0; j < 3; j++)
			{
				res[i][j] = 0;
			}
		}

		for (int i = 0; i < pointcloudpairname.size(); i++)
		{
			

			std::vector<std::string> filename = split(pointcloudpairname[i], ".");
			std::vector<std::string> pointfilename = split(filename[0], "_");
			if (pointfilename.size() != 3)
			{
				std::cout << filename[0]<< std::endl;
				continue;
			}

			/*if ((filename[0] != "8_2934_2949"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
			{
				continue;
			}*/

			std::cout << filename[0] << std::endl;

			std::string sequence_name = std::string(2 - pointfilename[0].length(), '0') + pointfilename[0];
			std::string first_pc_name = std::string(6 - pointfilename[1].length(), '0') + pointfilename[1];
			std::string second_pc_name = std::string(6 - pointfilename[2].length(), '0') + pointfilename[2];
			
			/*if ((sequence_name == "10"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
			{
				continue;
			}
			//8_3419_3446
			if ((filename[0] != "8_3419_3446" &&  sequence_name != "09"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
			{
				continue;
			}
			if ((filename[0] != "8_3419_3446" && filename[0] != "9_1210_1217"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
			{
				continue;
			}*/
			std::vector<string> re;
			re.push_back(sequence_name);
			re.push_back(first_pc_name);
			re.push_back(second_pc_name);

			
			//read pose
			ifstream fin;
			MatrixXd gt_pose(4, 4);
			fin.open(pointcloudpath+"/"+ pointcloudpairname[i], ios::in);
			if (!fin.is_open())
			{
				std::cout << pointcloudpath + "/" + pointcloudpairname[i] << "无法找到这个文件！" << endl;
			}
			else
			{
				for (int v = 0; v < 4; v++)
				{
						for (int c = 0; c < 4; c++)
						{
							fin >> gt_pose(v, c);
						}
				}
					
			}

		
			fin.close();

			std::string featuretype = "fpfh";
			std::string pointcloudpath = "E:/pointclouddata/data_odometry_velodyne/sequences/" + sequence_name + "/pointcloud/";
			
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;

				if ((featuretype == "fcgf"))
				{
					std::string featpath = "E:/pointclouddata/data_odometry_velodyne/feat_fcgf_txt/" + filename[0] + ".npz.txt";
					std::cout << featpath << std::endl;
					if (!readfeat_stc_tgt_kitti(featpath, fristkeypoint, secondkeypoint, fristfeature, secondfeature))
					{
						continue;
					}
				}
				
				else if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpath + first_pc_name + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + first_pc_name + "_fpfh.npz", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + second_pc_name + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + second_pc_name + "_fpfh.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				if (fristkeypoint.rows() > 5000)
					samplepoint(fristkeypoint, fristfeature, 5000);
				if (secondkeypoint.rows() > 5000)
					samplepoint(secondkeypoint, secondfeature,5000);
				cout << "samplepoint " << fristkeypoint.rows() << " " << secondkeypoint.rows() << endl;

				std::string fcpath, scpath;
				fcpath = pointcloudpath + first_pc_name + ".bin.ply";
				scpath = pointcloudpath + second_pc_name + ".bin.ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

				MatrixXf fristpoint, secondpoint;
				fristpoint = loadPLYSimple(fcpath.c_str(), 1);
				secondpoint = loadPLYSimple(scpath.c_str(), 1);


			
				//MatrixXf model = transformPCPose(fristpoint, pointpose[i-0].inverse());

				//writePLY(model, "G:/paper/modelf.ply");
				//writePLY(secondpoint, "G:/paper/scenef.ply");
               // system("pause");
				//continue;

// 
// 				
				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);

				//ICP_Optimizing(MatrixXf source, MatrixXf target, int max_iter);
				//根据对比论文
				
				{//拼接点云计算法向
					MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
					MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
					fristm << fristkeypoint, fristpoint.block(0,0, fristpoint.rows(), 3);
					fristpoint = fristm;
					secondm << secondkeypoint, secondpoint.block(0, 0, secondpoint.rows(), 3);
					secondpoint = secondm;
				}
				
				

				std::cout << "计算法向" << std::endl;
				Vector3f centerpoint(0, 0, 0);
				MatrixXf fristpointcloud = normal(fristpoint, centerpoint, 25, 0.08);//30
				MatrixXf secondpointcloud = normal(secondpoint, centerpoint, 25, 0.08);//30
				std::cout << "计算法向结束" << std::endl;

				
					
				detectorpso.set_groundturth_pose(gt_pose);
					//std::cout << "groundturth:" << endl;
					//std::cout << pointpose[i + 1]*pointpose[i].inverse()  << endl;
					//std::cout << (pointpose[i + 1]*pointpose[i].inverse()).inverse() << endl;

				
				

				clock_t tick1 = clock();
				if ((featuretype == "fcgf")  || (featuretype == "fpfh"))
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配

					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
		
					//system("pause");
				}
				

				
				//system("pause");



				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				
				Matrix4d prop = results[0].Pose;
				
				//system("pause");


				//MatrixXf model = transformPCPose(fristpointcloud, gt_pose);
				//overlap = calvoerlap(model.block(0,0, model.rows(),3), secondpointcloud.block(0, 0, secondpointcloud.rows(), 3), 0.03);

				//std::cout << "fristpointcloud rows=" << fristpointcloud.rows() << endl;
				//std::cout << "secondpointcloud rows=" << secondpointcloud.rows() << endl;
				//writePLY(model, "G:/paper/modelf.ply");
				//writePLY(secondpointcloud, "G:/paper/scenef.ply");

				//MatrixXf pmodel = transformPCPose(fristpointcloud, prop);
				//overlap = calvoerlap(model.block(0,0, model.rows(),3), secondpointcloud.block(0, 0, secondpointcloud.rows(), 3), 0.03);

				//std::cout << "fristpointcloud rows=" << fristpointcloud.rows() << endl;
				//std::cout << "secondpointcloud rows=" << secondpointcloud.rows() << endl;
				//writePLY(pmodel, "G:/paper/pmodelf.ply");
				



				{
					re.push_back(to_string((double)(tick3 - tick1) / CLOCKS_PER_SEC));
					double acc = 0;
					double rre = 0;
					double rte = 0;
					{
						Matrix4d dp = gt_pose * prop.inverse();
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1)    d = -1;
						acos(d);

						double dis = (gt_pose.block(0, 3, 3, 1) - prop.block(0, 3, 3, 1)).norm();
						dis;

						std::cout << "est_pose da=" << acos(d)/M_PI*180 << endl;
						std::cout << "est_pose dt=" << dis << endl;

						re.push_back(to_string(acos(d) / M_PI * 180));
						re.push_back(to_string(dis));

						if (acos(d)/ M_PI *180 < 5 && dis < 2.0)
						{
							re.push_back("1");
							acc = 1;
							rre = acos(d) / M_PI * 180;
							rte = dis * 100;
						}
						else
						{
							re.push_back("0");
						}

					}
					


					//std::cout << "pose " << std::endl << prop << std::endl;
					//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
					std::cout << "  registration in "
						<< (double)(tick3 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;
					
					
					ofstream fout;
					fout.open("E:/pointclouddata/data_odometry_velodyne/re_" + sequence_name + ".txt", ios::out);
					if ((sequence_name == "08"))
					{
						re_08.push_back(re);
						for(int ri=0;ri<re_08.size();ri++)
						{
							for (int rj = 0; rj < re_08[ri].size(); rj++)
								fout << re_08[ri][rj] << " ";
							fout << std::endl;
						}
						res[0][0] += 1.0;
						res[0][1] += acc;
						res[0][2] += rre;
						res[0][3] += rte;
						res[0][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;

					}
					else if ((sequence_name == "09"))
					{
						re_09.push_back(re);
						for (int ri = 0; ri < re_09.size(); ri++)
						{
							for (int rj = 0; rj < re_09[ri].size(); rj++)
								fout << re_09[ri][rj] << " ";
							fout << std::endl;
						}
						res[1][0] += 1.0;
						res[1][1] += acc;
						res[1][2] += rre;
						res[1][3] += rte;
						res[1][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					}
					else
					{
						re_10.push_back(re);
						for (int ri = 0; ri < re_10.size(); ri++)
						{
							for (int rj = 0; rj < re_10[ri].size(); rj++)
								fout << re_10[ri][rj] << " ";
							fout << std::endl;
						}
						res[2][0] += 1.0;
						res[2][1] += acc;
						res[2][2] += rre;
						res[2][3] += rte;
						res[2][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					}
					fout.close();

					double total_num =0;
					double total_acc = 0;
					double total_rre = 0;
					double total_rte = 0;
					double total_time = 0;
					double avg_acc = 0;
					for (int ri = 0; ri < 3; ri++)
					{
						total_num += res[ri][0];
						total_acc += res[ri][1];
						total_rre += res[ri][2];
						total_rte += res[ri][3];
						total_time += res[ri][4];
						avg_acc += res[ri][1] / res[ri][0];
						cout << res[ri][0] << " " << res[ri][1] << " " << res[ri][1] / res[ri][0] << " " << res[ri][2] / res[ri][1] << " " << res[ri][3] / res[ri][1]
							<< " " << res[ri][4] / res[ri][0] << std::endl;
						
					}
					cout << total_num << " " << total_acc << " " << total_acc / total_num << " " << total_rre / total_acc << " " << total_rte / total_acc
						<< " " << total_time / total_num << std::endl;

					analysis_re(ana, 1) = total_num;
					analysis_re(ana, 2) = total_acc;
					analysis_re(ana, 3) = total_acc / total_num;
					analysis_re(ana, 4) = avg_acc/3.0;
					analysis_re(ana, 5) = total_rre / total_acc;
					analysis_re(ana, 6) = total_rte / total_acc;
					analysis_re(ana, 7) = total_time / total_num;
					cout << analysis_re << std::endl;

					std::cout << std::endl << std::endl;
					std::cout << std::endl << std::endl;
				}

				//system("pause");
			}
		}


	
		//return;
	}
}


void PointCloudRegistration::Registration_PPF_inlier(std::string dataname, std::string savename, MatrixXi r_pair, double value)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num = value;// 500;
	model_parameter.clutter_veri = 8 + 0.1;//8
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	model_parameter.overlap_vl_angle = 4.0;
	model_parameter.overlap_n = 0.1;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离
	scenematching_parameter.ICP_nbIterations = 10;



	clock_t tick1 = clock();

	if (1)
	{


		//文件句柄
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		string p;
		std::vector<std::string> gtdirname;

		std::vector<int> firstpointcloudname;
		std::vector<int> secondpointcloudname;
		//std::vector<int> firstpointcloudname_est;
		//std::vector<int> secondpointcloudname_est;




		//读位姿
		std::vector<Matrix4d> pointpose;
		//std::vector<Matrix4d> pointposeest;
		std::vector<Vector3i> overlapest;
		std::vector<Vector3f> gt_overlap;
		std::vector<MatrixXd> pc_info;
		//pointpose.resize(1591);

		{
			string gtpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/gt.log";
			string infopath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/gt.info";
			string overlappath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/gt_overlap.log";


			//string gtpath = "E:/pointclouddata/3DLoMatch/" + dataname + "/gt.log";
			//string infopath = "E:/pointclouddata/3DLoMatch/" + dataname + "/gt.info";
			//string overlappath = "E:/pointclouddata/3DLoMatch/" + dataname + "/gt_overlap.log";

			//string gtpath = "F:/pointclouddata/3DLoMatch/"+ dataname +"/gt.log";
			//string overlappath = "F:/pointclouddata/3DLoMatch/" + dataname + "/gt.info";

			pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);
			//pointposeest = readpose(opestpath, firstpointcloudname_est, secondpointcloudname_est);

			ifstream fin;
			fin.open(infopath, ios::in);
			if (!fin.is_open())
			{
				std::cout << infopath << "无法找到这个文件！" << endl;
			}
			else
			{

				while (!fin.eof())
				{
					int i, j, k;

					fin >> i;
					if (fin.eof())
						break;
					fin >> j;
					fin >> k;

					Vector3i a;
					a(0) = i;
					a(1) = j;
					a(2) = k;
					MatrixXd info(6, 6);
					double b;
					for (int v = 0; v < 6; v++)
					{
						for (int c = 0; c < 6; c++)
						{
							fin >> info(v, c);
						}
					}
					if (fabs(i - j) > 1)
					{
						overlapest.push_back(a);
						pc_info.push_back(info);
					}

					//std::cout << k << endl;
					//std::cout << info << endl;
				}

			}
			fin.close();

			ifstream overlapfin;
			overlapfin.open(overlappath, ios::in);
			if (!overlapfin.is_open())
			{
				std::cout << overlappath << "无法找到这个文件！" << endl;
			}
			else
			{
				std::cout << overlappath << endl;

				while (!overlapfin.eof())
				{
					int i, j;

					overlapfin >> i;
					if (overlapfin.eof())
						break;
					overlapfin.get();
					overlapfin >> j;
					overlapfin.get();
					float overlap;
					overlapfin >> overlap;
					Vector3f a;
					a(0) = i;
					a(1) = j;
					a(2) = overlap;
					//std::cout<<i<<" "<<j << " " << overlap << endl;
					gt_overlap.push_back(a);

				}

			}
			overlapfin.close();
			//system("pause");
		}


		bool yn = true;
		ofstream fout;
		fout.open("E:/pointclouddata/3DMatch/" + dataname + "/" + savename + ".txt", ios::out);
		string proestpath = "E:/pointclouddata/3DMatch/" + dataname + "/" + savename + ".log";
		

		std::string pointcloudpath = "E:/pointclouddata/3DMatch/" + dataname + "/" + dataname + "/cloud_bin_";
		std::string feat_match = "E:/pointclouddata/3Dmatch/" + dataname + "/GeoTransformer/";
		std::string featuretype = "fpfh";//_match//fpfh//fcgf

		for (int i = 0; i < pointpose.size(); i++)
		{
			//
			std::cout << i << " " << pointpose.size() << std::endl;
			std::cout << "overlap_angle：" << value << std::endl;
			int i1 = firstpointcloudname[i];
			int i2 = secondpointcloudname[i];
			std::cout << i1 << " " << i2 << std::endl;

			//system("pause");
			int total_fram = 0;
			MatrixXd overlap_info;
			double st_overlap;
			for (int j = 0; j < overlapest.size(); j++)
			{
				//cout << overlapest[j](0)<<" "<< overlapest[j](1) << endl;
				if (fabs(overlapest[j](0) - i1) < 0.1 && fabs(overlapest[j](1) - i2) < 0.1)
				{
					total_fram = overlapest[j](2);
					//cout << total_fram << endl;
					overlap_info = pc_info[j];
					break;
				}
			}

			for (int j = 0; j < gt_overlap.size(); j++)
			{
				//cout << overlapest[j](0)<<" "<< overlapest[j](1) << endl;
				if (fabs(gt_overlap[j](0) - i1) < 0.1 && fabs(gt_overlap[j](1) - i2) < 0.1)
				{
					st_overlap = gt_overlap[j](2);
					break;
				}
			}

			std::vector<Vector3f>;
			if (total_fram == 0)
				continue;
			//cout << total_fram << endl;

			//Matrix4d est_pose = pointposeest[i].inverse();
			Matrix4d gt_pose = pointpose[i].inverse();

			for (int j = 0; j < overlapest.size(); j++)
			{
				if (fabs(firstpointcloudname[j] - i1) < 0.1
					&& fabs(secondpointcloudname[j] - i2) < 0.1)
				{
					gt_pose = pointpose[j].inverse();
					break;
				}
			}

			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				if ((featuretype == "fcgf"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz", fristkeypoint, fristfeature,false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				else if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}

				std::string fcpath, scpath;
				fcpath = pointcloudpath + to_string(firstpointcloudname[i]) + ".ply";
				scpath = pointcloudpath + to_string(secondpointcloudname[i]) + ".ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

				MatrixXf fristpoint, secondpoint;
				fristpoint = loadPLYSimple_bin(fcpath.c_str(), 0);
				secondpoint = loadPLYSimple_bin(scpath.c_str(), 0);


				std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
				fristpoint = StatisticsDenoise(fristpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;

				std::cout << "模型2去噪前" << secondpoint.rows() << std::endl;
				secondpoint = StatisticsDenoise(secondpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型2去噪后" << secondpoint.rows() << std::endl;

				if ((featuretype == "fcgf")|| (featuretype == "fpfh"))//拼接点云计算法向
				{
					MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
					MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
					fristm << fristkeypoint, fristpoint;
					fristpoint = fristm;
					secondm << secondkeypoint, secondpoint;
					secondpoint = secondm;
				}
				std::cout << "计算法向" << std::endl;
				Vector3f centerpoint(0, 0, 0);
				MatrixXf fristpointcloud = normal(fristpoint, centerpoint, 25, 0.08);//30
				MatrixXf secondpointcloud = normal(secondpoint, centerpoint, 25, 0.08);//30
				std::cout << "计算法向结束" << std::endl;
				//writePLY(fristpointcloud, "modelf.ply");
				//writePLY(secondpointcloud, "scenef.ply");
				//system("pause");
				//writePLY(fristpointcloud, (pointcloudsavepath + to_string(firstpointcloudname[i]) + ".ply").c_str());
				//writePLY(secondpointcloud, (pointcloudsavepath + to_string(secondpointcloudname[i]) + ".ply").c_str());
				//writePLY(fristpointcloud, "model_4.ply");
				//writePLY(secondpointcloud, "scene_18.ply");


				//对应点变化




				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);

				Matrix4d pose;
				{
					pose = gt_pose;
					detectorpso.set_groundturth_pose(pose);

				}

				clock_t tick1 = clock();
				{
					//模型下采样
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());

					//计算特征并匹配

					//int n = detectorpso.FeatureMatch_inlier(fristfeature, secondfeature);

					//system("pause");
					//fout << i << " " << firstpointcloudname[i] << " "
					//	<< secondpointcloudname[i] << " " << n << " " << st_overlap << endl;
				}


				//system("pause");

				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;


				//system("pause");
			}
		}

		fout.close();
		return;
	}






}

void PointCloudRegistration::Registration_PPF_Dome(std::string source, std::string target)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num = 500;// 500;
	model_parameter.clutter_veri = 8 + 0.1;//8
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	model_parameter.overlap_vl_angle = 1.0;
	model_parameter.overlap_n = 0.1;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 2.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	clock_t tick1 = clock();

	if (1)
	{


		{
			std::cout << source << std::endl;
			std::cout << target << std::endl;

			std::vector<std::string> first_pc_name = split(source, ".");
			std::vector<std::string> second_pc_name = split(target, ".");

			


			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				//if ((featuretype == "fpfh"))
				{
					std::cout << first_pc_name[0] + "_fpfh.npz" << std::endl;
					if (!readfeature(first_pc_name[0] + "_fpfh.npz", fristkeypoint, fristfeature, true))
					{
						return;
					}
					std::cout <<  second_pc_name[0] + "_fpfh.npz" << std::endl;
					if (!readfeature( second_pc_name[0] + "_fpfh.npz", secondkeypoint, secondfeature, true))
					{
						return;
					}
				}

				


				MatrixXf fristpointcloud = loadPLYSimple_bin(source.c_str(), 1);
				MatrixXf secondpointcloud = loadPLYSimple_bin(target.c_str(), 1);


				{//拼接点云计算法向
					MatrixXf fristm(fristkeypoint.rows() + fristpointcloud.rows(), 6);
					MatrixXf secondm(secondkeypoint.rows() + secondpointcloud.rows(), 6);
					fristm << fristkeypoint, fristpointcloud;
					fristpointcloud = fristm;
					secondm << secondkeypoint, secondpointcloud;
					secondpointcloud = secondm;
				}



			
				PPF_PSO_Match detectorpso(model_parameter);

				clock_t tick1 = clock();
				//if ((featuretype == "fpfh"))//拼接点云计算法向
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配

					//writePLY(detectorpso._R_model_para.scene.downsample_scene, "G:/pointclouddata/wuhan/6-Campus/our_PPF/scenef.ply");

					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
					//system("pause");
				}


				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
			
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();


				Matrix4d prop = results[0].Pose;
				std::cout << prop << std::endl;

				MatrixXf model = transformPCPose(fristpointcloud, prop);
				writePLY(model, (first_pc_name[0]+"SVC.ply").c_str());
			
			}
		}


		
		return;
	}
}

void PointCloudRegistration::Registration_PPF_WUSTL(std::string dataname, std::string savename, MatrixXi r_pair, double value)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(1 + 0.1);//200
	model_parameter.feature_num = value;// 500;
	model_parameter.clutter_veri = 8 + 0.1;//8
	model_parameter.overlap_angle =130;// 130;
	model_parameter.overlap_dis =2.5;// 2.5;
	model_parameter.overlap_vl_angle=3.0;
	model_parameter.overlap_n=0.1;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 3.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	clock_t tick1 = clock();

	if (1)
	{


		//文件句柄
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		string p;
		std::string pointcloudpath = "E:/pointclouddata/wuhan/"+ dataname;
		std::vector<std::string> pointcloudpairname;
		if ((hFile = _findfirst(p.assign(pointcloudpath).append("3-GroundTruth/*").c_str(), &fileinfo)) != -1)
		{
			do
			{
				//如果是目录,迭代之
				//如果不是,加入列表
				//cout<< fileinfo.name<<" " << fileinfo.attrib<<" " << _A_SUBDIR << endl;
				if ((fileinfo.attrib & _A_SUBDIR))
				{
					if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					{
						std::string name = fileinfo.name;
                        cout << name << endl;
						pointcloudpairname.push_back(name);
						
					}
	
				}
			} while (_findnext(hFile, &fileinfo) == 0);

		}
		_findclose(hFile);

		
		std::vector<std::string> gtdirname;
		MatrixXf reslutvalue(pointcloudpairname.size(), 6);
		

		for (int i = 0; i < pointcloudpairname.size(); i++)//
		{

			

			string gtpath = pointcloudpath+"3-GroundTruth/"+ pointcloudpairname[i]+"/transformation.txt";

			std::cout << gtpath << std::endl;
			Matrix4d gt_pose;
			ifstream fin;
			fin.open(gtpath, ios::in);
			if (!fin.is_open())
			{
				std::cout << gtpath << "无法找到这个文件！" << endl;
			}
			else
			{
				for (int v = 0; v < 4; v++)
				{
					for (int c = 0; c < 4; c++)
					{
						fin >> gt_pose(v, c);
					}
				}

			}
			fin.close();
		


			bool yn = true;
			ofstream fout;

			fout.open(pointcloudpath + "our_PPF/" + pointcloudpairname[i] + ".txt", ios::out);
			string proestpath = pointcloudpath + "our_PPF/ " + pointcloudpairname[i] + ".log";


			ofstream posefout;
			posefout.open(proestpath, ios::out);

			std::string pointcloudpathname = pointcloudpath + "pointcloudnormal/";

			
			std::vector<std::string> pointfilename = split(pointcloudpairname[i], "-");
			if (pointfilename.size() != 2)
			{
				std::cout << pointcloudpairname[i] << std::endl;
				continue;
			}
			std::cout << pointcloudpairname[i] << std::endl;
			if ((pointcloudpairname[i] != "6-5" && pointcloudpairname[i] != "8-6"))//
			{
				//continue;
			}

			

			std::string first_pc_name = std::string(2 - pointfilename[0].length(), '0') + pointfilename[0];
			std::string second_pc_name = std::string(2 - pointfilename[1].length(), '0') + pointfilename[1];

			std::cout << first_pc_name << std::endl;
			std::cout << second_pc_name << std::endl;
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				//if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpathname + first_pc_name + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpathname + first_pc_name + "_fpfh.npz", fristkeypoint, fristfeature,true))
					{
						continue;
					}
					std::cout << pointcloudpathname + second_pc_name + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpathname + second_pc_name + "_fpfh.npz", secondkeypoint, secondfeature, true))
					{
						continue;
					}
				}

				std::string fcpath, scpath;
				fcpath = pointcloudpathname + first_pc_name + ".ply";
				scpath = pointcloudpathname + second_pc_name + ".ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

		
				MatrixXf fristpointcloud = loadPLYSimple_bin(fcpath.c_str(), 1);
				MatrixXf secondpointcloud = loadPLYSimple_bin(scpath.c_str(), 1);

				
				{//拼接点云计算法向
					MatrixXf fristm(fristkeypoint.rows() + fristpointcloud.rows(), 6);
					MatrixXf secondm(secondkeypoint.rows() + secondpointcloud.rows(), 6);
					fristm << fristkeypoint, fristpointcloud;
					fristpointcloud = fristm;
					secondm << secondkeypoint, secondpointcloud;
					secondpointcloud = secondm;
				}


				
				//system("pause");
				//writePLY(fristpointcloud, (pointcloudsavepath + to_string(firstpointcloudname[i]) + ".ply").c_str());
				//writePLY(secondpointcloud, (pointcloudsavepath + to_string(secondpointcloudname[i]) + ".ply").c_str());



				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);

				
				{
					detectorpso.set_groundturth_pose(gt_pose.inverse());
				}

				clock_t tick1 = clock();
				//if ((featuretype == "fpfh"))//拼接点云计算法向
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					
					//writePLY(detectorpso._R_model_para.model.downsample_model, "G:/pointclouddata/wuhan/6-Campus/our_PPF/modelf.ply");
					//writePLY(detectorpso._R_model_para.scene.downsample_scene, "G:/pointclouddata/wuhan/6-Campus/our_PPF/scenef.ply");

					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
					//system("pause");
				}
				
				
				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//detectorpso.match_PPF_registration_ablation(results, inst_count, record, scenematching_parameter);
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				


				Matrix4d prop = results[0].Pose;
				for (int p1 = 0; p1 < 4; p1++)
				{
					for (int p2 = 0; p2 < 3; p2++)
					{
						posefout << prop(p1, p2) << "\t";
					}
					posefout << prop(p1, 3) << endl;
				}






				{

					fout << i << "  ";
					fout << first_pc_name << "  ";
					fout << second_pc_name << "  ";


					fout << (double)(tick2 - tick1) / CLOCKS_PER_SEC << "  " << (double)(tick3 - tick1) / CLOCKS_PER_SEC << "  ";
					for (int id = 0; id < record.size(); id++)
					{
						fout << record[id] << "  ";
					}


					{
						Matrix4d dp = prop.block(0,0,3,3).transpose() * gt_pose.block(0, 0, 3, 3);
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1)    d = -1;
						record.push_back(acos(d));

						double dis = (gt_pose.block(0, 3, 3, 1) - prop.block(0, 3, 3, 1)).norm();
						record.push_back(dis);

						std::cout << "est_pose da=" << acos(d)/ M_PI * 180 << endl;
						std::cout << "est_pose dt=" << dis << endl;
						if (acos(d) / M_PI * 180 < 5 && dis < 2.0)
							reslutvalue(i, 0) = 1;
						else
							reslutvalue(i, 0) = 0;

						reslutvalue(i, 1) = acos(d) / M_PI * 180;
						reslutvalue(i, 2) = dis;
						reslutvalue(i, 3) = (double)(tick3 - tick1) / CLOCKS_PER_SEC;

						reslutvalue(i, 4) = atoi(pointfilename[0].c_str());
						reslutvalue(i, 5) = atoi(pointfilename[1].c_str());
					}

					fout << std::endl;

					std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
					std::cout << "est_pose " << std::endl << prop << std::endl;
					std::cout << "  registration in "
						<< (double)(tick3 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;
					std::cout << std::endl << std::endl;
					std::cout << std::endl << std::endl;
					

				}
				fout.close();
		        posefout.close();
				//system("pause");
			}
		}

		
		cout << reslutvalue << endl;

		ofstream fout;
		cout << "E:/pointclouddata/wuhan/ab/" + dataname.substr(0,dataname.length()-1) + "_" + savename + ".txt" << endl;
		fout.open( "E:/pointclouddata/wuhan/ab/" +dataname.substr(0,dataname.length() - 1) +"_" + savename + ".txt", ios::out);
		fout << reslutvalue << endl;
		fout.close();
		return;
	}


}



void PointCloudRegistration::Registration_PPF_3DLoMatch(std::string dataname, std::string savename, MatrixXi r_pair, double value)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num = value;// 500;
	model_parameter.clutter_veri = 8 + 0.1;//8
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	clock_t tick1 = clock();

	if (1)
	{


		//文件句柄
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		string p;
		std::vector<std::string> gtdirname;

		std::vector<int> firstpointcloudname;
		std::vector<int> secondpointcloudname;
		//std::vector<int> firstpointcloudname_est;
		//std::vector<int> secondpointcloudname_est;




		//读位姿
		std::vector<Matrix4d> pointpose;
		//std::vector<Matrix4d> pointposeest;
		std::vector<Vector3i> overlapest;
		std::vector<MatrixXd> pc_info;
		//pointpose.resize(1591);

		{


			string gtpath = "E:/pointclouddata/3DLoMatch/"+ dataname +"/gt.log";
			string overlappath = "E:/pointclouddata/3DLoMatch/" + dataname + "/gt.info";

			pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);
			//pointposeest = readpose(opestpath, firstpointcloudname_est, secondpointcloudname_est);

			ifstream fin;
			fin.open(overlappath, ios::in);
			if (!fin.is_open())
			{
				std::cout << overlappath << "无法找到这个文件！" << endl;
			}
			else
			{

				while (!fin.eof())
				{
					int i, j, k;

					fin >> i;
					if (fin.eof())
						break;
					fin >> j;
					fin >> k;

					Vector3i a;
					a(0) = i;
					a(1) = j;
					a(2) = k;
					MatrixXd info(6, 6);
					double b;
					for (int v = 0; v < 6; v++)
					{
						for (int c = 0; c < 6; c++)
						{
							fin >> info(v, c);
						}
					}
					if (fabs(i - j) > 1)
					{
						overlapest.push_back(a);
						pc_info.push_back(info);
					}

					//std::cout << k << endl;
					//std::cout << info << endl;
				}

			}
			fin.close();
		}


		bool yn = true;
		ofstream fout;

		fout.open("E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".txt", ios::out);
		string proestpath = "E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".log";


		ofstream posefout;
		posefout.open(proestpath, ios::out);
		std::string pointcloudpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "/cloud_bin_";
		std::string feat_match = "E:/pointclouddata/3Dmatch/" + dataname + "/GeoTransformer/";
		std::string featuretype = "overlap_match";//geo_match//fpfh//fcgf//geo_match

		for (int i = 0; i < pointpose.size(); i++)
		{
			std::cout << i << " " << pointpose.size() << std::endl;
			std::cout << "overlap_angle：" << value << std::endl;
			int i1 = firstpointcloudname[i];
			int i2 = secondpointcloudname[i];
			std::cout << i1 << " " << i2 << std::endl;
			if (0)
			{
				bool yn = true;
				int row = r_pair.rows();
				//std::cout << r_pair << std::endl;
				for (int r = 0; r < row; r++)
				{
					if (r_pair(r, 0) == i1 && r_pair(r, 1) == i2)
					{
						yn = false;
						break;
					}

				}
				if (yn)
					continue;
			}
			int total_fram = 0;
			MatrixXd overlap_info;
			for (int j = 0; j < overlapest.size(); j++)
			{
				//cout << overlapest[j](0)<<" "<< overlapest[j](1) << endl;
				if (fabs(overlapest[j](0) - i1) < 0.1 && fabs(overlapest[j](1) - i2) < 0.1)
				{
					total_fram = overlapest[j](2);
					//cout << total_fram << endl;
					overlap_info = pc_info[j];
					break;
				}
			}
			if (total_fram == 0)
				continue;
			//cout << total_fram << endl;

			//Matrix4d est_pose = pointposeest[i].inverse();
			Matrix4d gt_pose = pointpose[i];

			for (int j = 0; j < overlapest.size(); j++)
			{
				if (fabs(firstpointcloudname[j] - i1) < 0.1
					&& fabs(secondpointcloudname[j] - i2) < 0.1)
				{
					gt_pose = pointpose[j];
					break;
				}
			}
			//if (yn)
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				if ((featuretype == "fcgf"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz", fristkeypoint, fristfeature,false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				else if ((featuretype == "geo_match"))
				{
					std::string path = feat_match + to_string(firstpointcloudname[i]) + "_"
						+ to_string(secondpointcloudname[i]) + ".npz_corr_lo.txt";
					std::cout << path << std::endl;
					readfeat_match(path, fristkeypoint, secondkeypoint, corr_scores);
				}
				else if ((featuretype == "overlap_match"))
				{
					std::string path = feat_match + to_string(firstpointcloudname[i]) + "_"
						+ to_string(secondpointcloudname[i]) + ".pth.npz_corr_lo.txt";
					std::cout << path << std::endl;
					
					readfeat_stc_tgt(path, fristkeypoint, secondkeypoint, fristfeature, secondfeature);
				}
				else if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}

				std::string fcpath, scpath;
				fcpath = pointcloudpath + to_string(firstpointcloudname[i]) + ".ply";
				scpath = pointcloudpath + to_string(secondpointcloudname[i]) + ".ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

				MatrixXf fristpoint, secondpoint;
				fristpoint = loadPLYSimple_bin(fcpath.c_str(), 0);
				secondpoint = loadPLYSimple_bin(scpath.c_str(), 0);


				std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
				fristpoint = StatisticsDenoise(fristpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;

				std::cout << "模型2去噪前" << secondpoint.rows() << std::endl;
				secondpoint = StatisticsDenoise(secondpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型2去噪后" << secondpoint.rows() << std::endl;
				//cout << fristkeypoint.rows() << " " << fristpoint.rows() << endl;
				if ((featuretype == "fcgf")
					|| (featuretype == "geo_match") || (featuretype == "overlap_match")
					|| (featuretype == "GeoTransformer") || (featuretype == "fpfh"))//拼接点云计算法向
				{
					MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
					MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
					fristm << fristkeypoint, fristpoint;
					fristpoint = fristm;
					secondm << secondkeypoint, secondpoint;
					secondpoint = secondm;
				}
				//cout << fristkeypoint.rows() << " " << fristpoint.rows() << endl;
				//writePLY(fristkeypoint, "E:/modelf1.ply");
				//writePLY(secondkeypoint, "E:/scenef1.ply");
				std::cout << "计算法向" << std::endl;
				Vector3f centerpoint(0, 0, 0);
				MatrixXf fristpointcloud = normal(fristpoint, centerpoint, 25, 0.08);//30
				MatrixXf secondpointcloud = normal(secondpoint, centerpoint, 25, 0.08);//30
				//std::cout << "计算法向结束" << std::endl;
				//writePLY(fristpointcloud, "E:/modelf.ply");
				//writePLY(secondpointcloud, "E:/scenef.ply");
				//cout << fristpointcloud.rows() << " " << fristpointcloud.rows() << endl;
				//system("pause");
				//writePLY(fristpointcloud, (pointcloudsavepath + to_string(firstpointcloudname[i]) + ".ply").c_str());
				//writePLY(secondpointcloud, (pointcloudsavepath + to_string(secondpointcloudname[i]) + ".ply").c_str());



				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);

				Matrix4d pose;
				{


					pose = gt_pose;
					//std::cout << pose << endl;
					//std::cout << "groundturth:" << endl;
					//std::cout << pose << endl;
					//std::cout << pose << endl;
					detectorpso.set_groundturth_pose(pose);
					//std::cout << "groundturth:" << endl;
					//std::cout << pointpose[i + 1]*pointpose[i].inverse()  << endl;
					//std::cout << (pointpose[i + 1]*pointpose[i].inverse()).inverse() << endl;

					//MatrixXf model = transformPCPose(fristpointcloud, pose);
					//overlap = calvoerlap(model.block(0,0, model.rows(),3), secondpointcloud.block(0, 0, secondpointcloud.rows(), 3), 0.03);

					//std::cout << "fristpointcloud rows=" << fristpointcloud.rows() << endl;
					//std::cout << "secondpointcloud rows=" << secondpointcloud.rows() << endl;

				}

				clock_t tick1 = clock();
				if ((featuretype == "fcgf") || (featuretype == "GeoTransformer") || (featuretype == "fpfh") || (featuretype == "overlap_match"))//拼接点云计算法向
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配

					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
					//system("pause");
				}
				else if ((featuretype == "geo_match"))
				{

					//模型下采样
					detectorpso.Modeldown(fristpointcloud, fristkeypoint.rows());
					detectorpso.Scenedown(secondpointcloud, secondkeypoint.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					detectorpso.FeatureMatch_match(corr_scores);
				}
				else
				{
					//模型下采样
					//detectorpso.Modeldown(fristpointcloud,0);
					//detectorpso.Scenedown(secondpointcloud,0);
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					//detectorpso.FeatureMatch();
				}

				//system("pause");



				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				std::cout << i1 << " " << i2 << std::endl;


				posefout << firstpointcloudname[i] << "\t";
				posefout << secondpointcloudname[i] << "\t";
				posefout << total_fram << endl;
				Matrix4d prop = results[0].Pose.inverse();
				Matrix4d est_pose = prop;
				for (int p1 = 0; p1 < 4; p1++)
				{
					for (int p2 = 0; p2 < 3; p2++)
					{
						posefout << prop(p1, p2) << "\t";
					}
					posefout << prop(p1, 3) << endl;
				}




				{

					fout << i << "  ";
					fout << firstpointcloudname[i] << "  ";
					fout << secondpointcloudname[i] << "  ";


					fout << (double)(tick2 - tick1) / CLOCKS_PER_SEC << "  " << (double)(tick3 - tick1) / CLOCKS_PER_SEC << "  ";
					for (int id = 0; id < record.size(); id++)
					{
						fout << record[id] << "  ";
					}


					{
						Matrix3d dp = est_pose.block(0, 0, 3, 3).transpose() * (gt_pose).block(0, 0, 3, 3);
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1)    d = -1;
						record.push_back(acos(d));

						double dis = (gt_pose.block(0, 3, 3, 1) - est_pose.block(0, 3, 3, 1)).norm();
						record.push_back(dis);

						std::cout << "est_pose da=" << acos(d) / M_PI * 180 << endl;
						std::cout << "est_pose dt=" << dis << endl;

					}

					fout << std::endl;

					std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
					std::cout << "est_pose " << std::endl << est_pose << std::endl;
					std::cout << "  registration in "
						<< (double)(tick3 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;
					std::cout << std::endl << std::endl;
					std::cout << std::endl << std::endl;
					/*fout << d << "  ";
					if (d <= 0.04)
					{
						fout<< 1 <<"  ";
					}
					else
					{
						fout << 0 <<"  ";
						//system("pause");
					}

					fout << d1 << "  ";
					if (d1 <= 0.04)
					{
						fout << 1 << std::endl;
					}
					else
					{
						fout << 0 << std::endl;
					}*/

				}

				//system("pause");
			}
		}

		fout.close();
		posefout.close();
		return;
	}






}


void PointCloudRegistration::Registration_PPF(std::string dataname, std::string savename, std::vector<double>& value)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num = 500;// 500;
	model_parameter.clutter_veri = 7 + 0.1;//10
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	model_parameter.overlap_vl_angle = 2.0;
	model_parameter.overlap_n = 2.0;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	clock_t tick1 = clock();

	std::vector<std::string> data_name = { "redkitchen",
	   "home_at", "home_md", "hotel_uc", "hotel_1", "hotel3", "study","lab" };
	std::vector< std::vector<double>> res;

	for (int ds = 0; ds < data_name.size(); ds++)
	{

		dataname = data_name[ds];

		//文件句柄
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		string p;
		std::vector<std::string> gtdirname;

		std::vector<int> firstpointcloudname;
		std::vector<int> secondpointcloudname;
		//std::vector<int> firstpointcloudname_est;
		//std::vector<int> secondpointcloudname_est;




		//读位姿
		std::vector<Matrix4d> pointpose;
		//std::vector<Matrix4d> pointposeest;
		std::vector<Vector3i> overlapest;
		std::vector<MatrixXd> pc_info;
		//pointpose.resize(1591);

		{
			string gtpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/gt.log";
			string overlappath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/gt.info";

			//string gtpath = "E:/pointclouddata/3DLoMatch/"+ dataname +"/gt.log";
			//string overlappath = "e:/pointclouddata/3DLoMatch/" + dataname + "/gt.info";

			pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);
			//pointposeest = readpose(opestpath, firstpointcloudname_est, secondpointcloudname_est);

			ifstream fin;
			fin.open(overlappath, ios::in);
			if (!fin.is_open())
			{
				std::cout << overlappath << "无法找到这个文件！" << endl;
			}
			else
			{

				while (!fin.eof())
				{
					int i, j, k;

					fin >> i;
					if (fin.eof())
						break;
					fin >> j;
					fin >> k;

					Vector3i a;
					a(0) = i;
					a(1) = j;
					a(2) = k;
					MatrixXd info(6, 6);
					double b;
					for (int v = 0; v < 6; v++)
					{
						for (int c = 0; c < 6; c++)
						{
							fin >> info(v, c);
						}
					}
					if (fabs(i - j) > 1)
					{
						overlapest.push_back(a);
						pc_info.push_back(info);
					}

					//std::cout << k << endl;
					//std::cout << info << endl;
				}

			}
			fin.close();
		}


		bool yn = true;
		ofstream fout;
		fout.open("E:/pointclouddata/3Dmatch/" + dataname + "/" + savename + ".txt", ios::out);
		string proestpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/" + savename + ".log";

		//fout.open("E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".txt", ios::out);
		//string proestpath = "E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".log";


		ofstream posefout;
		posefout.open(proestpath, ios::out);
		std::string pointcloudpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "/cloud_bin_";
		std::string feat_match = "E:/pointclouddata/3Dmatch/" + dataname + "/GeoTransformer/";
		std::string featuretype = "fcgf";//geo_match//fpfh//fcgf

		int num = 0;
		int acc = 0;
		double total_time = 0;
		double te = 0;
		double re = 0;
	
		value.clear();
		value.resize(5);

		for (int i = 0; i < pointpose.size(); i++)
		{
			std::cout << i << " " << pointpose.size() << std::endl;

			//std::cout << "overlap_angle：" << value << std::endl;
			int i1 = firstpointcloudname[i];
			int i2 = secondpointcloudname[i];
			std::cout << i1 << " " << i2 << std::endl;

			int total_fram = 0;
			MatrixXd overlap_info;
			for (int j = 0; j < overlapest.size(); j++)
			{
				//cout << overlapest[j](0)<<" "<< overlapest[j](1) << endl;
				if (fabs(overlapest[j](0) - i1) < 0.1 && fabs(overlapest[j](1) - i2) < 0.1)
				{
					total_fram = overlapest[j](2);
					//cout << total_fram << endl;
					overlap_info = pc_info[j];
					break;
				}
			}
			if (total_fram == 0)
				continue;
			//cout << total_fram << endl;

			//Matrix4d est_pose = pointposeest[i].inverse();
			Matrix4d gt_pose = pointpose[i];

			for (int j = 0; j < overlapest.size(); j++)
			{
				if (fabs(firstpointcloudname[j] - i1) < 0.1
					&& fabs(secondpointcloudname[j] - i2) < 0.1)
				{
					gt_pose = pointpose[j];
					break;
				}
			}
			//if (yn)
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				if ((featuretype == "fcgf"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fcgf.npz", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fcgf.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				else if ((featuretype == "geo_match"))
				{
					std::string path = feat_match + to_string(firstpointcloudname[i]) + "_"
						+ to_string(secondpointcloudname[i]) + ".npz_corr.txt";
					std::cout << path << std::endl;
					readfeat_match(path, fristkeypoint, secondkeypoint, corr_scores);
				}
				else if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(firstpointcloudname[i]) + "_fpfh.npz", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz" << std::endl;
					if (!readfeature(pointcloudpath + to_string(secondpointcloudname[i]) + "_fpfh.npz", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}

				std::string fcpath, scpath;
				fcpath = pointcloudpath + to_string(firstpointcloudname[i]) + ".ply";
				scpath = pointcloudpath + to_string(secondpointcloudname[i]) + ".ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

				MatrixXf fristpoint, secondpoint;
				fristpoint = loadPLYSimple_bin(fcpath.c_str(), 0);
				secondpoint = loadPLYSimple_bin(scpath.c_str(), 0);


				std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
				fristpoint = StatisticsDenoise(fristpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;

				std::cout << "模型2去噪前" << secondpoint.rows() << std::endl;
				secondpoint = StatisticsDenoise(secondpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型2去噪后" << secondpoint.rows() << std::endl;

				if ((featuretype == "fcgf")
					|| (featuretype == "geo_match")
					|| (featuretype == "GeoTransformer") || (featuretype == "fpfh"))//拼接点云计算法向
				{
					MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
					MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
					fristm << fristkeypoint, fristpoint;
					fristpoint = fristm;
					secondm << secondkeypoint, secondpoint;
					secondpoint = secondm;
				}
				std::cout << "计算法向" << std::endl;
				Vector3f centerpoint(0, 0, 0);
				MatrixXf fristpointcloud = normal(fristpoint, centerpoint, 25, 0.12);//30
				MatrixXf secondpointcloud = normal(secondpoint, centerpoint, 25, 0.12);//30
				std::cout << "计算法向结束" << std::endl;
				//writePLY(fristpointcloud, "modelf.ply");
				//writePLY(secondpointcloud, "scenef.ply");
				//system("pause");
				//writePLY(fristpointcloud, (pointcloudsavepath + to_string(firstpointcloudname[i]) + ".ply").c_str());
				//writePLY(secondpointcloud, (pointcloudsavepath + to_string(secondpointcloudname[i]) + ".ply").c_str());



				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);

				Matrix4d pose;
				{


					detectorpso.set_groundturth_pose(gt_pose);
				

				}

				clock_t tick1 = clock();
				if ((featuretype == "fcgf") || (featuretype == "fpfh"))//拼接点云计算法向
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配

					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
					//system("pause");
				}
				else if ((featuretype == "geo_match"))
				{

					//模型下采样
					detectorpso.Modeldown(fristpointcloud, fristkeypoint.rows());
					detectorpso.Scenedown(secondpointcloud, secondkeypoint.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					detectorpso.FeatureMatch_match(corr_scores);
				}

				//system("pause");



				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
				//detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//detectorpso.match_ransac_registration(results, inst_count, record, scenematching_parameter);
				//system("pause");
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				std::cout << i1 << " " << i2 << std::endl;

				total_time += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
				num++;

				posefout << firstpointcloudname[i] << "\t";
				posefout << secondpointcloudname[i] << "\t";
				posefout << total_fram << endl;
				Matrix4d prop = results[0].Pose.inverse();
				Matrix4d est_pose = prop;
				for (int p1 = 0; p1 < 4; p1++)
				{
					for (int p2 = 0; p2 < 3; p2++)
					{
						posefout << prop(p1, p2) << "\t";
					}
					posefout << prop(p1, 3) << endl;
				}



				{

					fout << i << "  ";
					fout << firstpointcloudname[i] << "  ";
					fout << secondpointcloudname[i] << "  ";


					fout << (double)(tick2 - tick1) / CLOCKS_PER_SEC << "  " << (double)(tick3 - tick1) / CLOCKS_PER_SEC << "  ";
					for (int id = 0; id < record.size(); id++)
					{
						fout << record[id] << "  ";
					}


					{
						Matrix3d dp = est_pose.block(0, 0, 3, 3).transpose() * (gt_pose).block(0, 0, 3, 3);
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1)    d = -1;
						record.push_back(acos(d));

						double dis = (gt_pose.block(0, 3, 3, 1) - est_pose.block(0, 3, 3, 1)).norm();
						record.push_back(dis);

						std::cout << "est_pose da=" << acos(d) / M_PI * 180 << endl;
						std::cout << "est_pose dt=" << dis << endl;

						if (acos(d) / M_PI * 180 < 15 && dis < 0.3)
						{
							acc++;
							te += dis * 100.00;
							re += acos(d) / M_PI * 180;

							value[1] = acc;
							value[2] = te / acc;
							value[3] = re / acc;
						}
						value[4] = total_time / num;
						value[0] = num;
						//fout << std::endl;

						//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
						//std::cout << "est_pose " << std::endl << est_pose << std::endl;
						for (int v = 0; v < 5; v++)
							std::cout << value[v] << " ";
						std::cout << value[1] * 1.0 / value[0] << std::endl;

						double total_num = 0;
						double total_acc = 0;
						double total_te = 0;
						double total_re = 0;
						double total_avg = 0;
						int size = res.size();
						std::cout << size << std::endl;
						for (int d = 0; d < size; d++)
						{
							for (int v = 0; v < 5; v++)
								std::cout << res[d][v] << " ";
							std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
							total_num += res[d][0];
							total_acc += res[d][1];
							total_avg += res[d][1] * 1.0 / res[d][0];
							total_te += res[d][2] * res[d][1];
							total_re += res[d][3] * res[d][1];
						}
						std::cout << "total num " << total_num << " "
							<< "total acc " << total_acc << " "
							<< "total avg " << total_acc / total_num << " "
							<< "total TE " << total_te / total_acc << " "
							<< "total RE " << total_re / total_acc << " "
							<< "scene avg " << total_avg / data_name.size() << " ";
					}

					

					fout << std::endl;

					//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
					//std::cout << "est_pose " << std::endl << est_pose << std::endl;
					std::cout << "  registration in "
						<< (double)(tick3 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;
					std::cout << std::endl << std::endl;
					std::cout << std::endl << std::endl;
					/*fout << d << "  ";
					if (d <= 0.04)
					{
						fout<< 1 <<"  ";
					}
					else
					{
						fout << 0 <<"  ";
						//system("pause");
					}

					fout << d1 << "  ";
					if (d1 <= 0.04)
					{
						fout << 1 << std::endl;
					}
					else
					{
						fout << 0 << std::endl;
					}*/

				}

				//system("pause");
			}
		}
		res.push_back(value);
		double total_num = 0;
		double total_acc = 0;
		double total_te = 0;
		double total_re = 0;
		double total_avg = 0;
		int size = res.size();
		std::cout << size << std::endl;
		for (int d = 0; d < size; d++)
		{
			for (int v = 0; v < 5; v++)
				std::cout << res[d][v] << " ";
			std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
			total_num += res[d][0];
			total_acc += res[d][1];
			total_avg += res[d][1] * 1.0 / res[d][0];
			total_te += res[d][2] * res[d][1];
			total_re += res[d][3] * res[d][1];
		}
		std::cout << "total num " << total_num << " "
			<< "total acc " << total_acc << " "
			<< "total avg " << total_acc / total_num << " "
			<< "total TE " << total_te / total_acc << " "
			<< "total RE " << total_re / total_acc << " "
			<< "scene avg " << total_avg / data_name.size() << " ";

		fout.close();
		posefout.close();
		
	}






}
void PointCloudRegistration::Registration_DP_KITTI()
{
	


	//文件句柄
	intptr_t hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> pointcloudpairname;


	std::string pointcloudpath = "E:/pointclouddata/data_odometry_velodyne/icp";

	if ((hFile = _findfirst(p.assign(pointcloudpath).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			//cout<< fileinfo.name<<" " << fileinfo.attrib<<" " << _A_SUBDIR << endl;
			if ((fileinfo.attrib & _A_ARCH))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					std::string name = fileinfo.name;
					std::vector<std::string> filename = split(name, ".");
					//cout << name << endl;
					if (filename.size() > 1 && filename[filename.size() - 1] == "npy")
					{
						pointcloudpairname.push_back(name);
						//cout << name << endl;
						//firstpointcloudname.push_back(atoi(filename[0].c_str()));
						//secondpointcloudname.push_back(atoi(filename[1].c_str()));
					}
					else
					{
						//std::cout << " file error " << std::endl;
					}
				}
				//getFiles(p.assign(path).append("\\").append(fileinfo.name), files);*/
			}
		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);

	/*para.Corres_Sample_num = 200;//250
	para.Corres_IR_num = 1000;// 1500
	para.Sample_Iter = 500000;//30000
	para.dis_consistence_thre2 = 0.06;//0.06
	para.dis_min_thre = 1.0;// 5.0
	//para.angle_thre = 10.0; //10.0

	para.vote_angle_discrete = 5.0;//9.0 ?
	para.vote_posenum_max = 120.0;//200
	para.Clusteringdistance = 3.0;//3.0
	para.ClusterNum = 75;//100
	para.ClusterNum2 = 10;//
	para.VoteMaxtoMin = 4.0;//10.0
	para.very_num = 5;//10
	para.very_num1 = 5;
	para.very_num2 = 5;
	//para.IR_dis_min=0.08;//0.08
	para.Overlap_dis_min1 = 2.0;//2.0
	para.Overlap_dis_min2 = 1.5;//1.5
	para.Overlap_weight = 0.7;//0.7 da0.4

	//para.feat_point_num = 2500;

	para.very_index = 3.5;//3.5
	para.ViewpointOverlap_dis = 1.5;//1.5
	para.Projected_grid_sphere = 80;//90*/

	PD_model_parameter para;
	
	/*para.Corres_Sample_num = 150;//250
	para.Corres_IR_num = 600;// 1500
	para.Sample_Iter = 500000;//30000
	para.dis_consistence_thre2 = 0.06;//0.06
	para.dis_min_thre = 0.5;// 5.0
	//para.angle_thre = 10.0; //10.0

	para.vote_angle_discrete = 10.0;//9.0 ?
	para.vote_posenum_max = 100.0;//200
	para.Clusteringdistance = 2.5;//3.0
	para.ClusterNum = 35;//100
	para.ClusterNum2 = 10;//10
	para.VoteMaxtoMin = 4.0;//10.0
	para.very_num = 5;//10
	para.very_num1 = 5;
	para.very_num2 = 5;
	//para.IR_dis_min=0.08;//0.08
	para.Overlap_dis_min1 = 2.5;//2.0
	para.Overlap_dis_min2 = 1.5;//1.5
	para.Overlap_weight = 0.8;//0.7 da0.4

	//para.feat_point_num = 2500;

	para.very_index = 2.5;//3.5
	para.ViewpointOverlap_dis = 2.0;//1.5
	para.Projected_grid_sphere = 40;//90*/


	para.Corres_Sample_num = 200;//250
	para.Corres_IR_num = 1500;// 1500
	para.Sample_Iter = 500000;//30000
	para.dis_consistence_thre2 = 0.05;//0.06
	para.dis_min_thre = 1.5;// 5.0
	//para.angle_thre = 10.0; //10.0

	para.vote_angle_discrete = 5.0;//9.0 ?
	para.vote_posenum_max = 150.0;//200
	para.Clusteringdistance = 0.25;//3.0
	para.ClusterNum = 100;//100
	para.ClusterNum2 = 20;//
	para.VoteMaxtoMin = 5.0;//10.0
	para.very_num = 10;//10

	para.icp_num = 5;
	para.icp_pointnum = 1000;
	//para.IR_dis_min=0.08;//0.08
	para.Overlap_dis_min1 = 2.0;//2.0
	para.Overlap_dis_min2 = 1.0;//1.5
	para.Overlap_weight = 0.7;//0.7 da0.4
	para.Overlap_dis_fine = 0.25;//0.55
	//para.feat_point_num = 2500;

	para.very_index = 3.5;//3.5
	para.ViewpointOverlap_dis = 1.5;//1.5
	para.Projected_grid_sphere = 80;

	para.normal_nei = 10;
	para.angle_con = cos(30.0 / 180.0 * M_PI);
	para.angle_overlap_con = cos(15.0 / 180.0 * M_PI);

	
	

	std::vector<MatrixXf> fpc(556);
	std::vector<MatrixXf> spc(556);
	std::vector<MatrixXf> ffeat(556);
	std::vector<MatrixXf> sfeat(556);
	for (int i = 0; i < 556; i++)
	{
		fpc[i].resize(10, 10);
		spc[i].resize(10, 10);
		ffeat[i].resize(10, 10);
		sfeat[i].resize(10, 10);
	}
	int ank = 3;
	MatrixXf distributoin_angle_re = MatrixXf::Zero(ank, 21);
	MatrixXf distributoin_dis_re = MatrixXf::Zero(ank, 21);
	MatrixXf angle_re = MatrixXf::Zero(ank, 21);
	MatrixXf dis_re = MatrixXf::Zero(ank, 21);
	MatrixXf analysis_re = MatrixXf::Zero(ank, 8);

	int re_size = 3;
	for (int kkk = 0; kkk <1; kkk++)	
	{
		para.icp_pointnum =1000-kkk*500;
		for (int ana = 0; ana < re_size; ana++)
		{
			para.Overlap_dis_min2 =1.0 - ana * 0.2;
			//if(para.vote_posenum_max < para.ClusterNum)
			//	continue;
			analysis_re(kkk* re_size +ana, 0) = para.Overlap_dis_min2 *10.0 + para.icp_pointnum *10;
			distributoin_angle_re(kkk * re_size + ana, 0) = analysis_re(kkk * re_size + ana, 0);
			distributoin_dis_re(kkk * re_size + ana, 0) = analysis_re(kkk * re_size + ana, 0);

		

			bool yn = true;
			std::vector<std::vector<string>> re_08;
			std::vector<std::vector<string>> re_09;
			std::vector<std::vector<string>> re_10;

			std::vector<std::vector<double>> res;
			res.resize(3);
			for (int i = 0; i < 3; i++)
			{
				res[i].resize(5);
				for (int j = 0; j < 3; j++)
				{
					res[i][j] = 0;
				}
			}

			for (int i = 0; i < pointcloudpairname.size(); i++)//
			{


				std::vector<std::string> filename = split(pointcloudpairname[i], ".");
				std::vector<std::string> pointfilename = split(filename[0], "_");
				if (pointfilename.size() != 3)
				{
					std::cout << filename[0] << std::endl;
					continue;
				}

				/*if ((filename[0] != "8_2934_2949"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
				{
					continue;
				}*/

				std::cout << filename[0] << std::endl;

				std::string sequence_name = std::string(2 - pointfilename[0].length(), '0') + pointfilename[0];
				std::string first_pc_name = std::string(6 - pointfilename[1].length(), '0') + pointfilename[1];
				std::string second_pc_name = std::string(6 - pointfilename[2].length(), '0') + pointfilename[2];

				/*if ((sequence_name == "10"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
				{
					continue;
				}
				//8_3419_3446
				if ((filename[0] != "8_3419_3446" &&  sequence_name != "09"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
				{
					continue;
				}
				if ((filename[0] != "8_3419_3446" && filename[0] != "9_1210_1217"))//9_65_74 9_75_84 8_3347_3357 8_3113_3125 8_2934_2949 8_2659_2670
				{
					continue;
				}*/

				//read pose
				ifstream fin;
				MatrixXd gt_pose(4, 4);
				fin.open(pointcloudpath + "/" + pointcloudpairname[i], ios::in);
				if (!fin.is_open())
				{
					std::cout << pointcloudpath + "/" + pointcloudpairname[i] << "无法找到这个文件！" << endl;
				}
				else
				{
					for (int v = 0; v < 4; v++)
					{
						for (int c = 0; c < 4; c++)
						{
							fin >> gt_pose(v, c);
						}
					}

				}


				fin.close();


				std::string featuretype = "fpfh";
				std::string pointcloudpath = "E:/pointclouddata/data_odometry_velodyne/sequences/" + sequence_name + "/pointcloud/";

				{
					MatrixXf fristkeypoint, secondkeypoint;
					MatrixXf fristfeature, secondfeature;
					VectorXf corr_scores;

					if (spc[i].rows() < 100)
					{

						if ((featuretype == "fcgf"))
						{
							std::string featpath = "E:/pointclouddata/data_odometry_velodyne/feat_fcgf_txt/" + filename[0] + ".npz.txt";
							std::cout << featpath << std::endl;
							if (!readfeat_stc_tgt_kitti(featpath, fristkeypoint, secondkeypoint, fristfeature, secondfeature))
							{
								continue;
							}
						}

						else if ((featuretype == "fpfh"))
						{
							std::string featpath = "E:/pointclouddata/data_odometry_velodyne/feat_fpfh_txt/" + filename[0] + ".npz.txt";
							std::cout << featpath << std::endl;
							if (!readfeat_stc_tgt_kitti(featpath, fristkeypoint, secondkeypoint, fristfeature, secondfeature))
							{
								continue;
							}

							/*std::cout << pointcloudpath + first_pc_name + "_fpfh.npz" << std::endl;
							if (!readfeature(pointcloudpath + first_pc_name + "_fpfh.npz", fristkeypoint, fristfeature, false))
							{
								continue;
							}
							std::cout << pointcloudpath + second_pc_name + "_fpfh.npz" << std::endl;
							if (!readfeature(pointcloudpath + second_pc_name + "_fpfh.npz", secondkeypoint, secondfeature, false))
							{
								continue;
							}*/
						}
						fpc[i] = fristkeypoint;
						ffeat[i] = fristfeature;
						spc[i] = secondkeypoint;
						sfeat[i] = secondfeature;
					}
					else
					{
						fristkeypoint = fpc[i];
						fristfeature = ffeat[i];
						secondkeypoint = spc[i];
						secondfeature = sfeat[i];
					}

					/*if (fristkeypoint.rows() > para.Corres_IR_num)
						samplepoint(fristkeypoint, fristfeature, para.Corres_IR_num);
					if (secondkeypoint.rows() > para.Corres_IR_num)
						samplepoint(secondkeypoint, secondfeature, para.Corres_IR_num);
					cout << "samplepoint " << fristkeypoint.rows() << " " << secondkeypoint.rows() << endl;*/

					Dual_Point PR;
					PR.para = para;
					PR.gt_pose = gt_pose;
					PR.original_feature_model = fristfeature;
					PR.original_feature_scene = secondfeature;
					PR.original_feat_point_model = fristkeypoint;
					PR.original_feat_point_scene = secondkeypoint;
					//cout << PR.V_model.transpose() << endl;
					PR.V_model.setZero();
					PR.V_scene.setZero();
					PR.withnormal = false;
					//std::cout << fristfeature.rows() << " "<< secondfeature.rows() << endl;
					auto start = steady_clock::now();
					//clock_t tick1 = clock();
					PR.samplepoint(para.Corres_IR_num);
					
					//std::cout << fristfeature.rows() << " 11 " << secondfeature.rows() << endl;
					if ((featuretype == "fcgf"))
					{
						PR.FeatureMatch_cos_kd(false);
					}
					else
					{
						PR.FeatureMatch_cos_kd(true);
					}
	
					auto start2 = steady_clock::now();
					//std::cout << fristfeature.rows() << " 22 " << secondfeature.rows() << endl;

					Matrix4d prop = PR.Registration();
					auto end = steady_clock::now();
					

					

					auto last = duration_cast<microseconds>(end - start);

					{

						double acc = 0;
						double rre = 0;
						double rte = 0;

						Matrix4d dp = gt_pose * prop.inverse();
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1) d = -1;


						double dis = (gt_pose.block(0, 3, 3, 1) - prop.block(0, 3, 3, 1)).norm();
						dis;

						std::cout << "est_pose da=" << acos(d) / M_PI * 180 << endl;
						std::cout << "est_pose dt=" << dis << endl;



						if (acos(d) / M_PI * 180 < 5 && dis < 2.0)
						{

							acc = 1;
							rre = acos(d) / M_PI * 180;
							rte = dis * 100;
						}

						if (acos(d) / M_PI * 180 < 5 && dis < 2.0)
						{

							acc = 1;
							rre = acos(d) / M_PI * 180;
							rte = dis * 100;
						}
						if (acos(d) / M_PI * 180 < 5 && dis < 2.0)
						{
							
							for (int acc = 0; acc < 20; acc++)
							{
								if(acos(d) / M_PI * 180 < (5.0/20.0*(acc+1)))
									distributoin_angle_re(kkk* re_size + ana,acc+1) += 1;
							}
							
						}
						if (acos(d) / M_PI * 180 < 5 && dis < 2.0)
						{
							
							for (int acc = 0; acc < 20; acc++)
							{
								if (dis < (2.0 / 20.0 * (acc + 1)))
									distributoin_dis_re(kkk * re_size + ana, acc+1) += 1;
							}

						}
						
						//std::cout << "pose " << std::endl << prop << std::endl;
						//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
						std::cout << "  registration in "
							<< duration_cast<microseconds>(start2 - start).count()<< " sec" <<"  "<< last.count()<<"us" << std::endl;

						double time_us = last.count();
						ofstream fout;

						if ((sequence_name == "08"))
						{

							res[0][0] += 1.0;
							res[0][1] += acc;
							res[0][2] += rre;
							res[0][3] += rte;
							res[0][4] += time_us;

						}
						else if ((sequence_name == "09"))
						{

							res[1][0] += 1.0;
							res[1][1] += acc;
							res[1][2] += rre;
							res[1][3] += rte;
							res[1][4] += time_us;
						}
						else
						{

							res[2][0] += 1.0;
							res[2][1] += acc;
							res[2][2] += rre;
							res[2][3] += rte;
							res[2][4] += time_us;
						}


						double total_num = 0;
						double total_acc = 0;
						double total_rre = 0;
						double total_rte = 0;
						double total_time = 0;
						double avg_acc = 0;
						for (int ri = 0; ri < 3; ri++)
						{
							total_num += res[ri][0];
							total_acc += res[ri][1];
							total_rre += res[ri][2];
							total_rte += res[ri][3];
							total_time += res[ri][4];
							avg_acc += res[ri][1] / res[ri][0];
							cout << res[ri][0] << " " << res[ri][1] << " " << res[ri][1] / res[ri][0] << " " << res[ri][2] / res[ri][1] << " " << res[ri][3] / res[ri][1]
								<< " " << res[ri][4] / res[ri][0] << std::endl;

						}
						cout << total_num << " " << total_acc << " " << total_acc / total_num << " " << total_rre / total_acc << " " << total_rte / total_acc
							<< " " << total_time / total_num << " " << time_us << std::endl;

						analysis_re(kkk * re_size + ana, 1) = total_num;
						analysis_re(kkk* re_size + ana, 2) = total_acc;
						analysis_re(kkk* re_size + ana, 3) = total_acc / total_num;
						analysis_re(kkk * re_size + ana, 4) = avg_acc / 3.0;
						analysis_re(kkk * re_size + ana, 5) = total_rre / total_acc;
						analysis_re(kkk* re_size + ana, 6) = total_rte / total_acc;
						analysis_re(kkk* re_size + ana, 7) = total_time / total_num;

						angle_re.row(kkk* re_size + ana) = distributoin_angle_re.row(kkk * re_size + ana) / total_num;
						angle_re(kkk* re_size + ana, 0) = distributoin_angle_re(kkk * re_size + ana, 0);
						dis_re.row(kkk* re_size + ana) = distributoin_dis_re.row(kkk * re_size + ana) / total_num;
						dis_re(kkk * re_size + ana, 0) = distributoin_dis_re(kkk * re_size + ana, 0);
						cout << analysis_re << std::endl << std::endl;
						cout << angle_re << std::endl << std::endl;
						cout << dis_re  << std::endl << std::endl;
						std::cout << std::endl << std::endl;
						std::cout << std::endl << std::endl;
					}

					//system("pause");
				}
			}



			//return;
		}
	}
	
}

void PointCloudRegistration::Registration_DP()
{
	std::vector<std::string> data_name = { "7-scenes-redkitchen",
		   "sun3d-home_at-home_at_scan1_2013_jan_1",
		   "sun3d-hotel_uc-scan3",
		   "sun3d-hotel_umd-maryland_hotel1",
		   "sun3d-hotel_umd-maryland_hotel3",
		   "sun3d-home_md-home_md_scan9_2012_sep_30",
		   "sun3d-mit_76_studyroom-76-1studyroom2",
		   "sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika" };
	// big 
	PD_model_parameter para;
	para.Corres_Sample_num =  350;//400
	para.Corres_IR_num =  1500;//2500
	para.Sample_Iter = 500000;//30000
	para.dis_consistence_thre2 = 0.10;//0.1
	para.dis_min_thre = 0.20;// 0.1
	//para.angle_thre = 10.0; //10.0

	para.vote_angle_discrete = 8.0;//6.0 ?
	para.vote_posenum_max = 1000.0;//3000
	para.Clusteringdistance = 0.20;//
	para.ClusterNum = 250;//300
	para.ClusterNum2 = 20;//40
	para.VoteMaxtoMin = 4.0;//4.0
	para.very_num = 10;//15

	//para.IR_dis_min=0.08;//0.08
	para.Overlap_dis_min1 = 0.15;//0.15
	para.Overlap_dis_min2 = 0.10;//0.05
	para.Overlap_weight = 0.2;//0.1
	
	para.icp_num = 3;
	para.icp_pointnum = 1000;
	//para.feat_point_num = 2500;

	para.very_index = 3.0;//3.5
	para.ViewpointOverlap_dis = 0.10;//0.05
	para.Projected_grid_sphere=80;
	 //3dlo 
	/*PD_model_parameter para;
	para.Corres_Sample_num =  350;//400
	para.Corres_IR_num =  2500;//2500
	para.Sample_Iter = 500000;//30000
	para.dis_consistence_thre2 = 0.08;//0.1
	para.dis_min_thre = 0.1;// 0.1
	//para.angle_thre = 10.0; //10.0

	para.vote_angle_discrete =10.0;//6.0 ?
	para.vote_posenum_max = 1000.0;//2500
	para.Clusteringdistance = 0.2;//
	para.ClusterNum = 1000;//1700
	para.ClusterNum2 = 25;//40
	para.VoteMaxtoMin = 4.0;//4.0
	para.very_num = 15;//15

	para.icp_num = 5;
	para.icp_pointnum = 1000;

	//para.IR_dis_min=0.08;//0.08
	para.Overlap_dis_min1 = 0.15;//0.15
	para.Overlap_dis_min2 = 0.10;//0.05
	para.Overlap_weight = 0.1;//0.1
	
	//para.feat_point_num = 2500;

	para.very_index = 3.5;//3.5
	para.ViewpointOverlap_dis = 0.05;//0.05
	para.Projected_grid_sphere=80;//80*/

	para.normal_nei=7;
	para.angle_con=cos(30.0/180.0*M_PI);
	para.angle_overlap_con = cos(15.0 / 180.0 * M_PI);

	
	MatrixXf analysis_re = MatrixXf::Zero(2, 8);

	int aaa_size = 2;
	for (int kkk = 0; kkk < 1; kkk++)
	{
		para.vote_posenum_max =1000 + kkk*50;
		for (int ana = 0; ana < aaa_size; ana++)
		{
			para.ClusterNum2 = 15 +ana* 5;
			analysis_re(aaa_size*kkk+ ana, 0) = para.ClusterNum2 *1.0 + para.vote_posenum_max *10;
	

			std::vector< std::vector<double>> res;
			for (int dn = 0; dn < data_name.size(); dn++)
			{
				std::vector<MatrixXf> fpc(80);
				std::vector<MatrixXf> spc(80);
				std::vector<MatrixXf> ffeat(80);
				std::vector<MatrixXf> sfeat(80);
				for (int i = 0; i < 80; i++)
				{
					fpc[i].resize(10, 10);
					spc[i].resize(10, 10);
					ffeat[i].resize(10, 10);
					sfeat[i].resize(10, 10);
				}


				std::string  dataname = data_name[dn];


				std::vector<int> firstpointcloudname;
				std::vector<int> secondpointcloudname;

				//读位姿
				std::vector<Matrix4d> pointpose;
				string gtpath = "E:/pointclouddata/3Dmatch/Processed_3dmatch_3dlomatch/" + dataname + "/gt.log";
				//string gtpath = "E:/pointclouddata/3DLoMatch/3DLoMatch/" + dataname + "/gt.log";
				pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);

				//std::string pointcloudpath = "E:/pointclouddata/3Dmatch/noise/0.00/" + dataname + "/cloud_bin_";
				//std::string featuretype = "fcgf";//geo_match//fpfh//fcgf'

				std::string pointcloudpath = "E:/pointclouddata/3Dmatch/noise/0.00_fpfh/" + dataname + "/cloud_bin_";
				std::string featuretype = "fpfh";//geo_match//fpfh//fcgf

				int num = 0;
				int acc = 0;
				double total_time = 0;
				double total_feat_time = 0;
				double te = 0;
				double re = 0;
				std::vector<double>  value;
				value.resize(6);
				for (int i = 0; i < pointpose.size(); i++)
				{
					std::cout << i << " " << pointpose.size() << std::endl;

					int i1 = firstpointcloudname[i];
					int i2 = secondpointcloudname[i];
					std::cout << i1 << " " << i2 << std::endl;


					if (fabs(i1 - i2) < 2)
						continue;
					/*if (i1 != 30 || i2 != 32)
					{
						continue;
					}*/
					//cout << total_fram << endl;

					//Matrix4d est_pose = pointposeest[i].inverse();
					Matrix4d gt_pose = pointpose[i].inverse();

					MatrixXf fristkeypoint, secondkeypoint;
					MatrixXf fristfeature, secondfeature;
					VectorXf corr_scores;
					//cout << (featuretype == "fcgf") << endl;

					if ((featuretype == "fcgf"))
					{
						if (fpc[firstpointcloudname[i]].rows() < 100)
						{
							std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fcgf.txt" << std::endl;
							if (!readfeature2(pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fcgf.txt", fristkeypoint, fristfeature, false))
							{
								continue;
							}
							fpc[firstpointcloudname[i]] = fristkeypoint;
							ffeat[firstpointcloudname[i]] = fristfeature;
						}
						else
						{
							fristkeypoint = fpc[firstpointcloudname[i]];
							fristfeature = ffeat[firstpointcloudname[i]];
						}
						if (spc[secondpointcloudname[i]].rows() < 100)
						{
							std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fcgf.txt" << std::endl;
							if (!readfeature2(pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fcgf.txt", secondkeypoint, secondfeature, false))
							{
								continue;
							}
							spc[secondpointcloudname[i]] = secondkeypoint;
							sfeat[secondpointcloudname[i]] = secondfeature;
						}
						else
						{
							secondkeypoint = spc[secondpointcloudname[i]];
							secondfeature = sfeat[secondpointcloudname[i]];
						}
					}
					else if ((featuretype == "fpfh"))
					{
						if (fpc[firstpointcloudname[i]].rows() < 100)
						{
							std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fpfh.txt" << std::endl;
							if (!readfeature2(pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fpfh.txt", fristkeypoint, fristfeature, false))
							{
								continue;
							}
							fpc[firstpointcloudname[i]] = fristkeypoint;
							ffeat[firstpointcloudname[i]] = fristfeature;
						}
						else
						{
							fristkeypoint = fpc[firstpointcloudname[i]];
							fristfeature = ffeat[firstpointcloudname[i]];
						}

						if (spc[secondpointcloudname[i]].rows() < 100)
						{
							std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fpfh.txt" << std::endl;
							if (!readfeature2(pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fpfh.txt", secondkeypoint, secondfeature, false))
							{
								continue;
							}
							spc[secondpointcloudname[i]] = secondkeypoint;
							sfeat[secondpointcloudname[i]] = secondfeature;
						}
						else
						{
							secondkeypoint = spc[secondpointcloudname[i]];
							secondfeature = sfeat[secondpointcloudname[i]];
						}

					}


					/*if (fristkeypoint.rows() > para.Corres_IR_num)
						samplepoint(fristkeypoint, fristfeature, para.Corres_IR_num);
					if (secondkeypoint.rows() > para.Corres_IR_num)
						samplepoint(secondkeypoint, secondfeature, para.Corres_IR_num);
					cout << "samplepoint " << fristkeypoint.rows() << " " << secondkeypoint.rows() << endl;*/


					Dual_Point PR;
					PR.original_feature_model = fristfeature;
					PR.original_feature_scene = secondfeature;
					PR.original_feat_point_model = fristkeypoint;
					PR.original_feat_point_scene = secondkeypoint;

					PR.para = para;
					PR.gt_pose = gt_pose;

					PR.withnormal = false;
					//cout << PR.V_model.transpose() << endl;
					PR.V_model.setZero();
					PR.V_scene.setZero();
					clock_t tick1 = clock();
					clock_t tick3;
					Matrix4d est_pose;
					auto start = steady_clock::now();
					//std::cout << fristfeature.rows() << " "<< secondfeature.rows() << endl;
					if (PR.withnormal)
					{
						//std::cout << fristfeature.rows() << " -1 " << secondfeature.rows() << endl;
						PR.normal(para.normal_nei);
						//std::cout << fristfeature.rows() << " 00 " << secondfeature.rows() << endl;
						PR.samplepoint_normal(para.Corres_IR_num);
						//std::cout << fristfeature.rows() << " 11 " << secondfeature.rows() << endl;
						PR.FeatureMatch_cos();
						//std::cout << fristfeature.rows() << " 22 " << secondfeature.rows() << endl;
						tick3 = clock();
						est_pose = PR.Registration_normal();
					}
					else
					{
						//std::cout << fristfeature.rows() << " 00 " << secondfeature.rows() << endl;
						tick1 = clock();
						PR.samplepoint(para.Corres_IR_num);
						
						//std::cout << fristfeature.rows() << " 11 " << secondfeature.rows() << endl;
						if ((featuretype == "fcgf"))
						{
							PR.FeatureMatch_cos_kd(false);
						}
						else
						{
							PR.FeatureMatch_cos_kd(true);
						}

						//std::cout << fristfeature.rows() << " 22 " << secondfeature.rows() << endl;
						tick3 = clock();
						est_pose = PR.Registration();
					}
					auto end = steady_clock::now();
					clock_t tick2 = clock();

					auto last = duration_cast<microseconds>(end - start);


					std::cout << "  frame PR in "
						<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
						<< " sec  "<< last.count()<<"us" << std::endl;
					//system("pause");
					total_feat_time += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					total_time += last.count();
					num++;



					{



						Matrix3d dp = est_pose.block(0, 0, 3, 3) * (gt_pose).block(0, 0, 3, 3).transpose();
						double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
						if (d > 1) d = 1;
						if (d < -1)    d = -1;
						double dis = (gt_pose.block(0, 3, 3, 1) - est_pose.block(0, 3, 3, 1)).norm();


						std::cout << "est_pose da=" << acos(d) / M_PI * 180 << endl;
						std::cout << "est_pose dt=" << dis << endl;

						if (acos(d) / M_PI * 180 <= 15 && dis <= 0.3)
						{
							acc++;
							te += dis * 100.00;
							re += acos(d) / M_PI * 180;

							value[1] = acc;
							value[2] = te / acc;
							value[3] = re / acc;
						}
						value[4] = total_time / num;
						value[5] = total_feat_time / num;
						value[0] = num;
						//fout << std::endl;

						//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
						//std::cout << "est_pose " << std::endl << est_pose << std::endl;
						for (int v = 0; v < 6; v++)
							std::cout << value[v] << " ";
						std::cout << value[1] * 1.0 / value[0] << std::endl;

						double total_num = 0;
						double total_acc = 0;
						double total_te = 0;
						double total_re = 0;
						double total_avg = 0;
						double total_times = 0;
						int size = res.size();
						std::cout << size << std::endl;
						for (int d = 0; d < size; d++)
						{
							for (int v = 0; v < 6; v++)
								std::cout << res[d][v] << " ";
							std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
							total_num += res[d][0];
							total_acc += res[d][1];
							total_avg += res[d][1] * 1.0 / res[d][0];
							total_te += res[d][2] * res[d][1];
							total_re += res[d][3] * res[d][1];
							total_times += res[d][4] * res[d][0];
						}
						std::cout << "total num " << total_num << " "
							<< "total acc " << total_acc << " "
							<< "total avg " << total_acc / total_num << " "
							<< "total TE " << total_te / total_acc << " "
							<< "total RE " << total_re / total_acc << " "
							<< "total Times " << total_times / total_num << " "
							<< "scene avg " << total_avg / size << std::endl << std::endl;

						analysis_re(aaa_size* kkk + ana, 1) = total_num;
						analysis_re(aaa_size* kkk + ana, 2) = total_acc;
						analysis_re(aaa_size* kkk + ana, 3) = total_acc / total_num;
						analysis_re(aaa_size* kkk + ana, 4) = total_avg / size;
						analysis_re(aaa_size* kkk + ana, 5) = total_te / total_acc;
						analysis_re(aaa_size * kkk + ana, 6) = total_re / total_acc;
						analysis_re(aaa_size * kkk + ana, 7) = total_times / total_num;
						std::cout << analysis_re << std::endl << std::endl;
					}

				}

				res.push_back(value);
				double total_num = 0;
				double total_acc = 0;
				double total_te = 0;
				double total_re = 0;
				double total_avg = 0;
				double total_times = 0;
				int size = res.size();
				std::cout << size << std::endl;
				for (int d = 0; d < size; d++)
				{
					for (int v = 0; v < 5; v++)
						std::cout << res[d][v] << " ";
					std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
					total_num += res[d][0];
					total_acc += res[d][1];
					total_avg += res[d][1] * 1.0 / res[d][0];
					total_te += res[d][2] * res[d][1];
					total_re += res[d][3] * res[d][1];
					total_times += res[d][4] * res[d][0];
				}
				std::cout << "total num " << total_num << " "
					<< "total acc " << total_acc << " "
					<< "total avg " << total_acc / total_num << " "
					<< "total TE " << total_te / total_acc << " "
					<< "total RE " << total_re / total_acc << " "
					<< "total Times " << total_times / total_num << " "
					<< "scene avg " << total_avg / size << std::endl << std::endl;
				analysis_re(aaa_size* kkk + ana, 1) = total_num;
				analysis_re(aaa_size* kkk + ana, 2) = total_acc;
				analysis_re(aaa_size* kkk + ana, 3) = total_acc / total_num;
				analysis_re(aaa_size* kkk + ana, 4) = total_avg / size;
				analysis_re(aaa_size* kkk + ana, 5) = total_te / total_acc;
				analysis_re(aaa_size* kkk + ana, 6) = total_re / total_acc;
				analysis_re(aaa_size* kkk + ana, 7) = total_times / total_num;
				std::cout << analysis_re << std::endl << std::endl;
			}

		}
	}
	
}

void PointCloudRegistration::Registration_PPF_noise(std::string dataname, std::vector<double>& value2)
{
	registration_parameter para;
	para.denoisingneighborhood = 10;
	para.DenoisingConfidence = 0.99;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(200 + 0.1);//200
	model_parameter.feature_num = 300;// 500;
	model_parameter.clutter_veri = 8 + 0.1;//10
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	model_parameter.overlap_vl_angle = 2.0;
	model_parameter.overlap_n = 2.0;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10



	/* fpfh
	para.denoisingneighborhood = 20;
	para.DenoisingConfidence = 0.9;
	std::vector<Matrix4d> pose;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.SamplingNumber_f = 8000;
	//model_parameter.PPF_distance = value;//0.01
	model_parameter.PPF_anglenumber = 60;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = int(300 + 0.1);//200
	model_parameter.feature_num = 800;// 500;
	model_parameter.clutter_veri = 50 + 0.1;//10
	model_parameter.overlap_angle = 60;// 130;
	model_parameter.overlap_dis = 3.0;// 2.5; 采样半径的板粟
	model_parameter.overlap_vl_angle = 5.0;//2.0
	model_parameter.overlap_n = 2.0;
	//投票相关参数初始化，为简化使用，不重要的参数做了固定设置
	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.PPF_weight = 2.0;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 10;//10 */

	clock_t tick1 = clock();
	
	std::vector<std::string> data_name = { "7-scenes-redkitchen",
		   "sun3d-home_at-home_at_scan1_2013_jan_1",
		   "sun3d-home_md-home_md_scan9_2012_sep_30",
		   "sun3d-hotel_uc-scan3",
		   "sun3d-hotel_umd-maryland_hotel1",
		   "sun3d-hotel_umd-maryland_hotel3",
		   "sun3d-mit_76_studyroom-76-1studyroom2",
		   "sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika" };

	std::vector< std::vector<double>> res;

	for (int dn =0; dn < data_name.size(); dn++)
	{
		dataname = data_name[dn];

		//文件句柄
		intptr_t hFile = 0;
		//文件信息
		struct _finddata_t fileinfo;
		string p;
		std::vector<std::string> gtdirname;

		std::vector<int> firstpointcloudname;
		std::vector<int> secondpointcloudname;
		//std::vector<int> firstpointcloudname_est;
		//std::vector<int> secondpointcloudname_est;

		

		
		//读位姿
		std::vector<Matrix4d> pointpose;
		//std::vector<Matrix4d> pointposeest;
		std::vector<Vector3i> overlapest;
		std::vector<MatrixXd> pc_info;
		//pointpose.resize(1591);
		
		
		string gtpath = "E:/pointclouddata/3Dmatch/Processed_3dmatch_3dlomatch/"+ dataname +"/gt.log";	
		pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);
			//pointposeest = readpose(opestpath, firstpointcloudname_est, secondpointcloudname_est);

		

		//bool yn = true;
		//ofstream fout;
		//fout.open("E:/pointclouddata/3Dmatch/" + dataname + "/"+ savename +".txt", ios::out);
		//string proestpath = "E:/pointclouddata/3Dmatch/" + dataname + "/" + dataname + "-evaluation/"+ savename +".log";

		//fout.open("E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".txt", ios::out);
		//string proestpath = "E:/pointclouddata/3DLoMatch/" + dataname + "/" + savename + ".log";


		//ofstream posefout;
		//posefout.open(proestpath, ios::out);
		std::string pointcloudpath = "E:/pointclouddata/3Dmatch/noise/0.00/" + dataname + "/cloud_bin_";
		std::string featuretype = "fcgf";//geo_match//fpfh//fcgf

		int num = 0;
		int acc = 0;
		double total_time = 0;
		double te = 0;
		double re = 0;
		std::vector<double>  value;
		value.resize(5);
		for (int i =0; i < pointpose.size(); i++)
		{
			std::cout << i<<" "<< pointpose.size() << std::endl;

			int i1 = firstpointcloudname[i];
			int i2 = secondpointcloudname[i];
			std::cout << i1 << " " << i2 << std::endl;
			
			
			if (fabs(i1-i2) <2)
				continue;
			//cout << total_fram << endl;

			//Matrix4d est_pose = pointposeest[i].inverse();
			Matrix4d gt_pose = pointpose[i];

			
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				VectorXf corr_scores;
				//cout << (featuretype == "fcgf") << endl;
				if ((featuretype == "fcgf"))
				{
					std::cout << pointcloudpath  + to_string(firstpointcloudname[i]) + ".ply_fcgf.txt" << std::endl;
					if (!readfeature2(pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fcgf.txt", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath  + to_string(secondpointcloudname[i]) + ".ply_fcgf.txt" << std::endl;
					if (!readfeature2(pointcloudpath  + to_string(secondpointcloudname[i]) + ".ply_fcgf.txt", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				else if ((featuretype == "fpfh"))
				{
					std::cout << pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fpfh.txt" << std::endl;
					if (!readfeature2(pointcloudpath + to_string(firstpointcloudname[i]) + ".ply_fpfh.txt", fristkeypoint, fristfeature, false))
					{
						continue;
					}
					std::cout << pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fpfh.txt" << std::endl;
					if (!readfeature2(pointcloudpath + to_string(secondpointcloudname[i]) + ".ply_fpfh.txt", secondkeypoint, secondfeature, false))
					{
						continue;
					}
				}
				
				std::string fcpath, scpath;
				fcpath = pointcloudpath + to_string(firstpointcloudname[i]) + ".ply";
				scpath = pointcloudpath + to_string(secondpointcloudname[i]) + ".ply";
				std::cout << fcpath << std::endl;
				std::cout << scpath << std::endl;

				MatrixXf fristpoint, secondpoint;
				fristpoint=loadPLYSimple_bin(fcpath.c_str(), 0);
				secondpoint = loadPLYSimple_bin(scpath.c_str(), 0);
				fristpoint.conservativeResize(fristpoint.rows(), 3);
				secondpoint.conservativeResize(secondpoint.rows(), 3);

				std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
				fristpoint = StatisticsDenoise(fristpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;

				std::cout << "模型2去噪前" << secondpoint.rows() << std::endl;
				secondpoint = StatisticsDenoise(secondpoint,
					para.denoisingneighborhood, para.DenoisingConfidence);
				std::cout << "模型2去噪后" << secondpoint.rows() << std::endl;

				//拼接点云计算法向
				{
					MatrixXf fristm(fristkeypoint.rows() +fristpoint.rows(),3);
					MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
					fristm << fristkeypoint, fristpoint;
					fristpoint = fristm;
					secondm<< secondkeypoint, secondpoint;
					secondpoint = secondm;
				}
				std::cout << "计算法向"  << std::endl;
				Vector3f centerpoint(0, 0, 0);
				MatrixXf fristpointcloud = normal(fristpoint, centerpoint, 25, 0.08);//30 fpfh noise 50  0.1
				MatrixXf secondpointcloud = normal(secondpoint, centerpoint, 25, 0.08);//30 fpfh noise 50 0.1
				std::cout << "计算法向结束" << std::endl;
				//writePLY(fristpointcloud, "modelf.ply");
				//writePLY(secondpointcloud, "scenef.ply");
				//system("pause");
				//writePLY(fristpointcloud, (pointcloudsavepath + to_string(firstpointcloudname[i]) + ".ply").c_str());
				//writePLY(secondpointcloud, (pointcloudsavepath + to_string(secondpointcloudname[i]) + ".ply").c_str());

			
				
				//初始化配准方法类
				PPF_PSO_Match detectorpso(model_parameter);
				
				Matrix4d pose;
				{
						

						
						//std::cout << pose << endl;
						//std::cout << "groundturth:" << endl;
						//std::cout << pose << endl;
						//std::cout << pose << endl;
						detectorpso.set_groundturth_pose(gt_pose);
						//std::cout << "groundturth:" << endl;
						//std::cout << pointpose[i + 1]*pointpose[i].inverse()  << endl;
						//std::cout << (pointpose[i + 1]*pointpose[i].inverse()).inverse() << endl;
						
						//MatrixXf model = transformPCPose(fristpointcloud, pose);
						//overlap = calvoerlap(model.block(0,0, model.rows(),3), secondpointcloud.block(0, 0, secondpointcloud.rows(), 3), 0.03);
					
						//std::cout << "fristpointcloud rows=" << fristpointcloud.rows() << endl;
						//std::cout << "secondpointcloud rows=" << secondpointcloud.rows() << endl;

				}
					
				clock_t tick1 = clock();
				if ((featuretype == "fcgf") || (featuretype == "fpfh"))//拼接点云计算法向
				{
					//模型下采样
					//clock_t tick11 = clock();
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
					//clock_t tick21 = clock();
					//writePLY(pc, "E:/modelf.ply");
					//cout<<"test                    " << (double)(tick21 - tick11) / CLOCKS_PER_SEC << endl;
					//system("pause");
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					
					detectorpso.FeatureMatch_cos(fristfeature,secondfeature);
					//system("pause");
				}
				else if ((featuretype == "geo_match"))
				{
					
					//模型下采样
					detectorpso.Modeldown(fristpointcloud, fristkeypoint.rows());
					detectorpso.Scenedown(secondpointcloud, secondkeypoint.rows());
					//detectorpso.Model_Scene_down(fristpointcloud, secondpointcloud);
					//计算特征并匹配
					detectorpso.FeatureMatch_match(corr_scores);
				}
				
				//system("pause");
				

			
				//初始化配准方法模型的PPF特征哈希表化
				//detectorpso.trainModel_PPF_R();
				clock_t tick2 = clock();

				std::cout << "  frame trainModel in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;
                
				
			
				//record.push_back((double)(tick2 - tick1) / CLOCKS_PER_SEC);
				
				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的PPF配准
				//detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				detectorpso.match_PPF_registration2(results, inst_count, record, scenematching_parameter);
				//detectorpso.match_ransac_registration(results, inst_count, record, scenematching_parameter);
				//system("pause");
				//std::cout << point_each + 2 << " match_PPF in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				//std::cout << i1 << " " << i2 << std::endl;
				total_time += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
				num++;
				
				
				//posefout << firstpointcloudname[i] << "\t";
				///posefout << secondpointcloudname[i] << "\t";
				//posefout << total_fram <<endl;
				Matrix4d prop = results[0].Pose.inverse();
				Matrix4d est_pose = prop;
				/*for (int p1 = 0; p1 < 4; p1++)
				{
					for (int p2 = 0; p2 < 3; p2++)
					{
						posefout << prop(p1,p2) <<"\t";
					}
					posefout << prop(p1, 3) <<endl;
				}*/
				

				
				{
					
					
					
					Matrix3d dp = est_pose.block(0, 0, 3, 3) * (gt_pose).block(0, 0, 3, 3).transpose();
					double d = (dp.block(0, 0, 3, 3).trace() - 1.0) / 2.0;
					if (d > 1) d = 1;
					if (d < -1)    d = -1;
					record.push_back(acos(d));

					double dis = (gt_pose.block(0, 3, 3, 1) - est_pose.block(0, 3, 3, 1)).norm();
					record.push_back(dis);

					std::cout << "est_pose da=" << acos(d) / M_PI * 180 << endl;
					std::cout << "est_pose dt=" << dis << endl;

					if (acos(d) / M_PI * 180 <= 15 && dis <= 0.3)
					{
						acc++;
						te += dis * 100.00;
						re += acos(d) / M_PI * 180;

						value[1] = acc;
						value[2] = te / acc;
						value[3] = re / acc;
					}
					value[4] = total_time / num;
					value[0] = num;
					//fout << std::endl;

					//std::cout << "gt_pose " << std::endl << gt_pose << std::endl;
					//std::cout << "est_pose " << std::endl << est_pose << std::endl;
					for (int v = 0; v < 5; v++)
						std::cout << value[v] << " ";
					std::cout << value[1] * 1.0 / value[0] << std::endl;

					double total_num = 0;
					double total_acc = 0;
					double total_te = 0;
					double total_re = 0;
					double total_avg = 0;
					int size = res.size();
					std::cout << size << std::endl;
					for (int d = 0; d < size; d++)
					{
						for (int v = 0; v < 5; v++)
							std::cout << res[d][v] << " ";
						std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
						total_num += res[d][0];
						total_acc += res[d][1];
						total_avg += res[d][1] * 1.0 / res[d][0];
						total_te += res[d][2] * res[d][1];
						total_re += res[d][3] * res[d][1];
					}
					std::cout << "total num " << total_num << " "
						<< "total acc " << total_acc << " "
						<< "total avg " << total_acc / total_num << " "
						<< "total TE " << total_te / total_acc << " "
						<< "total RE " << total_re / total_acc << " "
						<< "scene avg " << total_avg / data_name.size() << " ";
				}
					
				//system("pause");
			}
		}
			
		
		res.push_back(value);
		double total_num = 0;
		double total_acc = 0;
		double total_te = 0;
		double total_re = 0;
		double total_avg = 0;
		int size = res.size();
		std::cout << size << std::endl;
		for (int d = 0; d < size; d++)
		{
			for (int v = 0; v < 5; v++)
				std::cout << res[d][v] << " ";
			std::cout << res[d][1] * 1.0 / res[d][0] << std::endl;
			total_num += res[d][0];
			total_acc += res[d][1];
			total_avg += res[d][1] * 1.0 / res[d][0];
			total_te += res[d][2] * res[d][1];
			total_re += res[d][3] * res[d][1];
		}
		std::cout << "total num " << total_num << " "
			<< "total acc " << total_acc << " "
			<< "total avg " << total_acc / total_num << " "
			<< "total TE " << total_te / total_acc << " "
			<< "total RE " << total_re / total_acc << " "
			<< "scene avg " << total_avg / data_name.size() << " ";
		//fout.close();
		//posefout.close();
		//return;
	}

	

	


}

	void PointCloudRegistration::Registration(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		
	}
	/// 功能：实现将第一帧点云与其他帧点云配准，主要用于求取模型与点云之间的变换矩阵，也可用在模型上添加补充点云
	/// @参数 cloud: 输入所要配准的点云列表，要求点云与第一帧有重叠。 
	/// @参数 newcloud: 配准后的点云列表。
	/// @参数 para: 参数结构体。
	/// @参数 pose: 估计的位姿矩阵列表。与除了点云序列第一帧之外的点云一一对应。
	void PointCloudRegistration::Registration_fine(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		
	}
	void PointCloudRegistration::EstimationLineMODO(std::vector<PointCloud*> cloud, std::vector<PointCloud*> &newcloud, registration_parameter para, std::vector<Matrix4d> &pose)
	{
		
	}
	

	bool PointCloudRegistration::savePoses(const std::vector<std::string> &poseName, const std::vector<Matrix4d> &pose, const std::string filepath)
	{
		std::ofstream file(filepath);
		if (!file.is_open())
			return false;
		assert(poseName.size() == pose.size());
		for (int i = 0; i < pose.size(); i++)
		{
			file << poseName[i] << std::endl;
			file << pose[i] << std::endl;
		}
		file.close();
		return true;
	}
}

