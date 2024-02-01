#include "point_cloud_registration.h"
#include "registration/registration_helpers.hpp"
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
using namespace scvc;
using namespace std;

namespace registration
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

void samplepoint(MatrixXf &pc, MatrixXf &feat,int pointnum)
{
	std::mt19937 gen(1); //10000 (unsigned int)time(NULL)+ (unsigned int)clock()
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



void PointCloudRegistration::Registration_KITTI(std::string featuretype, std::string posepath, std::string featruepath, std::string pointcloudpath)
{
	
	std::vector<Matrix4d> pose;
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
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

	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = int(60 + 0.1);//90/60
	scenematching_parameter.votepeaks_number = 1;
	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 3.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 15;//10
	scenematching_parameter.ICP_radius = 5;
	scenematching_parameter.ICP_neibor = 30; //re 30

	clock_t tick1 = clock();

	intptr_t hFile = 0;
	struct _finddata_t fileinfo;
	std::string p;
	std::vector<std::string> pointcloudpairname;


	if ((hFile = _findfirst(p.assign(posepath).append("/*").c_str(), &fileinfo)) != -1)
	{
		do
		{
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
					}

				}
			}
		} while (_findnext(hFile, &fileinfo) == 0);

	}
	_findclose(hFile);
	


	std::vector<MatrixXf> ffeat(600);
	std::vector<MatrixXf> sfeat(600);
	std::vector<MatrixXf> fpoint(600);
	std::vector<MatrixXf> spoint(600);
	std::vector<MatrixXf> fnormal(600);
	std::vector<MatrixXf> snormal(600);

	for (int i = 0; i < 600; i++)
	{
		ffeat[i]= MatrixXf::Zero(3,3);
		sfeat[i] = MatrixXf::Zero(3, 3);
		fpoint[i] = MatrixXf::Zero(3, 3);
		spoint[i] = MatrixXf::Zero(3, 3);
		fnormal[i] = MatrixXf::Zero(3, 3);
		snormal[i] = MatrixXf::Zero(3, 3);
	}

	MatrixXf analysis_re = MatrixXf::Zero(10, 8);
	for (int ana = 0; ana < 10; ana++)
	{
		//scenematching_parameter.ICP_radius = 5 + 5 * int(ana/4);
		analysis_re(ana, 0) = ana + 1;

		
		bool yn = true;
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

	

			std::cout << filename[0] << std::endl;

			std::string sequence_name = std::string(2 - pointfilename[0].length(), '0') + pointfilename[0];
			std::string first_pc_name = std::string(6 - pointfilename[1].length(), '0') + pointfilename[1];
			std::string second_pc_name = std::string(6 - pointfilename[2].length(), '0') + pointfilename[2];
			
			//read pose
			ifstream fin;
			MatrixXd gt_pose(4, 4);
			fin.open(posepath +"/"+ pointcloudpairname[i], ios::in);
			if (!fin.is_open())
			{
				std::cout << posepath + "/" + pointcloudpairname[i] << "无法找到这个文件！" << endl;
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

					
		
			{
				MatrixXf fristkeypoint, secondkeypoint;
				MatrixXf fristfeature, secondfeature;
				
				MatrixXf fristpointcloud;
				MatrixXf secondpointcloud;
				


				std::string featpath = featruepath + filename[0] + ".npz.txt";
				if(ffeat[i].rows() <10)
				{
					std::cout << featpath << std::endl;
					if (!readfeat_stc_tgt_kitti(featpath, fristkeypoint, secondkeypoint, fristfeature, secondfeature))
					{
						continue;
					}

					if (fristkeypoint.rows() > 5000)
						samplepoint(fristkeypoint, fristfeature, 5000);
					if (secondkeypoint.rows() > 5000)
						samplepoint(secondkeypoint, secondfeature, 5000);
					cout << "samplepoint " << fristkeypoint.rows() << " " << secondkeypoint.rows() << endl;


					std::string pcpath = pointcloudpath + sequence_name + "/pointcloud/";
					std::string fcpath, scpath;
					fcpath = pcpath + first_pc_name + ".bin.ply";
					scpath = pcpath + second_pc_name + ".bin.ply";
					std::cout << fcpath << std::endl;
					std::cout << scpath << std::endl;

					MatrixXf fristpoint, secondpoint;
					fristpoint = loadPLYSimple(fcpath.c_str(), 1);
					secondpoint = loadPLYSimple(scpath.c_str(), 1);


					


					{//拼接点云计算法向
						MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
						MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
						fristm << fristkeypoint, fristpoint.block(0, 0, fristpoint.rows(), 3);
						fristpoint = fristm;
						secondm << secondkeypoint, secondpoint.block(0, 0, secondpoint.rows(), 3);
						secondpoint = secondm;
					}



					std::cout << "计算法向" << std::endl;
					Vector3f centerpoint(0, 0, 0);
					fristpointcloud = normal(fristpoint, centerpoint, 25, 0.08);//30
					secondpointcloud = normal(secondpoint, centerpoint, 25, 0.08);//30
					std::cout << "计算法向结束" << std::endl;


					ffeat[i] = fristfeature;
					sfeat[i]= secondfeature;
					fpoint[i]= fristkeypoint;
					spoint[i]= secondkeypoint;

					fnormal[i]= fristpointcloud;
					snormal[i]= secondpointcloud;
				}
				else
				{
					fristfeature = ffeat[i];
					secondfeature=sfeat[i];
					fristkeypoint=fpoint[i] ;
					secondkeypoint=spoint[i];

					fristpointcloud=fnormal[i];
					secondpointcloud=snormal[i];

				}
			
				
					
				
				

				//
				//初始化配准方法类
				SCVC_PSO_Match detectorpso(model_parameter);

				detectorpso.set_groundturth_pose(gt_pose);
				

				clock_t tick1 = clock();
				
				{
					detectorpso.Modeldown(fristpointcloud, fristfeature.rows());	
					detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
					detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
				}
				clock_t tick2 = clock();

				std::cout << " FeatureMatch_cos in "
					<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
					<< " sec" << std::endl;

			

				double caltimme = 0;

				int inst_count = 1;//输入的实例数量
				std::vector<SPose3D_R> results;
				std::vector<double> record;
				//模型的SCVC配准
				detectorpso.match_SCVC_registration(results, scenematching_parameter);
				//std::cout << point_each + 2 << " match_SCVC in "<< (double)(tick4 - tick3) / CLOCKS_PER_SEC<< " sec" << std::endl;
				clock_t tick3 = clock();

				
				Matrix4d prop = results[0].Pose;
				
				//system("pause");
				{
					
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

						

						if (acos(d)/ M_PI *180 < 5 && dis < 2.0)
						{
						
							acc = 1;
							rre = acos(d) / M_PI * 180;
							rte = dis * 100;
						}
						

					}

					std::cout << "registration in "
						<< (double)(tick3 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;

		
					if ((sequence_name == "08"))
					{
						res[0][0] += 1.0;
						res[0][1] += acc;
						res[0][2] += rre;
						res[0][3] += rte;
						res[0][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;

					}
					else if ((sequence_name == "09"))
					{
						res[1][0] += 1.0;
						res[1][1] += acc;
						res[1][2] += rre;
						res[1][3] += rte;
						res[1][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					}
					else
					{

						res[2][0] += 1.0;
						res[2][1] += acc;
						res[2][2] += rre;
						res[2][3] += rte;
						res[2][4] += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					}
				

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




void PointCloudRegistration::Registration_3DMatch3DLoMatch(std::string dataset, std::string featuretype, std::string posepath, std::string featruepath, std::string pointcloudpath)
{

	std::vector<Matrix4d> pose;
	ppf_model_parameter model_parameter;
	model_parameter.relativeSamplingnum = 4000;// ;
	model_parameter.PPF_anglenumber = 90;//90
	model_parameter.Sampling_threshold_Angles = 25;
	model_parameter.line_minlenght = 0.01;
	model_parameter.voting_top = 200;//200
	model_parameter.feature_num = 500;// 500;
	model_parameter.clutter_veri = 25;//10
	model_parameter.overlap_angle = 130;// 130;
	model_parameter.overlap_dis = 2.5;// 2.5;
	model_parameter.overlap_vl_angle = 2.0;
	model_parameter.overlap_n = 2.0;

	ppf_scenematching_parameter scenematching_parameter;
	scenematching_parameter.votepeakseffective = 2.0;
	scenematching_parameter.voteAnglenum = 60;//90/60
	scenematching_parameter.votepeaks_number = 1;

	scenematching_parameter.samplingrate = 0.1;
	scenematching_parameter.Clusteringdistance = 1.0;//位姿聚类距离 1.0
	scenematching_parameter.ICP_nbIterations = 15;//
	scenematching_parameter.ICP_radius =12;// re 10
	scenematching_parameter.ICP_neibor = 30; //re 30
	
	

	clock_t tick1 = clock();
	
	std::vector<std::string> data_name = { "7-scenes-redkitchen",
		   "sun3d-hotel_uc-scan3",
		   "sun3d-hotel_umd-maryland_hotel1",
		   "sun3d-hotel_umd-maryland_hotel3",
			"sun3d-home_at-home_at_scan1_2013_jan_1",
		   "sun3d-mit_76_studyroom-76-1studyroom2",
		   "sun3d-home_md-home_md_scan9_2012_sep_30",
		   "sun3d-mit_lab_hj-lab_hj_tea_nov_2_2012_scan1_erika" };

	

	std::vector<MatrixXf> ffeat(88);
	std::vector<MatrixXf> sfeat(88);
	std::vector<MatrixXf> fpoint(88);
	std::vector<MatrixXf> spoint(88);
	std::vector<MatrixXf> fnormal(88);
	std::vector<MatrixXf> snormal(88);
	MatrixXf ansys(10, 8);
	ansys.setZero();
	for (int ana = 0; ana < 10; ana++)
	{
		ansys(ana, 0) = ana+1;

		std::vector< std::vector<double>> res;
		for (int dn = 0; dn < data_name.size(); dn++)
		{
			for (int i = 0; i < 88; i++)
			{
				ffeat[i].resize(3, 3);
				sfeat[i].resize(3, 3);
				fpoint[i].resize(3, 3);
				spoint[i].resize(3, 3);
				fnormal[i].resize(3, 3);
				snormal[i].resize(3, 3);
			}

			std::string dataname = data_name[dn];

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


			string gtpath = posepath + dataname + "/gt.log";
			//string gtpath = "E:/pointclouddata/3Dmatch/benchmarks/3DLoMatch/" + dataname + "/gt.log";
			pointpose = readpose(gtpath, firstpointcloudname, secondpointcloudname);
			//pointposeest = readpose(opestpath, firstpointcloudname_est, secondpointcloudname_est);

			string pcpath = pointcloudpath + dataname + "/cloud_bin_";
			string fpath = featruepath + dataname + "/cloud_bin_";

			int num = 0;
			int acc = 0;
			double total_time = 0;
			double te = 0;
			double re = 0;
			std::vector<double>  value;
			value.resize(5);
			for (int i = 0; i < pointpose.size(); i++)
			{
				std::cout << i << " " << pointpose.size() << std::endl;

				int i1 = firstpointcloudname[i];
				int i2 = secondpointcloudname[i];
				std::cout << i1 << " " << i2 << std::endl;


				if (fabs(i1 - i2) < 2)
					continue;
				//cout << total_fram << endl;

				//Matrix4d est_pose = pointposeest[i].inverse();
				Matrix4d gt_pose = pointpose[i];


				{
					MatrixXf fristkeypoint, secondkeypoint;
					MatrixXf fristfeature, secondfeature;
					VectorXf corr_scores;
					{
						std::cout << fpath + to_string(firstpointcloudname[i]) + ".ply_"+ featuretype +".txt" << std::endl;
						if (ffeat[firstpointcloudname[i]].rows() < 5)
						{
							if (!readfeature2(fpath + to_string(firstpointcloudname[i]) + ".ply_" + featuretype + ".txt", fristkeypoint, fristfeature, false))
							{
								continue;
							}
							ffeat[firstpointcloudname[i]] = fristfeature;
							fpoint[firstpointcloudname[i]] = fristkeypoint;
						}
						else
						{
							fristfeature = ffeat[firstpointcloudname[i]];
							fristkeypoint = fpoint[firstpointcloudname[i]];
						}
						if (sfeat[secondpointcloudname[i]].rows() < 5)
						{
							std::cout << fpath + to_string(secondpointcloudname[i]) + ".ply_" + featuretype + ".txt" << std::endl;
							if (!readfeature2(fpath + to_string(secondpointcloudname[i]) + ".ply_" + featuretype + ".txt", secondkeypoint, secondfeature, false))
							{
								continue;
							}
							sfeat[secondpointcloudname[i]] = secondfeature;
							spoint[secondpointcloudname[i]] = secondkeypoint;
						}
						else
						{
							secondfeature = sfeat[secondpointcloudname[i]];
							secondkeypoint = spoint[secondpointcloudname[i]];
						}

					}
					

					std::string fcpath, scpath;
					fcpath = pcpath + to_string(firstpointcloudname[i]) + ".ply";
					scpath = pcpath + to_string(secondpointcloudname[i]) + ".ply";
					
					std::cout << fcpath << std::endl;
					std::cout << scpath << std::endl;
					MatrixXf fristpointcloud;
					MatrixXf secondpointcloud;
					{

						if (fnormal[firstpointcloudname[i]].rows() < 5)
						{
							MatrixXf fristpoint;
							fristpoint = loadPLYSimple_bin(fcpath.c_str(), 0);
							fristpoint.conservativeResize(fristpoint.rows(), 3);

							std::cout << "模型1去噪前" << fristpoint.rows() << std::endl;
							fristpoint = StatisticsDenoise(fristpoint,10,0.9);
							std::cout << "模型1去噪后" << fristpoint.rows() << std::endl;
							//拼接点云计算法向
							MatrixXf fristm(fristkeypoint.rows() + fristpoint.rows(), 3);
							fristm << fristkeypoint, fristpoint;
							fristpoint = fristm;

							std::cout << "计算法向" << std::endl;
							Vector3f centerpoint(0, 0, 0);
							fristpointcloud = normal(fristpoint, centerpoint, 30, 0.08);//30 fpfh noise 50  0.1
							std::cout << "计算法向结束" << std::endl;

							fnormal[firstpointcloudname[i]] = fristpointcloud;
						}
						else
						{
							fristpointcloud = fnormal[firstpointcloudname[i]];
						}

						if (snormal[secondpointcloudname[i]].rows() < 5)
						{
							MatrixXf  secondpoint;
							secondpoint = loadPLYSimple_bin(scpath.c_str(), 0);
							secondpoint.conservativeResize(secondpoint.rows(), 3);

							std::cout << "模型2去噪前" << secondpoint.rows() << std::endl;
							secondpoint = StatisticsDenoise(secondpoint, 10, 0.9);
							std::cout << "模型2去噪后" << secondpoint.rows() << std::endl;
							//拼接点云计算法向
							MatrixXf secondm(secondkeypoint.rows() + secondpoint.rows(), 3);
							secondm << secondkeypoint, secondpoint;
							secondpoint = secondm;

							std::cout << "计算法向" << std::endl;
							Vector3f centerpoint(0, 0, 0);
							secondpointcloud = normal(secondpoint, centerpoint, 30, 0.08);//30 fpfh noise 50 0.1
							std::cout << "计算法向结束" << std::endl;
							snormal[secondpointcloudname[i]] = secondpointcloud;
						}
						else
						{
							secondpointcloud = snormal[secondpointcloudname[i]];
						}
					}


					//初始化配准方法类
					SCVC_PSO_Match detectorpso(model_parameter);

					Matrix4d pose;
					detectorpso.set_groundturth_pose(gt_pose);


					clock_t tick1 = clock();
					//拼接点云计算法向
					{
						//模型下采样
						detectorpso.Modeldown(fristpointcloud, fristfeature.rows());
						detectorpso.Scenedown(secondpointcloud, secondfeature.rows());
						//计算特征并匹配
						detectorpso.FeatureMatch_cos(fristfeature, secondfeature);
						//system("pause");
					}

					clock_t tick2 = clock();

					std::cout << "  frame trainModel in "
						<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
						<< " sec" << std::endl;



					double caltimme = 0;

					int inst_count = 1;//输入的实例数量
					std::vector<SPose3D_R> results;
					std::vector<double> record;
					//模型的SCVC配准
					detectorpso.match_SCVC_registration(results, scenematching_parameter);

					clock_t tick3 = clock();
					total_time += (double)(tick3 - tick1) / CLOCKS_PER_SEC;
					num++;

					Matrix4d prop = results[0].Pose.inverse();
					Matrix4d est_pose = prop;


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
					value[0] = num;

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
						<< "scene avg " << total_avg / data_name.size() << std::endl;

					ansys(ana, 1) = total_num;
					ansys(ana, 2) = total_acc;
					ansys(ana, 3) = total_acc / total_num;
					ansys(ana, 4) = total_te / total_acc;
					ansys(ana, 5) = total_re / total_acc;
					ansys(ana, 6) = total_avg / data_name.size();
					ansys(ana, 7) = total_time / num;
					std::cout << ansys << endl;

					//system("pause");
				}
			}


			res.push_back(value);
			double total_num = 0;
			double total_acc = 0;
			double total_te = 0;
			double total_re = 0;
			double total_avg = 0;
			total_time = 0;
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
				total_time += res[d][4]* res[d][0];
			}
			std::cout << "total num " << total_num << " "
				<< "total acc " << total_acc << " "
				<< "total avg " << total_acc / total_num << " "
				<< "total TE " << total_te / total_acc << " "
				<< "total RE " << total_re / total_acc << " "
				<< "scene avg " << total_avg / data_name.size() << std::endl;

			ansys(ana, 1) = total_num;
			ansys(ana, 2) = total_acc;
			ansys(ana, 3) = total_acc / total_num;
			ansys(ana, 4) = total_te / total_acc;
			ansys(ana, 5) = total_re / total_acc;
			ansys(ana, 6) = total_avg / data_name.size();
			ansys(ana, 7) = total_time / total_num;
			std::cout << ansys << endl << endl;;
			//fout.close();
			//posefout.close();
			//return;
		}

	}
	
}

}
