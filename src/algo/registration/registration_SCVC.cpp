#include "registration_SCVC.h"
//2024.2.5
#include<ctime>
#include <random>

#include <algorithm>


#include"../ICP_Princeton/TriMesh.h"
#include"../ICP_Princeton/TriMesh_algo.h"
#include"../ICP_Princeton/ICP.h"
#include"../ICP_Princeton/KDtree.h"


using namespace std;


namespace scvc
{
	static const size_t SCVC_LENGTH = 5;

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
		double an = acos(a.dot(b));
		//if (an > M_PI / 2.0)
		//	an = M_PI - an;
		return an;
	}


	
	static bool pose3DPtrCompareVotes_R(const SPose3D_R & a, const SPose3D_R& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.numVotes > b.numVotes);
	}

	static bool Correspondence_R(const Correspondence& a, const Correspondence& b)
	{
		//CV_Assert(!a.empty() && !b.empty()); 需要加入空的判断
		return (a.value > b.value);
	}

	
	// quantize ppf and hash it for proper indexing
	static KeyType hashkey(unsigned long long  num)
	{
		return std::hash<unsigned long long>{}(num);
	}




	

	

	//SCVC_PSO_Match类
	SCVC_PSO_Match::SCVC_PSO_Match()
	{}
	SCVC_PSO_Match::SCVC_PSO_Match(ppf_model_parameter model_para)
	{
		model_parameter = model_para;
		groundtruth_enable = false;
		_R_model_para.model.hash_table.clear();
		_R_model_para.model.hash_nodes = NULL;
	}
	SCVC_PSO_Match::~SCVC_PSO_Match()
	{
	
		if (_R_model_para.model.hash_nodes != NULL)
			free(_R_model_para.model.hash_nodes);
		//if(model_features!= NULL)
		//       model_features.~shared_ptr();
		//if (scene_features != NULL)
		//      scene_features.~shared_ptr();
		delete _R_model_para.scene.down_kdt;
	}
	
	void SCVC_PSO_Match::Pose_Verfication(SPose3D_R& resultsS)
	{

		//cout << "model_parameter.overlap_angle = " << model_parameter.overlap_angle  << endl;;
		//cout << "model_parameter.overlap_dis = " << model_parameter.overlap_dis << endl;;

		double  over_thre_a = cos(model_parameter.overlap_angle / 180.0 * M_PI);//40
		double  thre_dis = _R_model_para.model.model_samplingdistance * model_parameter.overlap_dis;//2.0
		{
			MatrixXf gtmodel = transformPCPose(_R_model_para.model.feature_point, resultsS.Pose);

			int num = 0;
			int rows = gtmodel.rows();
			double  thre_dis_o = thre_dis * thre_dis;
			for (int i = 0; i < rows; i++)
			{
				if ((gtmodel.row(i).head(3) - _R_model_para.scene.feature_point.row(i).head(3)).squaredNorm() < thre_dis_o
					&& gtmodel.row(i).tail(3).dot(_R_model_para.scene.feature_point.row(i).tail(3)) > over_thre_a)
				{
					num++;
				}
			}
			resultsS.overlap = float(num) + 0.00001;
			resultsS.occlude = -resultsS.overlap;
			//return;

		}

		MatrixXf  source = transformPCPose(_R_model_para.model.downsample_model, resultsS.Pose);
		RowVector3f Sview = resultsS.Pose.block(0, 3, 3, 1).transpose().cast<float>();

		double  thre_a = cos(model_parameter.overlap_vl_angle/ 180.0 * M_PI);
		
		int sourcerow = source.rows();
		MatrixXf source_t = source.block(0, 0, sourcerow, 3);
		VectorXf source_t_dis = source_t.rowwise().norm();
		MatrixXf source_t_dir(sourcerow, 3);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < 3; i++)
		{
			source_t_dir.col(i) = source_t.col(i).array() / source_t_dis.array();
		}


		int targetrow = _R_model_para.scene.downsample_scene.rows();
		MatrixXf target_t = _R_model_para.scene.downsample_scene.block(0, 0, targetrow, 3);
		VectorXf target_t_dis = target_t.rowwise().norm();
		MatrixXf target_t_dir(targetrow, 3);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < 3; i++)
		{
			target_t_dir.col(i) = target_t.col(i).array() / target_t_dis.array();
		}

		VectorXi target_overlap = VectorXi::Zero(sourcerow);
		VectorXf target_distance = VectorXf::Zero(sourcerow);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < sourcerow; i++)
		{

			VectorXf angle =  target_t_dir*source_t_dir.row(i).transpose() ;

			for (int j = 0; j < 2; j++)
			{
				float maxs;
				int indexmaxs;
				maxs = angle.maxCoeff(&indexmaxs);
				if (maxs > thre_a)
				{
					double g = _R_model_para.scene.downsample_scene.row(indexmaxs).tail(3).dot(source.row(i).tail(3));
					double d = fabs(target_t_dis(indexmaxs) - source_t_dis(i));


					if (g > over_thre_a && d < thre_dis)
					{
						target_overlap(i) = 1;
						target_distance(i) = 0;

						break;
					}
					else if (target_t_dis(indexmaxs) + thre_dis >= source_t_dis(i))
					{
						target_distance(i) = d> target_distance(i)? d: target_distance(i);
						angle(indexmaxs) = -100;
					}
					
				}
				else
				{
				
					break;
				}
			}
		}


		//form source viewpoint view
		VectorXi source_overlap = VectorXi::Zero(sourcerow);
		VectorXf source_distance = VectorXf::Zero(sourcerow);
		VectorXi target_index = VectorXi::Zero(sourcerow);
		{
			MatrixXf source_s = source.block(0, 0, sourcerow, 3) - Sview.replicate(sourcerow, 1);
			MatrixXf target_s = _R_model_para.scene.downsample_scene.block(0, 0, targetrow, 3) - Sview.replicate(targetrow, 1);
			VectorXf target_s_dis = target_s.rowwise().norm();
			MatrixXf target_s_dir(targetrow, 3);

#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int i = 0; i < 3; i++)
			{
				target_s_dir.col(i) = target_s.col(i).array() / target_s_dis.array();
			}


#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int i = 0; i < sourcerow; i++)
			{

				double source_s_dis = source_s.row(i).norm();
				Vector3f source_s_dir = (source_s.row(i) / source_s_dis).transpose();
				VectorXf angle = target_s_dir * source_s_dir;

				for (int j = 0; j < 2; j++)
				{
					float maxs;
					int indexmaxs;
					maxs = angle.maxCoeff(&indexmaxs);
					

					if (maxs > thre_a)
					{
						//target_no_overlap[indexmaxs] = 0;


						double g = source.row(i).tail(3).dot(_R_model_para.scene.downsample_scene.row(indexmaxs).tail(3));
						double d = fabs(target_s_dis(indexmaxs) - source_s_dis);

						if (g > over_thre_a && d < thre_dis)
						{
							source_overlap(i) = 1;
							source_distance(i) = 0;

							break;
						}
						else if (source_s_dis + thre_dis >= target_s_dis(indexmaxs))
						{							
							source_distance(i) = d> source_distance(i)? d : source_distance(i);
							angle(indexmaxs) = -100;
						}
					}
					else
					{
						break;
					}
				}
			}
		}

		//cout << double(source_distance.sum() + target_distance.sum()) << " " << resultsS.overlap << endl;

		resultsS.occlude = double(source_distance.sum() + target_distance.sum()) / pow(resultsS.overlap, model_parameter.overlap_n);
		resultsS.overlap = double(source_overlap.sum() + target_overlap.sum()) / (sourcerow + targetrow);


	}

		
	void  SCVC_PSO_Match::Modeldown(const MatrixXf& original_model, int const feature_rows)
	{
		//设置模型数量

		std::vector<Vector2f> xRangelist;
		std::vector<Vector2f> yRangelist;
		std::vector<Vector2f> zRangelist;


		//模型原始数据
		_R_model_para.model.original_model = original_model;//
		//cout << original_model[model_id].maxCoeff() << original_model[model_id].minCoeff();
		//包围盒
		Vector2f xRange, yRange, zRange;
		computeBboxStd(_R_model_para.model.original_model, xRange, yRange, zRange);
		xRangelist.push_back(xRange);
		yRangelist.push_back(yRange);
		zRangelist.push_back(zRange);
		// compute sampling step from diameter of bbox 计算包围盒直径和 采样半径
		float dx = xRange[1] - xRange[0];
		float dy = yRange[1] - yRange[0];
		float dz = zRange[1] - zRange[0];

		//模型中心
		_R_model_para.model.center = Vector3d((xRange[1] + xRange[0]) * 0.5, (yRange[1] + yRange[0]) * 0.5, (zRange[1] + zRange[0]) * 0.5);
		//模型直径
		_R_model_para.model.model_diameter = sqrt(dx * dx + dy * dy + dz * dz);
		//模型参考点
		//_R_model_para.model.ferencepoint.resize(4, 4);
		_R_model_para.model.ferencepoint.setOnes();
		_R_model_para.model.ferencepoint.block(0, 0, 3, 1) = _R_model_para.model.center;
		_R_model_para.model.ferencepoint.block(0, 1, 3, 1) = _R_model_para.model.center;
		_R_model_para.model.ferencepoint.block(0, 2, 3, 1) = _R_model_para.model.center;
		_R_model_para.model.ferencepoint.block(0, 3, 3, 1) = _R_model_para.model.center;
		_R_model_para.model.ferencepoint(0, 1) += _R_model_para.model.model_diameter;
		_R_model_para.model.ferencepoint(1, 2) += _R_model_para.model.model_diameter;
		_R_model_para.model.ferencepoint(2, 3) += _R_model_para.model.model_diameter;
		//cout << _R_model_para.model.center << endl;
		//cout << _R_model_para.model.ferencepoint << endl;
		//模型下采样步长
		_R_model_para.model.model_samplingdistance = _R_model_para.model.model_diameter * 0.01;
		//cout << model_para.model[model_id].model_samplingdistance << " " << model_para.model[model_id].model_diameter << " " << sampling_step_relative << " " << endl;
		


		{
			//边界探索下采样
			VectorXi sumnum_model;
			//非均匀下采样
			double min = 0.5;
			double max = 0.0001;
			int pnum = model_parameter.relativeSamplingnum;//采样点数
			int n = 100;
			_R_model_para.model.model_samplingdistance = _R_model_para.model.model_diameter * 0.01;
	
			while (n--)
			{
				_R_model_para.model.downsample_model = samplePCByQuantization(_R_model_para.model.original_model, xRange, yRange, zRange, _R_model_para.model.model_samplingdistance, 0);
				if (fabs(_R_model_para.model.downsample_model.rows() - pnum) < 500)
				{
					break;
				}
				if (_R_model_para.model.downsample_model.rows() > pnum)
				{
					max = _R_model_para.model.model_samplingdistance / _R_model_para.model.model_diameter;
					_R_model_para.model.model_samplingdistance = _R_model_para.model.model_diameter * (max + min) / 2.0;
				}
				else
				{
					min = _R_model_para.model.model_samplingdistance / _R_model_para.model.model_diameter;
					_R_model_para.model.model_samplingdistance = _R_model_para.model.model_diameter * (max + min) / 2.0;
				}
			}

			if (feature_rows > 1)
			{
				_R_model_para.model.downsample_model_f = original_model.block(0, 0, feature_rows, 6);
			}

		}
	
	}
	
	void SCVC_PSO_Match::FeatureMatch(const MatrixXf& feature_model, const MatrixXf& feature_scene)
	{
		KDTree* kdt1 = BuildKDTree(feature_model);
		std::vector<std::vector<int>> model_indices;
		std::vector<std::vector<float>> model_dists;
		SearchKDTree(kdt1, feature_scene, model_indices, model_dists, 2);
		delete kdt1;

		KDTree* kdt2 = BuildKDTree(feature_scene);
		std::vector<std::vector<int>> scene_indices;
		std::vector<std::vector<float>> scene_dists;
		SearchKDTree(kdt2, feature_model, scene_indices, scene_dists, 2);
		delete kdt2;

		Correspondencepoint.clear();
		std::vector <Correspondence>  noCorrespondencepoint;
		int model_row = feature_model.rows();
		for (size_t i = 0; i < model_row; ++i)
		{

			// scene find model
			if (isfinite(scene_dists[i][0]) && isfinite(model_dists[scene_indices[i][0]][0]) && i == model_indices[scene_indices[i][0]][0])
			{
				Correspondence a;
				a.model = i;
				a.scene = scene_indices[i][0];
				//a.value = squaredDistances[0];

				double a1 = feature_cos(RowVectorXf(feature_model.row(i)),
					RowVectorXf(feature_scene.row(scene_indices[i][0])));
				a.value = a1;
				Correspondencepoint.emplace_back(a);
			}
			else
			{
				Correspondence a;
				a.model = i;
				a.scene = scene_indices[i][0];

				a.value = feature_cos(RowVectorXf(feature_model.row(i)),
					RowVectorXf(feature_scene.row(scene_indices[i][0])));

				noCorrespondencepoint.emplace_back(a);
			}
		}

		std::stable_sort(Correspondencepoint.begin(), Correspondencepoint.end(), Correspondence_R);
		std::stable_sort(noCorrespondencepoint.begin(), noCorrespondencepoint.end(), Correspondence_R);
		

		int corresnum = 1000;
		if (Correspondencepoint.size() > corresnum)
		{
			Correspondencepoint.resize(corresnum);
		}
		else
		{
			int size = Correspondencepoint.size();
			int num = noCorrespondencepoint.size() > (corresnum - size) ?
				(corresnum - size) : noCorrespondencepoint.size();
			for (int i = 0; i < num; i++)
			{
				Correspondencepoint.push_back(noCorrespondencepoint[i]);
			}
		}

		corresnum = Correspondencepoint.size();
		

		MatrixXf model_f(corresnum, 6);
		MatrixXf scene_f(corresnum, 6);
		for (int i = 0; i < corresnum; i++)
		{
			model_f.row(i) = _R_model_para.model.downsample_model_f.row(Correspondencepoint[i].model);
			scene_f.row(i) = _R_model_para.scene.downsample_scene_f.row(Correspondencepoint[i].scene);
			Correspondencepoint[i].model = i;
			Correspondencepoint[i].scene = i;
			//cout << model_f.row(i).tail(3).norm() << " " << scene_f.row(i).tail(3).norm() << endl;

		}
		
		MatrixXf model(corresnum + _R_model_para.model.downsample_model.rows(), 6);
		MatrixXf scene(corresnum + _R_model_para.scene.downsample_scene.rows(), 6);
		model << model_f,
			_R_model_para.model.downsample_model;
		scene << scene_f,
			_R_model_para.scene.downsample_scene;
		_R_model_para.model.downsample_model = model;
		_R_model_para.scene.downsample_scene = scene;

	}

	void SCVC_PSO_Match::FeatureMatch_match(const VectorXf& corr_scores)
	{
		
		int  row = corr_scores.size();
		for (int i = 0; i < row; i++)
		{
			Correspondence a;
			a.model = i;
			a.scene = i;
			a.value = corr_scores(i);
			Correspondencepoint.push_back(a);
		}
		std::stable_sort(Correspondencepoint.begin(), Correspondencepoint.end(), Correspondence_R);

		int corresnum = 1500;
		if (Correspondencepoint.size() > corresnum)
		{
			Correspondencepoint.resize(corresnum);
		}
		
		corresnum = Correspondencepoint.size();
		std::cout << "特征对 " << corresnum << std::endl;
		MatrixXf model_f(corresnum, 6);
		MatrixXf scene_f(corresnum, 6);
		for (int i = 0; i < corresnum; i++)
		{
			model_f.row(i) = _R_model_para.model.downsample_model_f.row(Correspondencepoint[i].model);
			scene_f.row(i) = _R_model_para.scene.downsample_scene_f.row(Correspondencepoint[i].scene);
			Correspondencepoint[i].model = i;
			Correspondencepoint[i].scene = i;
		}

		MatrixXf model(corresnum + _R_model_para.model.downsample_model.rows(), 6);
		MatrixXf scene(corresnum + _R_model_para.scene.downsample_scene.rows(), 6);
		model << model_f,
			_R_model_para.model.downsample_model;
		scene << scene_f,
			_R_model_para.scene.downsample_scene;
		_R_model_para.model.downsample_model = model;
		_R_model_para.scene.downsample_scene = scene;
		
	}
	void SCVC_PSO_Match::FeatureMatch_cos(const MatrixXf& feature_model, const MatrixXf& feature_scene)
	{


		int model_row = feature_model.rows();
		int scene_row = feature_scene.rows();
		
		Correspondencepoint.clear();
		std::vector <Correspondence>  noCorrespondencepoint;
		Correspondencepoint.resize(model_row);
		noCorrespondencepoint.resize(model_row);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < model_row; ++i)
		{
			RowVectorXf feat_cos(scene_row);
			for (int j = 0; j < scene_row; j++)
			{
				double a1 = feature_cos(RowVectorXf(feature_model.row(i)),
					RowVectorXf(feature_scene.row(j)));
				feat_cos(j) = a1;
			}
			int rowmaxindex;
			int colmaxindex;
			double a1 = feat_cos.maxCoeff(&rowmaxindex);

			RowVectorXf feat_cos_col(model_row);
			for (int j = 0; j < model_row; j++)
			{
				double a = feature_cos(RowVectorXf(feature_model.row(j)),
					RowVectorXf(feature_scene.row(rowmaxindex)));
				feat_cos_col(j) = a;
			}
			double a2 = feat_cos_col.maxCoeff(&colmaxindex);
			//double a2 = feat_cos.col(rowmaxindex).maxCoeff(&colmaxindex);
			feat_cos(rowmaxindex) = 0;
			double a3 = feat_cos.maxCoeff();
			//feat_cos(i, rowmaxindex) = a1;
			//cout << a1 << "  " << a2 << "  " << a3 << endl;
			if (i == colmaxindex)
			{
				//cout << a1 << "  " << a2 << endl;
				Correspondence a;
				a.model = i;
				a.scene = rowmaxindex;

				a.value = a1 /(a3 + 0.00001);
				Correspondencepoint[i]=a;
				noCorrespondencepoint[i].value = -1;
			}
			else
			{
				Correspondence a;
				a.model = i;
				a.scene = rowmaxindex;
				a.value = a1/ (a3 + 0.00001);
				noCorrespondencepoint[i]=a;
				Correspondencepoint[i].value = -1;
			}
			
		}


		std::stable_sort(Correspondencepoint.begin(), Correspondencepoint.end(), Correspondence_R);
		std::stable_sort(noCorrespondencepoint.begin(), noCorrespondencepoint.end(), Correspondence_R);
		for (size_t i = 0; i < model_row; ++i)
		{
			if (Correspondencepoint[i].value < 0)
			{
				Correspondencepoint.resize(i);
				break;
			}
		}
		for (size_t i = 0; i < model_row; ++i)
		{
			if (noCorrespondencepoint[i].value < 0)
			{
				noCorrespondencepoint.resize(i);
				break;
			}

		}
		int feature_num = model_row;
		{
			int feature_k;
			int k = 0;
			_R_model_para.model.feature_point.resize(feature_num, 6);
			_R_model_para.scene.feature_point.resize(feature_num, 6);
			
			for (k = 0; k < feature_num && k < Correspondencepoint.size(); k++)
			{
				_R_model_para.model.feature_point.row(k) = _R_model_para.model.downsample_model_f.row(Correspondencepoint[k].model);
				_R_model_para.scene.feature_point.row(k) = _R_model_para.scene.downsample_scene_f.row(Correspondencepoint[k].scene);
			}

			for (;k < feature_num && k < noCorrespondencepoint.size()+Correspondencepoint.size(); k++)
			{
				_R_model_para.model.feature_point.row(k) 
					= _R_model_para.model.downsample_model_f.row(noCorrespondencepoint[k- Correspondencepoint.size()].model);
				_R_model_para.scene.feature_point.row(k) 
					= _R_model_para.scene.downsample_scene_f.row(noCorrespondencepoint[k- Correspondencepoint.size()].scene);
			}
			if (k < feature_num)
			{
				_R_model_para.model.feature_point.conservativeResize(k, 6);
				_R_model_para.scene.feature_point.conservativeResize(k, 6);
			}
		}
	
		int corresnum = model_parameter.feature_num;
		if (Correspondencepoint.size() > corresnum)
		{
			Correspondencepoint.resize(corresnum);
		}
		else
		{
			int size = Correspondencepoint.size();
			int num = noCorrespondencepoint.size() > (corresnum - size) ? 
				 (corresnum - size) : noCorrespondencepoint.size();
			for (int i = 0; i < num; i++)
			{
				Correspondencepoint.push_back(noCorrespondencepoint[i]);
			}
		}
			
		corresnum = Correspondencepoint.size();
		//std::cout << "特征对2 " << corresnum << std::endl;
		MatrixXf model_f(corresnum, 6);
		MatrixXf scene_f(corresnum, 6);

#ifdef _OPENMP
#pragma omp parallel for 
#endif
		for (int i = 0; i < corresnum; i++)
		{
			model_f.row(i) = _R_model_para.model.downsample_model_f.row(Correspondencepoint[i].model);
			scene_f.row(i) = _R_model_para.scene.downsample_scene_f.row(Correspondencepoint[i].scene);
			Correspondencepoint[i].model = i;
			Correspondencepoint[i].scene = i;
			//cout << model_f.row(i).tail(3).norm() << " " << scene_f.row(i).tail(3).norm() << endl;
		}

		
		MatrixXf model(corresnum + _R_model_para.model.downsample_model.rows(), 6);
		MatrixXf scene(corresnum + _R_model_para.scene.downsample_scene.rows(), 6);
		model << model_f,
			_R_model_para.model.downsample_model;
		scene << scene_f,
			_R_model_para.scene.downsample_scene;
		_R_model_para.model.downsample_model = model;
		_R_model_para.scene.downsample_scene = scene;

	}
		
	
	double SCVC_PSO_Match::feature_cos(RowVectorXf& a, RowVectorXf& b)
	{
		double a1, a2, a3;
		
		a1 = a.dot(b);
		a2 = a.norm();
		a3 = b.norm();
		if (a2 > 0.00001 && a3 > 0.00001)
		{
			a1 = a1/a2/a3;
		}
		else
		{
			a1 = 0;
		}
		return a1;
	}

	
	void SCVC_PSO_Match::Scenedown(const MatrixXf& original_scene, int const feature_rows)
	{
		//计算包围盒
		Vector2f xRange, yRange, zRange;
		computeBboxStd(original_scene, xRange, yRange, zRange); //验证

		//计算包围盒直径
		float dx = xRange[1] - xRange[0];
		float dy = yRange[1] - yRange[0];
		float dz = zRange[1] - zRange[0];
		float diameter = sqrt(dx * dx + dy * dy + dz * dz);

		//场景直径
		_R_model_para.scene.scene_diameter = diameter;
		//保存场景
		_R_model_para.scene.original_scene = original_scene;


		{
			//计算场景下采样距离
			float SampleStepdistance = _R_model_para.model.model_samplingdistance;
			//std::cout << "模型采样步长=" << SampleStepdistance << " " << (SampleStepdistance / _R_model_para.model.model_diameter) << std::endl;
			_R_model_para.scene.scene_samplingdistance = SampleStepdistance;
			VectorXi sumnum_scene;
			//场景下采样
			_R_model_para.scene.downsample_scene = samplePCByQuantization(original_scene, xRange, yRange, zRange, SampleStepdistance, 0);
			if (feature_rows > 1)
			{
				_R_model_para.scene.downsample_scene_f = original_scene.block(0, 0, feature_rows, 6);
			}
			else
			{
				_R_model_para.scene.downsample_scene_f = samplePCByQuantization(original_scene, xRange, yRange, zRange,
					_R_model_para.model.model_feat_samplingdistance, 0);
			}
			
			
		}
	}

	
	void SCVC_PSO_Match::match_SCVC_registration(std::vector<SPose3D_R>& resultsS,const ppf_scenematching_parameter parameter)
	{

			//std::cout << "模型点数=" << _R_model_para.model.downsample_model.rows() << endl;
			int corres_size = Correspondencepoint.size();
			//模型ppf离散步长
			_R_model_para.model.model_ppf_distance_step = _R_model_para.model.model_samplingdistance;

			double model_ppf_distance_step = _R_model_para.model.model_ppf_distance_step;
			double model_minlenght = model_parameter.line_minlenght * _R_model_para.model.model_diameter;
			
			double angle_step_radians = M_2PI / model_parameter.PPF_anglenumber;

			double t1 = 1.0 / 180 * 3.1415;
			double t2 = 91.0 / 180 * 3.1415;
			double t3 = 89.0 / 180 * 3.1415;
			

			_R_model_para.scene.down_kdt = BuildKDTree(_R_model_para.scene.downsample_scene.block(0, 0, _R_model_para.scene.downsample_scene.rows(), 3));
			
			//投票角度数量
			int numAngles = parameter.voteAnglenum;

			//模型直径
			double model_diameter = _R_model_para.model.model_diameter;
			float eachinterval = M_2PI / numAngles;
  		   //场景和模型的点数
			int scenerows = _R_model_para.scene.downsample_scene.rows();
			int modelrows = _R_model_para.model.downsample_model.rows();

			clock_t tick1 = clock();
			//记录结果数组
			std::vector<SPose3D_R> resultsS3;
			resultsS3.resize(corres_size);
#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int p = 0; p < corres_size; p++)
			{
				
				std::vector<hashtable_list> hash_table(modelrows);//建立哈希表;
				THash_R* hash_nodes = (THash_R*)calloc(modelrows, sizeof(THash_R));;
				//
				Vector4d f;
				int i = Correspondencepoint[p].model;
				const Vector3f p1 = _R_model_para.model.downsample_model.row(i).head(3);
				const Vector3f n1 = _R_model_para.model.downsample_model.row(i).tail(3);

				for (int j = 0; j < modelrows; j++)
				{
					if (i != j)
					{

						const Vector3f p2 = _R_model_para.model.downsample_model.row(j).head(3);//获得坐标
						const Vector3f n2 = _R_model_para.model.downsample_model.row(j).tail(3);//获得法向


						computePPFFeatures(p1, n1, p2, n2, f);//计算特征值f
						//对一些不符合常规的特征去掉，主要为平面上奇异性大的点对
						if (f[3] > model_minlenght && f[2] > t1 && (f[0] > t2 || f[0] < t3))//	&& f[3] <radiusmax
						{
							
							//获得特征值

							//用于检测SCVC特征值是否一致
							unsigned long long nn = ((unsigned long long)(f[3] / model_ppf_distance_step)) * 1000000000 +
								((unsigned long long)(f[0] / angle_step_radians)) * 1000000 +
								((unsigned long long)(f[1] / angle_step_radians)) * 1000 +
								((unsigned long long)(f[2] / angle_step_radians));


							KeyType hashValue = hashkey(nn) % modelrows;//计算哈希表位置

							//获得属性表的记录位置
							THash_R* hashNode = &(hash_nodes[j]);
							hashNode->it = j;
							hashNode->ppfindex = nn;//特征转换为哈希表的关键值
							//hashNode->ppfInd = ppfInd;//特征点点对另一点在hash_nodes中的位置
							//插入结点
							hash_table[hashValue].index.push_back(j);
						

						}
					}
				}
			
			
				//
				MatrixXi modelcorrespoint(numAngles, 1000);
				MatrixXi secenecorrespoint(numAngles, 1000);

				//投票记录矩阵
				VectorXf accumulator = VectorXf::Zero(numAngles);
				VectorXi correspointindex = VectorXi::Zero(numAngles);
				//旋转和平移
				Matrix3f  PointRotate;
				//PointRotate.resize;
				Vector3f  PointTran;


				int n = Correspondencepoint[p].scene;
		
				//n点的 坐标 和  法向
			
				Vector3f sp1= _R_model_para.scene.downsample_scene.block(n, 0, 1, 3).transpose();
				Vector3f sn1= _R_model_para.scene.downsample_scene.block(n, 3, 1, 3).transpose();
		
				accumulator.setZero();

				//投票
				{
					
					Vector3f mp1= p1;
					Vector3f mn1= n1;

					//mn1 = mn1 / mn1.norm();
					Vector3f nms;
					nms = (mn1.cross(sn1)).normalized();
					if (nms.norm() < 0.5)
					{
						PointRotate.setIdentity();
					}
					else
					{
						Matrix3f R1, R2;
						R1.col(0) = mn1;
						R1.col(1) = nms;
						R1.col(2) = mn1.cross(nms);

						R2.col(0) = sn1;
						R2.col(1) = nms;
						R2.col(2) = sn1.cross(nms);

						PointRotate = R2 * R1.inverse();
					}

					PointTran = sp1 - PointRotate * mp1;
					
					for (int jj = 0; jj < scenerows; jj++)
					{
						
						const Vector3f sp2(_R_model_para.scene.downsample_scene.block(jj, 0, 1, 3).transpose());
						Vector3f sn2(_R_model_para.scene.downsample_scene.block(jj, 3, 1, 3).transpose());


						computePPFFeatures(sp1, sn1, sp2, sn2, f);//计算特征值f

						//是否是投票范围内的点			
						if (f[3] > model_minlenght) //f[3] <= radiusmax &&
						{

							//ppf特征索引				
							unsigned long long ppfindex = ((unsigned long long)(f[3] / model_ppf_distance_step)) * 1000000000 +
								((unsigned long long)(f[0] / angle_step_radians)) * 1000000 +

								((unsigned long long)(f[1] / angle_step_radians)) * 1000 +
								((unsigned long long)(f[2] / angle_step_radians));

		
							KeyType hashValue = hashkey(ppfindex) % modelrows;//计算索引哈希表索引
							//获得同一结点的哈希表指针
							int node=hash_table[hashValue].index.size();

							for(int it=0;it< node;it++)//遍历所有的结点上的模型SCVC
							{
								
								//获得模型的ppf信息
								THash_R* tData = &(hash_nodes[hash_table[hashValue].index[it]]);
								unsigned long long nn1 = tData->ppfindex;
								
								//特征值一样进行投票
								if (nn1 == ppfindex)//accumulatoryn(corrI) &&
								{
									int corrJ = tData->it;
									
									Vector3f mp2;
									mp2 = _R_model_para.model.downsample_model.block(corrJ, 0, 1, 3).transpose();

									Vector3f mp2_s = PointRotate * mp2 + PointTran;

									Vector3f  mp1p2 = mp2_s - sp1;
									Vector3f  sp1p2 = sp2 - sp1;
									Vector3f mpn = mp1p2.normalized();
									Vector3f spn = sp1p2.normalized();;

									Vector3f mpnn = (mpn - mpn.dot(sn1) * sn1).normalized();;
									Vector3f spnn = (spn - spn.dot(sn1) * sn1).normalized();;

									float angle = acos(max(min(mpnn.dot(spnn), 1.0f), -1.0f));
									float direction = (mpnn.cross(spnn)).dot(sn1);
									if (direction < 0) angle = M_2PI - angle;


									int alpha_index = int(angle / eachinterval)% numAngles;

									//cout << alpha_index << "  " << numAngles << "  " << alpha_index << endl;
									accumulator[alpha_index] += 1;

									if (correspointindex[alpha_index] < 1000)
									{
										modelcorrespoint(alpha_index, correspointindex[alpha_index]) = corrJ;
										secenecorrespoint(alpha_index, correspointindex[alpha_index]) = jj;
										correspointindex[alpha_index] ++;
									}
								}
						
							}
						}
					}
				}
		
	
				free(hash_nodes);

		
				
				{
					int  alphaIndMax = 0;
					double maxVotes = 0;
					//查找最大位置及最大投票数
					{
						for (int j = 0; j < numAngles; j++)
						{
							const double accVal = accumulator[j];

							if (accVal > maxVotes)
							{
								maxVotes = accVal;
								alphaIndMax = j;
							}
						}
					}
					//////////////// 投票数必须大于设置的最小值 
					if (maxVotes < parameter.votepeakseffective) //parameter.minpairnum
					{
						SPose3D_R pose;
						pose.numVotes = 0;
						resultsS3[p] = pose;
					}
					else
					{
						SPose3D_R pose;
						pose.numVotes = maxVotes;
						//插入相似点对
						int size = correspointindex[alphaIndMax];
						
						for (int j = 0; j < size; j++)
						{
								
								pose.modelpair.emplace_back(modelcorrespoint(alphaIndMax, j));
								pose.scenepair.emplace_back(secenecorrespoint(alphaIndMax, j));
									

						}
						//pose.numVotes = pose.modelpair.size();//用于测试
						pose.modelpair.emplace_back(Correspondencepoint[p].model);
						pose.scenepair.emplace_back(n);
						//resultsS3.push_back();
						resultsS3[p] = pose;
					}

				}

			}

			std::stable_sort(resultsS3.begin(), resultsS3.end(), pose3DPtrCompareVotes_R);
			for (int r = 0; r < resultsS3.size(); r++)
			{
				if (resultsS3[r].numVotes<2)
				{
					resultsS3.resize(r);
					break;
				}
			}
		
			if (resultsS3.size() > model_parameter.voting_top)
				resultsS3.resize(model_parameter.voting_top);

#ifdef _OPENMP
#pragma omp parallel for 
#endif
			for (int r = 0; r < resultsS3.size(); r++)
			{
				int row = resultsS3[r].modelpair.size();
				MatrixXd corresmodel(row, 3);
				MatrixXd corresscene(row, 3);
				MatrixXd corresmodel_t(1, 3);
				MatrixXd corresscene_t(1, 3);
				corresmodel_t.setZero();
				corresscene_t.setZero();


				for (int i = 0; i < row; i++)
				{
					corresmodel.row(i) = (_R_model_para.model.downsample_model.row(resultsS3[r].modelpair[i]).head(3)).cast<double>();
					corresscene.row(i) = (_R_model_para.scene.downsample_scene.row(resultsS3[r].scenepair[i]).head(3)).cast<double>();
					corresmodel_t += corresmodel.row(i);
					corresscene_t += corresscene.row(i);
		
				}
				Vector3d centermodel = (corresmodel_t / row).transpose();
				Vector3d centerscene = (corresscene_t / row).transpose();

				PointCloud model = corresmodel - (centermodel.transpose()).replicate(corresmodel.rows(), 1);
				PointCloud scene = corresscene - (centerscene.transpose()).replicate(corresscene.rows(), 1);

			
				Matrix3d W = Matrix3d::Zero(3, 3);
				for (int i = 0; i < model.rows(); i++)
					W = W + scene.row(i).transpose() * model.row(i);
	
				JacobiSVD<Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
				// Computing the rigid transformation
				RotMatrix rotation = svd.matrixU() * svd.matrixV().transpose();
				if (rotation.determinant() < 0)
				{
					rotation = -rotation;
				}
				Vector3d  translation = centerscene - rotation * centermodel;

				Matrix4d Tmg;
				if (isfinite(rotation(0, 0)))
				{
					MatrixXd P(3, 4);

					P << rotation, translation;
					Tmg << P,
						RowVector4d(0, 0, 0, 1);
				}
				else
				{
					Tmg.setIdentity();
					resultsS3[r].numVotes = 0;
				}

			
				resultsS3[r].Pose = Tmg;
				if(parameter.ICP_nbIterations>0)
				   Optimizing_registration(resultsS3[r], parameter.ICP_nbIterations, parameter.ICP_radius, parameter.ICP_neibor, true);
				
			}

		
		if (resultsS3.size() > 1)
		{

			//对结果聚类
			std::vector<SPose3D_R> rs;
			std::vector<Matrix4d> rw;
			
			int rwsize = 0;
			int rsize = resultsS3.size();
			rw.resize(rsize);
			Matrix4d ferencepoint;

			double dmax = _R_model_para.model.model_samplingdistance * parameter.Clusteringdistance;//1.5
			ferencepoint = _R_model_para.model.ferencepoint;

			for (int i = 0; i < rsize; i++)
			{

				Matrix4d w = resultsS3[i].Pose * ferencepoint;
				bool yn = true;
				for (int j = 0; j < rwsize; j++)
				{
					
					double d = (w.col(0) - rw[j].col(0)).norm();
					d += (w.col(1) - rw[j].col(1)).norm();
					d += (w.col(2) - rw[j].col(2)).norm();
					d += (w.col(3) - rw[j].col(3)).norm();
					d = d / 4;
					if (d < dmax)
					{
						rs[j].overlap += resultsS3[i].numVotes;
						rs[j].numVotes = rs[j].numVotes + 1;
						size_t size1 = resultsS3[i].modelpair.size();
						for (int p = 0; p < size1; p++)
						{
							rs[j].modelpair.emplace_back(resultsS3[i].modelpair[p]);
							rs[j].scenepair.emplace_back(resultsS3[i].scenepair[p]);
						}
						yn = false;
						break;
					}
				}
				if (yn && rwsize < rw.size())
				{
					resultsS3[i].overlap = resultsS3[i].numVotes;
					resultsS3[i].numVotes = 1;

					rs.push_back(resultsS3[i]);

					rw[rwsize] = w;
					rwsize++;
				}
			}
			if (rwsize < rs.size())
				rs.resize(rwsize);

		
			{
				resultsS3.clear();
				std::stable_sort(rs.begin(), rs.end(), pose3DPtrCompareVotes_R);
				//取前几个用于验证
				if (model_parameter.clutter_veri < rs.size())
					rs.resize(model_parameter.clutter_veri);
				rsize = rs.size();
				for (int i = 0; i < rsize; i++)
				{
					if (rs[i].numVotes < 2)
					{
						break;
					}
					rs[i].overlap = 0;
					rs[i].occlude = 0;

					resultsS3.push_back(rs[i]);
				}
				bool yn = false;
				rsize = resultsS3.size();
#ifdef _OPENMP
#pragma omp parallel for 
#endif
				for (int p = 0; p < rsize; p++)
				{
					int row = resultsS3[p].modelpair.size();

					MatrixXd corresmodel(row, 3);
					MatrixXd corresscene(row, 3);
					MatrixXd corresmodel_t(1, 3);
					MatrixXd corresscene_t(1, 3);
					corresmodel_t.setZero();
					corresscene_t.setZero();

					for (int i = 0; i < row; i++)
					{
						corresmodel.row(i) = (_R_model_para.model.downsample_model.row(resultsS3[p].modelpair[i]).head(3)).cast<double>();
						corresscene.row(i) = (_R_model_para.scene.downsample_scene.row(resultsS3[p].scenepair[i]).head(3)).cast<double>();
						corresmodel_t += corresmodel.row(i);
						corresscene_t += corresscene.row(i);
						
					}

					Vector3d centermodel = (corresmodel_t / row).transpose();
					Vector3d centerscene = (corresscene_t / row).transpose();
					PointCloud model = corresmodel - (centermodel.transpose()).replicate(corresmodel.rows(), 1);
					PointCloud scene = corresscene - (centerscene.transpose()).replicate(corresscene.rows(), 1);

					//Computing the product matrix W
					Matrix3d W = Matrix3d::Zero(3, 3);
					for (int i = 0; i < model.rows(); i++)
						W = W + scene.row(i).transpose() * model.row(i);


					JacobiSVD<Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);


					RotMatrix rotation = svd.matrixU() * svd.matrixV().transpose();
					if (rotation.determinant() < 0)
						rotation = -rotation;
					Vector3d  translation = centerscene - rotation * centermodel;

					Matrix4d Tmg;
					MatrixXd P(3, 4);
					P << rotation, translation;
					Tmg << P,
						RowVector4d(0, 0, 0, 1);
				
					resultsS3[p].Pose = Tmg;


					
				}
#ifdef _OPENMP
#pragma omp parallel for 
#endif
				for (int p = 0; p < resultsS3.size(); p++)
				{
					//验证姿态
					if (parameter.ICP_nbIterations > 0)
					     Optimizing_registration(resultsS3[p], parameter.ICP_nbIterations, parameter.ICP_radius, parameter.ICP_neibor, true);
				}
				{
					int p = 0;
					Pose_Verfication(resultsS3[p]);
				

				

					if (resultsS3[p].overlap < resultsS3[0].overlap / 5.0)
					{
						resultsS3[p].occlude = 10000000.0;
					}
					resultsS3[p].numVotes = -resultsS3[p].occlude;
				}

				for (int p = 1; p < resultsS3.size(); p++)
				{
					Pose_Verfication(resultsS3[p]);
				

					if (resultsS3[p].overlap < resultsS3[0].overlap / 5.0)
					{
						resultsS3[p].occlude = 10000000.0;
					}
					resultsS3[p].numVotes = -resultsS3[p].occlude;
				}
					
				std::stable_sort(resultsS3.begin(), resultsS3.end(), pose3DPtrCompareVotes_R);
			}

		}
		resultsS.push_back(resultsS3[0]);//选择第一个为配准最优结果
	}


	
	void SCVC_PSO_Match::PoseMatrixToAngle(Matrix4d pose, Vector3d & angle, Vector3d & translation)
	{
		translation = pose.block(0, 3, 3, 1);

		angle(0) = atan2(pose(2, 1), pose(2, 2)) / M_PI * 180;
		angle(1) = atan2(0 - pose(2, 0),sqrt(pose(2, 1) * pose(2, 1)+ pose(2, 2) * pose(2, 2))) / M_PI * 180;
		angle(2) = atan2(pose(1, 0), pose(0, 0)) / M_PI * 180;

	}
	void SCVC_PSO_Match::computePPFFeatures(const Vector3f& p1, const Vector3f& n1,
		const Vector3f& p2, const Vector3f& n2,
		Vector4d& f)
	{
		Vector3f d(p2 - p1);
		//f[3] = cv::norm(d);
		f[3] = d.norm();
		if (f[3] <= EPS)
			return;
		d *= 1.0 / f[3];

		f[0] = TAngle3Normalized(n1, d);
		f[1] = TAngle3Normalized(n2, d);
		f[2] = TAngle3Normalized(n1, n2);
	}

	void  SCVC_PSO_Match::Optimizing_registration(SPose3D_R& resultsS, int max_iter, double radius, int neibor, bool down)
	{
		
		MatrixXf  scene, model;
		if(down)
		{
			scene = _R_model_para.scene.downsample_scene;
			model = transformPCPose(_R_model_para.model.downsample_model, resultsS.Pose);;

		}
		else
		{
			scene = _R_model_para.scene.original_scene;
			model = transformPCPose(_R_model_para.model.original_model, resultsS.Pose);;
		}

		
		VectorXi modelindex(model.rows());
		VectorXi sceneindex(scene.rows());
		modelindex.setZero();
		sceneindex.setZero();

		KDTree* kdt1;
		if (down)
		{
			kdt1 = _R_model_para.scene.down_kdt;
		}
		else
		{
			kdt1 = BuildKDTree(scene.block(0, 0, scene.rows(), 3));
		}
		
		std::vector<std::vector<int>> indices;
		std::vector<std::vector<float>> dists;
		SearchKDTree(kdt1, model.block(0, 0, model.rows(), 3), indices, dists, neibor);
		double max_dis = _R_model_para.model.model_samplingdistance * radius;//20
		if (!down)
			max_dis = _R_model_para.model.model_samplingdistance * radius;//20
		max_dis = max_dis * max_dis;
		//radiusSearchhKDTree(kdt1, model.block(0, 0, model.rows(), 3), indices, dists, max_dis);
		if (!down)
		{
			
			cout << "用内存" << kdt1->usedMemory() << endl;
			//kdt1->~Index();
			//cout <<"用内存" << kdt1->usedMemory() << endl;
			delete kdt1;
		}
			

		int  mrow = model.rows();
		
		for (int i = 0; i < mrow; i++)
		{
			int size = dists[i].size();
			if (dists[i][0]< max_dis )
				modelindex(i) = 1;
			for (int j = 0; j < neibor; j++)
			{
				if (dists[i][j] < max_dis )
					sceneindex(indices[i][j]) = 1;
			}
		}

		int  srow = scene.rows();
		MatrixXf  scene_o(sceneindex.sum(),6);
		MatrixXf  model_o(modelindex.sum(),6);
		if (scene_o.rows() < 50 || model_o.rows() < 50)
			return;
		int index = 0;
		
		for (int i = 0; i < srow; i++)
		{
			if (sceneindex(i) == 1)
			{
				scene_o.row(index) = scene.row(i);
				index ++;
			}
		}
		index = 0;
		
		for (int i = 0; i < mrow; i++)
		{
			if (modelindex(i) == 1)
			{
				model_o.row(index) = model.row(i);
				index++;
			}
		}
		Matrix4d pose1 = ICP_Optimizing(model_o, scene_o, max_iter);
		//Matrix4d pose1 = SparceICP_Optimizing(model_o, scene_o, max_iter);
		Vector3d   angle, translation;
		PoseMatrixToAngle(pose1, angle, translation);
		//std::cout  << angle.transpose() << " " << translation.norm() << " " << model_para.model[model_id].model_diameter*0.5 << std::endl << pose1 << std::endl;
		if (fabs(angle(0)) < 30
			&& fabs(angle(1)) < 30
			&& fabs(angle(2)) < 30
			&& translation.norm() < _R_model_para.model.model_diameter * 1.0)
		{
			resultsS.Pose = pose1 * resultsS.Pose;
		}
	}
	
	Matrix4d   SCVC_PSO_Match::ICP_Optimizing(MatrixXf source, MatrixXf target, int max_iter)
	{
		trimesh::TriMesh *mesh1 = new trimesh::TriMesh();
		trimesh::TriMesh *mesh2 = new trimesh::TriMesh();

		int modelpointnum1 = target.rows();
		int modelpointnum2 = source.rows();
		mesh1->vertices.resize(modelpointnum1);
		mesh1->normals.resize(modelpointnum1);

		mesh2->vertices.resize(modelpointnum2);
		mesh2->normals.resize(modelpointnum2);

		for (int i = 0; i < modelpointnum1; i++)
		{
			mesh1->vertices[i][0] = target(i, 0);
			mesh1->vertices[i][1] = target(i, 1);
			mesh1->vertices[i][2] = target(i, 2);

			mesh1->normals[i][0] = target(i, 3);
			mesh1->normals[i][1] = target(i, 4);
			mesh1->normals[i][2] = target(i, 5);
		}

		for (int i = 0; i < modelpointnum2; i++)
		{
			mesh2->vertices[i][0] = source(i, 0);
			mesh2->vertices[i][1] = source(i, 1);
			mesh2->vertices[i][2] = source(i, 2);

			mesh2->normals[i][0] = source(i, 3);
			mesh2->normals[i][1] = source(i, 4);
			mesh2->normals[i][2] = source(i, 5);
		}

		clock_t tick1 = clock();
		Matrix4d p;
		trimesh::ICP_xform_type xform_type = trimesh::ICP_RIGID;


		trimesh::KDtree *kd1 = new  trimesh::KDtree(mesh1->vertices);
		trimesh::KDtree *kd2 = new trimesh::KDtree(mesh2->vertices);
		vector<float> weights1, weights2;
	
		double m[16];
		m[0] = 1;  m[1] = 0;  m[2] = 0;  m[3] = 0;
		m[4] = 0;  m[5] = 1;  m[6] = 0;  m[7] = 0;
		m[8] = 0;  m[9] = 0;  m[10] = 1; m[11] = 0;
		m[12] = 0; m[13] = 0; m[14] = 0; m[15] = 1;
		trimesh::xform xf1;
		trimesh::xform xf2;
		xf1 = m;
		xf2 = m;

		

		float err = ICP(mesh1, mesh2, xf1, xf2, kd1, kd2, weights1, weights2,
			0.0f, 0, max_iter, xform_type);
		//
		p(0, 0) = xf2[0];
		p(0, 1) = xf2[1];
		p(0, 2) = xf2[2];
		p(0, 3) = xf2[3];

		p(1, 0) = xf2[4];
		p(1, 1) = xf2[5];
		p(1, 2) = xf2[6];
		p(1, 3) = xf2[7];

		p(2, 0) = xf2[8];
		p(2, 1) = xf2[9];
		p(2, 2) = xf2[10];
		p(2, 3) = xf2[11];

		p(3, 0) = xf2[12];
		p(3, 1) = xf2[13];
		p(3, 2) = xf2[14];
		p(3, 3) = xf2[15];
		//std::cout << p.transpose() << std::endl;
		mesh1->clear();
		mesh2->clear();
		vector<float>().swap(weights1);
		vector<float>().swap(weights2);
		delete mesh1;
		delete mesh2;
		
		delete kd1;
		delete kd2;

		return p.transpose();
	}
	

}
