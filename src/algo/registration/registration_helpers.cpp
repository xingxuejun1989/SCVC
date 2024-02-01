#include"registration_stdafx.h"
#include <random>
#include <ctime>
#include  <direct.h>  


namespace scvc
{






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
			float fea[3];
			if (ifs.read((char*)&fea[0], 3 * sizeof(float)))
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


} // namespace 

