// PPFsurfacematching.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"PPF_Match.h"
#include"Eigen/Core"
#include<ctime>

using namespace std;
using namespace ppf_match;
using namespace Eigen;

int main()
{

#ifdef _OPENMP
	cout << "Running with OpenMP" << endl;
#else
	cout << "Running without OpenMP and without TBB" << endl;
#endif

	omp_init_lock(&lock); // 初始化互斥锁

	//string modelFileName = (string)"D:\\data\\xpmodel-normal-s5-2.ply";
	//modelFileName = (string)"D:\\data\\xpcence-normal-s5.ply";
	//string sceneFileName = (string)"D:\\data\\xpcence-normal-s4.ply";

	string modelFileName = (string)"G:\\test\\surfacematching\\data\\parasaurolophus_6700.ply";
	string sceneFileName = (string)"G:\\test\\surfacematching\\data\\rs1_normals.ply";


	MatrixXf pc = loadPLYSimple(modelFileName.c_str(), 1);//读入模型文件
	clock_t tick1 = clock();
	//data_tree_int3 = BuildKDTree(pc.block(0, 0, pc.rows(), 3));

	PPF_Match detector(0.03, 0.04, 120,0);//采样聚类，阈值，360的等份数
	
	detector.trainModel(pc);
	clock_t tick2 = clock();

	cout << endl << "Training complete in "
		<< (double)(tick2 - tick1) / CLOCKS_PER_SEC
		<< " sec" << endl << "Loading model..." << endl;

cout << "Training..." << endl;

	
	// Read the scene
    MatrixXf pcTest = loadPLYSimple(sceneFileName.c_str(), 1);


	// Match the model to the scene and get the pose
	cout << endl << "Starting matching..." << endl;
	vector<SPose3D> results;
	tick1 = clock();
	detector.match2(pcTest, results, 0.025,10,0.05,50);//场景、结果、关键点采样比例、下采样值、最小关键点数、旋转的相似性、优化姿态数
	tick2 = clock();
	cout << endl << "PPF Elapsed Time " <<
		(double)(tick2 - tick1) / CLOCKS_PER_SEC << " sec" << endl;
	
	//check results size from match call above
		size_t results_size = results.size();
	cout << "Number of matching poses: " << results_size<< endl;
	//cout << "Number of matching poses: " << (Pose3DPtr)(results.at(0)). << endl;
		if (results_size == 0) {
		cout << endl << "No matching poses found. Exiting." ;
		exit(0);
	}

	// Get only first N results - but adjust to results size if num of results are less than that specified by N
	size_t N = 10;
	if (results_size < N) {
		cout << endl << "Reducing matching poses to be reported (as specified in code): "
			<< N << " to the number of matches found: " << results_size << endl;
		N = results_size;
	}
	vector<SPose3D> resultsSub(results.begin(), results.begin() + N);

	// Create an instance of ICP
	//ICP icp(100, 0.005f, 2.5f, 8);
	tick1 = clock();

	// Register for all selected poses
	cout << endl << "Performing ICP on " << N << " poses..." << endl;
	//icp.registerModelToScene(pc, pcTest, resultsSub);
	tick2 = clock();

	cout << endl << "ICP Elapsed Time " <<
		(double)(tick2 - tick1) / CLOCKS_PER_SEC << " sec" << endl;

	cout << "Poses: " << endl;
	// debug first five poses
	for (size_t i = 0; i<resultsSub.size(); i++)
	{
		SPose3D result = resultsSub[i];
		cout << "Pose Result " << i << endl;
		cout << result.overlap<<" "<< result.numVotes << endl;
		cout << result.Pose << endl;
		string str = "para6700PCTrans" + to_string(i) + "b.ply";

		MatrixXf pct = transformPCPose(pc, result.Pose);
		//std::cout << pct.block(0,0,10,6) << std::endl;
		//int mm;
		//std::cin >> mm;
		writePLY(pct, str.c_str());

	}

	cout << "over " << endl;

	system("pause");
    return 0;
}

