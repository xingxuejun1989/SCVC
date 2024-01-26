#include "registration_IcpOptimizer.h"
#include "registration_ppf_helpers.hpp"

using namespace std;
using namespace Eigen;




IcpOptimizer::IcpOptimizer(Matrix<double, Dynamic, 6> _firstCloud, Matrix<double, Dynamic, 6>  _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p, IcpMethod _method, bool _verbose,int m) :
firstCloud(_firstCloud.block(0, 0, _firstCloud.rows(), 3)), secondCloud(_secondCloud.block(0,0, _secondCloud.rows(),3)), nbIterations(_nbIterations), nbIterationsIn(_nbIterationsIn), mu(_mu), nbIterShrink(_nbIterShrink), p(_p), method(_method), verbose(_verbose), kNormals(_kNormals)
{
	firstNormals = _firstCloud.block(0, 3, _firstCloud.rows(), 3);
	secondNormals = _secondCloud.block(0, 3, _secondCloud.rows(), 3);
	//Initialize the computed transformation
	computedTransfo = RigidTransfo(RotMatrix::Identity(), TransMatrix::Zero(3, 1));

	//Initialize the Lagrange multipliers to 0 for step 2.1
	lambda.resize(firstCloud.rows(), 3);
	lambda.setZero();

	//Initialize the reference distance (bounding box diagonal of cloud 1)
	Matrix<double, 1, 3> minCloudOne = firstCloud.colwise().minCoeff();
	Matrix<double, 1, 3> maxCloudOne = firstCloud.colwise().maxCoeff();
	referenceDist = (maxCloudOne - minCloudOne).norm();
	//cout << "The reference distance is : " << referenceDist << endl;

	//Initialize the other parameter
	hasBeenComputed = false;
	//:
	//firstCloud(_firstCloud), secondCloud(_secondCloud), nbIterations(_nbIterations), nbIterationsIn(_nbIterationsIn), mu(_mu), nbIterShrink(_nbIterShrink), p(_p), method(_method), verbose(_verbose)
}
/*
Main constructor. Initializes the point clouds and the sparse ICP parameters.
*/
IcpOptimizer::IcpOptimizer(Matrix<double,Dynamic,3> _firstCloud, Matrix<double,Dynamic,3> _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p, IcpMethod _method, bool _verbose) : 
firstCloud(_firstCloud), secondCloud(_secondCloud), kNormals(_kNormals), nbIterations(_nbIterations), nbIterationsIn(_nbIterationsIn), mu(_mu), nbIterShrink(_nbIterShrink), p(_p), method(_method), verbose(_verbose)
{
  //Normal estimation
  cout << "Estimating normals for first cloud" << endl;
  firstNormals = estimateNormals(_firstCloud,kNormals);
  if(method == pointToPlane)
  {
    cout << "Estimating normals for second cloud" << endl;
    secondNormals = estimateNormals(_secondCloud,kNormals);
    cout << "Done with normal estimation" << endl;
  }

  //Initialize the computed transformation
  computedTransfo = RigidTransfo(RotMatrix::Identity(),TransMatrix::Zero(3,1));

  //Initialize the Lagrange multipliers to 0 for step 2.1
  lambda.resize(firstCloud.rows(),3);
  lambda.setZero();

  //Initialize the reference distance (bounding box diagonal of cloud 1)
  Matrix<double,1,3> minCloudOne = firstCloud.colwise().minCoeff();
  Matrix<double,1,3> maxCloudOne = firstCloud.colwise().maxCoeff();
  referenceDist = (maxCloudOne-minCloudOne).norm();
 // cout << "The reference distance is : " << referenceDist << endl;

  //Initialize the other parameter
  hasBeenComputed = false;
}

int IcpOptimizer::performSparceICP_single()
{
	if (firstCloud.rows() == 0 || secondCloud.rows() == 0)
		return 1;
	// if(method == pointToPoint)
	//   cout << "Beginning ICP with method Point to Point" << endl;
	// else if (method == pointToPlane)
	//   cout << "Beginning ICP with method Point to Plane" << endl;
	 //Initialize the point cloud that is going to move
	movingPC = firstCloud;
	movingNormals = firstNormals;

	//Beginning of the algorithm itself
	//for (int iter = 0; iter < nbIterations; iter++)
	{
		// cout << "Iteration " << iter << endl;
		 //1st step : Computing correspondances
		PointCloud matchPC = secondCloud; //Selecting y
		selectedNormals = secondNormals;

		//2nd step : Computing transformation
		RigidTransfo iterTransfo;
		for (int iterTwo = 0; iterTwo < nbIterationsIn; iterTwo++)
		{
			// step 2.1 Computing z

			//Compute h
			PointCloud h = movingPC - matchPC + lambda / mu;
			//Optimizing z with the shrink operator
			PointCloud z = PointCloud::Zero(h.rows(), 3);
			for (int i = 0; i < h.rows(); i++)
				z.row(i) = shrink(h.row(i).transpose());

			// step 2.2 point-to-point ICP

			//Compute C
			PointCloud c = matchPC + z - lambda / mu;
			//Make a standard ICP iteration
			
			iterTransfo = rigidTransformPointToPlane(movingPC, c, selectedNormals);
		
			//Updating the moving pointCloud
			movingPC = movePointCloud(movingPC, iterTransfo);
			movingNormals = (iterTransfo.first*movingNormals.transpose()).transpose();
			computedTransfo = compose(iterTransfo, computedTransfo);
			updateIter(iterTransfo); //Updating the iterations measure

			// step 2.3 Updating the Lagrange multipliers
			PointCloud delta = movingPC - matchPC - z;
			lambda = lambda + mu * delta;
		}
	}

	hasBeenComputed = true;
	return 0;
}

/*
This function is the main implementation of the algorithm where every step are made explicit.
*/
int IcpOptimizer::performSparceICP()
{
  if(firstCloud.rows() == 0 || secondCloud.rows() == 0)
    return 1;
 // if(method == pointToPoint)
 //   cout << "Beginning ICP with method Point to Point" << endl;
 // else if (method == pointToPlane)
 //   cout << "Beginning ICP with method Point to Plane" << endl;
  //Initialize the point cloud that is going to move
  movingPC = firstCloud;
  movingNormals = firstNormals;

  //Beginning of the algorithm itself
  for(int iter = 0; iter<nbIterations ; iter++)
  {
   // cout << "Iteration " << iter << endl;
    //1st step : Computing correspondances
    vector<int> matchIndice = computeCorrespondances(secondCloud,movingPC);
    PointCloud matchPC = selectSubsetPC(secondCloud,matchIndice); //Selecting y
    if(method == pointToPlane)
      selectedNormals = selectSubsetPC(secondNormals,matchIndice);

    //2nd step : Computing transformation
    RigidTransfo iterTransfo;
    for(int iterTwo = 0; iterTwo<nbIterationsIn;iterTwo++)
    {
      // step 2.1 Computing z
    
      //Compute h
      PointCloud h = movingPC-matchPC+lambda/mu;
      //Optimizing z with the shrink operator
      PointCloud z = PointCloud::Zero(h.rows(),3);
      for(int i=0;i<h.rows();i++)
        z.row(i) = shrink(h.row(i).transpose());

      // step 2.2 point-to-point ICP

      //Compute C
      PointCloud c = matchPC + z - lambda/mu;
      //Make a standard ICP iteration
      if(method == pointToPoint)
        iterTransfo = rigidTransformPointToPoint(movingPC,c);
      else if(method == pointToPlane)
        iterTransfo = rigidTransformPointToPlane(movingPC,c,selectedNormals);
      else
        cout << "Warning ! The method you try to use is incorrect !" << endl;

      //Updating the moving pointCloud
      movingPC = movePointCloud(movingPC,iterTransfo);
      movingNormals = (iterTransfo.first*movingNormals.transpose()).transpose();
      computedTransfo = compose(iterTransfo,computedTransfo);
      updateIter(iterTransfo); //Updating the iterations measure

      // step 2.3 Updating the Lagrange multipliers
      PointCloud delta = movingPC-matchPC - z;
      lambda = lambda + mu * delta;
    }
  }
  
  hasBeenComputed = true;
  return 0;
}

/* 
This function computes each closest point in refCloud for each point in queryCloud using the nanoflann kd-tree implementation. It returns the indice of the closest points of queryCloud.
*/
vector<int> IcpOptimizer::computeCorrespondances(Matrix<double,Dynamic,3> refCloud, Matrix<double,Dynamic,3> queryCloud)
{
  
    
   
    scvc::KDTree* kdt1 = scvc::BuildKDTree(refCloud.cast<float>());
    
    
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<float>> dists;
   scvc::SearchKDTree(kdt1, queryCloud.cast<float>(), indices, dists, 1);

    //Create an index
   
    vector<int> nearestIndices;
    nearestIndices.resize(queryCloud.rows());
    for (int i = 0; i < queryCloud.rows(); i++)
    {
        //Current query point

        nearestIndices[i]=indices[i][0];

   
    }
    return nearestIndices;


}

/*
Move the pointCloud according to the rigid transformation in t 
*/
PointCloud IcpOptimizer::movePointCloud(PointCloud pointCloud, RigidTransfo t)
{
  return (t.first*pointCloud.transpose()+t.second.replicate(1,pointCloud.rows())).transpose();
}

/* 
This function estimates the normals for the point cloud pointCloud. It makes use of the k nearest neighbour algorithm implemented in FLANN
*/
Matrix<double,Dynamic,3> IcpOptimizer::estimateNormals(Matrix<double,Dynamic,3> pointCloud, const size_t k)
{
 
  Matrix<double,Dynamic,3> normals;
  
  return normals;
}

/*
This function is the standard point to point ICP
a : moving cloud
b : reference cloud
*/
RigidTransfo IcpOptimizer::rigidTransformPointToPoint(PointCloud a, PointCloud b) const
{
  //Centering the point clouds
  Matrix<double,1,3> centerA = a.colwise().sum()/a.rows();
  Matrix<double,1,3> centerB = b.colwise().sum()/b.rows();
  PointCloud aCenter = a - centerA.replicate(a.rows(),1);
  PointCloud bCenter = b - centerB.replicate(b.rows(),1);

  //Computing the product matrix W
  Matrix<double,3,3> W = Matrix<double,3,3>::Zero(3,3);
  for(int i=0;i<a.rows();i++)
    W = W + a.row(i).transpose()*b.row(i);

  //Computing singular value decomposition
  JacobiSVD<Matrix<double,3,3>> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

  //Computing the rigid transformation
  RotMatrix rotation = svd.matrixV()*svd.matrixU().transpose();
  TransMatrix translation = (centerB.transpose()-rotation*centerA.transpose()).transpose();

  //Outputing the transformation
  if(verbose)
  { 
    cout << endl << endl << "Rotation Matrix : " << endl << rotation << endl;
    cout << "Translation Matrix : " << endl << translation << endl << endl << endl;
  }

  return RigidTransfo(rotation,translation);
}

/*
This function is the standard point to plane ICP
a : moving cloud
b : reference cloud
n : normal to the reference cloud b
*/
RigidTransfo IcpOptimizer::rigidTransformPointToPlane(PointCloud a, PointCloud b, Matrix<double,Dynamic,3> n) const
{
  //Initialize linear system
  Matrix<double,6,6> leftMember = Matrix<double,6,6>::Zero(6,6);
  Matrix<double,6,1> rightMember = Matrix<double,6,1>::Zero(6,1);

  //PointCloud c = PointCloud::Zero(a.rows(),3);
  for(int i=0;i<a.rows();i++)
  {
    //Computing c = a x n
    Matrix<double,1,3> c = a.row(i).cross(n.row(i));

    //Updating left member
    leftMember.block(0,0,3,3) += c.transpose()*c;                         //Top-left block
    leftMember.block(3,3,3,3) += n.row(i).transpose()*n.row(i);           //Bottom-right block
    leftMember.block(0,3,3,3) += 
      n.row(i).replicate(3,1).cwiseProduct(c.transpose().replicate(1,3)); //Top-right block
    leftMember.block(3,0,3,3) +=
      n.row(i).transpose().replicate(1,3).cwiseProduct(c.replicate(3,1)); //Bottom-left block

    //Updating right member
    double factor = (a.row(i)-b.row(i))*n.row(i).transpose();
    rightMember.block(0,0,3,1) -= factor*c.transpose();        //Top 3 elements
    rightMember.block(3,0,3,1) -= factor*n.row(i).transpose(); //Bottom 3 elements
  }
  
  //Solving linear system
  LDLT<Matrix<double,6,6>> ldlt(leftMember);
  Matrix<double,6,1> solution = ldlt.solve(rightMember);

  //Expressing the resulting transformation
  RotMatrix rotation = (AngleAxisd(AngleAxisd::Scalar(solution(0,0)), Vector3d::UnitX()) * AngleAxisd(AngleAxisd::Scalar(solution(1,0)), Vector3d::UnitY()) * AngleAxisd(AngleAxisd::Scalar(solution(2,0)), Vector3d::UnitZ())).matrix();
  TransMatrix translation = solution.block(3,0,3,1);

  return RigidTransfo(rotation,translation);
}

/*
This function implements the shrink operator which optimizes the function
f(z) = ||z||_2^p + mu/2*||z-h||_2^2
*/
TransMatrix IcpOptimizer::shrink(TransMatrix h) const
{
  double alpha_a = pow((2./mu)*(1.-p),1./(2.-p));
  double hTilde = alpha_a+(p/mu)*pow(alpha_a,p-1);
  double hNorm = h.norm();
  if(hNorm <= hTilde)
    return 0*h;
  double beta = ((alpha_a)/hNorm+1.)/2.;
  for(int i=0;i<nbIterShrink;i++)
    beta = 1 - (p/mu)*pow(hNorm,p-2.)*pow(beta,p-1);
  return beta*h;
}

/*
Computing composition of tNew by tOld (tNew o tOld)
*/
RigidTransfo IcpOptimizer::compose(RigidTransfo tNew, RigidTransfo tOld) const
{
  return RigidTransfo(tNew.first*tOld.first,tNew.first*tOld.second+tNew.second);
}

/*
Selects the subset of rows whose index is in indice in the Point Cloud p
*/
PointCloud IcpOptimizer::selectSubsetPC(PointCloud p, vector<int> indice) const
{
  PointCloud selection = PointCloud::Zero(indice.size(),3);
  for(int i=0;i<indice.size();i++)
    selection.row(i) = p.row(indice[i]);
  return selection;
}

/*
Updates the iterations measure by estimating the amplitude of rigid motion t
*/
void IcpOptimizer::updateIter(RigidTransfo t)
{
  Matrix<double,4,4> id = Matrix<double,4,4>::Identity();
  Matrix<double,4,4> curT = Matrix<double,4,4>::Identity();
  curT.block(0,0,3,3) = t.first;
  curT.block(0,3,3,1) = t.second / referenceDist;
  Matrix<double,4,4> diff = curT - id; //Difference between t and identity
  iterations.push_back((diff*diff.transpose()).trace()); //Returning matrix norm
}

/*
Save iterations to file
*/
void IcpOptimizer::saveIter(string pathToFile)
{
  ofstream txtStream(pathToFile.c_str());
  for(int i=0;i<iterations.size();i++)
    txtStream << iterations[i] << endl;
  txtStream.close();
}

/* 
Just a getter to the normals of the first cloud (moving cloud)
*/
Matrix<double,Dynamic,3> IcpOptimizer::getFirstNormals() const
{
  return firstNormals;
}

/*
Return a copy of the first point cloud which has been moved by the computed 
rigid motion.
If the rigid motion has not been computed it returns just the original first point cloud.
*/
PointCloud IcpOptimizer::getMovedNormals() const
{
  if(hasBeenComputed)
    return firstNormals;
  else
  {
    cout << "Warning ! The transformation has not been computed ! Please use the method \
    performSparceICP() before retrieving the moved normals." << endl;
    return movingNormals;
  }
}

/*
Return a copy of the first point cloud which has been moved by the computed 
rigid motion.
If the rigid motion has not been computed it returns just the original first point cloud.
*/
PointCloud IcpOptimizer::getMovedPointCloud() const
{
  if(hasBeenComputed)
    return movingPC;
  else
  {
    cout << "Warning ! The transformation has not been computed ! Please use the method \
    performSparceICP() before retrieving the moved point cloud." << endl;
    return firstCloud;
  }
}

/*
Return the computed transformation.
If it has not been computed, just returns the identity.
*/
RigidTransfo IcpOptimizer::getComputedTransfo() const
{
  if(!hasBeenComputed)
    cout << "Warning ! The transformation has not been computed ! Please use the method \
      performSparceICP() before retrieving the rigid motion." << endl;
  return computedTransfo; 
}

/*
Returns the reference distance which is the length of the great diagonal of the first 
point cloud's bounding box.
*/
double IcpOptimizer::getReferenceDist() const
{
  return referenceDist;
}
