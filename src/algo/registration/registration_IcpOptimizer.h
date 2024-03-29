#ifndef ICP_OPTIMIZER_H
#define ICP_OPTIMIZER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <Eigen/Geometry>


/* Each point in a point cloud is loaded as a line vector, 
but every computation is made with the mathematical convention ! (column vectors)
As a consequence, TransMatrix is a column vector */

typedef Eigen::Matrix<double,3,3> RotMatrix;               //A type for the rotation matrix
typedef Eigen::Matrix<double,3,1> TransMatrix;             //A type for the translation matrix
typedef std::pair<RotMatrix,TransMatrix> RigidTransfo;     //A type for the rigid transform
 //A type for the point clouds

//Enumerator for setting the underlying ICP method used
enum IcpMethod {pointToPoint,pointToPlane};

class IcpOptimizer
{
public:
   

  // Constructor
  IcpOptimizer(Eigen::Matrix<double, Eigen::Dynamic, 6>  _firstCloud, Eigen::Matrix<double, Eigen::Dynamic, 6>  _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p, IcpMethod _method, bool _verbose, int m);

  
  //Constructor
  IcpOptimizer(Eigen::Matrix<double, Eigen::Dynamic, 3> _firstCloud, Eigen::Matrix<double, Eigen::Dynamic, 3> _secondCloud, size_t _kNormals, int _nbIterations, int _nbIterationsIn, double _mu, int _nbIterShrink, double _p, IcpMethod _method, bool _verbose);

  //The algorithm itself
  int performSparceICP();
  int  performSparceICP_single();
  //First step : compute correspondances
  std::vector<int> computeCorrespondances(PointCloud refCloud, PointCloud queryCloud);

  //Apply rigid transformation to a point cloud
  PointCloud movePointCloud(PointCloud poinCloud, RigidTransfo t);

  //Normal estimation
  Eigen::Matrix<double,Eigen::Dynamic,3> estimateNormals(PointCloud pointCloud, const size_t k);

  //Classical rigid transform estimation (point-to-point)
  RigidTransfo rigidTransformPointToPoint(PointCloud a, PointCloud b) const;

  //Classical rigid transform estimation (point-to-plane)
  RigidTransfo rigidTransformPointToPlane(PointCloud a, PointCloud b, Eigen::Matrix<double,Eigen::Dynamic,3> n) const;

  //Shrink operator
  TransMatrix shrink(TransMatrix h) const;

  //Computing composition of tNew by tOld (tNew o tOld)
  RigidTransfo compose(RigidTransfo tNew, RigidTransfo tOld) const;

  //Selection of a subset in a PointCloud
  PointCloud selectSubsetPC(PointCloud p, std::vector<int> indice) const;

  //Updates the iterations measure by estimating the amplitude of rigid motion t
  void updateIter(RigidTransfo t);

  //Save iterations to file
  void saveIter(std::string pathToFile);

  //Getters
  Eigen::Matrix<double,Eigen::Dynamic,3> getFirstNormals() const;
  Eigen::Matrix<double,Eigen::Dynamic,3> getMovedNormals() const;
  PointCloud getMovedPointCloud() const;
  RigidTransfo getComputedTransfo() const;
  double getReferenceDist() const;
private:
  const PointCloud firstCloud;
  const PointCloud secondCloud;
  PointCloud movingPC;
  Eigen::Matrix<double,Eigen::Dynamic,3> movingNormals;
  Eigen::Matrix<double,Eigen::Dynamic,3> selectedNormals; //For point-to-plane
  RigidTransfo computedTransfo;
  std::vector<double> iterations;
  double referenceDist;
  bool hasBeenComputed;
  /*I don't use the PointCloud name for the normals in order to distinguish them 
  from the vertice*/
  Eigen::Matrix<double,Eigen::Dynamic,3> firstNormals;
  Eigen::Matrix<double,Eigen::Dynamic,3> secondNormals;

  Eigen::Matrix<double,Eigen::Dynamic,3> lambda; //Lagrange multiplier for step 2.1

  //Algorithm parameters
  const size_t kNormals;    //K-nn parameter for normal computation
  const int nbIterations;   //Number of iterations for the algorithm
  const int nbIterationsIn; //Number of iterations for the step 2 of the algorithm
  const double mu;          //Parameter for ICP step 2.1
  const int nbIterShrink;   //Number of iterations for the shrink part (2.1)
  const double p;           //We use the norm L_p
  const bool verbose;       //Verbosity trigger
  const IcpMethod method;   //The used method (point to point or point to plane)
};

#endif
