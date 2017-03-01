//
//  myicp.h
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//


#ifndef icpmultimesh_h
#define icpmultimesh_h
#include "ANN/ANN.h"
using namespace std;
using namespace Eigen;

typedef void (*updateView)(vector<MatrixXd>);
vector<MatrixXd> icpMultimesh(vector<MatrixXd> vecV,updateView uv);
map<int,int>findClosestPoints(MatrixXd p, MatrixXd q, int maxPts);




#endif /* myicp_h */
