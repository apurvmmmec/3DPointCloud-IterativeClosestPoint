//
//  myicp.h
//  IGLFramework
//
//  Created by Apurv Nigam on 10/02/2017.
//
//


#ifndef icpSubsampled_h
#define icpSubsampled_h
#include "ANN/ANN.h"
using namespace std;
using namespace Eigen;

typedef void (*updateView)(vector<MatrixXd>);
vector<MatrixXd> icpSubsampled(vector<MatrixXd> vecV,float ssFact, updateView uv);
map<int,int>findClosestPoints(MatrixXd p, MatrixXd q, int maxPts);




#endif /* myicp_h */
